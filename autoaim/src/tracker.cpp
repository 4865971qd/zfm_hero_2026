#include "tracker.hpp"
#include "utils/logger.hpp"
#include "utils/math.hpp"
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace autoaim {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

double normalizeAngle(double angle) noexcept {
    return std::remainder(angle, kTwoPi);
}

double unwrapNear(double angle, double reference) noexcept {
    return reference + normalizeAngle(angle - reference);
}

double angleDifference(double lhs, double rhs) noexcept {
    return std::abs(normalizeAngle(lhs - rhs));
}

bool heldFor(bool condition, double now, double duration, double &since) noexcept {
    if (!condition || !std::isfinite(now)) {
        since = -1.0;
        return false;
    }
    if (since < 0.0 || now < since) {
        since = now;
    }
    return now - since >= std::max(0.0, duration);
}

}  // namespace

Tracker::Tracker(const YAML::Node &cfg) {
    auto &t = cfg["tracker"];
    max_match_dist_ = t["max_match_distance"].as<double>(0.5);
    max_match_yaw_  = t["max_match_yaw_diff"].as<double>(1.0);
    radius_min_     = t["radius_min"].as<double>(0.23);
    radius_max_     = t["radius_max"].as<double>(0.34);
    default_radius_ = std::clamp(t["default_radius"].as<double>(0.26),
                                 radius_min_, radius_max_);
    tracking_confirm_time_ = t["tracking_confirm_time"].as<double>(0.03);
    lost_time_ = t["lost_time"].as<double>(0.30);
    stationary_confirm_time_ = t["stationary_confirm_time"].as<double>(0.15);
    stationary_release_time_ = t["stationary_release_time"].as<double>(0.05);
    stationary_vel_threshold_ = t["stationary_vel_threshold"].as<double>(0.05);
    stationary_release_vel_threshold_ =
        t["stationary_release_vel_threshold"].as<double>(0.08);

    target_state_ = Eigen::VectorXd::Zero(X_N);
    measurement_  = Eigen::VectorXd::Zero(Z_N);
    last_t_ = std::chrono::steady_clock::now();
}

// ---- 初始化 ----
void Tracker::init(const std::list<Armor> &armors,
                   std::chrono::steady_clock::time_point t) {
    if (armors.empty()) return;
    last_t_ = t;

    double min_d = DBL_MAX;
    tracked_armor_ = armors.front();
    for (const auto &a : armors) {
        double center_distance = cv::norm(a.center - cv::Point2f(720, 540));
        if (center_distance < min_d) {
            min_d = center_distance;
            tracked_armor_ = a;
        }
    }

    tracked_id_ = tracked_armor_.number;
    target_.armor_type = tracked_armor_.type;

    if (tracked_id_ == "outpost") {
        outpost_ = std::make_unique<OutpostTracker>();
        tracked_num_ = ArmorsNum::OUTPOST_3;
        tracker_state_ = TrackerState::DETECTING;
        getLogger()->info("[Tracker] Outpost tracker initialized");
        return;
    }

    initEKF(tracked_armor_);
    tracked_num_ = ArmorsNum::NORMAL_4;
    target_.armor_num = 4;
    target_.normal_last_seen_time = 0;
    normal_last_seen_time_ = std::chrono::duration<double>(t.time_since_epoch()).count();
    target_.normal_last_seen_time = normal_last_seen_time_;
    tracking_since_ = -1.0;
    lost_since_ = -1.0;
    tracker_state_ = TrackerState::DETECTING;
    tracked_num_ = ArmorsNum::NORMAL_4;
}

void Tracker::initEKF(const Armor &a) {
    double xa = a.xyz_in_world.x(), ya = a.xyz_in_world.y(), za = a.xyz_in_world.z();
    double yaw = normalizeAngle(a.ypr_in_world[0]);
    double r = default_radius_;
    d_za_ = 0; d_zc_ = 0; another_r_ = r;
    stationary_mode_ = false;
    stationary_since_ = -1.0;
    moving_since_ = -1.0;

    target_state_ << xa + r*cos(yaw), 0, ya + r*sin(yaw), 0, za, 0, yaw, 0, r, d_zc_;

    Eigen::Matrix<double, X_N, X_N> P0 = Eigen::Matrix<double, X_N, X_N>::Identity();
    P0.diagonal() << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1;

    // 观测函数 lambda (直接用于 Ceres Jet 自动微分)
    auto h = [](const ceres::Jet<double, X_N> *x, ceres::Jet<double, X_N> *z) {
        z[0] = x[0] - ceres::cos(x[6]) * x[8];
        z[1] = x[2] - ceres::sin(x[6]) * x[8];
        z[2] = x[4] + x[9];
        z[3] = x[6];
    };
    auto ur = [](const Eigen::Matrix<double, Z_N, 1> &) -> Eigen::Matrix<double, Z_N, Z_N> {
        Eigen::Matrix<double, Z_N, Z_N> R = Eigen::Matrix<double, Z_N, Z_N>::Zero();
        R(0,0) = 2e-3; R(1,1) = 2e-3; R(2,2) = 2e-3; R(3,3) = 2e-3;
        return R;
    };

    ekf_ = std::make_unique<RobotEKF>(target_state_, P0, h, ur);
}

bool Tracker::handleArmorJump(const Armor &a, double yaw) {
    const Eigen::VectorXd previous_state = target_state_;
    const double previous_d_za = d_za_;
    const double previous_d_zc = d_zc_;
    const double previous_another_r = another_r_;
    double ly = target_state_(6);
    if (angleDifference(yaw, ly) > 0.4) {
        target_state_(6) = yaw;
        if (tracked_num_ == ArmorsNum::NORMAL_4) {
            d_za_ = target_state_(4) + target_state_(9) - a.xyz_in_world.z();
            std::swap(target_state_(8), another_r_);
            target_state_(8) = std::clamp(target_state_(8), radius_min_, radius_max_);
            another_r_ = std::clamp(another_r_, radius_min_, radius_max_);
            d_zc_ = (d_zc_ == 0) ? -d_za_ : 0;
            target_state_(9) = d_zc_;
        }
    }
    Eigen::Vector3d cp(a.xyz_in_world.x(), a.xyz_in_world.y(), a.xyz_in_world.z());
    Eigen::Vector3d ip(target_state_(0) - target_state_(8)*cos(target_state_(6)),
                       target_state_(2) - target_state_(8)*sin(target_state_(6)),
                       target_state_(4) + target_state_(9));
    if ((cp - ip).norm() > max_match_dist_) {
        d_zc_ = 0;
        double r = target_state_(8);
        target_state_(0) = a.xyz_in_world.x() + r*cos(yaw);
        target_state_(2) = a.xyz_in_world.y() + r*sin(yaw);
        target_state_(4) = a.xyz_in_world.z();
        target_state_(9) = d_zc_;
    }
    if (!target_state_.allFinite()) {
        target_state_ = previous_state;
        d_za_ = previous_d_za;
        d_zc_ = previous_d_zc;
        another_r_ = previous_another_r;
        return false;
    }
    ekf_->setState(target_state_);
    return true;
}

// ---- 更新 ----
void Tracker::update(const std::list<Armor> &armors, std::chrono::steady_clock::time_point t) {
    const double raw_dt = math::deltaTime(t, last_t_);
    const bool abnormal_dt = !std::isfinite(raw_dt) || raw_dt <= 0.0 || raw_dt > 0.05;
    const double dt = std::clamp(abnormal_dt ? 0.005 : raw_dt, 0.001, 0.05);
    last_t_ = t;
    const double t_sec = std::chrono::duration<double>(t.time_since_epoch()).count();

    if (abnormal_dt) {
        stationary_mode_ = false;
        stationary_since_ = -1.0;
        moving_since_ = -1.0;
    }

    if (tracked_id_ == "outpost") {
        if (!outpost_) {
            outpost_ = std::make_unique<OutpostTracker>();
        }

        bool found = false;
        bool accepted_reliable_observation = false;
        int outpost_candidate_count = 0;
        const Armor *selected_outpost = nullptr;
        double ox = 0, oy = 0, oz = 0, oyaw = 0;

        for (const auto &a : armors) {
            if (a.number == "outpost") {
                ++outpost_candidate_count;
                if (selected_outpost == nullptr) selected_outpost = &a;
            }
        }

        if (selected_outpost != nullptr) {
            const auto &a = *selected_outpost;
            ox = a.xyz_in_world.x(); oy = a.xyz_in_world.y();
            oz = a.xyz_in_world.z(); oyaw = a.ypr_in_world[0];
            found = true;

            static unsigned long long observation_log_sequence = 0;
            ++observation_log_sequence;
            if (observation_log_sequence % 10 == 1 || outpost_candidate_count != 1) {
                getLogger()->debug(
                    "[Outpost][OBS_INPUT] seq={} candidates={} confidence={:.3f} "
                    "image=({:.1f},{:.1f}) image_dist={:.1f} "
                    "camera=({:.3f},{:.3f},{:.3f}) "
                    "gimbal=({:.3f},{:.3f},{:.3f}) "
                    "world=({:.3f},{:.3f},{:.3f}) "
                    "gimbal_ypr=({:.1f},{:.1f},{:.1f})deg "
                    "world_ypr=({:.1f},{:.1f},{:.1f})deg",
                    observation_log_sequence, outpost_candidate_count, a.confidence,
                    a.center.x, a.center.y, a.distance_to_image_center,
                    a.xyz_in_camera.x(), a.xyz_in_camera.y(), a.xyz_in_camera.z(),
                    a.xyz_in_gimbal.x(), a.xyz_in_gimbal.y(), a.xyz_in_gimbal.z(),
                    a.xyz_in_world.x(), a.xyz_in_world.y(), a.xyz_in_world.z(),
                    a.ypr_in_gimbal[0] * 180.0 / M_PI,
                    a.ypr_in_gimbal[1] * 180.0 / M_PI,
                    a.ypr_in_gimbal[2] * 180.0 / M_PI,
                    a.ypr_in_world[0] * 180.0 / M_PI,
                    a.ypr_in_world[1] * 180.0 / M_PI,
                    a.ypr_in_world[2] * 180.0 / M_PI);
            }

            accepted_reliable_observation =
                outpost_->addMeasurement(ox, oy, oz, oyaw, t_sec);
        }

        if (outpost_->getState() == OutpostTracker::ACTIVE ||
            outpost_->getState() == OutpostTracker::STATIC) {

            if (outpost_->getState() == OutpostTracker::ACTIVE) {
                double cx, cy, R, omega;
                outpost_->getCircleParams(cx, cy, R, omega);
                const bool z_calibrated = outpost_->isZCalibrated();
                const int observed_plate = outpost_->currentPlate();
                const int ref_plate = z_calibrated ? 0 : observed_plate;
                double ref_ang = outpost_->getPlateAngle(t_sec, ref_plate);
                double ref_z = outpost_->getZCenter(ref_plate);

                target_state_(0) = cx; target_state_(1) = 0;
                target_state_(2) = cy; target_state_(3) = 0;
                target_state_(4) = ref_z; target_state_(5) = 0;
                target_state_(6) = ref_ang; target_state_(7) = omega;
                target_state_(8) = R;

                if (z_calibrated) {
                    target_state_(9) = outpost_->getZCenter(1) - ref_z;
                    d_za_ = outpost_->getZCenter(2) - ref_z;
                    for (int i = 0; i < 3; ++i) {
                        target_.outpost_angle_offsets[i] =
                            math::limitRad(outpost_->getAngleOffset(i) -
                                           outpost_->getAngleOffset(ref_plate));
                    }
                    target_.outpost_observed_plate = observed_plate;
                } else {
                    target_state_(9) = 0;
                    d_za_ = 0;
                    target_.outpost_angle_offsets[0] = 0.0;
                    target_.outpost_angle_offsets[1] = 2.0 * M_PI / 3.0;
                    target_.outpost_angle_offsets[2] = 4.0 * M_PI / 3.0;
                    target_.outpost_observed_plate = 0;
                }

                target_.another_r = R;
            } else {
                // Stopped outpost: track and shoot the single visible armor directly.
                auto predicted = outpost_->getPredictedPositions(t_sec);
                if (!predicted.empty()) {
                    target_state_(0) = predicted.front().x(); target_state_(1) = 0;
                    target_state_(2) = predicted.front().y(); target_state_(3) = 0;
                    target_state_(4) = predicted.front().z(); target_state_(5) = 0;
                }
                target_state_(6) = outpost_->getPlateAngle(t_sec, 0);
                target_state_(7) = 0;
                target_state_(8) = 0; target_state_(9) = 0;
                d_za_ = 0; target_.another_r = 0;
                target_.outpost_observed_plate = 0;
                for (double &offset : target_.outpost_angle_offsets) offset = 0.0;
            }

            target_.armor_num = 3;
            target_.state = target_state_;
            target_.d_za = d_za_;
            target_.normal_last_seen_time = 0;
            target_.outpost_z_calibrated = outpost_->isZCalibrated();
            if (accepted_reliable_observation) {
                target_.outpost_last_seen_time = t_sec;
            } else if (found) {
                target_.outpost_last_seen_time = 0;
            }
            target_.outpost_phase_error = outpost_->getPhaseError();
            target_.valid = true;
        }

        if (outpost_->getState() == OutpostTracker::COLLECTING ||
            outpost_->getState() == OutpostTracker::CALIBRATING) {
            tracker_state_ = TrackerState::DETECTING;
            outpost_detect_count_ = 0;
            target_.valid = false;
            target_.outpost_z_calibrated = false;
            target_.outpost_observed_plate = -1;
            return;
        }

        // Outpost 状态机
        if (tracker_state_ == TrackerState::DETECTING) {
            if (outpost_->getState() == OutpostTracker::ACTIVE ||
                outpost_->getState() == OutpostTracker::STATIC) {
                outpost_detect_count_++;
                if (outpost_detect_count_ > 1) {
                    outpost_detect_count_ = 0;
                    tracker_state_ = TrackerState::TRACKING;
                }
            }
        } else if (tracker_state_ == TrackerState::TRACKING) {
            if (!found) { tracker_state_ = TrackerState::TEMP_LOST; }
        } else if (tracker_state_ == TrackerState::TEMP_LOST) {
            if (found) { tracker_state_ = TrackerState::TRACKING; }
            else {
                if (t_sec - outpost_->getLastSeenTime() > 1.0) {
                    outpost_.reset();
                    outpost_detect_count_ = 0;
                    tracker_state_ = TrackerState::LOST;
                    target_.valid = false;
                    target_.outpost_z_calibrated = false;
                    target_.outpost_observed_plate = -1;
                    target_.outpost_last_seen_time = 0;
                    target_.outpost_phase_error = 0;
                }
            }
        }
        return;
    }

    // === 普通装甲板 EKF 跟踪 ===
    if (tracker_state_ == TrackerState::TRACKING) {
        const bool currently_stationary =
            std::abs(target_state_(1)) < stationary_vel_threshold_ &&
            std::abs(target_state_(3)) < stationary_vel_threshold_ &&
            std::abs(target_state_(5)) < stationary_vel_threshold_ &&
            std::abs(target_state_(7)) < stationary_vel_threshold_;
        if (!stationary_mode_) {
            stationary_mode_ = heldFor(
                currently_stationary, t_sec, stationary_confirm_time_, stationary_since_);
        } else {
            const bool clearly_moving =
                std::abs(target_state_(1)) > stationary_release_vel_threshold_ ||
                std::abs(target_state_(3)) > stationary_release_vel_threshold_ ||
                std::abs(target_state_(5)) > stationary_release_vel_threshold_ ||
                std::abs(target_state_(7)) > stationary_release_vel_threshold_;
            if (heldFor(clearly_moving, t_sec, stationary_release_time_, moving_since_)) {
                stationary_mode_ = false;
                stationary_since_ = -1.0;
                moving_since_ = -1.0;
            }
        }
    } else {
        stationary_mode_ = false;
        stationary_since_ = -1.0;
        moving_since_ = -1.0;
    }

    bool use_stationary_model = stationary_mode_;
    if (use_stationary_model) {
        target_state_(1) = 0;
        target_state_(3) = 0;
        target_state_(5) = 0;
        target_state_(7) = 0;
    }

    ekf_->state() = target_state_;
    const Eigen::VectorXd state_before_predict = target_state_;
    auto pred = ekf_->predict(dt);
    if (!pred.allFinite()) {
        if (!state_before_predict.allFinite()) {
            tracker_state_ = TrackerState::LOST;
            target_.valid = false;
            return;
        }
        pred = state_before_predict;
        ekf_->setState(pred);
        stationary_mode_ = false;
        stationary_since_ = -1.0;
        moving_since_ = -1.0;
    }
    if (use_stationary_model) {
        pred(1) = 0;
        pred(3) = 0;
        pred(5) = 0;
        pred(7) = 0;
        ekf_->setState(pred);
    }

    bool matched = false;
    target_state_ = pred;

    if (!armors.empty()) {
        Armor same_id; int same_count = 0;
        Eigen::Vector3d ppos(target_state_(0) - target_state_(8)*cos(target_state_(6)),
                             target_state_(2) - target_state_(8)*sin(target_state_(6)),
                             target_state_(4) + target_state_(9));
        double min_pos_diff = DBL_MAX;
        double yaw_diff = DBL_MAX;

        for (const auto &a : armors) {
            if (a.number == tracked_id_) {
                same_id = a; same_count++;
                Eigen::Vector3d ap(a.xyz_in_world.x(), a.xyz_in_world.y(), a.xyz_in_world.z());
                double pd = (ppos - ap).norm();
                if (pd < min_pos_diff) {
                    min_pos_diff = pd;
                    const double candidate_yaw = unwrapNear(
                        a.ypr_in_world[0], target_state_(6));
                    yaw_diff = angleDifference(candidate_yaw, target_state_(6));
                    tracked_armor_ = a;
                }
            }
        }

        if (min_pos_diff < max_match_dist_ && yaw_diff < max_match_yaw_) {
            double m_yaw = unwrapNear(
                tracked_armor_.ypr_in_world[0], target_state_(6));
            measurement_ << tracked_armor_.xyz_in_world.x(),
                            tracked_armor_.xyz_in_world.y(),
                            tracked_armor_.xyz_in_world.z(),
                            m_yaw;
            const Eigen::VectorXd state_before_update = target_state_;
            Eigen::VectorXd updated_state = ekf_->update(measurement_);
            if (updated_state.allFinite()) {
                matched = true;
                target_state_ = updated_state;
                normal_last_seen_time_ = t_sec;
                target_.armor_type = tracked_armor_.type;
            } else {
                target_state_ = state_before_update;
                ekf_->setState(target_state_);
            }
        } else if (same_count == 1 && yaw_diff > max_match_yaw_) {
            const double jump_yaw = unwrapNear(
                same_id.ypr_in_world[0], target_state_(6));
            matched = handleArmorJump(same_id, jump_yaw);
            if (matched) {
                tracked_armor_ = same_id;
                target_.armor_type = same_id.type;
                normal_last_seen_time_ = t_sec;
            }
        }
    }

    // 半径约束
    double clamped_radius = std::clamp(target_state_(8), radius_min_, radius_max_);
    if (clamped_radius != target_state_(8)) {
        target_state_(8) = clamped_radius;
        ekf_->setState(target_state_);
    }
    another_r_ = std::clamp(another_r_, radius_min_, radius_max_);

    // 状态机
    if (tracker_state_ == TrackerState::DETECTING) {
        if (matched) {
            if (heldFor(true, t_sec, tracking_confirm_time_, tracking_since_)) {
                tracking_since_ = -1.0;
                tracker_state_ = TrackerState::TRACKING;
            }
        } else {
            tracking_since_ = -1.0;
            tracker_state_ = TrackerState::LOST;
        }
    } else if (tracker_state_ == TrackerState::TRACKING) {
        if (!matched) {
            lost_since_ = t_sec;
            tracker_state_ = TrackerState::TEMP_LOST;
        }
    } else if (tracker_state_ == TrackerState::TEMP_LOST) {
        if (!matched) {
            if (heldFor(true, t_sec, lost_time_, lost_since_)) {
                lost_since_ = -1.0;
                tracker_state_ = TrackerState::LOST;
            }
        } else {
            lost_since_ = -1.0;
            tracker_state_ = TrackerState::TRACKING;
        }
    }

    // 更新 Target
    target_.state = target_state_;
    target_.d_za = d_za_;
    target_.another_r = another_r_;
    target_.normal_last_seen_time = normal_last_seen_time_;
    target_.outpost_z_calibrated = true;
    target_.outpost_observed_plate = -1;
    target_.outpost_last_seen_time = 0;
    target_.outpost_phase_error = 0;
    target_.valid = (tracker_state_ == TrackerState::TRACKING || tracker_state_ == TrackerState::TEMP_LOST);
}

// ---- 主入口 ----
void Tracker::track(std::list<Armor> &armors, std::chrono::steady_clock::time_point t) {
    if (tracker_state_ == TrackerState::LOST) init(armors, t);
    else update(armors, t);
}

std::vector<Eigen::Vector3d> Tracker::getOutpostPredicted(double t, double dt) {
    if (!outpost_) return {};
    return outpost_->getPredictedPositions(t, dt);
}

bool Tracker::getOutpostCircleParams(double &cx, double &cy, double &R, double &omega) {
    if (!outpost_) return false;
    return outpost_->getCircleParams(cx, cy, R, omega);
}

}  // namespace autoaim
