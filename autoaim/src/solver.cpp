#include "solver.hpp"

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace autoaim {
namespace {

bool isFresh(double now, double last_seen, double max_unseen) noexcept {
    if (!std::isfinite(now) || !std::isfinite(last_seen) || last_seen <= 0.0) {
        return false;
    }
    const double unseen = now - last_seen;
    return unseen >= -0.02 && unseen <= max_unseen;
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

constexpr double LIGHTBAR_LEN = 0.050;
constexpr double BIG_W = 0.225;
constexpr double SMALL_W = 0.133;

static const std::vector<cv::Point3f> BIG_OBJ_PTS{
    {0, BIG_W / 2, -LIGHTBAR_LEN / 2},
    {0, BIG_W / 2, LIGHTBAR_LEN / 2},
    {0, -BIG_W / 2, LIGHTBAR_LEN / 2},
    {0, -BIG_W / 2, -LIGHTBAR_LEN / 2},
};

static const std::vector<cv::Point3f> SMALL_OBJ_PTS{
    {0, SMALL_W / 2, -LIGHTBAR_LEN / 2},
    {0, SMALL_W / 2, LIGHTBAR_LEN / 2},
    {0, -SMALL_W / 2, LIGHTBAR_LEN / 2},
    {0, -SMALL_W / 2, -LIGHTBAR_LEN / 2},
};

Solver::Solver(const YAML::Node &cfg) {
    auto odom = cfg["odom2camera"];
    auto xyz = odom["xyz"].as<std::vector<double>>();
    auto rpy = odom["rpy"].as<std::vector<double>>();
    Eigen::Matrix3d R_optical_to_camera_link;
    R_optical_to_camera_link <<
         0,  0,  1,
        -1,  0,  0,
         0, -1,  0;
    R_c2g_ = math::eulerToMatrix(Eigen::Vector3d(rpy[0], rpy[1], rpy[2])) *
             R_optical_to_camera_link;
    t_c2g_ = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    R_g2w_ = Eigen::Matrix3d::Identity();

    K_ = (cv::Mat_<double>(3, 3) << 1997.88, 0, 720, 0, 1997.88, 540, 0, 0, 1);
    D_ = cv::Mat::zeros(1, 5, CV_64F);
    if (cfg["camera_matrix"]) {
        auto km = cfg["camera_matrix"].as<std::vector<double>>();
        K_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(km.data())).clone();
    }
    if (cfg["distort_coeffs"]) {
        auto dc = cfg["distort_coeffs"].as<std::vector<double>>();
        D_ = cv::Mat(1, static_cast<int>(dc.size()), CV_64F, const_cast<double *>(dc.data())).clone();
    }
    cx_ = K_.at<double>(0, 2);
    cy_ = K_.at<double>(1, 2);

    auto &s = cfg["solver"];
    compensator_ = CompensatorFactory::create(s["compensator_type"].as<std::string>("quadratic_drag"));
    compensator_->gravity = s["gravity"].as<double>(9.6);
    compensator_->velocity = s["bullet_speed"].as<double>(11.6);

    if (auto *qd = dynamic_cast<QuadraticDragCompensator *>(compensator_.get())) {
        qd->drag_coefficient = s["drag_coefficient"].as<double>(0.2);
        qd->air_density = s["air_density"].as<double>(1.225);
        qd->projectile_mass = s["projectile_mass"].as<double>(0.0445);
        qd->projectile_diameter = s["projectile_diameter"].as<double>(0.0425);
    }

    manual_comp_ = std::make_unique<ManualCompensator>();
    if (s["angle_offset"]) {
        manual_comp_->loadFromStrings(s["angle_offset"].as<std::vector<std::string>>());
    }

    shooting_w_ = s["shooting_range_width"].as<double>(0.135);
    shooting_h_ = s["shooting_range_height"].as<double>(0.135);
    max_tracking_v_yaw_ = s["max_tracking_v_yaw"].as<double>(60.0);
    prediction_delay_ = s["prediction_delay"].as<double>(-0.005);
    controller_delay_ = s["controller_delay"].as<double>(0.0);
    outpost_yaw_offset_ = s["outpost_yaw_offset"].as<double>(0.0);
    outpost_max_unseen_time_ = s["outpost_max_unseen_time"].as<double>(0.15);
    outpost_max_phase_error_ = s["outpost_max_phase_error"].as<double>(15.0) * M_PI / 180.0;
    normal_max_fire_unseen_time_ =
        s["normal_max_fire_unseen_time"].as<double>(0.10);
    center_switch_confirm_time_ =
        s["center_switch_confirm_time"].as<double>(0.05);
    side_angle_ = s["side_angle"].as<double>(45.0);
    min_switching_v_yaw_ = s["min_switching_v_yaw"].as<double>(0.5);
    yaw_dead_zone_ = s["yaw_dead_zone"].as<double>(0.1);
    pitch_dead_zone_ = s["pitch_dead_zone"].as<double>(0.1);

    if (s["muzzle"] && s["muzzle"]["xyz"]) {
        auto m = s["muzzle"]["xyz"].as<std::vector<double>>();
        if (m.size() == 3) {
            muzzle_offset_ = Eigen::Vector3d(m[0], m[1], m[2]);
        }
    }
}

void Solver::setImu(double roll, double pitch, double yaw) {
    current_yaw_ = yaw;
    current_pitch_ = pitch;
    Eigen::Quaterniond q = math::eulerToQuat(roll, pitch, yaw);
    R_g2w_ = q.toRotationMatrix();
}

bool Solver::solveArmor(Armor &armor) {
    const auto &obj_pts = (armor.type == ArmorType::LARGE) ? BIG_OBJ_PTS : SMALL_OBJ_PTS;
    auto image_pts = armor.landmarks();
    std::vector<cv::Mat> rvecs, tvecs;
    int solutions = cv::solvePnPGeneric(
        obj_pts, image_pts, K_, D_, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    if (solutions <= 0 || rvecs.empty() || tvecs.empty()) {
        getLogger()->warn("PnP failed");
        return false;
    }
    sortPnPResult(armor, obj_pts, rvecs, tvecs);

    Eigen::Vector3d xyz_cam;
    cv::cv2eigen(tvecs[0], xyz_cam);
    armor.xyz_in_camera = xyz_cam;
    armor.xyz_in_gimbal = R_c2g_ * xyz_cam + t_c2g_;
    armor.xyz_in_world = R_g2w_ * armor.xyz_in_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvecs[0], rmat);
    Eigen::Matrix3d R_ac;
    cv::cv2eigen(rmat, R_ac);
    Eigen::Matrix3d R_ag = R_c2g_ * R_ac;
    Eigen::Matrix3d R_aw = R_g2w_ * R_ag;
    armor.ypr_in_gimbal = math::eulers(R_ag, 2, 1, 0);
    armor.ypr_in_world = math::eulers(R_aw, 2, 1, 0);
    armor.ypd_in_world = math::xyz2ypd(armor.xyz_in_world);
    armor.distance_to_image_center =
        cv::norm(armor.center - cv::Point2f(static_cast<float>(cx_), static_cast<float>(cy_)));
    return armor.xyz_in_camera.allFinite() && armor.xyz_in_gimbal.allFinite() &&
           armor.xyz_in_world.allFinite() && armor.ypr_in_gimbal.allFinite() &&
           armor.ypr_in_world.allFinite();
}

double Solver::reprojectionError(const std::vector<cv::Point3f> &obj_pts,
                                 const std::vector<cv::Point2f> &img_pts,
                                 const cv::Mat &rvec,
                                 const cv::Mat &tvec) const {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj_pts, rvec, tvec, K_, D_, projected);
    if (projected.size() != img_pts.size() || projected.empty()) {
        return DBL_MAX;
    }

    double err = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        err += cv::norm(projected[i] - img_pts[i]);
    }
    return err / static_cast<double>(projected.size());
}

void Solver::sortPnPResult(const Armor &armor,
                           const std::vector<cv::Point3f> &obj_pts,
                           std::vector<cv::Mat> &rvecs,
                           std::vector<cv::Mat> &tvecs) const {
    if (rvecs.size() < 2 || tvecs.size() < 2) {
        return;
    }

    auto img_pts = armor.landmarks();
    double err1 = reprojectionError(obj_pts, img_pts, rvecs[0], tvecs[0]);
    double err2 = reprojectionError(obj_pts, img_pts, rvecs[1], tvecs[1]);
    constexpr double kErrorRatio = 3.0;

    if (err1 > err2 * kErrorRatio) {
        std::swap(rvecs[0], rvecs[1]);
        std::swap(tvecs[0], tvecs[1]);
        return;
    }
    if (err2 > err1 * kErrorRatio) {
        return;
    }

    cv::Mat rmat1, rmat2;
    cv::Rodrigues(rvecs[0], rmat1);
    cv::Rodrigues(rvecs[1], rmat2);

    Eigen::Matrix3d R1, R2;
    cv::cv2eigen(rmat1, R1);
    cv::cv2eigen(rmat2, R2);
    Eigen::Vector3d ypr1 = math::eulers(R_c2g_ * R1, 2, 1, 0);
    Eigen::Vector3d ypr2 = math::eulers(R_c2g_ * R2, 2, 1, 0);
    constexpr double kRollLimit = 10.0 * M_PI / 180.0;
    if (std::abs(ypr1[2]) > kRollLimit || std::abs(ypr2[2]) > kRollLimit) {
        return;
    }

    double l_angle = std::atan2(armor.left_light.axis.y, armor.left_light.axis.x) * 180.0 / M_PI;
    double r_angle = std::atan2(armor.right_light.axis.y, armor.right_light.axis.x) * 180.0 / M_PI;
    double tilt = (l_angle + r_angle) * 0.5 + 90.0;
    if (armor.number == "outpost") {
        tilt = -tilt;
    }

    if ((tilt > 0 && ypr1[0] > 0 && ypr2[0] < 0) ||
        (tilt < 0 && ypr1[0] < 0 && ypr2[0] > 0)) {
        std::swap(rvecs[0], rvecs[1]);
        std::swap(tvecs[0], tvecs[1]);
    }
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &center,
                                                        double yaw,
                                                        double r1,
                                                        double r2,
                                                        double d_zc,
                                                        double d_za,
                                                        int num,
                                                        const double *angle_offsets) const noexcept {
    std::vector<Eigen::Vector3d> res(num, Eigen::Vector3d::Zero());
    if (num == 3) {
        double default_offs[3] = {0, 2 * M_PI / 3, 4 * M_PI / 3};
        const double *offs = angle_offsets ? angle_offsets : default_offs;
        double zs[3] = {0, d_zc, d_za};
        for (int i = 0; i < 3; i++) {
            double a = yaw + offs[i];
            res[i] = {center.x() + r1 * std::cos(a),
                      center.y() + r1 * std::sin(a),
                      center.z() + zs[i]};
        }
        return res;
    }

    bool cur = true;
    for (int i = 0; i < num; i++) {
        double a = yaw + i * (2 * M_PI / num);
        double r = cur ? r1 : r2;
        double dz = d_zc + (cur ? 0 : d_za);
        res[i] = center + Eigen::Vector3d(-r * std::cos(a), -r * std::sin(a), dz);
        cur = !cur;
    }
    return res;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &positions,
                            const Eigen::Vector3d &center,
                            double yaw,
                            double v_yaw,
                            int num,
                            double flying_time,
                            const double *angle_offsets) const noexcept {
    (void)positions;
    double alpha = std::atan2(center.y(), center.x());
    double beta = yaw;
    Eigen::Matrix2d R_o2c, R_o2a;
    R_o2c << std::cos(alpha), std::sin(alpha), -std::sin(alpha), std::cos(alpha);
    R_o2a << std::cos(beta), std::sin(beta), -std::sin(beta), std::cos(beta);
    const double sin_decision =
        std::clamp((R_o2c.transpose() * R_o2a)(0, 1), -1.0, 1.0);
    double dec = -std::asin(sin_decision);

    double theta = 0.0;
    if (std::abs(v_yaw) >= min_switching_v_yaw_ && flying_time > 0.0) {
        double td = std::min(std::abs(v_yaw) * flying_time, side_angle_ * M_PI / 180.0);
        theta = (v_yaw > 0) ? td : -td;
    }
    double tmp = dec + M_PI / num - theta;
    if (tmp < 0) {
        tmp += 2 * M_PI;
    }
    int sel = static_cast<int>(tmp / (2 * M_PI / num));
    if (sel >= num) {
        sel = num - 1;
    }

    if (num == 3 && std::abs(v_yaw) < 0.01) {
        return sel;
    }
    if (num == 3) {
        double cd = std::atan2(-center.y(), -center.x());
        double default_offs[3] = {0, 2 * M_PI / 3, 4 * M_PI / 3};
        const double *offs = angle_offsets ? angle_offsets : default_offs;
        double diffs[3];
        bool app[3];
        for (int i = 0; i < 3; i++) {
            diffs[i] = math::limitRad(yaw + offs[i] - cd);
            app[i] = (v_yaw * diffs[i] < 0);
        }
        int best = -1;
        double best_a = 1e9;
        for (int i = 0; i < 3; i++) {
            if (app[i] && std::abs(diffs[i]) < best_a) {
                best_a = std::abs(diffs[i]);
                best = i;
            }
        }
        if (best < 0) {
            best = 0;
            for (int i = 1; i < 3; i++) {
                if (std::abs(diffs[i]) < std::abs(diffs[best])) {
                    best = i;
                }
            }
        }
        return best;
    }
    return sel;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p, double &yaw, double &pitch) const noexcept {
    yaw = std::atan2(p.y(), p.x());
    pitch = std::atan2(p.z(), p.head<2>().norm());
    double tmp = pitch;
    if (compensator_->compensate(p, tmp)) {
        pitch = tmp;
    }
}

bool Solver::isOnTarget(double cy, double cp, double ty, double tp, double dist) const noexcept {
    double ry = std::abs(std::atan2(shooting_w_ / 2, dist));
    double rp = std::abs(std::atan2(shooting_h_ / 2, dist));
    ry = std::max(ry, M_PI / 180.0);
    rp = std::max(rp, M_PI / 180.0);
    return std::abs(math::limitRad(cy - ty)) < ry && std::abs(cp - tp) < rp;
}

void Solver::applyCommandDeadZone(GimbalCommand &cmd) noexcept {
    if (!std::isfinite(cmd.yaw) || !std::isfinite(cmd.pitch)) {
        cmd = {};
        resetCommandDeadZone();
        return;
    }

    if (!dead_zone_initialized_) {
        last_cmd_yaw_ = cmd.yaw;
        last_cmd_pitch_ = cmd.pitch;
        dead_zone_initialized_ = true;
        return;
    }

    if (std::abs(cmd.yaw - last_cmd_yaw_) < yaw_dead_zone_) {
        cmd.yaw = last_cmd_yaw_;
    } else {
        last_cmd_yaw_ = cmd.yaw;
    }

    if (std::abs(cmd.pitch - last_cmd_pitch_) < pitch_dead_zone_) {
        cmd.pitch = last_cmd_pitch_;
    } else {
        last_cmd_pitch_ = cmd.pitch;
    }
}

GimbalCommand Solver::solve(const Target &target, double current_time) {
    if (target.state.size() < 10 || !target.state.allFinite() ||
        !std::isfinite(target.d_za) || !std::isfinite(target.another_r) ||
        !std::isfinite(current_time)) {
        has_selected_armor_ = false;
        selected_armor_index_ = -1;
        resetCommandDeadZone();
        return {};
    }

    Eigen::Vector3d target_position(target.state[0], target.state[2], target.state[4]);
    double target_yaw = target.state[6];
    double target_v_yaw = target.state[7];
    double r1 = target.state[8];
    double r2 = target.another_r;
    int armors_num = target.armor_num;
    if (armors_num <= 0) {
        aim_idx_ = -1;
        for (int i = 0; i < 3; ++i) {
            smooth_aim_z_[i] = 0.0;
            smooth_aim_z_initialized_[i] = false;
        }
        outpost_pending_idx_ = -1;
        outpost_pending_since_ = -1.0;
        outpost_last_switch_time_ = -1.0;
        has_selected_armor_ = false;
        selected_armor_index_ = -1;
        resetCommandDeadZone();
        return {};
    }

    const bool is_outpost = (armors_num == 3);
    const bool outpost_single_plate = is_outpost && !target.outpost_z_calibrated;
    const double *outpost_offsets = is_outpost ? target.outpost_angle_offsets : nullptr;
    if (!is_outpost) {
        aim_idx_ = -1;
        for (int i = 0; i < 3; ++i) {
            smooth_aim_z_[i] = 0.0;
            smooth_aim_z_initialized_[i] = false;
        }
        outpost_pending_idx_ = -1;
        outpost_pending_since_ = -1.0;
        outpost_last_switch_time_ = -1.0;
    }
    bool outpost_model_ok = true;
    bool outpost_prediction_ok = true;
    bool outpost_observation_fresh = false;
    bool outpost_phase_valid = false;
    const bool normal_model_ok = !is_outpost && isFresh(
        current_time, target.normal_last_seen_time, normal_max_fire_unseen_time_);
    if (is_outpost) {
        double unseen_time = current_time - target.outpost_last_seen_time;
        outpost_observation_fresh = target.outpost_last_seen_time > 0 &&
                                    unseen_time >= -0.02 &&
                                    unseen_time <= outpost_max_unseen_time_;
        outpost_phase_valid = std::isfinite(target.outpost_phase_error) &&
                              std::abs(target.outpost_phase_error) <=
                                  outpost_max_phase_error_;
        outpost_prediction_ok =
            outpost_single_plate || target.outpost_z_calibrated;
        outpost_model_ok = outpost_observation_fresh &&
                           (outpost_single_plate ||
                            (target.outpost_z_calibrated && outpost_phase_valid));
    }

    Eigen::Vector3d muzzle_world = R_g2w_ * muzzle_offset_;
    Eigen::Vector3d target_from_muzzle = target_position - muzzle_world;
    double flying_time = compensator_->getFlyingTime(target_from_muzzle);
    double dt = (current_time - target.current_time) + flying_time + prediction_delay_;

    Eigen::Vector3d target_position_predicted = target_position;
    target_position_predicted.x() += dt * target.state[1];
    target_position_predicted.y() += dt * target.state[3];
    target_position_predicted.z() += dt * target.state[5];
    target_yaw += dt * target_v_yaw;

    Eigen::Vector3d target_from_muzzle_predicted = target_position_predicted - muzzle_world;
    auto armor_positions = getArmorPositions(
        target_from_muzzle_predicted, target_yaw, r1, r2, target.state[9], target.d_za,
        armors_num, outpost_offsets);
    int idx = outpost_single_plate ?
        std::clamp(target.outpost_observed_plate, 0, std::max(0, armors_num - 1)) :
        selectBestArmor(
            armor_positions, target_from_muzzle_predicted, target_yaw, target_v_yaw,
            armors_num, flying_time, outpost_offsets);
    auto chosen_armor_position = armor_positions.at(idx);
    if (chosen_armor_position.norm() < 0.1) {
        has_selected_armor_ = false;
        selected_armor_index_ = -1;
        getLogger()->warn("No valid armor to shoot");
        resetCommandDeadZone();
        return {};
    }

    if (armors_num == 3 && !outpost_single_plate) {
        double refined_flying_time = compensator_->getFlyingTime(chosen_armor_position);
        double refined_dt = (current_time - target.current_time) + refined_flying_time + prediction_delay_;
        if (std::abs(refined_dt - dt) > 0.003) {
            double refined_yaw = target.state[6] + refined_dt * target_v_yaw;
            armor_positions = getArmorPositions(
                target_from_muzzle_predicted, refined_yaw, r1, r2,
                target.state[9], target.d_za, armors_num, outpost_offsets);
            idx = selectBestArmor(
                armor_positions, target_from_muzzle_predicted, refined_yaw,
                target_v_yaw, armors_num, refined_flying_time, outpost_offsets);
            chosen_armor_position = armor_positions.at(idx);
            if (chosen_armor_position.norm() < 0.1) {
                has_selected_armor_ = false;
                selected_armor_index_ = -1;
                getLogger()->warn("No valid armor (refined)");
                resetCommandDeadZone();
                return {};
            }
            target_yaw = refined_yaw;
            flying_time = refined_flying_time;
            dt = refined_dt;
        }
    } else {
        double refined_flying_time = compensator_->getFlyingTime(chosen_armor_position);
        double refined_dt = (current_time - target.current_time) + refined_flying_time + prediction_delay_;
        if (std::abs(refined_dt - dt) > 0.003) {
            Eigen::Vector3d refined_target_position = target_position;
            refined_target_position.x() += refined_dt * target.state[1];
            refined_target_position.y() += refined_dt * target.state[3];
            refined_target_position.z() += refined_dt * target.state[5];
            double refined_yaw = target.state[6] + refined_dt * target_v_yaw;
            Eigen::Vector3d refined_from_muzzle = refined_target_position - muzzle_world;
            armor_positions = getArmorPositions(
                refined_from_muzzle, refined_yaw, r1, r2,
                target.state[9], target.d_za, armors_num, outpost_offsets);
            idx = selectBestArmor(
                armor_positions, refined_from_muzzle, refined_yaw,
                target_v_yaw, armors_num, refined_flying_time, outpost_offsets);
            chosen_armor_position = armor_positions.at(idx);
            if (chosen_armor_position.norm() < 0.1) {
                has_selected_armor_ = false;
                selected_armor_index_ = -1;
                getLogger()->warn("No valid armor (refined)");
                resetCommandDeadZone();
                return {};
            }
            target_position_predicted = refined_target_position;
            target_from_muzzle_predicted = refined_from_muzzle;
            target_yaw = refined_yaw;
            flying_time = refined_flying_time;
            dt = refined_dt;
        }
    }

    double yaw = 0.0;
    double pitch = 0.0;
    calcYawAndPitch(chosen_armor_position, yaw, pitch);
    double distance = chosen_armor_position.norm();

    double outpost_diff_deg = 99.0;
    double aim_z = 0.0;
    Eigen::Vector3d selected_visualization_position = chosen_armor_position;
    double selected_visualization_yaw = target_yaw;
    int selected_visualization_index = outpost_single_plate ? 0 : idx;
    bool selected_visualization_valid = true;
    double default_outpost_offsets[3] = {0, 2 * M_PI / 3, 4 * M_PI / 3};
    const double *offs = outpost_offsets ? outpost_offsets : default_outpost_offsets;
    if (armors_num == 3 && !outpost_single_plate && std::abs(target_v_yaw) > 0.01) {
        const double arrival_fireline = std::atan2(
            -target_from_muzzle_predicted.y(), -target_from_muzzle_predicted.x());
        double diffs[3];
        bool approaching[3];
        for (int i = 0; i < 3; i++) {
            diffs[i] = math::limitRad(target_yaw + offs[i] - arrival_fireline);
            approaching[i] = (target_v_yaw * diffs[i] < 0);
        }

        int candidate = 0;
        double candidate_abs = std::abs(diffs[0]);
        for (int i = 0; i < 3; ++i) {
            const double abs_diff = std::abs(diffs[i]);
            if (abs_diff < candidate_abs) {
                candidate = i;
                candidate_abs = abs_diff;
            }
        }

        constexpr double kSwitchMargin = 22.0 * M_PI / 180.0;
        constexpr double kHoldWindow = 55.0 * M_PI / 180.0;
        constexpr double kForceSwitch = 88.0 * M_PI / 180.0;
        constexpr double kConfirmTime = 0.090;
        constexpr double kMinSwitchInterval = 0.160;
        if (aim_idx_ < 0 || aim_idx_ >= 3) {
            aim_idx_ = candidate;
            outpost_pending_idx_ = -1;
            outpost_pending_since_ = -1.0;
            outpost_last_switch_time_ = current_time;
        } else if (candidate != aim_idx_) {
            const double current_abs = std::abs(diffs[aim_idx_]);
            const bool candidate_much_better = candidate_abs + kSwitchMargin < current_abs;
            const bool current_unusable = !approaching[aim_idx_] || current_abs > kHoldWindow;
            const bool force_switch = current_abs > kForceSwitch;
            if (current_unusable && (candidate_much_better || force_switch)) {
                if (outpost_pending_idx_ != candidate) {
                    outpost_pending_idx_ = candidate;
                    outpost_pending_since_ = current_time;
                }
                if (!outpost_phase_valid) {
                    outpost_pending_since_ = current_time;
                } else {
                    const double pending = current_time - outpost_pending_since_;
                    const double since_switch = outpost_last_switch_time_ > 0 ?
                        current_time - outpost_last_switch_time_ : 1e9;
                    if (force_switch ||
                        (pending >= kConfirmTime && since_switch >= kMinSwitchInterval)) {
                        aim_idx_ = candidate;
                        outpost_pending_idx_ = -1;
                        outpost_pending_since_ = -1.0;
                        outpost_last_switch_time_ = current_time;
                    }
                }
            } else {
                outpost_pending_idx_ = -1;
                outpost_pending_since_ = -1.0;
            }
        } else {
            outpost_pending_idx_ = -1;
            outpost_pending_since_ = -1.0;
        }

        double z_offsets[3] = {0, target.state[9], target.d_za};
        aim_z = z_offsets[aim_idx_];
        if (!smooth_aim_z_initialized_[aim_idx_]) {
            smooth_aim_z_[aim_idx_] = aim_z;
            smooth_aim_z_initialized_[aim_idx_] = true;
        } else {
            smooth_aim_z_[aim_idx_] += 0.3 * (aim_z - smooth_aim_z_[aim_idx_]);
        }
        aim_z = smooth_aim_z_[aim_idx_];

        double plate_angle = target_yaw + offs[aim_idx_];
        double aim_angle = plate_angle -
            std::copysign(outpost_yaw_offset_ * M_PI / 180.0, target_v_yaw);
        Eigen::Vector3d predicted_hit_point(
            target_from_muzzle_predicted.x() + r1 * std::cos(plate_angle),
            target_from_muzzle_predicted.y() + r1 * std::sin(plate_angle),
            target_from_muzzle_predicted.z() + aim_z);
        Eigen::Vector3d aim_point(
            target_from_muzzle_predicted.x() + r1 * std::cos(aim_angle),
            target_from_muzzle_predicted.y() + r1 * std::sin(aim_angle),
            target_from_muzzle_predicted.z() + aim_z);
        selected_visualization_position = predicted_hit_point;
        selected_visualization_yaw = plate_angle;
        selected_visualization_index = aim_idx_;
        static int select_phase_log_count = 0;
        if (++select_phase_log_count % 20 == 1) {
            getLogger()->debug(
                "[Outpost][SELECT_PHASE] current_angle={:.3f} target_v_yaw={:.3f} "
                "normalized_omega={:.3f} prediction_dt={:.3f} future_angle={:.3f} "
                "current_plate={} selected_plate={} candidate_phase={:.3f} "
                "candidate_z={:.3f}",
                target.state[6], target_v_yaw, target_v_yaw, dt, target_yaw,
                target.outpost_observed_plate, aim_idx_, plate_angle,
                target_position_predicted.z() + aim_z);
        }
        calcYawAndPitch(aim_point, yaw, pitch);
        distance = aim_point.norm();
        outpost_diff_deg = diffs[aim_idx_] * 180.0 / M_PI;
    }

    GimbalCommand cmd;
    cmd.distance = distance;

    if (armors_num == 3 && !outpost_single_plate && std::abs(target_v_yaw) > 0.01) {
        bool in_window = std::abs(outpost_diff_deg) < 30.0 &&
                         (target_v_yaw * outpost_diff_deg < 0);
        cmd.fire_advice = outpost_model_ok && in_window &&
                          isOnTarget(current_yaw_, current_pitch_, yaw, pitch, distance);
    }

    switch (state_) {
    case TRACKING_ARMOR:
        center_exit_since_ = -1.0;
        if (heldFor(
                std::abs(target_v_yaw) > max_tracking_v_yaw_,
                current_time, center_switch_confirm_time_, center_enter_since_)) {
            state_ = TRACKING_CENTER;
            center_enter_since_ = -1.0;
        }
        if (controller_delay_ != 0 && armors_num != 3) {
            Eigen::Vector3d delayed_position = target_position_predicted;
            delayed_position.x() += controller_delay_ * target.state[1];
            delayed_position.y() += controller_delay_ * target.state[3];
            delayed_position.z() += controller_delay_ * target.state[5];
            target_yaw += controller_delay_ * target_v_yaw;
            armor_positions = getArmorPositions(
                delayed_position, target_yaw, r1, r2, target.state[9], target.d_za, armors_num);
            idx = selectBestArmor(
                armor_positions, delayed_position, target_yaw, target_v_yaw,
                armors_num, flying_time, nullptr);
            chosen_armor_position = armor_positions.at(idx);
            distance = chosen_armor_position.norm();
            if (distance < 0.1) {
                has_selected_armor_ = false;
                selected_armor_index_ = -1;
                resetCommandDeadZone();
                return {};
            }
            selected_visualization_position = chosen_armor_position;
            selected_visualization_yaw = target_yaw;
            selected_visualization_index = idx;
            calcYawAndPitch(chosen_armor_position, yaw, pitch);
        }
        break;
    case TRACKING_CENTER:
        center_enter_since_ = -1.0;
        if (heldFor(
                std::abs(target_v_yaw) < max_tracking_v_yaw_,
                current_time, center_switch_confirm_time_, center_exit_since_)) {
            state_ = TRACKING_ARMOR;
            center_exit_since_ = -1.0;
        }
        calcYawAndPitch(target_from_muzzle_predicted, yaw, pitch);
        distance = target_from_muzzle_predicted.norm();
        cmd.distance = distance;
        cmd.fire_advice = false;
        selected_visualization_valid = false;
        break;
    }

    if (armors_num != 3) {
        cmd.fire_advice = state_ != TRACKING_CENTER && normal_model_ok &&
                          isOnTarget(current_yaw_, current_pitch_, yaw, pitch, distance);
    } else if (outpost_single_plate && state_ != TRACKING_CENTER) {
        cmd.fire_advice = outpost_model_ok &&
                          isOnTarget(current_yaw_, current_pitch_, yaw, pitch, distance);
    } else if (std::abs(target_v_yaw) <= 0.01 && state_ != TRACKING_CENTER) {
        cmd.fire_advice = outpost_model_ok &&
                          isOnTarget(current_yaw_, current_pitch_, yaw, pitch, distance);
    }

    auto offset = manual_comp_->correct(target_position.head<2>().norm(), target_position.z());
    double pitch_off = offset[0] * M_PI / 180.0;
    double yaw_off = offset[1] * M_PI / 180.0;
    cmd.pitch = (pitch + pitch_off) * 180.0 / M_PI;
    cmd.yaw = math::limitRad(yaw + yaw_off) * 180.0 / M_PI;
    applyCommandDeadZone(cmd);

    const bool prediction_valid = selected_visualization_valid;
    const bool stale_observation_visual =
        is_outpost && !outpost_observation_fresh && target.outpost_last_seen_time <= 0.0;
    const bool selected_prediction_valid =
        selected_visualization_valid && (!is_outpost || outpost_prediction_ok);

    if (is_outpost && prediction_valid && !selected_prediction_valid) {
        static unsigned long long vis_invalid_log_sequence = 0;
        ++vis_invalid_log_sequence;
        if (vis_invalid_log_sequence % 20 == 1) {
            getLogger()->debug(
                "[Outpost][VIS_INVALID] seq={} model_ok={} fresh={} stale_observation={} "
                "chosen_plate={} prediction_valid={} selected_prediction_valid={}",
                vis_invalid_log_sequence, outpost_model_ok ? 1 : 0,
                outpost_observation_fresh ? 1 : 0, stale_observation_visual ? 1 : 0,
                selected_visualization_index, prediction_valid ? 1 : 0,
                selected_prediction_valid ? 1 : 0);
        }
    }

    if (selected_prediction_valid) {
        selected_armor_pos_ = muzzle_world + selected_visualization_position;
        selected_armor_yaw_ = selected_visualization_yaw;
        selected_armor_index_ = selected_visualization_index;
        has_selected_armor_ = true;
    } else {
        selected_armor_pos_ = Eigen::Vector3d::Zero();
        selected_armor_yaw_ = 0.0;
        has_selected_armor_ = false;
        selected_armor_index_ = -1;
    }

    if (is_outpost && has_selected_armor_) {
        static int select_log_count = 0;
        static int last_logged_index = -2;
        const bool selection_changed = selected_armor_index_ != last_logged_index;
        if (selection_changed || ++select_log_count % 100 == 0) {
            const double unseen = target.outpost_last_seen_time > 0 ?
                current_time - target.outpost_last_seen_time : -1.0;
            getLogger()->debug(
                "[Outpost][SELECT] mode={} selected={} observed={} changed={} "
                "fresh={} unseen={:.3f}s phase={:.1f}deg model_ok={} "
                "fire={} diff={:.1f}deg cmd=({:.2f},{:.2f})",
                outpost_single_plate ? "STOPPED" : "ROTATING",
                selected_armor_index_, target.outpost_observed_plate,
                selection_changed, outpost_observation_fresh, unseen,
                target.outpost_phase_error * 180.0 / M_PI, outpost_model_ok,
                cmd.fire_advice, outpost_diff_deg, cmd.yaw, cmd.pitch);
            last_logged_index = selected_armor_index_;
        }
    }

    return cmd;
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
    return compensator_->getTrajectory(15, 0);
}

std::vector<cv::Point2f> Solver::reproject(const Eigen::Vector3d &xyz_world,
                                            double yaw,
                                            ArmorType type,
                                            bool is_outpost) const {
    double sin_y = std::sin(yaw);
    double cos_y = std::cos(yaw);
    double pitch_off = is_outpost ? -15.0 * M_PI / 180.0 : 15.0 * M_PI / 180.0;
    double sp = std::sin(pitch_off);
    double cp = std::cos(pitch_off);
    Eigen::Matrix3d R_a2w;
    R_a2w << cos_y * cp, -sin_y, cos_y * sp,
             sin_y * cp, cos_y, sin_y * sp,
             -sp, 0, cp;

    Eigen::Matrix3d R_a2c = R_c2g_.transpose() * R_g2w_.transpose() * R_a2w;
    Eigen::Vector3d t_a2c = R_c2g_.transpose() * (R_g2w_.transpose() * xyz_world - t_c2g_);

    cv::Vec3d rvec;
    cv::Mat rmat;
    cv::eigen2cv(R_a2c, rmat);
    cv::Rodrigues(rmat, rvec);
    cv::Vec3d tvec(t_a2c[0], t_a2c[1], t_a2c[2]);

    const auto &obj = (type == ArmorType::LARGE) ? BIG_OBJ_PTS : SMALL_OBJ_PTS;
    std::vector<cv::Point2f> pts;
    cv::projectPoints(obj, rvec, tvec, K_, D_, pts);
    return pts;
}

}  // namespace autoaim
