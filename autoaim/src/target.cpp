#include "target.hpp"

#include "utils/logger.hpp"
#include "utils/math.hpp"

#include <algorithm>
#include <cmath>

namespace autoaim {

namespace {
constexpr double kOutpostOmega = 2.513;
constexpr double kMinCalibratedZGap = 0.04;
constexpr double kMaxActiveZAssociationError = 0.055;
constexpr double kMinActiveZAssociationMargin = 0.015;
constexpr double kMaxAssociationError = 55.0 * M_PI / 180.0;
constexpr double kMaxPhaseCorrection = 15.0 * M_PI / 180.0;
constexpr double kMinCircleFitDuration = 1.20;
constexpr double kMinCircleFitSpan = 0.30;
constexpr double kMinCircleFitSpanRadiusRatio = 1.70;
constexpr double kMaxCircleFitMedianResidual = 0.06;
constexpr double kMaxCircleFitP80Residual = 0.10;
constexpr double kMinFittedRadius = 0.15;
constexpr double kMaxFittedRadius = 0.36;
constexpr double kFitConfirmationInterval = 0.25;
constexpr double kMaxFitCenterShift = 0.05;
constexpr double kMaxFitRadiusShift = 0.03;
constexpr double kMaxCircleCollectionGap = 1.0;
constexpr double kStopConfirmationWindow = 0.35;
constexpr double kStaticMotionSpan = 0.05;
constexpr size_t kMinCircleFitSamples = 12;
constexpr size_t kMaxCircleFitSamples = 1024;

struct CircleFitResult {
    double cx = 0;
    double cy = 0;
    double free_radius = 0;
    double median_residual = 0;
    double p80_residual = 0;
    double span = 0;
};

bool fitRobustFreeRadiusCircle(const std::vector<double> &xs,
                               const std::vector<double> &ys,
                               CircleFitResult &result) {
    if (xs.size() != ys.size() || xs.size() < kMinCircleFitSamples) return false;

    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < xs.size(); ++i) {
        const Eigen::Vector3d row(2.0 * xs[i], 2.0 * ys[i], 1.0);
        const double value = xs[i] * xs[i] + ys[i] * ys[i];
        normal += row * row.transpose();
        rhs += row * value;
    }
    if (std::abs(normal.determinant()) < 1e-10) return false;

    const Eigen::Vector3d algebraic = normal.ldlt().solve(rhs);
    if (!algebraic.allFinite()) return false;
    double cx = algebraic.x();
    double cy = algebraic.y();
    const double radius_sq = algebraic.z() + cx * cx + cy * cy;
    if (!std::isfinite(radius_sq) || radius_sq <= 0) return false;
    double fitted_radius = std::sqrt(radius_sq);

    constexpr double kHuberDelta = 0.06;
    for (int iteration = 0; iteration < 12; ++iteration) {
        Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < xs.size(); ++i) {
            const double dx = cx - xs[i];
            const double dy = cy - ys[i];
            const double distance = std::hypot(dx, dy);
            if (!std::isfinite(distance) || distance < 1e-6) continue;
            const double error = distance - fitted_radius;
            const double weight = std::abs(error) <= kHuberDelta ?
                1.0 : kHuberDelta / std::abs(error);
            const Eigen::Vector3d jacobian(dx / distance, dy / distance, -1.0);
            hessian += weight * jacobian * jacobian.transpose();
            gradient += weight * jacobian * error;
        }
        if (std::abs(hessian.determinant()) < 1e-10) return false;
        Eigen::Vector3d step = -hessian.ldlt().solve(gradient);
        if (!step.allFinite()) return false;
        const double center_step = std::hypot(step.x(), step.y());
        if (center_step > 0.10) {
            step.x() *= 0.10 / center_step;
            step.y() *= 0.10 / center_step;
        }
        step.z() = std::clamp(step.z(), -0.05, 0.05);
        cx += step.x();
        cy += step.y();
        fitted_radius += step.z();
        if (!std::isfinite(fitted_radius) || fitted_radius <= 0.05) return false;
        if (step.norm() < 1e-6) break;
    }

    std::vector<double> residuals;
    residuals.reserve(xs.size());
    double max_span_sq = 0;
    for (size_t i = 0; i < xs.size(); ++i) {
        residuals.push_back(
            std::abs(std::hypot(xs[i] - cx, ys[i] - cy) - fitted_radius));
        for (size_t j = i + 1; j < xs.size(); ++j) {
            const double dx = xs[i] - xs[j];
            const double dy = ys[i] - ys[j];
            max_span_sq = std::max(max_span_sq, dx * dx + dy * dy);
        }
    }
    std::sort(residuals.begin(), residuals.end());
    const size_t median_index = residuals.size() / 2;
    const size_t p80_index = std::min(
        residuals.size() - 1,
        static_cast<size_t>(0.8 * static_cast<double>(residuals.size() - 1)));

    result.cx = cx;
    result.cy = cy;
    result.free_radius = fitted_radius;
    result.median_residual = residuals[median_index];
    result.p80_residual = residuals[p80_index];
    result.span = std::sqrt(max_span_sq);
    return std::isfinite(cx) && std::isfinite(cy) &&
           std::isfinite(result.free_radius) && std::isfinite(result.span);
}

double recomputeAccumulatedAngle(const std::vector<double> &xs,
                                 const std::vector<double> &ys,
                                 double cx,
                                 double cy,
                                 double &last_angle) {
    if (xs.empty() || xs.size() != ys.size()) {
        last_angle = 0;
        return 0;
    }

    constexpr double kPlateSpacing = 2.0 * M_PI / 3.0;
    double previous = std::atan2(ys.front() - cy, xs.front() - cx);
    double accumulated = 0;
    for (size_t i = 1; i < xs.size(); ++i) {
        const double angle = std::atan2(ys[i] - cy, xs[i] - cx);
        double step = math::limitRad(angle - previous);
        step -= std::round(step / kPlateSpacing) * kPlateSpacing;
        accumulated += step;
        previous = angle;
    }
    last_angle = previous;
    return accumulated;
}
}  // namespace

OutpostTracker::OutpostTracker() { reset(); }

void OutpostTracker::reset() {
    state_ = COLLECTING;
    cx_ = cy_ = omega_ = 0;
    last_obs_angle_ = last_obs_time_ = 0;
    last_obs_plate_ = 0;
    static_x_ = static_y_ = static_z_ = static_yaw_ = 0;
    traj_x_.clear();
    traj_y_.clear();
    motion_x_.clear();
    motion_y_.clear();
    motion_t_.clear();
    accum_dang_ = last_ang_ = last_t_ = last_z_ = 0;
    for (int i = 0; i < 3; ++i) {
        z_centers_[i] = 0;
        z_seen_[i] = false;
    }
    angle_offsets_[0] = 0;
    angle_offsets_[1] = 2.0 * M_PI / 3.0;
    angle_offsets_[2] = -2.0 * M_PI / 3.0;
    z_calibrated_ = false;
    center_init_ = false;
    collect_count_ = 0;
    collect_start_t_ = 0;
    collect_span_ = 0;
    fit_candidate_valid_ = false;
    fit_candidate_cx_ = fit_candidate_cy_ = fit_candidate_radius_ = 0;
    fit_candidate_time_ = 0;
    current_plate_ = 1;
    outlier_start_t_ = 0;
    radial_outlier_start_t_ = 0;
    last_measurement_time_ = 0;
    last_seen_t_ = 0;
    phase_error_ = 0;
}

double OutpostTracker::predictAngle(double t, int plate, double future_dt) const {
    const double observation_dt = std::clamp(
        t - last_obs_time_, 0.0, max_phase_extrapolation_);
    const double plate_off = (plate >= 0 && plate < 3) ? angle_offsets_[plate] : 0.0;
    const double last_off = (last_obs_plate_ >= 0 && last_obs_plate_ < 3) ?
        angle_offsets_[last_obs_plate_] : 0.0;
    return math::limitRad(last_obs_angle_ +
                          omega_ * (observation_dt + std::max(0.0, future_dt)) +
                          plate_off - last_off);
}

void OutpostTracker::updateZPlate(int plate, double z) {
    if (plate < 0 || plate >= 3 || !std::isfinite(z)) return;
    if (!z_seen_[plate]) {
        z_centers_[plate] = z;
        z_seen_[plate] = true;
        getLogger()->debug(
            "[Outpost][Z_SLOT] slot={} offset={:.1f}deg z={:.3f}",
            plate, angle_offsets_[plate] * 180.0 / M_PI, z);
        return;
    }
    z_centers_[plate] =
        (1.0 - z_learning_rate_) * z_centers_[plate] + z_learning_rate_ * z;
}

void OutpostTracker::switchToStatic(double x, double y, double z, double yaw) {
    state_ = STATIC;
    static_x_ = x;
    static_y_ = y;
    static_z_ = z;
    static_yaw_ = yaw;
    current_plate_ = 0;
    z_calibrated_ = false;
    phase_error_ = 0;
    outlier_start_t_ = 0;
}

bool OutpostTracker::addMeasurement(double x, double y, double z, double yaw, double t) {
    const double observation_gap = last_measurement_time_ > 0 ?
        t - last_measurement_time_ : 0.0;
    if (state_ == COLLECTING && last_measurement_time_ > 0 &&
        (!std::isfinite(observation_gap) || observation_gap < 0.0 ||
         observation_gap > kMaxCircleCollectionGap)) {
        getLogger()->debug(
            "[Outpost][FIT_WAIT] collection restarted after {:.3f}s observation gap",
            observation_gap);
        reset();
    }
    if (outlier_start_t_ > 0 &&
        (!std::isfinite(observation_gap) || observation_gap < 0.0 ||
         observation_gap > max_outlier_duration_)) {
        outlier_start_t_ = 0;
    }
    if (radial_outlier_start_t_ > 0 &&
        (!std::isfinite(observation_gap) || observation_gap < 0.0 ||
         observation_gap > max_outlier_duration_)) {
        radial_outlier_start_t_ = 0;
    }
    if (state_ == ACTIVE && last_measurement_time_ > 0 &&
        (!std::isfinite(observation_gap) || observation_gap < 0.0 ||
         observation_gap > max_observation_extrapolation_)) {
        motion_x_.clear();
        motion_y_.clear();
        motion_t_.clear();
    }
    last_measurement_time_ = t;

    if (state_ == COLLECTING) {
        last_seen_t_ = t;
        for (size_t i = 0; i < traj_x_.size(); ++i) {
            collect_span_ = std::max(
                collect_span_, std::hypot(x - traj_x_[i], y - traj_y_[i]));
        }
        traj_x_.push_back(x);
        traj_y_.push_back(y);
        if (traj_x_.size() > kMaxCircleFitSamples) {
            traj_x_.erase(traj_x_.begin());
            traj_y_.erase(traj_y_.begin());
        }
        const double new_cx = x - fixed_R_ * std::cos(yaw);
        const double new_cy = y - fixed_R_ * std::sin(yaw);
        const double opposite_cx = x + fixed_R_ * std::cos(yaw);
        const double opposite_cy = y + fixed_R_ * std::sin(yaw);

        static int collect_geometry_log_count = 0;
        if (++collect_geometry_log_count % 40 == 0) {
            getLogger()->debug(
                "[Outpost][GEOM_COLLECT] meas=({:.3f},{:.3f},{:.3f}) yaw={:.1f}deg "
                "center_minus=({:.3f},{:.3f}) center_plus=({:.3f},{:.3f})",
                x, y, z, yaw * 180.0 / M_PI,
                new_cx, new_cy, opposite_cx, opposite_cy);
        }

        if (!center_init_) {
            center_init_ = true;
            collect_count_ = 1;
            collect_start_t_ = t;
            last_t_ = t;
            last_z_ = z;
            return false;
        }

        last_t_ = t;
        last_z_ = z;
        ++collect_count_;

        if (collect_count_ < 3 || t - collect_start_t_ < min_collect_duration_) return false;

        const double dx = traj_x_.back() - traj_x_.front();
        const double dy = traj_y_.back() - traj_y_.front();
        const double endpoint_span = std::hypot(dx, dy);
        if (collect_span_ < kStaticMotionSpan) {
            switchToStatic(x, y, z, yaw);
            getLogger()->info(
                "[Outpost] STATIC after {:.3f}s observation span={:.3f} endpoint={:.3f}",
                t - collect_start_t_, collect_span_, endpoint_span);
            return true;
        }

        const double collect_duration = t - collect_start_t_;
        if (collect_duration < kMinCircleFitDuration ||
            traj_x_.size() < kMinCircleFitSamples || collect_count_ % 5 != 0) {
            return false;
        }

        CircleFitResult fit;
        const bool fit_solved = fitRobustFreeRadiusCircle(traj_x_, traj_y_, fit);
        const double fit_coverage = fit_solved && fit.free_radius > 0 ?
            fit.span / fit.free_radius : 0.0;
        const bool fit_quality_ok = fit_solved && fit.span >= kMinCircleFitSpan &&
            fit_coverage >= kMinCircleFitSpanRadiusRatio &&
            fit.free_radius >= kMinFittedRadius && fit.free_radius <= kMaxFittedRadius &&
            fit.median_residual <= kMaxCircleFitMedianResidual &&
            fit.p80_residual <= kMaxCircleFitP80Residual;
        if (!fit_quality_ok) {
            fit_candidate_valid_ = false;
            static int rejected_fit_log_count = 0;
            if (++rejected_fit_log_count % 10 == 0) {
                getLogger()->debug(
                    "[Outpost][FIT_WAIT] solved={} n={} duration={:.3f}s span={:.3f} "
                    "coverage={:.2f} free_R={:.3f} med={:.3f} p80={:.3f}",
                    fit_solved, traj_x_.size(), collect_duration, fit.span,
                    fit_coverage, fit.free_radius,
                    fit.median_residual, fit.p80_residual);
            }
            return false;
        }

        if (!fit_candidate_valid_) {
            fit_candidate_valid_ = true;
            fit_candidate_cx_ = fit.cx;
            fit_candidate_cy_ = fit.cy;
            fit_candidate_radius_ = fit.free_radius;
            fit_candidate_time_ = t;
            getLogger()->debug(
                "[Outpost][FIT_CANDIDATE] center=({:.3f},{:.3f}) R={:.3f} "
                "med={:.3f} p80={:.3f} span={:.3f} coverage={:.2f} n={}",
                fit.cx, fit.cy, fit.free_radius, fit.median_residual,
                fit.p80_residual, fit.span, fit_coverage, traj_x_.size());
            return false;
        }

        const double center_shift = std::hypot(
            fit.cx - fit_candidate_cx_, fit.cy - fit_candidate_cy_);
        const double radius_shift = std::abs(fit.free_radius - fit_candidate_radius_);
        const double confirmation_age = t - fit_candidate_time_;
        if (!std::isfinite(confirmation_age) || confirmation_age < 0.0 ||
            center_shift > kMaxFitCenterShift || radius_shift > kMaxFitRadiusShift) {
            getLogger()->debug(
                "[Outpost][FIT_RESTART] age={:.3f}s center_shift={:.3f} "
                "radius_shift={:.3f} new=({:.3f},{:.3f},R={:.3f})",
                confirmation_age, center_shift, radius_shift,
                fit.cx, fit.cy, fit.free_radius);
            fit_candidate_cx_ = fit.cx;
            fit_candidate_cy_ = fit.cy;
            fit_candidate_radius_ = fit.free_radius;
            fit_candidate_time_ = t;
            return false;
        }
        if (confirmation_age < kFitConfirmationInterval) return false;

        double fitted_last_angle = 0;
        const double fitted_accumulated_angle = recomputeAccumulatedAngle(
            traj_x_, traj_y_, fit.cx, fit.cy, fitted_last_angle);
        if (!std::isfinite(fitted_accumulated_angle) ||
            std::abs(fitted_accumulated_angle) < 0.15) {
            getLogger()->debug(
                "[Outpost][FIT_WAIT] direction ambiguous accum={:.3f}rad n={} span={:.3f}",
                fitted_accumulated_angle, traj_x_.size(), fit.span);
            return false;
        }

        cx_ = fit.cx;
        cy_ = fit.cy;
        accum_dang_ = fitted_accumulated_angle;
        last_ang_ = fitted_last_angle;
        omega_ = (accum_dang_ >= 0) ? kOutpostOmega : -kOutpostOmega;
        z_centers_[0] = z;
        z_seen_[0] = true;
        current_plate_ = 0;
        last_obs_angle_ = fitted_last_angle;
        last_obs_time_ = t;
        last_obs_plate_ = current_plate_;
        state_ = CALIBRATING;
        outlier_start_t_ = 0;
        getLogger()->info(
            "[Outpost] CALIBRATING cx=({:.3f},{:.3f}) omega={:.3f} "
            "fit_R={:.3f} med={:.3f} p80={:.3f} span={:.3f} coverage={:.2f} n={} "
            "confirm={:.3f}s shift=({:.3f},{:.3f}) slot0_z={:.3f}",
            cx_, cy_, omega_, fit.free_radius, fit.median_residual,
            fit.p80_residual, fit.span, fit_coverage, traj_x_.size(), confirmation_age,
            center_shift, radius_shift, z);
        return true;
    }

    if (state_ == STATIC) {
        constexpr double alpha = 0.3;
        static_x_ = alpha * x + (1.0 - alpha) * static_x_;
        static_y_ = alpha * y + (1.0 - alpha) * static_y_;
        static_z_ = alpha * z + (1.0 - alpha) * static_z_;
        static_yaw_ = math::limitRad(
            static_yaw_ + alpha * math::limitRad(yaw - static_yaw_));
        last_seen_t_ = t;
        return true;
    }

    const double ang = std::atan2(y - cy_, x - cx_);
    const double rho = std::hypot(x - cx_, y - cy_);
    const double radial_residual = rho - fixed_R_;
    const double yaw_radial_diff = math::limitRad(yaw - ang);
    const double minus_cx = x - fixed_R_ * std::cos(yaw);
    const double minus_cy = y - fixed_R_ * std::sin(yaw);
    const double plus_cx = x + fixed_R_ * std::cos(yaw);
    const double plus_cy = y + fixed_R_ * std::sin(yaw);

    if (std::abs(radial_residual) > residual_thres_) {
        const bool streak_started = radial_outlier_start_t_ <= 0;
        if (streak_started) radial_outlier_start_t_ = t;
        const double streak = std::max(0.0, t - radial_outlier_start_t_);

        static int radial_outlier_log_count = 0;
        if (streak_started || ++radial_outlier_log_count % 20 == 0) {
            getLogger()->debug(
                "[Outpost][RADIAL] rho={:.3f} res={:+.3f} streak={:.3f}s "
                "meas=({:.3f},{:.3f},{:.3f}) yaw={:.1f}deg radial={:.1f}deg "
                "yaw_radial={:.1f}deg model_center=({:.3f},{:.3f}) "
                "center_minus=({:.3f},{:.3f}) center_plus=({:.3f},{:.3f})",
                rho, radial_residual, streak, x, y, z,
                yaw * 180.0 / M_PI, ang * 180.0 / M_PI,
                yaw_radial_diff * 180.0 / M_PI, cx_, cy_,
                minus_cx, minus_cy, plus_cx, plus_cy);
        }
        return false;
    }
    radial_outlier_start_t_ = 0;

    static int accepted_geometry_log_count = 0;
    if (++accepted_geometry_log_count % 100 == 0) {
        getLogger()->debug(
            "[Outpost][GEOM_OK] state={} rho={:.3f} res={:+.3f} "
            "yaw_radial={:.1f}deg model_center=({:.3f},{:.3f}) "
            "center_minus=({:.3f},{:.3f}) center_plus=({:.3f},{:.3f})",
            static_cast<int>(state_), rho, radial_residual,
            yaw_radial_diff * 180.0 / M_PI, cx_, cy_,
            minus_cx, minus_cy, plus_cx, plus_cy);
    }

    if (state_ == ACTIVE) {
        motion_x_.push_back(x);
        motion_y_.push_back(y);
        motion_t_.push_back(t);
        while (!motion_t_.empty() &&
               t - motion_t_.front() > kStopConfirmationWindow) {
            motion_x_.erase(motion_x_.begin());
            motion_y_.erase(motion_y_.begin());
            motion_t_.erase(motion_t_.begin());
        }

        if (motion_t_.size() >= 8 &&
            motion_t_.back() - motion_t_.front() >= 0.30) {
            double motion_span = 0;
            for (size_t i = 0; i < motion_x_.size(); ++i) {
                for (size_t j = i + 1; j < motion_x_.size(); ++j) {
                    motion_span = std::max(
                        motion_span,
                        std::hypot(motion_x_[i] - motion_x_[j],
                                   motion_y_[i] - motion_y_[j]));
                }
            }
            if (motion_span < kStaticMotionSpan) {
                const double motion_duration = motion_t_.back() - motion_t_.front();
                last_seen_t_ = t;
                switchToStatic(x, y, z, yaw);
                getLogger()->info(
                    "[Outpost] STOPPED after {:.3f}s fresh motion span={:.3f}",
                    motion_duration, motion_span);
                return true;
            }
        }
    }

    int observed_plate = 0;
    double corrected_angle = ang;
    double phase_residual = 0;
    double predicted_angles[3] = {ang, ang, ang};
    double association_errors[3] = {0, 0, 0};
    if ((state_ == CALIBRATING || state_ == ACTIVE) && last_obs_time_ > 0) {
        int best_plate = 0;
        for (int i = 0; i < 3; ++i) {
            predicted_angles[i] = predictAngle(t, i);
            association_errors[i] = math::limitRad(ang - predicted_angles[i]);
        }
        double best_error = association_errors[0];
        double best_abs_error = std::abs(best_error);
        for (int i = 1; i < 3; ++i) {
            const double error = association_errors[i];
            if (std::abs(error) < best_abs_error) {
                best_plate = i;
                best_error = error;
                best_abs_error = std::abs(error);
            }
        }

        bool phase_reanchored = false;
        if (state_ == ACTIVE && z_calibrated_) {
            int z_plate = 0;
            double z_errors[3] = {
                std::abs(z - z_centers_[0]),
                std::abs(z - z_centers_[1]),
                std::abs(z - z_centers_[2])};
            for (int i = 1; i < 3; ++i) {
                if (z_errors[i] < z_errors[z_plate]) z_plate = i;
            }
            double z_second_error = 1e9;
            for (int i = 0; i < 3; ++i) {
                if (i != z_plate) {
                    z_second_error = std::min(z_second_error, z_errors[i]);
                }
            }

            const double z_best_error = z_errors[z_plate];
            const double z_margin = z_second_error - z_best_error;
            const bool z_identity_confident =
                std::isfinite(z_best_error) && std::isfinite(z_margin) &&
                z_best_error <= kMaxActiveZAssociationError &&
                z_margin >= kMinActiveZAssociationMargin;

            const int tracked_plate = current_plate_ >= 0 && current_plate_ < 3 ?
                current_plate_ : last_obs_plate_;
            const int phase_best = best_plate;
            const double phase_gate = kMaxAssociationError;
            const double tracked_phase_error = association_errors[tracked_plate];
            const double tracked_phase_abs_error = std::abs(tracked_phase_error);
            const double measured_angle_jump = std::abs(
                math::limitRad(ang - last_obs_angle_));
            const bool tracked_phase_lost =
                tracked_phase_abs_error > phase_gate;
            const double new_phase_err =
                math::limitRad(ang - predictAngle(t, phase_best));

            if (!tracked_phase_lost) {
                // A continuous angular track is still the same physical plate.
                // View-dependent PnP Z drift must not change its identity.
                if (z_identity_confident && z_plate != tracked_plate) {
                    static unsigned long long identity_lock_log_sequence = 0;
                    ++identity_lock_log_sequence;
                    if (identity_lock_log_sequence % 20 == 1) {
                        getLogger()->debug(
                            "[Outpost][IDENTITY_LOCK] t={:.3f} tracked={} "
                            "phase_best={} height_candidate={} tracked_err={:+.1f}deg "
                            "new_phase_err={:+.1f}deg jump={:.1f}deg z={:.3f} "
                            "z_err={:.3f} margin={:.3f}",
                            t, tracked_plate, phase_best, z_plate,
                            tracked_phase_error * 180.0 / M_PI,
                            new_phase_err * 180.0 / M_PI,
                            measured_angle_jump * 180.0 / M_PI,
                            z, z_best_error, z_margin);
                    }
                }
                best_plate = tracked_plate;
                best_error = tracked_phase_error;
                best_abs_error = tracked_phase_abs_error;
            } else if (best_abs_error <= phase_gate) {
                getLogger()->debug(
                    "[Outpost][SWITCH_COMMIT] t={:.3f} from={} phase_best={} "
                    "height_plate={} tracked_err={:+.1f}deg new_phase_err={:+.1f}deg "
                    "jump={:.1f}deg z={:.3f} z_err={:.3f} margin={:.3f} z_confident={}",
                    t, tracked_plate, phase_best, z_plate,
                    tracked_phase_error * 180.0 / M_PI,
                    new_phase_err * 180.0 / M_PI,
                    measured_angle_jump * 180.0 / M_PI,
                    z, z_best_error, z_margin, z_identity_confident ? 1 : 0);
                best_plate = phase_best;
                best_error = new_phase_err;
                best_abs_error = std::abs(best_error);
                phase_reanchored = true;
            }
        }

        if (!phase_reanchored && best_abs_error > kMaxAssociationError) {
            const bool streak_started = outlier_start_t_ <= 0;
            if (streak_started) outlier_start_t_ = t;
            static unsigned long long phase_reject_log_sequence = 0;
            ++phase_reject_log_sequence;
            if (streak_started || phase_reject_log_sequence % 10 == 1) {
                getLogger()->debug(
                    "[Outpost][OBS_REJECT] reason=phase state={} dt={:.4f}s "
                    "world=({:.3f},{:.3f},{:.3f}) raw_ang={:.1f}deg "
                    "pred=({:.1f},{:.1f},{:.1f})deg "
                    "err=({:+.1f},{:+.1f},{:+.1f})deg best={} best_abs={:.1f}deg "
                    "streak={:.3f}s",
                    static_cast<int>(state_), observation_gap, x, y, z,
                    ang * 180.0 / M_PI,
                    predicted_angles[0] * 180.0 / M_PI,
                    predicted_angles[1] * 180.0 / M_PI,
                    predicted_angles[2] * 180.0 / M_PI,
                    association_errors[0] * 180.0 / M_PI,
                    association_errors[1] * 180.0 / M_PI,
                    association_errors[2] * 180.0 / M_PI,
                    best_plate, best_abs_error * 180.0 / M_PI,
                    std::max(0.0, t - outlier_start_t_));
            }
            if (t - outlier_start_t_ > max_outlier_duration_) {
                getLogger()->warn("[Outpost] Too many phase outliers, resetting");
                reset();
                return true;
            }
            return false;
        }
        observed_plate = best_plate;
        if (phase_reanchored) {
            // The phase model identifies the next slot; switch plate identity
            // and the observed phase together instead of freezing on the old slot.
            corrected_angle = ang;
            phase_residual = 0;
        } else {
            const double correction =
                std::clamp(best_error, -kMaxPhaseCorrection, kMaxPhaseCorrection);
            corrected_angle = math::limitRad(
                predictAngle(t, observed_plate) + correction);
            phase_residual = math::limitRad(ang - corrected_angle);
        }
    }

    const bool plate_changed = observed_plate != current_plate_;
    outlier_start_t_ = 0;
    last_seen_t_ = t;
    last_ang_ = ang;
    last_t_ = t;
    last_z_ = z;
    updateZPlate(observed_plate, z);
    current_plate_ = observed_plate;
    phase_error_ = phase_residual;
    last_obs_angle_ = corrected_angle;
    last_obs_time_ = t;
    last_obs_plate_ = observed_plate;

    static unsigned long long association_log_sequence = 0;
    ++association_log_sequence;
    if (plate_changed || association_log_sequence % 10 == 1) {
        getLogger()->debug(
            "[Outpost][OBS_ASSOC] seq={} state={} dt={:.4f}s "
            "world=({:.3f},{:.3f},{:.3f}) yaw={:.1f}deg "
            "rho={:.3f} radial_res={:+.3f} raw_ang={:.1f}deg "
            "pred=({:.1f},{:.1f},{:.1f})deg err=({:+.1f},{:+.1f},{:+.1f})deg "
            "chosen={} changed={} corrected={:.1f}deg residual={:+.1f}deg "
            "z=({:.3f},{:.3f},{:.3f}) seen=({},{},{})",
            association_log_sequence, static_cast<int>(state_), observation_gap,
            x, y, z, yaw * 180.0 / M_PI, rho, radial_residual,
            ang * 180.0 / M_PI,
            predicted_angles[0] * 180.0 / M_PI,
            predicted_angles[1] * 180.0 / M_PI,
            predicted_angles[2] * 180.0 / M_PI,
            association_errors[0] * 180.0 / M_PI,
            association_errors[1] * 180.0 / M_PI,
            association_errors[2] * 180.0 / M_PI,
            observed_plate, plate_changed ? 1 : 0,
            corrected_angle * 180.0 / M_PI,
            phase_residual * 180.0 / M_PI,
            z_centers_[0], z_centers_[1], z_centers_[2],
            z_seen_[0] ? 1 : 0, z_seen_[1] ? 1 : 0, z_seen_[2] ? 1 : 0);
    }

    if (state_ == CALIBRATING) {
        int seen = 0;
        for (int i = 0; i < 3; ++i) {
            if (z_seen_[i]) ++seen;
        }
        if (seen == 3) {
            double sorted_z[3] = {z_centers_[0], z_centers_[1], z_centers_[2]};
            std::sort(sorted_z, sorted_z + 3);
            const double low_gap = sorted_z[1] - sorted_z[0];
            const double high_gap = sorted_z[2] - sorted_z[1];
            if (low_gap < kMinCalibratedZGap || high_gap < kMinCalibratedZGap) {
                static int z_wait_log_count = 0;
                if (++z_wait_log_count % 40 == 0) {
                    getLogger()->debug(
                        "[Outpost][Z_WAIT] phase_z=[{:.3f},{:.3f},{:.3f}] "
                        "sorted_gaps=({:.3f},{:.3f})",
                        z_centers_[0], z_centers_[1], z_centers_[2],
                        low_gap, high_gap);
                }
                return false;
            }
            z_calibrated_ = true;
            state_ = ACTIVE;
            motion_x_.clear();
            motion_y_.clear();
            motion_t_.clear();
            getLogger()->info(
                "[Outpost] ACTIVE omega={:.3f} phase_z=[{:.3f},{:.3f},{:.3f}] "
                "sorted_gaps=({:.3f},{:.3f})",
                omega_, z_centers_[0], z_centers_[1], z_centers_[2],
                low_gap, high_gap);
            return true;
        }
    }

    return state_ == ACTIVE;
}

std::vector<Eigen::Vector3d> OutpostTracker::getPredictedPositions(
    double t, double future_dt) {
    std::vector<Eigen::Vector3d> positions;
    if (state_ == STATIC) {
        positions.emplace_back(static_x_, static_y_, static_z_);
    } else if (state_ == ACTIVE) {
        for (int i = 0; i < 3; ++i) {
            const double angle = predictAngle(t, i, future_dt);
            positions.emplace_back(cx_ + fixed_R_ * std::cos(angle),
                                   cy_ + fixed_R_ * std::sin(angle), z_centers_[i]);
        }
    }
    return positions;
}

double OutpostTracker::getPlateAngle(double t, int plate) const {
    if (state_ == STATIC) return static_yaw_;
    return predictAngle(t, plate);
}

bool OutpostTracker::getCircleParams(double &cx, double &cy, double &R, double &omega) {
    if (state_ != ACTIVE) return false;
    cx = cx_;
    cy = cy_;
    R = fixed_R_;
    omega = omega_;
    return true;
}

}  // namespace autoaim
