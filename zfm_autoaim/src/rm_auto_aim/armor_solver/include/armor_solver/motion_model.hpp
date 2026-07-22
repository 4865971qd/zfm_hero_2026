#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace zfm::auto_aim {

enum class MotionModel {
  CONSTANT_VELOCITY = 0,
  CONSTANT_ROTATION = 1,
  CONSTANT_VEL_ROT = 2,
  CONSTANT_STATIONARY = 3
};

constexpr int X_N = 10, Z_N = 4;

struct Predict {
  explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT)
  : dt(dt), model(model) {}

  template <typename T>
  void operator()(const T x0[X_N], T x1[X_N]) {
    for (int i = 0; i < X_N; i++) x1[i] = x0[i];
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_VELOCITY) {
      x1[0] += x0[1] * dt;
      x1[2] += x0[3] * dt;
      x1[4] += x0[5] * dt;
    } else {
      x1[1] *= 0.; x1[3] *= 0.; x1[5] *= 0.;
    }
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_ROTATION) {
      x1[6] += x0[7] * dt;
    } else {
      x1[7] *= 0.;
    }
  }
  double dt;
  MotionModel model;
};

struct Measure {
  template <typename T>
  void operator()(const T x[Z_N], T z[Z_N]) {
    z[0] = x[0] - ceres::cos(x[6]) * x[8];
    z[1] = x[2] - ceres::sin(x[6]) * x[8];
    z[2] = x[4] + x[9];
    z[3] = x[6];
  }
};

using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

class OutpostTracker {
public:
  enum OutpostState { COLLECTING = 0, CALIBRATING = 1, ACTIVE = 2, STATIC = 3 };

  OutpostTracker() { resetCollection(); }

  bool addMeasurement(double x, double y, double z, double yaw, double t) {
    (void)yaw;
    const double observation_gap = last_seen_t_ > 0 ? t - last_seen_t_ : 0.0;
    if (outlier_start_t_ > 0 &&
        (!std::isfinite(observation_gap) || observation_gap < 0.0 ||
         observation_gap > max_outlier_duration_)) {
      outlier_start_t_ = 0;
    }
    if (state_ == COLLECTING) {
      last_seen_t_ = t;
      traj_x_.push_back(x);
      traj_y_.push_back(y);
      if (traj_x_.size() > max_circle_fit_samples_) {
        traj_x_.erase(traj_x_.begin());
        traj_y_.erase(traj_y_.begin());
      }

      if (!center_initialized_) {
        center_initialized_ = true;
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
      if (std::hypot(dx, dy) < 0.05) {
        switchToStatic(x, y, z);
        ZFM_INFO("armor_solver", "[Outpost] STATIC after {:.3f}s observation",
                 t - collect_start_t_);
        return true;
      }

      const double collect_duration = t - collect_start_t_;
      if (collect_duration < min_circle_fit_duration_ ||
          traj_x_.size() < min_circle_fit_samples_ || collect_count_ % 5 != 0) {
        return false;
      }

      CircleFitResult fit;
      const bool fit_solved = fitKnownRadiusCircle(fit);
      const bool fit_quality_ok = fit_solved && fit.span >= min_circle_fit_span_ &&
        fit.free_radius >= min_fitted_radius_ && fit.free_radius <= max_fitted_radius_ &&
        fit.median_residual <= max_circle_fit_median_residual_ &&
        fit.p80_residual <= max_circle_fit_p80_residual_;
      if (!fit_quality_ok) {
        static int rejected_fit_log_count = 0;
        if (++rejected_fit_log_count % 10 == 0) {
          ZFM_DEBUG(
            "armor_solver",
            "[Outpost][FIT_WAIT] solved={} n={} duration={:.3f}s span={:.3f} "
            "free_R={:.3f} med={:.3f} p80={:.3f}",
            fit_solved, traj_x_.size(), collect_duration, fit.span,
            fit.free_radius, fit.median_residual, fit.p80_residual);
        }
        return false;
      }

      double fitted_last_angle = 0;
      const double fitted_accumulated_angle =
        recomputeAccumulatedAngle(fit.cx, fit.cy, fitted_last_angle);
      if (!std::isfinite(fitted_accumulated_angle) ||
          std::abs(fitted_accumulated_angle) < 0.15) {
        ZFM_DEBUG(
          "armor_solver",
          "[Outpost][FIT_WAIT] direction ambiguous accum={:.3f}rad n={} span={:.3f}",
          fitted_accumulated_angle, traj_x_.size(), fit.span);
        return false;
      }

      cx_ = fit.cx;
      cy_ = fit.cy;
      accum_dang_ = fitted_accumulated_angle;
      last_ang_ = fitted_last_angle;
      omega_ = (accum_dang_ >= 0) ? outpost_omega_ : -outpost_omega_;
      z_centers_[0] = z;
      z_seen_[0] = true;
      z_initialized_ = true;
      current_plate_ = 0;
      last_obs_angle_ = fitted_last_angle;
      last_obs_time_ = t;
      last_obs_plate_ = current_plate_;
      state_ = CALIBRATING;
      outlier_start_t_ = 0;
      ZFM_INFO(
        "armor_solver",
        "[Outpost] CALIBRATING cx=({:.3f},{:.3f}) omega={:.3f} "
        "fit_R={:.3f} med={:.3f} p80={:.3f} span={:.3f} n={}",
        cx_, cy_, omega_, fit.free_radius, fit.median_residual,
        fit.p80_residual, fit.span, traj_x_.size());
      return true;
    }

    if (state_ == STATIC) {
      constexpr double alpha = 0.3;
      static_x_ = alpha * x + (1.0 - alpha) * static_x_;
      static_y_ = alpha * y + (1.0 - alpha) * static_y_;
      static_z_ = alpha * z + (1.0 - alpha) * static_z_;
      last_seen_t_ = t;
      return false;
    }

    const double ang = std::atan2(y - cy_, x - cx_);
    const double rho = std::hypot(x - cx_, y - cy_);
    const double radial_residual = rho - fixed_R_;
    if (std::abs(radial_residual) > residual_threshold_) {
      const bool streak_started = outlier_start_t_ <= 0;
      if (streak_started) outlier_start_t_ = t;
      const double streak = std::max(0.0, t - outlier_start_t_);
      static int radial_outlier_log_count = 0;
      if (streak_started || ++radial_outlier_log_count % 20 == 0) {
        ZFM_DEBUG(
          "armor_solver",
          "[Outpost][RADIAL] rho={:.3f} res={:+.3f} streak={:.3f}s "
          "meas=({:.3f},{:.3f},{:.3f}) model_center=({:.3f},{:.3f})",
          rho, radial_residual, streak, x, y, z, cx_, cy_);
      }
      return false;
    }

    int observed_plate = findNearestPlate(z);
    double corrected_angle = ang;
    double phase_residual = 0;
    if ((state_ == CALIBRATING || state_ == ACTIVE) && last_obs_time_ > 0) {
      int best_plate = 0;
      double best_error = normalizeAngle(ang - predictAngle(t, 0));
      double best_abs_error = std::abs(best_error);
      for (int i = 1; i < 3; ++i) {
        const double error = normalizeAngle(ang - predictAngle(t, i));
        if (std::abs(error) < best_abs_error) {
          best_plate = i;
          best_error = error;
          best_abs_error = std::abs(error);
        }
      }
      bool phase_reanchored = false;
      if (state_ == ACTIVE && z_fully_calibrated_) {
        const int tracked_plate = current_plate_ >= 0 && current_plate_ < 3 ?
            current_plate_ : last_obs_plate_;
        const int phase_best = best_plate;
        const double tracked_error =
            normalizeAngle(ang - predictAngle(t, tracked_plate));
        const double tracked_abs_error = std::abs(tracked_error);
        if (tracked_abs_error <= max_association_error_) {
          best_plate = tracked_plate;
          best_error = tracked_error;
          best_abs_error = tracked_abs_error;
        } else if (best_abs_error <= max_association_error_) {
          ZFM_DEBUG(
            "armor_solver",
            "[Outpost][SWITCH_COMMIT] t={:.3f} from={} phase_best={} "
            "tracked_err={:+.1f}deg new_phase_err={:+.1f}deg",
            t, tracked_plate, phase_best,
            tracked_error * 180.0 / M_PI, best_error * 180.0 / M_PI);
          best_plate = phase_best;
          phase_reanchored = true;
        }
      }
      if (!phase_reanchored && best_abs_error > max_association_error_) {
        if (outlier_start_t_ <= 0) outlier_start_t_ = t;
        static int phase_outlier_log_count = 0;
        if (++phase_outlier_log_count % 10 == 1) {
          ZFM_DEBUG(
            "armor_solver",
            "[Outpost][OBS_REJECT] reason=phase state={} raw={:.1f}deg best={} "
            "err={:.1f}deg streak={:.3f}s",
            static_cast<int>(state_), ang * 180.0 / M_PI, best_plate,
            best_abs_error * 180.0 / M_PI, std::max(0.0, t - outlier_start_t_));
        }
        return false;
      }
      observed_plate = best_plate;
      if (phase_reanchored) {
        corrected_angle = ang;
        phase_residual = 0;
      } else {
        const double correction =
          std::clamp(best_error, -max_phase_correction_, max_phase_correction_);
        corrected_angle = normalizeAngle(predictAngle(t, observed_plate) + correction);
        phase_residual = normalizeAngle(ang - corrected_angle);
      }
    }

    outlier_start_t_ = 0;
    last_seen_t_ = t;
    last_ang_ = ang;
    last_t_ = t;
    last_z_ = z;
    if (state_ != ACTIVE) {
      updateZPlate(observed_plate, z);
    }
    current_plate_ = observed_plate;
    phase_error_ = phase_residual;
    last_obs_angle_ = corrected_angle;
    last_obs_time_ = t;
    last_obs_plate_ = observed_plate;

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
        if (low_gap < min_calibrated_z_gap_ || high_gap < min_calibrated_z_gap_) {
          static int z_wait_log_count = 0;
          if (++z_wait_log_count % 40 == 0) {
            ZFM_DEBUG(
              "armor_solver",
              "[Outpost][Z_WAIT] phase_z=[{:.3f},{:.3f},{:.3f}] "
              "sorted_gaps=({:.3f},{:.3f})",
              z_centers_[0], z_centers_[1], z_centers_[2], low_gap, high_gap);
          }
          return false;
        }
        phase_error_ = 0;
        z_fully_calibrated_ = true;
        state_ = ACTIVE;
        ZFM_INFO(
          "armor_solver",
          "[Outpost] ACTIVE omega={:.3f} phase_z=[{:.3f},{:.3f},{:.3f}] "
          "sorted_gaps=({:.3f},{:.3f})",
          omega_, z_centers_[0], z_centers_[1], z_centers_[2],
          low_gap, high_gap);
        return true;
      }
    }

    traj_x_.push_back(x);
    traj_y_.push_back(y);
    if (traj_x_.size() > 30) {
      traj_x_.erase(traj_x_.begin());
      traj_y_.erase(traj_y_.begin());
    }
    return false;
  }

  double getLastSeenTime() const { return last_seen_t_; }

  std::vector<Eigen::Vector3d> getPredictedPositions(double t, double future_dt = 0) {
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

  double getPlateAngle(double t, int plate) const { return predictAngle(t, plate); }

  bool getCircleParams(double &cx, double &cy, double &R, double &omega) {
    if (state_ != ACTIVE && state_ != STATIC) return false;
    cx = cx_; cy = cy_; R = fixed_R_; omega = omega_;
    return true;
  }

  OutpostState getState() const { return state_; }
  int getCollectCount() const { return collect_count_; }
  double getZCenter(int i) const { return (i >= 0 && i < 3) ? z_centers_[i] : 0; }
  double getAngleOffset(int i) const { return (i >= 0 && i < 3) ? angle_offsets_[i] : 0; }
  bool isZCalibrated() const { return z_fully_calibrated_; }
  double getPhaseError() const { return phase_error_; }
  int currentPlate() const { return current_plate_; }

  void resetCollection() {
    state_ = COLLECTING;
    cx_ = cy_ = omega_ = 0;
    last_obs_angle_ = last_obs_time_ = 0;
    last_obs_plate_ = 0;
    static_x_ = static_y_ = static_z_ = 0;
    traj_x_.clear();
    traj_y_.clear();
    accum_dang_ = last_ang_ = last_t_ = last_z_ = 0;
    for (int i = 0; i < 3; ++i) {
      z_centers_[i] = 0;
      z_seen_[i] = false;
    }
    angle_offsets_[0] = 0;
    angle_offsets_[1] = 2.0 * M_PI / 3.0;
    angle_offsets_[2] = -2.0 * M_PI / 3.0;
    z_initialized_ = false;
    z_fully_calibrated_ = false;
    center_initialized_ = false;
    collect_count_ = 0;
    collect_start_t_ = 0;
    current_plate_ = 0;
    outlier_start_t_ = 0;
    last_seen_t_ = 0;
    phase_error_ = 0;
  }

private:
  struct CircleFitResult {
    double cx = 0;
    double cy = 0;
    double free_radius = 0;
    double median_residual = 0;
    double p80_residual = 0;
    double span = 0;
  };

  static double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  bool fitKnownRadiusCircle(CircleFitResult &result) const {
    if (traj_x_.size() != traj_y_.size() ||
        traj_x_.size() < min_circle_fit_samples_) {
      return false;
    }

    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < traj_x_.size(); ++i) {
      const Eigen::Vector3d row(2.0 * traj_x_[i], 2.0 * traj_y_[i], 1.0);
      const double value =
        traj_x_[i] * traj_x_[i] + traj_y_[i] * traj_y_[i];
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

    constexpr double huber_delta = 0.08;
    for (int iteration = 0; iteration < 12; ++iteration) {
      Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
      Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
      for (size_t i = 0; i < traj_x_.size(); ++i) {
        const double dx = cx - traj_x_[i];
        const double dy = cy - traj_y_[i];
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance) || distance < 1e-6) continue;
        const double error = distance - fixed_R_;
        const double weight = std::abs(error) <= huber_delta ?
          1.0 : huber_delta / std::abs(error);
        const Eigen::Vector2d jacobian(dx / distance, dy / distance);
        hessian += weight * jacobian * jacobian.transpose();
        gradient += weight * jacobian * error;
      }
      if (std::abs(hessian.determinant()) < 1e-10) return false;
      Eigen::Vector2d step = -hessian.ldlt().solve(gradient);
      if (!step.allFinite()) return false;
      if (step.norm() > 0.10) step *= 0.10 / step.norm();
      cx += step.x();
      cy += step.y();
      if (step.norm() < 1e-6) break;
    }

    std::vector<double> residuals;
    residuals.reserve(traj_x_.size());
    double max_span_sq = 0;
    for (size_t i = 0; i < traj_x_.size(); ++i) {
      residuals.push_back(std::abs(
        std::hypot(traj_x_[i] - cx, traj_y_[i] - cy) - fixed_R_));
      for (size_t j = i + 1; j < traj_x_.size(); ++j) {
        const double dx = traj_x_[i] - traj_x_[j];
        const double dy = traj_y_[i] - traj_y_[j];
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
    result.free_radius = std::sqrt(radius_sq);
    result.median_residual = residuals[median_index];
    result.p80_residual = residuals[p80_index];
    result.span = std::sqrt(max_span_sq);
    return std::isfinite(cx) && std::isfinite(cy) &&
      std::isfinite(result.free_radius) && std::isfinite(result.span);
  }

  double recomputeAccumulatedAngle(double cx, double cy, double &last_angle) const {
    if (traj_x_.empty() || traj_x_.size() != traj_y_.size()) {
      last_angle = 0;
      return 0;
    }

    constexpr double plate_spacing = 2.0 * M_PI / 3.0;
    double previous = std::atan2(traj_y_.front() - cy, traj_x_.front() - cx);
    double accumulated = 0;
    for (size_t i = 1; i < traj_x_.size(); ++i) {
      const double angle = std::atan2(traj_y_[i] - cy, traj_x_[i] - cx);
      double step = normalizeAngle(angle - previous);
      step -= std::round(step / plate_spacing) * plate_spacing;
      accumulated += step;
      previous = angle;
    }
    last_angle = previous;
    return accumulated;
  }

  double predictAngle(double t, int plate, double future_dt = 0.0) const {
    const double observation_dt = std::clamp(
      t - last_obs_time_, 0.0, max_phase_extrapolation_);
    const double plate_offset = (plate >= 0 && plate < 3) ? angle_offsets_[plate] : 0;
    const double observed_offset = (last_obs_plate_ >= 0 && last_obs_plate_ < 3) ?
      angle_offsets_[last_obs_plate_] : 0;
    return normalizeAngle(last_obs_angle_ +
      omega_ * (observation_dt + std::max(0.0, future_dt)) +
      plate_offset - observed_offset);
  }

  int findNearestPlate(double z) const {
    int best = 0;
    double best_distance = std::abs(z - z_centers_[0]);
    for (int i = 1; i < 3; ++i) {
      const double distance = std::abs(z - z_centers_[i]);
      if (distance < best_distance) {
        best = i;
        best_distance = distance;
      }
    }
    return best;
  }

  void updateZPlate(int plate, double z) {
    if (!z_initialized_ || plate < 0 || plate >= 3 || !std::isfinite(z)) return;
    if (!z_seen_[plate]) {
      z_centers_[plate] = z;
      z_seen_[plate] = true;
      ZFM_DEBUG(
        "armor_solver",
        "[Outpost][Z_SLOT] slot={} offset={:.1f}deg z={:.3f}",
        plate, angle_offsets_[plate] * 180.0 / M_PI, z);
      return;
    }
    z_centers_[plate] =
      (1.0 - z_learning_rate_) * z_centers_[plate] + z_learning_rate_ * z;
  }

  void switchToStatic(double x, double y, double z) {
    state_ = STATIC;
    static_x_ = x; static_y_ = y; static_z_ = z;
    outlier_start_t_ = 0;
  }

  OutpostState state_ = COLLECTING;
  double cx_ = 0, cy_ = 0;
  const double fixed_R_ = 0.275;
  double omega_ = 0;
  double last_obs_angle_ = 0, last_obs_time_ = 0;
  int last_obs_plate_ = 0;
  double static_x_ = 0, static_y_ = 0, static_z_ = 0;
  std::vector<double> traj_x_, traj_y_;
  double accum_dang_ = 0;
  double last_ang_ = 0, last_t_ = 0, last_z_ = 0;
  double z_centers_[3]{};
  double angle_offsets_[3]{};
  bool z_seen_[3]{};
  bool z_initialized_ = false;
  bool z_fully_calibrated_ = false;
  double z_learning_rate_ = 0.05;
  bool center_initialized_ = false;
  int collect_count_ = 0;
  double collect_start_t_ = 0;
  int current_plate_ = 0;
  double outlier_start_t_ = 0;
  double last_seen_t_ = 0;
  double phase_error_ = 0;

  static constexpr double outpost_omega_ = 2.513;
  static constexpr double min_collect_duration_ = 0.4;
  static constexpr double min_circle_fit_duration_ = 0.65;
  static constexpr double min_circle_fit_span_ = 0.30;
  static constexpr double max_circle_fit_median_residual_ = 0.08;
  static constexpr double max_circle_fit_p80_residual_ = 0.13;
  static constexpr double min_fitted_radius_ = 0.18;
  static constexpr double max_fitted_radius_ = 0.36;
  static constexpr size_t min_circle_fit_samples_ = 12;
  static constexpr size_t max_circle_fit_samples_ = 512;
  static constexpr double max_phase_extrapolation_ = 1.0;
  static constexpr double min_calibrated_z_gap_ = 0.04;
  static constexpr double max_outlier_duration_ = 0.15;
  static constexpr double residual_threshold_ = 0.2;
  static constexpr double max_association_error_ = 55.0 * M_PI / 180.0;
  static constexpr double max_phase_correction_ = 15.0 * M_PI / 180.0;
};

}  // namespace zfm::auto_aim

#endif  // ARMOR_SOLVER_MOTION_MODEL_HPP_
