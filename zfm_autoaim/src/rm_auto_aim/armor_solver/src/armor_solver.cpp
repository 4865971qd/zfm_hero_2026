#include "armor_solver/armor_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace zfm::auto_aim {
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

Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  auto muzzle_xyz = node->declare_parameter(
      "solver.muzzle.xyz", std::vector<double>{0.095, 0.0, 0.0});
  if (muzzle_xyz.size() == 3) {
    muzzle_offset_ = Eigen::Vector3d(muzzle_xyz[0], muzzle_xyz[1], muzzle_xyz[2]);
  } else {
    ZFM_WARN("armor_solver", "solver.muzzle.xyz must contain exactly 3 values; use default");
  }
  outpost_yaw_offset_ = node->declare_parameter("solver.outpost_yaw_offset", 0.0);
  outpost_max_unseen_time_ = node->declare_parameter("solver.outpost_max_unseen_time", 0.15);
  outpost_max_phase_error_ =
      node->declare_parameter("solver.outpost_max_phase_error", 15.0) * M_PI / 180.0;
  normal_max_fire_unseen_time_ =
      node->declare_parameter("solver.normal_max_fire_unseen_time", 0.10);
  center_switch_confirm_time_ =
      node->declare_parameter("solver.center_switch_confirm_time", 0.05);
  side_angle_ = node->declare_parameter("solver.side_angle", 60.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "quadratic_drag");
  ZFM_INFO("armor_solver", "Compensator type: {}", compenstator_type);
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.81);

  if (compenstator_type == "quadratic_drag") {
    auto qd = dynamic_cast<QuadraticDragCompensator *>(trajectory_compensator_.get());
    if (qd) {
      qd->drag_coefficient = node->declare_parameter("solver.drag_coefficient", 0.50);
      qd->air_density = node->declare_parameter("solver.air_density", 1.225);
      qd->projectile_mass = node->declare_parameter("solver.projectile_mass", 0.0445);
      qd->projectile_diameter = node->declare_parameter("solver.projectile_diameter", 0.0425);
      ZFM_INFO("armor_solver", "Quadratic drag: Cd={:.3f}, rho={:.3f}, m={:.4f}kg, D={:.4f}m",
               qd->drag_coefficient, qd->air_density, qd->projectile_mass, qd->projectile_diameter);
    }
  } else {
    trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.014);
  }

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if (!manual_compensator_->updateMapFlow(angle_offset)) {
    ZFM_WARN("armor_solver", "Manual compensator update failed!");
  }

  state = State::TRACKING_ARMOR;
  node.reset();
}

// Solve gimbal yaw/pitch command from tracked target.
rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                             const rclcpp::Time &current_time,
                                             std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  aim_visualization_ = AimVisualization{};
  const bool finite_target =
    std::isfinite(target.position.x) && std::isfinite(target.position.y) &&
    std::isfinite(target.position.z) && std::isfinite(target.velocity.x) &&
    std::isfinite(target.velocity.y) && std::isfinite(target.velocity.z) &&
    std::isfinite(target.yaw) && std::isfinite(target.v_yaw) &&
    std::isfinite(target.radius_1) && std::isfinite(target.radius_2) &&
    std::isfinite(target.d_zc) && std::isfinite(target.d_za);
  if (!finite_target) {
    throw std::runtime_error("Non-finite target state");
  }

  // Parameters are cached in constructor via declare_parameter;
  // they do not change at runtime, so no need for per-frame get_parameter().
  tf2::Quaternion tf_q;
  Eigen::Vector3d gimbal_origin(0, 0, 0);
  Eigen::Matrix3d R_gimbal_eigen = Eigen::Matrix3d::Identity();
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];

    gimbal_origin = Eigen::Vector3d(
        gimbal_tf.transform.translation.x,
        gimbal_tf.transform.translation.y,
        gimbal_tf.transform.translation.z);
    tf2::Matrix3x3 R_gimbal(tf_q);
    R_gimbal_eigen << R_gimbal.getRow(0).x(), R_gimbal.getRow(0).y(), R_gimbal.getRow(0).z(),
                      R_gimbal.getRow(1).x(), R_gimbal.getRow(1).y(), R_gimbal.getRow(1).z(),
                      R_gimbal.getRow(2).x(), R_gimbal.getRow(2).y(), R_gimbal.getRow(2).z();

    if (debug_) {
      static int tf_log_cnt = 0;
      if (++tf_log_cnt % 30 == 0) {
        ZFM_DEBUG("armor_solver", "[Solver] TF rpy=({:.1f}°,{:.1f}°,{:.1f}°)",
                  rpy_[0]*180/M_PI, rpy_[1]*180/M_PI, rpy_[2]*180/M_PI);
      }
    }
  } catch (tf2::TransformException &ex) {
    ZFM_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  const Eigen::Vector3d observed_target_position = target_position;
  const double observed_target_yaw = target_yaw;
  const double observation_delay = (current_time - rclcpp::Time(target.header.stamp)).seconds();

  Eigen::Vector3d muzzle_odom = gimbal_origin + R_gimbal_eigen * muzzle_offset_;
  Eigen::Vector3d target_from_muzzle = target_position - muzzle_odom;

  double flying_time = trajectory_compensator_->getFlyingTime(target_from_muzzle);
  double dt = observation_delay + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;

  Eigen::Vector3d target_from_muzzle_predicted = target_position - muzzle_odom;

  if (target.armors_num <= 0) {
    throw std::runtime_error("Invalid armors_num");
  }
  const bool is_outpost = (target.armors_num == 3);
  const bool outpost_single_plate = is_outpost && !target.outpost_z_calibrated;
  const double *outpost_offsets = is_outpost ? target.outpost_angle_offsets.data() : nullptr;
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
    current_time.seconds(), target.normal_last_seen_time, normal_max_fire_unseen_time_);
  if (is_outpost) {
    double unseen_time = current_time.seconds() - target.outpost_last_seen_time;
    outpost_observation_fresh = target.outpost_last_seen_time > 0.0 &&
                                unseen_time >= -0.02 &&
                                unseen_time <= outpost_max_unseen_time_;
    outpost_phase_valid = std::isfinite(target.outpost_phase_error) &&
                          std::abs(target.outpost_phase_error) <=
                              outpost_max_phase_error_;
    outpost_prediction_ok = outpost_single_plate || target.outpost_z_calibrated;
    outpost_model_ok = outpost_observation_fresh &&
                       (outpost_single_plate ||
                        (target.outpost_z_calibrated && outpost_phase_valid));
  }
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
      target_from_muzzle_predicted, target_yaw, target.radius_1, target.radius_2,
      target.d_zc, target.d_za, target.armors_num, outpost_offsets);
  int idx = outpost_single_plate ?
      std::clamp(target.outpost_observed_plate, 0, std::max(0, target.armors_num - 1)) :
      selectBestArmor(armor_positions, target_from_muzzle_predicted, target_yaw,
                      target.v_yaw, target.armors_num, flying_time, outpost_offsets);
  auto chosen_armor_position = armor_positions.at(idx);
  if (chosen_armor_position.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  if (target.armors_num == 3 && !outpost_single_plate) {
    double refined_flying_time = trajectory_compensator_->getFlyingTime(chosen_armor_position);
    double refined_dt = observation_delay + refined_flying_time + prediction_delay_;
    if (std::abs(refined_dt - dt) > 0.003) {
      double refined_yaw = observed_target_yaw + refined_dt * target.v_yaw;
      armor_positions = getArmorPositions(
          target_from_muzzle_predicted, refined_yaw, target.radius_1, target.radius_2,
          target.d_zc, target.d_za, target.armors_num, outpost_offsets);
      idx = selectBestArmor(armor_positions, target_from_muzzle_predicted, refined_yaw,
                            target.v_yaw, target.armors_num, refined_flying_time,
                            outpost_offsets);
      chosen_armor_position = armor_positions.at(idx);
      if (chosen_armor_position.norm() < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
      }
      target_yaw = refined_yaw;
      flying_time = refined_flying_time;
      dt = refined_dt;
    }
  } else {
    double refined_flying_time = trajectory_compensator_->getFlyingTime(chosen_armor_position);
    double refined_dt = observation_delay + refined_flying_time + prediction_delay_;
    if (std::abs(refined_dt - dt) > 0.003) {
      Eigen::Vector3d refined_target_position = observed_target_position;
      refined_target_position.x() += refined_dt * target.velocity.x;
      refined_target_position.y() += refined_dt * target.velocity.y;
      refined_target_position.z() += refined_dt * target.velocity.z;
      double refined_yaw = observed_target_yaw + refined_dt * target.v_yaw;
      Eigen::Vector3d refined_target_from_muzzle = refined_target_position - muzzle_odom;

      armor_positions = getArmorPositions(
          refined_target_from_muzzle, refined_yaw, target.radius_1, target.radius_2,
          target.d_zc, target.d_za, target.armors_num, outpost_offsets);
      idx = selectBestArmor(
          armor_positions, refined_target_from_muzzle, refined_yaw,
          target.v_yaw, target.armors_num, refined_flying_time, outpost_offsets);
      chosen_armor_position = armor_positions.at(idx);
      if (chosen_armor_position.norm() < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
      }

      target_position = refined_target_position;
      target_yaw = refined_yaw;
      target_from_muzzle_predicted = refined_target_from_muzzle;
      flying_time = refined_flying_time;
      dt = refined_dt;
    }
  }

  Eigen::Vector3d selected_visualization_position = chosen_armor_position;
  double selected_visualization_yaw = target_yaw;
  int selected_visualization_plate = idx;
  bool selected_visualization_valid = true;

  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);

  double distance = chosen_armor_position.norm();

  double outpost_cam_dir = 0;
  double outpost_sel_diff_deg = 99;
  double aim_z = 0;
  double default_offsets[3] = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0};
  const double *offsets = outpost_offsets ? outpost_offsets : default_offsets;
  if (target.armors_num == 3 && !outpost_single_plate && std::abs(target.v_yaw) > 0.01) {
    const bool has_current_observation =
        target.outpost_has_observed_z && outpost_observation_fresh;
    outpost_cam_dir = std::atan2(-target_from_muzzle_predicted.y(),
                                 -target_from_muzzle_predicted.x());
    double arrival_cam_dir = outpost_cam_dir;

    // Compute each plate's angular difference vs arrival window center.
    double diffs[3];
    bool approaching[3];
    for (int i = 0; i < 3; i++) {
      diffs[i] =
          angles::normalize_angle(target_yaw + offsets[i] - arrival_cam_dir);
      approaching[i] = (target.v_yaw * diffs[i] < 0);
    }

    // Pre-aim switching: when the current plate leaves the ±30° fire
    // window, switch to the next approaching plate closest to the window.
    constexpr double kSwitchMargin = 22.0 * M_PI / 180.0;
    constexpr double kHoldWindow = 55.0 * M_PI / 180.0;
    constexpr double kForceSwitch = 88.0 * M_PI / 180.0;
    constexpr double kConfirmTime = 0.090;
    constexpr double kMinSwitchInterval = 0.160;
    const double now = current_time.seconds();

    int current_plate = target.outpost_observed_plate;
    if (current_plate < 0 || current_plate >= 3) {
      current_plate = (aim_idx_ >= 0 && aim_idx_ < 3) ? aim_idx_ : idx;
    }
    current_plate = std::clamp(current_plate, 0, 2);

    auto positiveAngle = [](double angle) {
      double wrapped = std::fmod(angle, 2.0 * M_PI);
      if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
      }
      return wrapped;
    };
    int next_plate = current_plate;
    double next_step = 1e9;
    for (int i = 0; i < 3; ++i) {
      if (i == current_plate) continue;
      const double step = target.v_yaw < 0.0 ?
        positiveAngle(offsets[current_plate] - offsets[i]) :
        positiveAngle(offsets[i] - offsets[current_plate]);
      if (step > 1e-6 && step < next_step) {
        next_step = step;
        next_plate = i;
      }
    }

    if (aim_idx_ >= 0 && aim_idx_ < 3 &&
        aim_idx_ != current_plate && aim_idx_ != next_plate) {
      aim_idx_ = current_plate;
      outpost_pending_idx_ = -1;
      outpost_pending_since_ = -1.0;
    }

    const double current_abs_for_candidate = std::abs(diffs[current_plate]);
    const double next_abs = std::abs(diffs[next_plate]);
    const bool current_unusable_for_candidate =
        !approaching[current_plate] || current_abs_for_candidate > kHoldWindow;
    int candidate = current_plate;
    if (current_unusable_for_candidate &&
        (approaching[next_plate] ||
         next_abs + kSwitchMargin < current_abs_for_candidate ||
         current_abs_for_candidate > kForceSwitch)) {
      candidate = next_plate;
    }
    const double candidate_abs = std::abs(diffs[candidate]);

    if (has_current_observation) {
      if (aim_idx_ != current_plate) {
        outpost_last_switch_time_ = now;
      }
      aim_idx_ = current_plate;
      outpost_pending_idx_ = -1;
      outpost_pending_since_ = -1.0;
    } else if (aim_idx_ < 0 || aim_idx_ >= 3) {
      aim_idx_ = candidate;
      outpost_pending_idx_ = -1;
      outpost_pending_since_ = -1.0;
      outpost_last_switch_time_ = now;
    } else if (candidate != aim_idx_) {
      const double current_abs = std::abs(diffs[aim_idx_]);
      const bool candidate_much_better = candidate_abs + kSwitchMargin < current_abs;
      const bool current_unusable = !approaching[aim_idx_] || current_abs > kHoldWindow;
      const bool force_switch = current_abs > kForceSwitch;
      if (current_unusable && (candidate_much_better || force_switch)) {
        if (outpost_pending_idx_ != candidate) {
          outpost_pending_idx_ = candidate;
          outpost_pending_since_ = now;
        }
        if (!outpost_phase_valid) {
          outpost_pending_since_ = now;
        } else {
          const double pending = now - outpost_pending_since_;
          const double since_switch = outpost_last_switch_time_ > 0 ?
            now - outpost_last_switch_time_ : 1e9;
          if (force_switch ||
              (pending >= kConfirmTime && since_switch >= kMinSwitchInterval)) {
            aim_idx_ = candidate;
            outpost_pending_idx_ = -1;
            outpost_pending_since_ = -1.0;
            outpost_last_switch_time_ = now;
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

    // Lead-angle aim point.
    double z_offsets[3] = {0.0, target.d_zc, target.d_za};
    aim_z = z_offsets[aim_idx_];
    const bool use_observed_z =
        aim_idx_ == target.outpost_observed_plate &&
        target.outpost_has_observed_z &&
        outpost_observation_fresh &&
        std::isfinite(target.outpost_observed_z);
    if (use_observed_z) {
      aim_z = target.outpost_observed_z - target_position.z();
      smooth_aim_z_[aim_idx_] = aim_z;
      smooth_aim_z_initialized_[aim_idx_] = true;
    } else if (!smooth_aim_z_initialized_[aim_idx_]) {
      smooth_aim_z_[aim_idx_] = aim_z;
      smooth_aim_z_initialized_[aim_idx_] = true;
    } else {
      smooth_aim_z_[aim_idx_] += 0.3 * (aim_z - smooth_aim_z_[aim_idx_]);
    }
    aim_z = smooth_aim_z_[aim_idx_];

    double plate_angle = target_yaw + offsets[aim_idx_];
    double yaw_offset_angle =
        std::copysign(outpost_yaw_offset_ * M_PI / 180.0, target.v_yaw);
    double aim_angle = plate_angle - yaw_offset_angle;

    Eigen::Vector3d predicted_hit_point(
        target_from_muzzle_predicted.x() +
            target.radius_1 * std::cos(plate_angle),
        target_from_muzzle_predicted.y() +
            target.radius_1 * std::sin(plate_angle),
        target_from_muzzle_predicted.z() + aim_z);
    Eigen::Vector3d aim_point(
        target_from_muzzle_predicted.x() +
            target.radius_1 * std::cos(aim_angle),
        target_from_muzzle_predicted.y() +
            target.radius_1 * std::sin(aim_angle),
        target_from_muzzle_predicted.z() + aim_z);
    selected_visualization_position = predicted_hit_point;
    selected_visualization_yaw = plate_angle;
    selected_visualization_plate = aim_idx_;
    calcYawAndPitch(aim_point, rpy_, yaw, pitch);
    distance = aim_point.norm();

    // Window-centric diff for fire decision and logging (degrees).
    outpost_sel_diff_deg = diffs[aim_idx_] * 180.0 / M_PI;

    if (debug_) {
      static int aim_log_cnt = 0;
      if (++aim_log_cnt % 30 == 0) {
        ZFM_DEBUG("armor_solver",
          "[Outpost] AIM plate={} z={:.3f} "
          "arrival_cam={:.1f}° d2arrival=[{:.1f}° {:.1f}° {:.1f}°] "
          "lead={:.1f}° yaw={:.1f}° pit={:.1f}°",
          aim_idx_, aim_z, arrival_cam_dir * 180.0 / M_PI,
          diffs[0] * 180.0 / M_PI, diffs[1] * 180.0 / M_PI,
          diffs[2] * 180.0 / M_PI, -yaw_offset_angle * 180.0 / M_PI,
          yaw * 180.0 / M_PI, pitch * 180.0 / M_PI);
      }
      static int status_cnt = 0;
      if (++status_cnt % 30 == 0) {
        ZFM_DEBUG("armor_solver",
          "[Outpost] STATUS cx=({:.2f},{:.2f}) ω={:.2f} t_yaw={:.1f}° "
          "zc=[{:.3f},{:.3f}] fly={:.3f}s",
          target_position.x(), target_position.y(), target.v_yaw,
          target.yaw * 180.0 / M_PI,
          target_position.z() + target.d_zc,
          target_position.z() + target.d_za, flying_time);
      }
    }
  }

  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  if (target.armors_num == 3 && !outpost_single_plate && std::abs(target.v_yaw) > 0.01) {
    // Fire: plate is within ±30° of arrival window AND approaching.
    bool in_fire_window =
        std::abs(outpost_sel_diff_deg) < 30.0 &&
        (target.v_yaw * outpost_sel_diff_deg < 0);
    gimbal_cmd.fire_advice =
        outpost_model_ok && in_fire_window && isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
    if (debug_) {
      ZFM_DEBUG("armor_solver", "[Outpost] FIRE={} diff={:.1f}° app={}",
                gimbal_cmd.fire_advice ? "是" : "否", outpost_sel_diff_deg,
                (target.v_yaw * outpost_sel_diff_deg < 0) ? "是" : "否");
    }
  } else if (is_outpost && !outpost_single_plate) {
    gimbal_cmd.fire_advice =
        outpost_model_ok && isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
  } else if (!outpost_single_plate) {
    gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
  } else {
    gimbal_cmd.fire_advice = false;
  }

  switch (state) {
    case TRACKING_ARMOR: {
      center_exit_since_ = -1.0;
      if (heldFor(
            std::abs(target.v_yaw) > max_tracking_v_yaw_,
            current_time.seconds(), center_switch_confirm_time_, center_enter_since_)) {
        state = TRACKING_CENTER;
        center_enter_since_ = -1.0;
      }
      // controller_delay_ not applicable to outpost — the unified
      // outpost block above already computed lead-angle yaw/pitch.
      if (controller_delay_ != 0 && target.armors_num != 3) {
        Eigen::Vector3d delayed_target_from_muzzle = target_from_muzzle_predicted;
        delayed_target_from_muzzle.x() += controller_delay_ * target.velocity.x;
        delayed_target_from_muzzle.y() += controller_delay_ * target.velocity.y;
        delayed_target_from_muzzle.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(delayed_target_from_muzzle, target_yaw,
                                            target.radius_1, target.radius_2,
                                            target.d_zc, target.d_za, target.armors_num,
                                            outpost_offsets);
        idx = selectBestArmor(
          armor_positions, delayed_target_from_muzzle, target_yaw, target.v_yaw,
          target.armors_num, flying_time, outpost_offsets);
        chosen_armor_position = armor_positions.at(idx);
        distance = chosen_armor_position.norm();
        gimbal_cmd.distance = distance;
        if (distance < 0.1)
          throw std::runtime_error("No valid armor to shoot");
        selected_visualization_position = chosen_armor_position;
        selected_visualization_yaw = target_yaw;
        selected_visualization_plate = idx;
        calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
      }
      break;
    }
    case TRACKING_CENTER: {
      center_enter_since_ = -1.0;
      if (heldFor(
            std::abs(target.v_yaw) < max_tracking_v_yaw_,
            current_time.seconds(), center_switch_confirm_time_, center_exit_since_)) {
        state = TRACKING_ARMOR;
        center_exit_since_ = -1.0;
      }
      calcYawAndPitch(target_from_muzzle_predicted, rpy_, yaw, pitch);
      distance = target_from_muzzle_predicted.norm();
      gimbal_cmd.distance = distance;
      gimbal_cmd.fire_advice = false;
      selected_visualization_valid = false;
      break;
    }
  }

  if (!is_outpost) {
    gimbal_cmd.fire_advice = state != TRACKING_CENTER && normal_model_ok &&
      isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
  }

  auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;
  gimbal_cmd.yaw_diff = angles::normalize_angle(cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (!std::isfinite(gimbal_cmd.yaw) || !std::isfinite(gimbal_cmd.pitch) ||
      !std::isfinite(gimbal_cmd.yaw_diff) || !std::isfinite(gimbal_cmd.pitch_diff) ||
      !std::isfinite(gimbal_cmd.distance)) {
    throw std::runtime_error("Non-finite gimbal command");
  }

  const bool selected_prediction_valid =
      selected_visualization_valid && (!is_outpost || outpost_prediction_ok);
  if (selected_prediction_valid && selected_visualization_position.allFinite() &&
      std::isfinite(selected_visualization_yaw)) {
    aim_visualization_.valid = true;
    aim_visualization_.is_outpost = is_outpost;
    aim_visualization_.selected_plate = selected_visualization_plate;
    aim_visualization_.point = muzzle_odom + selected_visualization_position;
    aim_visualization_.yaw = selected_visualization_yaw;
    aim_visualization_.prediction_horizon = std::max(0.0, dt);
  }

  if (debug_) {
    static int cmd_log_cnt = 0;
    ++cmd_log_cnt;
    if (target.armors_num == 3 && (cmd_log_cnt <= 60 || cmd_log_cnt % 10 == 0)) {
      ZFM_DEBUG("armor_solver",
        "[Outpost] CMD plate={} yaw={:.1f}° p={:.1f}° diff=({:.1f}°,{:.1f}°) "
        "fire={} dist={:.1f}m fly={:.3f}s lead={:.1f}°",
        aim_idx_, gimbal_cmd.yaw, gimbal_cmd.pitch,
        gimbal_cmd.yaw_diff, gimbal_cmd.pitch_diff,
        gimbal_cmd.fire_advice?"是":"否", gimbal_cmd.distance,
        flying_time,
        (flying_time + prediction_delay_) * target.v_yaw * 180 / M_PI);
    } else if (target.armors_num != 3 && cmd_log_cnt % 30 == 0) {
      ZFM_DEBUG("armor_solver",
        "[Solver] CMD pitch={:.1f}° diff={:.1f}° dist={:.1f}m v0={:.1f}",
        gimbal_cmd.pitch, gimbal_cmd.pitch_diff, gimbal_cmd.distance,
        trajectory_compensator_->velocity);
    }
  }

  return gimbal_cmd;
}

// Check if gimbal is within firing tolerance.
bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(angles::normalize_angle(cur_yaw - target_yaw)) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

// Compute armor plate positions relative to target center.
std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num,
                                                       const double *angle_offsets) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  if (armors_num == 3) {
    double cx = target_center.x();
    double cy = target_center.y();
    double default_offsets[3] = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0};
    const double *offsets = angle_offsets ? angle_offsets : default_offsets;
    double z_offsets[3] = {0.0, d_zc, d_za};
    for (size_t i = 0; i < 3; i++) {
      double ang = target_yaw + offsets[i];
      armor_positions[i] = Eigen::Vector3d(
          cx + r1 * cos(ang),
          cy + r1 * sin(ang),
          target_center.z() + z_offsets[i]
      );
    }
    return armor_positions;
  }
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    r = is_current_pair ? r1 : r2;
    target_dz = d_zc + (is_current_pair ? 0 : d_za);
    is_current_pair = !is_current_pair;
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

// Select the best armor plate to aim at.
int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num,
                            double flying_time,
                            const double *angle_offsets) const noexcept {
  (void)armor_positions;
  double alpha = std::atan2(target_center.y(), target_center.x());
  double beta = target_yaw;

  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha),
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta),
                 -std::sin(beta), std::cos(beta);
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  const double sin_decision = std::clamp(R_center2armor(0, 1), -1.0, 1.0);
  double decision_angle = -std::asin(sin_decision);

  double theta = 0;
  if (std::abs(target_v_yaw) >= min_switching_v_yaw_ && flying_time > 0.0) {
    double theta_dynamic = std::abs(target_v_yaw) * flying_time;
    double theta_max = side_angle_ * M_PI / 180.0;
    double theta_magnitude = std::min(theta_dynamic, theta_max);
    theta = (target_v_yaw > 0) ? theta_magnitude : -theta_magnitude;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  if (selected_id >= static_cast<int>(armors_num)) selected_id = static_cast<int>(armors_num) - 1;

  if (armors_num == 3 && std::abs(target_v_yaw) < 0.01) {
    return selected_id;
  }
  if (armors_num == 3) {
    // Initial plate pick — the unified outpost block in solve() handles
    // pre-aim switching on subsequent frames.
    double cam_dir = std::atan2(-target_center.y(), -target_center.x());
    double default_offsets[3] = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0};
    const double *offsets = angle_offsets ? angle_offsets : default_offsets;

    double diffs[3];
    bool approaching[3];
    for (int i = 0; i < 3; i++) {
      diffs[i] = angles::normalize_angle(target_yaw + offsets[i] - cam_dir);
      approaching[i] = (target_v_yaw * diffs[i] < 0);
    }

    // Prefer approaching plates; pick the one with smallest |diff|.
    int best = -1;
    double best_abs = 1e9;
    for (int i = 0; i < 3; i++) {
      if (approaching[i] && std::abs(diffs[i]) < best_abs) {
        best_abs = std::abs(diffs[i]);
        best = i;
      }
    }
    if (best < 0) {
      best = 0;
      for (int i = 1; i < 3; i++) {
        if (std::abs(diffs[i]) < std::abs(diffs[best])) best = i;
      }
    }

    if (debug_) {
      static int solver_log_cnt = 0;
      if (++solver_log_cnt % 30 == 0) {
        ZFM_DEBUG("armor_solver",
          "[Solver] selectBest yaw={:.1f}° diff=[{:.1f}° {:.1f}° {:.1f}°] best={}",
          target_yaw*180/M_PI,
          diffs[0]*180/M_PI, diffs[1]*180/M_PI, diffs[2]*180/M_PI, best);
      }
    }
    return best;
  }

  return selected_id;
}

// Calculate yaw and pitch angles to target position.
void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

// Get bullet trajectory points for visualization.
std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace zfm::auto_aim
