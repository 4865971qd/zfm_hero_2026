#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

#include <array>
#include <memory>
#include <utility>
#include <vector>

#include <tf2_ros/buffer.h>
#include "/opt/ros/humble/include/angles/angles/angles.h"

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"

namespace zfm::auto_aim {

class Solver {
public:
  struct AimVisualization {
    bool valid = false;
    bool is_outpost = false;
    int selected_plate = -1;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double prediction_horizon = 0.0;
  };

  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  ~Solver() = default;

  // Solve gimbal yaw/pitch command from tracked target.
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state = TRACKING_ARMOR;

  void setDebug(bool debug) { debug_ = debug; }
  const AimVisualization &aimVisualization() const noexcept { return aim_visualization_; }

  // Get bullet trajectory points for visualization.
  std::vector<std::pair<double, double>> getTrajectory() const noexcept;

  void setBulletSpeed(double speed) noexcept { trajectory_compensator_->velocity = speed; }
  void resetTrackingState() noexcept {
    state = TRACKING_ARMOR;
    center_enter_since_ = -1.0;
    center_exit_since_ = -1.0;
    aim_idx_ = -1;
    for (int i = 0; i < 3; ++i) {
      smooth_aim_z_[i] = 0.0;
      smooth_aim_z_initialized_[i] = false;
    }
    outpost_pending_idx_ = -1;
    outpost_pending_since_ = -1.0;
    outpost_last_switch_time_ = -1.0;
    aim_visualization_ = AimVisualization{};
  }

private:
  // Compute armor plate positions relative to target center.
  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 const double yaw,
                                                 const double r1,
                                                 const double r2,
                                                 const double d_zc,
                                                 const double d_za,
                                                 const size_t armors_num,
                                                 const double *angle_offsets = nullptr) const noexcept;

  // Select the best armor plate to aim at.
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      const double target_yaw,
                      const double target_v_yaw,
                      const size_t armors_num,
                      double flying_time = 0.0,
                      const double *angle_offsets = nullptr) const noexcept;

  // Calculate yaw and pitch angles to target position.
  void calcYawAndPitch(const Eigen::Vector3d &p,
                       const std::array<double, 3> rpy,
                       double &yaw,
                       double &pitch) const noexcept;

  // Check if gimbal is within firing tolerance.
  bool isOnTarget(const double cur_yaw,
                  const double cur_pitch,
                  const double target_yaw,
                  const double target_pitch,
                  const double distance) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;

  std::array<double, 3> rpy_;

  double prediction_delay_;
  double controller_delay_;
  Eigen::Vector3d muzzle_offset_{0.095, 0.0, 0.0};
  double outpost_yaw_offset_;
  double outpost_max_unseen_time_;
  double outpost_max_phase_error_;
  double normal_max_fire_unseen_time_;

  double shooting_range_w_;
  double shooting_range_h_;

  double max_tracking_v_yaw_;
  double center_switch_confirm_time_;
  double center_enter_since_ = -1.0;
  double center_exit_since_ = -1.0;

  double side_angle_;
  double min_switching_v_yaw_;

  mutable int aim_idx_ = -1;
  mutable double smooth_aim_z_[3] = {0.0, 0.0, 0.0};
  mutable bool smooth_aim_z_initialized_[3] = {false, false, false};
  mutable int outpost_pending_idx_ = -1;
  mutable double outpost_pending_since_ = -1.0;
  mutable double outpost_last_switch_time_ = -1.0;

  bool debug_ = false;
  AimVisualization aim_visualization_;

  std::weak_ptr<rclcpp::Node> node_;
};

}  // namespace zfm::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_
