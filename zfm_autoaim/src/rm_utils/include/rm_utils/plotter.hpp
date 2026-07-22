#ifndef RM_UTILS__PLOTTER_HPP
#define RM_UTILS__PLOTTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <vector>

namespace zfm {

/// Lightweight debug data publisher for PlotJuggler visualization.
///
/// Publishes four Float64MultiArray topics at different rates so
/// PlotJuggler can subscribe and render time-series curves.
///
/// Topic layout (each array element maps to one curve):
///   /debug/gimbal_cmd:   [cmd_yaw(deg), cmd_pitch(deg), yaw_diff(deg), pitch_diff(deg), distance(m), fire_advice(0|1)]
///   /debug/gimbal_state: [actual_yaw(deg), actual_pitch(deg), target_yaw(rad), target_v_yaw(rad/s), dt(s)]
///   /debug/target:       [ekf_xc, ekf_yc, ekf_zc, ekf_vx, ekf_vy, ekf_vz, ekf_yaw, ekf_vyaw, ekf_r, tracker_state(int)]
///   /debug/ekf_residual: [meas_x, meas_y, meas_z, meas_yaw, ekf_xc, ekf_yc, ekf_zc, ekf_yaw]
class Plotter {
public:
  explicit Plotter(rclcpp::Node &node)
  : gimbal_cmd_pub_(node.create_publisher<std_msgs::msg::Float64MultiArray>("/debug/gimbal_cmd", 10)),
    gimbal_state_pub_(node.create_publisher<std_msgs::msg::Float64MultiArray>("/debug/gimbal_state", 10)),
    target_pub_(node.create_publisher<std_msgs::msg::Float64MultiArray>("/debug/target", 10)),
    ekf_residual_pub_(node.create_publisher<std_msgs::msg::Float64MultiArray>("/debug/ekf_residual", 10))
  {}

  /// Publish gimbal command (call from timerCallback, ~250 Hz).
  void publishGimbalCmd(double cmd_yaw_deg, double cmd_pitch_deg,
                        double yaw_diff_deg, double pitch_diff_deg,
                        double distance, bool fire_advice)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {cmd_yaw_deg, cmd_pitch_deg, yaw_diff_deg, pitch_diff_deg,
                distance, fire_advice ? 1.0 : 0.0};
    gimbal_cmd_pub_->publish(msg);
  }

  /// Publish gimbal actual state vs target (call from timerCallback, ~250 Hz).
  void publishGimbalState(double actual_yaw_deg, double actual_pitch_deg,
                          double target_yaw_rad, double target_v_yaw_rad_s,
                          double dt_s)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {actual_yaw_deg, actual_pitch_deg, target_yaw_rad,
                target_v_yaw_rad_s, dt_s};
    gimbal_state_pub_->publish(msg);
  }

  /// Publish EKF target state (call from armorsCallback, ~60 Hz).
  void publishTarget(const std::vector<double> &ekf_state, int tracker_state)
  {
    // ekf_state expected: [xc, vx, yc, vy, zc, vz, yaw, vyaw, r, d_zc]
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = ekf_state;
    msg.data.push_back(static_cast<double>(tracker_state));
    target_pub_->publish(msg);
  }

  /// Publish EKF measurement vs prediction residual (call from armorsCallback, ~60 Hz).
  void publishEkfResidual(double meas_x, double meas_y, double meas_z, double meas_yaw,
                          double ekf_xc, double ekf_yc, double ekf_zc, double ekf_yaw)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {meas_x, meas_y, meas_z, meas_yaw, ekf_xc, ekf_yc, ekf_zc, ekf_yaw};
    ekf_residual_pub_->publish(msg);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ekf_residual_pub_;
};

}  // namespace zfm

#endif  // RM_UTILS__PLOTTER_HPP
