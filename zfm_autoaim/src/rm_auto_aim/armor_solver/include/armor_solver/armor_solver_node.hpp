#ifndef ARMOR_SOLVER_SOLVER_NODE_HPP_
#define ARMOR_SOLVER_SOLVER_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

#include "armor_solver/armor_solver.hpp"
#include "armor_solver/armor_tracker.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/measurement.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/plotter.hpp"

namespace zfm::auto_aim {

using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;

class ArmorSolverNode : public rclcpp::Node {
public:
  explicit ArmorSolverNode(const rclcpp::NodeOptions &options);

private:
  // Process incoming armor detections: transform, filter, track, solve.
  void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_ptr);

  // Initialize RViz visualization markers.
  void initMarkers() noexcept;

  // Publish visualization markers for target state and trajectory.
  void publishMarkers(const rm_interfaces::msg::Target &target_msg,
                      const rm_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept;

  // Handle vision mode switch service request.
  void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                       std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  // Receive bullet speed flag from serial.
  void serialCallback(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg);

  // Publish gimbal command at fixed interval.
  void timerCallback();

  bool debug_mode_;

  HeartBeatPublisher::SharedPtr heartbeat_;

  rclcpp::Time last_time_;
  double dt_;

  double s2qx_, s2qy_, s2qz_, s2qyaw_, s2qr_, s2qd_zc_;
  double r_x_, r_y_, r_z_, r_yaw_;
  double lost_time_thres_;
  double tracking_confirm_time_;
  double normal_max_aim_unseen_time_;
  std::unique_ptr<Tracker> tracker_;

  std::unique_ptr<Solver> solver_;

  rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_sub_;
  uint8_t mode_flag_ = 0;
  double bullet_speed_;

  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
  rm_interfaces::msg::Target armor_target_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  rclcpp::Publisher<rm_interfaces::msg::Measurement>::SharedPtr measure_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  bool enable_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  double pitch_dead_zone_;
  double yaw_dead_zone_;
  double last_sent_pitch_ = 0;
  double last_sent_yaw_ = 0;
  bool dead_zone_initialized_ = false;

  bool stationary_mode_ = false;
  double stationary_confirm_time_;
  double stationary_release_time_;
  double stationary_vel_threshold_;
  double stationary_release_vel_threshold_;
  double stationary_since_ = -1.0;
  double moving_since_ = -1.0;

  // PlotJuggler debug visualization
  std::unique_ptr<zfm::Plotter> plotter_;
  double last_serial_yaw_ = 0.0;
  double last_serial_pitch_ = 0.0;

  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker selection_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace zfm::auto_aim

#endif  // ARMOR_SOLVER_SOLVER_NODE_HPP_
