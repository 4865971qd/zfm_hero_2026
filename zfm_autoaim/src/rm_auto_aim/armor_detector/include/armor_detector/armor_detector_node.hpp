#ifndef ARMOR_DETECTOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_DETECTOR_NODE_HPP_

// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "armor_detector/armor_detector.hpp"
#include "armor_detector/armor_pose_estimator.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_utils/heartbeat.hpp"

namespace zfm::auto_aim {

// Armor Detector Node
// Subscribe to the image topic, run the armor detection alogorithm and publish
// the detected armors
class ArmorDetectorNode : public rclcpp::Node {
public:
  ArmorDetectorNode(const rclcpp::NodeOptions &options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::unique_ptr<Detector> initDetector();

  std::vector<Armor>
  detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  void createDebugPublishers() noexcept;
  void destroyDebugPublishers() noexcept;

  void publishMarkers() noexcept;

  // Dynamic Parameter
  rcl_interfaces::msg::SetParametersResult
  onSetParameters(std::vector<rclcpp::Parameter> parameters);
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      on_set_parameters_callback_handle_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  // Armor Detector (traditional)
  std::unique_ptr<Detector> detector_;

  // Pose Solver
  bool use_ba_;
  std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

  // Detected armors publisher
  rm_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  // Image subscription
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // FPS tracking
  rclcpp::Time last_fps_time_{0, 0, RCL_ROS_TIME};
  int frame_count_{0};
  double fps_{0.0};

  // ReceiveData subscripiton
  std::string odom_frame_;
  Eigen::Matrix3d imu_to_camera_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> visualize_only_base_cb_handle_;
  rclcpp::Publisher<rm_interfaces::msg::DebugLights>::SharedPtr
      lights_data_pub_;
  rclcpp::Publisher<rm_interfaces::msg::DebugArmors>::SharedPtr
      armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;
};

} // namespace zfm::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_NODE_HPP_
