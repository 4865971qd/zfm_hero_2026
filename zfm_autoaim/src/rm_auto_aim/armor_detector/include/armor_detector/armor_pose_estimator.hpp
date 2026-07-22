#ifndef ARMOR_DETECTOR_ARMOR_POSE_ESTIMATOR_HPP_
#define ARMOR_DETECTOR_ARMOR_POSE_ESTIMATOR_HPP_

// std
#include <array>
#include <memory>
#include <vector>
// OpenCV
#include <opencv2/opencv.hpp>
// Eigen
#include <Eigen/Dense>
// ros2
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
// project
#include "armor_detector/ba_solver.hpp"
#include "rm_interfaces/msg/armor.hpp"
#include "rm_utils/math/pnp_solver.hpp"

namespace zfm::auto_aim {
class ArmorPoseEstimator {
public:
  explicit ArmorPoseEstimator(sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

  std::vector<rm_interfaces::msg::Armor> extractArmorPoses(const std::vector<Armor> &armors,
                                               Eigen::Matrix3d R_imu_camera);

  void enableBA(bool enable) { use_ba_ = enable; }

private:
  // Select the best PnP solution according to the armor's direction in image, only available for SOLVEPNP_IPPE
  void sortPnPResult(const Armor &armor, std::vector<cv::Mat> &rvecs,
                     std::vector<cv::Mat> &tvecs) const;

  // Convert a rotation matrix to RPY
  static Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d &R);

  bool use_ba_;

  Eigen::Matrix3d R_gimbal_camera_;

  std::unique_ptr<BaSolver> ba_solver_;
  std::unique_ptr<PnPSolver> pnp_solver_;
};
} // namespace zfm::auto_aim
#endif // ARMOR_POSE_ESTIMATOR_HPP_