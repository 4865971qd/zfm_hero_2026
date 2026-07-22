#include "armor_detector/armor_pose_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "armor_detector/types.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace zfm::auto_aim {
ArmorPoseEstimator::ArmorPoseEstimator(
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
  // 设置 PnP 解算器：用于从2D图像点和装甲板的3D模型点，估计装甲板的位姿
  pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
  // 注册小装甲板的物体坐标（单位：米），用于PnP中作为3D基准
  pnp_solver_->setObjectPoints(
      "small", Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH,
                                                     SMALL_ARMOR_HEIGHT));
  // 注册大装甲板的物体坐标
  pnp_solver_->setObjectPoints(
      "large", Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH,
                                                     LARGE_ARMOR_HEIGHT));
  // BA求解器：在PnP初值的基础上进一步优化位姿，提高稳定性与精度
  ba_solver_ = std::make_unique<BaSolver>(camera_info->k, camera_info->d);

  R_gimbal_camera_ = Eigen::Matrix3d::Identity();
  // 相机坐标 -> 云台坐标 的坐标轴重映射矩阵
  // 这里的矩阵表示：x_cam -> z_gimbal, y_cam -> -x_gimbal, z_cam -> -y_gimbal
  R_gimbal_camera_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
}

std::vector<rm_interfaces::msg::Armor>
ArmorPoseEstimator::extractArmorPoses(const std::vector<Armor> &armors,
                                   Eigen::Matrix3d R_imu_camera) {
  std::vector<rm_interfaces::msg::Armor> armors_msg;

  for (const auto &armor : armors) {
    std::vector<cv::Mat> rvecs, tvecs;

    // 先用PnP得到装甲板姿态的两个候选解（rvecs, tvecs）
    if (pnp_solver_->solvePnPGeneric(
            armor.landmarks(), rvecs, tvecs,
            (armor.type == ArmorType::SMALL ? "small" : "large"))) {
      // 根据几何先验选择更合理的解
      sortPnPResult(armor, rvecs, tvecs);
      cv::Mat rmat;
      cv::Rodrigues(rvecs[0], rmat);

      Eigen::Matrix3d R = utils::cvToEigen(rmat);
      Eigen::Vector3d t = utils::cvToEigen(tvecs[0]);

      // 计算云台系下装甲板的roll角，过大时通常不进行BA优化
      double armor_roll =
          rotationMatrixToRPY(R_gimbal_camera_ * R)[0] * 180 / M_PI;

      if (use_ba_ && std::abs(armor_roll) < 15) {
        // BA优化：在小倾角时使用，进一步优化旋转矩阵R
        R = ba_solver_->solveBa(armor, t, R, R_imu_camera);
      }
      if (!R.allFinite() || !t.allFinite()) {
        continue;
      }
      Eigen::Quaterniond q(R);

      // 打包ROS消息：包含类型、编号、位姿等
      rm_interfaces::msg::Armor armor_msg;

      // Fill basic info
      armor_msg.type = armorTypeToString(armor.type);
      armor_msg.number = armor.number;

      // Fill pose
      armor_msg.pose.position.x = t(0);
      armor_msg.pose.position.y = t(1);
      armor_msg.pose.position.z = t(2);
      armor_msg.pose.orientation.x = q.x();
      armor_msg.pose.orientation.y = q.y();
      armor_msg.pose.orientation.z = q.z();
      armor_msg.pose.orientation.w = q.w();

      // 计算到图像中心的距离，用于排序或权重
      armor_msg.distance_to_image_center =
          pnp_solver_->calculateDistanceToCenter(armor.center);

      armors_msg.push_back(std::move(armor_msg));
    } else {
      ZFM_WARN("armor_detector", "PnP Failed!");
    }
  }

  return armors_msg;
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d &R) {
  // Transform to camera frame
  Eigen::Quaterniond q(R);
  // Get armor yaw
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  Eigen::Vector3d rpy;
  tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

void ArmorPoseEstimator::sortPnPResult(const Armor &armor,
                                    std::vector<cv::Mat> &rvecs,
                                    std::vector<cv::Mat> &tvecs) const {
  constexpr float PROJECT_ERR_THRES = 3.0;

  if (rvecs.size() < 2 || tvecs.size() < 2) {
    return;
  }

  // 获取这两个解
  cv::Mat &rvec1 = rvecs.at(0);
  cv::Mat &tvec1 = tvecs.at(0);
  cv::Mat &rvec2 = rvecs.at(1);
  cv::Mat &tvec2 = tvecs.at(1);

  // 将旋转向量转换为旋转矩阵
  cv::Mat R1_cv, R2_cv;
  cv::Rodrigues(rvec1, R1_cv);
  cv::Rodrigues(rvec2, R2_cv);

  // 转换为Eigen矩阵
  Eigen::Matrix3d R1 = utils::cvToEigen(R1_cv);
  Eigen::Matrix3d R2 = utils::cvToEigen(R2_cv);

  // 计算云台系下装甲板的RPY角
  auto rpy1 = rotationMatrixToRPY(R_gimbal_camera_ * R1);
  auto rpy2 = rotationMatrixToRPY(R_gimbal_camera_ * R2);

  std::string coord_frame_name =
      (armor.type == ArmorType::SMALL ? "small" : "large");
  double error1 = pnp_solver_->calculateReprojectionError(
      armor.landmarks(), rvec1, tvec1, coord_frame_name);
  double error2 = pnp_solver_->calculateReprojectionError(
      armor.landmarks(), rvec2, tvec2, coord_frame_name);

  // 重投影误差差距足够大时，直接保留误差更小的解。
  if (error1 > error2 * PROJECT_ERR_THRES) {
    std::swap(rvec1, rvec2);
    std::swap(tvec1, tvec2);
    return;
  }
  if (error2 > error1 * PROJECT_ERR_THRES) {
    return;
  }

  // 两个解误差接近时，再用姿态和灯条倾斜方向消除歧义。
  constexpr double kRollLimit = 10.0 * M_PI / 180.0;
  if (std::abs(rpy1[0]) > kRollLimit || std::abs(rpy2[0]) > kRollLimit) {
    return;
  }

  // 计算灯条在图像中的倾斜角度
  double l_angle =
      std::atan2(armor.left_light.axis.y, armor.left_light.axis.x) * 180 / M_PI;
  double r_angle =
      std::atan2(armor.right_light.axis.y, armor.right_light.axis.x) * 180 /
      M_PI;
  double angle = (l_angle + r_angle) / 2;
  angle += 90.0;

  if (armor.number == "outpost") angle = -angle;

  // 根据倾斜角度选择解
  // 如果装甲板左倾（angle > 0），选择Yaw为负的解
  // 如果装甲板右倾（angle < 0），选择Yaw为正的解
  if ((angle > 0 && rpy1[2] > 0 && rpy2[2] < 0) ||
      (angle < 0 && rpy1[2] < 0 && rpy2[2] > 0)) {
    std::swap(rvec1, rvec2);
    std::swap(tvec1, tvec2);
  }
}

} // namespace zfm::auto_aim
