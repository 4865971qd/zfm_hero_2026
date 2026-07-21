#ifndef AUTO_AIM_ARMOR_HPP_
#define AUTO_AIM_ARMOR_HPP_

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace autoaim {

enum class EnemyColor { RED = 0, BLUE = 1, WHITE = 2 };

enum class ArmorType { SMALL, LARGE, INVALID };

enum class ArmorPriority {
    first = 1,
    second = 2,
    third = 3,
    fourth = 4,
    fifth = 5
};

constexpr double SMALL_ARMOR_WIDTH = 0.133;
constexpr double SMALL_ARMOR_HEIGHT = 0.050;
constexpr double LARGE_ARMOR_WIDTH = 0.225;
constexpr double LARGE_ARMOR_HEIGHT = 0.050;

struct Light : public cv::RotatedRect {
    EnemyColor color = EnemyColor::WHITE;
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    cv::Point2f axis;
    double length = 0.0;
    double width = 0.0;
    float tilt_angle = 0.0f;

    Light() = default;
    Light(const std::vector<cv::Point> &contour);
};

struct Armor {
    std::string number;
    ArmorType type = ArmorType::INVALID;
    EnemyColor color = EnemyColor::WHITE;
    ArmorPriority priority = ArmorPriority::fifth;
    float confidence = 0.0f;

    cv::Point2f center;

    Eigen::Vector3d xyz_in_camera{0, 0, 0};
    Eigen::Vector3d xyz_in_gimbal{0, 0, 0};
    Eigen::Vector3d xyz_in_world{0, 0, 0};
    Eigen::Vector3d ypr_in_gimbal{0, 0, 0};
    Eigen::Vector3d ypr_in_world{0, 0, 0};
    Eigen::Vector3d ypd_in_world{0, 0, 0};

    Light left_light;
    Light right_light;

    cv::Mat number_img;
    std::string classification_result;
    float distance_to_image_center = 0.0f;

    Armor() = default;
    Armor(const Light &l1, const Light &l2);

    std::vector<cv::Point2f> landmarks() const;

    static std::vector<cv::Point3f> buildObjectPoints(double w, double h,
                                                      int n_pts = 4);
};

struct Target {
    ArmorType armor_type = ArmorType::SMALL;
    ArmorPriority priority = ArmorPriority::fifth;
    int armor_num = 4;

    // EKF state: [xc, vxc, yc, vyc, zc, vzc, yaw, v_yaw, r, d_zc].
    Eigen::VectorXd state{10};
    double d_za = 0.0;
    double another_r = 0.0;

    bool valid = false;
    double current_time = 0.0;
    double normal_last_seen_time = 0.0;
    bool outpost_z_calibrated = true;
    int outpost_observed_plate = -1;
    double outpost_angle_offsets[3] = {0.0, 2.0943951023931953, 4.1887902047863905};
    double outpost_last_seen_time = 0.0;
    double outpost_phase_error = 0.0;

    std::vector<Eigen::Vector4d> armorXYZA() const;
};

struct GimbalCommand {
    double yaw = 0.0;
    double pitch = 0.0;
    double distance = 0.0;
    bool fire_advice = false;
};

}  // namespace autoaim

#endif  // AUTO_AIM_ARMOR_HPP_
