#ifndef AUTO_AIM_SOLVER_HPP_
#define AUTO_AIM_SOLVER_HPP_

#include "armor.hpp"
#include "utils/manual_compensator.hpp"
#include "utils/trajectory.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace autoaim {

class Solver {
public:
    enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 };

    explicit Solver(const YAML::Node &cfg);

    void setImu(double roll, double pitch, double yaw);

    void setR_g2w(const Eigen::Matrix3d &R_gimbal, const Eigen::Matrix3d &R_cam) {
        R_g2w_ = R_gimbal;
        Eigen::Vector3d fwd = R_cam.col(2);
        current_yaw_ = std::atan2(fwd.y(), fwd.x());
        current_pitch_ = std::atan2(fwd.z(), fwd.head<2>().norm());
    }

    void setBulletSpeed(double v) noexcept { compensator_->velocity = v; }

    bool solveArmor(Armor &armor);

    std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &center,
                                                    double yaw, double r1, double r2,
                                                    double d_zc, double d_za,
                                                    int num,
                                                    const double *angle_offsets = nullptr) const noexcept;

    int selectBestArmor(const std::vector<Eigen::Vector3d> &positions,
                         const Eigen::Vector3d &center,
                         double yaw, double v_yaw, int num,
                         double flying_time = 0,
                         const double *angle_offsets = nullptr) const noexcept;

    GimbalCommand solve(const Target &target, double current_time);

    std::vector<std::pair<double, double>> getTrajectory() const noexcept;

    void setDebug(bool d) { debug_ = d; }

    const Eigen::Matrix3d &R_gimbal2world() const noexcept { return R_g2w_; }
    double bulletSpeed() const noexcept { return compensator_->velocity; }
    void resetCommandDeadZone() noexcept {
        dead_zone_initialized_ = false;
        center_enter_since_ = -1.0;
        center_exit_since_ = -1.0;
        state_ = TRACKING_ARMOR;
    }

    bool hasSelectedArmor() const { return has_selected_armor_; }
    Eigen::Vector3d selectedArmorPosition() const { return selected_armor_pos_; }
    double selectedArmorYaw() const { return selected_armor_yaw_; }
    int selectedArmorIndex() const { return selected_armor_index_; }

    std::vector<cv::Point2f> reproject(const Eigen::Vector3d &xyz_world, double yaw,
                                       ArmorType type, bool is_outpost = false) const;

private:
    void sortPnPResult(const Armor &armor,
                       const std::vector<cv::Point3f> &obj_pts,
                       std::vector<cv::Mat> &rvecs,
                       std::vector<cv::Mat> &tvecs) const;
    double reprojectionError(const std::vector<cv::Point3f> &obj_pts,
                             const std::vector<cv::Point2f> &img_pts,
                             const cv::Mat &rvec,
                             const cv::Mat &tvec) const;
    void applyCommandDeadZone(GimbalCommand &cmd) noexcept;

    void calcYawAndPitch(const Eigen::Vector3d &p, double &yaw, double &pitch) const noexcept;
    bool isOnTarget(double cur_yaw, double cur_pitch,
                    double target_yaw, double target_pitch,
                    double dist) const noexcept;

    Eigen::Matrix3d R_c2g_;
    Eigen::Vector3d t_c2g_;
    Eigen::Matrix3d R_g2w_;
    Eigen::Vector3d muzzle_offset_{0.095, 0.0, 0.0};
    double current_yaw_ = 0.0;
    double current_pitch_ = 0.0;

    cv::Mat K_, D_;
    double cx_ = 0.0;
    double cy_ = 0.0;

    std::unique_ptr<TrajectoryCompensator> compensator_;
    std::unique_ptr<ManualCompensator> manual_comp_;

    double shooting_w_ = 0.135;
    double shooting_h_ = 0.135;
    double max_tracking_v_yaw_ = 60.0;
    double prediction_delay_ = -0.005;
    double controller_delay_ = 0.0;
    double outpost_yaw_offset_ = 0.0;
    double outpost_max_unseen_time_ = 0.15;
    double outpost_max_phase_error_ = 0.2617993877991494;
    double normal_max_fire_unseen_time_ = 0.10;
    double side_angle_ = 45.0;
    double min_switching_v_yaw_ = 0.5;
    double normal_select_hysteresis_base_ = 0.010;
    double normal_select_hysteresis_gain_ = 0.015;
    double normal_select_hysteresis_max_ = 0.045;
    double yaw_dead_zone_ = 0.1;
    double pitch_dead_zone_ = 0.1;

    double center_switch_confirm_time_ = 0.05;
    double center_enter_since_ = -1.0;
    double center_exit_since_ = -1.0;
    State state_ = TRACKING_ARMOR;

    mutable int aim_idx_ = -1;
    mutable double smooth_aim_z_[3] = {0.0, 0.0, 0.0};
    mutable bool smooth_aim_z_initialized_[3] = {false, false, false};
    mutable int outpost_pending_idx_ = -1;
    mutable double outpost_pending_since_ = -1.0;
    mutable double outpost_last_switch_time_ = -1.0;
    mutable int normal_stable_idx_ = -1;

    Eigen::Vector3d selected_armor_pos_{0, 0, 0};
    double selected_armor_yaw_ = 0;
    int selected_armor_index_ = -1;
    bool has_selected_armor_ = false;

    bool debug_ = false;
    bool dead_zone_initialized_ = false;
    double last_cmd_yaw_ = 0.0;
    double last_cmd_pitch_ = 0.0;
};

}  // namespace autoaim

#endif  // AUTO_AIM_SOLVER_HPP_
