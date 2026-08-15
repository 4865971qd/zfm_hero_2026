#ifndef AUTO_AIM_TRACKER_HPP_
#define AUTO_AIM_TRACKER_HPP_

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "utils/ekf.hpp"

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace autoaim {

enum class ArmorsNum { NORMAL_4 = 4, OUTPOST_3 = 3 };
enum class TrackerState { LOST, DETECTING, TRACKING, TEMP_LOST };

class Tracker {
public:
    struct DebugInfo {
        bool matched = false;
        int same_count = 0;
        int matched_plate = -1;
        bool plate_pending = false;
        double dt = 0.0;
        double pos_diff = -1.0;
        double yaw_diff = -1.0;
        Eigen::Vector4d measurement{0, 0, 0, 0};
        Eigen::Vector4d prediction{0, 0, 0, 0};
        Eigen::Vector4d residual{0, 0, 0, 0};
    };

    explicit Tracker(const YAML::Node &cfg);

    void track(std::list<Armor> &armors, std::chrono::steady_clock::time_point t);

    Target getTarget() const { return target_; }
    bool hasTarget() const {
        return tracker_state_ == TrackerState::TRACKING ||
               tracker_state_ == TrackerState::TEMP_LOST;
    }

    TrackerState state() const { return tracker_state_; }
    std::string trackedId() const { return tracked_id_; }
    ArmorsNum armorsNum() const { return tracked_num_; }
    const DebugInfo &debugInfo() const noexcept { return debug_info_; }
    void setDebug(bool d) { debug_ = d; }

    std::vector<Eigen::Vector3d> getOutpostPredicted(double t, double dt = 0);
    bool getOutpostCircleParams(double &cx, double &cy, double &R, double &omega);


private:
    void init(const std::list<Armor> &armors, std::chrono::steady_clock::time_point t);
    void update(const std::list<Armor> &armors, std::chrono::steady_clock::time_point t);
    void initEKF(const Armor &a);
    bool handleArmorJump(const Armor &a, double measured_yaw);

    std::unique_ptr<RobotEKF> ekf_;
    std::unique_ptr<OutpostTracker> outpost_;

    TrackerState tracker_state_ = TrackerState::LOST;
    std::string tracked_id_;
    ArmorsNum tracked_num_ = ArmorsNum::NORMAL_4;
    Armor tracked_armor_;

    Eigen::VectorXd measurement_{Z_N};
    Eigen::VectorXd target_state_{X_N};

    Target target_;

    double max_match_dist_ = 0.5;
    double max_match_yaw_ = 1.0;
    double radius_min_ = 0.23;
    double radius_max_ = 0.34;
    double default_radius_ = 0.26;
    double tracking_confirm_time_ = 0.03;
    double lost_time_ = 0.30;
    double stationary_confirm_time_ = 0.15;
    double stationary_release_time_ = 0.05;
    double stationary_vel_threshold_ = 0.05;
    double stationary_release_vel_threshold_ = 0.08;

    double tracking_since_ = -1.0;
    double lost_since_ = -1.0;
    double stationary_since_ = -1.0;
    double moving_since_ = -1.0;
    bool stationary_mode_ = false;
    double normal_last_seen_time_ = 0;
    int normal_stable_plate_ = -1;
    int normal_pending_plate_ = -1;
    int normal_pending_frames_ = 0;
    int outpost_detect_count_ = 0;
    double d_za_ = 0;
    double d_zc_ = 0;
    double another_r_ = 0;
    bool debug_ = false;
    DebugInfo debug_info_;

    std::chrono::steady_clock::time_point last_t_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_TRACKER_HPP_
