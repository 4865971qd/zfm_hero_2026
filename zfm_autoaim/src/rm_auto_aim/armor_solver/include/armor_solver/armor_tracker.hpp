#ifndef ARMOR_SOLVER_TRACKER_HPP_
#define ARMOR_SOLVER_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Eigen>

#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"
#include "armor_solver/motion_model.hpp"

namespace zfm::auto_aim {

enum class ArmorsNum { NORMAL_4 = 4, OUTPOST_3 = 3 };

class Tracker {
public:
  Tracker(double max_match_distance,
          double max_match_yaw,
          double radius_min = 0.23,
          double radius_max = 0.34,
          double default_radius = 0.26);

  using Armors = rm_interfaces::msg::Armors;
  using Armor = rm_interfaces::msg::Armor;

  // Initialize tracker with first armor detections.
  void init(const Armors::SharedPtr &armors_msg) noexcept;

  // Update tracker state with incoming armor detections.
  void update(const Armors::SharedPtr &armors_msg) noexcept;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  std::unique_ptr<RobotStateEKF> ekf;

  double tracking_confirm_time = 0.03;
  double lost_time = 0.30;

  Armor tracked_armor;
  std::string tracked_id;
  ArmorsNum tracked_armors_num;
  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;

  double d_za, another_r;
  double d_zc;
  double normal_last_seen_time = 0.0;

  std::unique_ptr<OutpostTracker> outpost_tracker;

  bool debug_ = false;

  // Enable or disable debug output.
  void setDebug(bool debug) {
    debug_ = debug;
  }

  // Get predicted outpost plate positions at a future time.
  std::vector<Eigen::Vector3d> getOutpostPredicted(double current_time, double future_dt = 0);
  bool isOutpostMode() const { return tracked_id == "outpost"; }
  bool isOutpostActive() const {
    return outpost_tracker && outpost_tracker->getState() == OutpostTracker::ACTIVE;
  }
  bool isOutpostStatic() const {
    return outpost_tracker && outpost_tracker->getState() == OutpostTracker::STATIC;
  }
  bool isOutpostZCalibrated() const {
    return tracked_id != "outpost" || (outpost_tracker && outpost_tracker->isZCalibrated());
  }
  double getOutpostLastSeenTime() const {
    return outpost_tracker ? outpost_tracker->getLastSeenTime() : 0.0;
  }
  double getOutpostPhaseError() const {
    return outpost_tracker ? outpost_tracker->getPhaseError() : 0.0;
  }
  int getOutpostObservedPlate() const {
    return outpost_tracker ? outpost_tracker->currentPlate() : -1;
  }
  double getOutpostAngleOffset(int i) const {
    return outpost_tracker ? outpost_tracker->getAngleOffset(i) : 0.0;
  }

  // Get outpost circle parameters (center, radius, angular velocity).
  bool getOutpostCircleParams(double &cx, double &cy, double &R, double &omega);

  double last_outpost_meas_x = 0, last_outpost_meas_y = 0, last_outpost_meas_z = 0;
  bool has_outpost_meas = false;
  bool has_reliable_outpost_observation = false;

private:
  // Initialize EKF state from an armor measurement.
  void initEKF(const Armor &a) noexcept;

  // Handle sudden armor ID switch (jump detection and state correction).
  bool handleArmorJump(const Armor &a, double measured_yaw) noexcept;

  // Convert quaternion to continuous yaw angle.
  static double rawOrientationYaw(const geometry_msgs::msg::Quaternion &q) noexcept;

  // Extract armor plate position from EKF state vector.
  static Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x) noexcept;

  // Keep normal-robot estimated radius inside the physical target range.
  double clampRadius(double radius) const noexcept;

  double max_match_distance_;
  double max_match_yaw_diff_;
  double radius_min_;
  double radius_max_;
  double default_radius_;
  double tracking_since_ = -1.0;
  double lost_since_ = -1.0;
  int outpost_detect_count_ = 0;
};

}  // namespace zfm::auto_aim

#endif  // ARMOR_SOLVER_TRACKER_HPP_
