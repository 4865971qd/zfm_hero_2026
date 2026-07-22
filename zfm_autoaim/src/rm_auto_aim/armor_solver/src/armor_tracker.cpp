#include "armor_solver/armor_tracker.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "/opt/ros/humble/include/angles/angles/angles.h"

#include "rm_utils/logger/log.hpp"

namespace zfm::auto_aim {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

double normalizeAngle(double angle) noexcept {
  return std::remainder(angle, kTwoPi);
}

double unwrapNear(double angle, double reference) noexcept {
  return reference + normalizeAngle(angle - reference);
}

double angleDifference(double lhs, double rhs) noexcept {
  return std::abs(normalizeAngle(lhs - rhs));
}

bool heldFor(bool condition, double now, double duration, double &since) noexcept {
  if (!condition || !std::isfinite(now)) {
    since = -1.0;
    return false;
  }
  if (since < 0.0 || now < since) {
    since = now;
  }
  return now - since >= std::max(0.0, duration);
}

}  // namespace

Tracker::Tracker(double max_match_distance,
                 double max_match_yaw_diff,
                 double radius_min,
                 double radius_max,
                 double default_radius)
: tracker_state(LOST)
, tracked_id(std::string(""))
, measurement(Eigen::VectorXd::Zero(4))
, target_state(Eigen::VectorXd::Zero(10))
, d_za(0.0)
, another_r(default_radius)
, d_zc(0.0)
, outpost_tracker(nullptr)
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, radius_min_(radius_min)
, radius_max_(radius_max)
, default_radius_(default_radius) {
  if (radius_min_ > radius_max_) {
    std::swap(radius_min_, radius_max_);
  }
  default_radius_ = clampRadius(default_radius_);
  another_r = default_radius_;
}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept {
  if (armors_msg->armors.empty()) {
    return;
  }

  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto &armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  tracked_id = tracked_armor.number;

  if (tracked_id == "outpost") {
    outpost_tracker = std::make_unique<OutpostTracker>();
    tracked_armors_num = ArmorsNum::OUTPOST_3;
    tracker_state = DETECTING;
    ZFM_INFO("armor_solver", "Outpost tracker initialized!");
    return;
  }

  initEKF(tracked_armor);
  normal_last_seen_time =
    armors_msg->header.stamp.sec + armors_msg->header.stamp.nanosec * 1e-9;
  tracking_since_ = -1.0;
  lost_since_ = -1.0;
  tracker_state = DETECTING;
  tracked_armors_num = ArmorsNum::NORMAL_4;
}

void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept {
  const double t =
    armors_msg->header.stamp.sec + armors_msg->header.stamp.nanosec * 1e-9;
  if (tracked_id == "outpost") {
    if (!outpost_tracker) {
      outpost_tracker = std::make_unique<OutpostTracker>();
    }

    bool found = false;
    bool accepted_reliable_observation = false;
    double x = 0, y = 0, z = 0, yaw = 0;

    for (const auto &armor : armors_msg->armors) {
      if (armor.number == "outpost") {
        x = armor.pose.position.x;
        y = armor.pose.position.y;
        z = armor.pose.position.z;
        yaw = rawOrientationYaw(armor.pose.orientation);
        found = true;
        break;
      }
    }

    if (found) {
      last_outpost_meas_x = x; last_outpost_meas_y = y; last_outpost_meas_z = z;
      accepted_reliable_observation = outpost_tracker->addMeasurement(x, y, z, yaw, t);
      if (accepted_reliable_observation) {
        if (outpost_tracker->getState() == OutpostTracker::STATIC) {
          ZFM_INFO("armor_solver", "Outpost tracker: STATIC");
        } else {
          ZFM_INFO("armor_solver", "Outpost circle fitted! State: ACTIVE");
        }
      }
    }
    has_outpost_meas = found;
    has_reliable_outpost_observation = accepted_reliable_observation;

    if (outpost_tracker->getState() == OutpostTracker::ACTIVE ||
        outpost_tracker->getState() == OutpostTracker::STATIC) {

      if (outpost_tracker->getState() == OutpostTracker::ACTIVE) {
        // Direct passthrough — OutpostTracker locks cx/cy/omega after
        // COLLECTING, no additional smoothing needed at Tracker layer.
        double cx, cy, R, omega;
        outpost_tracker->getCircleParams(cx, cy, R, omega);
        const bool z_calibrated = outpost_tracker->isZCalibrated();
        const int observed_plate = outpost_tracker->currentPlate();
        const int ref_plate = z_calibrated ? 0 : observed_plate;
        double ref_plate_angle = outpost_tracker->getPlateAngle(t, ref_plate);
        double ref_z = outpost_tracker->getZCenter(ref_plate);

        target_state(0) = cx;
        target_state(1) = 0;
        target_state(2) = cy;
        target_state(3) = 0;
        target_state(4) = ref_z;
        target_state(5) = 0;
        target_state(6) = ref_plate_angle;
        target_state(7) = omega;
        target_state(8) = R;

        if (z_calibrated) {
          target_state(9) = outpost_tracker->getZCenter(1) - ref_z;
          d_za = outpost_tracker->getZCenter(2) - ref_z;
        } else {
          target_state(9) = 0; d_za = 0;
        }

        if (debug_) {
          static int tracker_log_cnt = 0;
          if (++tracker_log_cnt % 30 == 0) {
            ZFM_DEBUG("armor_solver",
              "[Outpost] 跟踪: yaw={:.1f}° ω={:.2f} cx=({:.2f},{:.2f}) z_cal={}",
              ref_plate_angle*180/M_PI, omega, cx, cy,
              outpost_tracker->isZCalibrated()?"是":"否");
          }
        }
      } else {
        // STATIC
        tracked_armors_num = ArmorsNum::OUTPOST_3;
        auto predicted_all = outpost_tracker->getPredictedPositions(t);
        if (!predicted_all.empty()) {
          target_state(0) = predicted_all[0].x();
          target_state(1) = 0;
          target_state(2) = predicted_all[0].y();
          target_state(3) = 0;
          target_state(4) = predicted_all[0].z();
          target_state(5) = 0;
        }
        target_state(6) = 0; target_state(7) = 0; target_state(8) = 0;
        target_state(9) = 0; d_za = 0; another_r = 0;
      }
    }

    if (outpost_tracker && outpost_tracker->getLastSeenTime() > 0.0 &&
        t - outpost_tracker->getLastSeenTime() > 1.0) {
      ZFM_INFO("armor_solver", "Outpost usable observation stale, transition to LOST for re-init");
      outpost_tracker.reset();
      outpost_detect_count_ = 0;
      tracker_state = LOST;
      return;
    }

    if (outpost_tracker->getState() == OutpostTracker::COLLECTING ||
        outpost_tracker->getState() == OutpostTracker::CALIBRATING) {
      tracker_state = DETECTING;
      outpost_detect_count_ = 0;
      return;
    }

    if (tracker_state == DETECTING) {
      if (outpost_tracker->getState() == OutpostTracker::ACTIVE ||
          outpost_tracker->getState() == OutpostTracker::STATIC) {
        outpost_detect_count_++;
        if (outpost_detect_count_ > 1) {
          outpost_detect_count_ = 0;
          tracker_state = TRACKING;
          ZFM_DEBUG("armor_solver", "Outpost tracker state: TRACKING");
        }
      }
    } else if (tracker_state == TRACKING) {
      if (!found) {
        tracker_state = TEMP_LOST;
      }
    } else if (tracker_state == TEMP_LOST) {
      if (found) {
        tracker_state = TRACKING;
      } else {
        if (t - outpost_tracker->getLastSeenTime() > 1.0) {
          ZFM_INFO("armor_solver", "Outpost lost, transition to LOST for re-init");
          outpost_tracker.reset();
          outpost_detect_count_ = 0;
          tracker_state = LOST;
        }
      }
    }
    return;
  }

  const Eigen::VectorXd state_before_predict = target_state;
  Eigen::VectorXd ekf_prediction = ekf->predict();
  if (!ekf_prediction.allFinite()) {
    if (!state_before_predict.allFinite()) {
      tracker_state = LOST;
      return;
    }
    ekf_prediction = state_before_predict;
    ekf->setState(ekf_prediction);
  }

  bool matched = false;
  double selected_yaw = target_state(6);
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    Armor same_id_armor;
    int same_id_armors_count = 0;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    for (const auto &armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        same_id_armor = armor;
        same_id_armors_count++;
        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff) {
          min_position_diff = position_diff;
          const double candidate_yaw = unwrapNear(
            rawOrientationYaw(armor.pose.orientation), ekf_prediction(6));
          yaw_diff = angleDifference(candidate_yaw, ekf_prediction(6));
          selected_yaw = candidate_yaw;
          tracked_armor = armor;
          tracked_armors_num = ArmorsNum::NORMAL_4;
        }
      }
    }

    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
      auto p = tracked_armor.pose.position;
      measurement = Eigen::Vector4d(p.x, p.y, p.z, selected_yaw);
      const Eigen::VectorXd state_before_update = target_state;
      Eigen::VectorXd updated_state = ekf->update(measurement);
      if (updated_state.allFinite()) {
        matched = true;
        target_state = updated_state;
        normal_last_seen_time = t;
      } else {
        target_state = state_before_update;
        ekf->setState(target_state);
      }
    } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
      const double jump_yaw = unwrapNear(
        rawOrientationYaw(same_id_armor.pose.orientation), ekf_prediction(6));
      matched = handleArmorJump(same_id_armor, jump_yaw);
      if (matched) normal_last_seen_time = t;
    } else if (debug_) {
      ZFM_DEBUG("armor_solver", "No matched armor found");
    }
  }

  double previous_r = target_state(8);
  double previous_another_r = another_r;
  target_state(8) = clampRadius(target_state(8));
  another_r = clampRadius(another_r);
  if (target_state(8) != previous_r || another_r != previous_another_r) {
    ekf->setState(target_state);
  }

  if (tracker_state == DETECTING) {
    if (matched) {
      if (heldFor(true, t, tracking_confirm_time, tracking_since_)) {
        tracking_since_ = -1.0;
        tracker_state = TRACKING;
        ZFM_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      }
    } else {
      tracking_since_ = -1.0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_since_ = t;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      if (heldFor(true, t, lost_time, lost_since_)) {
        lost_since_ = -1.0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_since_ = -1.0;
    }
  }
}

// Get predicted outpost plate positions at a future time.
std::vector<Eigen::Vector3d> Tracker::getOutpostPredicted(double current_time, double future_dt) {
  if (!outpost_tracker) return {};
  return outpost_tracker->getPredictedPositions(current_time, future_dt);
}

// Get outpost circle parameters (center, radius, angular velocity).
bool Tracker::getOutpostCircleParams(double &cx, double &cy, double &R, double &omega) {
  if (!outpost_tracker) return false;
  return outpost_tracker->getCircleParams(cx, cy, R, omega);
}

// Initialize EKF state from an armor measurement.
void Tracker::initEKF(const Armor &a) noexcept {
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  double yaw = normalizeAngle(rawOrientationYaw(a.pose.orientation));

  target_state = Eigen::VectorXd::Zero(X_N);
  double r = default_radius_;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  d_za = 0, d_zc = 0, another_r = r;
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf->setState(target_state);
}

// Handle sudden armor ID switch (jump detection and state correction).
bool Tracker::handleArmorJump(const Armor &current_armor, double yaw) noexcept {
  const Eigen::VectorXd previous_state = target_state;
  const double previous_d_za = d_za;
  const double previous_d_zc = d_zc;
  const double previous_another_r = another_r;
  double last_yaw = target_state(6);

  if (angleDifference(yaw, last_yaw) > 0.4) {
    target_state(6) = yaw;
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      target_state(8) = clampRadius(target_state(8));
      another_r = clampRadius(another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    ZFM_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) {
    d_zc = 0;
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);
    target_state(1) = 0;
    target_state(2) = p.y + r * sin(yaw);
    target_state(3) = 0;
    target_state(4) = p.z;
    target_state(5) = 0;
    target_state(9) = d_zc;
    ZFM_WARN("armor_solver", "State wrong!");
  }

  if (!target_state.allFinite()) {
    target_state = previous_state;
    d_za = previous_d_za;
    d_zc = previous_d_zc;
    another_r = previous_another_r;
    return false;
  }
  ekf->setState(target_state);
  return true;
}

double Tracker::rawOrientationYaw(const geometry_msgs::msg::Quaternion &q) noexcept {
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  tf2::Matrix3x3 R(tf_q);

  double nx = R.getColumn(2).x();
  double ny = R.getColumn(2).y();

  return std::atan2(ny, nx);
}

// Extract armor plate position from EKF state vector.
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept {
  double xc = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

double Tracker::clampRadius(double radius) const noexcept {
  return std::clamp(radius, radius_min_, radius_max_);
}

}  // namespace zfm::auto_aim
