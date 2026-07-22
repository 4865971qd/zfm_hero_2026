#include "armor_solver/armor_solver_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "armor_solver/motion_model.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"
#include "/opt/ros/humble/include/angles/angles/angles.h"

namespace zfm::auto_aim {
namespace {

bool isFresh(double now, double last_seen, double max_unseen) noexcept {
  if (!std::isfinite(now) || !std::isfinite(last_seen) || last_seen <= 0.0) {
    return false;
  }
  const double unseen = now - last_seen;
  return unseen >= -0.02 && unseen <= max_unseen;
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

ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions &options)
: Node("armor_solver", options), solver_(nullptr) {
  ZFM_REGISTER_LOGGER("armor_solver", "~/zfm2026-log", DEBUG);
  ZFM_INFO("armor_solver", "Starting ArmorSolverNode!");

  debug_mode_ = this->declare_parameter("debug", true);

  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  double radius_min = this->declare_parameter("tracker.radius_min", 0.23);
  double radius_max = this->declare_parameter("tracker.radius_max", 0.34);
  double default_radius = this->declare_parameter("tracker.default_radius", 0.26);
  tracker_ = std::make_unique<Tracker>(
      max_match_distance, max_match_yaw_diff, radius_min, radius_max, default_radius);
  tracker_->setDebug(debug_mode_);
  tracking_confirm_time_ =
    this->declare_parameter("tracker.tracking_confirm_time", 0.03);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
  tracker_->tracking_confirm_time = tracking_confirm_time_;
  tracker_->lost_time = lost_time_thres_;
  normal_max_aim_unseen_time_ =
    this->declare_parameter("solver.normal_max_aim_unseen_time", 0.30);

  auto f = Predict(0.005);
  auto h = Measure();
  s2qx_ = declare_parameter("ekf.sigma2_q_x", 20.0);
  s2qy_ = declare_parameter("ekf.sigma2_q_y", 20.0);
  s2qz_ = declare_parameter("ekf.sigma2_q_z", 20.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 800.0);
  s2qd_zc_ = declare_parameter("ekf.sigma2_q_d_zc", 800.0);

  auto u_q = [this]() {
    Eigen::Matrix<double, X_N, X_N> q;
    double t = dt_, x = s2qx_, y = s2qy_, z = s2qz_, yaw = s2qyaw_, r = s2qr_, d_zc = s2qd_zc_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * z, q_vz_vz = pow(t, 2) * z;
    double q_yaw_yaw = pow(t, 4) / 4 * yaw, q_yaw_vyaw = pow(t, 3) / 2 * yaw,
           q_vyaw_vyaw = pow(t, 2) * yaw;
    double q_r = pow(t, 4) / 4 * r;
    double q_d_zc = pow(t, 4) / 4 * d_zc;
    q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0, 0,
         q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0, q_y_y, q_y_vy, 0, 0, 0, 0, 0, 0,
         0, 0, q_y_vy, q_vy_vy, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, q_z_z, q_z_vz, 0, 0, 0, 0,
         0, 0, 0, 0, q_z_vz, q_vz_vz, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, q_yaw_yaw, q_yaw_vyaw, 0, 0,
         0, 0, 0, 0, 0, 0, q_yaw_vyaw, q_vyaw_vyaw, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, q_r, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0, q_d_zc;
    return q;
  };
  r_x_ = declare_parameter("ekf.r_x", 0.05);
  r_y_ = declare_parameter("ekf.r_y", 0.05);
  r_z_ = declare_parameter("ekf.r_z", 0.05);
  r_yaw_ = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::Matrix<double, Z_N, 1> &z) {
    Eigen::Matrix<double, Z_N, Z_N> r;
    constexpr double kMinMeasurementVariance = 1e-6;
    r << std::max(r_x_ * std::abs(z[0]), kMinMeasurementVariance), 0, 0, 0,
         0, std::max(r_y_ * std::abs(z[1]), kMinMeasurementVariance), 0, 0,
         0, 0, std::max(r_z_ * std::abs(z[2]), kMinMeasurementVariance), 0,
         0, 0, 0, r_yaw_;
    return r;
  };
  Eigen::DiagonalMatrix<double, X_N> p0;
  p0.setIdentity();
  tracker_->ekf = std::make_unique<RobotStateEKF>(f, h, u_q, u_r, p0);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_, *tf2_buffer_, target_frame_, 10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  tf2_filter_->registerCallback(&ArmorSolverNode::armorsCallback, this);

  measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>("armor_solver/measurement", rclcpp::SensorDataQoS());
  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("armor_solver/target", rclcpp::SensorDataQoS());
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal", rclcpp::SensorDataQoS());
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4), std::bind(&ArmorSolverNode::timerCallback, this));
  armor_target_.header.frame_id = "";

  pitch_dead_zone_ = this->declare_parameter("solver.pitch_dead_zone", 0.1);
  yaw_dead_zone_ = this->declare_parameter("solver.yaw_dead_zone", 0.1);
  stationary_confirm_time_ =
    this->declare_parameter("ekf.stationary_confirm_time", 0.15);
  stationary_release_time_ =
    this->declare_parameter("ekf.stationary_release_time", 0.05);
  stationary_vel_threshold_ = this->declare_parameter("ekf.stationary_vel_threshold", 0.05);
  stationary_release_vel_threshold_ =
    this->declare_parameter("ekf.stationary_release_vel_threshold", 0.08);

  enable_ = true;
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode", std::bind(&ArmorSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  if (debug_mode_) {
    initMarkers();
  }

  heartbeat_ = HeartBeatPublisher::create(this);

  if (debug_mode_) {
    plotter_ = std::make_unique<zfm::Plotter>(*this);
  }

  bullet_speed_ = this->declare_parameter("solver.bullet_speed", 11.3);
  mode_flag_ = 0;
  serial_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>(
      "serial/receive", rclcpp::SensorDataQoS(),
      std::bind(&ArmorSolverNode::serialCallback, this, std::placeholders::_1));
}

// Receive mode flag and gimbal angles from serial.
void ArmorSolverNode::serialCallback(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg) {
  mode_flag_ = msg->mode_flag;
  last_serial_yaw_ = msg->yaw;
  last_serial_pitch_ = msg->pitch;
}

// Publish gimbal command at fixed interval.
void ArmorSolverNode::timerCallback() {
  if (solver_ == nullptr) {
    return;
  }

  if (!enable_) {
    return;
  }

  solver_->setBulletSpeed(bullet_speed_);

  rm_interfaces::msg::GimbalCmd control_msg;

  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  const double now_sec = this->now().seconds();
  const bool normal_target_stale = armor_target_.id != "outpost" &&
    !isFresh(now_sec, armor_target_.normal_last_seen_time, normal_max_aim_unseen_time_);
  if (armor_target_.tracking && !normal_target_stale) {
    try {
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_);

      if (!dead_zone_initialized_) {
        last_sent_pitch_ = control_msg.pitch;
        last_sent_yaw_ = control_msg.yaw;
        dead_zone_initialized_ = true;
      } else {
        if (std::abs(control_msg.pitch - last_sent_pitch_) < pitch_dead_zone_) {
          control_msg.pitch = last_sent_pitch_;
        } else {
          last_sent_pitch_ = control_msg.pitch;
        }
        if (std::abs(control_msg.yaw - last_sent_yaw_) < yaw_dead_zone_) {
          control_msg.yaw = last_sent_yaw_;
        } else {
          last_sent_yaw_ = control_msg.yaw;
        }
      }

      if (plotter_) {
        plotter_->publishGimbalCmd(control_msg.yaw, control_msg.pitch,
                                   control_msg.yaw_diff, control_msg.pitch_diff,
                                   control_msg.distance, control_msg.fire_advice);
        plotter_->publishGimbalState(last_serial_yaw_, last_serial_pitch_,
                                     armor_target_.yaw, armor_target_.v_yaw, dt_);
      }
    } catch (...) {
      ZFM_ERROR("armor_solver", "Something went wrong in solver!");
      control_msg.yaw_diff = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance = -1;
      control_msg.fire_advice = false;
    }
  } else {
    dead_zone_initialized_ = false;
    solver_->resetTrackingState();
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.fire_advice = false;
  }
  gimbal_pub_->publish(control_msg);

  if (debug_mode_) {
    static double last_marker_publish_time = -1.0;
    const double marker_time = this->now().seconds();
    if (last_marker_publish_time < 0.0 || marker_time < last_marker_publish_time ||
        marker_time - last_marker_publish_time >= 1.0 / 30.0) {
      publishMarkers(armor_target_, control_msg);
      last_marker_publish_time = marker_time;
    }
  }
}

// Initialize RViz visualization markers.
void ArmorSolverNode::initMarkers() noexcept {
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "filtered_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;
  selection_marker_.ns = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.g = 1.0;
  selection_marker_.color.r = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;
  trajectory_marker_.points.clear();

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
}

// Process incoming armor detections: transform, filter, track, solve.
void ArmorSolverNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_msg) {
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
    solver_->setDebug(debug_mode_);
  }

  for (auto &armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::TransformException &ex) {
      ZFM_ERROR("armor_solver", "Transform error: {}", ex.what());
      return;
    }
  }

  armors_msg->armors.erase(std::remove_if(armors_msg->armors.begin(),
                                          armors_msg->armors.end(),
                                          [](const rm_interfaces::msg::Armor &armor) {
                                            return abs(armor.pose.position.z) > 2;
                                          }),
                           armors_msg->armors.end());

  rm_interfaces::msg::Measurement measure_msg;
  rm_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    const double raw_dt = (time - last_time_).seconds();
    const bool abnormal_dt = raw_dt <= 0 || raw_dt > 0.05 || !std::isfinite(raw_dt);
    dt_ = std::clamp(abnormal_dt ? 0.005 : raw_dt, 0.001, 0.05);
    if (abnormal_dt) {
      stationary_mode_ = false;
      stationary_since_ = -1.0;
      moving_since_ = -1.0;
    }
    if (debug_mode_ && abnormal_dt) {
      ZFM_DEBUG("armor_solver", "Clamped abnormal frame dt {:.6f}s to {:.6f}s", raw_dt, dt_);
    }

    // Outpost tracker does not use EKF — only set predict model for normal targets.
    if (tracker_->tracked_id != "outpost") {
      MotionModel model = MotionModel::CONSTANT_VEL_ROT;
      if (tracker_->tracker_state == Tracker::TRACKING) {
        const auto &s = tracker_->target_state;
        const double now = time.seconds();
        bool currently_stationary = std::abs(s(1)) < stationary_vel_threshold_ &&
                                     std::abs(s(3)) < stationary_vel_threshold_ &&
                                     std::abs(s(5)) < stationary_vel_threshold_ &&
                                     std::abs(s(7)) < stationary_vel_threshold_;
        if (!stationary_mode_) {
          stationary_mode_ = heldFor(
            currently_stationary, now, stationary_confirm_time_, stationary_since_);
        } else {
          const bool clearly_moving =
            std::abs(s(1)) > stationary_release_vel_threshold_ ||
            std::abs(s(3)) > stationary_release_vel_threshold_ ||
            std::abs(s(5)) > stationary_release_vel_threshold_ ||
            std::abs(s(7)) > stationary_release_vel_threshold_;
          if (heldFor(clearly_moving, now, stationary_release_time_, moving_since_)) {
            stationary_mode_ = false;
            stationary_since_ = -1.0;
            moving_since_ = -1.0;
          }
        }
        if (stationary_mode_) {
          model = MotionModel::CONSTANT_STATIONARY;
        }
      } else {
        stationary_mode_ = false;
        stationary_since_ = -1.0;
        moving_since_ = -1.0;
      }
      tracker_->ekf->setPredictFunc(Predict{dt_, model});
    }
    tracker_->update(armors_msg);

    measure_msg.x = tracker_->measurement(0);
    measure_msg.y = tracker_->measurement(1);
    measure_msg.z = tracker_->measurement(2);
    measure_msg.yaw = tracker_->measurement(3);
    measure_pub_->publish(measure_msg);

    if (plotter_) {
      const auto &s = tracker_->target_state;
      int ts = static_cast<int>(tracker_->tracker_state);
      plotter_->publishTarget({s(0), s(1), s(2), s(3), s(4), s(5), s(6), s(7), s(8), s(9)}, ts);
      plotter_->publishEkfResidual(
          tracker_->measurement(0), tracker_->measurement(1),
          tracker_->measurement(2), tracker_->measurement(3),
          s(0), s(2), s(4), s(6));
    }

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (tracker_->tracker_state == Tracker::TRACKING ||
               tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      const auto &state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.outpost_z_calibrated = tracker_->isOutpostZCalibrated();
      target_msg.outpost_observed_plate = tracker_->getOutpostObservedPlate();
      if (target_msg.armors_num == 3 && tracker_->outpost_tracker) {
        const bool z_calibrated = tracker_->isOutpostZCalibrated();
        const int ref_plate = z_calibrated ? 0 : tracker_->getOutpostObservedPlate();
        for (int i = 0; i < 3; ++i) {
          if (z_calibrated) {
            target_msg.outpost_angle_offsets[i] =
                angles::normalize_angle(tracker_->getOutpostAngleOffset(i) -
                                        tracker_->getOutpostAngleOffset(ref_plate));
          } else {
            target_msg.outpost_angle_offsets[i] = i * 2.0 * M_PI / 3.0;
          }
        }
        if (!z_calibrated) {
          target_msg.outpost_observed_plate = 0;
        }
      } else {
        target_msg.outpost_angle_offsets[0] = 0.0;
        target_msg.outpost_angle_offsets[1] = 2.0 * M_PI / 3.0;
        target_msg.outpost_angle_offsets[2] = 4.0 * M_PI / 3.0;
      }
      target_msg.outpost_last_seen_time = tracker_->getOutpostLastSeenTime();
      target_msg.outpost_phase_error = tracker_->getOutpostPhaseError();
      target_msg.outpost_has_observed_z =
        target_msg.armors_num == 3 && tracker_->has_reliable_outpost_observation;
      target_msg.outpost_observed_z =
        target_msg.outpost_has_observed_z ? tracker_->last_outpost_meas_z : 0.0;
      target_msg.normal_last_seen_time = tracker_->normal_last_seen_time;
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.d_zc = state(9);
      target_msg.d_za = tracker_->d_za;
    }
  }

  armor_target_ = target_msg;
  target_pub_->publish(target_msg);

  last_time_ = time;
}

// Publish visualization markers for target state and trajectory.
void ArmorSolverNode::publishMarkers(const rm_interfaces::msg::Target &target_msg,
                                     const rm_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept {
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armors_marker_.header = target_msg.header;
  selection_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;

  auto mkDelete = [&](const std::string &ns, int id = 0) {
    visualization_msgs::msg::Marker marker;
    marker.ns = ns;
    marker.id = id;
    marker.header = target_msg.header;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  };

  auto appendOutpostDeletes = [&]() {
    const char *namespaces[] = {
      "state", "raw_meas", "circle_z0", "circle_z1", "circle_z2", "center",
      "rotation", "model_future", "strike_pred", "plates_now", "selected_aim",
      "selected_aim_label", "static_mode"
    };
    for (const char *ns : namespaces) {
      marker_array.markers.push_back(mkDelete(ns));
    }
  };

  // Conditional markers are deleted before this frame's valid markers are appended.
  appendOutpostDeletes();

  auto mkSphere = [&](const char* ns, double x, double y, double z, double size,
                      float r, float g, float b, float a = 1.0f) {
    visualization_msgs::msg::Marker m;
    m.ns = ns;
    m.header = target_msg.header;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.scale.x = m.scale.y = m.scale.z = size;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
    return m;
  };

  if (target_msg.tracking) {
    if (target_msg.id == "outpost" && tracker_->isOutpostMode()) {
      marker_array.markers.push_back(mkDelete("position"));
      marker_array.markers.push_back(mkDelete("linear_v"));
      marker_array.markers.push_back(mkDelete("angular_v"));
      marker_array.markers.push_back(mkDelete("selection"));
      marker_array.markers.push_back(mkDelete("obs_center"));
      marker_array.markers.push_back(mkDelete("obs_ekf_diff"));
      for (int id = 0; id < 4; ++id) {
        marker_array.markers.push_back(mkDelete("filtered_armors", id));
      }

      auto mkText = [&](const char* ns, const std::string& text,
                        double x, double y, double z, float size, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.ns = ns;
        m.header = target_msg.header;
        m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        m.text = text;
        m.scale.z = size;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
        return m;
      };

      std::string state_str;
      if (tracker_->isOutpostActive())
        state_str = "ACTIVE";
      else if (tracker_->isOutpostStatic())
        state_str = "STATIC";
      else if (tracker_->outpost_tracker &&
               tracker_->outpost_tracker->getState() == OutpostTracker::CALIBRATING)
        state_str = "CALIBRATING Z";
      else if (tracker_->outpost_tracker)
        state_str = "COLLECTING";
      if (!state_str.empty() && !tracker_->isOutpostActive()) {
        double text_z = target_msg.position.z + 0.5;
        marker_array.markers.push_back(mkText("state", state_str,
          target_msg.position.x, target_msg.position.y, text_z, 0.4, 1, 1, 1));
      }

      if (tracker_->has_outpost_meas) {
        marker_array.markers.push_back(mkSphere("raw_meas",
          tracker_->last_outpost_meas_x,
          tracker_->last_outpost_meas_y,
          tracker_->last_outpost_meas_z,
          0.06, 1.0, 0.2, 0.8));
      }

      const auto &aim_visualization = solver_->aimVisualization();
      if (aim_visualization.valid && aim_visualization.is_outpost) {
        visualization_msgs::msg::Marker selected_marker;
        selected_marker.ns = "selected_aim";
        selected_marker.header = target_msg.header;
        selected_marker.type = visualization_msgs::msg::Marker::CUBE;
        selected_marker.action = visualization_msgs::msg::Marker::ADD;
        selected_marker.scale.x = 0.03;
        selected_marker.scale.y = 0.135;
        selected_marker.scale.z = 0.055;
        selected_marker.color.r = 1.0;
        selected_marker.color.g = 0.05;
        selected_marker.color.b = 0.5;
        selected_marker.color.a = 1.0;
        selected_marker.pose.position.x = aim_visualization.point.x();
        selected_marker.pose.position.y = aim_visualization.point.y();
        selected_marker.pose.position.z = aim_visualization.point.z();
        tf2::Quaternion selected_orientation;
        selected_orientation.setRPY(0, -0.2618, aim_visualization.yaw);
        selected_marker.pose.orientation = tf2::toMsg(selected_orientation);
        marker_array.markers.push_back(selected_marker);

        marker_array.markers.push_back(mkText(
          "selected_aim_label",
          "AIM P" + std::to_string(aim_visualization.selected_plate),
          aim_visualization.point.x(), aim_visualization.point.y(),
          aim_visualization.point.z() + 0.12, 0.12, 1.0, 0.05, 0.5));
      }

      if (tracker_->isOutpostStatic()) {
        marker_array.markers.push_back(
          mkText("static_mode", "STATIC",
            target_msg.position.x, target_msg.position.y,
            target_msg.position.z + 0.3, 0.5, 0.5, 0.5, 0.5));
      }

      // Trajectory for outpost
      trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
      trajectory_marker_.points.clear();
      trajectory_marker_.header.frame_id = "gimbal_link";
      trajectory_marker_.header.stamp = target_msg.header.stamp;
      for (const auto &point : solver_->getTrajectory()) {
        geometry_msgs::msg::Point p;
        p.x = point.first;
        p.z = point.second;
        trajectory_marker_.points.emplace_back(p);
      }
      trajectory_marker_.color.r = gimbal_cmd.fire_advice ? 0.0 : 1.0;
      trajectory_marker_.color.g = 1.0;
      trajectory_marker_.color.b = gimbal_cmd.fire_advice ? 0.0 : 1.0;

      marker_array.markers.push_back(trajectory_marker_);
      marker_pub_->publish(marker_array);
      return;
    }

    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double d_za = target_msg.d_za, d_zc = target_msg.d_zc;

    // --- Observation markers: raw detector measurement vs EKF prediction ---
    if (tracker_->tracker_state == Tracker::TRACKING) {
      double meas_x = tracker_->measurement(0);
      double meas_y = tracker_->measurement(1);
      double meas_z = tracker_->measurement(2);
      double meas_yaw = tracker_->measurement(3);

      // Yellow sphere: raw measured armor plate position (what the detector saw)
      marker_array.markers.push_back(
        mkSphere("raw_meas", meas_x, meas_y, meas_z, 0.065, 1.0, 0.85, 0.0));

      // Orange sphere: center position implied by raw measurement
      double obs_cx = meas_x + r1 * std::cos(meas_yaw);
      double obs_cy = meas_y + r1 * std::sin(meas_yaw);
      marker_array.markers.push_back(
        mkSphere("obs_center", obs_cx, obs_cy, zc, 0.05, 1.0, 0.55, 0.0));

      // Orange line: observation center → EKF center (shows filter residual)
      visualization_msgs::msg::Marker diff_line;
      diff_line.ns = "obs_ekf_diff";
      diff_line.header = target_msg.header;
      diff_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      diff_line.scale.x = 0.015;
      diff_line.color.r = 1.0; diff_line.color.g = 0.5; diff_line.color.b = 0.0;
      diff_line.color.a = 0.8;
      diff_line.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point p_obs, p_ekf;
      p_obs.x = obs_cx; p_obs.y = obs_cy; p_obs.z = zc;
      p_ekf.x = xc; p_ekf.y = yc; p_ekf.z = zc;
      diff_line.points.push_back(p_obs);
      diff_line.points.push_back(p_ekf);
      marker_array.markers.push_back(diff_line);
    }

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = zc;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::msg::Marker::ADD;
    armors_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = zc + d_zc + (is_current_pair ? 0 : d_za);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = zc;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);
      armors_marker_.id = i;
      armors_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.2618 : 0.2618, tmp_yaw);
      armors_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armors_marker_);
    }

    const auto &aim_visualization = solver_->aimVisualization();
    if (aim_visualization.valid && !aim_visualization.is_outpost) {
      selection_marker_.action = visualization_msgs::msg::Marker::ADD;
      selection_marker_.pose.position.x = aim_visualization.point.x();
      selection_marker_.pose.position.y = aim_visualization.point.y();
      selection_marker_.pose.position.z = aim_visualization.point.z();
    } else {
      selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
    }

    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.header.frame_id = "gimbal_link";
    for (const auto &point : solver_->getTrajectory()) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire_advice) {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } else {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }

  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    selection_marker_.action = visualization_msgs::msg::Marker::DELETE;

    marker_array.markers.push_back(mkDelete("raw_meas"));
    marker_array.markers.push_back(mkDelete("obs_center"));
    marker_array.markers.push_back(mkDelete("obs_ekf_diff"));
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  marker_array.markers.emplace_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

// Handle vision mode switch service request.
void ArmorSolverNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    ZFM_ERROR("armor_solver", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
    case VisionMode::AUTO_AIM_BLUE: {
      enable_ = true;
      break;
    }
    default: {
      enable_ = false;
      if (solver_ != nullptr) {
        solver_->resetTrackingState();
      }
      if (debug_mode_ && !armor_target_.header.frame_id.empty()) {
        auto clear_target = armor_target_;
        clear_target.tracking = false;
        rm_interfaces::msg::GimbalCmd clear_command;
        publishMarkers(clear_target, clear_command);
      }
      break;
    }
  }

  ZFM_WARN("armor_solver", "Set Mode to {}", visionModeToString(mode));
}

}  // namespace zfm::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(zfm::auto_aim::ArmorSolverNode)
