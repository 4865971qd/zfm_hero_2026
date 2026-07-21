#ifndef AUTO_AIM_TARGET_HPP_
#define AUTO_AIM_TARGET_HPP_

#include "armor.hpp"
#include "utils/ekf.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace autoaim {

constexpr int X_N = 10;
constexpr int Z_N = 4;

struct MeasureFunc {
    template <typename T>
    void operator()(const T x[X_N], T z[Z_N]) const {
        using std::cos;
        using std::sin;
        z[0] = x[0] - cos(x[6]) * x[8];
        z[1] = x[2] - sin(x[6]) * x[8];
        z[2] = x[4] + x[9];
        z[3] = x[6];
    }
};

using RobotEKF = ExtendedKalmanFilter<X_N, Z_N>;

class OutpostTracker {
public:
    enum State { COLLECTING = 0, CALIBRATING = 1, ACTIVE = 2, STATIC = 3 };

    OutpostTracker();

    bool addMeasurement(double x, double y, double z, double yaw, double t);
    double getLastSeenTime() const { return last_seen_t_; }
    std::vector<Eigen::Vector3d> getPredictedPositions(double t, double future_dt = 0);
    double getPlateAngle(double t, int plate) const;
    bool getCircleParams(double &cx, double &cy, double &R, double &omega);

    State getState() const { return state_; }
    double getZCenter(int i) const { return (i >= 0 && i < 3) ? z_centers_[i] : 0; }
    double getAngleOffset(int i) const { return (i >= 0 && i < 3) ? angle_offsets_[i] : 0; }
    bool isZCalibrated() const { return z_calibrated_; }
    double getPhaseError() const { return phase_error_; }
    int currentPlate() const { return current_plate_; }
    int collectCount() const { return collect_count_; }
    void reset();

private:
    double predictAngle(double t, int plate, double future_dt = 0.0) const;
    void updateZPlate(int plate, double z);
    void switchToStatic(double x, double y, double z, double yaw);

    State state_ = COLLECTING;
    double cx_ = 0;
    double cy_ = 0;
    const double fixed_R_ = 0.275;
    double omega_ = 0;
    double last_obs_angle_ = 0;
    double last_obs_time_ = 0;
    int last_obs_plate_ = 0;
    double static_x_ = 0;
    double static_y_ = 0;
    double static_z_ = 0;
    double static_yaw_ = 0;

    std::vector<double> traj_x_;
    std::vector<double> traj_y_;
    std::vector<double> motion_x_;
    std::vector<double> motion_y_;
    std::vector<double> motion_t_;
    double accum_dang_ = 0;
    double last_ang_ = 0;
    double last_t_ = 0;
    double last_z_ = 0;

    double z_centers_[3]{};
    double angle_offsets_[3]{};
    bool z_seen_[3]{};
    bool z_calibrated_ = false;
    double z_learning_rate_ = 0.05;

    bool center_init_ = false;
    int collect_count_ = 0;
    double collect_start_t_ = 0;
    double collect_span_ = 0;
    bool fit_candidate_valid_ = false;
    double fit_candidate_cx_ = 0;
    double fit_candidate_cy_ = 0;
    double fit_candidate_radius_ = 0;
    double fit_candidate_time_ = 0;
    const double min_collect_duration_ = 0.4;
    const double max_observation_extrapolation_ = 0.15;
    const double max_phase_extrapolation_ = 1.0;

    int current_plate_ = 1;
    double outlier_start_t_ = 0;
    double radial_outlier_start_t_ = 0;
    const double max_outlier_duration_ = 0.15;
    const double residual_thres_ = 0.2;

    double last_measurement_time_ = 0;
    double last_seen_t_ = 0;
    double phase_error_ = 0;
};

}  // namespace autoaim

#endif  // AUTO_AIM_TARGET_HPP_
