#ifndef RM_UTILS_TRAJECTORY_COMPENSATOR_HPP_
#define RM_UTILS_TRAJECTORY_COMPENSATOR_HPP_

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <tuple>

namespace zfm {

class TrajectoryCompensator {
public:
  TrajectoryCompensator() = default;
  virtual ~TrajectoryCompensator() = default;

  // Compensate the trajectory of the bullet, return the pitch increment
  virtual bool compensate(const Eigen::Vector3d &target_position, double &pitch) const noexcept;

  virtual double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept = 0;

  std::vector<std::pair<double, double>> getTrajectory(double distance,
                                                       double angle) const noexcept;

  double velocity = 15.0;
  int iteration_times = 20;
  double gravity = 9.8;
  double resistance = 0.01;

protected:
  // Calculate the trajectory of the bullet, return the vertical impact point
  virtual double calculateTrajectory(const double x, const double angle) const noexcept = 0;
};

// IdealCompensator does not consider the air resistance
class IdealCompensator : public TrajectoryCompensator {
public:
  double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept override;

protected:
  double calculateTrajectory(const double x, const double angle) const noexcept override;
};

// ResistanceCompensator considers the air resistance
class ResistanceCompensator : public TrajectoryCompensator {
public:
  double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept override;

protected:
  double calculateTrajectory(const double x, const double angle) const noexcept override;
};

// QuadraticDragCompensator: full 2D quadratic drag model with RK4 integration
// State: [x, y, vx, vy]
//   dvx/dt = -k * v * vx      where k = 0.5*rho*Cd*A/m, v = sqrt(vx^2+vy^2)
//   dvy/dt = -g - k * v * vy
//   dx/dt = vx,  dy/dt = vy
class QuadraticDragCompensator : public TrajectoryCompensator {
public:
  QuadraticDragCompensator() = default;

  bool compensate(const Eigen::Vector3d &target_position, double &pitch) const noexcept override;
  double getFlyingTime(const Eigen::Vector3d &target_position) const noexcept override;

  // Physical projectile parameters
  double drag_coefficient = 0.50;       // Cd (dimensionless)
  double air_density = 1.225;           // kg/m^3
  double projectile_mass = 0.0445;      // kg
  double projectile_diameter = 0.0425;  // m

protected:
  double calculateTrajectory(const double x, const double angle) const noexcept override;

private:
  // Compute drag constant k = 0.5 * rho * Cd * A / m
  double dragConstant() const noexcept;

  // Single RK4 step: state = [x, y, vx, vy]
  void rk4Step(std::array<double, 4> &state, double dt, double k, double g) const noexcept;

  // Full integration until x >= target_x, returns {y_at_target, t_at_target}
  // Returns {NaN, NaN} if target not reached within max_time
  struct ImpactResult {
    double y;
    double t;
  };
  ImpactResult integrateToDistance(double target_x, double angle, double k, double g,
                                   double max_time = 3.0) const noexcept;
};

// Factory class for trajectory compensator
class CompensatorFactory {
public:
  static std::unique_ptr<TrajectoryCompensator> createCompensator(const std::string &type) {
    if (type == "ideal") {
      return std::make_unique<IdealCompensator>();
    } else if (type == "resistance") {
      return std::make_unique<ResistanceCompensator>();
    } else if (type == "quadratic_drag") {
      return std::make_unique<QuadraticDragCompensator>();
    } else {
      return nullptr;
    }
  }

private:
  CompensatorFactory() = delete;
  ~CompensatorFactory() = delete;
};

}  // namespace zfm
#endif
