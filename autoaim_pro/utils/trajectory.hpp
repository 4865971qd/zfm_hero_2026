#ifndef AUTO_AIM_TRAJECTORY_HPP_
#define AUTO_AIM_TRAJECTORY_HPP_

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace autoaim {

class TrajectoryCompensator {
public:
    TrajectoryCompensator() = default;
    virtual ~TrajectoryCompensator() = default;

    virtual bool compensate(const Eigen::Vector3d &target, double &pitch) const noexcept;
    virtual double getFlyingTime(const Eigen::Vector3d &target) const noexcept = 0;

    std::vector<std::pair<double, double>> getTrajectory(double dist, double angle) const noexcept;

    double velocity = 15.0;
    int    iteration_times = 20;
    double gravity = 9.8;

protected:
    virtual double calculateTrajectory(double x, double angle) const noexcept = 0;
};

// ---- 理想弹道 (真空) ----
class IdealCompensator : public TrajectoryCompensator {
public:
    double getFlyingTime(const Eigen::Vector3d &target) const noexcept override;
protected:
    double calculateTrajectory(double x, double angle) const noexcept override;
};

// ---- 二次阻力 (RK4 积分) ----
class QuadraticDragCompensator : public TrajectoryCompensator {
public:
    bool compensate(const Eigen::Vector3d &target, double &pitch) const noexcept override;
    double getFlyingTime(const Eigen::Vector3d &target) const noexcept override;

    double drag_coefficient = 0.50;
    double air_density = 1.225;
    double projectile_mass = 0.0445;
    double projectile_diameter = 0.0425;

protected:
    double calculateTrajectory(double x, double angle) const noexcept override;

private:
    double dragConstant() const noexcept;
    void rk4Step(std::array<double, 4> &state, double dt, double k, double g) const noexcept;

    struct ImpactResult { double y, t; };
    ImpactResult integrateToDistance(double target_x, double angle,
                                     double k, double g,
                                     double max_time = 3.0) const noexcept;
};

// ---- 工厂 ----
class CompensatorFactory {
public:
    static std::unique_ptr<TrajectoryCompensator> create(const std::string &type);
};

}  // namespace autoaim

#endif  // AUTO_AIM_TRAJECTORY_HPP_
