#include "trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace autoaim {

// ============ TrajectoryCompensator ============

bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target,
                                        double &pitch) const noexcept {
    double h = target.z();
    double dist = std::sqrt(target.x() * target.x() + target.y() * target.y());
    double iterative_h = h;
    double angle = std::atan2(h, dist);
    double dh = 0.0;

    for (int i = 0; i < iteration_times; ++i) {
        angle = std::atan2(iterative_h, dist);
        if (std::abs(angle) > M_PI / 2.5) break;
        double impact = calculateTrajectory(dist, angle);
        dh = h - impact;
        if (std::abs(dh) < 0.01) break;
        iterative_h += dh;
    }
    if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) return false;
    pitch = angle;
    return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
        double dist, double angle) const noexcept {
    std::vector<std::pair<double, double>> traj;
    if (dist < 0) return traj;
    for (double x = 0; x < dist; x += 0.03)
        traj.emplace_back(x, calculateTrajectory(x, angle));
    return traj;
}

// ============ IdealCompensator ============

double IdealCompensator::calculateTrajectory(double x, double angle) const noexcept {
    double t = x / (velocity * std::cos(angle));
    return velocity * std::sin(angle) * t - 0.5 * gravity * t * t;
}

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target) const noexcept {
    double dist = std::sqrt(target.x() * target.x() + target.y() * target.y());
    double angle = std::atan2(target.z(), dist);
    return dist / (velocity * std::cos(angle));
}

// ============ QuadraticDragCompensator ============

double QuadraticDragCompensator::dragConstant() const noexcept {
    double area = M_PI * projectile_diameter * projectile_diameter / 4.0;
    return 0.5 * air_density * drag_coefficient * area / projectile_mass;
}

void QuadraticDragCompensator::rk4Step(std::array<double, 4> &s,
                                        double dt, double k, double g) const noexcept {
    auto deriv = [k, g](const std::array<double, 4> &p) -> std::array<double, 4> {
        double v = std::sqrt(p[2] * p[2] + p[3] * p[3]);
        double drag = k * v;
        return {p[2], p[3], -drag * p[2], -g - drag * p[3]};
    };
    std::array<double, 4> k1 = deriv(s);
    std::array<double, 4> s2; for (int i = 0; i < 4; ++i) s2[i] = s[i] + 0.5 * dt * k1[i];
    std::array<double, 4> k2 = deriv(s2);
    std::array<double, 4> s3; for (int i = 0; i < 4; ++i) s3[i] = s[i] + 0.5 * dt * k2[i];
    std::array<double, 4> k3 = deriv(s3);
    std::array<double, 4> s4; for (int i = 0; i < 4; ++i) s4[i] = s[i] + dt * k3[i];
    std::array<double, 4> k4 = deriv(s4);
    for (int i = 0; i < 4; ++i)
        s[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

auto QuadraticDragCompensator::integrateToDistance(
        double target_x, double angle, double k, double g, double max_time) const noexcept -> ImpactResult {
    ImpactResult r = {std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN()};
    double v0 = velocity;
    std::array<double, 4> state = {0.0, 0.0, v0 * std::cos(angle), v0 * std::sin(angle)};
    constexpr double dt = 0.001;
    double t = 0.0, prev_x = 0.0, prev_y = 0.0, prev_t = 0.0;
    int max_steps = static_cast<int>(max_time / dt);
    for (int step = 0; step < max_steps; ++step) {
        prev_x = state[0]; prev_y = state[1]; prev_t = t;
        rk4Step(state, dt, k, g);
        t += dt;
        if (state[0] >= target_x) {
            double dx = state[0] - prev_x;
            if (dx > 1e-12) {
                double frac = (target_x - prev_x) / dx;
                r.y = prev_y + frac * (state[1] - prev_y);
                r.t = prev_t + frac * dt;
            } else { r.y = state[1]; r.t = t; }
            return r;
        }
        if (state[1] < -10.0) return r;
    }
    return r;
}

double QuadraticDragCompensator::calculateTrajectory(double x, double angle) const noexcept {
    auto r = integrateToDistance(x, angle, dragConstant(), gravity);
    return std::isnan(r.y) ? -1e6 : r.y;
}

double QuadraticDragCompensator::getFlyingTime(const Eigen::Vector3d &target) const noexcept {
    double dist = std::sqrt(target.x() * target.x() + target.y() * target.y());
    double angle = std::atan2(target.z(), dist);
    auto r = integrateToDistance(dist, angle, dragConstant(), gravity);
    return std::isnan(r.t) ? (dist / (velocity * std::cos(std::abs(angle) < 1e-6 ? 1e-6 : angle)))
                           : r.t;
}

bool QuadraticDragCompensator::compensate(const Eigen::Vector3d &target,
                                           double &pitch) const noexcept {
    double h = target.z();
    double dist = std::sqrt(target.x() * target.x() + target.y() * target.y());
    if (dist < 0.01) { pitch = std::atan2(h, 0.01); return true; }
    double k = dragConstant(), g = gravity;
    double iterative_h = h;
    double angle = std::atan2(h, dist);
    double dh = 0.0;
    for (int i = 0; i < iteration_times; ++i) {
        angle = std::atan2(iterative_h, dist);
        if (std::abs(angle) > M_PI / 2.5) break;
        auto r = integrateToDistance(dist, angle, k, g);
        if (std::isnan(r.y)) break;
        dh = h - r.y;
        if (std::abs(dh) < 0.01) break;
        iterative_h += dh;
    }
    if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) return false;
    pitch = angle;
    return true;
}

// ============ Factory ============

std::unique_ptr<TrajectoryCompensator> CompensatorFactory::create(const std::string &type) {
    if (type == "ideal")           return std::make_unique<IdealCompensator>();
    if (type == "quadratic_drag")  return std::make_unique<QuadraticDragCompensator>();
    return nullptr;
}

}  // namespace autoaim
