#include "rm_utils/math/trajectory_compensator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace zfm {
bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  // 目标高度（Z轴），单位：米
  double target_height = target_position(2);
  // 迭代使用的“假设高度”，每次根据误差进行修正
  double iterative_height = target_height;
  // 最终弹道在水平距离处的落点高度
  double impact_height = 0;
  // 水平距离：仅取 X-Y 平面上的距离
  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  // 初始发射角：用目标的仰角作为初值
  double angle = std::atan2(target_height, distance);
  // 高度误差（目标高度 - 轨迹高度）
  double dh = 0;
  // 迭代寻找使落点高度等于目标高度的发射角
  for (int i = 0; i < iteration_times; ++i) {
    // 根据当前假设高度，重新估计发射角
    angle = std::atan2(iterative_height, distance);
    // 限制极端角度，防止数值不稳定
    if (std::abs(angle) > M_PI / 2.5) {
      break;
    }
    // 计算该角度下，子弹在该水平距离的高度（抛物线/阻力模型）
    impact_height = calculateTrajectory(distance, angle);
    // 与目标高度的差值
    dh = target_height - impact_height;
    // 收敛条件：高度误差小于 1cm
    if (std::abs(dh) < 0.01) {
      break;
    }
    // 用误差修正“假设高度”，相当于简单的一维牛顿法
    iterative_height += dh;
  }
  // 若未收敛或角度越界，返回失败
  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }
  // 输出补偿后的俯仰角
  pitch = angle;
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  for (double x = 0; x < distance; x += 0.03) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }
  return trajectory;
}

double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  // 理想模型：忽略空气阻力
  // 飞行时间：水平速度为 v*cos(angle)
  double t = x / (velocity * cos(angle));
  // 竖直位移：y = v*sin(angle)*t - 1/2*g*t^2
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = distance / (velocity * cos(angle));
  return t;
}

double ResistanceCompensator::calculateTrajectory(const double x,
                                                  const double angle) const noexcept {
  // 简化阻力模型：以水平位移x估计时间，t = (e^{r x}-1)/(r v cos(angle))
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double t = (exp(r * x) - 1) / (r * velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = (exp(r * distance) - 1) / (r * velocity * cos(angle));
  return t;
}

// ============================================================================
// QuadraticDragCompensator: full 2D quadratic drag with RK4 integration
// ============================================================================

double QuadraticDragCompensator::dragConstant() const noexcept {
  double area = M_PI * projectile_diameter * projectile_diameter / 4.0;
  return 0.5 * air_density * drag_coefficient * area / projectile_mass;
}

void QuadraticDragCompensator::rk4Step(std::array<double, 4> &state, double dt, double k,
                                       double g) const noexcept {
  // state = [x, y, vx, vy]
  // Derivatives: [vx, vy, -k*v*vx, -g - k*v*vy]  where v = sqrt(vx^2+vy^2)
  auto deriv = [k, g](const std::array<double, 4> &s) -> std::array<double, 4> {
    double v = std::sqrt(s[2] * s[2] + s[3] * s[3]);
    double drag = k * v;
    return {s[2], s[3], -drag * s[2], -g - drag * s[3]};
  };

  std::array<double, 4> k1 = deriv(state);
  std::array<double, 4> s2;
  for (int i = 0; i < 4; ++i) s2[i] = state[i] + 0.5 * dt * k1[i];
  std::array<double, 4> k2 = deriv(s2);
  std::array<double, 4> s3;
  for (int i = 0; i < 4; ++i) s3[i] = state[i] + 0.5 * dt * k2[i];
  std::array<double, 4> k3 = deriv(s3);
  std::array<double, 4> s4;
  for (int i = 0; i < 4; ++i) s4[i] = state[i] + dt * k3[i];
  std::array<double, 4> k4 = deriv(s4);

  for (int i = 0; i < 4; ++i)
    state[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
}

QuadraticDragCompensator::ImpactResult QuadraticDragCompensator::integrateToDistance(
    double target_x, double angle, double k, double g, double max_time) const noexcept {
  ImpactResult result = {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN()};

  double v0 = velocity;
  std::array<double, 4> state = {0.0, 0.0, v0 * std::cos(angle), v0 * std::sin(angle)};

  constexpr double dt = 0.001;  // 1ms time step
  double t = 0.0;

  // Store previous state for interpolation
  double prev_x = state[0];
  double prev_y = state[1];
  double prev_t = t;

  int max_steps = static_cast<int>(max_time / dt);
  for (int step = 0; step < max_steps; ++step) {
    prev_x = state[0];
    prev_y = state[1];
    prev_t = t;

    rk4Step(state, dt, k, g);
    t += dt;

    // Check if we've passed the target x
    if (state[0] >= target_x) {
      // Linear interpolation to find y at exactly target_x
      double dx = state[0] - prev_x;
      if (dx > 1e-12) {
        double frac = (target_x - prev_x) / dx;
        result.y = prev_y + frac * (state[1] - prev_y);
        result.t = prev_t + frac * dt;
      } else {
        result.y = state[1];
        result.t = t;
      }
      return result;
    }

    // Stop if projectile hit the ground (y < -10m) to avoid wasting steps
    if (state[1] < -10.0) {
      return result;
    }
  }

  return result;  // Didn't reach target
}

double QuadraticDragCompensator::calculateTrajectory(const double x,
                                                     const double angle) const noexcept {
  double k = dragConstant();
  double g = gravity;
  ImpactResult result = integrateToDistance(x, angle, k, g);
  return std::isnan(result.y) ? -1e6 : result.y;
}

double QuadraticDragCompensator::getFlyingTime(
    const Eigen::Vector3d &target_position) const noexcept {
  double distance = std::sqrt(target_position(0) * target_position(0) +
                              target_position(1) * target_position(1));
  double angle = std::atan2(target_position(2), distance);
  double k = dragConstant();
  double g = gravity;
  ImpactResult result = integrateToDistance(distance, angle, k, g);
  return std::isnan(result.t) ? (distance / (velocity * std::cos(std::abs(angle) < 1e-6 ? 1e-6 : angle)))
                              : result.t;
}

bool QuadraticDragCompensator::compensate(const Eigen::Vector3d &target_position,
                                           double &pitch) const noexcept {
  double target_height = target_position(2);
  double distance = std::sqrt(target_position(0) * target_position(0) +
                              target_position(1) * target_position(1));

  if (distance < 0.01) {
    pitch = std::atan2(target_height, 0.01);
    return true;
  }

  double k = dragConstant();
  double g = gravity;

  // Initial guess: ideal parabolic angle + drag correction
  double iterative_height = target_height;
  double angle = std::atan2(target_height, distance);
  double dh = 0;

  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);
    if (std::abs(angle) > M_PI / 2.5) break;

    ImpactResult result = integrateToDistance(distance, angle, k, g);
    if (std::isnan(result.y)) break;

    dh = target_height - result.y;
    if (std::abs(dh) < 0.01) break;
    iterative_height += dh;
  }

  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }

  pitch = angle;
  return true;
}

}  // namespace zfm
