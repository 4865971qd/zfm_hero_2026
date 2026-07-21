#ifndef AUTO_AIM_MATH_HPP_
#define AUTO_AIM_MATH_HPP_

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>

namespace autoaim::math {

// ---- 欧拉角 / 旋转矩阵 ----

enum class EulerOrder { XYZ, XZY, YXZ, YZX, ZXY, ZYX };

template <typename Vec3Like>
inline Eigen::Matrix3d eulerToMatrix(const Vec3Like &euler, EulerOrder order = EulerOrder::XYZ) {
    auto r = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
    auto p = Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY());
    auto y = Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
    switch (order) {
        case EulerOrder::XYZ: return (y * p * r).matrix();
        case EulerOrder::XZY: return (p * y * r).matrix();
        case EulerOrder::YXZ: return (y * r * p).matrix();
        case EulerOrder::YZX: return (r * y * p).matrix();
        case EulerOrder::ZXY: return (p * r * y).matrix();
        case EulerOrder::ZYX: return (r * p * y).matrix();
    }
    return Eigen::Matrix3d::Identity();
}

inline Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d &R,
                                     EulerOrder order = EulerOrder::XYZ) noexcept {
    return R.eulerAngles(static_cast<int>(order) % 3,
                         (static_cast<int>(order) / 3 + 1) % 3,
                         (static_cast<int>(order) / 3 * 2 + 2) % 3);
}

// 自定义 RPY 提取（zfm_ws1 原版逻辑）
inline Eigen::Vector3d getRPY(const Eigen::Matrix3d &R) {
    double yaw = std::atan2(R(0, 1), R(0, 0));
    double c2 = Eigen::Vector2d(R(2, 2), R(1, 2)).norm();
    double pitch = std::atan2(-R(0, 2), c2);
    double s1 = std::sin(yaw), c1 = std::cos(yaw);
    double roll = std::atan2(s1 * R(2, 0) - c1 * R(2, 1),
                             c1 * R(1, 1) - s1 * R(1, 0));
    return -Eigen::Vector3d(roll, pitch, yaw);
}

// 提取欧拉角 (axis0, axis1, axis2)
inline Eigen::Vector3d eulers(const Eigen::Matrix3d &R, int a0, int a1, int a2) {
    return R.eulerAngles(a0, a1, a2);
}

// ---- 角度工具 ----

inline double limitRad(double angle) {
    while (angle > M_PI)  angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

inline double shortestAngularDistance(double from, double to) {
    return limitRad(to - from);
}

// ---- 四元数 ----

inline Eigen::Quaterniond eulerToQuat(double roll, double pitch, double yaw) {
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
}

// ---- OpenCV ↔ Eigen ----

template <typename T, int R, int C>
inline cv::Mat eigenToCv(const Eigen::Matrix<T, R, C> &m) {
    cv::Mat cv_mat;
    cv::eigen2cv(m, cv_mat);
    return cv_mat;
}

inline Eigen::MatrixXd cvToEigen(const cv::Mat &cv_mat) noexcept {
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(cv_mat.rows, cv_mat.cols);
    cv::cv2eigen(cv_mat, m);
    return m;
}

// ---- 坐标转换 ----

// 笛卡尔 → 球坐标 (yaw, pitch, distance)
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d &xyz) {
    return {
        std::atan2(xyz.y(), xyz.x()),
        std::atan2(xyz.z(), xyz.head<2>().norm()),
        xyz.norm()
    };
}

// xyz2ypd 的雅可比
inline Eigen::Matrix3d xyz2ypdJacobian(const Eigen::Vector3d &xyz) {
    double x = xyz.x(), y = xyz.y(), z = xyz.z();
    double d2 = x * x + y * y;
    double d = std::sqrt(d2);
    double r2 = d2 + z * z;
    double r = std::sqrt(r2);

    Eigen::Matrix3d J;
    // d(yaw)/d(x,y,z)
    J(0, 0) = -y / d2;   J(0, 1) = x / d2;   J(0, 2) = 0;
    // d(pitch)/d(x,y,z)
    J(1, 0) = -x * z / (d * r2);  J(1, 1) = -y * z / (d * r2);  J(1, 2) = d / r2;
    // d(distance)/d(x,y,z)
    J(2, 0) = x / r;      J(2, 1) = y / r;      J(2, 2) = z / r;
    return J;
}

// 获取两个向量之间的绝对夹角 (rad)
inline double getAbsAngle(const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
    double dot = a.dot(b);
    double cross = a.x() * b.y() - a.y() * b.x();
    return std::abs(std::atan2(cross, dot));
}

inline double square(double x) { return x * x; }

// ---- 时间工具 ----

template <typename Clock = std::chrono::steady_clock>
inline double deltaTime(const typename Clock::time_point &t1,
                        const typename Clock::time_point &t2) {
    return std::chrono::duration<double>(t1 - t2).count();
}

}  // namespace autoaim::math

#endif  // AUTO_AIM_MATH_HPP_
