#ifndef AUTO_AIM_EKF_HPP_
#define AUTO_AIM_EKF_HPP_

#include <Eigen/Dense>
#if __has_include(<ceres/jet.h>)
#include <ceres/jet.h>
#define HAS_CERES 1
#else
#define HAS_CERES 0
#endif
#include <functional>
#include <cmath>

#include "math.hpp"

namespace autoaim {

// 扩展卡尔曼滤波器 (Ceres Jet 自动微分)
// N_X: 状态维数  N_Z: 观测维数
template <int N_X, int N_Z>
class ExtendedKalmanFilter {
public:
    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
    using VectorX  = Eigen::Matrix<double, N_X, 1>;
    using VectorZ  = Eigen::Matrix<double, N_Z, 1>;

    using UpdateQFunc = std::function<MatrixXX()>;
    using UpdateRFunc = std::function<MatrixZZ(const VectorZ &z)>;
    using PredictFunc  = std::function<void(const ceres::Jet<double, N_X> *,
                                            ceres::Jet<double, N_X> *)>;
    using MeasureFunc  = std::function<void(const ceres::Jet<double, N_X> *,
                                            ceres::Jet<double, N_X> *)>;

    ExtendedKalmanFilter() = default;

    // 简化构造: 只需要观测函数 h 和 R 更新函数
    ExtendedKalmanFilter(const Eigen::VectorXd &x0, const MatrixXX &P0,
                         MeasureFunc h, UpdateRFunc ur)
        : h_(std::move(h)), update_R_(std::move(ur)),
          P_post_(P0)
    {
        for (int i = 0; i < N_X && i < x0.size(); ++i) x_post_[i] = x0[i];
        F_ = MatrixXX::Identity();
        H_ = MatrixZX::Zero();
    }

    void setState(const VectorX &x) noexcept { x_post_ = x; }
    const VectorX &state() const noexcept { return x_post_; }
    VectorX &state() noexcept { return x_post_; }

    // ---- 预测 (恒定速度模型，显式 dt) ----
    VectorX predict(double dt) noexcept {
        // 状态转移矩阵 F (恒定速度)
        F_ = MatrixXX::Identity();
        F_(0, 1) = dt; F_(2, 3) = dt; F_(4, 5) = dt; F_(6, 7) = dt;

        // 先验状态: x_pri = F * x_post
        x_pri_ = F_ * x_post_;
        x_pri_[6] = math::limitRad(x_pri_[6]);  // yaw 归一化

        // 过程噪声 Q (Piecewise White Noise)
        double a = dt * dt * dt * dt / 4.0;
        double b = dt * dt * dt / 2.0;
        double c = dt * dt;
        double v1 = 100.0, v2 = 400.0;
        if (std::abs(x_post_[7]) > 2.5) { v1 = 10.0; v2 = 0.1; }

        Q_ = MatrixXX::Zero();
        auto setQ = [&](int i, double v) {
            Q_(i, i) = a * v; Q_(i, i+1) = b * v;
            Q_(i+1, i) = b * v; Q_(i+1, i+1) = c * v;
        };
        setQ(0, v1); setQ(2, v1); setQ(4, v1); setQ(6, v2);

        // 先验协方差
        P_pri_ = F_ * P_post_ * F_.transpose() + Q_;
        x_post_ = x_pri_;
        return x_pri_;
    }

    // ---- 更新 (Ceres Jet 自动微分 / 解析 Jacobian 回退) ----
    VectorX update(const Eigen::VectorXd &z_in) noexcept {
        VectorZ z;
        for (int i = 0; i < N_Z && i < z_in.size(); ++i) z[i] = z_in[i];
        VectorZ z_pri;
#if HAS_CERES
        ceres::Jet<double, N_X> x_jet[N_X];
        for (int i = 0; i < N_X; ++i) {
            x_jet[i].a = x_pri_[i];
            x_jet[i].v[i] = 1.0;
        }
        ceres::Jet<double, N_X> zp_jet[N_Z];
        h_(x_jet, zp_jet);

        for (int i = 0; i < N_Z; ++i) {
            z_pri[i] = zp_jet[i].a;
            H_.row(i) = zp_jet[i].v.transpose();
        }
#else
        // 解析 Jacobian: z = [x0-cos(x6)*x8, x2-sin(x6)*x8, x4+x9, x6]
        double s6 = std::sin(x_pri_[6]), c6 = std::cos(x_pri_[6]);
        z_pri << x_pri_[0] - c6 * x_pri_[8],
                 x_pri_[2] - s6 * x_pri_[8],
                 x_pri_[4] + x_pri_[9],
                 x_pri_[6];

        H_ = MatrixZX::Zero();
        H_(0, 0) = 1; H_(0, 6) =  x_pri_[8] * s6;  H_(0, 8) = -c6;
        H_(1, 2) = 1; H_(1, 6) = -x_pri_[8] * c6;  H_(1, 8) = -s6;
        H_(2, 4) = 1; H_(2, 9) = 1;
        H_(3, 6) = 1;
#endif

        R_ = update_R_(z);
        MatrixZZ S = H_ * P_pri_ * H_.transpose() + R_;
        K_ = P_pri_ * H_.transpose() * S.inverse();
        x_post_ = x_pri_ + K_ * (z - z_pri);
        P_post_ = (MatrixXX::Identity() - K_ * H_) * P_pri_;
        return x_post_;
    }

    // ---- 访问器 ----
    const MatrixXX &P() const noexcept { return P_post_; }
    const VectorX &x()  const noexcept { return x_post_; }
    VectorX &x() noexcept { return x_post_; }

private:
    PredictFunc  f_;
    MeasureFunc  h_;
    UpdateQFunc  update_Q_;
    UpdateRFunc  update_R_;

    MatrixXX F_, P_pri_, P_post_, Q_;
    MatrixZX H_;
    MatrixZZ R_;
    MatrixXZ K_;
    VectorX  x_pri_, x_post_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_EKF_HPP_
