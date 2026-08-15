#ifndef AUTO_AIM_PNP_HPP_
#define AUTO_AIM_PNP_HPP_

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoaim {

class PnPSolver {
public:
    PnPSolver(const std::array<double, 9> &camera_matrix,
              const std::vector<double> &distortion_coeffs,
              cv::SolvePnPMethod method = cv::SOLVEPNP_IPPE);

    void setObjectPoints(const std::string &name,
                         const std::vector<cv::Point3f> &pts) noexcept;

    template <class InputArray>
    bool solve(const InputArray &image_points,
               cv::Mat &rvec, cv::Mat &tvec,
               const std::string &coord_frame_name) {
        auto it = obj_pts_.find(coord_frame_name);
        if (it == obj_pts_.end()) return false;
        return cv::solvePnP(it->second, image_points,
                            camera_matrix_, dist_coeffs_,
                            rvec, tvec, false, method_);
    }

    double reprojectionError(const std::vector<cv::Point2f> &img_pts,
                             const cv::Mat &rvec, const cv::Mat &tvec,
                             const std::string &coord_frame_name) const noexcept;

    float distanceToCenter(const cv::Point2f &pt) const noexcept {
        return cv::norm(pt - cv::Point2f(cx_, cy_));
    }

    const cv::Mat &cameraMatrix() const noexcept { return camera_matrix_; }

private:
    std::unordered_map<std::string, std::vector<cv::Point3f>> obj_pts_;
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::SolvePnPMethod method_;
    float cx_, cy_;
};

}  // namespace autoaim

#endif  // AUTO_AIM_PNP_HPP_
