#include "pnp.hpp"
#include <opencv2/calib3d.hpp>

namespace autoaim {

PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix,
                     const std::vector<double> &distortion_coeffs,
                     cv::SolvePnPMethod method)
    : method_(method)
{
    camera_matrix_ = cv::Mat(3, 3, CV_64F,
        const_cast<double *>(camera_matrix.data())).clone();
    dist_coeffs_ = cv::Mat(1, 5, CV_64F,
        const_cast<double *>(distortion_coeffs.data())).clone();
    cx_ = static_cast<float>(camera_matrix_.at<double>(0, 2));
    cy_ = static_cast<float>(camera_matrix_.at<double>(1, 2));
}

void PnPSolver::setObjectPoints(const std::string &name,
                                const std::vector<cv::Point3f> &pts) noexcept {
    obj_pts_[name] = pts;
}

double PnPSolver::reprojectionError(const std::vector<cv::Point2f> &img_pts,
                                     const cv::Mat &rvec, const cv::Mat &tvec,
                                     const std::string &coord_frame_name) const noexcept {
    auto it = obj_pts_.find(coord_frame_name);
    if (it == obj_pts_.end()) return 0.0;
    std::vector<cv::Point2f> projected;
    cv::projectPoints(it->second, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
    double error = 0.0;
    for (size_t i = 0; i < img_pts.size(); ++i)
        error += cv::norm(img_pts[i] - projected[i]);
    return error;
}

}  // namespace autoaim
