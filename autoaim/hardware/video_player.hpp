#ifndef AUTO_AIM_VIDEO_PLAYER_HPP_
#define AUTO_AIM_VIDEO_PLAYER_HPP_

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

namespace autoaim::hardware {

// 离线视频回放 (zfm_ws1 video_player 移植)
class VideoPlayer {
public:
    VideoPlayer(const std::string &path, double fps = 110.0, bool loop = false);

    bool read(cv::Mat &bgr_img, std::chrono::steady_clock::time_point &timestamp);

    int width()  const noexcept { return width_; }
    int height() const noexcept { return height_; }
    bool isOpened() const noexcept { return cap_.isOpened(); }

private:
    cv::VideoCapture cap_;
    double frame_period_ms_;
    bool loop_;
    int width_ = 0, height_ = 0;
    std::chrono::steady_clock::time_point next_t_;
};

}  // namespace autoaim::hardware

#endif  // AUTO_AIM_VIDEO_PLAYER_HPP_
