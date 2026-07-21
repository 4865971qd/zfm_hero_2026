#include "video_player.hpp"
#include "utils/logger.hpp"
#include <thread>

namespace autoaim::hardware {

VideoPlayer::VideoPlayer(const std::string &path, double fps, bool loop)
    : frame_period_ms_(1000.0 / fps), loop_(loop) {
    cap_.open(path);
    if (!cap_.isOpened()) {
        getLogger()->error("Cannot open video: {}", path);
        return;
    }
    width_  = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    next_t_ = std::chrono::steady_clock::now();
    getLogger()->info("VideoPlayer: {} ({}x{} @ {:.1f}fps)", path, width_, height_, fps);
}

bool VideoPlayer::read(cv::Mat &bgr_img, std::chrono::steady_clock::time_point &timestamp) {
    if (!cap_.isOpened()) return false;

    // 按帧率节流
    auto now = std::chrono::steady_clock::now();
    if (now < next_t_) {
        std::this_thread::sleep_for(next_t_ - now);
    }
    next_t_ = std::chrono::steady_clock::now() +
              std::chrono::microseconds(static_cast<int64_t>(frame_period_ms_ * 1000));

    cap_ >> bgr_img;
    if (bgr_img.empty()) {
        if (loop_) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            cap_ >> bgr_img;
            if (bgr_img.empty()) return false;
        } else {
            getLogger()->info("Video ended");
            return false;
        }
    }

    timestamp = std::chrono::steady_clock::now();
    return true;
}

}  // namespace autoaim::hardware
