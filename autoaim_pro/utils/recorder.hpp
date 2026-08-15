#ifndef AUTO_AIM_RECORDER_HPP_
#define AUTO_AIM_RECORDER_HPP_

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <thread>

namespace autoaim {

// 视频 + IMU 四元数同步录制
class Recorder {
public:
    Recorder(double fps = 110.0);
    ~Recorder();

    void record(const cv::Mat &img,
                const Eigen::Quaterniond &q,
                const std::chrono::steady_clock::time_point &t);

private:
    struct Frame {
        cv::Mat img;
        Eigen::Quaterniond q;
        std::chrono::steady_clock::time_point t;
    };

    void saveLoop();

    bool init_done_ = false;
    std::atomic<bool> stop_{false};
    double fps_;
    std::string txt_path_, vid_path_;
    std::ofstream txt_out_;
    cv::VideoWriter vid_out_;
    std::chrono::steady_clock::time_point start_t_, last_t_;
    std::queue<Frame> queue_;
    std::mutex q_mtx_;
    std::thread save_thread_;
    void init(const cv::Mat &img);
};

}  // namespace autoaim

#endif  // AUTO_AIM_RECORDER_HPP_
