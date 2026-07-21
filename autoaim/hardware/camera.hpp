#ifndef AUTO_AIM_CAMERA_HPP_
#define AUTO_AIM_CAMERA_HPP_

#if __has_include("MvCameraControl.h")
#include "MvCameraControl.h"
#define HAS_MVS 1
#else
#define HAS_MVS 0
#endif

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace autoaim::hardware {

#if HAS_MVS
// 海康 MVS 相机驱动 — 独立取图线程 + 双缓冲
class HikCamera {
public:
    HikCamera(int device_index = 0);
    ~HikCamera();

    void setExposureTime(double us);
    void setGain(double gain);
    void setFrameRate(double fps);

    // 返回最新已转换的帧（浅拷贝，不等待相机）
    bool read(cv::Mat &bgr_img, std::chrono::steady_clock::time_point &timestamp);

    int width()  const noexcept { return width_; }
    int height() const noexcept { return height_; }

private:
    bool initDevice();
    bool startGrabbing();
    void captureLoop();

    int device_index_;
    void *handle_ = nullptr;
    int width_ = 1440, height_ = 1080;
    bool grabbing_ = false;

    // 双缓冲
    std::vector<uint8_t> buf_[2];
    int front_ = 0;            // 写缓冲索引（取图线程写入）
    std::atomic<int> back_{-1}; // 读缓冲索引（-1=无帧）
    std::atomic<uint64_t> frame_seq_{0}; // 帧序号（取图线程递增）
    std::chrono::steady_clock::time_point last_ts_;

    // 线程同步
    std::thread capture_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_{false};
};
#else
// 桩: MVS SDK 不可用
class HikCamera {
public:
    HikCamera(int = 0) {}
    ~HikCamera() = default;
    void setExposureTime(double) {}
    void setGain(double) {}
    void setFrameRate(double) {}
    bool read(cv::Mat &, std::chrono::steady_clock::time_point &) { return false; }
    int width()  const noexcept { return 0; }
    int height() const noexcept { return 0; }
};
#endif

enum class CameraType { Hikvision, Video };

}  // namespace autoaim::hardware

#endif  // AUTO_AIM_CAMERA_HPP_
