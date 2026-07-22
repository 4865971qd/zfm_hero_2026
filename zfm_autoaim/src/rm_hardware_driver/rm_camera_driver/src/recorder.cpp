#include "rm_camera_driver/recorder.hpp"
// std
#include <filesystem>
#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/logger/log.hpp"

namespace zfm::camera_driver {
Recorder::Recorder(const std::filesystem::path &file, int fps, cv::Size size,
                   int fourcc, int queue_size)
: path(file), size_(size), fps_(fps), fourcc_(fourcc),
  max_queue_size_(queue_size), recoring_(false) {}

bool Recorder::start() {
  if (!std::filesystem::exists(path)) {
    std::filesystem::create_directories(path.parent_path());
  }

  if (!writer_.open(path.string(), fourcc_, fps_, size_, true)) {
    return false;
  }
  recoring_ = true;
  recorder_thread_ = std::thread(&Recorder::recorderThread, this);
  return true;
}

Recorder::~Recorder() { stop(); }

void Recorder::addFrame(const Frame &frame) {
  total_in_++;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_queue_.size() < static_cast<size_t>(max_queue_size_)) {
      frame_queue_.push_back(frame);
    } else if (total_in_ % 300 == 1) {
      std::cerr << "[Recorder] Queue full (" << max_queue_size_
                << "), dropping frames (in=" << total_in_
                << " out=" << total_out_ << ")" << std::endl;
    }
  }
  cv_.notify_one();
}

void Recorder::stop() {
  recoring_ = false;
  cv_.notify_all();
  recorder_thread_.join();
  writer_.release();
  /* if (total_in_ > total_out_) {
    std::cerr << "[Recorder] Final: captured=" << total_in_
              << " written=" << total_out_
              << " dropped=" << (total_in_ - total_out_) << std::endl;
  }
  */
}

void Recorder::recorderThread() {
  while (recoring_) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this] { return !frame_queue_.empty() || !recoring_; });
    if (!recoring_) {
      break;
    }
    auto buffer = std::move(frame_queue_.front());
    frame_queue_.pop_front();
    lock.unlock();

    if (!buffer.empty() && writer_.isOpened() && size_.area() > 0) {
      try {
        cv::Mat frame(size_, CV_8UC3, buffer.data());
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        writer_.write(frame);
        total_out_++;
      } catch (const cv::Exception &) {
      }
    }
  }

  // Drain remaining frames in queue
  std::lock_guard<std::mutex> lock(mutex_);
  while (!frame_queue_.empty()) {
    auto buffer = std::move(frame_queue_.front());
    frame_queue_.pop_front();
    if (!buffer.empty() && size_.area() > 0) {
      try {
        cv::Mat frame(size_, CV_8UC3, buffer.data());
        cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
        writer_.write(frame);
        total_out_++;
      } catch (const cv::Exception &) {
      }
    }
  }
}

}  // namespace zfm::camera_driver
