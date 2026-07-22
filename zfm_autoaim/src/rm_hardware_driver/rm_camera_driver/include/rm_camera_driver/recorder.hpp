#ifndef RM_CAMERA_DRIVER_RECORDER_HPP_
#define RM_CAMERA_DRIVER_RECORDER_HPP_

// std
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>
// OpenCV
#include <opencv2/videoio.hpp>

namespace zfm::camera_driver {
class Recorder {
public:
  using Frame = std::vector<unsigned char>;
  Recorder(const std::filesystem::path &file, int fps, cv::Size size,
           int fourcc = 0, int queue_size = 300);
  ~Recorder();

  void addFrame(const Frame &frame);
  bool start();
  void stop();

  std::filesystem::path path;

private:
  void recorderThread();

  cv::Size size_;
  int fps_;
  int fourcc_;
  int max_queue_size_;
  cv::VideoWriter writer_;

  std::deque<Frame> frame_queue_;

  std::mutex mutex_;
  std::atomic<bool> recoring_;
  std::condition_variable cv_;
  std::thread recorder_thread_;

  uint64_t total_in_ = 0;
  uint64_t total_out_ = 0;
};
}  // namespace zfm::camera_driver
#endif  // RM_CAMERA_DRIVER_RECORDER_HPP_
