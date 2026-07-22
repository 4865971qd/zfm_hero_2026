#ifndef DOORLOCK_SNIPER_VIDEO_ENCODER_NODE_HPP_
#define DOORLOCK_SNIPER_VIDEO_ENCODER_NODE_HPP_

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <deque>

#include "doorlock_sniper/msg/video_packet.hpp"

namespace doorlock_sniper
{

class VideoEncoderNode : public rclcpp::Node
{
public:
  explicit VideoEncoderNode(const rclcpp::NodeOptions & options);
  ~VideoEncoderNode() override;

private:
  void initialize_gstreamer();
  void shutdown_gstreamer();
  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  cv::Mat preprocess_image(
    const cv::Mat & input,
    cv::Mat * roi_downsample = nullptr,
    cv::Mat * static_removed = nullptr);
  
  void push_frame_to_gstreamer(const cv::Mat & frame);
  void pull_stream_and_packetize();
  
  void display_loop();

  GstElement * pipeline_;
  GstElement * appsrc_;
  GstElement * appsink_;
  GstBus * bus_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<doorlock_sniper::msg::VideoPacket>::SharedPtr packet_pub_;
  
  // Runtime counters and state.
  uint64_t packet_sequence_id_ = 0;
  uint32_t frame_count_ = 0;
  
  // Display thread.
  std::thread display_thread_;
  std::atomic<bool> display_running_;
  std::mutex frame_mutex_;
  cv::Mat display_raw_frame_;
  cv::Mat display_roi_frame_;
  cv::Mat display_static_frame_;
  cv::Mat display_frame_;
  
  // Stream packetization buffer.
  std::vector<uint8_t> stream_buffer_;
  std::deque<std::pair<int64_t, size_t>> sent_window_;
  size_t sent_window_bytes_ = 0;
  uint64_t dropped_bytes_ = 0;
  uint32_t dropped_events_ = 0;
  int64_t last_telemetry_ns_ = 0;
  std::mutex buffer_mutex_;

  // Frame-rate control.
  int64_t last_encode_stamp_ns_ = 0;
  uint64_t display_frame_counter_ = 0;
  cv::Mat background_gray_f32_;
  cv::Mat last_gray_u8_;
  cv::Mat center_weight_bgr_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;
  int shake_recover_frames_left_ = 0;
  cv::Size center_weight_cached_size_;
  int center_weight_cached_clear_size_ = -1;
  
  // Parameters.
  int param_crop_size_ = 800;
  int param_output_size_ = 400;
  int param_output_fps_ = 60;
  int param_target_bitrate_ = 40;
  int param_packet_size_ = 150;
  bool param_static_simplify_ = true;
  int param_motion_threshold_ = 14;
  int param_motion_erode_px_ = 1;
  int param_motion_dilate_px_ = 2;
  int param_motion_trail_frames_ = 3;
  double param_trail_disable_motion_ratio_ = 0.30;
  bool param_shake_compensation_enable_ = true;
  double param_shake_min_response_ = 0.12;
  double param_shake_fullframe_ratio_ = 0.60;
  int param_shake_residual_threshold_ = 35;
  int param_shake_white_v_threshold_ = 150;
  int param_shake_white_s_max_ = 100;
  int param_shake_green_g_threshold_ = 100;
  int param_shake_green_margin_ = 20;
  int param_shake_recover_frames_ = 8;
  double param_bg_update_alpha_ = 0.01;
  double param_bg_blur_sigma_ = 1.2;
  int param_center_clear_size_ = 100;
  bool param_armor_edge_enable_ = true;
  int param_armor_edge_roi_size_ = 180;
  int param_armor_dark_threshold_ = 28;
  int param_armor_edge_threshold_ = 45;
  int param_armor_edge_dilate_px_ = 1;
  bool param_force_monochrome_ = false;
  double param_bandwidth_limit_kbytes_ = 7.0;
  double param_bandwidth_window_s_ = 2.0;
  double param_max_tx_delay_s_ = 1.0;
  bool param_enable_display_ = true;
  bool param_debug_dump_enable_ = false;
  int param_debug_dump_every_n_frames_ = 20;
  bool param_debug_dump_save_raw_ = true;
  bool param_debug_dump_save_roi_ = true;
  bool param_debug_dump_save_static_ = true;
  bool param_debug_dump_save_final_ = true;
  std::string param_input_topic_;
  std::string param_x264_preset_ = "auto";
  std::string param_debug_dump_dir_ = "sniper_debug_imgs";
};

} // namespace doorlock_sniper

#endif
