#include "rm_utils/heartbeat.hpp"

namespace zfm {
HeartBeatPublisher::SharedPtr HeartBeatPublisher::create(rclcpp::Node *node) {
  return std::shared_ptr<HeartBeatPublisher>(new HeartBeatPublisher(node));
}

HeartBeatPublisher::HeartBeatPublisher(rclcpp::Node *node) {
  // Initialize message
  message_.data = 0;
  // Create publisher
  std::string node_name = node->get_name();
  std::string topic_name = node_name + "/heartbeat";
  publisher_ = node->create_publisher<std_msgs::msg::Int64>(topic_name, 1);
  // Start publishing thread
  pub_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {
      message_.data++;
      publisher_->publish(message_);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
}

HeartBeatPublisher::~HeartBeatPublisher() {
  if (pub_thread_.joinable()) {
    pub_thread_.join();
  }
}

}  // namespace zfm
