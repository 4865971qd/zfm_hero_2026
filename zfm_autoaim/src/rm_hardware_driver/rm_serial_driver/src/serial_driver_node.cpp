#include "rm_serial_driver/serial_driver_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
// std
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
// ros2
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_serial_driver/uart_transporter.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace zfm::serial_driver {
SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions &options)
: Node("serial_driver", options) {
  ZFM_REGISTER_LOGGER("serial_driver", "~/zfm2026-log", INFO);

  // Task thread
  listen_thread_ = std::make_unique<std::thread>(&SerialDriverNode::listenLoop, this);
}

void SerialDriverNode::init() {
  ZFM_INFO("serial_driver", "Initializing SerialDriverNode!");
  // Init
  target_frame_ = this->declare_parameter("target_frame", "odom");
  std::string port_name = this->declare_parameter("port_name", "/dev/ttyACM0");
  std::string protocol_type = this->declare_parameter("protocol", "hero");
  bool enable_data_print = this->declare_parameter("enable_data_print", false);
  // Create Protocol
  protocol_ = ProtocolFactory::createProtocol(protocol_type, port_name, enable_data_print);
  if (protocol_ == nullptr) {
    ZFM_FATAL("serial_driver", "Failed to create protocol with type: {}", protocol_type);
    rclcpp::shutdown();
    return;
  }
  ZFM_INFO(
    "serial_driver", "Protocol has been created with type: {}, port: {}", protocol_type, port_name);

  // Subscriptions
  subscriptions_ = protocol_->getSubscriptions(this->shared_from_this());
  for (auto sub : subscriptions_) {
    ZFM_INFO("serial_driver", "Subscribe to topic: {}", sub->get_topic_name());
  }
  // Publisher
  serial_receive_data_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", rclcpp::SensorDataQoS());

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Param client
  for (auto client : protocol_->getClients(this->shared_from_this())) {
    std::string name = client->get_service_name();
    set_mode_clients_.emplace(name, client);
    ZFM_INFO("serial_driver", "Create client for service: {}", name);
  }

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);

  ZFM_INFO("serial_driver", "SerialDriverNode has been initialized!");
}

SerialDriverNode::~SerialDriverNode() {
  ZFM_INFO("serial_driver", "Destroy SerialDriverNode!");
  rclcpp::shutdown();
  if (listen_thread_ != nullptr) {
    listen_thread_->join();
  }
}

void SerialDriverNode::listenLoop() {
  if (protocol_ == nullptr) {
    // Lazy init because shared_from_this() is not available in constructor
    init();
  }

  rm_interfaces::msg::SerialReceiveData receive_data;
  while (rclcpp::ok()) {
    if (protocol_->receive(receive_data)) {
      receive_data.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      receive_data.header.frame_id = target_frame_;
      serial_receive_data_pub_->publish(receive_data);

      for (auto &[service_name, client] : set_mode_clients_) {
        if (client.mode.load() != receive_data.mode && !client.on_waiting.load()) {
          setMode(client, receive_data.mode);
        }
      }

      // 创建一次要发布的TF消息结构体（世界->云台）
      geometry_msgs::msg::TransformStamped t;
      timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
      // 时间戳使用上位机当前时间加可调的时间偏置，保证与图像/雷达等传感器时间对齐
      t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      // 父坐标系（世界/里程计系），从参数 target_frame_ 读取，通常为 "odom"
      t.header.frame_id = target_frame_;
      // 子坐标系为云台坐标系
      t.child_frame_id = "gimbal_link";
      // 下位机上报的欧拉角单位为度，这里转换为弧度
      auto roll = receive_data.roll * M_PI / 180.0;
      // pitch 在协议中定义与右手系相反，这里取负号对齐TF的右手坐标
      auto pitch = -receive_data.pitch * M_PI / 180.0;
      auto yaw = receive_data.yaw * M_PI / 180.0;
      // 将RPY欧拉角转换为四元数
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      // 写入旋转部分，位移默认为0（云台原点与世界原点重合或通过其他模块给出）
      t.transform.rotation = tf2::toMsg(q);
      // 发送 TF: odom -> gimbal_link
      tf_broadcaster_->sendTransform(t);

      // odom_rectify: 额外发布一个只包含roll的中间坐标系，便于上层进行俯仰/偏航解算
      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy = utils::getRPY(q_eigen.toRotationMatrix());
      q.setRPY(rpy[0], 0, 0);
      t.header.frame_id = target_frame_;
      t.child_frame_id = target_frame_ + "_rectify";
      tf_broadcaster_->sendTransform(t);
      
    } else {
      auto error_message = protocol_->getErrorMessage();
      error_message = error_message.empty() ? "unknown" : error_message;
      ZFM_WARN("serial_driver", "Failed to reveive packet! error message :{}", error_message);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void SerialDriverNode::setMode(SetModeClient &client, const uint8_t mode) {
  using namespace std::chrono_literals;

  std::string service_name = client.ptr->get_service_name();

  if (!client.ptr->wait_for_service(0s)) {
    // Service not ready yet, will retry on next receive cycle
    static int log_cnt = 0;
    if (++log_cnt % 30 == 0) {
      ZFM_INFO("serial_driver", "Service {} not available, will retry...", service_name);
    }
    return;
  }

  auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
  req->mode = mode;

  client.on_waiting.store(true);
  auto result = client.ptr->async_send_request(
    req, [mode, &client, service_name, this](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
      client.on_waiting.store(false);
      auto response = result.get();
      if (response->success) {
        client.mode.store(mode);
        RCLCPP_INFO(this->get_logger(), "Successfully set mode to %d for %s",
                   mode, service_name.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to set mode to %d for %s: %s",
                   mode, service_name.c_str(), response->message.c_str());
      }
    });
}

}  // namespace zfm::serial_driver

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(zfm::serial_driver::SerialDriverNode)
