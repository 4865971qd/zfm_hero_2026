#ifndef SERIAL_DRIVER_PROTOCOL_HPP_
#define SERIAL_DRIVER_PROTOCOL_HPP_

// std
#include <memory>
#include <string>
#include <string_view>
// ros2
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
// project
#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"
#include "rm_serial_driver/uart_transporter.hpp"

namespace zfm::serial_driver {
namespace protocol {
typedef enum : unsigned char { Fire = 0x01, NotFire = 0x00 } FireState;

// Protocol interface
class Protocol {
public:
  virtual ~Protocol() = default;

  // Send gimbal command
  virtual void send(const rm_interfaces::msg::GimbalCmd &data) = 0;

  // Receive data from serial port
  virtual bool receive(rm_interfaces::msg::SerialReceiveData &data) = 0;

  // Create subscriptions for SerialDriverNode
  virtual std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(
    rclcpp::Node::SharedPtr node) = 0;

  // Cretate setMode client for SerialDriverNode
  virtual std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const = 0;

  virtual std::string getErrorMessage() = 0;

private:
};

}  // namespace protocol
}  // namespace zfm::serial_driver
#endif  // SERIAL_DRIVER_PROTOCOLS_HPP_
