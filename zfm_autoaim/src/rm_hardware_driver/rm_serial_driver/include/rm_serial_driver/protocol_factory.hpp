#ifndef SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
#define SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_

#include <memory>
#include <string_view>

#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol/default_protocol.hpp"


namespace zfm::serial_driver {

class ProtocolFactory {
public:
  ProtocolFactory() = delete;
  // Factory method to create a protocol
  static std::unique_ptr<protocol::Protocol> createProtocol(std::string_view protocol_type,
                                                            std::string_view port_name,
                                                            bool enable_data_print) {
    if (protocol_type == "hero") {
      return std::make_unique<protocol::DefaultProtocol>(port_name, enable_data_print);
    }
    return nullptr;
  }
};

};      // namespace zfm::serial_driver
#endif  // SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
