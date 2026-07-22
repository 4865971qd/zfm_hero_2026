#ifndef SERIAL_DRIVER_TRANSPORTER_INTERFACE_HPP_
#define SERIAL_DRIVER_TRANSPORTER_INTERFACE_HPP_

// std
#include <memory>
#include <string>

namespace zfm::serial_driver {

// Transporter device interface to transport data between embedded systems
// (stm32,c51) and PC
class TransporterInterface {
public:
  using SharedPtr = std::shared_ptr<TransporterInterface>;
  virtual ~TransporterInterface() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() = 0;
  // return recv len>0, return <0 if error
  virtual int read(void *buffer, size_t len) = 0;
  // return send len>0, return <0 if error
  virtual int write(const void *buffer, size_t len) = 0;
  // get error message when open() return false.
  virtual std::string errorMessage() = 0;
};

}  // namespace zfm::serial_driver

#endif  // SERIAL_DRIVER_TRANSPORTER_INTERFACE_HPP_
