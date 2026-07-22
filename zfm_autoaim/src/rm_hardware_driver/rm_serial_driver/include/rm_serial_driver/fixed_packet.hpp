#ifndef SERIAL_DRIVER_FIXED_PACKET_HPP_
#define SERIAL_DRIVER_FIXED_PACKET_HPP_

// std
#include <cstring>
#include <memory>

namespace zfm::serial_driver {

// 定长数据包封装
// [head_byte(0xff),...(data_bytes)...,check_byte,tail_byte(0x0d)]
template <int capacity = 16>
class FixedPacket {
 public:
  using SharedPtr = std::shared_ptr<FixedPacket>;
  FixedPacket() {
    memset(buffer_, 0, capacity);
    buffer_[0] = 0xff;             // 帧头
    buffer_[capacity - 1] = 0x0d;  // 帧尾
  }

 public:
  // 清除缓存, date_bytes和check_byte都用0填充
  void clear() { memset(buffer_ + 1, 0, capacity - 2); }
  // 设置flag
  void setCheckByte(uint8_t check_byte) {
    buffer_[capacity - 2] = check_byte;
  }
  // copy数据到缓存buffer
  void copyFrom(const void* src) { memcpy(buffer_, src, capacity); }
  // 获取缓存buffer
  const uint8_t* buffer() const { return buffer_; }

  // 自定义装载数据
  template <typename T, int data_len = sizeof(T)>
  bool loadData(T const &data, int index) {
    // 越界检测
    if (index > 0 && ((index + data_len) < (capacity))) {
      memcpy(buffer_ + index, &data, data_len);
      return true;
    }
    return false;
  }

  // 自定义解析数据
  template <typename T, int data_len = sizeof(T)>
  bool unloadData(T &data, int index) {
    // 越界检测
    if (index > 0 && ((index + data_len) < (capacity))) {
      memcpy(&data, buffer_ + index, data_len);
      return true;
    }
    return false;
  }

 private:
  // 数据包缓存buffer
  uint8_t buffer_[capacity];  // NOLINT
};

using FixedPacket16 = FixedPacket<16>;
using FixedPacket32 = FixedPacket<32>;
using FixedPacket64 = FixedPacket<64>;

}  // namespace zfm::serial_driver

#endif  // SERIAL_DRIVER_FIXED_PACKET_HPP_
