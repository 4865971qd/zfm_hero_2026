#ifndef AUTO_AIM_SERIAL_HPP_
#define AUTO_AIM_SERIAL_HPP_

#include <chrono>
#include <cstdint>
#include <string>

namespace autoaim::hardware {

// 串口接收数据 (从下位机)
struct SerialRx {
    uint8_t mode = 0;              // 0=idle, 1=auto_aim
    uint8_t bullet_speed = 0;
    float roll = 0, pitch = 0, yaw = 0;
};

// 串口发送数据 (到下位机)
struct SerialTx {
    uint8_t fire = 0;              // 0x01=开火, 0x00=不开火
    float pitch = 0;               // 度
    float yaw = 0;                 // 度
    float distance = 0;            // 米
};

// UART 16 字节定长协议 (zfm_ws1 格式)
// TX: [0xFF][fire:u8][pitch:f32][yaw:f32][distance:f32][CRC][0x0D]
// RX: [0xFF][mode:u8][roll:f32][pitch:f32][yaw:f32][bullet_speed:u8][0x0D]
class Serial {
public:
    Serial(const std::string &port, int baudrate = 115200);
    ~Serial();

    bool isOpen() const noexcept { return fd_ >= 0; }

    // 发送云台指令
    void send(const SerialTx &cmd);

    // 接收下位机数据（非阻塞，有数据返回 true）
    bool receive(SerialRx &data);

    // 获取最近一次接收时间
    std::chrono::steady_clock::time_point lastRxTime() const noexcept { return last_rx_; }

private:
    int fd_ = -1;
    std::chrono::steady_clock::time_point last_rx_;

    uint8_t calcCRC(const uint8_t *buf, int len) const noexcept;
};

}  // namespace autoaim::hardware

#endif  // AUTO_AIM_SERIAL_HPP_
