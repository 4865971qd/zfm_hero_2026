#ifndef AUTO_AIM_SERIAL_HPP_
#define AUTO_AIM_SERIAL_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace autoaim::hardware {

struct SerialRx {
    uint8_t mode = 0;
    uint8_t bullet_speed = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
};

struct SerialTx {
    uint8_t fire = 0;
    float pitch = 0;
    float yaw = 0;
    float distance = 0;
};

// UART 16-byte fixed protocol.
// TX: [0xFF][fire:u8][pitch:f32][yaw:f32][distance:f32][CRC][0x0D]
// RX: [0xFF][mode:u8][roll:f32][pitch:f32][yaw:f32][bullet_speed:u8][0x0D]
class Serial {
public:
    struct ImuSyncInfo {
        bool valid = false;
        bool interpolated = false;
        bool clamped = false;
        double query_error_ms = 0.0;
        double bracket_span_ms = 0.0;
        std::size_t buffer_size = 0;
    };

    Serial(const std::string &port,
           int baudrate = 115200,
           double imu_time_offset_ms = 0.0);
    ~Serial();

    bool isOpen() const noexcept { return fd_ >= 0; }
    void send(const SerialTx &cmd);

    // Return the pose corresponding to the requested host monotonic timestamp.
    bool imuAt(std::chrono::steady_clock::time_point timestamp,
               SerialRx &data,
               ImuSyncInfo *info = nullptr) const;

    std::chrono::steady_clock::time_point lastRxTime() const;

private:
    struct ImuSample {
        SerialRx data;
        std::chrono::steady_clock::time_point timestamp;
    };

    static constexpr std::size_t kMaxImuSamples = 200;
    static constexpr double kMaxSyncGapSeconds = 0.020;

    int fd_ = -1;
    double imu_time_offset_ms_ = 0.0;
    std::atomic<bool> rx_quit_{false};
    std::thread rx_thread_;
    mutable std::mutex imu_mutex_;
    std::deque<ImuSample> imu_buffer_;
    std::chrono::steady_clock::time_point last_rx_{};

    uint8_t calcCRC(const uint8_t *buf, int len) const noexcept;
    void receiveLoop();
    void pushImuFrame(const uint8_t *buf,
                      std::chrono::steady_clock::time_point timestamp);
};

}  // namespace autoaim::hardware

#endif  // AUTO_AIM_SERIAL_HPP_
