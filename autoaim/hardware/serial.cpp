#include "serial.hpp"

#include "utils/logger.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iterator>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace autoaim::hardware {

namespace {

double interpolateAngleDegrees(double a, double b, double k) {
    double delta = std::fmod(b - a + 180.0, 360.0);
    if (delta < 0.0) delta += 360.0;
    delta -= 180.0;
    return a + k * delta;
}

}  // namespace

Serial::Serial(const std::string &port, int baudrate, double imu_time_offset_ms)
    : imu_time_offset_ms_(imu_time_offset_ms) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        getLogger()->error("Cannot open serial port: {}", port);
        return;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    tcgetattr(fd_, &tty);

    speed_t baud = B115200;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default: break;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);
    tcsetattr(fd_, TCSANOW, &tty);

    getLogger()->info("Serial opened: {} @ {} 8N1", port, baudrate);
    rx_thread_ = std::thread(&Serial::receiveLoop, this);
}

Serial::~Serial() {
    rx_quit_ = true;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) ::close(fd_);
}

uint8_t Serial::calcCRC(const uint8_t *buf, int len) const noexcept {
    uint8_t crc = 0;
    for (int i = 0; i < len; ++i) crc ^= buf[i];
    return crc;
}

void Serial::send(const SerialTx &cmd) {
    if (fd_ < 0) return;

    uint8_t buf[16];
    std::memset(buf, 0, sizeof(buf));
    buf[0] = 0xFF;
    buf[1] = cmd.fire;
    std::memcpy(buf + 2, &cmd.pitch, 4);
    std::memcpy(buf + 6, &cmd.yaw, 4);
    std::memcpy(buf + 10, &cmd.distance, 4);
    buf[14] = calcCRC(buf, 14);
    buf[15] = 0x0D;

    [[maybe_unused]] auto ignored = ::write(fd_, buf, sizeof(buf));
}

void Serial::pushImuFrame(
    const uint8_t *buf, std::chrono::steady_clock::time_point timestamp) {
    SerialRx data;
    data.mode = buf[1];
    std::memcpy(&data.roll, buf + 2, 4);
    std::memcpy(&data.pitch, buf + 6, 4);
    std::memcpy(&data.yaw, buf + 10, 4);
    data.bullet_speed = buf[14];

    if (!std::isfinite(data.roll) || !std::isfinite(data.pitch) ||
        !std::isfinite(data.yaw)) {
        getLogger()->warn("[Serial][IMU] Ignoring non-finite frame");
        return;
    }

    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back({data, timestamp});
    while (imu_buffer_.size() > kMaxImuSamples) imu_buffer_.pop_front();
    last_rx_ = timestamp;
}

void Serial::receiveLoop() {
    std::vector<uint8_t> pending;
    pending.reserve(64);

    while (!rx_quit_) {
        if (fd_ < 0) break;

        uint8_t chunk[128];
        const int n = static_cast<int>(::read(fd_, chunk, sizeof(chunk)));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto timestamp = std::chrono::steady_clock::now();
        pending.insert(pending.end(), chunk, chunk + n);

        while (pending.size() >= 16) {
            auto header = std::find(pending.begin(), pending.end(), uint8_t{0xFF});
            if (header == pending.end()) {
                pending.clear();
                break;
            }
            pending.erase(pending.begin(), header);
            if (pending.size() < 16) break;

            if (pending[15] != 0x0D) {
                pending.erase(pending.begin());
                continue;
            }

            pushImuFrame(pending.data(), timestamp);
            pending.erase(pending.begin(), pending.begin() + 16);
        }
    }
}

bool Serial::imuAt(
    std::chrono::steady_clock::time_point timestamp,
    SerialRx &data,
    ImuSyncInfo *info) const {
    ImuSyncInfo local_info;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    local_info.buffer_size = imu_buffer_.size();
    if (imu_buffer_.empty()) {
        if (info) *info = local_info;
        return false;
    }

    const auto query_timestamp = timestamp -
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(imu_time_offset_ms_ / 1000.0));
    const auto &first = imu_buffer_.front();
    const auto &last = imu_buffer_.back();
    const auto max_gap = std::chrono::duration<double>(kMaxSyncGapSeconds);

    if (query_timestamp <= first.timestamp) {
        const double gap =
            std::chrono::duration<double>(first.timestamp - query_timestamp).count();
        if (first.timestamp - query_timestamp > max_gap) {
            if (info) *info = local_info;
            return false;
        }
        data = first.data;
        local_info.clamped = true;
        local_info.query_error_ms = gap * 1000.0;
    } else if (query_timestamp >= last.timestamp) {
        const double gap =
            std::chrono::duration<double>(query_timestamp - last.timestamp).count();
        if (query_timestamp - last.timestamp > max_gap) {
            if (info) *info = local_info;
            return false;
        }
        data = last.data;
        local_info.clamped = true;
        local_info.query_error_ms = -gap * 1000.0;
    } else {
        auto after = std::upper_bound(
            imu_buffer_.begin(), imu_buffer_.end(), query_timestamp,
            [](const auto &t, const ImuSample &sample) {
                return t < sample.timestamp;
            });
        const auto before = std::prev(after);
        const double span =
            std::chrono::duration<double>(after->timestamp - before->timestamp).count();
        const double elapsed =
            std::chrono::duration<double>(query_timestamp - before->timestamp).count();
        const double k = span > 0.0 ? std::clamp(elapsed / span, 0.0, 1.0) : 0.0;

        data = before->data;
        data.roll = static_cast<float>(
            interpolateAngleDegrees(before->data.roll, after->data.roll, k));
        data.pitch = static_cast<float>(
            interpolateAngleDegrees(before->data.pitch, after->data.pitch, k));
        data.yaw = static_cast<float>(
            interpolateAngleDegrees(before->data.yaw, after->data.yaw, k));
        local_info.interpolated = true;
        local_info.bracket_span_ms = span * 1000.0;
    }

    local_info.valid = true;
    if (info) *info = local_info;
    return true;
}

std::chrono::steady_clock::time_point Serial::lastRxTime() const {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return last_rx_;
}

}  // namespace autoaim::hardware
