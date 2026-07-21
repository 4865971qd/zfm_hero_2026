#include "serial.hpp"
#include "utils/logger.hpp"

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace autoaim::hardware {

Serial::Serial(const std::string &port, int baudrate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        getLogger()->error("Cannot open serial port: {}", port);
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd_, &tty);

    // 波特率
    speed_t baud = B115200;
    switch (baudrate) {
        case 9600:   baud = B9600;   break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        case 921600: baud = B921600; break;
        default:     baud = B115200; break;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // 8N1
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;  // 非阻塞

    tcflush(fd_, TCIOFLUSH);
    tcsetattr(fd_, TCSANOW, &tty);

    getLogger()->info("Serial opened: {} @ {} 8N1", port, baudrate);
}

Serial::~Serial() {
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
    memset(buf, 0, 16);
    buf[0] = 0xFF;                             // 帧头
    buf[1] = cmd.fire;                         // fire
    memcpy(buf + 2,  &cmd.pitch,    4);        // pitch (float32)
    memcpy(buf + 6,  &cmd.yaw,      4);        // yaw (float32)
    memcpy(buf + 10, &cmd.distance, 4);        // distance (float32)
    buf[14] = calcCRC(buf, 14);               // CRC
    buf[15] = 0x0D;                            // 帧尾

    [[maybe_unused]] auto _ = ::write(fd_, buf, 16);
}

bool Serial::receive(SerialRx &data) {
    if (fd_ < 0) return false;

    uint8_t buf[16];
    int n = ::read(fd_, buf, 16);
    if (n < 16) return false;

    // 校验帧头帧尾 (RX 包无 CRC)
    if (buf[0] != 0xFF || buf[15] != 0x0D) return false;

    data.mode              = buf[1];
    memcpy(&data.roll,  buf + 2,  4);
    memcpy(&data.pitch, buf + 6,  4);
    memcpy(&data.yaw,   buf + 10, 4);
    data.bullet_speed = buf[14];

    last_rx_ = std::chrono::steady_clock::now();
    return true;
}

}  // namespace autoaim::hardware
