#!/usr/bin/env python3
"""
MCU 串口桥接 — 独占 /dev/ttyACM0，创建虚拟串口给自瞄项目用。

功能:
  1. 打开真实串口 /dev/ttyACM0 @115200 8N1
  2. 创建 PTY 对，slave 端 symlink 到 /tmp/ttyACM_mcu
  3. 双向转发: MCU ↔ 自瞄（透明透传，不丢字节）
  4. 解析 MCU→PC 的 16 字节定长包，提取 mode 字节写入 /tmp/mcu_mode
  5. 24小时运行，不受 ROS 崩溃影响

运行方式:
  python3 mcu_bridge.py
  或 systemd 开机自启
"""
import os
import sys
import time
import struct
import select
import signal
import threading
import termios
import tty

# ============================================================================
# 配置
# ============================================================================
REAL_SERIAL = "/dev/ttyACM0"
BAUDRATE = 115200
VIRTUAL_LINK = "/tmp/ttyACM_mcu"
MODE_FILE = "/tmp/mcu_mode"

# 串口协议常量（与 autoaim FixedPacket<16> 一致）
PACKET_SIZE = 16
PACKET_HEADER = 0xFF
PACKET_TAIL = 0x0D
MODE_OFFSET = 14  # mode_flag 字节在包里的位置（系统级模式切换）

# ============================================================================
# 串口初始化
# ============================================================================

def open_real_serial(path: str, baudrate: int):
    """打开真实串口，配置为 raw 模式 8N1"""
    import serial as pyserial

    ser = pyserial.Serial(
        port=path,
        baudrate=baudrate,
        bytesize=pyserial.EIGHTBITS,
        parity=pyserial.PARITY_NONE,
        stopbits=pyserial.STOPBITS_ONE,
        timeout=0.01,  # 10ms 超时，保证 select 不会卡死
    )
    # 清空缓冲区
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def create_pty():
    """创建 PTY 对，返回 (master_fd, slave_path)"""
    import pty as _pty
    master_fd, slave_fd = _pty.openpty()
    # 设置 slave 为 raw 模式（和串口行为一致）
    attr = termios.tcgetattr(slave_fd)
    attr.c_iflag &= ~(termios.BRGINT | termios.ICRNL | termios.INPCK | termios.ISTRIP | termios.IXON)
    attr.c_oflag &= ~(termios.OPOST)
    attr.c_cflag |= (termios.CS8)
    attr.c_lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    attr.c_cc[termios.VMIN] = 1
    attr.c_cc[termios.VTIME] = 0
    termios.tcsetattr(slave_fd, termios.TCSANOW, attr)
    os.close(slave_fd)  # 我们只用 master_fd 来读写，slave 由自瞄打开

    slave_path = os.ttyname(master_fd).replace("/ptm", "/pts")
    # pty.openpty 返回的 slave fd 路径需要在 /dev/pts/ 里找
    import fcntl
    import pty as _pty2
    # 更可靠的方式：用 ptsname
    slave_name = os.ttyname(slave_fd) if 'slave_fd' in dir() else None

    return master_fd, slave_path


def create_pty_v2():
    """创建 PTY 对，返回 (master_fd, slave_path) — 更可靠的方法"""
    import pty
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    os.close(slave_fd)
    return master_fd, slave_path


# ============================================================================
# 协议解析
# ============================================================================

class PacketParser:
    """从字节流中提取 16 字节定长包，只解析 mode"""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> int | None:
        """喂入字节，返回最新解析出的 mode 值（一次处理完所有积压的包），或 None"""
        self._buf.extend(data)
        result = None

        while len(self._buf) >= PACKET_SIZE:
            # 查找帧头 0xFF
            head_idx = self._buf.find(PACKET_HEADER)
            if head_idx < 0:
                self._buf.clear()
                return result
            if head_idx > 0:
                del self._buf[:head_idx]

            if len(self._buf) < PACKET_SIZE:
                return result

            # 验证帧尾 0x0D
            if self._buf[PACKET_SIZE - 1] != PACKET_TAIL:
                # 帧尾不对，跳过这一字节重新找头
                del self._buf[0]
                continue

            # 提取 mode，继续循环以处理所有积压包，返回最新的 mode
            result = self._buf[MODE_OFFSET]
            del self._buf[:PACKET_SIZE]

        return result


# ============================================================================
# 主循环
# ============================================================================

def write_mode_file(mode: int):
    """原子写入 mode 文件"""
    try:
        with open(MODE_FILE + ".tmp", "w") as f:
            f.write(str(mode))
        os.rename(MODE_FILE + ".tmp", MODE_FILE)
    except OSError:
        pass


def main():
    print(f"[mcu_bridge] Starting...")
    print(f"[mcu_bridge] Real serial: {REAL_SERIAL} @ {BAUDRATE}")
    print(f"[mcu_bridge] Virtual link: {VIRTUAL_LINK}")

    # 打开真实串口
    ser = open_real_serial(REAL_SERIAL, BAUDRATE)
    print(f"[mcu_bridge] Opened {REAL_SERIAL}")

    # 创建 PTY 对
    master_fd, slave_path = create_pty_v2()
    print(f"[mcu_bridge] PTY created: master_fd={master_fd}, slave={slave_path}")

    # 创建 /tmp/ttyACM_mcu 符号链接
    if os.path.exists(VIRTUAL_LINK):
        os.unlink(VIRTUAL_LINK)
    os.symlink(slave_path, VIRTUAL_LINK)
    print(f"[mcu_bridge] Symlink: {VIRTUAL_LINK} -> {slave_path}")

    # 写入初始 mode = 0
    write_mode_file(0)

    parser = PacketParser()
    running = True

    def shutdown(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    last_mode = 0
    total_bytes_m2p = 0  # MCU → PC
    total_bytes_p2m = 0  # PC → MCU

    print(f"[mcu_bridge] Running. Forwarding MCU <-> {VIRTUAL_LINK}")

    while running:
        try:
            # select 同时监听两个方向
            rlist = [ser.fileno(), master_fd]
            readable, _, _ = select.select(rlist, [], [], 0.1)

            for fd in readable:
                if fd == ser.fileno():
                    # MCU → PTY
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        total_bytes_m2p += len(data)
                        # 解析 mode（必须在 PTY write 之前，否则 PTY write 失败时 parser 被跳过）
                        mode = parser.feed(data)
                        if mode is not None and mode != last_mode:
                            print(f"[mcu_bridge] Mode: {last_mode} -> {mode}")
                            write_mode_file(mode)
                            last_mode = mode
                        # 转发到 PTY（没人读时可能失败，不影响 mode 解析）
                        try:
                            os.write(master_fd, data)
                        except OSError:
                            pass

                elif fd == master_fd:
                    # PTY → MCU
                    data = os.read(master_fd, 4096)
                    if data:
                        total_bytes_p2m += len(data)
                        ser.write(data)
                        ser.flush()

        except OSError as e:
            print(f"[mcu_bridge] IO error: {e}", file=sys.stderr)
            time.sleep(0.5)
        except Exception as e:
            print(f"[mcu_bridge] Unexpected error: {e}", file=sys.stderr)

    # 清理
    ser.close()
    os.close(master_fd)
    if os.path.exists(VIRTUAL_LINK):
        os.unlink(VIRTUAL_LINK)
    print(f"[mcu_bridge] Stopped. MCU→PC: {total_bytes_m2p}B, PC→MCU: {total_bytes_p2m}B")


if __name__ == "__main__":
    main()
