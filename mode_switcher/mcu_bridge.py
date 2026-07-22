#!/usr/bin/env python3
"""
MCU 串口桥接 — 读取 MCU 数据，提取 mode，双向转发（非阻塞写）。
"""
import os, time, select, pty, glob, sys, fcntl

BAUDRATE = 115200
VIRTUAL_LINK = "/tmp/ttyACM_mcu"
MODE_FILE = "/tmp/mcu_mode"
PACKET_SIZE = 16
PACKET_HEADER = 0xFF
PACKET_TAIL = 0x0D
MODE_OFFSET = 14
RX_REPORT_GATE = 0


def log(msg):
    print(f"[bridge] {msg}", flush=True)


def setup_pty():
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    if os.path.lexists(VIRTUAL_LINK):
        os.unlink(VIRTUAL_LINK)
    os.symlink(slave_path, VIRTUAL_LINK)
    return master_fd, slave_fd


def write_mode(val):
    try:
        with open(MODE_FILE + ".tmp", "w") as f:
            f.write(str(val))
        os.rename(MODE_FILE + ".tmp", MODE_FILE)
    except Exception as e:
        log(f"WRITE FAIL: {e}")


class Parser:
    def __init__(self):
        self._buf = bytearray()
    def feed(self, data):
        self._buf.extend(data)
        r = None
        while len(self._buf) >= PACKET_SIZE:
            h = self._buf.find(PACKET_HEADER)
            if h < 0: self._buf.clear(); return r
            if h > 0: del self._buf[:h]
            if len(self._buf) < PACKET_SIZE: return r
            if self._buf[PACKET_SIZE-1] != PACKET_TAIL:
                del self._buf[0]; continue
            r = self._buf[MODE_OFFSET]
            del self._buf[:PACKET_SIZE]
        return r


def open_acm(dev):
    import serial
    try:
        ser = serial.Serial(dev, BAUDRATE, timeout=0.05)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # 设为非阻塞写，防止视频数据太多时卡住
        fl = fcntl.fcntl(ser.fd, fcntl.F_GETFL)
        fcntl.fcntl(ser.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        log(f"Opened {dev}")
        return ser
    except Exception as e:
        log(f"Failed {dev}: {e}")
        return None


def try_write(ser, data, label):
    """非阻塞写串口，失败静默忽略"""
    try:
        ser.write(data)
    except (BlockingIOError, OSError):
        pass
    except Exception:
        log(f"Write err {label}")


def main():
    log("Starting...")
    master_fd, slave_fd = setup_pty()
    fl = fcntl.fcntl(master_fd, fcntl.F_GETFL)
    fcntl.fcntl(master_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    write_mode(0)
    log(f"PTY: {VIRTUAL_LINK} ready")

    sers = {}
    parser = Parser()
    last_mode = 0
    last_hb = 0
    global RX_REPORT_GATE
    pty_tx = 0
    pty_rx = 0

    while True:
        now = time.time()
        if now - last_hb > 30:
            last_hb = now
            log(f"♥ ports={list(sers.keys())} mode={last_mode} tx={pty_tx} rx={pty_rx}")

        try:
            # 扫描 ACM
            for dev in sorted(glob.glob("/dev/ttyACM*")):
                if dev not in sers:
                    ser = open_acm(dev)
                    if ser:
                        sers[dev] = ser

            fds = []
            fd_map = {}
            for name, ser in list(sers.items()):
                try:
                    fd = ser.fileno()
                    fds.append(fd)
                    fd_map[fd] = (name, ser)
                except Exception:
                    sers.pop(name, None)

            # 也监控 PTY 主端
            fds.append(master_fd)

            if not sers:
                time.sleep(1)
                continue

            rd, _, _ = select.select(fds, [], [], 0.5)

            for fd in rd:
                if fd == master_fd:
                    # PTY → MCU（非阻塞转发）
                    try:
                        data = os.read(master_fd, 4096)
                        pty_rx += len(data)
                    except OSError:
                        # 从端没打开，睡一下让出时间给串口
                        time.sleep(0.05)
                        continue
                    if data:
                        for name, ser in list(sers.items()):
                            try_write(ser, data, name)

                elif fd in fd_map:
                    name, ser = fd_map[fd]
                    try:
                        data = ser.read(1024)
                    except Exception:
                        log(f"Lost {name}")
                        sers.pop(name)
                        continue
                    if data:
                        # RX 报告（每5秒）
                        t5 = int(time.time()) // 5
                        if t5 != RX_REPORT_GATE:
                            RX_REPORT_GATE = t5
                            log(f"RX {len(data)}B from {name}")
                        # 解析 mode
                        m = parser.feed(data)
                        if m is not None:
                            # 打印每次解析到的 mode（不限于变化时）
                            log(f"MODE={m} (last was {last_mode})")
                            if m != last_mode:
                                log(f"[{time.strftime('%H:%M:%S')}] Mode: {last_mode} -> {m}")
                            write_mode(m)
                            last_mode = m
                        else:
                            # 有数据但没找到有效包——打印首尾字节排查
                            if len(data) >= 16 and data[0] == PACKET_HEADER:
                                log(f"PKT: {data[:16].hex()}")
                        # 转发到 PTY
                        try:
                            pty_tx += len(data)
                            os.write(master_fd, data)
                        except OSError:
                            pass

        except Exception as e:
            log(f"Error: {e}")
            for ser in sers.values():
                try: ser.close()
                except: pass
            sers.clear()
            time.sleep(1)


if __name__ == "__main__":
    main()
