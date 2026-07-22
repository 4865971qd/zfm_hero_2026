"""
Serial bridge: VideoPacket (ROS) → 0x0310 serial frames for RM image transmission link.

Converts H264 byte stream from doorlock_sniper encoder into compact video format
(3B header + up to 297B payload), wraps in 0x0310 serial frames with CRC, and
transmits at the link rate (default 48 Hz) over UART to the VTX module.

Matches:
  - rm_compress SerialTx (CRC-8/CRC-16 tables, 0x0310 framing)
  - rm-native-viewer Video0310Chunk (compact video header)
"""
import struct
import glob
import time
import threading

import serial as pyserial
import rclpy
from rclpy.node import Node
from doorlock_sniper.msg import VideoPacket


# ============================================================================
# CRC tables — identical to rm_compress SerialTx
# ============================================================================

_CRC8_TABLE = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]


def _make_crc16_table():
    table = [0] * 256
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
        table[i] = crc
    return table


_CRC16_TABLE = _make_crc16_table()


def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc = _CRC8_TABLE[(crc ^ b) & 0xFF]
    return crc


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc


# ============================================================================
# 0x0310 Serial frame builder — matches rm_compress SerialTx
# ============================================================================

_SOF = 0xA5
_CMD_0310 = 0x0310
_MAX_0310_PAYLOAD = 300


def _build_0310_frame(payload: bytes, seq: int) -> bytes:
    """Build a 309-byte 0x0310 serial frame."""
    # Data field is always exactly 300 bytes (zero-padded)
    data_len = _MAX_0310_PAYLOAD
    padded = payload + b'\x00' * (data_len - len(payload))

    # CRC8 over SOF + data_len(2) + seq
    header_crc_input = bytes([_SOF, data_len & 0xFF, (data_len >> 8) & 0xFF, seq & 0xFF])
    header_crc = crc8(header_crc_input)

    # Build frame
    frame = bytearray()
    frame.append(_SOF)                                  # SOF
    frame.append(data_len & 0xFF)                       # data_len lo
    frame.append((data_len >> 8) & 0xFF)                # data_len hi
    frame.append(seq & 0xFF)                            # seq
    frame.append(header_crc)                            # header CRC8
    frame.append(_CMD_0310 & 0xFF)                      # cmd_id lo
    frame.append((_CMD_0310 >> 8) & 0xFF)               # cmd_id hi
    frame.extend(padded)                                # 300B data (zero-padded)
    frame_crc = crc16(bytes(frame))                     # CRC16 over header+cmd+data
    frame.append(frame_crc & 0xFF)
    frame.append((frame_crc >> 8) & 0xFF)

    return bytes(frame)


# ============================================================================
# Compact video header — matches rm-native-viewer Video0310Chunk
# ============================================================================

_VIDEO_HEADER_BYTES = 3
_VIDEO_MAX_PAYLOAD = 297  # 300 - 3B header


def _build_video_header(payload_len: int, seq: int, reset: bool = False) -> bytes:
    """Build 3-byte compact video header.

    byte[0] = flags | ((payload_len >> 8) & 1) << 1
    byte[1] = sequence
    byte[2] = payload_len & 0xFF
    """
    flags = 0
    if reset:
        flags |= 0x01  # VIDEO_0310_FLAG_RESET
    flags |= ((payload_len >> 8) & 0x01) << 1  # PAYLOAD_BYTES_MSB

    return bytes([flags, seq & 0xFF, payload_len & 0xFF])


# ============================================================================
# SerialBridgeNode
# ============================================================================

class SerialBridgeNode(Node):
    """Bridge encoded H264 VideoPackets to 0x0310 serial frames."""

    def __init__(self):
        super().__init__('serial_bridge')

        # Parameters
        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('send_hz', 48)
        self.declare_parameter('max_payload', _VIDEO_MAX_PAYLOAD)

        self._port_cfg = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._send_hz = self.get_parameter('send_hz').value
        self._max_payload = self.get_parameter('max_payload').value

        # H264 byte buffer (fed by VideoPacket callbacks)
        self._buffer = bytearray()
        self._buffer_lock = threading.Lock()

        # Video sequence (8-bit, wraps at 256)
        self._video_seq = 0
        self._first_chunk = True  # Set RESET flag on first chunk

        # 0x0310 serial sequence
        self._serial_seq = 0

        # Statistics
        self._bytes_buffered = 0
        self._chunks_sent = 0
        self._overruns = 0

        # Connect serial
        self._ser = None
        self._connect_serial()

        # Subscriber
        self._sub = self.create_subscription(
            VideoPacket,
            '/video_stream',
            self._packet_callback,
            rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=300,
            ),
        )

        # Send timer
        period = 1.0 / max(self._send_hz, 1)
        self._timer = self.create_timer(period, self._send_loop)

        self.get_logger().info(
            f'SerialBridge started: port={self._port_cfg} baud={self._baudrate} '
            f'{self._send_hz}Hz max_payload={self._max_payload}'
        )

    # ------------------------------------------------------------------
    # Serial
    # ------------------------------------------------------------------

    def _scan_serial_ports(self) -> list:
        """Scan for serial port candidates, like SerialTx::open()."""
        candidates = []
        candidates.extend(sorted(glob.glob('/dev/serial/by-id/*')))
        candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
        candidates.extend(sorted(glob.glob('/dev/ttyACM*')))
        # Remove duplicates while preserving order
        seen = set()
        unique = []
        for p in candidates:
            if p not in seen:
                seen.add(p)
                unique.append(p)
        return unique

    def _connect_serial(self) -> bool:
        """Open the configured serial port."""
        if self._port_cfg == 'auto':
            ports = self._scan_serial_ports()
            if not ports:
                self.get_logger().error(
                    'No serial ports found (checked /dev/serial/by-id/*, '
                    '/dev/ttyUSB*, /dev/ttyACM*). Waiting for device...'
                )
                return False
            self._active_port = ports[0]
        else:
            self._active_port = self._port_cfg

        try:
            self._ser = pyserial.Serial(
                port=self._active_port,
                baudrate=self._baudrate,
                bytesize=pyserial.EIGHTBITS,
                parity=pyserial.PARITY_NONE,
                stopbits=pyserial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=1.0,
            )
            self.get_logger().info(
                f'Serial port {self._active_port} opened at {self._baudrate} baud, 8N1'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to open {self._active_port}: {e}')
            return False

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _packet_callback(self, msg: VideoPacket):
        """Accumulate H264 data from incoming VideoPackets."""
        with self._buffer_lock:
            # Extract actual data bytes (VideoPacket.data is always 150B,
            # but last packet of a frame may have padding with zeros.
            # We just append all 150 bytes — decoder's h264parse handles
            # trailing zeros gracefully.)
            self._buffer.extend(bytes(msg.data))
            self._bytes_buffered += len(msg.data)

    def _send_loop(self):
        """Timer callback: send chunks at the link rate."""
        if self._ser is None or not self._ser.is_open:
            # Try to reconnect
            if not self._connect_serial():
                return

        with self._buffer_lock:
            if len(self._buffer) >= self._max_payload:
                # Pop one chunk
                chunk = bytes(self._buffer[:self._max_payload])
                self._buffer = self._buffer[self._max_payload:]

                # Build compact video header + 0x0310 frame
                video_header = _build_video_header(
                    len(chunk), self._video_seq, self._first_chunk
                )
                self._first_chunk = False

                # 0x0310 payload: video header + H264 data
                raw_payload = video_header + chunk  # 3 + 297 = 300B
                frame = _build_0310_frame(raw_payload, self._serial_seq)

                try:
                    self._ser.write(frame)
                    self._ser.flush()
                    self._video_seq = (self._video_seq + 1) & 0xFF
                    self._serial_seq = (self._serial_seq + 1) & 0xFF
                    self._chunks_sent += 1
                except Exception as e:
                    self.get_logger().error(f'Serial write error: {e}')
                    self._ser.close()
                    self._ser = None

        # Telemetry logging (every ~2s = 100 ticks at 48Hz)
        if self._chunks_sent % 100 == 0 and self._chunks_sent > 0:
            buf_size = len(self._buffer) if self._buffer_lock.locked() is False else '?'
            with self._buffer_lock:
                buf_size = len(self._buffer)
            self.get_logger().info(
                f'TX: chunks_sent={self._chunks_sent} '
                f'buffer={buf_size}B buffered={self._bytes_buffered}B '
                f'port={self._active_port}'
            )

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
