# 【RM2026】部署模式低带宽落点图传 

用于 RoboMaster 部署模式下英雄机器人观测弹丸落点的低带宽图传系统。

[演示视频](https://www.bilibili.com/video/BV12aDMBcEob) · [RM社区开源报告](https://bbs.robomaster.com/article/1883295)

<img width="502" height="323" alt="视频封面" src="https://github.com/user-attachments/assets/a72e1683-be42-44d2-a3ae-7686517728fd" />

---

## 系统架构总览

```
┌───────────  miniPC 端 (机载) ───────────┐     ┌── 选手端 (笔记本) ──┐
│                                          │     │                      │
│  海康相机 ─→ hik_camera ─→ video_encoder  │     │  MQTT CustomByteBlock │
│              Bayer→BGR      静态/动态分离  │     │         ↓              │
│              @250fps         x264编码      │     │  rm-native-viewer     │
│                                 ↓         │     │  (或 doorlock_decoder) │
│                          serial_bridge    │     │  重组H264→ffmpeg解码   │
│                          0x0310帧封装      │     │  准星叠加→OpenGL显示   │
│                              ↓            │     │                      │
│                    UART 921600 8N1 ──────→ VTX → 2.4GHz → 接收端      │
└──────────────────────────────────────────┘     └──────────────────────┘
```

四个核心模块：

| 模块 | 节点 | 语言 | 职责 |
|------|------|------|------|
| 相机图像获取 | `hik_camera` | C++ | 海康 MVS SDK 取流，Bayer→BGR，发布 `/image_raw` |
| 图像压缩与关键信息保留 | `video_encoder` | C++ | 运动/静态分离，x264 编码，带宽限速分包 |
| 图传链路通信 | `serial_bridge` | Python | VideoPacket→0x0310 串口帧，CRC 校验 |
| 选手端接收 | `rm-native-viewer` / `doorlock_decoder` | Rust / Python | H264 重组解码，准星叠加显示 |

---

## 一、相机图像获取 (`hik_camera`)

**文件**: `src/hik_camera/src/hik_camera_node.cpp`

### 1.1 相机枚举与连接

```
MVS SDK 枚举 USB 设备 → 获取第一个相机 → 创建句柄 → 打开设备
```

关键代码流程：

1. **枚举设备**: 调用 `MV_CC_EnumDevices(MV_USB_DEVICE, &device_list)` 扫描所有海康 USB 相机。如果未找到相机，每秒重试直到设备出现。
2. **创建句柄**: `MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0])` — 绑定第一个枚举到的设备。
3. **打开设备**: `MV_CC_OpenDevice(camera_handle_)` — 建立与相机的通信通道。
4. **获取图像参数**: `MV_CC_GetImageInfo()` 读取传感器的最大分辨率 (`nHeightMax × nWidthMax`)，预分配 ROS Image 消息的 data 缓冲区。

### 1.2 相机参数配置 (`declareParameters`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `exposure_time` | 1000.0 μs | 曝光时间，范围从相机固件读取 |
| `gain` | 相机当前值 | 模拟增益，范围从相机固件读取 |
| `AcquisitionFrameRate` | **250 Hz** | 硬编码，相机以最高帧率输出 |
| `ColorTransformationEnable` | true | 启用颜色变换 |
| `Gamma` | **7.5** | 高 gamma 值增强暗部对比度 |
| `CCMEnable` | false | 关闭颜色校正矩阵 |

这些参数支持运行时动态调整，通过 ROS 2 的 `add_on_set_parameters_callback` 机制实现：修改 `exposure_time` 或 `gain` 时，回调函数直接调用 `MV_CC_SetFloatValue` 更新相机寄存器。

### 1.3 图像采集线程

**核心循环**（独立线程，与 ROS 主线程分离）：

```
while(rclcpp::ok()):
    MV_CC_GetImageBuffer(&out_frame, 1000ms超时)
        ↓ 成功
    OpenCV BayerRG→BGR 转换
        ↓
    camera_pub_.publish(image_msg, camera_info_msg)
        ↓
    MV_CC_FreeImageBuffer(&out_frame)
```

关键实现细节：

1. **Bayer 解码优化**: 使用 OpenCV 的 `cv::cvtColor(bayer_mat, rgb_mat, cv::COLOR_BayerRG2RGB)` 而非海康 SDK 自带的像素格式转换。代码注释说明 "OpenCV bayer converter is much faster than HIK SDK"。

2. **Buffer 复用**: `image_msg_.data` 在初始化时预分配 `width × height × 3` 字节，`cv::Mat` 直接 wrap 这个缓冲区 (`image_msg_.data.data()`)，避免每帧重新分配内存。

3. **故障恢复**: 取帧失败时自动 `StopGrabbing → StartGrabbing` 重新启动采集。连续失败超过 5 次则触发 `rclcpp::shutdown()`。

4. **帧率**: 相机以 250 fps 全速输出，但下游编码器会根据 `output_fps` 参数做抽帧。不限制帧率可以保证取到的帧时间戳最接近编码器需要的时刻。

### 1.4 发布接口

- **Topic**: `/image_raw` (通过 `image_transport::CameraPublisher` 同时发布 Image + CameraInfo)
- **编码**: `bgr8`
- **QoS**: `rmw_qos_profile_sensor_data` (可靠传输，适配传感器数据)

---

## 二、图像压缩与关键信息保留 (`video_encoder`)

**文件**: `src/doorlock_sniper/src/video_encoder_node.cpp`

这是系统的核心模块。核心理念：**部署模式下背景静止，只有弹丸/目标运动。将字节集中分配到运动区域，静态区域大幅压缩。**

### 2.1 处理管线概览

```
ROS Image消息 (BGR8)
    ↓
┌─ 预处理阶段 (CPU, OpenCV) ─────────────────────────────┐
│  1. 中心裁剪 (crop_size)                               │
│  2. 缩放至编码分辨率 (output_size)                       │
│  3. 灰度化 (如果是单色模式)                               │
│  4. 静态/动态分离                                       │
│     ├─ 背景建模 (滑动平均)                               │
│     ├─ 差分 → 阈值 → 腐蚀 → 膨胀 = 运动掩码              │
│     ├─ 中心保护区强制标记为"运动"                         │
│     ├─ 静态区域: Gaussian 模糊 + 灰度化                  │
│     ├─ 运动区域: 保留原细节                              │
│     └─ 运动拖影: 时域 max 叠加历史帧                      │
│  5. 输出: 合成帧 (静态模糊 + 运动清晰 + 拖影)             │
└───────────────────────────────────────────────────────┘
    ↓
┌─ 编码阶段 (GStreamer x264) ────────────────────────────┐
│  appsrc → videoconvert → x264enc → h264parse → appsink │
│  参数: bitrate=80kbps, veryslow preset,                 │
│        bframes=4, ref=5, subme=8, trellis=2             │
└───────────────────────────────────────────────────────┘
    ↓
┌─ 分包与限速 ──────────────────────────────────────────┐
│  H264 Annex-B 字节流                                   │
│    → 150B 分包 (VideoPacket.msg)                       │
│    → 2秒滑动窗口限速 (14 kB/s 硬上限)                   │
│    → 超时丢弃 (对齐 Annex-B 起始码)                     │
│    → 发布到 /video_stream                              │
└───────────────────────────────────────────────────────┘
```

### 2.2 预处理：静态/动态分离 (`preprocess_image`)

这是图像质量优化的核心，共 7 个步骤：

#### Step 1 — 中心裁剪

```cpp
// 从原图中心裁剪 crop_size × crop_size 区域
int x = (input.cols - crop_size) / 2;
int y = (input.rows - crop_size) / 2;
cv::Mat cropped = input(Rect(x, y, crop_size, crop_size));
```

目的：相机画面边缘可能包含不需要的机械结构或背景，只保留画面中心感兴趣区域。

#### Step 2 — 缩放

```cpp
cv::resize(cropped, resized, Size(output_size, output_size), 0, 0, INTER_LINEAR);
```

将裁剪后的图像缩放到编码分辨率（默认 300×300）。双线性插值保证缩放质量。

#### Step 3 — 灰度化（可选）

如果 `force_monochrome = true` 或低码率模式下，将 BGR 转为灰度再转回 BGR（三通道相同），x264 对灰度内容的压缩效率更高。

#### Step 4 — 背景建模

```cpp
// 滑动平均更新背景模型
cv::accumulateWeighted(gray, background_gray_f32_, alpha);
```

- 使用 `cv::accumulateWeighted` 做指数滑动平均，构建灰度背景模型 `background_gray_f32_`
- `alpha = 0.01` 意味着背景约 100 帧收敛到稳定值
- 首帧直接设为初始背景

#### Step 5 — 运动检测

```cpp
cv::absdiff(gray, bg_u8, diff);                     // 当前帧与背景差分
cv::threshold(diff, motion_mask, threshold, 255, THRESH_BINARY);  // 阈值二值化
cv::erode(motion_mask, motion_mask, kernel);         // 腐蚀去噪
cv::dilate(motion_mask, motion_mask, kernel);        // 膨胀填补空洞
```

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `motion_threshold` | 14 | 亮度差 >14 判定为运动 |
| `motion_erode_px` | 2 | 腐蚀半径（消除孤立噪点） |
| `motion_dilate_px` | 6 | 膨胀半径（连接断裂的运动区域） |

#### Step 6 — 中心保护区

```cpp
// 画面中心的 clear_size 区域强制设为"运动"
cv::rectangle(motion_mask, center_rect, Scalar(255), FILLED);
```

`center_clear_size = 150` 像素的中心区域强制标记为运动，确保准星周围始终清晰，不因静态模糊而丢失目标细节。

#### Step 7 — 合成与拖影

```cpp
// 静态区域: Gaussian 模糊 + 灰度化
cv::GaussianBlur(static_base, blurred_static, Size(), bg_blur_sigma);

// 运动区域: 保留原图细节
working.copyTo(focused, motion_mask);

// 运动拖影: 历史帧的 per-pixel max 叠加
cv::max(trail_img, trail_frame_history_[i], trail_img);
trail_img.copyTo(focused, trail_mask);
```

- **静态区域模糊**: `bg_blur_sigma = 1.8`，大幅降低高频细节，让 x264 用极少的 bit 编码
- **运动拖影**: 保留最近 `motion_trail_frames` 帧的运动区域，做逐像素 max 叠加，使弹道轨迹在画面上留下短暂拖影
- **全局运动抑制**: 当运动像素占比超过 `trail_disable_motion_ratio = 0.30`（30%）时，判定为相机大范围抖动，**禁用拖影**以避免整个画面被涂满

### 2.3 GStreamer x264 编码

**管道结构**:

```
appsrc → videoconvert → x264enc → h264parse → appsink
 BGR raw    色彩转换     H264编码    Annex-B解析  输出缓冲
```

**低码率模式**（目标码率 ≤ 80 kbps 时自动启用）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `bitrate` | 80 kbps | 目标编码码率 |
| `speed-preset` | 9 (veryslow) | 极限压缩效率 |
| `bframes` | 4 | 最大 B 帧数 |
| `ref` | 5 | 参考帧数 |
| `rc-lookahead` | 40 | 码率控制前瞻帧数 |
| `subme` | 8 | 亚像素运动估计精度 |
| `trellis` | 2 | Trellis 量化优化 |
| `aq-mode` | 2 | 自适应量化（自动变暗均匀区域） |
| `vbv-buf-capacity` | 500 | VBV 缓冲区大小 (限制 I 帧突发) |
| `key-int-max` | 480 (8×60fps) | 最大关键帧间隔 ≈ 8秒 |
| `scenecut` | 0 | 关闭场景切换检测（静态场景不需要） |

**流式输出**: `h264parse` 设置 `config-interval=-1`，每个 IDR 前都会插入 SPS/PPS，使解码端在码流任意位置都可以快速恢复解码。输出格式为 `byte-stream`（Annex-B），每个 NALU 以起始码 `00 00 01` 或 `00 00 00 01` 分隔。

### 2.4 分包与发送限速 (`pull_stream_and_packetize`)

编码后的 H264 Annex-B 字节流需要被拆分为固定大小的 VideoPacket 并通过 ROS 主题发送。

**VideoPacket 格式** (`msg/VideoPacket.msg`):

```
uint64 sequence_id      # 全局包序号 (用于丢包检测和重组)
uint64 timestamp_ns     # ROS 时间戳
uint8[150] data         # 固定 150 字节 H264 负载
```

**滑动窗口限速**:

```
┌─────────────────────────────────────────┐
│  过去 2 秒内的所有已发送包               │
│  sent_window_ = [(t1,150), (t2,150), …]  │
│  sent_window_bytes_ = 累计字节数          │
└─────────────────────────────────────────┘
```

- 每个周期先驱逐 2 秒之前的旧记录
- 如果 `sent_window_bytes_ + 150 > window_limit_bytes`（14 kB/s × 2s = 28 kB），**暂停发送**，数据积压在 `stream_buffer_` 中
- 这保证了瞬时码率峰值（如 I 帧）不会溢出图传链路

**队列超时丢弃**:

当 `stream_buffer_` 超过 `max_backlog_bytes`（14 kB/s × 1s = 14 kB）时触发丢弃：
- 优先丢弃到下一个 Annex-B 起始码（`00 00 01` 或 `00 00 00 01`），从而丢弃完整的 NALU
- 避免从 NALU 中间截断导致解码器长时间无法恢复

### 2.5 关键参数对画质的影响

| 参数 | 增大效果 | 减小效果 |
|------|----------|----------|
| `crop_size` | 更大视野 | 更小视野，更高细节密度 |
| `output_size` | 更清晰，但码率压力大 | 更模糊，更省码率 |
| `motion_threshold` | 更少区域被识别为运动 → 更多区域被模糊 | 更多区域保留细节，但可能把噪声当运动 |
| `bg_blur_sigma` | 静态区域更模糊 → 省码率 | 静态区域更清晰 |
| `center_clear_size` | 准星周围更清晰 | 省码率 |
| `motion_trail_frames` | 弹道拖影更长 | 拖影更短 |
| `target_bitrate` | 整体画质提升 | 整体画质下降 |

---

## 三、图传链路通信 (`serial_bridge`)

**文件**: `src/serial_bridge/serial_bridge/serial_bridge_node.py`

将 ROS 内部的 VideoPacket（150B）转换为 RoboMaster 官方图传链路所需的 0x0310 串口帧，通过 UART 发送给 VTX（Video Transmitter）模块。

### 3.1 协议栈

```
┌──────────────────────────────────────────┐
│          应用层: H264 Annex-B 字节流       │
├──────────────────────────────────────────┤
│  紧凑视频层: 3B 头 + 最多297B H264 数据    │
│  byte[0] = flags | (payload_len>>8)<<1    │
│  byte[1] = sequence (8-bit, wrap@256)     │
│  byte[2] = payload_len & 0xFF             │
├──────────────────────────────────────────┤
│  0x0310 帧层 (共 309 字节)                 │
│  SOF(1) + data_len(2) + seq(1) + CRC8(1)  │
│  + cmd_id(2) + data(300) + CRC16(2)       │
├──────────────────────────────────────────┤
│  物理层: UART 921600 bps 8N1              │
└──────────────────────────────────────────┘
```

### 3.2 紧凑视频头（3 字节）

**定义**（与 `rm-native-viewer` 的 `parse_video_0310_chunk` 完全一致）：

```
byte[0]: flags_and_payload_msb
  bit 0:   RESET flag — 首帧置 1，通知接收端重置解码器
  bit 1:   payload_len[8] — 负载长度的第 9 位 (bit 8)
  bit 2-7: 保留 (必须为 0)

byte[1]: sequence
  8-bit 帧序号，从 0 到 255 循环递增

byte[2]: payload_lsb
  负载长度的低 8 位

payload_len = ((byte[0] >> 1) & 1) << 8 | byte[2]
             范围: 0 ~ 297 (VIDEO_0310_PAYLOAD_BYTES)
```

**最大负载**: 300 (0x0310 data区) - 3 (视频头) = **297 字节**

### 3.3 0x0310 串口帧结构

```
 [SOF] [DATA_LEN_LO] [DATA_LEN_HI] [SEQ] [CRC8] [CMD_LO] [CMD_HI] [DATA…300B] [CRC16_LO] [CRC16_HI]
   0       1             2         3      4      5         6       7~306        307        308
```

**字段说明**:

| 字段 | 大小 | 值/含义 |
|------|------|---------|
| SOF | 1B | `0xA5` 帧头标识 |
| data_len | 2B (LE) | 数据区长度，固定 300 |
| seq | 1B | 串口帧序号 (与视频序列号独立) |
| header_crc8 | 1B | 对 SOF+data_len+seq 的 CRC-8 校验 |
| cmd_id | 2B (LE) | `0x0310` (机器人到自定义客户端的视频数据命令) |
| data | 300B | 3B 视频头 + 297B H264 数据（不足补零） |
| crc16 | 2B (LE) | 对前面全部 307 字节的 CRC-16/MCRF4XX 校验 |

**CRC 算法**:
- **CRC-8**: 多项式 `0x31` (x⁸+x⁵+x⁴+1), 初始值 `0xFF`, 查表法
- **CRC-16/MCRF4XX**: 多项式 `0x1021`, 初始值 `0xFFFF`, 反射输入/输出, 查表法

上述 CRC 表与 `rm_compress` 的 `SerialTx` 和官方通信协议完全一致。

### 3.4 数据流处理

```
ROS Subscriber (/video_stream, RELIABLE QoS)
    ↓ 每收到一个 VideoPacket (150B)
_buffer (bytearray, 线程安全)
    ↓ 追加 VideoPacket.data 全部 150 字节
    ↓
Timer 回调 @ 48 Hz (每 ~20.83ms)
    ↓
检查 _buffer 是否 ≥ 297 字节
    ↓ 是
弹出 297 字节 → 构造 3B 视频头 → 封 0x0310 帧
    ↓
serial.write(frame) + serial.flush()  (tcdrain 效果)
    ↓
_video_seq++, _serial_seq++ (均 8-bit wrap)
```

**关键设计决策**:

1. **48 Hz 发送速率**: 实测 50 Hz 会引起 0x0310 链路突发丢包（`rm_compress` 文档确认），48 Hz 是安全上限。

2. **VideoPacket 的 150B 与 0x0310 的 300B 的关系**: 两个 VideoPacket 的数据 (2×150=300B) 填充一个 0x0310 数据区。但为了灵活处理 NALU 边界，不使用固定 2:1 映射，而是累积字节流后按 297B 切块。

3. **首帧 RESET 标志**: `_first_chunk = True` 确保接收端的 H264 解码器在收到第一个 0x0310 帧时重置，从 SPS/PPS/IDR 开始解码。

4. **串口自动发现**: 当 `serial_port = 'auto'` 时，按优先级扫描 `/dev/serial/by-id/*` → `/dev/ttyUSB*` → `/dev/ttyACM*`，选第一个可用的。与 `SerialTx::open()` 行为一致。

5. **断线重连**: `_send_loop` 每次 tick 检测串口状态，若已断开则尝试 `_connect_serial()` 重新扫描并打开。

### 3.5 码率计算

```
48 chunks/s × 297 bytes/chunk = 14,256 bytes/s ≈ 114 kbps

(这是视频净荷的理论上限，实际 H264 码率由编码器目标码率 80 kbps 控制，
 大部分时间不需要填满 297B，payload_len 会小于 297)
```

---

## 四、选手端图像接收

有两种接收方案：

### 方案 A: `doorlock_decoder` (Python, ROS 直连)

**文件**: `src/doorlock_decoder/doorlock_decoder/video_decoder_node.py`

**适用场景**: 本地调试，编码端和解码端在同一 ROS 网络中。

#### 工作流程

```
ROS Subscriber (/video_stream, RELIABLE QoS)
    ↓
VideoPacket 到达 (_packet_callback)
    ↓
丢包检测: if seq != last_seq + 1 → gap! → 重置解码器
    ↓
bytes(msg.data) → codec.parse(chunk) → 解析 H264 NALU
    ↓
codec.decode(packet) → 解码为 cv2/numpy frame (BGR24)
    ↓
frame_queue (maxsize=3, 非阻塞放入)
    ↓
_display_loop (独立线程)
    ↓
cv2.resize (NEAREST, display_scale=2x) → 300→600
    ↓
_draw_overlay: 画十字准心 + 中心圆点
    ↓
cv2.imshow("Doorlock Decoder")
```

#### 丢包恢复策略

```python
if msg.sequence_id != self.last_seq + 1:
    self.gap_count += 1
    self._reset_decoder(reason='sequence gap')
```

- 任意 VideoPacket 丢失都会导致 H264 码流失同步
- 策略：**立即重置解码器**（创建新的 `av.CodecContext`），等待下一个 SPS/PPS + IDR 组合
- `h264parse` 在编码端设置 `config-interval=-1`（每个 IDR 前插入 SPS/PPS），确保重置后最多等待一个关键帧间隔即可恢复
- 帧间隔: 60 fps / key-int-max=480 → 最长 8 秒，典型场景远小于此

#### 准星叠加

```python
# 淡紫色十字准心
cv2.line(img, (0, cy), (w-1, cy), (230,190,235), crosshair_width)
cv2.line(img, (cx, 0), (cx, h-1), (230,190,235), crosshair_width)
# 画面中心淡绿色圆点
cv2.circle(img, center, 24, (170,255,170), 1)
```

- 准心位置可调 (`crosshair_offset_x/y`)
- 中心圆点固定不可调

### 方案 B: `rm-native-viewer` (Rust, MQTT 链路)

**文件**: `../rm-native-viewer/src/custom_client.rs`

**适用场景**: 正式比赛，通过官方 MQTT CustomByteBlock 接收 0x0310 视频数据。

#### 数据链路

```
VTX 接收 → 裁判系统 → MQTT Broker (192.168.12.1:3333)
                          ↓
              rm-native-viewer 订阅 CustomByteBlock
                          ↓
          decode_custom_byte_block() 解析 protobuf field 1
                          ↓
          parse_video_0310_chunk() 解析 300B 紧凑视频帧
              提取: 3B 头 (flags, seq, len) + 297B H264
                          ↓
          H264 Annex-B 字节流重拼接
                          ↓
          ffmpeg 子进程 stdin 喂流 → 解码 → stdout 输出 RGBA
                          ↓
          eframe/egui 原生窗口显示 + telemetry 状态叠加
```

#### 紧凑视频帧解析 (`parse_video_0310_chunk`)

```rust
byte[0] & 0xFC != 0       → 保留位非零，丢弃（非视频数据）
byte[0] & 0x01            → RESET flag (通知 FFmpeg 重置)
byte[1]                   → sequence (8-bit, 用于丢帧检测)
payload_len = (byte[2] as usize) | ((byte[0] & 0x02) != 0) << 8
                          → 有效负载长度 (0~297)
```

#### Telemetry 回退

当 `CustomByteBlock.data` 不符合视频帧约束时，程序回退为 telemetry v1 解析（`parse_vehicle_telemetry`）：

```
Magic "PDL1" (4B) → version(2B) → struct_bytes(2B) →
flags(4B) → unix_ms(8B) → frame_seq(4B) →
image_width/height(2B each) → fps/gain(2B each) →
exposure_us(4B) → gimbal_mode(1B) → bullet_count(2B) →
yaw/yaw_vel/pitch/pitch_vel/bullet_speed(f32 each) →
quaternion[4](f32 each) → status_text(变长)
```

画面上叠加的 OSD 信息包括：相机在线、云台在线/模式、帧序号、FPS、曝光/增益、yaw/pitch、弹速/弹量、状态文本。

---

## 五、使用指南

### 5.1 两个项目的定位

| | Pacific_doorlock_sniper | rm-native-viewer |
|---|---|---|
| **运行位置** | miniPC（机载端） | 笔记本（选手端） |
| **硬件依赖** | 海康相机 + USB-TTL 串口 | 无（仅需网络） |
| **通信方式** | UART 串口 → VTX 发射 | MQTT 订阅裁判系统 |
| **核心职责** | 采集 → 压缩 → 编码 → 发送 | 接收 → 重组 → 解码 → 显示 |
| **需要 ROS 2?** | 是 | 否 |
| **语言** | C++ / Python | Rust |

完整数据流向：

```
海康相机 ─USB─→ miniPC ─UART─→ VTX ─2.4GHz─→ 裁判系统 ─MQTT─→ 选手端笔记本
                                                              ↓
                                                     rm-native-viewer
                                                     显示画面 + OSD
```

### 5.2 三个 Launch 文件的使用场景

| Launch 文件 | 用途 | 相机来源 | 输出目标 |
|-------------|------|----------|----------|
| `local_sniper.launch.py` | **本地开发测试** | 海康相机 | ROS 话题 → doorlock_decoder 显示 |
| `onboard_sniper.launch.py` | **miniPC 场上部署** | 海康 CS016-10UC | 串口 0x0310 → VTX |
| `sniper.launch.py` | **原始演示**（海康直连 ROS 解码） | 海康相机 | ROS 话题 → doorlock_decoder 显示 |

---

### 5.3 Pacific_doorlock_sniper — 功能清单

**编码端核心功能**：

| 功能 | 参数 | 效果 |
|------|------|------|
| 静态区域模糊 | `bg_blur_sigma` (1.8) | 背景用高斯模糊抹去纹理，x264 省码率 |
| 运动区域保留 | `motion_threshold` (14) | 弹丸/目标运动处保留完整细节 |
| 中心保护区 | `center_clear_size` (150px) | 准星周围始终清晰 |
| 弹道拖影 | `motion_trail_frames` (15 max) | 时域叠加最近帧的运动区域，显示飞行路径 |
| 全局抖动抑制 | `trail_disable_motion_ratio` (30%) | 相机大幅抖动时自动禁用拖影 |
| 低码率优化 | `force_monochrome`, x264 veryslow | 极限带宽下可切换灰度 + 最高效编码参数 |
| 码率硬限速 | `bandwidth_limit_kbytes` (14 kB/s) | 2 秒滑动窗口，超出暂停发送 |
| 过载丢弃 | `max_tx_delay_s` (1s) | 队列积压超时丢弃，对齐 Annex-B 边界 |
| 解码端准星 | `crosshair_offset_x/y` | 十字准心 + 中心圆点，位置可调 |
| 调试 dump | `debug_dump_enable` | 定期保存 Raw/ROI/Static/Final/Decoder 窗口截图 |

**串口通信功能**：

| 功能 | 参数 | 说明 |
|------|------|------|
| 串口自动发现 | `serial_port=auto` | 扫描 by-id → ttyUSB → ttyACM |
| 0x0310 帧封装 | CRC-8 + CRC-16 | 与裁判系统协议完全一致 |
| 紧凑视频头 | 3B 头 + 297B H264 | 与 rm-native-viewer 兼容 |
| 48 Hz 发送 | `send_hz=48` | 匹配图传链路容量上限 |
| 断线重连 | 自动 | 每个 tick 检测并重新扫描串口 |

---

### 5.4 Pacific_doorlock_sniper — 本地开发测试

**场景**：只有一台电脑，没有海康相机，想先跑通管线看看效果。

```bash
# 1. 安装依赖（一次性）
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-av python3-serial \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav \
  ros-humble-usb-cam

# 2. 编译
source /opt/ros/humble/setup.bash
colcon build

# 3. 启动（USB 摄像头 → 编码 → 解码，全部在本机）
source install/setup.bash
ros2 launch bringup local_sniper.launch.py
```

**启动后会看到 5 个 OpenCV 窗口**：

| 窗口 | 内容 |
|------|------|
| Doorlock Sniper Raw | 摄像头原始画面（缩小 1/2） |
| Doorlock Sniper ROI | 中心裁剪 + 缩放后的图像 |
| Doorlock Sniper Static | 静态模糊 + 运动保留后的合成帧 |
| Doorlock Sniper | 最终送入 x264 的画面（含拖影） |
| Doorlock Decoder | 解码后的画面（含准星叠加） |

**怎么判断系统正常**：

```bash
# 看编码端统计（每 1 秒打印一次）
# 终端里找 "TX stats" 行：
#   window=1.05/28.00kB  → 过去 2 秒只发了 1KB（静止画面）
#   avg=0.53kB/s         → 非常低的码率
#   backlog=50B          → 队列很小
#   dropped=0B           → 没有丢包

# 看解码端日志
#   Rx packets=600 parsed_h264=... decoded_frames=... gaps=0
#   gaps=0 表示 VideoPacket 没有丢失
```

**在 5 个窗口前挥挥手**：你应该看到 ROI 窗口有你的手，Static 窗口出现运动掩码（白色区域），Final 窗口在你手的位置保留清晰细节而背景模糊。Decoder 窗口能看到解码后的画面。

---

### 5.5 Pacific_doorlock_sniper — miniPC 场上部署

**场景**：miniPC 已连接海康相机和 USB-TTL 串口模块，要输出到图传链路。

miniPC 上需要有的环境：
- Ubuntu 22.04 + ROS 2 Humble
- MVS SDK（`/opt/MVS`，海康相机驱动）
- GStreamer 相关插件（ugly + bad）
- python3-serial（串口通信）

```bash
# 1. 确保串口模块已插入且用户在 dialout 组
sudo usermod -aG dialout $USER
# 重新登录生效

# 2. 编译
source /opt/ros/humble/setup.bash
colcon build

# 3. 启动
source install/setup.bash
ros2 launch bringup onboard_sniper.launch.py
```

**期望的启动日志**：

```
[serial_bridge]: SerialBridge started: port=auto baud=921600 48Hz
[serial_bridge]: Serial port /dev/ttyUSB0 opened at 921600 baud, 8N1
[video_encoder]: GStreamer encoder ready (low-bitrate mode, byte-stream)
[video_encoder]: TX stats: window=1.05/28.00kB avg=0.53kB/s backlog=50B dropped=0B
[serial_bridge]: TX: chunks_sent=100 buffer=30B port=/dev/ttyUSB0
```

**关键验证项**：

1. `TX stats` 中 `dropped=0B` — 没有因码率超限丢包
2. `serial_bridge` 中 `chunks_sent` 持续增长 — 串口正在发送
3. 确认 serial_bridge 没有打印 `Serial write error`
4. `backlog` 稳定在几百字节以内

**串口没找到时**：

```
[serial_bridge]: No serial ports found. Waiting for device...
```

检查：`ls /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/*`，确认 USB-TTL 模块已插入并被识别。可以使用 `serial_port:=/dev/ttyUSB0` 手动指定端口：

```bash
ros2 launch bringup onboard_sniper.launch.py serial_port:=/dev/ttyUSB0
```

---

### 5.6 rm-native-viewer — 功能清单

| 功能 | 说明 |
|------|------|
| **MQTT 主链路** (默认) | 订阅 `CustomByteBlock`，重组 0x0310 紧凑视频分片 |
| **H264 解码** | 调用系统 ffmpeg，自动处理 SPS/PPS/IDR 恢复 |
| **窄带序列修复** | 检测 sequence gap → 丢弃直到下一个 SPS → 等待 IDR 恢复 |
| **TELEMETRY 叠加** | 解析 vehicle telemetry v1，画面上叠加相机/gimbal/弹速/弹量 |
| **UDP/3334 旧链路** | 兼容旧版 HEVC 原始图传（`--raw-udp --input-format hevc`） |
| **本地 0x0310 UDP** | 本地直连测试（`--0310-udp`） |
| **自动分辨率** | 启动时 `xrandr` 检测显示器分辨率，最大 2560×1600 |
| **headless 烟测** | `--headless-seconds 8` 无窗口自检，适合 CI / 远程部署验证 |
| **开机自启** | `install-autostart.sh` 安装 desktop 自启动项 |

---

### 5.7 rm-native-viewer — 选手端使用

**场景**：场上笔记本，连接裁判系统 WiFi，接收图传画面。

**一次性安装**：

```bash
# 系统依赖（Ubuntu 22.04 / 24.04）
sudo apt install -y ffmpeg pkg-config libxkbcommon-dev

# Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
# 重新打开终端或 source ~/.cargo/env

# 编译
cd ~/RM_zidinyi/rm-native-viewer
cargo build --release
```

**场上使用**：

```bash
# 最基本用法 — 什么都不用配
cargo run --release

# 如果裁判系统分配的 client ID 不是 101
cargo run --release -- 103

# 完整手动指定
cargo run --release -- \
  --mqtt-host 192.168.12.1 \
  --mqtt-port 3333 \
  --mqtt-topic CustomByteBlock \
  --client-id 103 \
  --input-format h264
```

**启动后看到的界面**：

```
┌────────────────────────────────────┐
│ RM Native Viewer                   │
│ ────────────────────────────────   │
│ 0310 Video: connected      (绿色)  │
│ UDP Raw: waiting           (灰色)  │
│ Decode: running            (绿色)  │
│ MQTT: connected            (绿色)  │
│ MQTT endpoint: 192.168.12.1:3333  │
│ Packets: 12543                     │
│ Bytes: 3.72 MiB                    │
│ Assembled frames: 12543            │
│ Dropped packets: 0                 │
│ Decoded frames: 12380              │
│ Display FPS: 59.8                  │
│ ────────────────────────────────   │
│ camera=online gimbal=online        │
│ frame=42 1280x720 fps=59.9        │
│ yaw=1.23 pitch=-0.45              │
│ bullet_speed=15.2 bullet_count=7  │
└────────────────────────────────────┘
        视频画面区域
```

**状态指示含义**：

| 字段 | 绿 | 黄 | 灰/其他 |
|------|-----|-----|---------|
| 0310 Video | 正在接收视频 | 等待/重连中 | 链路未配置 |
| Decode | ffmpeg 解码中 | 等待 IDR 关键帧 | 解码器停止 |
| MQTT | 已连接并接收 | 已连接但无数据 | 未启用 |

---

### 5.8 rm-native-viewer — 本地回放测试

**场景**：没有裁判系统，想在本地测试 viewer 是否能正常解码。

需要一个能重放 0x0310 视频数据的工具。可以配合本地 MQTT broker 做回放：

```bash
# 1. 安装 mosquitto
sudo apt install -y mosquitto mosquitto-clients

# 2. 启动本地 broker (终端1)
mosquitto -p 3333

# 3. 用录好的 0x0310 数据重放到 CustomByteBlock (终端2)
# （需要提前录好 MQTT CustomByteBlock 的 payload，保存为 raw 文件）

# 4. viewer 连本地 broker (终端3)
cargo run --release -- --mqtt-host 127.0.0.1 --mqtt-port 3333
```

---

### 5.9 rm-native-viewer — headless 烟测

**场景**：在 miniPC 或服务器上无图形环境验证链路是否正常。

```bash
cargo run --release -- --headless-seconds 8
```

输出示例：
```
[headless] udp_ok=false decode_ok=true mqtt_ok=true packets=384 assembled=384 decoded=370
```

退出码为 0 表示成功解码了至少一帧，非 0 表示失败。适合集成到 CI 或部署后自检脚本。

---

### 5.10 rm-native-viewer — 开机自启

**场景**：选手端笔记本开机自动启动 viewer，操作手不用每次手动打开。

```bash
cd ~/RM_zidinyi/rm-native-viewer
./scripts/install-autostart.sh --default-client 101
```

这会安装两个 desktop 自启动项（Client 1 / Client 101），默认启用 101。切换默认 ID：

```bash
./scripts/install-autostart.sh --default-client 1
```

---

### 5.11 rm-native-viewer — 所有命令行参数

```
rm-native-viewer [mqtt-client-id]        # 快捷参数，例如 101 或 1

--mqtt-host <host>        默认 192.168.12.1
--mqtt-port <port>        默认 3333
--mqtt-topic <topic>      默认 CustomByteBlock
--client-id <id>          默认 101
--input-format <fmt>      默认 h264，旧链路可设 hevc

--raw-udp                 启用 UDP/3334 旧 HEVC 图传
--no-udp                  关闭 UDP/3334
--0310-udp                启用本地 0x0310 UDP 直连
--no-mqtt                 关闭 MQTT，仅用 UDP

--width <n>               显示宽度（默认自动检测显示器）
--height <n>              显示高度
--ffmpeg <path>           ffmpeg 路径，默认 ffmpeg

--headless-seconds <n>    无窗口烟测模式
```

环境变量（优先级高于默认值但低于命令行）：
- `RM_VIEWER_MQTT_HOST` / `RM_VIEWER_MQTT_PORT` / `RM_VIEWER_MQTT_TOPIC`
- `RM_VIEWER_CLIENT_ID` / `RM_VIEWER_INPUT_FORMAT`
- `RM_VIEWER_ENABLE_RAW_UDP` / `RM_VIEWER_DISABLE_RAW_UDP` / `RM_VIEWER_DISABLE_MQTT`
- `RM_VIEWER_FFMPEG`

---

### 5.12 两端配合：完整测试流程

**Step 1 — miniPC 端**：启动编码+串口发送

```bash
# miniPC 上
ros2 launch bringup onboard_sniper.launch.py
# 确认: TX stats dropped=0, serial_bridge chunks_sent 增长
```

**Step 2 — 选手端**：启动 viewer

```bash
# 选手笔记本上，先查 client ID
python3 probe_mqtt_client_ids.py --host 192.168.12.1 --port 3333 --ids 101,102,103,104,105

# 用成功的 ID 启动
cargo run --release -- 103
```

**Step 3 — 验证**：在 viewer 界面上检查
- 0310 Video: **connected** (绿)
- Decode: **running** (绿)
- MQTT: **connected** (绿)
- Assembled frames 在增长
- Dropped packets: 0
- 能看到画面

---

## 环境要求与安装

### Pacific_doorlock_sniper (机载 miniPC)

```bash
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-av python3-serial \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-libav

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

还需要 MVS SDK（`/opt/MVS`）用于海康相机驱动。

### rm-native-viewer (选手端笔记本)

```bash
sudo apt install -y ffmpeg pkg-config libxkbcommon-dev
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
cargo build --release
```

不需要 ROS、不需要 MVS SDK、不需要 GStreamer。

---

## 项目结构

```
Pacific_doorlock_sniper/
├── src/
│   ├── hik_camera/            # 海康相机驱动
│   ├── doorlock_sniper/        # 编码器 + VideoPacket 消息
│   ├── serial_bridge/          # 0x0310 串口桥接
│   ├── doorlock_decoder/       # H264 解码器 (ROS 直连调试用)
│   └── bringup/                # Launch 文件
│       ├── launch/local_sniper.launch.py      # 本地测试
│       ├── launch/onboard_sniper.launch.py    # miniPC 部署
│       └── launch/sniper.launch.py            # 原始演示

rm-native-viewer/              # 选手端原生解码客户端 (Rust)
├── src/
│   ├── main.rs                # GUI + MQTT + 解码管线
│   └── custom_client.rs       # 0x0310 视频/遥测解析
├── scripts/
│   ├── run-viewer.sh          # 启动脚本
│   └── install-autostart.sh   # 开机自启安装
└── deploy/autostart/          # desktop 模板
```

---

本工程为演示工程，开发过程中使用了 LLM 作为辅助。欢迎大家基于这个思路开发更好的自定义客户端。
