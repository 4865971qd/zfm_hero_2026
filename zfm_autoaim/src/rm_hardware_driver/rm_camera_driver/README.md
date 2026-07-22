# rm_camera_driver

## 1. 包概述

`rm_camera_driver` 是 RoboMaster 自瞄系统的相机驱动包，负责从 USB 相机（海康机器人 MV 系列）采集图像数据，或从本地视频文件回放模拟图像流，并发布为 ROS2 图像消息供后续视觉处理节点使用。

主要功能模块：

- **HikCameraNode**（命名空间 `hik_camera`）：使用海康机器人 MvCameraControl SDK 枚举 USB 设备、打开相机、配置分辨率（1440x1080）、执行 Bayer 到 RGB8 像素格式转换（`MV_CC_ConvertPixelType`），并通过 `image_transport` 发布图像。同时支持录制视频到本地文件。
- **VideoPlayerNode**（命名空间 `zfm::camera_driver`）：从预录制的视频文件（如 `.avi`）读取帧，按指定帧率发布 ROS2 图像消息，用于无硬件环境下的调试与仿真。
- **Recorder**（命名空间 `zfm::camera_driver`）：线程安全的视频录制器，基于 OpenCV `VideoWriter`（MJPEG 编码，AVI 容器）。采用生产者-消费者模型，使用有界队列（最大 5 帧）和 `condition_variable` 实现帧缓冲与异步写入。

## 2. 关键变量含义

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `exposure_time` | int | 动态可调 | 相机曝光时间，单位微秒。支持运行时动态调整 |
| `gain` | double | 动态可调 | 相机增益值。支持运行时动态调整 |
| `use_sensor_data_qos` | bool | false | 是否使用传感器数据 QoS（`SensorDataQoS`），否则使用默认 QoS |
| `camera_info_url` | string | "" | 相机内参文件 URL（YAML 格式），通过 `camera_calibration_parsers` 加载 |
| `camera_name` | string | "camera" | 相机名称，用于 CameraInfo 消息 |
| `frame_rate` | int | 30 | 视频回放帧率（仅 VideoPlayerNode 使用） |
| `path` | string | "" | 视频文件路径（仅 VideoPlayerNode 使用） |
| `keep_looping` | bool | false | 是否循环播放视频（仅 VideoPlayerNode 使用） |
| `record_enable` | bool | false | 是否启用视频录制 |
| `record_path` | string | "" | 录制视频保存路径 |
| `record_fps` | int | 30 | 录制视频帧率 |

## 3. 模块逻辑与函数执行流程

### HikCameraNode

```
初始化（构造函数）
├── initializeCamera()
│   ├── MV_CC_EnumDevices()          // 枚举 USB 设备
│   ├── MV_CC_CreateHandle()         // 创建设备句柄
│   └── MV_CC_OpenDevice()           // 打开设备
├── declareParameters()             // 声明 ROS 参数
├── loadCameraInfo()                // 加载相机内参
├── startGrabbing()                 // MV_CC_StartGrabbing 开始采集
└── startCaptureThread()            // 启动采集线程
        │
        └── 采集线程循环
            ├── MV_CC_GetImageBuffer()          // 获取原始帧（带超时）
            ├── convertBayerToRGB8()            // Bayer → RGB8 转换
            ├── [record_enable] Recorder::feed() // 送入录制队列
            ├── 时间戳校正
            └── camera_publisher_.publish()     // 发布图像 + CameraInfo
```

关键设计点：

- 采集在**独立线程**中运行，不阻塞 ROS 主线程，保证高帧率下的实时性。
- 像素格式转换使用 SDK 提供的 `MV_CC_ConvertPixelType`，支持从 Bayer 格式转换为 RGB8。
- 图像发布使用 `image_transport::CameraPublisher`，同时发布 `Image` 和 `CameraInfo` 消息。
- 视频录制通过 `Recorder` 类异步写入，避免 I/O 阻塞采集流程。

### VideoPlayerNode

```
初始化（构造函数）
├── 声明参数（frame_rate, path, keep_looping）
├── 创建 image_transport 发布者
├── cv::VideoCapture::open(path)
└── 创建定时器（周期 = 1000/frame_rate ms）

定时器回调
├── cap_.read(frame)
├── [失败] 若 keep_looping 则 cap_.set(CAP_PROP_POS_FRAMES, 0) 重置
├── [失败且不循环] 停止定时器
└── 发布图像消息
```

### Recorder

```
feed(frame)           // 生产者调用，将帧加入队列
    ├── unique_lock 加锁
    ├── 队列满则等待（condition_variable）
    └── 帧入队，通知消费者线程

消费者线程循环：
    ├── unique_lock 加锁
    ├── 队列空则等待（condition_variable）
    └── 出队，VideoWriter::write() 写入文件
```

该模式有效解耦了相机采集与磁盘写入，减少录制对采集帧率的影响。

### 已应用的重要修复

- **定时器优化**：去除了冗余的 1ms 定时器 + `WallRate` 轮询模式，改用以 `frame_rate` 为周期的单一定时器。
- **Spin 移除**：去除了构造函数中的阻塞式 `rclcpp::spin()` 调用，使节点支持组件化（Composable）部署。
- **Recorder 初始化**：修复了 `recoring_` 标志位未初始化为 `false` 的问题，改用 `std::lock_guard` 实现 RAII 互斥锁管理。

## 4. 发布和订阅的消息

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `image_raw` | `sensor_msgs::msg::Image` | 通过 `image_transport::CameraPublisher` 发布的图像数据（RGB8 格式，1440x1080） |
| `image_raw/camera_info` | `sensor_msgs::msg::CameraInfo` | 同时发布的相机内参信息，包含从 `camera_info_url` 加载的标定数据 |

### 订阅

无外部订阅话题（独立数据源驱动）。

## 5. 调试方法与参数调整

### 参数调整

所有参数均可通过 ROS2 参数机制在启动时设置或运行时动态调整：

```bash
# 启动时设置参数
ros2 run rm_camera_driver hik_camera_node \
    --ros-args -p exposure_time:=5000 -p gain:=15.0 -p record_enable:=true

# 运行时动态调整（需节点支持）
ros2 param set /hik_camera_node exposure_time 8000
ros2 param set /hik_camera_node gain 20.0
```

### 视频回放模式

在无相机硬件时，可使用 VideoPlayerNode 进行调试：

```bash
ros2 run rm_camera_driver video_player_node \
    --ros-args -p path:=/path/to/test_video.avi -p frame_rate:=30
```

### 录制验证

启用录制后，可在 `record_path` 指定的路径下查看录制的 AVI 文件，验证采集与发布流程是否正常。

### 常见问题排查

- **找不到相机设备**：确认 USB 设备已连接，检查 `lsusb` 输出；确认非 root 用户具有 `/dev/bus/usb/*` 的读写权限。
- **图像发布频率异常**：检查 `exposure_time` 是否过大导致帧率下降；检查 CPU 负载是否过高。
- **录制文件无法打开**：确认 `record_path` 目录存在且可写；确认系统支持 MJPEG 编码。
- **视频回放不流畅**：调整 `frame_rate` 参数匹配原始录制帧率，或检查视频文件编码是否为 OpenCV 支持的格式。
