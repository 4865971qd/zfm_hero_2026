# rm_serial_driver

## 1. 包概述

`rm_serial_driver` 是 RoboMaster 自瞄系统的串口通信驱动包，负责与下位机（嵌入式 MCU）进行双向串行数据交换，传输云台控制指令与接收状态反馈。同时提供纯仿真模式，可在无硬件环境下验证系统行为。

主要功能模块：

- **SerialDriverNode**（命名空间 `zfm::serial_driver`）：真实串口通信节点。通过 `ProtocolFactory` 创建协议解析器（当前默认协议为 `"hero"`），使用 `UartTransporter`（基于 Linux termios）操作串口。发送 `GimbalCmd` 数据，接收 `SerialReceiveData`（包含模式、欧拉角、弹速标志等），并发布 TF 变换。在检测到模式切换时自动调用各视觉节点的 `set_mode` 服务。
- **VirtualSerialNode**（命名空间 `fyt::serial_driver`）：纯仿真串口节点，无需硬件即可运行。以 200Hz 频率（5ms 定时器）发布模拟的 `SerialReceiveData` 消息，广播仿真 TF 变换，并创建 `SetMode` 服务客户端以模拟模式切换。
- **DefaultProtocol**：基于 `FixedPacket<16>` 的 16 字节定长协议。发送端包含 `fire_advice`（字节 1）、pitch（float，字节 2-5）、yaw（float，字节 6-9）、distance（float，字节 10-13）。接收端包含 mode（字节 1）、roll（float，字节 2-5）、pitch（float，字节 6-9）、yaw（float，字节 10-13）、bullet_speed_flag（字节 14）。
- **UartTransporter**：底层串口 I/O 封装，基于 Linux termios API。支持可配置的设备路径、波特率、数据位、停止位、校验位和流控，提供 `open`/`close`/`read`/`write` 接口及完善的错误处理。

## 2. 关键变量含义

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port_name` | string | `/dev/ttyUSB0` | 串口设备路径。默认 CH340 转串口设备 |
| `protocol` | string | `"hero"` | 通信协议类型，传递给 `ProtocolFactory` 创建对应协议实例 |
| `enable_data_print` | bool | false | 是否在终端打印接收到的串口数据，用于调试 |
| `timestamp_offset` | double | 0.0 | 时间戳偏移量，用于同步时间 |
| `target_frame` | string | `"gimbal_link"` | TF 变换的目标坐标系 |
| `roll` | double | 0.0 | 仿真模式下的 roll 角度（仅 VirtualSerialNode） |
| `pitch` | double | 0.0 | 仿真模式下的 pitch 角度（仅 VirtualSerialNode） |
| `yaw` | double | 0.0 | 仿真模式下的 yaw 角度（仅 VirtualSerialNode） |
| `vision_mode` | int | 0 | 仿真模式下的视觉模式（仅 VirtualSerialNode） |
| `has_rune` | bool | false | 是否启用符（rune）检测/解算服务客户端（仅 VirtualSerialNode） |

### DefaultProtocol 数据格式

**发送数据（16 字节）：**

| 字节偏移 | 内容 | 类型 | 说明 |
|---------|------|------|------|
| 0 | 帧头 | uint8 | 固定包头 |
| 1 | fire_advice | uint8 | 开火建议 |
| 2-5 | pitch | float | 云台 pitch 角度 |
| 6-9 | yaw | float | 云台 yaw 角度 |
| 10-13 | distance | float | 目标距离 |
| 14-15 | 校验/结尾 | uint16 | 固定包尾与校验 |

**接收数据（16 字节）：**

| 字节偏移 | 内容 | 类型 | 说明 |
|---------|------|------|------|
| 0 | 帧头 | uint8 | 固定包头 |
| 1 | mode | uint8 | 当前模式 |
| 2-5 | roll | float | 云台 roll 角度 |
| 6-9 | pitch | float | 云台 pitch 角度 |
| 10-13 | yaw | float | 云台 yaw 角度 |
| 14 | bullet_speed_flag | uint8 | 弹速标志位 |
| 15 | 校验 | uint8 | 校验字节 |

## 3. 模块逻辑与函数执行流程

### SerialDriverNode

```
初始化（构造函数）
├── 声明参数 (port_name, protocol, enable_data_print, timestamp_offset)
├── ProtocolFactory::create(protocol)   // 创建协议解析器
├── UartTransporter::open(port_name)    // 打开串口
├── 创建接收定时器 (read_timer_)        // 周期性接收数据
├── 创建发送订阅 (cmd_gimbal_sub_)      // 订阅云台控制指令
└── 创建 TF 广播器 (tf_broadcaster_)

接收定时器回调
├── transporter.read(buffer)            // 从串口读取原始字节
├── protocol->receiveData(buffer)       // 解析协议，获得 SerialReceiveData
├── [enable_data_print] 打印调试信息
├── 检查模式是否变化
│   └── [模式变化] 调用 setMode() 服务
│       ├── armor_detector/set_mode
│       ├── armor_solver/set_mode
│       └── [has_rune] rune_* /set_mode
├── 发布 SerialReceiveData 消息
└── 发送 TF (odom → gimbal_link, odom → odom_rectify)

GimbalCmd 订阅回调
├── protocol->sendData(cmd)             // 将 GimbalCmd 打包为协议字节流
└── transporter.write(buffer)           // 写入串口发送
```

关键设计点：

- 使用**定时器驱动**的接收模式，无需独立接收线程，降低复杂度。
- 模式变化时自动调用各视觉节点的 `set_mode` 服务，实现云台模式与视觉算法的联动切换。
- TF 变换广播 `odom` 到 `gimbal_link` 和 `odom_rectify`，用于自瞄系统的坐标变换。

### VirtualSerialNode

```
初始化（构造函数）
├── 声明参数 (roll, pitch, yaw, vision_mode, has_rune)
├── 创建 SerialReceiveData 发布者
├── 创建 TF 广播器
├── 创建 SetMode 服务客户端
│   ├── armor_detector/set_mode
│   ├── armor_solver/set_mode
│   └── [has_rune] rune_detector/set_mode, rune_solver/set_mode
└── 创建仿真定时器 (5ms 周期)

仿真定时器回调
├── 构造 SerialReceiveData（使用参数中设置的 roll/pitch/yaw/mode）
├── 发布消息
└── 广播仿真 TF (odom → gimbal_link, odom → odom_rectify)
```

### UartTransporter

```
open(device)
├── open() 系统调用打开设备文件（O_RDWR | O_NOCTTY）
├── tcsetattr() 配置 termios
│   ├── 波特率（默认 115200）
│   ├── 数据位（默认 8）
│   ├── 停止位（默认 1）
│   ├── 校验位（默认无）
│   └── 流控（默认无）
└── tcflush() 清空缓冲区

read(buffer, size)
├── 检查文件描述符有效性
├── fd_set + select() 超时等待
├── read() 系统调用读取字节
└── 错误处理与日志

write(buffer, size)
├── 检查文件描述符有效性
└── write() 系统调用写入字节
```

### 已应用的重要修复

- **默认串口修正**：默认端口从 `/dev/ttyACM0` 改为 `/dev/ttyUSB0`，适配 CH340 转串口模块。
- **默认协议修正**：协议默认从 `"infantry"` 改为 `"hero"`，与 `ProtocolFactory` 实际支持的协议类型匹配。
- **虚拟串口定时器优化**：仿真定时器周期从 1ms（1000Hz）降低至 5ms（200Hz），减少不必要的 CPU 占用。
- **代码清理**：移除 `default_protocol.cpp` 中注释掉的调试打印块，移除 `virtual_serial_node.cpp` 中未使用的头文件包含。

## 4. 发布和订阅的消息

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `serial/receive` | `rm_interfaces::msg::SerialReceiveData` | 从下位机接收到的状态数据，包含 mode、roll、pitch、yaw、bullet_speed_flag |
| `/tf` | `tf2_msgs::msg::TFMessage` | TF 变换：`odom` → `gimbal_link`、`odom` → `odom_rectify` |

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `armor_solver/cmd_gimbal` | `rm_interfaces::msg::GimbalCmd` | 来自装甲板解算节点的云台控制指令，包含目标 pitch、yaw 和距离 |

### 服务客户端

| 服务 | 说明 |
|------|------|
| `armor_detector/set_mode` | 设置装甲板检测节点的工作模式 |
| `armor_solver/set_mode` | 设置装甲板解算节点的工作模式 |
| `rune_detector/set_mode` | （可选）设置符检测节点的工作模式 |
| `rune_solver/set_mode` | （可选）设置符解算节点的工作模式 |

## 5. 调试方法与参数调整

### 参数调整

```bash
# 启动真实串口节点
ros2 run rm_serial_driver serial_driver_node \
    --ros-args \
    -p port_name:=/dev/ttyUSB0 \
    -p protocol:=hero \
    -p enable_data_print:=true

# 启动虚拟串口节点（无硬件仿真）
ros2 run rm_serial_driver virtual_serial_node \
    --ros-args \
    -p roll:=0.0 \
    -p pitch:=0.1 \
    -p yaw:=0.5 \
    -p vision_mode:=1 \
    -p has_rune:=false
```

### 串口通信调试

- **数据打印**：设置 `enable_data_print:=true` 可在终端实时查看接收到的串口数据，用于初步验证通信是否正常。
- **波特率确认**：确保下位机串口配置与上位机一致（默认 115200 8N1）。可通过 `UartTransporter` 的参数调整。
- **设备权限**：确保运行用户有 `/dev/ttyUSB0` 的读写权限，可通过 `sudo usermod -aG dialout $USER` 添加用户到 `dialout` 组。

### 虚拟串口模式

在没有硬件的情况下，使用 `virtual_serial_node` 可以：

- 验证 `armor_detector`、`armor_solver` 等节点对 `SerialReceiveData` 的响应逻辑。
- 测试 `set_mode` 服务调用链。
- 通过调整 `roll`、`pitch`、`yaw` 参数模拟不同云台姿态，观察 TF 变换是否正常。

### 数据流验证

使用 ROS2 命令行工具验证数据流：

```bash
# 查看接收到的串口数据
ros2 topic echo /serial/receive

# 查看 TF 变换
ros2 run tf2_ros tf2_echo odom gimbal_link

# 列表查看所有话题
ros2 topic list
```

### 常见问题排查

- **无法打开串口**：确认设备路径正确（`ls /dev/ttyUSB*`），确认权限足够，检查是否有其他程序占用串口。
- **数据解析异常**：确认上下位机的协议类型匹配（`protocol` 参数），检查波特率、数据位等串口参数是否一致。
- **模式切换不生效**：检查 `set_mode` 服务是否正常运行（`ros2 service list`），确认视觉节点已启动。
- **虚拟串口无输出**：确认 `vision_mode` 参数已设置，检查是否有节点在接收 `/serial/receive` 话题。
