# AutoAim 纯 C++ 自瞄系统 使用手册

## 1. 项目结构

```
autoaim/
├── CMakeLists.txt              # 顶层构建文件
├── USE.md                      # 本文档
├── configs/
│   └── standard.yaml           # 主配置文件（所有参数）
├── models/
│   ├── lenet.onnx              # 数字分类模型 (ONNX)
│   ├── mlp.onnx                # 备用数字分类模型 (ONNX)
│   └── label.txt               # 分类标签
├── utils/                      # 工具库
│   ├── math.hpp/cpp            # 欧拉角/四元数/矩阵/坐标转换
│   ├── pnp.hpp/cpp             # IPPE PnP 解算器
│   ├── ekf.hpp/cpp             # 扩展卡尔曼滤波器 (Ceres/解析回退)
│   ├── trajectory.hpp/cpp      # 弹道模型 (ideal + quadratic_drag)
│   ├── manual_compensator.hpp/cpp  # 手动 2D 补偿表
│   ├── plotter.hpp/cpp         # PlotJuggler UDP 实时曲线
│   ├── recorder.hpp/cpp        # 视频 + IMU 同步录制
│   ├── logger.hpp/cpp          # spdlog 日志封装
│   └── exiter.hpp/cpp          # 按 q 键退出
├── hardware/                   # 硬件抽象层
│   ├── camera.hpp/cpp          # 海康 MVS 相机驱动
│   ├── video_player.hpp/cpp    # 离线视频回放
│   └── serial.hpp/cpp          # UART 16 字节串口协议
├── src/                        # 核心流水线
│   ├── main.cpp                # 主程序入口
│   ├── armor.hpp/cpp           # 数据结构 (Armor/Target/GimbalCommand)
│   ├── classifier.hpp/cpp      # ONNX 数字分类器
│   ├── detector.hpp/cpp        # 传统 CV 检测器
│   ├── solver.hpp/cpp          # PnP + 坐标变换 + 瞄准 + 弹道 + 击发
│   ├── tracker.hpp/cpp         # EKF 跟踪 + 前哨站几何跟踪 + 状态机
│   └── target.hpp/cpp          # 前哨站 OutpostTracker
└── tests/
    └── offline_test.cpp        # 离线视频测试（不连硬件）
```

---

## 2. 编译

### 2.1 必备依赖

| 依赖 | 用途 |
|---|---|
| OpenCV 4.x | 图像处理、imshow 可视化 |
| Eigen3 | 矩阵/向量运算 |
| fmt | 格式化输出 |
| spdlog | 日志 |
| yaml-cpp | 配置文件解析 |

### 2.2 可选依赖

| 依赖 | 用途 | 没有的后果 |
|---|---|---|
| ONNX Runtime | 数字分类器 (LeNet/MLP) | 自动回退到 OpenCV DNN |
| Ceres Solver | EKF 自动微分求雅可比 | 自动回退到解析雅可比 |
| 海康 MVS SDK | 相机驱动 | 相机不可用，只能用视频回放 |

### 2.3 编译命令

```bash
cd autoaim
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

---

## 3. 配置文件说明 (configs/standard.yaml)

### 3.1 相机参数
```yaml
camera:
  device_index: 0          # 设备序号，多相机时选
  exposure_time: 2500.0    # 曝光时间 (μs)
  gain: 12.0               # 增益
  frame_rate: -1.0          # -1 = 最大帧率自动
```

### 3.2 离线回放（调试用）
```yaml
video:
  enable: false             # true = 读视频文件代替相机
  path: "test.avi"          # 视频路径
  fps: 110.0                # 回放帧率
  loop: false               # 循环播放
```

### 3.3 外参
```yaml
odom2camera:
  xyz: [0.246, 0.0, 0.049]   # 相机在云台系下的位置 (米)
  rpy: [0.0, 0.275, 0.015]   # 相机在云台系下的姿态 (弧度)
```

### 3.4 检测参数
```yaml
detector:
  detect_color: 1           # 0=红色方 1=蓝色方
  binary_thres: 90          # 二值化阈值，光线暗就调低

  light:                    # 灯条过滤
    min_ratio: 0.0001       # 最小面积比
    max_ratio: 1.0          # 最大面积比
    max_angle: 40.0         # 最大倾斜角 (度)
    color_diff_thresh: 20   # 颜色区分阈值

  armor:                    # 装甲板配对
    min_light_ratio: 0.8    # 最小灯条长度比
    min_small_center_distance: 0.8   # 小装甲板最小灯条间距/灯条长
    max_small_center_distance: 3.5
    min_large_center_distance: 3.5   # 大装甲板最小灯条间距/灯条长
    max_large_center_distance: 8.0
    max_angle: 35.0         # 最大配对角度差

  classifier_threshold: 0.7  # 数字分类置信度阈值
  ignore_classes: ["negative"]
  classifier_model: "../models/lenet.onnx"
  classifier_label: "../models/label.txt"
```

### 3.5 跟踪参数
```yaml
tracker:
  max_match_distance: 0.5   # EKF 预测与观测的最大匹配距离 (米)
  max_match_yaw_diff: 1.0   # 最大匹配 yaw 角差 (弧度)
  radius_min: 0.23
  radius_max: 0.34
  default_radius: 0.26
  tracking_confirm_time: 0.03       # 连续匹配多久后进入 TRACKING (秒)
  lost_time: 0.30                   # 连续丢失多久后进入 LOST (秒)
  stationary_confirm_time: 0.15     # 低速持续多久后启用静止模型 (秒)
  stationary_release_time: 0.05     # 明显运动持续多久后退出静止模型 (秒)
  stationary_vel_threshold: 0.05
  stationary_release_vel_threshold: 0.08
```

### 3.6 解算 & 弹道
```yaml
solver:
  prediction_delay: -0.005       # 预测延迟补偿 (秒)
  controller_delay: 0.0          # 控制器延迟 (秒)
  outpost_yaw_offset: 0.0        # 前哨站 yaw 偏置 (度)
  outpost_max_unseen_time: 0.15  # 前哨站开火允许的最大无真实观测时间
  outpost_max_phase_error: 15.0  # 前哨站开火允许的最大相位残差 (度)
  normal_max_fire_unseen_time: 0.10 # 普通目标开火允许的最大无观测时间
  center_switch_confirm_time: 0.05  # 装甲/中心模式切换确认时间
  max_tracking_v_yaw: 60.0       # 超过此转速瞄中心 (rad/s)
  min_switching_v_yaw: 0.5       # 侧向前量生效的最小转速
  side_angle: 45.0               # 侧向前量最大值 (度)
  shooting_range_width: 0.135    # 击打窗口宽度 (米)
  shooting_range_height: 0.135   # 击打窗口高度 (米)
  pitch_dead_zone: 0.1           # 输出 pitch 死区 (度)
  yaw_dead_zone: 0.1             # 输出 yaw 死区 (度)
  muzzle:
    xyz: [0.095, 0.0, 0.0]      # 枪口在云台系下的位置 (米)

  bullet_speed: 11.6             # 固定弹速 (m/s)

  compensator_type: quadratic_drag  # ideal / quadratic_drag
  gravity: 9.6
  # quadratic_drag 参数
  drag_coefficient: 0.2
  air_density: 1.225
  projectile_mass: 0.0445        # 弹丸质量 (kg)
  projectile_diameter: 0.0425    # 弹丸直径 (m)

  # 手动补偿表: [距离下限 距离上限 高度下限 高度上限 pitch补偿(°) yaw补偿(°)]
  angle_offset:
    - "4.0 5.0 -1.0 0.4 0.0 0.0"
    - "5.0 6.0 -1.0 0.4 0.0 0.0"
    # ... 更多行
```

### 3.7 串口
```yaml
serial:
  port_name: "/dev/ttyACM_mcu"   # 串口设备
  baudrate: 115200               # 波特率
```

### 3.8 调试开关
```yaml
debug:
  show_image: true        # cv::imshow 显示检测结果
  enable_plotter: true    # PlotJuggler UDP 发送
  enable_record: false    # 视频录制
  record_fps: 110         # 录制帧率
  log_level: "info"       # trace/debug/info/warn/error
```

---

## 4. 使用方法

### 4.1 实时运行（连硬件）

```bash
# 确认串口连接
ls /dev/ttyACM*

# 运行
./build/autoaim configs/standard.yaml
```

### 4.2 离线回放调试（不连硬件）

修改 `configs/standard.yaml`:
```yaml
video:
  enable: true
  path: "/path/to/recorded.avi"
```

```bash
./build/autoaim configs/standard.yaml
```
./autoaim ../configs/standard.yaml

### 4.3 离线批量测试

```bash
./build/offline_test configs/standard.yaml
```
输出每帧检测数、跟踪状态、总体统计。

---

## 5. 调试方法

### 5.1 图像级调试
运行过程中弹出 OpenCV 窗口，实时显示：
- 绿色框：检测到的装甲板灯条
- 白色文字：分类结果和置信度
- 左上角：当前跟踪状态 (LOST / DETECTING / TRACKING)

**怎么调**：打开 `debug.show_image: true`，观察灯条提取是否完整、数字识别是否正确。

### 5.2 曲线调试 (PlotJuggler)
```bash
# 先启动 PlotJuggler
plotjuggler

# UDP 数据会自动出现在 Data 列表，拖到右侧即可绘图
```
发送的字段：
| 字段 | 含义 |
|---|---|
| `dist` | 目标距离 (m) |
| `yaw` | EKF 估计的目标 yaw 角 (°) |
| `v_yaw` | EKF 估计的目标角速度 (rad/s) |
| `cmd_yaw` | 发送给云台的 yaw 指令 (°) |
| `cmd_pitch` | 发送给云台的 pitch 指令 (°) |
| `fire` | 击发信号 (0/1) |

**怎么调**：打开 `debug.enable_plotter: true`，观察 `dist` 是否平滑、`v_yaw` 是否稳定、`fire` 是否在正确时机触发。

### 5.3 录制回放
```bash
# 启用录制（config: enable_record: true），运行后会在 records/ 目录生成
# - YYYY-MM-DD_HH-MM-SS.avi  （视频）
# - YYYY-MM-DD_HH-MM-SS.txt  （每帧时间戳 + IMU 四元数）

# 回放：修改 video 配置指向录制的文件
```

### 5.4 日志
日志同时输出到控制台和 `logs/autoaim.log`：
```yaml
debug.log_level: "debug"   # 打印更多细节
```
关键日志标记：
- `[Tracker]` — 跟踪状态切换
- `[Outpost]` — 前哨站圆拟合和状态切换
- `[Solver]` — 弹道解算结果

---

## 6. 串口协议

### TX (→下位机) 16 字节
```
[0]  0xFF      帧头
[1]  uint8     fire (0x01=开火, 0x00=不开火)
[2-5]  float32  pitch (度)
[6-9]  float32  yaw (度)
[10-13] float32 distance (米)
[14]  uint8     CRC (异或校验)
[15]  0x0D      帧尾
```

### RX (←下位机) 16 字节
```
[0]  0xFF      帧头
[1]  uint8     mode (0=idle, 1=auto_aim)
[2-5]  float32  IMU roll (rad)
[6-9]  float32  IMU pitch (rad)
[10-13] float32 IMU yaw (rad)
[14]  uint8     保留字段
[15]  0x0D      帧尾
```

---

## 7. 常见调参场景

### 7.1 检测不到装甲板
- 增大 `binary_thres` 或减小（看二值化效果）
- 检查 `detect_color` 是否正确
- 调 `light.max_angle` 和 `armor.max_angle`
- 如果灯条提取太碎，放宽 `light.min_ratio`

### 7.2 跟踪容易丢失
- 适当增大 `tracker.lost_time`
- 若真实观测经常匹配不上，适当增大 `tracker.max_match_distance`；若容易串目标则减小
- 检查 PnP 输出是否跳变，以及 `max_match_yaw_diff` 是否过小
- 状态或命令出现非有限值时会被丢弃/回滚，不会继续下发

### 7.3 打不中
- **先确认弹速**：`solver.bullet_speed` 要与实际一致
- **调重力**：根据实测落点微调 `gravity`
- **添加补偿表**：在 `angle_offset` 中按距离/高度添加补偿值
- 增大 `shooting_range_width/height`（放宽击发窗口）

### 7.4 前哨站瞄不准
- 先确认已完成 Z 标定；`COLLECTING/CALIBRATING` 阶段不会切板或开火
- 检查 `outpost_yaw_offset`（正值 = 向反旋转方向增加补偿）
- 检查 `max_tracking_v_yaw`（转速超过此值会自动瞄中心）
