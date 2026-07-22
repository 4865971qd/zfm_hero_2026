# rm_bringup

## 1. 包概述

`rm_bringup` 是 RoboMaster 自瞄系统的启动与配置包。负责：

- 定义系统启动描述文件 `bringup.launch.py`，编排所有节点的启动顺序与生命周期
- 提供两级配置体系：顶层 `launch_params.yaml` 控制节点启停逻辑，`node_params/` 目录存放各节点的独立参数文件
- 利用 `ComposableNodeContainer`（多线程容器）实现 `camera_driver` 与 `armor_detector` 之间的进程内通信（intra-process communication），避免图像数据的序列化/反序列化开销

## 2. 关键变量含义

### launch_params.yaml 启动参数

| 参数 | 类型 | 默认值 | 含义 |
|------|------|--------|------|
| `odom2camera.xyz` | string | `"0.246 0.0 0.049"` | 相机在 gimbal 坐标系下的安装位置 (x y z, 米)，传递给 URDF xacro |
| `odom2camera.rpy` | string | `"0.00 0.27 -0.02"` | 相机在 gimbal 坐标系下的安装姿态 (roll pitch yaw, 弧度) |
| `namespace` | string | `""` | ROS 命名空间，所有节点/话题都将以此开头 |
| `video_play` | bool | `false` | `true` 时使用 `VideoPlayerNode` 回放录制的视频；`false` 时驱动真实相机（海康相机） |
| `virtual_serial` | bool | `false` | `true` 时启动 `virtual_serial_node`（虚拟下位机）；`false` 时连接真实串口 |
| `hero_solver` | bool | `false` | `true` 时使用英雄解算节点 `hero_armor_solver_node`；`false` 时使用传统步兵解算节点 |
| `rune` | bool | `false` | `true` 时开启打符模式（传递给 virtual_serial 的 `has_rune` 参数） |
| `navigation` | bool | `false` | `true` 时额外启动导航 TF（sentry.urdf.xacro） |

### node_params/ 各节点参数文件

| 文件 | 对应节点 | 关键参数 |
|------|----------|----------|
| `camera_driver_params.yaml` | `camera_driver` | `camera_name`（相机型号）、`ip_address`、`serial_number`、`port`、`exposure_time`、`camera_calibration_file` |
| `video_player_params.yaml` | `video_player` | `path`（视频路径）、`frame_rate`、`start_frame`、`keep_looping`、`frame_id` |
| `serial_driver_params.yaml` | `serial_driver` | `port_name`（串口设备路径）、`protocol`（协议类型）、`timestamp_offset`、`enable_data_print` |
| `virtual_serial_params.yaml` | `virtual_serial` | `roll`、`pitch`、`yaw`（模拟姿态角）、`vision_mode`（视觉模式枚举值） |
| `armor_detector_params.yaml` | `armor_detector` | `detect_color`（敌方颜色）、`binary_thres`（二值化阈值）、`use_pca`/`use_ba`（优化开关）、灯条/装甲板筛选阈值、`classifier_threshold`（分类器阈值） |
| `armor_solver_params.yaml` | `armor_solver` | EKF 噪声参数（`sigma2_q_*`/`r_*`）、跟踪器匹配阈值、弹道补偿参数（`bullet_speed_0/1`、`gravity`、`resistance`、`iteration_times`）、角度硬补偿表 `angle_offset` |

## 3. 模块逻辑与函数执行流程

### 启动流程（bringup.launch.py）

```
1. 加载 launch_params.yaml
2. 构造 xacro 命令，生成机器人描述
   ├── rm_gimbal.urdf.xacro（必选）— 云台 + 相机 TF
   └── sentry.urdf.xacro（可选，由 navigation 参数控制）
3. 启动 robot_state_publisher × 2（gimbal + 可选 navigation）
4. 创建 PushRosNamespace（若 namespace 非空）
5. 以 TimerAction 延迟启动以下节点：
   ├── 1.5s → serial_driver / virtual_serial（串口通信）
   ├── 2.0s → ComposableNodeContainer（camera_detector_container）
   │           ├── camera_driver / video_player（图像源）
   │           └── armor_detector（装甲板检测，利用 intras process comm）
   └── 2.0s → armor_solver（PnP 解算 + EKF 跟踪，普通 Node）
```

### 节点依赖关系

```
camera_driver ──(intra)──→ armor_detector ──→ armor_solver
                                                      ↑
serial_driver ─────────────────────────────────────────┘
```

- `camera_driver` 发布原始图像到容器内，`armor_detector` 通过 intra-process 零拷贝获取图像
- `armor_detector` 检测到装甲板后发布目标消息，`armor_solver` 接收后进行 PnP 解算、卡尔曼滤波跟踪和弹道补偿，最终通过 `serial_driver` 将云台控制指令发送至下位机

## 4. 发布和订阅的消息

### 各节点消息接口

| 节点 | 发布话题 | 订阅话题 |
|------|----------|----------|
| `camera_driver` / `video_player` | `image_raw` (sensor_msgs/Image), `camera_info` (sensor_msgs/CameraInfo) | — |
| `armor_detector` | `/armor_detector/armors` (rm_interfaces/Armors), 调试话题 | `image_raw`, `camera_info` |
| `armor_solver` | `gimbal_cmd` (rm_interfaces/GimbalCmd), 调试话题 | 检测结果, 串口接收数据 |
| `serial_driver` | `serial_receive_data` (rm_interfaces/SerialReceiveData) | `gimbal_cmd` |
| `virtual_serial` | `serial_receive_data` (模拟数据) | `gimbal_cmd` |
| `robot_state_publisher` | `tf` (TF树) | `robot_description` |

## 5. 调试方法与参数调整

### 启动模式切换

```bash
# 使用视频回放（离线调试）
# 修改 launch_params.yaml: video_play: true
# 并在 video_player_params.yaml 中设置正确的视频路径

# 使用虚拟串口（无下位机调试）
# 修改 launch_params.yaml: virtual_serial: true

# 英雄模式
# 修改 launch_params.yaml: hero_solver: true
```

### 检测参数调优

在 `armor_detector_params.yaml` 中：

- `binary_thres`：调整二值化阈值，影响灯条提取的灵敏度
- `light.min_ratio` / `light.max_ratio`：灯条长宽比筛选范围
- `light.max_angle`：灯条最大倾斜角度（度）
- `armor.min_small_center_distance` / `armor.max_small_center_distance`：小装甲板灯条中心距范围
- `armor.min_large_center_distance` / `armor.max_large_center_distance`：大装甲板灯条中心距范围
- `classifier_threshold`：数字分类器置信度阈值
- `ignore_classes`：需要忽略的分类类别（如 `["negative"]`）

### 解算参数调优

在 `armor_solver_params.yaml` 中：

- `ekf.*`：卡尔曼滤波的噪声协方差矩阵参数
- `tracker.*`：目标匹配与丢失跟踪参数
- `solver.prediction_delay` / `solver.controller_delay`：预测延时补偿（秒）
- `solver.bullet_speed_0` / `solver.bullet_speed_1`：两种射速档位（m/s）
- `solver.compenstator_type`：弹道补偿模式，`"resistance"`（带空气阻力）或 `"ideal"`（无阻力）
- `solver.gravity`：重力加速度（m/s²，可调以适配实际弹道）
- `solver.resistance`：空气阻力系数
- `solver.iteration_times`：弹道补偿迭代次数
- `solver.angle_offset`：角度硬补偿查找表，格式为 `"距离下限 距离上限 高度下限 高度上限 pitch补偿 yaw补偿"`

### 查看调试信息

在 `armor_detector_params.yaml` 和 `armor_solver_params.yaml` 中将 `debug` 设为 `true`，节点会发布对应的可视化调试话题（如 `/debug_armors`、`/debug_lights` 等），可在 RViz 或 rqt_image_view 中查看。
