# rm_interfaces

## 1. 包概述

`rm_interfaces` 是 RoboMaster 自瞄系统的自定义 ROS 消息（Message）和服务（Service）定义包。系统内所有节点间的数据交换均依赖此包定义的数据结构。

包含 16 个 `.msg` 文件和 1 个 `.srv` 文件，涵盖：

- 装甲板检测结果（Armors / Armor）
- 跟踪目标状态（Target）
- 云台控制指令（GimbalCmd）
- 底盘控制指令（ChassisCmd）
- 串口收发数据（SerialReceiveData / Measurement）
- 打符目标（RuneTarget）
- 裁判系统数据（JudgeSystemData）
- 操作手指令（OperatorCommand）
- 调试可视化数据（DebugLights / DebugLight / DebugArmors / DebugArmor / DebugRuneAngle / Point2d）
- 模式切换服务（SetMode）

## 2. 关键变量含义

### 消息定义

#### Armor.msg — 单个装甲板信息

| 字段 | 类型 | 含义 |
|------|------|------|
| `number` | string | 装甲板编号（如 "1", "2", "3", "4" 对应不同编号装甲板） |
| `type` | string | 装甲板类型，如 "small"（小装甲）或 "large"（大装甲） |
| `distance_to_image_center` | float32 | 装甲板中心到图像中心的像素距离 |
| `pose` | geometry_msgs/Pose | 装甲板在相机坐标系下的位姿（由 PnP 解算得到） |

#### Armors.msg — 多装甲板集合

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头（时间戳、坐标系） |
| `armors` | Armor[] | 当前帧检测到的所有装甲板数组 |

#### Target.msg — 跟踪目标状态

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头 |
| `tracking` | bool | 是否正在跟踪目标 |
| `id` | string | 目标编号标识 |
| `armors_num` | int32 | 目标当前可见装甲板数量 |
| `outpost_z_calibrated` | bool | 前哨站三层高度是否已全部观测并完成标定 |
| `outpost_observed_plate` | int32 | 前哨站最近一次可靠观测关联到的物理板索引 |
| `outpost_has_observed_z` | bool | 当前帧是否有被 OutpostTracker 接受的可靠前哨观测高度 |
| `outpost_observed_z` | float64 | 当前可靠观测板高度，用于 Solver 选中观测板时修正击打 Z |
| `outpost_last_seen_time` | float64 | 前哨站最近一次真实观测时间，单位秒 |
| `outpost_phase_error` | float64 | 前哨站最近一次真实观测的相位残差，单位弧度 |
| `position` | geometry_msgs/Point | 目标在目标帧坐标系下的三维位置 (x, y, z) |
| `velocity` | geometry_msgs/Vector3 | 目标三维速度矢量 |
| `yaw` | float64 | 目标航向角 |
| `v_yaw` | float64 | 目标航向角速度 |
| `radius_1` | float64 | 目标旋转半径 1（装甲板安装半径） |
| `radius_2` | float64 | 目标旋转半径 2 |
| `d_za` | float64 | Z 轴偏移量 a |
| `d_zc` | float64 | Z 轴偏移量 c |
| `yaw_diff` | float64 | 当前目标 yaw 与测量值之差 |
| `position_diff` | float64 | 位置残差 |

#### GimbalCmd.msg — 云台控制指令

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头 |
| `pitch` | float64 | 云台 pitch 目标角度（弧度） |
| `yaw` | float64 | 云台 yaw 目标角度（弧度） |
| `yaw_diff` | float64 | yaw 角度增量（相对当前） |
| `pitch_diff` | float64 | pitch 角度增量（相对当前） |
| `distance` | float64 | 目标距离（米） |
| `fire_advice` | bool | 是否建议开火（当云台对准目标且在有效射程内） |

#### SerialReceiveData.msg — 串口接收数据（下位机 → 上位机）

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头 |
| `mode` | uint8 | 当前工作模式（对应 VisionMode 枚举） |
| `bullet_speed_flag` | uint8 | 弹速档位标志（0/1 对应两档射速） |
| `roll` | float32 | 云台 roll 角（弧度，来自下位机 IMU） |
| `yaw` | float32 | 云台 yaw 角（弧度） |
| `pitch` | float32 | 云台 pitch 角（弧度） |
| `judge_system_data` | JudgeSystemData | 裁判系统数据 |

#### Measurement.msg — 测量值（EKF 观测输入）

| 字段 | 类型 | 含义 |
|------|------|------|
| `x` | float64 | X 坐标测量值 |
| `y` | float64 | Y 坐标测量值 |
| `z` | float64 | Z 坐标测量值 |
| `yaw` | float64 | 航向角测量值 |

#### ChassisCmd.msg — 底盘控制指令

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头 |
| `is_spining` | bool | 底盘是否正在旋转 |
| `is_navigating` | bool | 底盘是否正在导航 |
| `twist` | geometry_msgs/Twist | 底盘速度指令（线速度 + 角速度） |

#### DebugLight.msg — 单灯条调试信息

| 字段 | 类型 | 含义 |
|------|------|------|
| `center_x` | int32 | 灯条中心 x 坐标（像素） |
| `is_light` | bool | 是否被判定为有效灯条 |
| `ratio` | float32 | 灯条长宽比 |
| `angle` | float32 | 灯条倾斜角度 |

#### DebugLights.msg — 灯条调试集合

| 字段 | 类型 | 含义 |
|------|------|------|
| `data` | DebugLight[] | 所有检测到的灯条数组 |

#### DebugArmor.msg — 单装甲板调试信息

| 字段 | 类型 | 含义 |
|------|------|------|
| `center_x` | int32 | 装甲板区域中心 x 坐标 |
| `type` | string | 装甲板类型 |
| `light_ratio` | float32 | 配对灯条长度比 |
| `center_distance` | float32 | 灯条中心距 |
| `angle` | float32 | 装甲板倾斜角度 |

#### DebugArmors.msg — 装甲板调试集合

| 字段 | 类型 | 含义 |
|------|------|------|
| `data` | DebugArmor[] | 所有装甲板调试信息数组 |

#### Point2d.msg — 二维点

| 字段 | 类型 | 含义 |
|------|------|------|
| `x` | float32 | X 坐标 |
| `y` | float32 | Y 坐标 |

#### RuneTarget.msg — 打符目标

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头 |
| `pts` | Point2d[5] | 能量机关五个角点 |
| `is_lost` | bool | 是否丢失目标 |
| `is_big_rune` | bool | 是否为大能量机关 |

#### DebugRuneAngle.msg — 打符角度调试

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | std_msgs/Header | 标准 ROS 头 |
| `data` | float64 | 当前解算角度值 |

#### JudgeSystemData.msg — 裁判系统数据

| 字段 | 类型 | 含义 |
|------|------|------|
| `game_status` | uint8 | 比赛状态 |
| `remaining_time` | int16 | 比赛剩余时间 |
| `blood` | int16 | 当前血量 |
| `outpost_hp` | int16 | 前哨站血量 |
| `operator_command` | OperatorCommand | 操作手指令 |

#### OperatorCommand.msg — 操作手指令

| 字段 | 类型 | 含义 |
|------|------|------|
| `is_retreating` | uint8 | 是否正在撤退 |
| `is_drone_avoiding` | uint8 | 是否正在避无人机 |
| `is_outpost_attacking` | uint8 | 是否正在攻击前哨站 |

### 服务定义

#### SetMode.srv — 模式切换服务

```
uint8 mode  # 0: 自瞄红方  1: 自瞄蓝方
---
bool success            # 切换是否成功
string message          # 附加信息
```

### 关键枚举（定义在 `rm_utils` 的 `common.hpp` 中）

#### EnemyColor

| 枚举值 | 数值 | 含义 |
|--------|------|------|
| `RED` | 0 | 敌方为红色 |
| `BLUE` | 1 | 敌方为蓝色 |
| `WHITE` | 2 | 敌方为白色 |

#### VisionMode

| 枚举值 | 数值 | 含义 |
|--------|------|------|
| `AUTO_AIM_RED` | 0 | 自瞄红方 |
| `AUTO_AIM_BLUE` | 1 | 自瞄蓝方 |
| `SMALL_RUNE_RED` | 2 | 小能量机关（红方） |
| `SMALL_RUNE_BLUE` | 3 | 小能量机关（蓝方） |
| `BIG_RUNE_RED` | 4 | 大能量机关（红方） |
| `BIG_RUNE_BLUE` | 5 | 大能量机关（蓝方） |

## 3. 模块逻辑与函数执行流程

`rm_interfaces` 本身为纯数据定义包，不包含运行时逻辑。其使用流程如下：

```
[camera_driver]          [armor_detector]         [armor_solver]          [serial_driver]
     │                        │                        │                      │
     │   image_raw            │                        │                      │
     ├───────────────────────►│                        │                      │
     │                        │  Armors.msg            │                      │
     │                        ├───────────────────────►│                      │
     │                        │                        │  GimbalCmd.msg       │
     │                        │                        ├─────────────────────►│
     │                        │                        │                      │
     │                        │   SerialReceiveData.msg│                      │
     │                        │◄──────────────────────────────────────────────┤
```

所有节点通过 `rm_interfaces` 定义的消息类型进行类型安全的通信。

## 4. 发布和订阅的消息

见第 3 节的流程图。核心消息流：

- `Armors`：由 `armor_detector` 发布，`armor_solver` 订阅
- `Target`：由 `armor_solver` 发布（给调试/可视化节点）
- `GimbalCmd`：由 `armor_solver` 发布，`serial_driver` / `virtual_serial` 订阅
- `SerialReceiveData`：由 `serial_driver` / `virtual_serial` 发布，`armor_solver` 订阅
- `ChassisCmd`：由导航节点发布，`serial_driver` 转发至下位机
- `SetMode`：由外部调用（如键盘控制节点），切换 `armor_solver` 的工作模式

调试消息（当 `debug` 参数为 `true` 时发布）：
- `DebugLights` / `DebugArmors`：由 `armor_detector` 发布，供 RViz 可视化
- `DebugRuneAngle`：由 `rune_solver` 发布

## 5. 调试方法与参数调整

`rm_interfaces` 包无需调试，但可以通过以下方式验证消息定义是否正确：

```bash
# 编译后查看消息定义
ros2 interface package rm_interfaces

# 查看具体消息结构
ros2 interface show rm_interfaces/msg/Armors
ros2 interface show rm_interfaces/msg/GimbalCmd
ros2 interface show rm_interfaces/srv/SetMode

# 在终端 echo 实际发布的消息（运行时）
ros2 topic echo /armor_detector/armors
ros2 topic echo /gimbal_cmd
```

如需新增消息类型，在 `msg/` 或 `srv/` 目录下创建文件，并在 `CMakeLists.txt` 的 `rosidl_generate_interfaces` 中加入新文件名，重新编译即可。
