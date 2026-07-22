# armor_solver

## 1. 包概述

`armor_solver` 是 RoboMaster 自瞄系统的跟踪与解算包。它接收 `armor_detector` 发布的装甲板检测结果，经过 TF 变换到惯性系后，使用 EKF（扩展卡尔曼滤波）进行单目标跟踪，并解算出云台控制指令（Yaw/Pitch）和开火建议。

- **命名空间**: `zfm::auto_aim`
- **节点名**: `armor_solver`
- **节点类型**: `ArmorSolverNode`（组件节点，通过 `rclcpp_components` 注册）

### 核心类

| 类名 | 头文件 | 职责 |
|------|--------|------|
| `Tracker` | `armor_tracker.hpp` | 单目标跟踪器，管理四种状态机（LOST/DETECTING/TRACKING/TEMP_LOST） |
| `OutpostTracker` | `motion_model.hpp` | 前哨站专用跟踪器，圆拟合 + Z 轴标定 + 旋转板预测 |
| `Solver` | `armor_solver.hpp` | 选板、弹道补偿、Yaw/Pitch 解算、开火判断 |
| `Predict` / `Measure` | `motion_model.hpp` | EKF 状态转移与观测函数（模板化，支持 Ceres Jet 自动求导） |
| `RobotStateEKF` | `rm_utils/math/extended_kalman_filter.hpp` | 10 维状态的扩展卡尔曼滤波器 |

### 依赖

- **ROS 包**: `rclcpp`, `rclcpp_components`, `geometry_msgs`, `visualization_msgs`, `message_filters`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `angles`, `rm_interfaces`, `rm_utils`
- **第三方库**: `Eigen3`, `OpenCV`
- **外部接口**: `rm_interfaces::msg::SerialReceiveData`（通过串口接收弹速档位）

---

## 2. 关键变量含义

以下数值以当前 `rm_bringup/config/node_params/armor_solver_params.yaml` 为准。未加载该 YAML 时，节点会使用源码中的声明默认值。

### 跟踪器参数 (tracker.*)

| 参数名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `tracker.max_match_distance` | double | 0.5 | EKF 预测位置与观测位置的最大匹配距离（米），超过则认为不是同一目标 |
| `tracker.max_match_yaw_diff` | double | 1.0 | 预测 Yaw 与观测 Yaw 的最大差值（弧度），超过则认为装甲板发生跳变 |
| `tracker.radius_min/default/max` | double | 0.23/0.26/0.34 | 普通目标半径估计的物理范围和初值（米） |
| `tracker.tracking_confirm_time` | double | 0.03 | DETECTING 连续匹配达到该时间后进入 TRACKING（秒） |
| `tracker.lost_time_thres` | double | 0.3 | TEMP_LOST 状态下允许的最大丢失时间（秒），超过则进入 LOST |

### EKF 噪声参数 (ekf.*)

| 参数名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `ekf.sigma2_q_x` | double | 20.0 | 状态转移噪声方差 — X 方向位置 |
| `ekf.sigma2_q_y` | double | 20.0 | 状态转移噪声方差 — Y 方向位置 |
| `ekf.sigma2_q_z` | double | 20.0 | 状态转移噪声方差 — Z 方向位置 |
| `ekf.sigma2_q_yaw` | double | 100.0 | 状态转移噪声方差 — Yaw 角度 |
| `ekf.sigma2_q_r` | double | 800.0 | 状态转移噪声方差 — 旋转半径 |
| `ekf.sigma2_q_d_zc` | double | 800.0 | 状态转移噪声方差 — Z 方向偏移量 |
| `ekf.r_x` | double | 0.05 | 观测噪声系数 — X（乘以 \|x\| 得到自适应观测噪声） |
| `ekf.r_y` | double | 0.05 | 观测噪声系数 — Y |
| `ekf.r_z` | double | 0.05 | 观测噪声系数 — Z |
| `ekf.r_yaw` | double | 0.02 | 观测噪声系数 — Yaw |
| `ekf.stationary_confirm_time` | double | 0.15 | 低速持续达到该时间后启用静止模型（秒） |
| `ekf.stationary_release_time` | double | 0.05 | 明显运动持续达到该时间后退出静止模型（秒） |
| `ekf.stationary_vel_threshold` | double | 0.05 | 静止模型进入速度阈值 |
| `ekf.stationary_release_vel_threshold` | double | 0.08 | 静止模型退出速度阈值，和进入阈值构成迟滞 |

### 解算器参数 (solver.*)

| 参数名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `solver.bullet_speed` | double | 11.6 | 固定弹丸初速度（m/s） |
| `solver.shooting_range_width` | double | 0.135 | 射击范围的宽度（米），用于计算开火容忍角 |
| `solver.shooting_range_height` | double | 0.135 | 射击范围的高度（米） |
| `solver.max_tracking_v_yaw` | double | 60.0 | 目标 Yaw 角速度超过此值（rad/s）时切换到瞄准中心模式 |
| `solver.center_switch_confirm_time` | double | 0.05 | 装甲板/整车中心模式切换确认时间（秒） |
| `solver.prediction_delay` | double | -0.005 | 视觉、通信等固定延迟修正（秒） |
| `solver.controller_delay` | double | 0.0 | 在弹道预测结果上继续补偿控制器响应延迟，并重新选板 |
| `solver.normal_max_fire_unseen_time` | double | 0.10 | 普通目标开火允许的最大无真实观测时间（秒） |
| `solver.normal_max_aim_unseen_time` | double | 0.30 | 普通目标继续输出瞄准指令的最大无真实观测时间（秒） |
| `solver.compensator_type` | string | "quadratic_drag" | 弹道补偿器类型：`ideal` / `resistance` / `quadratic_drag` |
| `solver.outpost_yaw_offset` | double | 0.0 | 前哨站选板时对旋转方向的 Yaw 偏移量（度） |
| `solver.outpost_max_unseen_time` | double | 0.15 | 前哨站开火允许的最大无真实观测时间（秒） |
| `solver.outpost_max_phase_error` | double | 15.0 | 前哨站开火允许的最大相位残差（度） |
| `solver.side_angle` | double | 45.0 | 普通目标动态侧向选板前量上限（度） |
| `solver.min_switching_v_yaw` | double | 0.5 | 侧向选板前量开始生效的最小角速度（rad/s） |
| `solver.pitch_dead_zone/yaw_dead_zone` | double | 0.1 | 输出命令死区（度） |
| `solver.muzzle.xyz` | double[3] | [0.095, 0, 0] | 枪口在云台坐标系下的位置（米） |
| `solver.gravity` | double | 9.6 | 重力加速度（m/s^2） |
| `solver.drag_coefficient` | double | 0.2 | 二次阻力模型阻力系数 |
| `debug` | bool | true | 是否开启调试模式 |
| `target_frame` | string | "odom" | TF 目标坐标系 |

---

## 3. 模块逻辑与函数执行流程

### 整体流程

```
serialCallback ─── 更新模式标志和下位机云台角度
     │
armorsCallback (收到 Armors 消息，经 tf2_filter 变换到 target_frame)
     │
     ├─ 1. TF 变换: 将每块装甲板位姿从相机系变换到 target_frame
     │
     ├─ 2. 高度过滤: 剔除 |z| > 2.0m 的装甲板
     │
     ├─ 3. Tracker 处理
     │     ├─ LOST 状态 → tracker.init(): 选离图像中心最近的装甲板，初始化 EKF
     │     │     └─ 若编号为 "outpost" → 创建 OutpostTracker
     │     ├─ DETECTING/TRACKING/TEMP_LOST → tracker.update():
     │     │     ├─ 普通装甲板: EKF 预测 → 最近邻匹配 → EKF 更新
     │     │     └─ 前哨站: OutpostTracker.addMeasurement → 圆拟合/状态更新
     │     └─ 发布 Measurement + Target 消息
     │
     └─ 4. target_pub_ 发布 Target 消息

timerCallback (4ms 定时器)
     │
     ├─ Solver::solve (收到最新的 Target)
     │     ├─ 使用节点启动时缓存的参数
     │     ├─ 查询 gimbal_link 位姿 (获取当前云台 RPY)
     │     ├─ 弹道飞行时间计算 + 提前量预测 (位置/yaw)
     │     ├─ getArmorPositions: 根据 target 状态计算各装甲板位置
     │     ├─ selectBestArmor: 选择最优装甲板
     │     │     ├─ 4 装甲板: 基于决策角度 + 旋转方向选择
     │     │     ├─ 3 装甲板(前哨站): 可靠观测锁当前板；短暂无可靠观测时才按模型预选下一板
     │     │     └─ 角速度超限 → TRACKING_CENTER 状态，瞄准中心
     │     ├─ calcYawAndPitch: 含弹道补偿
     │     ├─ isOnTarget: 判断是否在射击范围内
     │     └─ 手动补偿 (ManualCompensator)
     │
     └─ 发布 GimbalCmd (yaw_diff, pitch_diff, fire_advice, distance)
```

### EKF 状态定义

状态向量（10 维）：

```
x = [xc, vxc, yc, vyc, zc, vzc, yaw, vyaw, r, d_zc]^T
```

其中：
- `(xc, yc, zc)` — 目标旋转中心的三维位置
- `(vxc, vyc, vzc)` — 对应速度
- `yaw` — 目标朝向角
- `vyaw` — Yaw 角速度
- `r` — 旋转半径（目标中心到装甲板平面的距离）
- `d_zc` — 装甲板高度偏移

观测向量（4 维）：

```
z = [xa, ya, za, yaw]^T
```

观测模型：
```
xa = xc - r * cos(yaw)
ya = yc - r * sin(yaw)
za = zc + d_zc
```

### 运动模型 (MotionModel)

| 模型 | 适用场景 | 状态转移行为 |
|------|----------|-------------|
| `CONSTANT_VEL_ROT` | 普通装甲板 | 位置保持匀速，Yaw 保持匀速旋转 |
| `CONSTANT_VELOCITY` | 一般情况 | 仅位置匀速，Yaw 速度置零 |
| `CONSTANT_ROTATION` | 前哨站 | 位置速度置零，Yaw 保持匀速旋转 |
| `CONSTANT_STATIONARY` | 普通静止目标 | 位置速度和 Yaw 速度置零 |

### 跟踪器状态机

```
LOST ──(init)──→ DETECTING ──(连续匹配 tracking_confirm_time)──→ TRACKING
                    ↑                                              │
                    │                                         (匹配失败)
                    │                                              ↓
                    │                                          TEMP_LOST
                    │                                              │
                    └──(lost_time_thres 超时)────────────────────←──┘
```

### 前哨站专用跟踪 (OutpostTracker)

前哨站检测器独立于主 EKF，采用几何圆拟合方法：

1. **COLLECTING 阶段**: 至少观察 0.4 秒，通过固定半径 `fixed_R_ = 0.275m` 和观测角度反推旋转中心 `(cx, cy)`，判断是否有旋转运动
2. **CALIBRATING 阶段**: 保持不可跟踪、不可开火，完成三个装甲板的 Z 聚类和物理板号映射
3. **ACTIVE 阶段**: Z 标定完成后才进入；可靠观测用于修正模型并锁定当前观测板，短暂无可靠观测时才按模型外推和预选下一板
4. **STATIC 阶段**: 0.4 秒观察窗口内位移不足 0.05m 时进入静态模式；Z 未标定时仍禁止开火
5. 暂时丢失观测时只允许有限时长的相位外推；开火还必须同时满足观测新鲜度和相位残差限制

前哨 Solver 的选板原则与 `autoaim` 一致：看得到可靠当前板时选择当前板，并使用 `outpost_observed_z` 修正击打高度；切板间隙看不到可靠观测时，只在上一块观测板和旋转顺序的下一块板之间外推选择。连续 1.0 秒没有可用观测后销毁模型，下次重新收集。

### Solver 状态切换

Solver 有两种状态：
- `TRACKING_ARMOR`: `|vyaw| > max_tracking_v_yaw` 连续达到 `center_switch_confirm_time` 后切换到 `TRACKING_CENTER`
- `TRACKING_CENTER`: `|vyaw|` 恢复正常并持续达到同一确认时间后切回 `TRACKING_ARMOR`；该状态禁止开火

### 弹道补偿器 (TrajectoryCompensator)

三种类型的行为差异：

| 类型 | pitch 补偿 | 飞行时间获取 | 性能 |
|------|-----------|-------------|------|
| `ideal` | 无阻力迭代 | `t = d / (v·cos(atan2(z,d)))` | 每次调用最多 20 次迭代 |
| `resistance` | 阻力模型迭代 | `t = (e^(r·d)-1) / (r·v·cos(atan2(z,d)))` | 同上 |
| `quadratic_drag` | 二次阻力数值积分与迭代 | 数值积分到目标水平距离 | 当前默认，适合使用弹丸物理参数标定 |

---

## 4. 发布和订阅的消息

### 订阅话题

| 话题名 | 消息类型 | QoS | 说明 |
|--------|----------|-----|------|
| `armor_detector/armors` | `rm_interfaces::msg::Armors` | SensorData | 经 `tf2_filter` 转换到 `target_frame` 的装甲板数据 |
| `serial/receive` | `rm_interfaces::msg::SerialReceiveData` | SensorData | 下位机串口上传的模式标志和云台状态 |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `armor_solver/target` | `rm_interfaces::msg::Target` | 跟踪目标的状态估计（位置、速度、Yaw、角速度、半径等） |
| `armor_solver/measurement` | `rm_interfaces::msg::Measurement` | EKF 的输入观测量（经最近邻匹配后的装甲板位置和 Yaw） |
| `armor_solver/cmd_gimbal` | `rm_interfaces::msg::GimbalCmd` | 云台控制指令（Yaw/Pitch 偏差、开火建议、距离） |

### 调试话题 (仅 debug=true 时)

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `armor_solver/marker` | `visualization_msgs::msg::MarkerArray` | RViz 可视化标记（目标位置球、速度箭头、角速度箭头、装甲板 Cubes、弹道轨迹、选中标记；前哨站模式下含旋转圆圈、打击预测、状态文字等） |

### 服务

| 服务名 | 服务类型 | 说明 |
|--------|----------|------|
| `armor_solver/set_mode` | `rm_interfaces::srv::SetMode` | 切换模式：`AUTO_AIM_RED` / `AUTO_AIM_BLUE` 启用，其他模式禁用 |

---

## 5. 调试方法与参数调整

### 性能调优

- **关闭调试模式**：将 `debug` 设为 `false` 可关闭高频调试输出、PlotJuggler 和 RViz marker；启动、状态切换和错误日志仍会保留。
- **预测/控制延迟** (`prediction_delay`, `controller_delay`)：两者都会影响最终板位；控制延迟在已有弹道预测结果上继续外推，并重新选板。
- **EKF 噪声参数**：若目标运动剧烈可适当增大 `sigma2_q_*` 系列参数；若观测噪声较大可增大 `r_*` 参数。

### 常用调试方法

1. 在 RViz 中订阅 `armor_solver/marker` 观察目标状态和弹道轨迹

普通装甲（NORMAL_4）tracking 时：

  ┌─────────────────┬────────────┬─────────────────────────┬─────────────────┬───────────────────────────────────────────────────┐
  │    namespace    │    类型    │          颜色           │      大小       │                       含义                        │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ raw_meas        │ SPHERE     │ 黄 (1.0, 0.85, 0)       │ 0.065           │ 检测器原始观测的装甲板位置                        │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ obs_center      │ SPHERE     │ 橙 (1.0, 0.55, 0)       │ 0.05            │ 观测值反推的机器人中心                            │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ obs_ekf_diff    │ LINE_STRIP │ 橙 (1.0, 0.5, 0)        │ 0.015           │ 观测中心 → EKF 中心连线（滤波残差）               │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ position        │ SPHERE     │ 绿 (0, 1, 0)            │ 0.1             │ EKF 预测的机器人中心                              │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ linear_v        │ ARROW      │ 红+绿 (1, 1, 0)         │ -               │ 线速度矢量（大小=箭头长度）                       │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ angular_v       │ ARROW      │ 蓝+绿 (0, 1, 1)         │ -               │ 角速度矢量（z方向=vyaw/π）                        │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ filtered_armors │ CUBE       │ 蓝 (0, 0, 1)            │ 0.03×type×0.125 │ EKF 预测的四块装甲板（type=small:0.135, 大:0.23） │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ trajectory      │ POINTS     │ 白 (1,1,1) / 绿 (0,1,0) │ 0.01            │ 选中装甲板的弹道（开火建议=true时变绿）           │
  ├─────────────────┼────────────┼─────────────────────────┼─────────────────┼───────────────────────────────────────────────────┤
  │ selection       │ SPHERE     │ 黄 (1, 1, 0)            │ 0.1             │ 当前瞄准点                                        │
  └─────────────────┴────────────┴─────────────────────────┴─────────────────┴───────────────────────────────────────────────────┘

  前哨站（OUTPOST_3）tracking 时：

  ┌─────────────────┬──────────────────┬──────────────────────┬──────┬───────────────────────────────────────────┐
  │    namespace    │       类型       │         颜色         │ 大小 │                   含义                    │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ state           │ TEXT_VIEW_FACING │ 白 (1,1,1)           │ 0.4  │ 状态文字：COLLECTING / CALIBRATING / STATIC / ACTIVE │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ raw_meas        │ SPHERE           │ 粉紫 (1.0, 0.2, 0.8) │ 0.06 │ 检测器原始观测的装甲板位置                │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ circle_z0/z1/z2 │ LINE_STRIP       │ 三层绿色渐变         │ 0.01 │ 三层高度拟合圆                            │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ center          │ SPHERE           │ 黄 (1.0, 0.8, 0)     │ 0.05 │ 圆拟合中心                                │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ rotation        │ ARROW            │ 橙 (1.0, 0.6, 0)     │ -    │ 旋转方向和速度                            │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ plates_now      │ SPHERE_LIST      │ 青 (0.5, 1.0, 1.0)   │ 0.05 │ 当前时刻三块装甲板预测位置                │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ strike_pred     │ SPHERE_LIST      │ 青 (0, 0.9, 0.9)     │ 0.07 │ 弹丸飞行时间后的装甲板预测位置            │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ strike_best     │ SPHERE           │ 红 (1.0, 0, 0)       │ 0.1  │ 最佳击打装甲板（strike_pred中最优的一个） │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ static_mode     │ TEXT_VIEW_FACING │ 灰 (0.5, 0.5, 0.5)   │ 0.5  │ 前哨站静止模式文字                        │
  ├─────────────────┼──────────────────┼──────────────────────┼──────┼───────────────────────────────────────────┤
  │ trajectory      │ POINTS           │ 白 (1, 1, 1)         │ 0.01 │ 选中装甲板的弹道                          │
  └─────────────────┴──────────────────┴──────────────────────┴──────┴───────────────────────────────────────────┘
2. 查看 `/armor_solver/target` 消息内容确认跟踪状态是否正确
3. 查看 `/armor_solver/cmd_gimbal` 消息中的 `fire_advice` 字段判断开火时机
4. `debug=true` 时观察 `[Solver]`、`[Outpost]` 调试日志分析选板、预测和开火门控
5. 前哨站模式下关注 `[Outpost]` 和 `[OutpostTracker]` 的调试输出，观察圆拟合和 Z 轴标定状态

### 前哨站参数调整

- `solver.outpost_yaw_offset`：前哨站旋转时，对选定板位施加一个额外的 Yaw 超前角偏移，补偿弹丸飞行时间
- `solver.outpost_max_unseen_time`：限制无真实观测时的开火新鲜度
- `solver.outpost_max_phase_error`：限制模型相位与观测相位的误差
- Z 标定完成前不进入正常三板预测，不切板、不建议开火
- 可靠观测存在时不触发提前切板；提前切换只属于切板间隙的模型外推

### 参数生效时机

Solver 参数在节点创建时声明并缓存。修改 YAML 后需要重启 `armor_solver` 节点；当前实现不会在每帧重新读取参数。
