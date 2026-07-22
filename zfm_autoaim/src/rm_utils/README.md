# rm_utils

## 1. 包概述

`rm_utils` 是 RoboMaster 自瞄系统的工具库，提供自瞄流水线所需的各类算法与基础设施组件。以静态/共享库形式供 `armor_detector`、`armor_solver` 等包链接。

编译生成两个库：
- `rm_utils`：核心算法库
- `zfmlogger`：独立日志库

主要组件包括：

| 组件 | 命名空间 | 功能 |
|------|----------|------|
| Logger（日志） | `zfm::logger` | 基于 fmt 的带颜色/等级/时间分文件的日志系统 |
| ExtendedKalmanFilter（扩展卡尔曼滤波） | `zfm` | 模板化 EKF，使用 Ceres Jet 自动求导雅可比 |
| TrajectoryCompensator（弹道补偿） | `zfm` | 子弹飞行时间计算与弹道下坠补偿 |
| ManualCompensator（手动补偿） | `zfm` | 基于距离-高度二维查表的 yaw/pitch 手动硬补偿 |
| PnPSolver（PnP 解算） | `zfm` | OpenCV solvePnP / solvePnPGeneric 封装 |
| Math Utils（数学工具） | `zfm::utils` | 欧拉角 ↔ 旋转矩阵转换、角度归一化、Eigen↔OpenCV 互转 |
| HeartBeatPublisher（心跳发布） | `zfm` | 节点健康状态监控，定时发布 Int64 消息 |
| URLResolver（URL 解析） | `zfm::utils` | 解析 `package://` 类型路径为实际文件系统路径 |
| Assert（断言） | 全局宏 | ZFM_ASSERT / ZFM_ASSERT_MSG 断言宏 |
| Common Enums（公共枚举） | `zfm` | VisionMode（6 种视觉模式）、EnemyColor（3 种颜色） |

## 2. 关键变量含义

### Logger（日志系统）

日志等级：`DEBUG` (灰色)、`INFO` (白色)、`WARN` (黄色)、`ERROR` (红色)、`FATAL` (蓝色)

可用宏：

| 宏 | 功能 |
|----|------|
| `ZFM_REGISTER_LOGGER(name, path, level)` | 注册一个名为 name 的日志器，保存到 path，可附加日期目录和日期后缀 |
| `ZFM_LOG(name, level, ...)` | 以指定等级输出日志 |
| `ZFM_DEBUG(name, ...)` | DEBUG 等级日志 |
| `ZFM_INFO(name, ...)` | INFO 等级日志 |
| `ZFM_WARN(name, ...)` | WARN 等级日志 |
| `ZFM_ERROR(name, ...)` | ERROR 等级日志 |
| `ZFM_FATAL(name, ...)` | FATAL 等级日志 |

日志选项宏：`DEFAULT_OPTIONS`、`DATE_DIR`（按日期创建子目录）、`DATE_SUFFIX`（文件名加日期后缀）、`OVER_WRITE`（覆盖写入）

### ExtendedKalmanFilter（扩展卡尔曼滤波）

模板参数：`<N_X, N_Z, PredicFunc, MeasureFunc>`

- `N_X`：状态向量维度
- `N_Z`：观测向量维度
- `PredicFunc`：非线性过程模型函数 `f(x) -> x'`
- `MeasureFunc`：非线性观测模型函数 `h(x) -> z`
- `Q`：过程噪声协方差矩阵（通过 `update_Q` 回调动态计算）
- `R`：观测噪声协方差矩阵（通过 `update_R` 回调动态计算，可依赖当前观测值 `z`）
- `P`：误差协方差矩阵

核心方法：
- `predict()`：状态预测，利用 Ceres Jets 自动计算雅可比矩阵 F
- `update(z)`：状态更新，利用 Ceres Jets 自动计算观测雅可比矩阵 H

### TrajectoryCompensator（弹道补偿）

基类提供：
- `compensate(target_position, pitch)`：对目标位置迭代求解 pitch 增量，补偿弹道下坠
- `getTrajectory(distance, angle)`：获取指定距离和初射角下的弹道轨迹

两个子类：
- `IdealCompensator`：理想弹道模型（无空气阻力），使用初等物理公式计算
- `ResistanceCompensator`：阻力弹道模型（带空气阻力），使用数值迭代求解

工厂类 `CompensatorFactory::createCompensator(type)` 根据字符串 `"ideal"` / `"resistance"` 创建对应实例。

公共属性：
- `velocity`：子弹初速度（m/s）
- `iteration_times`：迭代补偿次数
- `gravity`：重力加速度（m/s²）
- `resistance`：空气阻力系数

### ManualCompensator（手动硬补偿）

基于二维查表的分段补偿。数据结构为双层映射：

```
距离区间 → 高度区间 → {pitch_offset, yaw_offset}
```

- `DistMapNode`：距离区间 + 该距离内的多个 HeightsMapNode
- `HeightMapNode`：高度区间 + 对应的 pitch/yaw 补偿值

核心方法：
- `updateMapByStr(str)`：从格式字符串 `"距离下限 距离上限 高度下限 高度上限 pitch补偿 yaw补偿"` 更新补偿表
- `updateMapFlow(strs)`：批量更新
- `angleHardCorrect(dist, height)`：根据目标距离和高度查询补偿表，返回 `{pitch_offset, yaw_offset}`

### PnPSolver（PnP 解算）

- 构造函数接收相机内参矩阵和畸变系数，支持选择 PnP 算法（默认 `SOLVEPNP_IPPE`）
- `setObjectPoints(name, points)`：注册一个物体坐标系（如 "small" / "large" 装甲板），存储其三维角点
- `solvePnP(image_points, rvec, tvec, coord_frame_name)`：单解 PnP，返回旋转向量和平移向量
- `solvePnPGeneric(image_points, rvecs, tvecs, coord_frame_name)`：多解 PnP（处理对称目标时的歧义）
- `calculateDistanceToCenter(image_point)`：计算某个图像点到图像中心的像素距离（用于选择最近装甲板）
- `calculateReprojectionError(...)`：计算重投影误差

### Math Utils（数学工具）

| 函数 | 功能 |
|------|------|
| `eulerToMatrix(euler, order)` | 欧拉角 → 旋转矩阵，支持 6 种旋转顺序 |
| `matrixToEuler(R, order)` | 旋转矩阵 → 欧拉角 |
| `getRPY(R)` | 从旋转矩阵提取 RPY 角 |
| `eigenToCv(eigen_mat)` | Eigen 矩阵 → cv::Mat |
| `cvToEigen(cv_mat)` | cv::Mat → Eigen 矩阵 |

### HeartBeatPublisher（心跳发布器）

- `HeartBeatPublisher::create(node)`：挂载到指定 ROS2 节点，启动后台线程定时发布心跳消息
- 发布 `/heartbeat` 话题，类型 `std_msgs/msg/Int64`

### URLResolver（URL 解析）

- `getResolvedPath(url)`：将 `package://package_name/path/to/file` 格式的 URL 解析为实际文件系统路径

支持三种 URL 类型：
- `EMPTY`：空字符串
- `FILE`：普通文件路径
- `PACKAGE`：`package://` 协议路径

### Assert（断言）

| 宏 | 功能 |
|----|------|
| `ZFM_ASSERT(condition)` | 条件不满足时输出错误信息并 abort |
| `ZFM_ASSERT_MSG(condition, msg)` | 带自定义错误信息的断言 |

### Common Enums（公共枚举）

```cpp
enum class EnemyColor { RED = 0, BLUE = 1, WHITE = 2 };
enum VisionMode {
  AUTO_AIM_RED = 0,    // 自瞄红方
  AUTO_AIM_BLUE = 1,   // 自瞄蓝方
  SMALL_RUNE_RED = 2,  // 小能量机关（红）
  SMALL_RUNE_BLUE = 3, // 小能量机关（蓝）
  BIG_RUNE_RED = 4,    // 大能量机关（红）
  BIG_RUNE_BLUE = 5,   // 大能量机关（蓝）
};
```

## 3. 模块逻辑与函数执行流程

### 弹道补偿流程

```
目标三维位置 (x, y, z)
        │
        ▼
计算水平距离 distance = sqrt(x² + y²)
计算俯仰角初始值 pitch_init = atan(z / distance)
        │
        ▼
迭代 iteration_times 次：
  1. calculateTrajectory(distance, pitch) → 弹着点高度
  2. 计算偏差 → 调整 pitch
  3. 更新 pitch
        │
        ▼
getFlyingTime(target_position) → 飞行时间（用于预测前置量）
        │
        ▼
返回最终 pitch 增量
```

阻力模型（ResistanceCompensator）在弹道计算中计入空气阻力，使用数值积分求解运动方程。理想模型（IdealCompensator）使用初等物理公式。

### EKF 跟踪流程

```
初始化：x_post(初始状态), P_post(初始协方差), Q/R(噪声矩阵)
        │
        ▼
每帧：
  ┌─ predict():
  │   调用 f(x) 计算先验状态 x_pri
  │   Ceres Jet 自动求导 F = df/dx
  │   P_pri = F * P_post * F^T + Q
  │
  ├─ update(z):
  │   调用 h(x) 计算预测观测 z_pri
  │   Ceres Jet 自动求导 H = dh/dx
  │   K = P_pri * H^T * (H * P_pri * H^T + R)^(-1)
  │   x_post = x_pri + K * (z - z_pri)
  │   P_post = (I - K * H) * P_pri
  │
  └─ 返回更新后的状态
```

### 手动补偿查询流程

```
目标距离 dist, 目标高度 height
        │
        ▼
遍历 DistMapNode 列表
  └─ 匹配距离区间(dist_region)
      └─ 遍历该节点的 HeightMapNode 列表
          └─ 匹配高度区间(height_region)
              └─ 返回 {pitch_offset, yaw_offset}
```

### PnP 解算流程

```
注册物体坐标系角点 → setObjectPoints("small", 小装甲板三维点)
                     setObjectPoints("large", 大装甲板三维点)
        │
        ▼
接收图像 → 提取灯条 → 匹配装甲板 → 获取图像角点
        │
        ▼
根据装甲板类型选择对应物体坐标系
        │
        ▼
solvePnP(image_points, rvec, tvec, "small"/"large")
        │
        ▼
输出：旋转向量 rvec → 装甲板姿态 (yaw)
      平移向量 tvec → 装甲板三维位置 (x, y, z)
```

### Logger 使用流程

```cpp
// 1. 注册日志器（通常在节点初始化时）
ZFM_REGISTER_LOGGER("my_logger", "/path/to/logs", INFO);
// 这会创建一个日志器，输出到 /path/to/logs/[date]/my_logger_[date].log

// 2. 使用日志宏
ZFM_INFO("my_logger", "This is an info message: {}", value);
ZFM_ERROR("my_logger", "Error occurred: {}", err_msg);
ZFM_WARN("my_logger", "Warning: {}", warning_msg);
```

## 4. 发布和订阅的消息

`rm_utils` 本身不是 ROS 节点，不直接发布/订阅 ROS 话题。

但 `HeartBeatPublisher` 在被挂载到节点后，会发布以下话题：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/heartbeat` | `std_msgs/msg/Int64` | 定时发布时间戳，用于监控节点是否存活 |

其他组件编译为库，由各 ROS 节点（armor_detector、armor_solver 等）链接调用，不产生独立话题。

## 5. 调试方法与参数调整

### 弹道补偿

- `armor_solver_params.yaml` 中调整 `solver.gravity`（重力）、`solver.resistance`（空气阻力）、`solver.iteration_times`（迭代次数）
- 可在 `solver.compenstator_type` 中切换 `"resistance"` / `"ideal"` 对比效果
- 通过 `solver.angle_offset` 中的硬补偿表修正系统误差（射击偏移）

### EKF 调参

- `sigma2_q_*`：过程噪声方差，值越大滤波器对测量的响应越快但也越容易受噪声影响
- `r_*`：测量噪声方差，值越大滤波器越信赖模型预测而非测量值

### 手动补偿表

`angle_offset` 参数格式为 `"dist_low dist_high height_low height_high pitch_offset yaw_offset"`，可根据实测弹道散布调整各区间补偿值。

### Log 查看

```bash
# 日志文件默认输出到注册时指定的路径，按日期分目录
ls /path/to/logs/2026-05-22/
cat /path/to/logs/2026-05-22/my_logger_2026-05-22.log
```

日志内容包含等级标签和时间戳，可通过 `grep` 过滤：
```bash
grep "ERROR" /path/to/logs/*.log
```

### PnP 验证

可通过 `calculateReprojectionError` 接口评估当前帧 PnP 解算精度，较小的重投影误差表示解算结果可靠。
