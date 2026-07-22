# rm_robot_description

## 1. 包概述

`rm_robot_description` 是 RoboMaster 自瞄系统的机器人模型描述包，提供云台-相机系统的 URDF/Xacro 模型定义。

包含一个 xacro 文件 `rm_gimbal.urdf.xacro`，定义从 `odom`（里程计坐标系）到 `camera_optical_frame`（相机光学坐标系）的完整 TF 变换链路。该模型在系统启动时由 `rm_bringup` 的 `bringup.launch.py` 加载，并通过 `robot_state_publisher` 发布 TF 树。

### 文件结构

```
urdf/
  rm_gimbal.urdf.xacro   — 云台-相机 URDF 模型（xacro 格式）
```

## 2. 关键变量含义

### xacro 参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `xyz` | `"0.10 0 0.05"` | 相机在 gimbal_link 坐标系下的安装位置 (x y z, 米)，通过 `launch_params.yaml` 的 `odom2camera.xyz` 传入 |
| `rpy` | `"0 0 0"` | 相机在 gimbal_link 坐标系下的安装姿态 (roll pitch yaw, 弧度)，通过 `launch_params.yaml` 的 `odom2camera.rpy` 传入 |

### TF 树中的坐标系（frame）

| 坐标系 | 类型 | 含义 |
|--------|------|------|
| `odom` | link | 里程计世界坐标系，TF 树的根 |
| `gimbal_link` | link | 云台坐标系，通过 floating joint 连接到 odom（实车运行时由下位机提供 odom→gimbal 的变换） |
| `camera_link` | link | 相机坐标系，通过固定关节连接到 gimbal_link，位置/姿态由 xacro 参数 xyz/rpy 决定 |
| `camera_optical_frame` | link | 相机光学坐标系（遵循 ROS REP-103：Z 轴向前，X 轴向右，Y 轴向下），通过固定关节连接到 camera_link |

## 3. 模块逻辑与函数执行流程

### TF 树结构

```
odom
 │
 │  gimbal_joint (floating)
 │  变换由下位机/里程计提供
 ▼
gimbal_link
 │
 │  camera_joint (fixed)
 │  xyz = odom2camera.xyz
 │  rpy = odom2camera.rpy
 ▼
camera_link
 │
 │  camera_optical_joint (fixed)
 │  rpy = (-pi/2, 0, -pi/2)   ← 相机坐标系到光学坐标系的旋转
 ▼
camera_optical_frame
```

### 启动加载流程

```
bringup.launch.py
        │
        ▼
xacro rm_gimbal.urdf.xacro xyz=... rpy=...
        │
        ▼
robot_state_publisher (publish_frequency=1000Hz)
        │
        ▼
发布 /tf：odom → gimbal_link → camera_link → camera_optical_frame
        │
        ▼
armor_detector / armor_solver 通过 TF 监听获取相机位姿，
将检测结果从相机坐标系转换到目标坐标系（如 odom）
```

### 关节定义

| 关节 | 类型 | 父坐标系 | 子坐标系 | 说明 |
|------|------|----------|----------|------|
| `gimbal_joint` | floating | odom | gimbal_link | 6-DOF 浮动关节，实际变换由下位机里程计数据驱动 |
| `camera_joint` | fixed | gimbal_link | camera_link | 固定关节，相机在云台上的安装位置和姿态 |
| `camera_optical_joint` | fixed | camera_link | camera_optical_frame | 固定关节，将相机坐标系旋转至 ROS 标准光学坐标系 |

## 4. 发布和订阅的消息

`rm_robot_description` 本身是纯数据包，不包含任何 ROS 节点。URDF 内容由 `rm_bringup` 中的 `robot_state_publisher` 节点加载，该节点发布：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/tf` | tf2_msgs/TFMessage | 包含 odom → gimbal_link → camera_link → camera_optical_frame 的坐标系变换。gimbal 的变换由 floating joint 驱动（需外部传入 odom→gimbal 的变换），相机到云台的变换由 xacro 参数固定 |
| `/robot_description` | std_msgs/String | robot_state_publisher 加载的 URDF 字符串，供其他工具（如 RViz）读取模型信息 |

## 5. 调试方法与参数调整

### 在 RViz 中查看机器人模型

```bash
rviz2
# 添加 RobotModel 显示组件
# 设置 Fixed Frame 为 "odom"
# 添加 TF 显示组件查看完整 TF 树
```

### 调整相机安装位姿

修改 `launch_params.yaml` 中的 `odom2camera` 参数：

```yaml
odom2camera:
  xyz: "\"0.246 0.0  0.049\""   # 相机在云台上的安装位置（米）
  rpy: "\"0.00 0.27 -0.02\""     # 相机在云台上的安装姿态（弧度）
```

修改后重新启动系统，`robot_state_publisher` 会加载新的 URDF，TF 树自动更新。

### 验证 TF 树

```bash
# 打印完整 TF 树
ros2 run tf2_tools view_frames.py

# 监听特定坐标变换
ros2 run tf2_ros tf2_echo odom camera_optical_frame

# 查看静态 TF 信息
ros2 topic echo /tf_static
```
