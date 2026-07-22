# rm_auto_aim

## 概述

`rm_auto_aim` 是一个 ROS 元包（metapackage），用于将 `armor_detector` 和 `armor_solver` 两个功能包组织为一个整体，便于统一构建和依赖管理。

该包本身不包含任何源代码，仅通过 `package.xml` 声明两个子包的依赖关系，并通过 `CMakeLists.txt` 中的 `ament_auto_package()` 完成构建和安装。

## 依赖

根据 `package.xml` 和 `CMakeLists.txt`，该包的依赖如下：

| 类型 | 依赖 | 说明 |
|------|------|------|
| buildtool_depend | `ament_cmake` | ROS2 构建系统 |
| depend | `rm_interfaces` | 自定义消息/服务接口定义 |
| depend | `armor_detector` | 装甲板视觉检测子包 |
| depend | `armor_solver` | 跟踪与解算子包 |

## 工作流

```
相机图像
    │
    ▼
┌──────────────────┐
│  armor_detector  │  ← 视觉检测：灯条提取、装甲板配对、数字分类、PnP 位姿解算
│  (检测节点)       │  → 发布: Armors (含三维位姿)
└────────┬─────────┘
         │ rm_interfaces::msg::Armors
         ▼
┌──────────────────┐
│  armor_solver    │  ← 跟踪滤波：TF 变换、EKF 跟踪、选板、弹道补偿
│  (解算节点)       │  → 发布: GimbalCmd (云台控制指令 + 开火建议)
└────────┬─────────┘
         │ rm_interfaces::msg::GimbalCmd
         ▼
      下位机 / 云台控制器
```

## 构建

```bash
# 在工作空间中单独构建该元包（会自动构建其依赖的子包）
colcon build --packages-select rm_auto_aim

# 或构建整个工作空间
colcon build
```

## 启动

建议通过 launch 文件同时启动两个子包：

```bash
ros2 launch armor_detector detector_launch.py
ros2 launch armor_solver solver_launch.py
```
