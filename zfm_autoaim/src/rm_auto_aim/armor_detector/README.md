# armor_detector

## 1. 包概述

`armor_detector` 是 RoboMaster 自瞄系统的视觉前端包。它订阅相机原始图像和相机内参，执行完整的装甲板检测流程（图像预处理、灯条提取、灯条配对、数字分类、角点精化），并通过 PnP 解算和可选的 BA（Bundle Adjustment）优化获取每块装甲板在三维空间中的位姿，最终发布识别到的装甲板信息。

- **命名空间**: `zfm::auto_aim`
- **节点名**: `armor_detector`
- **节点类型**: `ArmorDetectorNode`（组件节点，通过 `rclcpp_components` 注册）

### 核心类

| 类名 | 头文件 | 职责 |
|------|--------|------|
| `Detector` | `armor_detector.hpp` | 主检测器：图像预处理、找灯条、灯条配对 |
| `NumberClassifier` | `number_classifier.hpp` | 基于 ONNX 模型（LeNet-5/MLP）的数字分类 |
| `LightCornerCorrector` | `light_corner_corrector.hpp` | PCA 角点矫正 |
| `ArmorPoseEstimator` | `armor_pose_estimator.hpp` | PnP 解算 + BA 优化装甲板位姿 |
| `BaSolver` / `GraphOptimizer` | `ba_solver.hpp` / `graph_optimizer.hpp` | g2o 后端 BA 优化器 |

### 依赖

- **ROS 包**: `rclcpp`, `rclcpp_components`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `cv_bridge`, `image_transport`, `image_transport_plugins`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `message_filters`, `rm_interfaces`, `rm_utils`, `vision_opencv`, `std_srvs`
- **第三方库**: `OpenCV 4.1.2+`, `Eigen3`, `fmt`, `Sophus`, `TBB`, `g2o`
- **模型文件**: `model/lenet.onnx`（或 `mlp.onnx`）, `model/label.txt`

---

## 2. 关键变量含义

以下为节点声明的 ROS 参数及其默认值：

### 检测参数

| 参数名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `binary_thres` | int | 160 | 灰度二值化阈值 |
| `classifier_threshold` | double | 0.7 | 数字分类置信度阈值，低于该值的分类结果被丢弃 |
| `ignore_classes` | string[] | `["negative"]` | 需要忽略的类别名称（如无装甲板负样本） |
| `use_pca` | bool | true | 是否启用 PCA 角点矫正 |
| `use_ba` | bool | true | 是否启用 BA 优化 |
| `target_frame` | string | "odom" | TF 的目标坐标系 |
| `debug` | bool | true | 是否开启调试模式（影响 debug 图像/数据发布） |

### 灯条参数 (light.*)

| 参数名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `light.min_ratio` | double | 0.08 | 灯条最小宽高比（宽度/长度） |
| `light.max_ratio` | double | 0.4 | 灯条最大宽高比 |
| `light.max_angle` | double | 40.0 | 灯条最大倾斜角度（度） |
| `light.color_diff_thresh` | int | 25 | 灯条颜色判断阈值，轮廓内 R 与 B 均值之差的绝对值需大于该值才被赋予颜色 |

### 装甲板参数 (armor.*)

| 参数名 | 类型 | 默认值 | 含义 |
|--------|------|--------|------|
| `armor.min_light_ratio` | double | 0.6 | 两灯条长度之比的最小值（短/长） |
| `armor.min_small_center_distance` | double | 0.8 | 小装甲板两灯条中心距与灯条平均长度之比的最小值 |
| `armor.max_small_center_distance` | double | 3.2 | 小装甲板两灯条中心距与灯条平均长度之比的最大值 |
| `armor.min_large_center_distance` | double | 3.2 | 大装甲板两灯条中心距与灯条平均长度之比的最小值 |
| `armor.max_large_center_distance` | double | 5.0 | 大装甲板两灯条中心距与灯条平均长度之比的最大值 |
| `armor.max_angle` | double | 35.0 | 装甲板水平方向最大倾斜角度（度） |

---

## 3. 模块逻辑与函数执行流程

### 整体流程

```
imageCallback (收到图像)
  │
  ├─ 1. 查询 TF: odom → gimbal_link，获取 imu_to_camera 旋转矩阵
  │
  ├─ 2. detectArmors (Detector::detect)
  │     ├─ preprocessImage: RGB→灰度 → 二值化 (binary_thres)
  │     ├─ findLights: findContours → minAreaRect → isLight (宽高比 + 角度 + 颜色)
  │     ├─ matchLights: 两两配对 → isArmor (长度比 + 中心距 + 角度) → 过滤 containLight
  │     ├─ NumberClassifier::extractNumber: 透视变换提取数字 ROI
  │     ├─ NumberClassifier::classify: ONNX 推理 → 数字分类
  │     ├─ LightCornerCorrector::correctCorners: PCA 角点矫正
  │     └─ classifier->eraseIgnoreClasses: 丢弃忽略类别
  │
  ├─ 3. ArmorPoseEstimator::extractArmorPoses
  │     ├─ PnP 求解 (IPPE/SOLVEPNP_IPPE)
  │     └─ BA 优化 (use_ba=true: g2o 后端优化 Yaw 角度)
  │
  └─ 4. 发布: armors_msg_ (Armors) + 调试信息
```

### 关键函数说明

#### Detector::preprocessImage
将 RGB 图像转为灰度图，再以 `binary_thres` 为阈值进行固定阈值二值化。由于工业相机动态范围有限，灯条中心常过曝，直接使用灰度二值化比基于颜色的 HSV 二值化更稳定。

#### Detector::findLights
1. `findContours` 提取所有外轮廓
2. 对每个轮廓用 `minAreaRect` 获得最小外接矩形
3. `isLight` 函数检查宽高比 ∈ (min_ratio, max_ratio) 且倾斜角度 < max_angle
4. 颜色判断：对轮廓内所有像素点的 R 通道和 B 通道分别求和，若 `|sum_R - sum_B| > color_diff_thresh`，则 `sum_R > sum_B` 时为 RED，否则为 BLUE
5. 按中心 x 坐标排序

#### Detector::matchLights
1. 只选择颜色等于 `detect_color`（RED/BLUE）的灯条
2. 两两配对，提前用 `containLight` 排除两灯条之间包含其他灯条的无效配对
3. `isArmor` 检查：灯条长度比、中心距与平均长度之比（分小装甲板/大装甲板两档）、水平连接角度
4. 根据中心距区分 SMALL / LARGE 装甲板

#### NumberClassifier::classify
- 使用透视变换将装甲板区域矫正到标准视角
- 提取数字 ROI，大津法二值化
- ONNX Runtime 调用 LeNet-5（或 MLP）模型推理
- 输出数字类别（0-9, "outpost", "negative"）及置信度

#### LightCornerCorrector::correctCorners
- 使用 PCA 求灯条的对称轴（主方向）
- 沿对称轴寻找亮度梯度最大的两个点作为灯条上下角点
- 提高 PnP 解算精度，减少二值化阈值变化带来的角点漂移

#### ArmorPoseEstimator::extractArmorPoses
- 使用 `PnPSolver`（IPPE 方法）从 2D-3D 对应点求解装甲板位姿
- 若 `use_ba=true`，使用 g2o 图优化库进行 BA 优化，通过最小化重投影误差来精化装甲板的 Yaw 角
- 装甲板尺寸定义在 `types.hpp`：小装甲板 133mm×50mm，大装甲板 225mm×50mm

---

## 4. 发布和订阅的消息

### 订阅话题

| 话题名 | 消息类型 | QoS | 说明 |
|--------|----------|-----|------|
| `image_raw` | `sensor_msgs::msg::Image` | SensorData | 相机 RGB 图像（仅接收一次 camera_info 后才开始处理） |
| `camera_info` | `sensor_msgs::msg::CameraInfo` | SensorData | 相机内参（接收一次后自行取消订阅） |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `armor_detector/armors` | `rm_interfaces::msg::Armors` | 检测到的装甲板列表（含位姿、数字、类型、距离等） |
| `armor_detector/marker` | `visualization_msgs::msg::MarkerArray` | RViz 可视化标记（装甲板 Cube + 数字标签） |

### 调试发布话题 (仅 debug=true 时)

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `armor_detector/debug_lights` | `rm_interfaces::msg::DebugLights` | 所有候选灯条的调试数据（中心、宽高比、角度、是否灯条） |
| `armor_detector/debug_armors` | `rm_interfaces::msg::DebugArmors` | 所有候选装甲板的调试数据（类型、中心距、角度等） |
| `armor_detector/binary_img` | `sensor_msgs::msg::Image` | 二值化图像 |
| `armor_detector/number_img` | `sensor_msgs::msg::Image` | 数字识别 ROI 拼接图 |
| `armor_detector/result_img` | `sensor_msgs::msg::Image` | 检测结果标注图（含灯条椭圆、装甲板边框、数字标签、延迟） |

### 服务

| 服务名 | 服务类型 | 说明 |
|--------|----------|------|
| `armor_detector/set_mode` | `rm_interfaces::srv::SetMode` | 切换自瞄模式：`AUTO_AIM_RED` / `AUTO_AIM_BLUE` / 其他（停用） |

---

## 5. 调试方法与参数调整

### 性能调优

- **关闭调试模式**：将 `debug` 参数设为 `false` 可显著提升帧率。调试模式下会额外发布 5 个话题并对每帧图像进行标注，开销较大。参数支持运行时动态切换。
- **二值化阈值** (`binary_thres`)：值过低会引入过多噪点，值过高可能导致灯条断裂。建议根据实际光照条件在 100~200 范围内调整。
- **PCA 角点矫正** (`use_pca`)：开启可提高 PnP 精度，但会增加少量计算开销。
- **BA 优化** (`use_ba`)：开启可提高位姿精度，但需依赖 g2o 库。若计算资源有限可关闭。
- **灯条颜色阈值** (`light.color_diff_thresh`)：环境光照变化大时需适当调整。值过大可能导致灯条颜色无法被正确识别。
- **分类置信度阈值** (`classifier_threshold`)：值越高误报越少但漏检可能增加。建议根据实测 0.6~0.8 范围调整。

### 运行时参数动态调整

节点注册了 `on_set_parameters` 回调，支持运行时通过 ROS 参数服务动态修改以下参数，无需重启节点：
- `binary_thres`, `classifier_threshold`
- `light.*` 全部参数
- `armor.*` 全部参数

### 常用调试方法

1. 在 RViz 中订阅 `armor_detector/result_img` 可视化检测结果
2. 订阅 `armor_detector/binary_img` 观察二值化效果
3. 查看 `armor_detector/debug_lights` 和 `armor_detector/debug_armors` 数据可定位漏检/误检原因
4. 通过 `rqt_reconfigure` 或 `ros2 param set` 动态调参，实时观察效果变化

### TF 变换说明

节点需要 `odom → gimbal_link` 的 TF 变换来获取 IMU 坐标系到相机坐标系的旋转矩阵 `R_imu_camera`，用于将装甲板位姿从相机系变换到世界系。若 TF 查询失败（超时 10ms），该帧图像将被跳过。
