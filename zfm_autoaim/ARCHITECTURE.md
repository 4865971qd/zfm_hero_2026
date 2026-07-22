# zfm_autoaim 架构说明

`zfm_autoaim` 是 ROS2 自瞄工作空间，视觉、解算、硬件驱动和接口消息分包维护。

前哨站专项实现、ROS2 接口和排障见 [`前哨自瞄.md`](前哨自瞄.md)。

```text
rm_camera_driver / video_player
  -> armor_detector: 传统 OpenCV 灯条检测 + LeNet 数字分类 + PnP/BA
  -> armor_solver: TF 坐标变换 + EKF/前哨站跟踪 + 弹道解算
  -> rm_serial_driver: 发送 GimbalCmd，接收下位机状态
```

## 包职责

| 包 | 职责 |
|---|---|
| `rm_bringup` | launch 和节点参数 |
| `rm_hardware_driver/rm_camera_driver` | 海康相机和视频输入 |
| `rm_hardware_driver/rm_serial_driver` | 串口协议和虚拟串口 |
| `rm_auto_aim/armor_detector` | 传统视觉检测、LeNet 分类、PnP/BA 位姿 |
| `rm_auto_aim/armor_solver` | EKF 跟踪、前哨站模型、选板、弹道和云台指令 |
| `rm_interfaces` | 自定义 msg/srv |
| `rm_utils` | 日志、数学、PnP、滤波、补偿工具 |

## 跟踪与解算约束

- 普通目标跟踪确认、丢失和静止模式切换都按时间计算，不依赖图像帧率。
- PnP 双解先比较重投影误差，误差接近时再使用姿态和灯条倾斜方向消歧；无效位姿不会发布给 Tracker。
- yaw 观测按 EKF 预测值展开到最近等价角；跳板会同步半径与高度对。预测、更新和跳板产生非有限状态时会回滚或重新初始化。
- Solver 先结合观测延迟、弹丸飞行时间和 `prediction_delay` 预测，再选板；`controller_delay` 在该结果上继续外推并重新选板。
- 普通目标瞄准和开火分别受观测新鲜度保护，高速转中心状态禁止开火。非有限云台命令会在发布前拦截。
- 前哨站采用 `COLLECTING -> CALIBRATING -> ACTIVE`；Z 标定完成前不切板、不开火，每块物理板维护独立高度状态。

## 当前检测策略

`armor_detector` 固定使用传统 OpenCV 检测和 LeNet 数字分类。YOLO/OpenVINO 路径已经移除，参数文件中不再提供 `use_yolo` 或 `yolo_*` 项。

## 主要调试入口

| 类型 | 入口 |
|---|---|
| 检测结果图 | `armor_detector/result_img` |
| 二值图 | `armor_detector/binary_img` |
| 灯条调试数据 | `armor_detector/debug_lights` |
| 装甲板调试数据 | `armor_detector/debug_armors` |
| 跟踪目标 | `armor_solver/target` |
| 云台命令 | `armor_solver/cmd_gimbal` |
| RViz marker | `armor_detector/marker`, `armor_solver/marker` |
