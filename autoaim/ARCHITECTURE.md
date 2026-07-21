# AutoAim 架构说明

`autoaim` 是纯 C++ 自瞄程序，运行时不依赖 ROS2。主流程在 `src/main.cpp`，核心链路如下：

前哨站专项实现、参数和排障见 [`前哨自瞄.md`](前哨自瞄.md)。

```text
相机/视频帧
  -> Detector: 传统 OpenCV 灯条检测 + 装甲板配对 + LeNet 数字分类
  -> Solver::solveArmor: PnP 位姿解算 + camera/gimbal/world 坐标变换
  -> Tracker: EKF 普通目标跟踪 / OutpostTracker 前哨站几何跟踪
  -> Solver::solve: 选板 + 弹道补偿 + yaw/pitch 解算 + 开火窗口判断
  -> Serial: 发送云台控制指令
```

## 模块边界

| 模块 | 位置 | 职责 |
|---|---|---|
| 硬件输入 | `hardware/` | 海康相机、视频回放、串口协议 |
| 视觉检测 | `src/detector.*`, `src/classifier.*` | OpenCV 传统灯条/装甲板检测，LeNet/MLP 数字分类 |
| 位姿解算 | `utils/pnp.*`, `src/solver.*` | IPPE PnP、坐标变换、重投影调试 |
| 目标跟踪 | `src/tracker.*`, `src/target.*`, `utils/ekf.*` | EKF 状态机、前哨站圆模型 |
| 弹道补偿 | `utils/trajectory.*`, `utils/manual_compensator.*` | 理想/二次阻力弹道和手动补偿表 |
| 调试输出 | `utils/plotter.*`, `utils/recorder.*` | PlotJuggler UDP、视频和 IMU 录制 |

## 跟踪与安全约束

- 普通目标状态机使用时间阈值，不依赖相机帧率；`DETECTING` 连续匹配后进入 `TRACKING`，短时丢失进入 `TEMP_LOST`。
- yaw 观测先按 EKF 预测值展开到最近等价角，避免跨越 `-pi/pi` 时出现整圈跳变。
- 装甲板跳变会同步半径和高度对；预测、更新或跳变产生非有限状态时回滚，不把异常状态送入解算器。
- PnP 失败或输出非有限位姿的检测会在进入 Tracker 前删除。
- 普通目标开火受观测新鲜度保护；高速转中心状态禁止开火。
- 前哨站按 `COLLECTING -> CALIBRATING -> ACTIVE` 建模，Z 标定完成前不切板、不开火；每块物理板维护独立高度。

## 预测与选板

Solver 先按观测延迟、弹丸飞行时间和 `prediction_delay` 预测整车中心与 yaw，再依据预测状态选板。飞行时间精炼或 `controller_delay` 改变预测时会重新计算板位并重新选板。所有瞄准坐标都以枪口为原点，输出前再次检查非有限值。

## 当前检测策略

检测路径固定为传统 OpenCV + LeNet 数字分类。YOLO/OpenVINO 路径已经移除，相关模型、参数和构建依赖不再保留。

传统检测调参顺序建议：

1. 先调 `detector.detect_color` 和 `detector.binary_thres`，确保灯条二值化稳定。
2. 再调 `detector.light.*`，减少灯条漏检和误检。
3. 接着调 `detector.armor.*`，处理灯条配对过宽、过窄或倾角异常。
4. 最后调 `classifier_threshold` 和 `ignore_classes`，控制数字分类误识别。
