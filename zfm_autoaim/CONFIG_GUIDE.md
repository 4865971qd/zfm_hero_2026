# zfm_autoaim 配置指南

主入口在 `src/rm_bringup/config`：

| 文件 | 作用 |
|---|---|
| `launch_params.yaml` | 是否视频回放、虚拟串口、导航 TF 等启动级开关 |
| `node_params/armor_detector_params.yaml` | 传统检测、LeNet 分类、PnP/BA 参数 |
| `node_params/armor_solver_params.yaml` | EKF、弹道、选板、开火窗口参数 |
| `node_params/camera_driver_params.yaml` | 相机参数 |
| `node_params/serial_driver_params.yaml` | 串口参数 |
| `camera_info.yaml` | 相机内参 |

## armor_detector

检测路径固定为 OpenCV 传统灯条检测 + LeNet 数字分类。

| 参数 | 说明 |
|---|---|
| `detect_color` | `0` 红方，`1` 蓝方 |
| `binary_thres` | 灰度二值化阈值 |
| `use_pca` | 是否启用 PCA 灯条角点修正 |
| `use_ba` | 是否启用 BA 优化 yaw |
| `light.*` | 灯条候选过滤 |
| `armor.*` | 装甲板配对过滤 |
| `classifier_threshold` | 数字分类置信度阈值 |
| `ignore_classes` | 分类后要忽略的类别 |

## armor_solver

| 参数 | 说明 |
|---|---|
| `ekf.sigma2_q_*` | 过程噪声 |
| `ekf.r_*` | 观测噪声 |
| `tracker.max_match_distance` | 预测和观测的最大匹配距离 |
| `tracker.max_match_yaw_diff` | 装甲板 yaw 匹配阈值 |
| `tracker.tracking_confirm_time` | 连续匹配达到该时间后进入 TRACKING，单位秒 |
| `tracker.lost_time_thres` | 连续丢失达到该时间后进入 LOST，单位秒 |
| `ekf.stationary_confirm_time/release_time` | 普通目标静止模型进入/退出确认时间 |
| `ekf.stationary_vel_threshold/release_vel_threshold` | 静止模型进入/退出速度迟滞阈值 |
| `solver.prediction_delay` | 预测延迟补偿 |
| `solver.controller_delay` | 在弹道预测结果上继续外推并重新选板 |
| `solver.bullet_speed` | 固定弹速 |
| `solver.shooting_range_width/height` | 开火窗口 |
| `solver.normal_max_fire_unseen_time` | 普通目标开火观测新鲜度限制 |
| `solver.normal_max_aim_unseen_time` | 普通目标继续输出瞄准的观测新鲜度限制 |
| `solver.center_switch_confirm_time` | 装甲板/整车中心模式切换确认时间 |
| `solver.outpost_max_unseen_time` | 前哨站开火允许的最大无真实观测时间 |
| `solver.outpost_max_phase_error` | 前哨站开火允许的最大相位残差，单位为度 |

状态确认和丢失阈值全部按时间计算，与相机帧率无关。异常帧间隔会被限制，并复位静止模型确认状态。

普通目标的 `Target` 消息包含 `normal_last_seen_time`。修改该消息后必须重新生成 `rm_interfaces`；若旧的 `build/install` 中仍有旧消息头，需要清理后重新构建工作区。

前哨站必须完成 Z 标定后才进入 ACTIVE；标定前只使用当前观测板高度，不切板、不建议开火。

## 推荐调参顺序

1. 先确认 `camera_info.yaml` 和 `launch_params.yaml`。
2. 调 `armor_detector_params.yaml`，确保二值图、灯条和数字分类稳定。
3. 调 `armor_solver_params.yaml` 的 EKF 匹配与噪声。
4. 调弹速、弹道参数和手动补偿表。
5. 最后通过 PlotJuggler/RViz 检查 `cmd_gimbal`、`target` 和 marker。
