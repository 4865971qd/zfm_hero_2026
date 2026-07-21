# AutoAim 配置指南

主配置文件是 `configs/standard.yaml`。参数按运行链路分组：相机/视频、内外参、检测、跟踪、解算弹道、串口、调试。

## 常用场景

### 离线视频调试

```yaml
video:
  enable: true
  path: "../demo.avi"
  fps: 60.0
  loop: false
```

调检测时建议打开：

```yaml
debug:
  show_image: true
  show_detection: true
  show_observation: false
  show_selected: true
```

### 实车运行

```yaml
video:
  enable: false

serial:
  port_name: "/dev/ttyACM0"
  baudrate: 115200
```

确认 `camera_matrix`、`distort_coeffs` 和 `odom2camera` 与实车一致后再调弹道。

## 检测参数

| 参数 | 现象 | 调整方向 |
|---|---|---|
| `detect_color` | 红蓝方识别反了 | 红方填 `0`，蓝方填 `1` |
| `binary_thres` | 灯条断裂或噪声多 | 断裂就调低，噪声多就调高 |
| `light.min_ratio/max_ratio` | 候选灯条太多或太少 | 漏检就放宽，误检就收紧 |
| `armor.*center_distance` | 配对过宽/过窄 | 根据灯条间距和装甲板大小调整 |
| `classifier_threshold` | 数字误识别或漏识别 | 误识别调高，漏识别调低 |

## 跟踪与解算

| 参数 | 作用 |
|---|---|
| `tracker.max_match_distance` | EKF 预测和观测的最大匹配距离 |
| `tracker.max_match_yaw_diff` | 预测与观测的最短 yaw 角差阈值 |
| `tracker.tracking_confirm_time` | 连续匹配达到该时间后进入 TRACKING，单位秒 |
| `tracker.lost_time` | 连续丢失达到该时间后回到 LOST，单位秒 |
| `tracker.stationary_confirm_time/release_time` | 静止模型进入/退出确认时间，单位秒 |
| `tracker.stationary_vel_threshold/release_vel_threshold` | 静止模型进入/退出速度迟滞阈值 |
| `solver.prediction_delay` | 视觉/通信延迟补偿 |
| `solver.controller_delay` | 在弹道预测结果上继续补偿控制器响应延迟，并重新选板 |
| `solver.bullet_speed` | 固定弹速，必须按实测填写 |
| `solver.shooting_range_width/height` | 开火窗口大小 |
| `solver.normal_max_fire_unseen_time` | 普通目标开火允许的最大无真实观测时间 |
| `solver.center_switch_confirm_time` | 装甲板/整车中心模式切换确认时间 |
| `solver.outpost_max_unseen_time` | 前哨站开火允许的最大无真实观测时间 |
| `solver.outpost_max_phase_error` | 前哨站开火允许的最大相位残差，单位为度 |

所有跟踪确认、丢失和模式切换阈值都按时间计算，与相机帧率无关。帧间隔异常时 EKF 预测步长会被限制，并复位静止状态确认，避免断流后一次大步外推。

普通目标只有最近观测仍新鲜时才允许开火。前哨站必须完成 Z 标定，并同时满足观测新鲜度与相位残差要求；标定完成前只能使用当前观测板高度，不切板、不建议开火。

## 调参顺序

1. 检查内参、外参、弹速。
2. 调好二值化和灯条过滤。
3. 确认 PnP 距离和 yaw 没有明显跳变。
4. 调 EKF 匹配距离和丢失阈值。
5. 调弹道和手动补偿表。
6. 最后再收紧开火窗口。
