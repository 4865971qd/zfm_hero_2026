# PlotJuggler 调试字段说明

本项目通过 UDP 向 PlotJuggler 输出 JSON。配置开关在 `configs/standard.yaml`：

```yaml
debug:
  enable_plotter: true
```

默认端口见 `utils/plotter.hpp`：`127.0.0.1:9870`。

## 字段分组

### 1. 时间同步

| 字段 | 含义 | 判断方法 |
| --- | --- | --- |
| `imu_sync_valid` | 是否按图像时间取到了可用 IMU | 经常为 `0` 说明串口缓存或时间对齐有问题 |
| `imu_query_error_ms` | 图像查询时间和实际使用 IMU 的时间差，插值时接近 0，夹取最新帧时通常为负 | 绝对值经常大于 `5~10ms`，云台运动时会导致坐标漂 |
| `imu_bracket_ms` | 插值使用的前后两帧 IMU 时间间隔 | 明显大于正常串口周期，说明串口丢帧或接收不稳 |
| `serial_age_ms` | 图像时间相对最新串口帧的时间差 | 长期为负或过大，都说明图像和串口时序不舒服 |

### 2. 观测值

| 字段 | 含义 |
| --- | --- |
| `obs_matched` | 当前帧是否有装甲板成功匹配进跟踪器 |
| `obs_x/y/z` | 当前匹配观测折算到等效装甲板的世界坐标 |
| `obs_yaw` | 当前匹配观测折算出的车体基础 yaw，单位度 |

看法：

- `obs_x/y/z` 静止时自己跳，优先查检测框、PnP、IMU 同步和外参。
- `obs_yaw` 偶发跳接近 `180deg`，优先怀疑 PnP 双解或装甲板 yaw 翻解。
- `obs_matched` 频繁掉到 `0`，但画面检测框正常，优先查匹配门限或 tracked id。

### 3. 模型值

| 字段 | 含义 |
| --- | --- |
| `model_cx/cy/model_z` | 跟踪器估计的车体中心 |
| `model_yaw` | 跟踪器估计的车体基础 yaw，单位度 |
| `model_vx/vy/vz` | 车体线速度 |
| `model_vyaw` | 车体自转角速度 |
| `model_r1/r2` | 两组装甲板半径 |
| `model_dz` | 两组装甲板高度差 |

看法：

- `obs` 稳而 `model` 慢慢漂，查跟踪更新、dt、IMU 零漂。
- `model_vyaw` 抖而 `obs_yaw` 稳，查 yaw 更新和切板归类。
- `model_r1/r2` 慢慢跑飞，查半径约束、误匹配、侧向 PnP 质量。
- `model_dz` 突然跳，查切到另一组板时的高度更新。

### 4. 残差

| 字段 | 含义 |
| --- | --- |
| `res_x/y/z` | 观测值减模型预测值 |
| `res_yaw` | yaw 残差，单位度 |
| `res_norm` | xyz 残差模长 |

看法：

- `res_x/y/z` 长期同方向偏，查外参、IMU 坐标、枪口偏置。
- `res_norm` 切板瞬间尖峰，查相位/板号匹配。
- `res_norm` 偶发尖峰马上恢复，查检测误识别或 PnP 双解。
- `res_yaw` 切板尖峰，查 `matched_plate` 相关逻辑和 yaw 解算。

### 5. 选板

| 字段 | 含义 |
| --- | --- |
| `sel_idx` | 最终选择击打的板 |
| `sel_raw_idx` | 按距离直接选出的板 |
| `sel_held` | 是否因为距离优势不明显而保持上一稳定板 |
| `sel_margin` | 最近板和第二近板的距离差 |
| `sel_dist` | 最终选择板距离 |
| `sel_best_dist` | 最近板距离 |
| `sel_second_dist` | 第二近板距离 |

看法：

- `sel_raw_idx` 抖但 `sel_idx` 稳，保持逻辑正常。
- `sel_idx` 和 `sel_raw_idx` 一起抖，保持阈值可能不够。
- `sel_margin` 很小，同时发生切换，这是正常边界竞争。
- `sel_idx` 跳错且 `res_norm` 同时大，优先查观测匹配或 PnP。

### 6. 指令与弹道

| 字段 | 含义 |
| --- | --- |
| `cmd_yaw/cmd_pitch` | 最终发给下位机的角度，单位度 |
| `cmd_target_yaw/cmd_target_pitch` | 未经过死区后的解算目标角，单位度 |
| `fly_time` | 弹丸飞行时间 |
| `fire` | 是否建议开火 |
| `dist/yaw/v_yaw` | 兼容旧字段，分别是距离、模型 yaw、模型角速度 |

看法：

- `cmd_target_yaw` 稳而 `cmd_yaw` 抖，查死区或输出侧。
- `cmd_target_yaw` 自己抖，查选板、弹道预测和模型。
- `fire` 一直为 `0`，但 `cmd_target` 与云台反馈误差很小，查开火窗口。

## 推荐排查顺序

1. 先看 `imu_sync_valid`、`imu_query_error_ms`、`imu_bracket_ms`。
2. 再看 `obs_x/y/z/yaw` 是否稳定。
3. 然后看 `res_norm` 和 `res_yaw`。
4. 最后看 `sel_idx/sel_raw_idx/sel_held/sel_margin`。

一句话：

```text
IMU 时间不对 -> obs 会漂
obs 稳但 model 漂 -> tracker 问题
model 稳但 sel 抖 -> 选板问题
sel 稳但 cmd 抖 -> 解算/输出问题
```
