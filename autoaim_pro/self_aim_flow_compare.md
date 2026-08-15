# 自瞄流程与方法对比（autoaim_pro）

本文按当前 `autoaim_pro` 的实现重写，目标是把自瞄拆成 5 个清晰模块：

1. 观测层
2. 建模层
3. 预测层
4. 决策层
5. 诊断层

核心原则只有四个：

- 稳定观测
- 短时外推
- 长时重建
- 最近板优先

---

## 1. 总流程

当前 `autoaim_pro` 的主链路是：

1. 图像帧进入
2. 按图像时间戳查询 IMU
3. 检测装甲板
4. 对每块板做 PnP
5. tracker 维护整车状态
6. solver 预测飞行时刻的板位
7. 选出最近且最稳的目标板
8. 输出击打角度与是否开火

这条链路里，最关键的是把“看到什么”和“打什么”分开。

---

## 2. 观测层

### 解决的问题

- 当前看到了什么
- 这块板的 PnP 是否可靠
- 图像时间与 IMU 时间是否对齐

### 当前实现

- 相机帧使用 `steady_clock`
- IMU 以接收时刻进入 buffer
- `imuAt()` 按图像时刻插值
- `imu_time_offset_ms` 做固定偏移修正
- `solveArmor()` 保留 PnP 双解，并计算重投影误差

### 关键字段

- `obs_x / obs_y / obs_z`
- `obs_yaw`
- `imu_sync_valid`
- `imu_query_error_ms`
- `imu_bracket_ms`
- `serial_age_ms`
- `pnp_reprojection_errors`
- `pose_ambiguity`

### 对比结论

- `sp_vision_25-main`：也是主机时间 + buffer 插值，思路很接近
- `dx_vision-main`：更偏 ROS 消息 stamp 拼帧
- `wust_vision-main`：更偏 TF / stamp 查询
- `autoaim_pro`：最实用的是“主机单调时钟 + 插值 + 固定 offset”

### 评价

这一层在 `autoaim_pro` 里已经够稳，不需要再堆复杂同步器。

---

## 3. 建模层

### 解决的问题

- 这是不是同一辆车
- 四块板怎么对应
- 当前处于什么相位
- 当前是静止、普通转动，还是前哨站

### 当前实现

- `Tracker` 负责整车状态
- 普通目标：EKF 维护中心、yaw、半径、z 偏移
- 前哨站：单独维护三板相位、当前可见板、切板确认
- 静止状态：用速度阈值 + 持续时间判断

### 关键字段

- `matched_plate`
- `same_count`
- `target.state`
- `target.d_za`
- `target.another_r`
- `target.outpost_observed_plate`
- `target.outpost_phase_error`
- `stationary_mode`

### 对比结论

- `sp_vision_25-main`：更偏多任务、多模型并存
- `dx_vision-main`：建模拆得最细，状态也最丰富
- `wust_vision-main`：任务分层明显，但整体更大
- `autoaim_pro`：保留轻量状态机，够用且好调

### 评价

`autoaim_pro` 这里的重点不是“状态越多越好”，而是“状态足够说明问题就够了”。

---

## 4. 预测层

### 解决的问题

- 下一帧板子会在哪
- 飞行时间内板子会转到哪
- 观测短暂丢失时怎么办

### 当前实现

- 普通目标：
  - 根据速度和飞行时间外推中心、yaw
  - 生成四块板的未来位置
- 前哨站：
  - 用当前相位和角速度外推下一块板
  - 切板间隙允许短时外推
  - 长时间未观测则重建模型

### 关键字段

- `prediction_delay`
- `controller_delay`
- `fly_time`
- `target_v_yaw`
- `selected_idx`
- `selection_pending`

### 对比结论

- `sp_vision_25-main`：更偏“目标预测 + planner”
- `dx_vision-main`：更偏“观测残差 + 模型更新”
- `wust_vision-main`：多种 motion model
- `autoaim_pro`：只保留短外推和重建，不做过重模型库

### 评价

这版的预测层是“轻而够用”，适合实车，不适合写成一整套学术型运动框架。

---

## 5. 决策层

### 解决的问题

- 打哪块板
- 什么时候切板
- 什么时候提前切
- 什么时候开火

### 当前实现

- 普通目标：
  - 距离更近的板优先
  - 加 hysteresis 防抖
  - 新板连续更好才切
- 前哨站：
  - 当前观测板优先
  - 观测丢失时按模型外推
  - 切板确认、防止刚看到就误切

### 当前保留的策略

- 最近板优先
- 稳定板优先
- 观测质量差时不急着换
- 长时间看不到就重建

### 关键字段

- `normal_stable_idx`
- `normal_pending_idx`
- `normal_pending_frames`
- `aim_idx`
- `outpost_pending_idx`
- `outpost_last_switch_time`

### 对比结论

- `sp_vision_25-main`：更偏 planner / 多阶段决策
- `dx_vision-main`：决策和 tracker 绑得更紧
- `wust_vision-main`：FSM 很明确，任务分支多
- `autoaim_pro`：决策层尽量保持简单，靠少量确认逻辑解决抖动

### 评价

这就是 `autoaim_pro` 的核心风格：不追求花哨，追求少跳、少错、能打。

---

## 6. 诊断层

### 解决的问题

- 为什么选错
- 为什么抖
- 为什么没跟上
- 是观测错、建模错、预测错，还是决策错

### 当前推荐字段

- 观测：`obs_x / obs_y / obs_z / obs_yaw`
- 模型：`model_cx / model_cy / model_z / model_yaw`
- 残差：`res_x / res_y / res_z / res_yaw / res_norm`
- 选板：`selected_idx / raw_selected_idx / angular_selected_idx`
- 同步：`imu_sync_valid / imu_query_error_ms / imu_bracket_ms`
- 前哨：`current_plate / next_plate / candidate_phase / selected_plate`

### 对比结论

- `sp_vision_25-main`：曲线不少，但更偏系统调试
- `dx_vision-main`：debug 颗粒度高
- `wust_vision-main`：可视化和消息链很强
- `autoaim_pro`：推荐压成最核心的几条，够定位问题就行

### 评价

诊断层不是越多越好，够定位问题就好。`autoaim_pro` 适合少而准的曲线。

---

## 7. 和四个项目的总体对比

### `sp_vision_25-main`

- 优点：工程完整，时间处理和多模块都比较成熟
- 缺点：整体偏重
- 适合：看系统化实现

### `dx_vision-main`

- 优点：状态和 debug 很细
- 缺点：结构更复杂
- 适合：看“为什么错”

### `wust_vision-main`

- 优点：ROS2 / TF 体系清楚，模块化强
- 缺点：工程层次更多
- 适合：看消息流和坐标流

### `autoaim_pro`

- 优点：轻量、稳、容易调
- 缺点：不追求大而全
- 适合：实车快速闭环

---

## 8. 结论

`autoaim_pro` 当前最合适的路线不是继续变复杂，而是继续把这四件事守住：

- 观测稳定
- 短时外推
- 长时重建
- 最近板优先

如果要再往上提，优先顺序是：

1. 时间同步更稳
2. 选板边界更稳
3. 前哨切板更稳
4. 诊断曲线更少但更准

这版文档就是按这个思路写的。
