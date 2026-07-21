# rm-native-viewer

`rm-native-viewer` 是一个原生 Rust 解码端，不依赖浏览器或网页前端，专门用于 RoboMaster 2026 官方 `0x0310 -> MQTT CustomByteBlock` 自定义客户端视频链路。

当前版本已经按正式链路工作：

- 通过 MQTT 订阅 `CustomByteBlock`
- 把 `0x0310` 的 300 字节紧凑视频分片重组为 H264 字节流
- 调用系统 `ffmpeg` 解码
- 在原生窗口中显示画面并叠加状态信息
- 对非视频 `0x0310` 数据兼容 telemetry v1 解析
- 默认关闭 UDP 3334 + HEVC，旧实验链路需要显式打开

## 功能概览

- 官方主链路：MQTT `CustomByteBlock` -> `0x0310` -> 紧凑视频分片 -> H264
- telemetry fallback：同一条 `0x0310` 链路上兼容解析状态结构体
- 旧链路兼容：可显式启用 UDP 3334 + HEVC
- 原生窗口显示：无浏览器依赖
- headless 烟测：适合 CI、本地回放和远端部署自检
- 默认 client id 为 `101`，也支持直接切到 `1`

## 目录结构

```text
rm-native-viewer/
├── Cargo.toml
├── src/
└── README.md
```

说明：

- `src/`：Rust 主程序与紧凑 `0x0310` 视频头 / telemetry 解析逻辑

## 环境要求

- Ubuntu Linux
- Rust stable（建议使用最新 stable）
- 系统 `ffmpeg`
- Wayland 或 X11 图形环境

推荐依赖安装：

```bash
sudo apt update
sudo apt install -y ffmpeg pkg-config libxkbcommon-dev
curl https://sh.rustup.rs -sSf | sh
```

## 编译

```bash
cargo build --release
```

生成的主程序位于：

```text
target/release/rm-native-viewer
```

## 运行

默认直接运行即可。当前默认值就是官方链路：`192.168.12.1:3333`、`CustomByteBlock`、H264、client id `101`、无 UDP 干扰。

```bash
cargo run --release
```

如果要把 client id 改成 `1`，只需要传这一个值：

```bash
cargo run --release -- 1
```

部署脚本同理：

```bash
./scripts/run-viewer.sh
./scripts/run-viewer.sh 1
```

默认行为：

- MQTT 连接 `192.168.12.1:3333`
- topic 为 `CustomByteBlock`
- client id 为 `101`
- 输入格式为 `h264`
- 显示输出默认自动跟随当前显示器，最高 `2560x1600`；无图形环境时回退为 `800x800`
- UDP/3334 原始图传和本地 `0x0310` UDP 默认关闭

需要显式指定 client id 时：

```bash
cargo run --release -- 101
cargo run --release -- 1
```

## 命令行参数

- `<mqtt-client-id>`：可选快捷参数，例如 `101` 或 `1`
- `--bind <addr:port>`：UDP 监听地址，默认 `0.0.0.0:3334`
- `--allow-source <ip>`：限制 UDP 来源 IP
- `--raw-udp` / `--enable-raw-udp`：启用 UDP/3334 HEVC 原始图传输入，默认关闭
- `--no-udp` / `--no-raw-udp`：关闭 UDP/3334 HEVC 原始图传输入
- `--0310-udp` / `--enable-0310-udp`：启用本地 0x0310 视频 UDP 直连接收，默认关闭
- `--width <n>`：显示宽度，默认自动跟随当前显示器，最高 `2560`
- `--height <n>`：显示高度，默认自动跟随当前显示器，最高 `1600`
- `--ffmpeg <path>`：ffmpeg 路径，默认 `ffmpeg`
- `--mqtt-host <host>`：MQTT 地址，默认 `192.168.12.1`
- `--mqtt-port <port>`：MQTT 端口，默认 `3333`
- `--mqtt-topic <topic>`：topic，默认 `CustomByteBlock`
- `--mqtt-client-id <id>` / `--client-id <id>`：客户端 ID，默认 `101`
- `--input-format <fmt>`：`h264` 或 `hevc`
- `--no-mqtt`：关闭 MQTT，仅保留 UDP 视频输入
- `--headless-seconds <n>`：无窗口烟测

## 支持的链路

### 1. 正式链路

这是当前主通路：

- 输入：MQTT `CustomByteBlock`
- 负载：官方 `0x0310`
- 业务封装：3 字节紧凑视频头
- 视频编码：H264

当前视频分片固定占满 `0x0310` 的 300 字节：

- 3 字节头：`flags_and_payload_hi` / `sequence` / `payload_bytes_lo`
- 297 字节视频净荷

设计目标：

- 50Hz 发送
- 理论总带宽约 15KB/s
- 实际视频净流顶到约 14.85KB/s

### 2. telemetry fallback

如果 `CustomByteBlock.data` 不符合当前紧凑视频头约束，程序会回退为 telemetry v1 解析，用于在画面上叠加：

- 相机在线状态
- gimbal 在线状态与模式
- `frame_seq` / 分辨率 / FPS
- `exposure` / `gain`
- `yaw` / `pitch` / `yaw_vel` / `pitch_vel`
- `bullet_speed` / `bullet_count`
- 状态文本

### 3. 旧实验链路

保留 UDP 3334 + HEVC 兼容模式。

如果你在接旧 sender，要显式加：

```bash
--raw-udp --input-format hevc
```

## 快捷启动

直接按 client id 启动：

```bash
./scripts/run-viewer.sh 101
./scripts/run-viewer.sh 1
```

兼容旧脚本时，也可以这样写：

```bash
./scripts/run-profile.sh 101
./scripts/run-profile.sh 1
```

可用环境变量：

- `RM_VIEWER_BIND`
- `RM_VIEWER_ALLOW_SOURCE`
- `RM_VIEWER_FFMPEG`
- `RM_VIEWER_MQTT_HOST`
- `RM_VIEWER_MQTT_PORT`
- `RM_VIEWER_MQTT_TOPIC`
- `RM_VIEWER_CLIENT_ID`
- `RM_VIEWER_INPUT_FORMAT`
- `RM_VIEWER_ENABLE_RAW_UDP`
- `RM_VIEWER_DISABLE_RAW_UDP`
- `RM_VIEWER_ENABLE_0310_UDP`
- `RM_VIEWER_DISABLE_MQTT`

## 烟测

本地或远端无窗口自检：

```bash
cargo run --release -- --headless-seconds 8
```

如果 8 秒内成功解码至少一帧，程序返回成功。

适合配合录包回放器使用，例如：

- 本地起 MQTT 服务
- 本地起 `replay-official-link`
- viewer 用 `--headless-seconds` 做验流

## 部署

先编译 release：

```bash
cargo build --release
```

把整个目录复制到目标机器后执行：

```bash
./scripts/install-autostart.sh
```

该脚本会：

- 自动执行 `cargo build --release`
- 复制 release 二进制到 `bin/`
- 复制简化启动脚本
- 安装两个桌面自启动项：`Client 1` 和 `Client 101`
- 自动清理旧的 `official/lab` viewer 自启动项

支持切换默认启用的 client id：

```bash
./scripts/install-autostart.sh --default-client 101
./scripts/install-autostart.sh --default-client 1
```

如果你已经手动编译过，也可以跳过构建：

```bash
./scripts/install-autostart.sh --skip-build
```

临时手动启动：

```bash
./bin/rm-native-viewer
./bin/rm-native-viewer-run
./bin/rm-native-viewer-run 101
./bin/rm-native-viewer-run 1
```

## 常见问题

### 1. 一开始提示 `Invalid data found when processing input`

这是正常现象。订阅发生在码流中途时，可能先收到非关键帧，等到 SPS/PPS/IDR 后会自动恢复。

### 2. 看不到视频但 MQTT 已连接

优先检查：

- 当前发送端是不是 H264
- `--input-format` 是否匹配
- `CustomByteBlock` topic 是否正确
- 发送端是否真的在发当前紧凑视频分片

### 3. 只想测 MQTT 主链路，不要 UDP 干扰

默认已经关闭 UDP/3334 和本地 `0x0310` UDP，只走官方 MQTT 主链路。需要旧链路调试时再显式加 `--raw-udp` 或 `--0310-udp`。
