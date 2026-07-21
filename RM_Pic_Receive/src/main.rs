mod custom_client;

use anyhow::{Context, Result, bail};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use custom_client::{
    MetadataSnapshot, VIDEO_0310_HEADER_BYTES, VehicleTelemetry, decode_custom_byte_block,
    parse_vehicle_telemetry, parse_video_0310_chunk,
};
use eframe::egui::{self, Align2, Color32, ColorImage, RichText, TextureHandle, TextureOptions};
use rumqttc::{Client, Event, MqttOptions, Packet, QoS};
use std::collections::{BTreeMap, HashMap};
use std::env;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const DEFAULT_BIND_ADDR: &str = "0.0.0.0:3334";
const DEFAULT_0310_UDP_BIND: &str = "0.0.0.0:3335";
const DEFAULT_WIDTH: usize = 800;
const DEFAULT_HEIGHT: usize = 800;
const DEFAULT_AUTO_MAX_WIDTH: usize = 2560;
const DEFAULT_AUTO_MAX_HEIGHT: usize = 1600;
const DEFAULT_MQTT_HOST: &str = "192.168.12.1";
const DEFAULT_MQTT_PORT: u16 = 3333;
const DEFAULT_MQTT_TOPIC: &str = "CustomByteBlock";
const DEFAULT_MQTT_CLIENT_ID: &str = "101";
const DEFAULT_INPUT_FORMAT: &str = "h264";
const DEFAULT_HEALTHY_GAP_MS: u64 = 3000;
const DEFAULT_FRAME_TIMEOUT_MS: u64 = 2000;
const DEFAULT_MAX_BUFFERED_FRAMES: usize = 64;
const DEFAULT_MAX_FRAME_BYTES: usize = 6 * 1024 * 1024;
const DEFAULT_MAX_SLICES_PER_FRAME: usize = 4096;

#[derive(Clone, Debug)]
struct AppConfig {
    bind_addr: SocketAddr,
    allowed_source: Option<Ipv4Addr>,
    ffmpeg_bin: String,
    mqtt_enabled: bool,
    mqtt_host: String,
    mqtt_port: u16,
    mqtt_topic: String,
    mqtt_client_id: String,
    input_format: String,
    raw_udp_enabled: bool,
    output_width: usize,
    output_height: usize,
    healthy_gap: Duration,
    frame_timeout: Duration,
    max_buffered_frames: usize,
    max_frame_bytes: usize,
    max_slices_per_frame: usize,
    udp_0310_enabled: bool,
    udp_0310_bind: SocketAddr,
    headless_seconds: Option<u64>,
    window_title: String,
}

impl Default for AppConfig {
    fn default() -> Self {
        let (output_width, output_height) = default_output_size();

        Self {
            bind_addr: DEFAULT_BIND_ADDR
                .parse()
                .expect("default bind addr must be valid"),
            allowed_source: None,
            ffmpeg_bin: default_ffmpeg_bin(),
            mqtt_enabled: default_mqtt_enabled(),
            mqtt_host: default_mqtt_host(),
            mqtt_port: default_mqtt_port(),
            mqtt_topic: env::var("RM_VIEWER_MQTT_TOPIC")
                .unwrap_or_else(|_| DEFAULT_MQTT_TOPIC.to_string()),
            mqtt_client_id: env::var("RM_VIEWER_CLIENT_ID")
                .unwrap_or_else(|_| DEFAULT_MQTT_CLIENT_ID.to_string()),
            input_format: env::var("RM_VIEWER_INPUT_FORMAT")
                .unwrap_or_else(|_| DEFAULT_INPUT_FORMAT.to_string()),
            raw_udp_enabled: default_raw_udp_enabled(),
            output_width,
            output_height,
            healthy_gap: Duration::from_millis(DEFAULT_HEALTHY_GAP_MS),
            frame_timeout: Duration::from_millis(DEFAULT_FRAME_TIMEOUT_MS),
            max_buffered_frames: DEFAULT_MAX_BUFFERED_FRAMES,
            max_frame_bytes: DEFAULT_MAX_FRAME_BYTES,
            max_slices_per_frame: DEFAULT_MAX_SLICES_PER_FRAME,
            udp_0310_enabled: default_udp_0310_enabled(),
            udp_0310_bind: DEFAULT_0310_UDP_BIND
                .parse()
                .expect("default 0310 udp bind addr must be valid"),
            headless_seconds: None,
            window_title: "RoboMaster Native Viewer".to_string(),
        }
    }
}

fn default_ffmpeg_bin() -> String {
    if let Ok(path) = env::var("RM_VIEWER_FFMPEG") {
        return path;
    }

    if Path::new("/usr/bin/ffmpeg").exists() {
        return "/usr/bin/ffmpeg".to_string();
    }

    "ffmpeg".to_string()
}

fn default_output_size() -> (usize, usize) {
    detect_display_size()
        .map(|(width, height)| {
            (
                width.min(DEFAULT_AUTO_MAX_WIDTH),
                height.min(DEFAULT_AUTO_MAX_HEIGHT),
            )
        })
        .unwrap_or((DEFAULT_WIDTH, DEFAULT_HEIGHT))
}

fn detect_display_size() -> Option<(usize, usize)> {
    if env::var_os("DISPLAY").is_none() && env::var_os("WAYLAND_DISPLAY").is_none() {
        return None;
    }

    let output = Command::new("xrandr")
        .arg("--current")
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
        .ok()?;

    if !output.status.success() {
        return None;
    }

    parse_xrandr_current_resolution(&String::from_utf8_lossy(&output.stdout))
}

fn parse_xrandr_current_resolution(output: &str) -> Option<(usize, usize)> {
    let lines: Vec<&str> = output.lines().collect();

    for matcher in [" connected primary ", " connected "] {
        for line in &lines {
            if !line.contains(matcher) || line.contains(" disconnected ") {
                continue;
            }

            for token in line.split_whitespace() {
                let Some((resolution, _offset)) = token.split_once('+') else {
                    continue;
                };
                let Some((width, height)) = resolution.split_once('x') else {
                    continue;
                };

                let width = width.parse().ok()?;
                let height = height.parse().ok()?;
                return Some((width, height));
            }
        }
    }

    None
}

fn default_mqtt_enabled() -> bool {
    !matches!(env::var("RM_VIEWER_DISABLE_MQTT"), Ok(value) if value == "1" || value.eq_ignore_ascii_case("true"))
}

fn default_mqtt_host() -> String {
    env::var("RM_VIEWER_MQTT_HOST").unwrap_or_else(|_| DEFAULT_MQTT_HOST.to_string())
}

fn default_mqtt_port() -> u16 {
    env::var("RM_VIEWER_MQTT_PORT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(DEFAULT_MQTT_PORT)
}

fn env_truthy(name: &str) -> bool {
    matches!(env::var(name), Ok(value) if value == "1" || value.eq_ignore_ascii_case("true") || value.eq_ignore_ascii_case("yes") || value.eq_ignore_ascii_case("on"))
}

fn default_raw_udp_enabled() -> bool {
    if env_truthy("RM_VIEWER_DISABLE_RAW_UDP") {
        return false;
    }
    env_truthy("RM_VIEWER_ENABLE_RAW_UDP")
}

fn default_udp_0310_enabled() -> bool {
    env_truthy("RM_VIEWER_ENABLE_0310_UDP")
}

impl AppConfig {
    fn from_args() -> Result<Self> {
        let mut config = Self::default();
        let mut args = env::args().skip(1);

        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--bind" => {
                    let value = args.next().context("--bind 缺少参数")?;
                    config.bind_addr = value
                        .parse()
                        .with_context(|| format!("非法 bind 地址: {value}"))?;
                }
                "--allow-source" => {
                    let value = args.next().context("--allow-source 缺少参数")?;
                    config.allowed_source = Some(
                        value
                            .parse()
                            .with_context(|| format!("非法 IPv4 地址: {value}"))?,
                    );
                }
                "--ffmpeg" => {
                    config.ffmpeg_bin = args.next().context("--ffmpeg 缺少参数")?;
                }
                "--mqtt-host" => {
                    config.mqtt_host = args.next().context("--mqtt-host 缺少参数")?;
                }
                "--mqtt-port" => {
                    let value = args.next().context("--mqtt-port 缺少参数")?;
                    config.mqtt_port = value
                        .parse()
                        .with_context(|| format!("非法 MQTT 端口: {value}"))?;
                }
                "--mqtt-topic" => {
                    config.mqtt_topic = args.next().context("--mqtt-topic 缺少参数")?;
                }
                "--mqtt-client-id" => {
                    config.mqtt_client_id = args.next().context("--mqtt-client-id 缺少参数")?;
                }
                "--client-id" => {
                    config.mqtt_client_id = args.next().context("--client-id 缺少参数")?;
                }
                "--input-format" => {
                    config.input_format = args.next().context("--input-format 缺少参数")?;
                }
                "--raw-udp" | "--enable-raw-udp" => {
                    config.raw_udp_enabled = true;
                }
                "--no-udp" | "--no-raw-udp" => {
                    config.raw_udp_enabled = false;
                }
                "--no-mqtt" => {
                    config.mqtt_enabled = false;
                }
                "--0310-udp-bind" => {
                    let value = args.next().context("--0310-udp-bind 缺少参数")?;
                    config.udp_0310_bind = value
                        .parse()
                        .with_context(|| format!("非法 0310 UDP 地址: {value}"))?;
                }
                "--no-0310-udp" => {
                    config.udp_0310_enabled = false;
                }
                "--0310-udp" | "--enable-0310-udp" => {
                    config.udp_0310_enabled = true;
                }
                "--width" => {
                    let value = args.next().context("--width 缺少参数")?;
                    config.output_width = value
                        .parse()
                        .with_context(|| format!("非法宽度: {value}"))?;
                }
                "--height" => {
                    let value = args.next().context("--height 缺少参数")?;
                    config.output_height = value
                        .parse()
                        .with_context(|| format!("非法高度: {value}"))?;
                }
                "--headless-seconds" => {
                    let value = args.next().context("--headless-seconds 缺少参数")?;
                    config.headless_seconds = Some(
                        value
                            .parse()
                            .with_context(|| format!("非法秒数: {value}"))?,
                    );
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other if !other.starts_with('-') => {
                    config.mqtt_client_id = other.to_string();
                }
                other => bail!("未知参数: {other}"),
            }
        }

        Ok(config)
    }
}

fn print_help() {
    println!("rm-native-viewer\n\n");
    println!(
        "  [mqtt-client-id]             可选快捷参数，例如: rm-native-viewer 101 或 rm-native-viewer 1"
    );
    println!("  --bind <addr:port>           监听地址，默认 {DEFAULT_BIND_ADDR}");
    println!("  --allow-source <ip>          仅接收该源 IP 的 UDP 包");
    println!("  --ffmpeg <path>              ffmpeg 可执行文件路径，默认 ffmpeg");
    println!("  --mqtt-host <host>           MQTT 服务器地址，默认 {DEFAULT_MQTT_HOST}");
    println!("  --mqtt-port <port>           MQTT 服务器端口，默认 {DEFAULT_MQTT_PORT}");
    println!("  --mqtt-topic <topic>         自定义字节流 topic，默认 {DEFAULT_MQTT_TOPIC}");
    println!("  --mqtt-client-id <id>        客户端 ID，默认 {DEFAULT_MQTT_CLIENT_ID}");
    println!("  --client-id <id>             --mqtt-client-id 的简写");
    println!(
        "  --input-format <fmt>         ffmpeg 输入格式，默认 {DEFAULT_INPUT_FORMAT}，可设 hevc 兼容旧 UDP sender"
    );
    println!("  --raw-udp / --enable-raw-udp 启用 UDP/3334 HEVC 原始图传输入，默认关闭");
    println!("  --no-udp / --no-raw-udp      禁用 UDP/3334 HEVC 原始图传输入");
    println!("  --no-mqtt                    禁用 MQTT metadata 接收");
    println!("  --0310-udp-bind <addr:port>  0x0310 UDP 监听地址，默认 {DEFAULT_0310_UDP_BIND}");
    println!("  --0310-udp / --enable-0310-udp 启用 0x0310 UDP 直连接收，默认关闭");
    println!("  --no-0310-udp                禁用 0x0310 UDP 直连接收");
    println!(
        "  --width <n>                  输出宽度，默认自动跟随显示器，最高 {DEFAULT_AUTO_MAX_WIDTH}；无显示环境回退 {DEFAULT_WIDTH}"
    );
    println!(
        "  --height <n>                 输出高度，默认自动跟随显示器，最高 {DEFAULT_AUTO_MAX_HEIGHT}；无显示环境回退 {DEFAULT_HEIGHT}"
    );
    println!("  --headless-seconds <n>       无窗口烟测模式");
}

#[derive(Clone, Debug)]
struct DecodedFrame {
    rgba: Vec<u8>,
    width: usize,
    height: usize,
}

#[derive(Default)]
struct StatusSnapshot {
    udp: UdpStats,
    udp_0310: Udp0310Stats,
    decoder: DecoderStats,
    metadata: MetadataSnapshot,
}

#[derive(Default)]
struct Udp0310Stats {
    bound: bool,
    packets_received: u64,
    bytes_received: u64,
    assembled_frames: u64,
    last_packet_at: Option<Instant>,
    last_source: Option<SocketAddr>,
    last_seq: Option<u32>,
}

#[derive(Default)]
struct UdpStats {
    bound: bool,
    packets_received: u64,
    bytes_received: u64,
    assembled_frames: u64,
    dropped_packets: u64,
    dropped_frames: u64,
    buffered_frames: usize,
    last_packet_at: Option<Instant>,
    last_packet_source: Option<SocketAddr>,
    last_frame_id: Option<u16>,
}

#[derive(Default)]
struct DecoderStats {
    ffmpeg_running: bool,
    decoded_frames: u64,
    last_frame_at: Option<Instant>,
    last_error: Option<String>,
}

#[derive(Clone)]
struct StatusView {
    udp_ok: bool,
    decode_ok: bool,
    mqtt_ok: bool,
    packets_received: u64,
    bytes_received: u64,
    assembled_frames: u64,
    dropped_packets: u64,
    dropped_frames: u64,
    buffered_frames: usize,
    last_frame_id: String,
    last_source: String,
    last_packet_age_ms: Option<u128>,
    last_decode_age_ms: Option<u128>,
    last_mqtt_age_ms: Option<u128>,
    decoded_frames: u64,
    ffmpeg_running: bool,
    last_error: Option<String>,
    udp_0310_ok: bool,
    udp_0310_packets: u64,
    udp_0310_assembled: u64,
    udp_0310_last_seq: String,
    udp_0310_age_ms: Option<u128>,
    mqtt_enabled: bool,
    mqtt_connected: bool,
    mqtt_messages_received: u64,
    mqtt_error: Option<String>,
    telemetry: Option<VehicleTelemetry>,
}

fn snapshot_view(shared: &Arc<Mutex<StatusSnapshot>>, healthy_gap: Duration) -> StatusView {
    let now = Instant::now();
    let guard = shared.lock().expect("status mutex poisoned");
    let udp_age = guard
        .udp
        .last_packet_at
        .map(|instant| now.saturating_duration_since(instant).as_millis());
    let decode_age = guard
        .decoder
        .last_frame_at
        .map(|instant| now.saturating_duration_since(instant).as_millis());
    let mqtt_age = guard
        .metadata
        .last_message_at
        .map(|instant| now.saturating_duration_since(instant).as_millis());
    let udp_0310_age = guard
        .udp_0310
        .last_packet_at
        .map(|instant| now.saturating_duration_since(instant).as_millis());

    StatusView {
        udp_ok: guard.udp.bound
            && udp_age
                .map(|age| age <= healthy_gap.as_millis())
                .unwrap_or(false),
        decode_ok: guard.decoder.ffmpeg_running
            && decode_age
                .map(|age| age <= healthy_gap.as_millis())
                .unwrap_or(false),
        mqtt_ok: guard.metadata.enabled
            && guard.metadata.connected
            && mqtt_age
                .map(|age| age <= healthy_gap.as_millis())
                .unwrap_or(false),
        packets_received: guard.udp.packets_received,
        bytes_received: guard.udp.bytes_received,
        assembled_frames: guard.udp.assembled_frames,
        dropped_packets: guard.udp.dropped_packets,
        dropped_frames: guard.udp.dropped_frames,
        buffered_frames: guard.udp.buffered_frames,
        last_frame_id: guard
            .udp
            .last_frame_id
            .map(|value| value.to_string())
            .unwrap_or_else(|| "-".to_string()),
        last_source: guard
            .udp
            .last_packet_source
            .map(|value| value.to_string())
            .unwrap_or_else(|| "-".to_string()),
        last_packet_age_ms: udp_age,
        udp_0310_ok: guard.udp_0310.bound
            && udp_0310_age
                .map(|age| age <= healthy_gap.as_millis())
                .unwrap_or(false),
        udp_0310_packets: guard.udp_0310.packets_received,
        udp_0310_assembled: guard.udp_0310.assembled_frames,
        udp_0310_last_seq: guard
            .udp_0310
            .last_seq
            .map(|v| v.to_string())
            .unwrap_or_else(|| "-".to_string()),
        udp_0310_age_ms: udp_0310_age,
        last_decode_age_ms: decode_age,
        last_mqtt_age_ms: mqtt_age,
        decoded_frames: guard.decoder.decoded_frames,
        ffmpeg_running: guard.decoder.ffmpeg_running,
        last_error: guard.decoder.last_error.clone(),
        mqtt_enabled: guard.metadata.enabled,
        mqtt_connected: guard.metadata.connected,
        mqtt_messages_received: guard.metadata.messages_received,
        mqtt_error: guard.metadata.last_error.clone(),
        telemetry: guard.metadata.telemetry.clone(),
    }
}

fn set_decoder_error(shared: &Arc<Mutex<StatusSnapshot>>, message: impl Into<String>) {
    if let Ok(mut guard) = shared.lock() {
        guard.decoder.last_error = Some(message.into());
    }
}

fn set_metadata_error(shared: &Arc<Mutex<StatusSnapshot>>, message: impl Into<String>) {
    if let Ok(mut guard) = shared.lock() {
        guard.metadata.last_error = Some(message.into());
        guard.metadata.connected = false;
    }
}

fn sequence_follows(prev: u32, next: u32, modulus: u32) -> bool {
    if modulus > 0 {
        next == prev.wrapping_add(1) % modulus
    } else {
        next == prev.wrapping_add(1)
    }
}

fn spawn_metadata_receiver(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    encoded_tx: Sender<Vec<u8>>,
) -> Result<()> {
    if let Ok(mut guard) = shared.lock() {
        guard.metadata.enabled = config.mqtt_enabled;
    }

    if !config.mqtt_enabled {
        return Ok(());
    }

    thread::Builder::new()
        .name("rm-native-mqtt".to_string())
        .spawn(move || mqtt_receiver_loop(config, shared, encoded_tx))
        .context("无法启动 MQTT 接收线程")?;
    Ok(())
}

fn mqtt_receiver_loop(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    encoded_tx: Sender<Vec<u8>>,
) {
    // chunk seq 连续性追踪: 0x0310 视频分片没有重传;
    // 只要丢一个 chunk, 后续直接拼接会让 ffmpeg 看到错乱的 NAL, 黑屏 / 花屏.
    // 策略: 一旦发现 sequence gap, 就把缓冲清掉, 丢弃后续到第一个 SPS (NAL type 7),
    //       下一个 IDR + SPS/PPS 到达后再恢复送解码器.
    // 启动时: 同样阻塞, 等到第一个 SPS 出现才开始喂 ffmpeg, 否则 ffmpeg 会因为
    //       从 NAL 中间开始 parse 而内部状态损坏, 即使后续 SPS/IDR 都收到也无法恢复.
    let mut last_seq: Option<u32> = None;
    let mut blocked_until_sps: bool = true; // 启动即阻塞
    let mut dropped_chunks_after_gap: u64 = 0;

    loop {
        let mut mqtt_options = MqttOptions::new(
            config.mqtt_client_id.clone(),
            config.mqtt_host.clone(),
            config.mqtt_port,
        );
        mqtt_options.set_keep_alive(Duration::from_secs(5));

        let (client, mut connection) = Client::new(mqtt_options, 10);
        match client.subscribe(config.mqtt_topic.clone(), QoS::AtLeastOnce) {
            Ok(_) => {
                if let Ok(mut guard) = shared.lock() {
                    guard.metadata.connected = true;
                    guard.metadata.last_error = None;
                }
            }
            Err(error) => {
                set_metadata_error(&shared, format!("MQTT 订阅失败: {error}"));
                thread::sleep(Duration::from_secs(1));
                continue;
            }
        }

        let mut disconnected = false;

        for notification in connection.iter() {
            match notification {
                Ok(Event::Incoming(Packet::Publish(publish))) => {
                    let raw_bytes = match decode_custom_byte_block(&publish.payload) {
                        Some(bytes) => bytes,
                        None => {
                            set_metadata_error(&shared, "CustomByteBlock protobuf 解析失败");
                            continue;
                        }
                    };

                    if let Some(chunk) = parse_video_0310_chunk(&raw_bytes) {
                        let now = Instant::now();
                        let payload_len = chunk.payload.len();

                        // 检测 sequence gap. flags & 1 == 1 表示 stream 重置;
                        // 否则若 seq 不是 last_seq+1 则进入 "丢弃直到下一个 SPS" 模式.
                        let stream_reset = (chunk.flags & 1) != 0;
                        if stream_reset {
                            // 流重启, 清空 ffmpeg 上游, 但仍然要等下一个 SPS 才能喂解码器.
                            blocked_until_sps = true;
                        } else if let Some(prev) = last_seq {
                            if !sequence_follows(prev, chunk.sequence, chunk.sequence_modulus) {
                                blocked_until_sps = true;
                                dropped_chunks_after_gap =
                                    dropped_chunks_after_gap.saturating_add(1);
                            }
                        }
                        last_seq = Some(chunk.sequence);

                        let send_payload: Option<Vec<u8>> = if blocked_until_sps {
                            // 在当前 chunk 内查找 SPS NAL (type=7).
                            // SPS 起始码: 0x00 00 (00) 01, 紧随的字节 & 0x1F == 7.
                            let p = chunk.payload.as_slice();
                            let mut sps_at: Option<usize> = None;
                            let mut i = 0usize;
                            while i + 3 < p.len() {
                                let three_byte_sc = p[i] == 0 && p[i + 1] == 0 && p[i + 2] == 1;
                                let four_byte_sc = i + 4 < p.len()
                                    && p[i] == 0
                                    && p[i + 1] == 0
                                    && p[i + 2] == 0
                                    && p[i + 3] == 1;
                                if three_byte_sc {
                                    if (p[i + 3] & 0x1F) == 7 {
                                        sps_at = Some(i);
                                        break;
                                    }
                                    i += 3;
                                } else if four_byte_sc {
                                    if (p[i + 4] & 0x1F) == 7 {
                                        sps_at = Some(i);
                                        break;
                                    }
                                    i += 4;
                                } else {
                                    i += 1;
                                }
                            }
                            match sps_at {
                                Some(off) => {
                                    blocked_until_sps = false;
                                    Some(p[off..].to_vec())
                                }
                                None => {
                                    dropped_chunks_after_gap =
                                        dropped_chunks_after_gap.saturating_add(1);
                                    None
                                }
                            }
                        } else if payload_len > 0 {
                            Some(chunk.payload.clone())
                        } else {
                            None
                        };

                        if let Some(bytes) = send_payload {
                            if let Err(error) = encoded_tx.send(bytes) {
                                set_decoder_error(
                                    &shared,
                                    format!("0310 视频解码通道已断开: {error}"),
                                );
                                continue;
                            }
                        }

                        if let Ok(mut guard) = shared.lock() {
                            guard.metadata.connected = true;
                            guard.metadata.messages_received += 1;
                            guard.metadata.last_message_at = Some(now);
                            guard.metadata.last_error = None;
                            guard.udp.bound = true;
                            guard.udp.packets_received += 1;
                            guard.udp.bytes_received += payload_len as u64;
                            guard.udp.assembled_frames += 1;
                            guard.udp.last_packet_at = Some(now);
                            guard.udp.last_packet_source = None;
                            guard.udp.last_frame_id = Some((chunk.sequence & 0xFFFF) as u16);
                            guard.udp.buffered_frames = 0;
                        }
                        continue;
                    }

                    let telemetry = match parse_vehicle_telemetry(&raw_bytes) {
                        Some(telemetry) => telemetry,
                        None => {
                            set_metadata_error(&shared, "vehicle telemetry v1 解析失败");
                            continue;
                        }
                    };

                    if let Ok(mut guard) = shared.lock() {
                        guard.metadata.connected = true;
                        guard.metadata.messages_received += 1;
                        guard.metadata.last_message_at = Some(Instant::now());
                        guard.metadata.last_error = None;
                        guard.metadata.telemetry = Some(telemetry);
                    }
                }
                Ok(_) => {}
                Err(error) => {
                    set_metadata_error(&shared, format!("MQTT 连接中断: {error}"));
                    disconnected = true;
                    break;
                }
            }
        }

        if !disconnected {
            set_metadata_error(&shared, "MQTT 连接已结束");
        }

        thread::sleep(Duration::from_secs(1));
    }
}

fn spawn_udp_0310_receiver(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    encoded_tx: Sender<Vec<u8>>,
) -> Result<()> {
    if !config.udp_0310_enabled {
        return Ok(());
    }

    thread::Builder::new()
        .name("rm-native-udp0310".to_string())
        .spawn(move || udp_0310_receiver_loop(config, shared, encoded_tx))
        .context("无法启动 0x0310 UDP 接收线程")?;
    Ok(())
}

fn udp_0310_receiver_loop(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    encoded_tx: Sender<Vec<u8>>,
) {
    let socket = match UdpSocket::bind(config.udp_0310_bind) {
        Ok(socket) => socket,
        Err(error) => {
            set_decoder_error(
                &shared,
                format!("0310 UDP bind {} 失败: {error}", config.udp_0310_bind),
            );
            return;
        }
    };

    if let Err(error) = socket.set_read_timeout(Some(Duration::from_millis(500))) {
        set_decoder_error(&shared, format!("0310 UDP 设置超时失败: {error}"));
        return;
    }

    if let Ok(mut guard) = shared.lock() {
        guard.udp_0310.bound = true;
    }

    let mut packet = vec![0_u8; 300];

    loop {
        match socket.recv_from(&mut packet) {
            Ok((size, source)) => {
                if size < VIDEO_0310_HEADER_BYTES {
                    continue;
                }

                let chunk = match parse_video_0310_chunk(&packet[..size]) {
                    Some(chunk) => chunk,
                    None => continue,
                };

                let now = Instant::now();
                let payload_len = chunk.payload.len();

                if payload_len > 0 {
                    if let Err(error) = encoded_tx.send(chunk.payload) {
                        set_decoder_error(&shared, format!("0310 UDP 解码通道已断开: {error}"));
                        continue;
                    }
                }

                if let Ok(mut guard) = shared.lock() {
                    guard.udp_0310.packets_received += 1;
                    guard.udp_0310.bytes_received += payload_len as u64;
                    guard.udp_0310.assembled_frames += 1;
                    guard.udp_0310.last_packet_at = Some(now);
                    guard.udp_0310.last_source = Some(source);
                    guard.udp_0310.last_seq = Some(chunk.sequence);
                }
            }
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock
                    || error.kind() == std::io::ErrorKind::TimedOut =>
            {
                // timeout, continue
            }
            Err(error) => {
                set_decoder_error(&shared, format!("0310 UDP 接收错误: {error}"));
                break;
            }
        }
    }
}

struct PartialFrame {
    total_bytes: usize,
    received_bytes: usize,
    slices: BTreeMap<u16, Vec<u8>>,
    updated_at: Instant,
}

fn spawn_udp_receiver(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    hevc_tx: Sender<Vec<u8>>,
) -> Result<()> {
    if !config.raw_udp_enabled {
        return Ok(());
    }

    thread::Builder::new()
        .name("rm-native-udp".to_string())
        .spawn(move || udp_receiver_loop(config, shared, hevc_tx))
        .context("无法启动 UDP 接收线程")?;
    Ok(())
}

fn udp_receiver_loop(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    hevc_tx: Sender<Vec<u8>>,
) {
    let socket = match UdpSocket::bind(config.bind_addr) {
        Ok(socket) => socket,
        Err(error) => {
            set_decoder_error(&shared, format!("UDP bind 失败: {error}"));
            return;
        }
    };

    if let Err(error) = socket.set_read_timeout(Some(Duration::from_millis(200))) {
        set_decoder_error(&shared, format!("UDP 设置超时失败: {error}"));
        return;
    }

    if let Ok(mut guard) = shared.lock() {
        guard.udp.bound = true;
    }

    let mut frame_buffer: HashMap<u16, PartialFrame> = HashMap::new();
    let mut packet = vec![0_u8; 2048];

    loop {
        match socket.recv_from(&mut packet) {
            Ok((size, source)) => {
                if let Some(allowed) = config.allowed_source {
                    if source.ip() != IpAddr::V4(allowed) {
                        if let Ok(mut guard) = shared.lock() {
                            guard.udp.dropped_packets += 1;
                        }
                        continue;
                    }
                }

                ingest_packet(
                    &config,
                    &shared,
                    &hevc_tx,
                    &mut frame_buffer,
                    &packet[..size],
                    source,
                );

                cleanup_expired_frames(&config, &shared, &mut frame_buffer);
            }
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock
                    || error.kind() == std::io::ErrorKind::TimedOut =>
            {
                cleanup_expired_frames(&config, &shared, &mut frame_buffer);
            }
            Err(error) => {
                set_decoder_error(&shared, format!("UDP 接收错误: {error}"));
                break;
            }
        }
    }
}

fn ingest_packet(
    config: &AppConfig,
    shared: &Arc<Mutex<StatusSnapshot>>,
    hevc_tx: &Sender<Vec<u8>>,
    frame_buffer: &mut HashMap<u16, PartialFrame>,
    packet: &[u8],
    source: SocketAddr,
) {
    if packet.len() < 9 {
        if let Ok(mut guard) = shared.lock() {
            guard.udp.dropped_packets += 1;
        }
        return;
    }

    let frame_id = u16::from_be_bytes([packet[0], packet[1]]);
    let slice_id = u16::from_be_bytes([packet[2], packet[3]]);
    let total_bytes = u32::from_be_bytes([packet[4], packet[5], packet[6], packet[7]]) as usize;
    let payload = &packet[8..];

    if total_bytes == 0 || total_bytes > config.max_frame_bytes || payload.is_empty() {
        if let Ok(mut guard) = shared.lock() {
            guard.udp.dropped_packets += 1;
        }
        return;
    }

    let now = Instant::now();
    if let Ok(mut guard) = shared.lock() {
        guard.udp.packets_received += 1;
        guard.udp.bytes_received += payload.len() as u64;
        guard.udp.last_packet_at = Some(now);
        guard.udp.last_packet_source = Some(source);
        guard.udp.last_frame_id = Some(frame_id);
    }

    if frame_buffer.len() >= config.max_buffered_frames && !frame_buffer.contains_key(&frame_id) {
        evict_oldest_frame(shared, frame_buffer, "缓存超限");
    }

    let entry = frame_buffer
        .entry(frame_id)
        .or_insert_with(|| PartialFrame {
            total_bytes,
            received_bytes: 0,
            slices: BTreeMap::new(),
            updated_at: now,
        });

    if entry.total_bytes != total_bytes {
        drop_frame(shared, frame_buffer, frame_id, "totalBytes 不一致");
        if let Ok(mut guard) = shared.lock() {
            guard.udp.dropped_packets += 1;
        }
        return;
    }

    if !entry.slices.contains_key(&slice_id) {
        if entry.slices.len() >= config.max_slices_per_frame {
            drop_frame(shared, frame_buffer, frame_id, "分片数超上限");
            if let Ok(mut guard) = shared.lock() {
                guard.udp.dropped_packets += 1;
            }
            return;
        }

        entry.updated_at = now;
        entry.received_bytes += payload.len();
        entry.slices.insert(slice_id, payload.to_vec());
    }

    if entry.received_bytes > entry.total_bytes {
        drop_frame(shared, frame_buffer, frame_id, "累计字节超出 totalBytes");
        if let Ok(mut guard) = shared.lock() {
            guard.udp.dropped_packets += 1;
        }
        return;
    }

    if entry.received_bytes == entry.total_bytes {
        let complete = match frame_buffer.remove(&frame_id) {
            Some(frame) => {
                let mut output = Vec::with_capacity(frame.total_bytes);
                for chunk in frame.slices.values() {
                    output.extend_from_slice(chunk);
                }
                output
            }
            None => return,
        };

        if complete.len() != total_bytes {
            if let Ok(mut guard) = shared.lock() {
                guard.udp.dropped_frames += 1;
            }
            return;
        }

        match hevc_tx.try_send(complete) {
            Ok(()) => {
                if let Ok(mut guard) = shared.lock() {
                    guard.udp.assembled_frames += 1;
                }
            }
            Err(TrySendError::Full(_)) => {
                if let Ok(mut guard) = shared.lock() {
                    guard.udp.dropped_frames += 1;
                }
            }
            Err(TrySendError::Disconnected(_)) => {
                set_decoder_error(shared, "编码视频解码通道已断开");
            }
        }
    }

    if let Ok(mut guard) = shared.lock() {
        guard.udp.buffered_frames = frame_buffer.len();
    }
}

fn cleanup_expired_frames(
    config: &AppConfig,
    shared: &Arc<Mutex<StatusSnapshot>>,
    frame_buffer: &mut HashMap<u16, PartialFrame>,
) {
    let now = Instant::now();
    let expired: Vec<u16> = frame_buffer
        .iter()
        .filter_map(|(frame_id, frame)| {
            (now.duration_since(frame.updated_at) > config.frame_timeout).then_some(*frame_id)
        })
        .collect();

    for frame_id in expired {
        drop_frame(shared, frame_buffer, frame_id, "分片超时");
    }

    if let Ok(mut guard) = shared.lock() {
        guard.udp.buffered_frames = frame_buffer.len();
    }
}

fn drop_frame(
    shared: &Arc<Mutex<StatusSnapshot>>,
    frame_buffer: &mut HashMap<u16, PartialFrame>,
    frame_id: u16,
    _reason: &str,
) {
    if frame_buffer.remove(&frame_id).is_some() {
        if let Ok(mut guard) = shared.lock() {
            guard.udp.dropped_frames += 1;
            guard.udp.buffered_frames = frame_buffer.len();
        }
    }
}

fn evict_oldest_frame(
    shared: &Arc<Mutex<StatusSnapshot>>,
    frame_buffer: &mut HashMap<u16, PartialFrame>,
    _reason: &str,
) {
    let oldest_id = frame_buffer
        .iter()
        .min_by_key(|(_, frame)| frame.updated_at)
        .map(|(frame_id, _)| *frame_id);

    if let Some(frame_id) = oldest_id {
        drop_frame(shared, frame_buffer, frame_id, "缓存超限");
    }
}

fn spawn_decoder(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    hevc_rx: Receiver<Vec<u8>>,
    frame_tx: Option<Sender<DecodedFrame>>,
) -> Result<()> {
    thread::Builder::new()
        .name("rm-native-decoder".to_string())
        .spawn(move || decoder_loop(config, shared, hevc_rx, frame_tx))
        .context("无法启动解码线程")?;
    Ok(())
}

fn decoder_loop(
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    hevc_rx: Receiver<Vec<u8>>,
    frame_tx: Option<Sender<DecodedFrame>>,
) {
    loop {
        let first_frame = match hevc_rx.recv() {
            Ok(frame) => frame,
            Err(_) => {
                eprintln!("decoder: 编码视频输入通道关闭，解码线程退出");
                break;
            }
        };

        let mut child = match spawn_ffmpeg(&config) {
            Ok(child) => child,
            Err(error) => {
                eprintln!("decoder: 启动 ffmpeg 失败: {error:#}");
                set_decoder_error(&shared, format!("启动 ffmpeg 失败: {error:#}"));
                return;
            }
        };

        if let Ok(mut guard) = shared.lock() {
            guard.decoder.ffmpeg_running = true;
        }

        let mut stdin = match child.stdin.take() {
            Some(stdin) => stdin,
            None => {
                set_decoder_error(&shared, "ffmpeg stdin 不可用");
                return;
            }
        };

        let stdout = match child.stdout.take() {
            Some(stdout) => stdout,
            None => {
                set_decoder_error(&shared, "ffmpeg stdout 不可用");
                return;
            }
        };

        let stderr = match child.stderr.take() {
            Some(stderr) => stderr,
            None => {
                set_decoder_error(&shared, "ffmpeg stderr 不可用");
                return;
            }
        };

        let stdout_state = shared.clone();
        let stderr_state = shared.clone();
        let bytes_per_frame = config.output_width * config.output_height * 4;
        let output_width = config.output_width;
        let output_height = config.output_height;
        let ui_tx = frame_tx.clone();

        thread::spawn(move || {
            let mut stdout = stdout;
            loop {
                let mut buffer = vec![0_u8; bytes_per_frame];
                match stdout.read_exact(&mut buffer) {
                    Ok(()) => {
                        if let Ok(mut guard) = stdout_state.lock() {
                            guard.decoder.decoded_frames += 1;
                            guard.decoder.last_frame_at = Some(Instant::now());
                            guard.decoder.last_error = None;
                        }

                        if let Some(tx) = &ui_tx {
                            let frame = DecodedFrame {
                                rgba: buffer,
                                width: output_width,
                                height: output_height,
                            };

                            let _ = tx.try_send(frame);
                        }
                    }
                    Err(error) => {
                        set_decoder_error(&stdout_state, format!("ffmpeg 输出中断: {error}"));
                        break;
                    }
                }
            }
        });

        thread::spawn(move || {
            let reader = BufReader::new(stderr);
            for line in reader.lines() {
                match line {
                    Ok(line) if !line.trim().is_empty() => set_decoder_error(&stderr_state, line),
                    Ok(_) => {}
                    Err(error) => {
                        set_decoder_error(
                            &stderr_state,
                            format!("ffmpeg stderr 读取失败: {error}"),
                        );
                        break;
                    }
                }
            }
        });

        let mut restart_requested = false;
        let mut input_closed = false;

        if let Err(error) = stdin.write_all(&first_frame) {
            eprintln!("decoder: 首帧写入 ffmpeg 失败: {error}");
            set_decoder_error(&shared, format!("写入 ffmpeg 失败: {error}"));
            restart_requested = true;
        }

        while !restart_requested {
            match hevc_rx.recv() {
                Ok(frame) => {
                    if let Err(error) = stdin.write_all(&frame) {
                        eprintln!("decoder: 后续帧写入 ffmpeg 失败，准备重启: {error}");
                        set_decoder_error(&shared, format!("写入 ffmpeg 失败: {error}"));
                        restart_requested = true;
                    }
                }
                Err(_) => {
                    eprintln!("decoder: 编码视频输入通道在运行中关闭");
                    input_closed = true;
                    break;
                }
            }
        }

        let _ = child.kill();
        let _ = child.wait();

        if let Ok(mut guard) = shared.lock() {
            guard.decoder.ffmpeg_running = false;
        }

        if input_closed {
            eprintln!("decoder: 输入通道关闭，结束解码循环");
            break;
        }
    }
}

fn ffmpeg_supports_filter(ffmpeg_bin: &str, filter_name: &str) -> bool {
    let output = match Command::new(ffmpeg_bin)
        .args(["-hide_banner", "-filters"])
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .output()
    {
        Ok(output) => output,
        Err(_) => return false,
    };

    if !output.status.success() {
        return false;
    }

    String::from_utf8_lossy(&output.stdout)
        .lines()
        .any(|line| line.split_whitespace().nth(1) == Some(filter_name))
}

fn build_ffmpeg_filter(config: &AppConfig) -> String {
    if ffmpeg_supports_filter(&config.ffmpeg_bin, "libplacebo") {
        let mut filter = format!(
            "libplacebo=w={w}:h={h}:force_original_aspect_ratio=decrease:normalize_sar=true:upscaler=ewa_lanczos:downscaler=ewa_lanczos:deband=true:deband_iterations=16:deband_radius=64:deband_threshold=12:deband_grain=2:disable_builtin=true:lut_entries=256:force_dither=true:dither_temporal=true,pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:black",
            w = config.output_width,
            h = config.output_height
        );

        if ffmpeg_supports_filter(&config.ffmpeg_bin, "cas") {
            filter.push_str(",cas=strength=0.6");
        }

        return filter;
    }

    format!(
        "scale={w}:{h}:force_original_aspect_ratio=decrease,pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:black",
        w = config.output_width,
        h = config.output_height
    )
}

fn spawn_ffmpeg(config: &AppConfig) -> Result<std::process::Child> {
    let filter = build_ffmpeg_filter(config);

    Command::new(&config.ffmpeg_bin)
        .args([
            "-loglevel",
            "warning",
            // 注意: 不再使用 "-fflags nobuffer -probesize 32 -analyzeduration 0",
            // 这套超低延迟参数仅适合 file/mmap 输入. 在 stdin pipe + 真实链路慢喂场景下,
            // ffmpeg parser 会在还没看到完整 NAL 时就 timeout 进入 codec init,
            // 从此 decoder 内部状态 lock 在错乱(花屏 + "non-existing PPS 0").
            // probesize/analyzeduration 给足让 parser 有时间收齐 SPS/PPS+IDR.
            "-fflags",
            "+discardcorrupt",
            "-flags",
            "low_delay",
            "-probesize",
            "65536",
            "-analyzeduration",
            "1000000",
            "-err_detect",
            "ignore_err",
            "-f",
            &config.input_format,
            "-i",
            "pipe:0",
            "-an",
            "-vf",
            &filter,
            "-pix_fmt",
            "rgba",
            "-f",
            "rawvideo",
            "pipe:1",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .with_context(|| format!("无法执行 ffmpeg: {}", config.ffmpeg_bin))
}

struct ViewerApp {
    config: AppConfig,
    shared: Arc<Mutex<StatusSnapshot>>,
    frame_rx: Receiver<DecodedFrame>,
    texture: Option<TextureHandle>,
    displayed_frames: u32,
    display_fps: f32,
    fps_window_started_at: Instant,
}

impl ViewerApp {
    fn new(
        config: AppConfig,
        shared: Arc<Mutex<StatusSnapshot>>,
        frame_rx: Receiver<DecodedFrame>,
    ) -> Self {
        Self {
            config,
            shared,
            frame_rx,
            texture: None,
            displayed_frames: 0,
            display_fps: 0.0,
            fps_window_started_at: Instant::now(),
        }
    }

    fn poll_frames(&mut self, ctx: &egui::Context) {
        while let Ok(frame) = self.frame_rx.try_recv() {
            let image =
                ColorImage::from_rgba_unmultiplied([frame.width, frame.height], &frame.rgba);
            match self.texture.as_mut() {
                Some(texture) => texture.set(image, TextureOptions::LINEAR),
                None => {
                    self.texture =
                        Some(ctx.load_texture("rm-native-frame", image, TextureOptions::LINEAR));
                }
            }

            self.displayed_frames += 1;
        }

        let elapsed = self.fps_window_started_at.elapsed();
        if elapsed >= Duration::from_secs(1) {
            self.display_fps = self.displayed_frames as f32 / elapsed.as_secs_f32();
            self.displayed_frames = 0;
            self.fps_window_started_at = Instant::now();
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_frames(ctx);

        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(Color32::BLACK))
            .show(ctx, |ui| {
                let available = ui.available_size();
                if let Some(texture) = &self.texture {
                    let image_size = texture.size_vec2();
                    let scale = (available.x / image_size.x)
                        .min(available.y / image_size.y)
                        .max(0.1);
                    let draw_size = image_size * scale;
                    ui.centered_and_justified(|ui| {
                        ui.add(
                            egui::Image::new((texture.id(), texture.size_vec2()))
                                .fit_to_exact_size(draw_size),
                        );
                    });
                } else {
                    ui.centered_and_justified(|ui| {
                        ui.label(
                            RichText::new("等待视频流...")
                                .color(Color32::LIGHT_GRAY)
                                .size(24.0),
                        );
                    });
                }
            });

        let view = snapshot_view(&self.shared, self.config.healthy_gap);
        let _udp_color = if view.udp_ok {
            Color32::from_rgb(54, 179, 126)
        } else {
            Color32::from_rgb(214, 69, 65)
        };
        let decode_color = if view.decode_ok {
            Color32::from_rgb(54, 179, 126)
        } else {
            Color32::from_rgb(214, 162, 65)
        };
        let mqtt_color = if !view.mqtt_enabled {
            Color32::from_rgb(130, 130, 130)
        } else if view.mqtt_ok {
            Color32::from_rgb(54, 179, 126)
        } else {
            Color32::from_rgb(214, 162, 65)
        };

        egui::Area::new("status_overlay".into())
            .anchor(Align2::LEFT_TOP, [16.0, 16.0])
            .show(ctx, |ui| {
                egui::Frame::default()
                    .fill(Color32::from_black_alpha(190))
                    .corner_radius(8.0)
                    .inner_margin(egui::Margin::same(10))
                    .show(ui, |ui| {
                        ui.set_width(420.0);
                        ui.label(
                            RichText::new("RM Native Viewer")
                                .strong()
                                .color(Color32::WHITE)
                                .size(18.0),
                        );
                        ui.separator();
                        let _has_0310 = view.mqtt_enabled || view.udp_0310_ok;
                        let udp_0310_ok = view.udp_0310_ok;
                        let mqtt_video_ok = view.mqtt_ok && view.mqtt_enabled;
                        let pv31_ok = mqtt_video_ok || udp_0310_ok;

                        ui.colored_label(
                            if pv31_ok {
                                Color32::from_rgb(54, 179, 126)
                            } else {
                                Color32::from_rgb(214, 162, 65)
                            },
                            format!(
                                "0310 Video: {}",
                                if pv31_ok { "connected" } else { "waiting" }
                            ),
                        );

                        let raw_ok = view.udp_ok;
                        ui.colored_label(
                            if raw_ok {
                                Color32::from_rgb(54, 179, 126)
                            } else {
                                Color32::from_rgb(130, 130, 130)
                            },
                            format!("UDP Raw: {}", if raw_ok { "connected" } else { "waiting" }),
                        );
                        ui.colored_label(
                            decode_color,
                            format!(
                                "Decode: {}",
                                if view.decode_ok {
                                    "running"
                                } else if view.ffmpeg_running {
                                    "waiting for IDR"
                                } else {
                                    "stopped"
                                }
                            ),
                        );
                        ui.colored_label(
                            mqtt_color,
                            format!(
                                "MQTT: {}",
                                if !view.mqtt_enabled {
                                    "disabled"
                                } else if view.mqtt_ok {
                                    "connected"
                                } else if view.mqtt_connected {
                                    "waiting payload"
                                } else {
                                    "reconnecting"
                                }
                            ),
                        );
                        ui.label(format!("UDP fallback bind: {}", self.config.bind_addr));
                        if view.mqtt_enabled {
                            ui.label(format!(
                                "MQTT endpoint: {}:{} / {} / client={} ",
                                self.config.mqtt_host,
                                self.config.mqtt_port,
                                self.config.mqtt_topic,
                                self.config.mqtt_client_id
                            ));
                        }
                        ui.label(format!("Source: {}", view.last_source));
                        ui.label(format!("Last frame id: {}", view.last_frame_id));
                        ui.label(format!("Packets: {}", view.packets_received));
                        ui.label(format!(
                            "Bytes: {:.2} MiB",
                            view.bytes_received as f64 / (1024.0 * 1024.0)
                        ));
                        ui.label(format!("Assembled frames: {}", view.assembled_frames));
                        ui.label(format!("Dropped packets: {}", view.dropped_packets));
                        ui.label(format!("Dropped frames: {}", view.dropped_frames));
                        ui.label(format!("Buffered frames: {}", view.buffered_frames));
                        ui.label(format!("Decoded frames: {}", view.decoded_frames));
                        ui.label(format!("MQTT messages: {}", view.mqtt_messages_received));
                        ui.label(format!(
                            "0x0310 UDP: {} pkts assembled={} seq={} age={}",
                            view.udp_0310_packets,
                            view.udp_0310_assembled,
                            view.udp_0310_last_seq,
                            view.udp_0310_age_ms
                                .map(|age| format!("{} ms", age))
                                .unwrap_or_else(|| "-".to_string())
                        ));
                        ui.label(format!("Display FPS: {:.1}", self.display_fps));
                        ui.label(format!(
                            "Last packet age: {}",
                            view.last_packet_age_ms
                                .map(|age| format!("{} ms", age))
                                .unwrap_or_else(|| "-".to_string())
                        ));
                        ui.label(format!(
                            "Last decode age: {}",
                            view.last_decode_age_ms
                                .map(|age| format!("{} ms", age))
                                .unwrap_or_else(|| "-".to_string())
                        ));
                        ui.label(format!(
                            "Last MQTT age: {}",
                            view.last_mqtt_age_ms
                                .map(|age| format!("{} ms", age))
                                .unwrap_or_else(|| "-".to_string())
                        ));

                        if let Some(telemetry) = &view.telemetry {
                            ui.separator();
                            ui.label(
                                RichText::new("Vehicle Telemetry")
                                    .strong()
                                    .color(Color32::WHITE),
                            );
                            ui.label(format!(
                                "camera={} gimbal={} mode={}",
                                if telemetry.camera_online() {
                                    "online"
                                } else {
                                    "offline"
                                },
                                if telemetry.gimbal_online() {
                                    "online"
                                } else {
                                    "offline"
                                },
                                telemetry.gimbal_mode_str()
                            ));
                            ui.label(format!(
                                "frame={} {}x{} telemetry_fps={:.1}",
                                telemetry.frame_seq,
                                telemetry.image_width,
                                telemetry.image_height,
                                telemetry.fps()
                            ));
                            ui.label(format!(
                                "yaw={:.2} pitch={:.2} yaw_vel={:.2} pitch_vel={:.2}",
                                telemetry.yaw,
                                telemetry.pitch,
                                telemetry.yaw_vel,
                                telemetry.pitch_vel
                            ));
                            ui.label(format!(
                                "bullet_speed={:.2} bullet_count={} exposure={}us gain={:.2}",
                                telemetry.bullet_speed,
                                telemetry.bullet_count,
                                telemetry.exposure_us,
                                telemetry.gain()
                            ));
                            if !telemetry.status_text.is_empty() {
                                ui.label(format!("status: {}", telemetry.status_text));
                            }
                        }

                        if let Some(error) = &view.last_error {
                            ui.separator();
                            ui.label(RichText::new(error).color(Color32::from_rgb(255, 188, 188)));
                        }
                        if let Some(error) = &view.mqtt_error {
                            ui.label(RichText::new(error).color(Color32::from_rgb(255, 210, 170)));
                        }
                    });
            });

        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

fn run_headless(config: AppConfig, shared: Arc<Mutex<StatusSnapshot>>) -> Result<()> {
    let seconds = config.headless_seconds.unwrap_or(5);
    let deadline = Instant::now() + Duration::from_secs(seconds);

    while Instant::now() < deadline {
        thread::sleep(Duration::from_millis(500));
        let view = snapshot_view(&shared, config.healthy_gap);
        println!(
            "[headless] udp_ok={} decode_ok={} mqtt_ok={} packets={} assembled={} decoded={} mqtt_msgs={} last_source={} last_error={} mqtt_error={}",
            view.udp_ok,
            view.decode_ok,
            view.mqtt_ok,
            view.packets_received,
            view.assembled_frames,
            view.decoded_frames,
            view.mqtt_messages_received,
            view.last_source,
            view.last_error.unwrap_or_else(|| "-".to_string()),
            view.mqtt_error.unwrap_or_else(|| "-".to_string())
        );
    }

    let view = snapshot_view(&shared, config.healthy_gap);
    if view.decoded_frames == 0 {
        bail!("headless 烟测失败：未解码到任何视频帧");
    }

    Ok(())
}

fn main() -> Result<()> {
    let config = AppConfig::from_args()?;
    let shared = Arc::new(Mutex::new(StatusSnapshot::default()));
    let (hevc_tx, hevc_rx) = bounded::<Vec<u8>>(2048);
    let (frame_tx, frame_rx) = bounded::<DecodedFrame>(2);

    spawn_udp_receiver(config.clone(), shared.clone(), hevc_tx.clone())?;
    spawn_udp_0310_receiver(config.clone(), shared.clone(), hevc_tx.clone())?;
    spawn_metadata_receiver(config.clone(), shared.clone(), hevc_tx)?;
    spawn_decoder(
        config.clone(),
        shared.clone(),
        hevc_rx,
        if config.headless_seconds.is_some() {
            None
        } else {
            Some(frame_tx)
        },
    )?;

    if config.headless_seconds.is_some() {
        return run_headless(config, shared);
    }

    let title = config.window_title.clone();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(title.clone())
            .with_inner_size([
                (config.output_width + 40) as f32,
                (config.output_height + 40) as f32,
            ])
            .with_min_inner_size([800.0, 480.0])
            .with_maximized(true)
            .with_always_on_top(),
        ..Default::default()
    };

    let app_config = config.clone();
    let app_shared = shared.clone();
    eframe::run_native(
        &title,
        native_options,
        Box::new(move |_cc| {
            Ok(Box::new(ViewerApp::new(
                app_config.clone(),
                app_shared.clone(),
                frame_rx.clone(),
            )))
        }),
    )
    .map_err(|error| anyhow::anyhow!("原生窗口启动失败: {error}"))?;

    Ok(())
}
