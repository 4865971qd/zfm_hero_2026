"""
MiniPC 部署启动文件 — 海康相机 + H264编码 + 串口图传链路。

部署流程：
  海康CS016-10UC → hik_camera → video_encoder (H264) → serial_bridge → UART 0x0310 → VTX

接收端（笔记本）只需运行 rm-native-viewer，通过 MQTT CustomByteBlock 解码显示。
"""
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    encode_size = 300
    crop_size = 800                   # 海康更高分辨率，可设更大裁剪
    target_bitrate_kbytes = 10.0
    hard_max_bitrate_kbytes = 14.0
    target_bitrate_kbps = int(target_bitrate_kbytes * 8.0)

    # 编码端容器 — 相机和编码器同进程零拷贝
    encoder_container = ComposableNodeContainer(
        name='encoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[{
                    'device_index': 1,       # 图传专用相机（第二个 USB 口）
                    'exposure_time': 12000.0,
                    'gain': 10.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='doorlock_sniper',
                plugin='doorlock_sniper::VideoEncoderNode',
                name='video_encoder',
                parameters=[{
                    'input_topic': '/image_raw',
                    'target_bitrate': target_bitrate_kbps,
                    'x264_preset': 'veryslow',
                    'output_fps': 60,
                    'packet_size': 150,
                    'enable_display': False,        # miniPC 无显示器，关闭
                    'debug_dump_enable': False,
                    'crop_size': crop_size,
                    'output_size': encode_size,
                    'static_simplify': True,
                    'motion_threshold': 14,
                    'motion_erode_px': 2,
                    'motion_dilate_px': 6,
                    'motion_trail_frames': 90,
                    'trail_disable_motion_ratio': 0.30,
                    'bg_update_alpha': 0.01,
                    'bg_blur_sigma': 1.8,
                    'center_clear_size': 150,
                    'force_monochrome': False,
                    'bandwidth_limit_kbytes': hard_max_bitrate_kbytes,
                    'bandwidth_window_s': 2.0,
                    'max_tx_delay_s': 1.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # 串口桥接 — VideoPacket → 0x0310 串口帧
    serial_bridge = Node(
        package='serial_bridge',
        executable='serial_bridge_node',
        name='serial_bridge',
        parameters=[{
            'serial_port': 'auto',           # 自动扫描 /dev/serial/by-id/*, ttyUSB*, ttyACM*
            'baudrate': 921600,
            'send_hz': 48,
            'max_payload': 297,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        encoder_container,
        serial_bridge,
    ])
