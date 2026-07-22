"""
本地测试启动文件：海康相机 + 编码 + 解码。
启动后可以看到编码端（4个窗口）和解码端（1个窗口）的实时画面。
"""
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    debug_dump_dir = '/tmp/sniper_debug_imgs'
    debug_dump_enable = False

    # 编码参数（海康相机 1440×1080 → 中心裁剪 800 → 编码 300×300）
    encode_size = 300
    crop_size = 800
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
                    'device_index': 0,       # 本地测试用第一个相机
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
                    'enable_display': True,
                    'debug_dump_enable': debug_dump_enable,
                    'debug_dump_dir': debug_dump_dir,
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

    # 解码端节点
    decoder_node = Node(
        package='doorlock_decoder',
        executable='decoder_node',
        name='video_decoder',
        parameters=[{
            'topic': '/video_stream',
            'display': True,
            'width': encode_size,
            'height': encode_size,
            'display_scale': 2,
            'crosshair_offset_x': 0,
            'crosshair_offset_y': 0,
            'crosshair_width': 1,
            'debug_dump_enable': debug_dump_enable,
            'debug_dump_save_decoder': True,
            'debug_dump_dir': debug_dump_dir,
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        encoder_container,
        decoder_node,
    ])
