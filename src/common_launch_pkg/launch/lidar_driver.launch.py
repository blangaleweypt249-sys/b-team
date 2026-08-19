"""雷达驱动模块启动文件。

启动内容：
  1. livox_ros_driver2_node  — MID360 雷达驱动，发布 /livox/lidar (PointCloud2) 和 /livox/imu
  2. fastlio_mapping         — FAST-LIO 里程计，订阅 /livox/lidar + /livox/imu，发布 /Odometry

启动顺序：
  阶段 0s : livox 雷达驱动（等待设备连接并发布点云）
  阶段 3s : FAST-LIO 里程计（延迟启动以等待雷达数据就绪）

错误处理：
  livox 驱动进程异常退出时输出错误日志并触发系统关闭。
  FAST-LIO 进程异常退出时输出错误日志（不强制关闭，感知节点仍可调试）。

独立启动命令：
  ros2 launch common_launch_pkg lidar_driver.launch.py

可选参数：
  xfer_format:=0  雷达点云格式 0=PointCloud2, 1=CustomMsg（默认 0，兼容球检测节点）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xfer_format = LaunchConfiguration('xfer_format')

    declare_xfer_format_cmd = DeclareLaunchArgument(
        'xfer_format', default_value='0',
        description='点云格式: 0=PointCloud2(兼容球检测), 1=CustomMsg(FAST-LIO原生)'
    )

    # ---- MID360 雷达驱动 ----
    mid360_config = PathJoinSubstitution([
        FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json',
    ])

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': xfer_format,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': mid360_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    # ---- FAST-LIO 里程计（延迟 3 秒启动，等待雷达数据就绪）----
    fastlio_config = PathJoinSubstitution([
        FindPackageShare('fast_lio'), 'config', 'mid360.yaml',
    ])

    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fastlio_config, {'use_sim_time': False}],
    )

    # ---- 错误处理：livox 驱动退出时关闭整个系统 ----
    livox_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=livox_driver,
            on_exit=[
                LogInfo(msg='[错误] MID360 雷达驱动已退出，正在关闭系统...'),
                EmitEvent(event=Shutdown(reason='livox_driver_exited')),
            ],
        )
    )

    fastlio_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=fastlio_node,
            on_exit=[
                LogInfo(msg='[警告] FAST-LIO 里程计节点已退出，感知节点将无法获取定位数据'),
            ],
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_xfer_format_cmd)

    # 阶段 1: 立即启动雷达驱动
    ld.add_action(LogInfo(msg='[雷达驱动] 正在启动 MID360 雷达驱动...'))
    ld.add_action(livox_driver)
    ld.add_action(livox_exit_handler)

    # 阶段 2: 延迟 3 秒启动 FAST-LIO
    ld.add_action(TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[雷达驱动] 正在启动 FAST-LIO 里程计...'),
            fastlio_node,
            fastlio_exit_handler,
            LogInfo(msg='[雷达驱动] FAST-LIO 已启动，等待里程计数据 /Odometry'),
        ],
    ))

    return ld
