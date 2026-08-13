"""系统集成启动文件 — 一键启动所有系统功能模块。

启动内容（按阶段顺序）：
  阶段 1 (0s) : 雷达驱动模块
    — livox_ros_driver2 (MID360 雷达驱动, /livox/lidar + /livox/imu)
    — fastlio_mapping    (FAST-LIO 里程计, /Odometry)  [延迟 3s 内部启动]

  阶段 2 (6s) : 感知节点模块
    — usb_camera_node        (USB 相机, /camera/image_raw)
    — orbbec_camera_node     (Orbbec RGB-D, /orbbec/color + /orbbec/depth)
    — ball_distance_node     (金球检测+融合测距, /perception/ball_position)  [延迟 2s]
    — block_distance_node    (红蓝块检测+测距, /perception/block_red/blue_position)  [延迟 2s]
    — field_localizer        (场地定位, /competition/field_pose)  [延迟 3s]
    — field_visualizer       (RViz 场地可视化)  [延迟 3s]

  阶段 3 (12s): 信息发布模块
    — serial_gateway (串口网关, /dev/ttyUSB0 → STM32, 50Hz)

通信连接建立顺序：
  /livox/lidar  ──→  fastlio_mapping  ──→  /Odometry  ──→  field_localizer  ──→  /competition/field_pose
       │                                                                                    │
       └──→  ball_distance_node  ──→  /perception/ball_position  ─────────────────────────┤
                                                                                            ↓
  orbbec_camera  ──→  block_distance_node  ──→  /perception/block_red/blue_position  ──→  serial_gateway  ──→  STM32

错误处理：
  - livox 驱动退出 → 关闭整个系统（所有模块依赖雷达数据）
  - 串口网关退出 → 关闭整个系统（STM32 通信中断）
  - 感知节点退出 → 输出警告日志，不影响其他模块运行

启动命令：
  ros2 launch common_launch_pkg common_launch.py

可选参数：
  rviz:=true              启动 RViz 可视化（默认 false）
  xfer_format:=0          雷达点云格式 0=PointCloud2, 1=CustomMsg（默认 0）
  serial_device:=/dev/ttyUSB0  串口设备路径
  baudrate:=115200        串口波特率

独立模块启动命令：
  ros2 launch common_launch_pkg lidar_driver.launch.py       # 仅雷达驱动+里程计
  ros2 launch common_launch_pkg perception_nodes.launch.py   # 仅感知节点
  ros2 launch common_launch_pkg serial_gateway.launch.py     # 仅串口网关
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_use = LaunchConfiguration('rviz')
    xfer_format = LaunchConfiguration('xfer_format')
    serial_device = LaunchConfiguration('serial_device')
    baudrate = LaunchConfiguration('baudrate')
    team = LaunchConfiguration('team')

    # ---- 声明启动参数 ----
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='启动 RViz 可视化'
    )
    declare_xfer_format_cmd = DeclareLaunchArgument(
        'xfer_format', default_value='0',
        description='点云格式: 0=PointCloud2, 1=CustomMsg'
    )
    declare_serial_cmd = DeclareLaunchArgument(
        'serial_device', default_value='/dev/ttyUSB0',
        description='串口设备路径'
    )
    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='串口波特率'
    )
    declare_team_cmd = DeclareLaunchArgument(
        'team', default_value='blue',
        description='队伍选择: blue=蓝方(bottom_left) red=红方(bottom_right)'
    )

    # ---- 模块 launch 文件路径 ----
    common_launch_dir = PathJoinSubstitution([
        FindPackageShare('common_launch_pkg'), 'launch',
    ])

    lidar_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([common_launch_dir, 'lidar_driver.launch.py'])
    )
    perception_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([common_launch_dir, 'perception_nodes.launch.py'])
    )
    gateway_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([common_launch_dir, 'serial_gateway.launch.py'])
    )

    # ---- 阶段 1: 雷达驱动模块（立即启动）----
    stage1_lidar = IncludeLaunchDescription(
        lidar_launch,
        launch_arguments={
            'rviz': rviz_use,
            'xfer_format': xfer_format,
        }.items(),
    )

    # ---- 阶段 2: 感知节点模块（延迟 6 秒，等待 FAST-LIO 里程计就绪）----
    stage2_perception = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg='========== [集成启动] 阶段 2/3: 启动感知节点模块 =========='),
            IncludeLaunchDescription(
                perception_launch,
                launch_arguments={'team': team}.items(),
            ),
        ],
    )

    # ---- 阶段 3: 信息发布模块（延迟 12 秒，等待感知数据就绪）----
    stage3_gateway = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='========== [集成启动] 阶段 3/3: 启动串口网关模块 =========='),
            IncludeLaunchDescription(
                gateway_launch,
                launch_arguments={
                    'serial_device': serial_device,
                    'baudrate': baudrate,
                }.items(),
            ),
            LogInfo(msg='========== [集成启动] 所有模块已启动 =========='),
        ],
    )

    ld = LaunchDescription()

    # 声明参数
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_xfer_format_cmd)
    ld.add_action(declare_serial_cmd)
    ld.add_action(declare_baudrate_cmd)
    ld.add_action(declare_team_cmd)

    # 启动概要日志
    ld.add_action(LogInfo(msg=''))
    ld.add_action(LogInfo(msg='============================================================'))
    ld.add_action(LogInfo(msg='  系统集成启动 — 比赛感知与串口通信系统'))
    ld.add_action(LogInfo(msg='============================================================'))
    ld.add_action(LogInfo(msg='  阶段 1 (0s) :  雷达驱动 + FAST-LIO 里程计'))
    ld.add_action(LogInfo(msg='  阶段 2 (6s) :  感知节点 (定位 + 球检测 + 红蓝块检测)'))
    ld.add_action(LogInfo(msg='  阶段 3 (12s):  串口网关 (信息发布 → STM32)'))
    ld.add_action(LogInfo(msg='============================================================'))
    ld.add_action(LogInfo(msg=''))

    # 按阶段顺序启动
    ld.add_action(LogInfo(msg='========== [集成启动] 阶段 1/3: 启动雷达驱动模块 =========='))
    ld.add_action(stage1_lidar)
    ld.add_action(stage2_perception)
    ld.add_action(stage3_gateway)

    return ld
