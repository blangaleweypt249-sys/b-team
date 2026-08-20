"""系统集成启动文件 — 一键启动所有系统功能模块。

启动内容（按阶段顺序）：
  阶段 1 (0s) : 雷达驱动模块
    — livox_ros_driver2 (MID360 雷达驱动, /livox/lidar + /livox/imu)
    — fastlio_mapping    (FAST-LIO 里程计, /Odometry)  [延迟 3s 内部启动]

  阶段 2 (6s) : 感知节点模块
    — orbbec_camera_node     (Orbbec RGB-D,块朝向正前方, /orbbec/color + /orbbec/depth)
    — block_distance_node    (块IOU跟踪+单块输出(不分色), /perception/block_position, 仅block/special1/special2区)  [延迟 2s]
    — field_localizer        (场地定位, /competition/field_pose + /competition/current_zone, 统一使用field.yaml, 内部坐标系不变)  [延迟 3s]

  阶段 3 (12s): 信息发布模块（串口网关,两套独立可执行文件）
    — team=blue: serial_gateway       (蓝方, /dev/ttyACM0 → STM32, X 原样发送, 50Hz/类 100Hz)
    — team=red : serial_gateway_red   (红方, 写死位置帧 field_x_m 取反, 感知块/球坐标不变)

通信连接建立顺序：
  /livox/lidar  ──→  fastlio_mapping  ──→  /Odometry  ──→  field_localizer  ──→  /competition/field_pose ──┐
                                                                                                             │
  orbbec_cam  ──→ block_distance_node ──→  /perception/block_position (单块,IOU确认,只在块/特殊区)  ────────────┤──→  serial_gateway(_red)  ──→  STM32

错误处理：
  - livox 驱动退出 → 关闭整个系统（所有模块依赖雷达数据）
  - 串口网关退出 → 关闭整个系统（STM32 通信中断）
  - 感知节点退出 → 输出警告日志，不影响其他模块运行

启动命令：
  ros2 launch common_launch_pkg common_launch.py                  # 蓝方
  ros2 launch common_launch_pkg common_launch.py team:=red        # 红方 (串口层X取反,内部不变)

可选参数：
  xfer_format:=0                雷达点云格式 0=PointCloud2, 1=CustomMsg（默认 0）
  serial_device:=/dev/ttyACM0   串口设备路径
  baudrate:=921600              串口波特率 (9600/57600/115200/460800/921600)
  team:=blue|red                队伍: blue=串口X原样; red=串口位置帧X取反(两套独立cpp executable)

独立模块启动命令：
  ros2 launch common_launch_pkg lidar_driver.launch.py                    # 雷达驱动+里程计
  ros2 launch common_launch_pkg perception_nodes.launch.py                # 感知节点
  ros2 launch common_launch_pkg serial_gateway.launch.py                  # 仅蓝方串口网关
  ros2 launch common_launch_pkg serial_gateway_red.launch.py              # 仅红方串口网关(写死X取反)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xfer_format = LaunchConfiguration('xfer_format')
    serial_device = LaunchConfiguration('serial_device')
    baudrate = LaunchConfiguration('baudrate')
    team = LaunchConfiguration('team')

    # ---- 声明启动参数 ----
    declare_xfer_format_cmd = DeclareLaunchArgument(
        'xfer_format', default_value='0',
        description='点云格式: 0=PointCloud2, 1=CustomMsg'
    )
    declare_serial_cmd = DeclareLaunchArgument(
        'serial_device', default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate', default_value='921600',
        description='串口波特率 (9600/57600/115200/460800/921600)'
    )
    declare_team_cmd = DeclareLaunchArgument(
        'team', default_value='blue',
        description='队伍: blue=蓝方 serial_gateway(X原样); red=红方 serial_gateway_red(位置帧X写死取反,内部坐标不变)'
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
    # 按队伍选择对应的串口网关launch文件(两套独立cpp,不用运行时切参数)
    gateway_launch_file = PythonExpression([
        "'", common_launch_dir, "/serial_gateway_red.launch.py' if '",
        team, "' == 'red' else '",
        common_launch_dir, "/serial_gateway.launch.py'"
    ])
    gateway_launch = PythonLaunchDescriptionSource(gateway_launch_file)

    # ---- 阶段 1: 雷达驱动模块（立即启动）----
    stage1_lidar = IncludeLaunchDescription(
        lidar_launch,
        launch_arguments={
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
            ),
        ],
    )

    # ---- 阶段 3: 信息发布模块（延迟 12 秒，等待感知数据就绪）
    # team=blue 启动 serial_gateway      (X 原样发送)
    # team=red  启动 serial_gateway_red  (写死位置帧 field_x_m 取反, CRC正确)  ----
    stage3_gateway = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=['========== [集成启动] 阶段 3/3: 启动串口网关(team=', team, ') ==========']),
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
    ld.add_action(declare_xfer_format_cmd)
    ld.add_action(declare_serial_cmd)
    ld.add_action(declare_baudrate_cmd)
    ld.add_action(declare_team_cmd)

    # 启动概要日志
    ld.add_action(LogInfo(msg=''))
    ld.add_action(LogInfo(msg='============================================================'))
    ld.add_action(LogInfo(msg='  系统集成启动 — 比赛感知与串口通信系统'))
    ld.add_action(LogInfo(msg=['  队伍: ', team, ' (blue=串口X原样, red=串口X取反,内部坐标系不变)']))
    ld.add_action(LogInfo(msg='============================================================'))
    ld.add_action(LogInfo(msg='  阶段 1 (0s) :  雷达驱动 + FAST-LIO 里程计'))
    ld.add_action(LogInfo(msg='  阶段 2 (6s) :  感知节点 (统一field.yaml定位 + IOU单块跟踪(仅块/特殊区))'))
    ld.add_action(LogInfo(msg='  阶段 3 (12s):  串口网关 (信息发布 → STM32, blue/red 用两套独立C++ executable)'))
    ld.add_action(LogInfo(msg='============================================================'))
    ld.add_action(LogInfo(msg=''))

    # 按阶段顺序启动
    ld.add_action(LogInfo(msg='========== [集成启动] 阶段 1/3: 启动雷达驱动模块 =========='))
    ld.add_action(stage1_lidar)
    ld.add_action(stage2_perception)
    ld.add_action(stage3_gateway)

    return ld
