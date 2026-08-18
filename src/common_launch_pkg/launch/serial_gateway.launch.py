"""信息发布模块启动文件（串口网关）。

启动内容：
  competition_gateway / serial_gateway
    — 独占 /dev/ttyUSB0 串口
    — 订阅感知话题，封装为固定帧格式发送给 STM32（50Hz）
    — 接收 STM32 状态帧并发布连接状态

订阅话题：
  /competition/field_pose           (geometry_msgs/PoseStamped)   — 机器人位姿
  /perception/ball_position         (geometry_msgs/PointStamped)  — 金球 3D 位置
  /perception/block_position        (geometry_msgs/PointStamped)  — 当前跟踪块 3D 位置

发布话题：
  /competition/serial/controller_connected  (std_msgs/Bool)   — STM32 连接状态
  /competition/serial/controller_state      (std_msgs/UInt8)  — STM32 状态机状态
  /competition/serial/controller_error      (std_msgs/UInt8)  — STM32 错误码

错误处理：
  串口网关进程异常退出时输出错误日志并触发系统关闭。
  串口打开失败时节点内部会输出 RCLCPP_ERROR 但不会崩溃。

独立启动命令：
  ros2 launch common_launch_pkg serial_gateway.launch.py

可选参数：
  serial_device:=/dev/ttyUSB0  指定串口设备路径
  baudrate:=115200             指定波特率 (9600/57600/115200/460800/921600)

注意：
  独立启动时需确保感知节点已在运行并发布上述话题，
  否则串口网关将因数据超时而在发送帧中清除所有有效位。
  串口设备 /dev/ttyUSB0 必须存在且未被其他进程占用。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_device = LaunchConfiguration('serial_device')
    baudrate = LaunchConfiguration('baudrate')

    declare_serial_cmd = DeclareLaunchArgument(
        'serial_device', default_value='/dev/ttyUSB0',
        description='串口设备路径'
    )
    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate', default_value='115200',
        description='串口波特率'
    )

    gateway_config = PathJoinSubstitution([
        FindPackageShare('competition_gateway'), 'config', 'serial_gateway.yaml',
    ])

    serial_gateway_node = Node(
        package='competition_gateway',
        executable='serial_gateway',
        name='serial_gateway',
        output='screen',
        parameters=[
            gateway_config,
            {
                'serial_device': serial_device,
                'baudrate': baudrate,
            },
        ],
    )

    # ---- 错误处理：串口网关退出时关闭系统 ----
    gateway_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=serial_gateway_node,
            on_exit=[
                LogInfo(msg='[错误] 串口网关已退出，STM32 通信中断，正在关闭系统...'),
                EmitEvent(event=Shutdown(reason='serial_gateway_exited')),
            ],
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_serial_cmd)
    ld.add_action(declare_baudrate_cmd)

    ld.add_action(LogInfo(msg='[信息发布] 正在启动串口网关 (competition_gateway)...'))
    ld.add_action(LogInfo(msg=[
        '[信息发布] 串口设备: ', serial_device, '  波特率: ', baudrate,
    ]))
    ld.add_action(serial_gateway_node)
    ld.add_action(gateway_exit_handler)
    ld.add_action(LogInfo(msg='[信息发布] 串口网关已启动，等待感知数据...'))

    return ld
