"""红方串口网关启动文件(negate_x=true,发送前field_x_m取反)。

内部坐标系(field_pose话题、区域判定、感知块/球坐标系)完全不变,
仍使用全局赛场坐标,只在串口发送时对位置帧field_x_m取反。

启动内容：
  competition_gateway / serial_gateway
    — 独占 /dev/ttyACM0 串口
    — 订阅感知话题，封装为固定帧格式发送给 STM32（50Hz）
    — 接收 STM32 状态帧并发布连接状态
    — negate_x=true: 位置帧 field_x_m 取反后再打包发送给 STM32

订阅话题：
  /competition/field_pose           (geometry_msgs/PoseStamped)   — 机器人位姿
  /perception/block_position        (geometry_msgs/PointStamped)  — 当前跟踪块 3D 位置

发布话题：
  /competition/serial/controller_connected  (std_msgs/Bool)   — STM32 连接状态
  /competition/serial/controller_state      (std_msgs/UInt8)  — STM32 状态机状态
  /competition/serial/controller_error      (std_msgs/UInt8)  — STM32 错误码

蓝方：请使用 serial_gateway.launch.py（negate_x=false，X原样发送）。

独立启动命令：
  ros2 launch common_launch_pkg serial_gateway_red.launch.py

可选参数：
  serial_device:=/dev/ttyACM0  指定串口设备路径
  baudrate:=921600             指定波特率
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
        'serial_device', default_value='/dev/ttyACM0',
        description='串口设备路径'
    )
    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate', default_value='921600',
        description='串口波特率'
    )

    gateway_config = PathJoinSubstitution([
        FindPackageShare('competition_gateway'), 'config', 'serial_gateway_red.yaml',
    ])

    # 红方:用 serial_gateway_red 可执行文件(cpp 内写死 field_x_m 取反,内部坐标系不变)
    serial_gateway_node = Node(
        package='competition_gateway',
        executable='serial_gateway_red',
        name='serial_gateway_red',
        output='screen',
        parameters=[
            gateway_config,
            {
                'serial_device': serial_device,
                'baudrate': baudrate,
            },
        ],
    )

    gateway_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=serial_gateway_node,
            on_exit=[
                LogInfo(msg='[错误] 红方串口网关已退出，STM32 通信中断，正在关闭系统...'),
                EmitEvent(event=Shutdown(reason='serial_gateway_exited')),
            ],
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_serial_cmd)
    ld.add_action(declare_baudrate_cmd)
    ld.add_action(LogInfo(msg='[信息发布-红方] 正在启动串口网关 (negate_x=是,field_x_m取反发送)...'))
    ld.add_action(LogInfo(msg=[
        '[信息发布-红方] 串口设备: ', serial_device, '  波特率: ', baudrate,
    ]))
    ld.add_action(serial_gateway_node)
    ld.add_action(gateway_exit_handler)
    ld.add_action(LogInfo(msg='[信息发布-红方] 串口网关已启动，等待感知数据...'))

    return ld
