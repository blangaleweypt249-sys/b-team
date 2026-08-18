"""通用感知节点模块启动文件(一车代码,红方只切串口yaml,X取反;蓝方X原样)。

内部坐标系(field_pose、区域判定、感知块/球)完全不变,统一使用 field.yaml,
只有串口网关按队伍加载不同 yaml:
  team:=blue → serial_gateway.yaml     (negate_x=false, X原样发送)
  team:=red  → serial_gateway_red.yaml (negate_x=true,  只串口X取反)

启动内容：
  1. fyt_pos (field_localizer + field_visualizer)
     — 配置: field.yaml (全局赛场坐标,两车统一,内部坐标系不变)
     — 依赖 /Odometry,发布 /competition/field_pose, /competition/current_zone
  2. ball_perception (usb_camera_node + ball_distance_node)
     — 发布 /perception/ball_position, /perception/ball_overlay
  3. block_perception (orbbec_camera_node + block_distance_node)
     — 只在 block/special1/special2 区域内检测块
     — 发布 /perception/block_position, /perception/block_overlay
  4. serial_gateway
     — 按 team 参数加载 serial_gateway.yaml 或 serial_gateway_red.yaml

启动顺序：
  阶段 0s : USB相机(球) + Orbbec相机(块)
  阶段 2s : 球检测融合 + 块检测融合
  阶段 3s : 场地定位节点
  阶段 4s : 串口网关

独立启动命令(先启动雷达):
  ros2 launch common_launch_pkg lidar_driver.launch.py
  ros2 launch common_launch_pkg perception_nodes.launch.py          # 蓝方,X原样
  ros2 launch common_launch_pkg perception_nodes.launch.py team:=red  # 红方,串口X取反
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    team = LaunchConfiguration('team')
    declare_team_cmd = DeclareLaunchArgument(
        'team', default_value='blue',
        description='队伍 blue/red:只切换串口网关yaml,内部坐标系不变'
    )

    # 定位配置:两车统一使用 field.yaml (内部坐标系不变)
    fyt_pos_config = PathJoinSubstitution([
        FindPackageShare('fyt_pos'), 'config', 'field.yaml',
    ])
    # 串口网关配置:两套独立 yaml
    gateway_config = PythonExpression([
        "'", FindPackageShare('competition_gateway'), "/config/",
        "('serial_gateway_red.yaml' if '", team, "' == 'red' else 'serial_gateway.yaml')",
        "'"
    ])
    # 串口网关可执行文件:两套独立 cpp 源,蓝方 serial_gateway,红方 serial_gateway_red
    gateway_exec = PythonExpression([
        "'('serial_gateway_red' if '", team, "' == 'red' else 'serial_gateway')'"
    ])
    gateway_node_name = PythonExpression([
        "'('serial_gateway_red' if '", team, "' == 'red' else 'serial_gateway')'"
    ])

    # ---- fyt_pos: 场地定位节点 ----
    field_localizer = Node(
        package='fyt_pos',
        executable='field_localizer',
        name='field_localizer',
        output='screen',
        parameters=[fyt_pos_config],
    )
    field_visualizer = Node(
        package='fyt_pos',
        executable='field_visualizer',
        name='field_visualizer',
        output='screen',
        parameters=[fyt_pos_config],
    )

    # ---- ball_perception: 金球检测 (一套,红蓝共用) ----
    usb_camera_node = Node(
        package='ball_perception',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[{
            'device_id': 2, 'width': 640, 'height': 480, 'fps': 30,
        }],
    )
    ball_model_path = PathJoinSubstitution([
        FindPackageShare('ball_perception'), 'models', 'best.pt',
    ])
    ball_distance_node = Node(
        package='ball_perception',
        executable='ball_distance_node',
        name='ball_distance_node',
        output='screen',
        parameters=[{
            'model_path': ball_model_path,
            'conf_threshold': 0.5,
            'extrinsic_x': -0.035,
            'extrinsic_y': -0.173,
            'extrinsic_z': 0.0,
            'extrinsic_roll': 0.0,
            'extrinsic_pitch': 0.0,
            'extrinsic_yaw': -1.5707963267948966,  # -90°
            'output_to_gripper': True,
            'lidar_to_gripper_x': -0.174,
            'lidar_to_gripper_y': 0.173,
            'lidar_to_gripper_yaw_deg': -90.0,
            'min_distance': 0.1,
            'max_distance': 10.0,
            'distance_alpha': 0.7,
        }],
    )

    # ---- block_perception: 块检测 (一套,红蓝共用) ----
    orbbec_camera_node = Node(
        package='block_perception',
        executable='orbbec_camera_node',
        name='orbbec_camera_node',
        output='screen',
        parameters=[{
            'color_width': 1280, 'color_height': 720,
            'depth_width': 0, 'depth_height': 0,
            'min_depth_mm': 20.0, 'max_depth_mm': 10000.0,
        }],
    )
    block_model_path = PathJoinSubstitution([
        FindPackageShare('block_perception'), 'models', 'best.pt',
    ])
    block_distance_node = Node(
        package='block_perception',
        executable='block_distance_node',
        name='block_distance_node',
        output='screen',
        parameters=[{
            'model_path': block_model_path,
            'conf_threshold': 0.75,
            'iou_threshold': 0.45,
            'distance_alpha': 0.7,
            'output_to_gripper': True,
            't_g_c_x_m': -0.102,
            't_g_c_y_m': -0.0525,
            't_g_c_z_m': 0.028,
            'active_zones': ['block', 'special1', 'special2'],
        }],
    )

    # ---- 串口网关:按 team 选两套独立可执行文件(cpp写死X取反差异) ----
    serial_gateway_node = Node(
        package='competition_gateway',
        executable=gateway_exec,
        name=gateway_node_name,
        output='screen',
        parameters=[gateway_config],
    )

    # ---- 错误处理 ----
    def make_exit_handler(node_name, impact):
        return RegisterEventHandler(
            event_handler=OnProcessExit(
                on_exit=[LogInfo(msg=f'[警告] {node_name} 已退出。影响：{impact}')],
            )
        )

    ld = LaunchDescription()
    ld.add_action(declare_team_cmd)

    ld.add_action(LogInfo(msg=[
        '[感知-启动] 队伍: ', team,
        ' (定位/相机/检测一套代码,仅串口网关yaml切换negate_x)'
    ]))

    # 阶段 1: 相机
    ld.add_action(LogInfo(msg='[感知] 启动 USB 相机(金球检测)...'))
    ld.add_action(usb_camera_node)
    ld.add_action(make_exit_handler('usb_camera_node', '球检测无图像'))
    ld.add_action(LogInfo(msg='[感知] 启动 Orbbec RGB-D 相机(块检测)...'))
    ld.add_action(orbbec_camera_node)
    ld.add_action(make_exit_handler('orbbec_camera_node', '块检测无图像'))

    # 阶段 2: 检测融合
    ld.add_action(TimerAction(period=2.0, actions=[
        LogInfo(msg='[感知] 启动金球检测+融合测距...'),
        ball_distance_node,
        make_exit_handler('ball_distance_node', '/perception/ball_position 停止发布'),
        LogInfo(msg='[感知] 启动块检测+测距 (仅block/special1/special2区)...'),
        block_distance_node,
        make_exit_handler('block_distance_node', '/perception/block_position 停止发布'),
    ]))

    # 阶段 3: 场地定位 (统一 field.yaml)
    ld.add_action(TimerAction(period=3.0, actions=[
        LogInfo(msg='[感知] 启动场地定位(统一field.yaml,内部坐标系不变)...'),
        field_localizer,
        field_visualizer,
        make_exit_handler('field_localizer', '/competition/field_pose 停止发布'),
        make_exit_handler('field_visualizer', 'RViz场地可视化停止'),
    ]))

    # 阶段 4: 串口网关(按队伍切 yaml)
    ld.add_action(TimerAction(period=4.0, actions=[
        LogInfo(msg=[
            '[感知] 启动串口网关 (team=', team,
            ', 仅当red时串口X取反,内部坐标系不变)...'
        ]),
        serial_gateway_node,
        make_exit_handler('serial_gateway', 'STM32通信中断'),
        LogInfo(msg='[感知] 所有节点启动完毕'),
    ]))

    return ld
