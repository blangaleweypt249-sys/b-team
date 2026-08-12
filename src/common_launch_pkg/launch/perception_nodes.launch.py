"""感知节点模块启动文件。

启动内容：
  1. fyt_pos (field_localizer + field_visualizer)
     — 依赖 /Odometry，发布 /competition/field_pose, /competition/field_markers
  2. ball_perception (usb_camera_node + ball_distance_node)
     — 依赖 /camera/image_raw (自带), /livox/lidar (需雷达驱动)
     — 发布 /perception/ball_position, /perception/ball_overlay
  3. block_perception (orbbec_camera_node + block_distance_node)
     — 依赖 /orbbec/color/image_raw, /orbbec/depth/image_raw (自带 Orbbec 相机)
     — 发布 /perception/block_red_position, /perception/block_blue_position

启动顺序：
  阶段 0s : 球感知相机 + 红蓝块相机（图像采集节点，无外部依赖）
  阶段 2s : 球检测融合 + 红蓝块检测融合（等待相机图像就绪）
  阶段 3s : 场地定位节点（等待 FAST-LIO 里程计 /Odometry）

错误处理：
  各感知节点异常退出时输出警告日志，不影响其他节点运行。

独立启动命令：
  ros2 launch common_launch_pkg perception_nodes.launch.py

注意：
  独立启动时需确保雷达驱动已运行（/livox/lidar）且 FAST-LIO 已发布 /Odometry，
  否则球检测测距和场地定位功能将无法正常工作。
"""

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 配置文件路径 ----
    fyt_pos_config = PathJoinSubstitution([
        FindPackageShare('fyt_pos'), 'config', 'field.yaml',
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

    # ---- ball_perception: 金球检测 ----
    usb_camera_node = Node(
        package='ball_perception',
        executable='usb_camera_node',
        name='usb_camera_node',
        output='screen',
        parameters=[{
            'device_id': 2,
            'width': 640,
            'height': 480,
            'fps': 30,
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
            'extrinsic_x': 0.0,
            'extrinsic_y': -0.198,
            'extrinsic_z': 0.0,
            'extrinsic_roll': 0.0,
            'extrinsic_pitch': 0.0,
            'extrinsic_yaw': 0.0,
            'min_distance': 0.1,
            'max_distance': 10.0,
            'distance_alpha': 0.7,
        }],
    )

    # ---- block_perception: 红蓝块检测 ----
    orbbec_camera_node = Node(
        package='block_perception',
        executable='orbbec_camera_node',
        name='orbbec_camera_node',
        output='screen',
        parameters=[{
            'color_width': 1280,
            'color_height': 720,
            'depth_width': 0,
            'depth_height': 0,
            'min_depth_mm': 20.0,
            'max_depth_mm': 10000.0,
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
            'red_class_name': 'red',
            'blue_class_name': 'blue',
            'distance_alpha': 0.7,
        }],
    )

    # ---- 错误处理：节点退出时输出警告 ----
    def make_exit_handler(node_name, impact):
        return RegisterEventHandler(
            event_handler=OnProcessExit(
                on_exit=[
                    LogInfo(msg=f'[警告] {node_name} 已退出。影响：{impact}'),
                ],
            )
        )

    ld = LaunchDescription()

    # 阶段 1: 启动相机节点（无外部依赖）
    ld.add_action(LogInfo(msg='[感知节点] 正在启动 USB 相机 (金球检测)...'))
    ld.add_action(usb_camera_node)
    ld.add_action(make_exit_handler('usb_camera_node', '球检测将无法获取图像'))

    ld.add_action(LogInfo(msg='[感知节点] 正在启动 Orbbec RGB-D 相机 (红蓝块检测)...'))
    ld.add_action(orbbec_camera_node)
    ld.add_action(make_exit_handler('orbbec_camera_node', '红蓝块检测将无法获取图像'))

    # 阶段 2: 延迟 2 秒启动检测融合节点（等待相机图像就绪）
    ld.add_action(TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[感知节点] 正在启动金球检测+融合测距节点...'),
            ball_distance_node,
            make_exit_handler('ball_distance_node', '/perception/ball_position 将停止发布'),

            LogInfo(msg='[感知节点] 正在启动红蓝块检测+测距节点...'),
            block_distance_node,
            make_exit_handler('block_distance_node', '/perception/block_red/blue_position 将停止发布'),
        ],
    ))

    # 阶段 3: 延迟 3 秒启动场地定位节点（等待 /Odometry）
    ld.add_action(TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[感知节点] 正在启动场地定位节点 (依赖 /Odometry)...'),
            field_localizer,
            field_visualizer,
            make_exit_handler('field_localizer', '/competition/field_pose 将停止发布'),
            make_exit_handler('field_visualizer', 'RViz 场地可视化将停止'),
            LogInfo(msg='[感知节点] 所有感知节点已提交启动'),
        ],
    ))

    return ld
