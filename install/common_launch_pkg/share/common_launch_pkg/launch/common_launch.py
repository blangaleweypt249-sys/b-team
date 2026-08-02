from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 先由 FAST-LIO 提供 /Odometry，再启动依赖该话题的定位节点。
    fastlio_launch = PathJoinSubstitution([
        FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py',
    ])
    position_config = PathJoinSubstitution([
        FindPackageShare('fyt_pos'), 'config', 'field.yaml',
    ])
    vision_config = PathJoinSubstitution([
        FindPackageShare('yolo_3d_detect_pkg'), 'config', 'vision.yaml',
    ])
    gateway_config = PathJoinSubstitution([
        FindPackageShare('competition_gateway'), 'config', 'serial_gateway.yaml',
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch),
            launch_arguments={'config_file': 'mid360.yaml', 'rviz': 'false'}.items(),
        ),
        # 定位与视觉仅发布感知结果；唯一打开串口并对接下位机的是 C++ 网关。
        Node(package='fyt_pos', executable='field_localizer', parameters=[position_config]),
        Node(package='fyt_pos', executable='field_visualizer', parameters=[position_config]),
        Node(package='yolo_3d_detect_pkg', executable='vision_detector', parameters=[vision_config]),
        Node(package='yolo_3d_detect_pkg', executable='block_detector', parameters=[vision_config]),
        Node(package='competition_gateway', executable='serial_gateway', parameters=[gateway_config]),
    ])