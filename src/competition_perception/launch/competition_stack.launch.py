from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # 三个节点共享同一份参数文件，以保证话题和坐标配置一致。
    config = PathJoinSubstitution([FindPackageShare('competition_perception'), 'config', 'field.yaml'])
    serial_config = PathJoinSubstitution([FindPackageShare('competition_gateway'), 'config', 'serial_gateway.yaml'])
    return LaunchDescription([
        Node(package='competition_perception', executable='field_localizer', name='field_localizer', parameters=[config]),
        Node(package='competition_perception', executable='vision_detector', name='vision_detector', parameters=[config]),
        Node(package='competition_perception', executable='field_visualizer', name='field_visualizer', parameters=[config]),
        # 网关独占串口，仅传递感知数据和下位机状态，不执行板端控制逻辑。
        Node(package='competition_gateway', executable='serial_gateway', name='serial_gateway', parameters=[serial_config]),
    ])