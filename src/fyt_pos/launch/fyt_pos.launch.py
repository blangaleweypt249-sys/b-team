from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('fyt_pos'), 'config', 'field.yaml'])
    return LaunchDescription([
        Node(package='fyt_pos', executable='field_localizer', parameters=[config]),
        Node(package='fyt_pos', executable='field_visualizer', parameters=[config]),
    ])