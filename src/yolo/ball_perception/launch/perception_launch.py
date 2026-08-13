import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('ball_perception')

    # USB相机节点
    camera_node = Node(
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

    # 球距离融合节点
    ball_model_path = PathJoinSubstitution([
        FindPackageShare('ball_perception'), 'models', 'best.pt',
    ])
    distance_node = Node(
        package='ball_perception',
        executable='ball_distance_node',
        name='ball_distance_node',
        output='screen',
        parameters=[{
            'model_path': ball_model_path,
                            'conf_threshold': 0.5,
            # 相机外参: 相机在雷达右侧17.3cm、后方3.5cm
            'extrinsic_x': -0.035,
            'extrinsic_y': -0.173,
            'extrinsic_z': 0.0,
            'extrinsic_roll': 0.0,
            'extrinsic_pitch': 0.0,
            'extrinsic_yaw': 0.0,
            # 输出坐标转换: 雷达→夹爪
            'output_to_gripper': True,
            'lidar_to_gripper_x': -0.174,
            'lidar_to_gripper_y': 0.173,
            'lidar_to_gripper_yaw_deg': -90.0,
            # 距离滤波
            'min_distance': 0.1,
            'max_distance': 10.0,
            'distance_alpha': 0.7,
        }],
    )

    return LaunchDescription([
        camera_node,
        distance_node,
    ])
