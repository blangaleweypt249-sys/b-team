import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('block_perception')

    # Orbbec 相机节点
    camera_node = Node(
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

    # 红蓝块检测+测距节点
    distance_node = Node(
        package='block_perception',
        executable='block_distance_node',
        name='block_distance_node',
        output='screen',
        parameters=[{
            'model_path': '/home/husky/TR/src/yolo/runs/segment/blue_red/weights/best.pt',
            'conf_threshold': 0.75,
            'iou_threshold': 0.45,
            'red_class_name': 'red',
            'blue_class_name': 'blue',
            'distance_alpha': 0.7,
        }],    
    )

    return LaunchDescription([
        camera_node,
        distance_node,
    ])
