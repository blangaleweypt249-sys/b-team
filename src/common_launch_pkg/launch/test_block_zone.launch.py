"""块检测区域测试启动文件(通用一套,不含球检测,定位+块检测)。

定位配置使用 field.yaml(全局赛场坐标,两车统一,内部坐标系不变)。
块检测只在 block/special1/special2 区域内运行。
如需串口通信,蓝方单独启动 serial_gateway.launch.py,红方启动 serial_gateway_red.launch.py。

启动内容：
  1. lidar 驱动 + FAST-LIO 由 lidar_driver.launch.py 单独启动,本文件不重复。
     本文件依赖 /Odometry 已发布。
  2. fyt_pos (field_localizer)   
     — 订阅 /Odometry,发布 /competition/field_pose, /competition/current_zone
  3. block_perception (orbbec_camera_node + block_distance_node)
     — 只在 block/special1/special2 区域内检测块
     — 发布 /perception/block_position

启动顺序：
  阶段 0s : Orbbec 相机
  阶段 2s : 块检测融合节点
  阶段 3s : 场地定位节点(等待 /Odometry)

独立启动命令(先启动雷达)：
  ros2 launch common_launch_pkg lidar_driver.launch.py
  ros2 launch common_launch_pkg test_block_zone.launch.py
需要串口通信(可选,另开终端):
  蓝方: ros2 launch common_launch_pkg serial_gateway.launch.py
  红方: ros2 launch common_launch_pkg serial_gateway_red.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定位配置:两车统一使用 field.yaml (内部坐标系不变)
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

    # ---- block_perception: 块检测 ----
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

    # ---- 错误处理 ----
    def make_exit_handler(node_name, impact):
        return RegisterEventHandler(
            event_handler=OnProcessExit(
                on_exit=[LogInfo(msg=f'[警告-测试] {node_name} 已退出。影响：{impact}')],
            )
        )

    ld = LaunchDescription()

    # 阶段 1: Orbbec 相机
    ld.add_action(LogInfo(msg='[块区域测试] 启动 Orbbec RGB-D 相机...'))
    ld.add_action(orbbec_camera_node)
    ld.add_action(make_exit_handler('orbbec_camera_node', '块检测将无法获取图像'))

    # 阶段 2: 延迟 2 秒启动块检测融合
    ld.add_action(TimerAction(period=2.0, actions=[
        LogInfo(msg='[块区域测试] 启动块检测+测距 (仅block/special1/special2区有效)...'),
        block_distance_node,
        make_exit_handler('block_distance_node', '/perception/block_position 将停止发布'),
    ]))

    # 阶段 3: 延迟 3 秒启动场地定位
    ld.add_action(TimerAction(period=3.0, actions=[
        LogInfo(msg='[块区域测试] 启动场地定位(统一field.yaml,内部坐标系不变)...'),
        field_localizer,
        field_visualizer,
        make_exit_handler('field_localizer', '/competition/field_pose 将停止发布'),
        make_exit_handler('field_visualizer', 'RViz 场地可视化将停止'),
        LogInfo(msg='[块区域测试] 所有节点已提交启动;需串口请另启serial_gateway(_red).launch.py'),
    ]))

    return ld
