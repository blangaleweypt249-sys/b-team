"""以 FAST-LIO 启动时的 LiDAR 位姿为原点，发布并显示相对位置。"""

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SimplePosition(Node):
    """用于快速验证雷达定位输出的最小相对位置节点。"""

    def __init__(self) -> None:
        super().__init__('simple_position')
        self.declare_parameter('odometry_topic', '/fastlio2/lio_odom')
        self.declare_parameter('position_topic', '/lidar/relative_position')
        self.declare_parameter('print_period_s', 0.5)

        self._origin = None
        self._latest_position = None
        odometry_topic = str(self.get_parameter('odometry_topic').value)
        position_topic = str(self.get_parameter('position_topic').value)
        self._position_pub = self.create_publisher(PointStamped, position_topic, 10)
        self.create_subscription(Odometry, odometry_topic, self._odometry_callback, 10)
        self.create_timer(float(self.get_parameter('print_period_s').value), self._print_position)
        self.get_logger().info(f'等待 {odometry_topic}，首帧将作为相对坐标原点。')

    def _odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        if self._origin is None:
            self._origin = (position.x, position.y, position.z)
            self.get_logger().info(
                f'原点已建立: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f} m'
            )

        relative_position = PointStamped()
        relative_position.header = message.header
        relative_position.header.frame_id = 'lidar_start'
        relative_position.point.x = position.x - self._origin[0]
        relative_position.point.y = position.y - self._origin[1]
        relative_position.point.z = position.z - self._origin[2]
        self._latest_position = relative_position
        self._position_pub.publish(relative_position)

    def _print_position(self) -> None:
        if self._latest_position is None:
            return
        point = self._latest_position.point
        self.get_logger().info(
            f'相对初始位置: x={point.x:.3f}, y={point.y:.3f}, z={point.z:.3f} m'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimplePosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()