"""以 FAST-LIO 启动时的车体中心位姿为原点，实时发布相对位置。

与 field_localizer 的区别：不做赛场原点采样和区域判定，仅做雷达→车体外参转换，
适合快速验证雷达定位和车体中心坐标是否正确。
"""

from math import atan2, cos, radians, sin

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from fyt_pos.field_geometry import Point2D, lidar_to_base_pose


class SimplePosition(Node):
    """实时发布车体中心相对启动位置的位移和朝向。"""

    def __init__(self) -> None:
        super().__init__('simple_position')
        self.declare_parameter('odometry_topic', '/fastlio2/lio_odom')
        self.declare_parameter('position_topic', '/lidar/relative_position')
        self.declare_parameter('print_period_s', 0.5)
        # 雷达→车体外参（与 field_localizer 保持一致）
        self.declare_parameter('lidar_to_base_enabled', True)
        self.declare_parameter('base_to_lidar_x_m', -0.205)
        self.declare_parameter('base_to_lidar_y_m', 0.16)
        self.declare_parameter('base_to_lidar_yaw_deg', 90.0)

        self._origin = None
        self._origin_yaw = 0.0
        self._latest_position = None
        self._latest_yaw = 0.0
        odometry_topic = str(self.get_parameter('odometry_topic').value)
        position_topic = str(self.get_parameter('position_topic').value)
        self._position_pub = self.create_publisher(PointStamped, position_topic, 10)
        self.create_subscription(Odometry, odometry_topic, self._odometry_callback, 10)
        self.create_timer(float(self.get_parameter('print_period_s').value), self._print_position)
        self.get_logger().info(f'等待 {odometry_topic}，首帧将作为相对坐标原点。')

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        return atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _odometry_callback(self, message: Odometry) -> None:
        position = Point2D(message.pose.pose.position.x, message.pose.pose.position.y)
        yaw = self._yaw_from_quaternion(message.pose.pose.orientation)

        # 雷达坐标系 → 车体中心坐标系
        if bool(self.get_parameter('lidar_to_base_enabled').value):
            position, yaw = lidar_to_base_pose(
                position,
                yaw,
                Point2D(
                    float(self.get_parameter('base_to_lidar_x_m').value),
                    float(self.get_parameter('base_to_lidar_y_m').value),
                ),
                radians(float(self.get_parameter('base_to_lidar_yaw_deg').value)),
            )

        if self._origin is None:
            self._origin = (position.x, position.y)
            self._origin_yaw = yaw
            self.get_logger().info(
                f'车体原点已建立: x={position.x:.3f}, y={position.y:.3f}, yaw={yaw:.3f} rad'
            )

        relative_x = position.x - self._origin[0]
        relative_y = position.y - self._origin[1]
        relative_yaw = yaw - self._origin_yaw

        relative_position = PointStamped()
        relative_position.header = message.header
        relative_position.header.frame_id = 'base_start'
        relative_position.point.x = relative_x
        relative_position.point.y = relative_y
        relative_position.point.z = relative_yaw  # z 存放相对偏航角（弧度）
        self._latest_position = relative_position
        self._latest_yaw = relative_yaw
        self._position_pub.publish(relative_position)

    def _print_position(self) -> None:
        if self._latest_position is None:
            return
        point = self._latest_position.point
        self.get_logger().info(
            f'车体相对位置: x={point.x:.3f}, y={point.y:.3f} m, '
            f'yaw={self._latest_yaw:.3f} rad ({self._latest_yaw * 57.2958:.1f}°)'
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