"""发布面向操作员的简洁 RViz 赛场标记。"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class FieldVisualizer(Node):
    """在 RViz 中绘制比赛边界与机器人实时位姿。"""

    def __init__(self) -> None:
        super().__init__('field_visualizer')
        self.declare_parameter('field_pose_topic', '/competition/field_pose')
        self.declare_parameter('marker_topic', '/competition/field_markers')
        self._markers = self.create_publisher(MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.create_subscription(PoseStamped, str(self.get_parameter('field_pose_topic').value), self._pose_callback, 10)

    def _pose_callback(self, pose: PoseStamped) -> None:
        # 半透明棕色平面用于提供场地轮廓，不代表精确的得分区域。
        boundary = Marker()
        boundary.header.frame_id = 'competition_field'
        boundary.header.stamp = self.get_clock().now().to_msg()
        boundary.ns = 'competition_field'
        boundary.id = 1
        boundary.type = Marker.CUBE
        boundary.action = Marker.ADD
        boundary.pose.position.x = 5.5
        boundary.pose.position.y = 3.0
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = 11.0
        boundary.scale.y = 6.0
        boundary.scale.z = 0.02
        boundary.color.r = 0.55
        boundary.color.g = 0.32
        boundary.color.b = 0.12
        boundary.color.a = 0.25

        # 青色箭头以车体姿态显示机器人在场地坐标系中的当前位置和朝向。
        robot = Marker()
        robot.header = boundary.header
        robot.ns = 'competition_robot'
        robot.id = 2
        robot.type = Marker.ARROW
        robot.action = Marker.ADD
        robot.pose = pose.pose
        robot.scale.x = 0.7
        robot.scale.y = 0.25
        robot.scale.z = 0.25
        robot.color.r = 0.1
        robot.color.g = 0.8
        robot.color.b = 1.0
        robot.color.a = 1.0
        self._markers.publish(MarkerArray(markers=[boundary, robot]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()