"""根据 FAST-LIO 里程计发布稳定的比赛场地位姿。"""

from math import atan2, cos, radians, sin
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from fyt_pos.field_geometry import (
    AxisAlignedZone,
    Point2D,
    classify_zone,
    lidar_to_base_pose,
    rotate_to_field,
    start_corner_transform,
)


class FieldLocalizer(Node):
    """在启动时锚定里程计原点，并判定机器人所在的比赛区域。"""

    def __init__(self) -> None:
        super().__init__('field_localizer')
        # 所有参数均可由 field.yaml 或 launch 文件覆盖，避免场地数据硬编码在逻辑中。
        self.declare_parameter('odometry_topic', '/fastlio2/lio_odom')
        self.declare_parameter('field_pose_topic', '/competition/field_pose')
        self.declare_parameter('field_zone_topic', '/competition/current_zone')
        self.declare_parameter('initial_sample_count', 30)
        self.declare_parameter('field_yaw_offset_deg', 0.0)
        self.declare_parameter('start_corner', 'bottom_left')
        self.declare_parameter('lidar_to_base_enabled', True)
        self.declare_parameter('base_to_lidar_x_m', -0.205)
        self.declare_parameter('base_to_lidar_y_m', 0.16)
        self.declare_parameter('base_to_lidar_yaw_deg', 90.0)
        self.declare_parameter('allowed_min_x', 0.0)
        self.declare_parameter('allowed_max_x', 11.0)
        self.declare_parameter('allowed_min_y', 0.0)
        self.declare_parameter('allowed_max_y', 6.0)
        self.declare_parameter('zones.names', rclpy.Parameter.Type.STRING_ARRAY)

        # 多帧取均值可减小 FAST-LIO 刚启动时的短暂抖动对比赛原点的影响。
        self._sample_count = int(self.get_parameter('initial_sample_count').value)
        self._yaw_offset = radians(float(self.get_parameter('field_yaw_offset_deg').value))
        self._samples: List[Point2D] = []
        self._origin: Point2D | None = None
        self._origin_yaw = 0.0
        self._zones = self._load_zones()

        odometry_topic = str(self.get_parameter('odometry_topic').value)
        pose_topic = str(self.get_parameter('field_pose_topic').value)
        zone_topic = str(self.get_parameter('field_zone_topic').value)
        self._pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        self._zone_pub = self.create_publisher(String, zone_topic, 10)
        self.create_subscription(Odometry, odometry_topic, self._odometry_callback, 10)
        self.get_logger().info(f'正在等待 {odometry_topic} 的 {self._sample_count} 帧里程计数据。')

    def _load_zones(self) -> List[AxisAlignedZone]:
        zones = []
        names = self.get_parameter('zones.names').value
        for name in names:
            # 每个区域使用 [min_x, max_x, min_y, max_y] 的四元素数组配置。
            self.declare_parameter(f'zones.{name}', rclpy.Parameter.Type.DOUBLE_ARRAY)
            bounds = self.get_parameter(f'zones.{name}').value
            if len(bounds) != 4:
                self.get_logger().warning(f'区域 {name} 的边界参数无效，已忽略。')
                continue
            zones.append(AxisAlignedZone(name, *map(float, bounds)))
        return zones

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        # 从 ROS 四元数提取绕 Z 轴的偏航角，适用于平面赛场。
        return atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _odometry_callback(self, message: Odometry) -> None:
        # FAST-LIO 位姿通常描述雷达；比赛区域和底盘控制使用车体中心。
        position = Point2D(message.pose.pose.position.x, message.pose.pose.position.y)
        yaw = self._yaw_from_quaternion(message.pose.pose.orientation)
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
            # 原点尚未建立时，仅累积样本，不向控制或显示模块发布不稳定坐标。
            self._samples.append(position)
            if len(self._samples) < self._sample_count:
                return
            self._origin = Point2D(
                sum(point.x for point in self._samples) / len(self._samples),
                sum(point.y for point in self._samples) / len(self._samples),
            )
            self._origin_yaw = yaw
            self.get_logger().info('比赛场地坐标原点初始化完成。')
            return

        # 先消除 LIO 启动朝向，再映射为以本方起点为原点的坐标。
        heading = self._origin_yaw + self._yaw_offset
        local_position = rotate_to_field(position, self._origin, heading)
        try:
            field_position = start_corner_transform(
                local_position,
                str(self.get_parameter('start_corner').value),
                0.0,
                0.0,
            )
        except ValueError as error:
            self.get_logger().error(f'{error}，已按 bottom_left 处理。')
            field_position = local_position
        field_x = field_position.x
        field_y = field_position.y
        # 场地坐标系下的朝向：车体 yaw 减去场地旋转基准。
        field_yaw = yaw - heading
        pose = PoseStamped()
        pose.header = message.header
        # 该 frame 只表示平面比赛场地，不替代 TF 树中的全局地图坐标系。
        pose.header.frame_id = 'competition_field'
        pose.pose.position.x = field_x
        pose.pose.position.y = field_y
        pose.pose.position.z = message.pose.pose.position.z
        # 将 field_yaw 编码为绕 Z 轴的四元数，供串口网关提取。
        pose.pose.orientation.z = sin(field_yaw / 2.0)
        pose.pose.orientation.w = cos(field_yaw / 2.0)
        self._pose_pub.publish(pose)

        # 先按业务区域分类，再将超出全场边界的结果提升为明确的安全告警。
        zone = classify_zone(Point2D(field_x, field_y), self._zones) or 'outside_configured_zones'
        if not self._within_allowed_field(field_x, field_y):
            zone = 'outside_field_boundary'
        self._zone_pub.publish(String(data=zone))

    def _within_allowed_field(self, x: float, y: float) -> bool:
        # 比赛场地总边界独立于功能区域，便于识别定位异常或违规越界。
        return (
            float(self.get_parameter('allowed_min_x').value) <= x <= float(self.get_parameter('allowed_max_x').value)
            and float(self.get_parameter('allowed_min_y').value) <= y <= float(self.get_parameter('allowed_max_y').value)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()