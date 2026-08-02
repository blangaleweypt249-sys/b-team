"""比赛金色球形灵石的 RGB-D 视觉检测节点。"""

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VisionDetector(Node):
    """检测金色圆形目标，并发布相机坐标系下的三维坐标。"""

    def __init__(self) -> None:
        super().__init__('vision_detector')
        # 参数集中声明，现场可通过 YAML 调整相机话题、内参和检测阈值。
        for name, default in (
            ('color_topic', '/camera/color/image_raw'),
            ('depth_topic', '/camera/depth/image_raw'),
            ('target_topic', '/competition/vision/target'),
            ('overlay_topic', '/competition/vision/overlay'),
            ('depth_is_registered', True),
            ('depth_sample_window', 5),
            ('target_min_radius_px', 20.0),
            ('target_max_radius_px', 220.0),
            ('target_min_depth_m', 0.2),
            ('target_max_depth_m', 3.0),
            ('camera_fx', 468.431622),
            ('camera_fy', 468.585260),
            ('camera_cx', 323.200725),
            ('camera_cy', 245.471993),
            ('show_debug_window', False),
        ):
            self.declare_parameter(name, default)

        self._bridge = CvBridge()
        self._target_pub = self.create_publisher(PointStamped, str(self.get_parameter('target_topic').value), 10)
        self._overlay_pub = self.create_publisher(Image, str(self.get_parameter('overlay_topic').value), 10)
        color_sub = message_filters.Subscriber(self, Image, str(self.get_parameter('color_topic').value), qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, str(self.get_parameter('depth_topic').value), qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=4, slop=0.05)
        self._sync.registerCallback(self._images_callback)

    def _images_callback(self, color_message: Image, depth_message: Image) -> None:
        try:
            # 彩色图用于颜色分割，深度图用于将二维候选点恢复为三维坐标。
            color = self._bridge.imgmsg_to_cv2(color_message, desired_encoding='bgr8')
            depth = self._bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
            candidate = self._find_gold_circle(color)
            overlay = color.copy()
            if candidate is not None:
                center_x, center_y, radius = candidate
                depth_x, depth_y = self._map_to_depth(center_x, center_y, color.shape, depth.shape)
                distance = self._median_depth_m(depth, depth_x, depth_y)
                if self._valid_depth(distance):
                    self._publish_target(color_message, depth_x, depth_y, distance)
                    cv2.circle(overlay, (center_x, center_y), radius, (0, 255, 0), 2)
                    cv2.putText(overlay, f'ling shi {distance:.2f}m', (center_x - radius, center_y - radius - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            self._overlay_pub.publish(self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
        except Exception as error:
            self.get_logger().warning(f'视觉回调执行失败：{error}')

    def _find_gold_circle(self, color: np.ndarray):
        # 先提取金黄色像素，再用最大轮廓的外接圆过滤出近似球形目标。
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([15, 70, 70]), np.array([45, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        if not float(self.get_parameter('target_min_radius_px').value) <= radius <= float(self.get_parameter('target_max_radius_px').value):
            return None
        return int(center_x), int(center_y), int(radius)

    def _map_to_depth(self, x, y, color_shape, depth_shape):
        # 未对齐的 RGB-D 设备需要按图像分辨率将彩色像素映射到深度像素。
        if bool(self.get_parameter('depth_is_registered').value):
            return x, y
        return int(x * depth_shape[1] / color_shape[1]), int(y * depth_shape[0] / color_shape[0])

    def _median_depth_m(self, depth, x, y) -> float:
        # 使用小窗口中值抑制边缘空洞和单点深度噪声；相机深度单位默认毫米。
        half = max(1, int(self.get_parameter('depth_sample_window').value) // 2)
        roi = depth[max(0, y - half):y + half + 1, max(0, x - half):x + half + 1]
        valid = roi[(roi > 0) & (roi < 20000)]
        return float(np.median(valid)) / 1000.0 if valid.size else 0.0

    def _valid_depth(self, distance: float) -> bool:
        return float(self.get_parameter('target_min_depth_m').value) <= distance <= float(self.get_parameter('target_max_depth_m').value)

    def _publish_target(self, header_message, pixel_x, pixel_y, distance) -> None:
        # 针孔模型反投影：输出约定为 x 向前、y 向左、z 向上，供夹爪对位模块使用。
        fx = float(self.get_parameter('camera_fx').value)
        fy = float(self.get_parameter('camera_fy').value)
        cx = float(self.get_parameter('camera_cx').value)
        cy = float(self.get_parameter('camera_cy').value)
        target = PointStamped()
        target.header = header_message.header
        target.point.x = distance
        target.point.y = -((pixel_x - cx) * distance / fx)
        target.point.z = -((pixel_y - cy) * distance / fy)
        self._target_pub.publish(target)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()