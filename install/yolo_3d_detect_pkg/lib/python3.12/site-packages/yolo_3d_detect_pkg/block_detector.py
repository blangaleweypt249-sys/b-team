"""使用可替换的 YOLO 模型检测目标，并发布 RGB-D 三维位置。"""

import os
import threading
from pathlib import Path
from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from yolo_3d_detect_pkg.block_geometry import (
    map_color_pixel_to_depth,
    median_depth_m,
    pixel_depth_to_robot_camera_point,
)


class BlockDetector(Node):
    """检测指定类别的目标中心，并将候选以相机坐标发布。"""

    def __init__(self) -> None:
        super().__init__('block_detector')
        for name, default in (
            ('color_topic', '/camera/color/image_raw'),
            ('depth_topic', '/camera/depth/image_raw'),
            ('blocks_topic', '/competition/vision/blocks'),
            ('overlay_topic', '/competition/vision/blocks_overlay'),
            ('model_path', ''),
            ('box_class_id', 0),
            ('confidence_threshold', 0.4),
            ('inference_image_size', 640),
            ('max_detections', 7),
            ('depth_is_registered', True),
            ('depth_sample_window', 5),
            ('min_depth_m', 0.2),
            ('max_depth_m', 2.5),
            ('camera_fx', 468.431622),
            ('camera_fy', 468.585260),
            ('camera_cx', 323.200725),
            ('camera_cy', 245.471993),
        ):
            self.declare_parameter(name, default)

        self._bridge = CvBridge()
        self._model = None
        self._model_lock = threading.Lock()
        self._model_error: Optional[str] = None
        self._blocks_pub = self.create_publisher(PoseArray, str(self.get_parameter('blocks_topic').value), 10)
        self._overlay_pub = self.create_publisher(Image, str(self.get_parameter('overlay_topic').value), 10)
        color_sub = message_filters.Subscriber(self, Image, str(self.get_parameter('color_topic').value), qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, str(self.get_parameter('depth_topic').value), qos_profile=qos_profile_sensor_data)
        self._sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=4, slop=0.05)
        self._sync.registerCallback(self._images_callback)
        # 模型加载可能很慢，放入后台线程避免阻塞 ROS 图像订阅建立。
        self._loader = threading.Thread(target=self._load_model, daemon=True)
        self._loader.start()

    def _load_model(self) -> None:
        # 空参数使用包内约定文件名；部署时无需修改源码即可替换训练权重。
        configured_model_path = str(self.get_parameter('model_path').value).strip()
        model_path = (
            Path(configured_model_path).expanduser()
            if configured_model_path
            else Path(get_package_share_directory('yolo_3d_detect_pkg')) / 'model' / 'detector.pt'
        )
        if not model_path.is_file():
            self._model_error = f'未找到检测模型：{model_path}'
            self.get_logger().error(self._model_error)
            return
        try:
            from ultralytics import YOLO

            model = YOLO(os.fspath(model_path))
            with self._model_lock:
                self._model = model
            self.get_logger().info(f'检测模型已加载：{model_path.name}')
        except Exception as error:
            self._model_error = f'加载检测模型失败：{error}'
            self.get_logger().error(self._model_error)

    def _images_callback(self, color_message: Image, depth_message: Image) -> None:
        try:
            color = self._bridge.imgmsg_to_cv2(color_message, desired_encoding='bgr8')
            depth = self._bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
            overlay = color.copy()
            blocks = PoseArray()
            blocks.header = color_message.header
            # 未加载模型时仍发布空数组，使消费者能明确区分“无目标”和旧数据。
            with self._model_lock:
                model = self._model
            if model is not None:
                blocks.poses = self._detect_blocks(model, color, depth, overlay)
            self._blocks_pub.publish(blocks)
            self._overlay_pub.publish(self._bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
        except Exception as error:
            self.get_logger().warning(f'目标视觉回调执行失败：{error}')

    def _detect_blocks(self, model, color: np.ndarray, depth: np.ndarray, overlay: np.ndarray) -> list[Pose]:
        # 仅请求训练配置指定的类别，避免其他类别进入抓取候选。
        results = model.predict(
            color,
            verbose=False,
            classes=[int(self.get_parameter('box_class_id').value)],
            conf=float(self.get_parameter('confidence_threshold').value),
            imgsz=int(self.get_parameter('inference_image_size').value),
            max_det=int(self.get_parameter('max_detections').value),
        )
        poses = []
        for result in results:
            if result.boxes is None:
                continue
            for box, confidence in zip(result.boxes.xyxy.cpu().numpy(), result.boxes.conf.cpu().numpy()):
                pose = self._box_to_pose(box, float(confidence), color.shape, depth, overlay)
                if pose is not None:
                    poses.append(pose)
        return poses

    def _box_to_pose(self, box, confidence: float, color_shape, depth: np.ndarray, overlay: np.ndarray) -> Optional[Pose]:
        x1, y1, x2, y2 = map(int, box)
        pixel_x = max(0, min(color_shape[1] - 1, (x1 + x2) // 2))
        pixel_y = max(0, min(color_shape[0] - 1, (y1 + y2) // 2))
        depth_x, depth_y = map_color_pixel_to_depth(
            pixel_x, pixel_y, color_shape, depth.shape, bool(self.get_parameter('depth_is_registered').value)
        )
        distance = median_depth_m(depth, depth_x, depth_y, int(self.get_parameter('depth_sample_window').value))
        if not float(self.get_parameter('min_depth_m').value) <= distance <= float(self.get_parameter('max_depth_m').value):
            return None
        camera_x, camera_y, camera_z = pixel_depth_to_robot_camera_point(
            depth_x,
            depth_y,
            distance,
            float(self.get_parameter('camera_fx').value),
            float(self.get_parameter('camera_fy').value),
            float(self.get_parameter('camera_cx').value),
            float(self.get_parameter('camera_cy').value),
        )
        pose = Pose()
        pose.position.x = camera_x
        pose.position.y = camera_y
        pose.position.z = camera_z
        # PoseArray 没有置信度字段，使用单位四元数避免将置信度伪装成姿态。
        pose.orientation.w = 1.0
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 180, 0), 2)
        cv2.putText(overlay, f'block {distance:.2f}m {confidence:.2f}', (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 2)
        return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()