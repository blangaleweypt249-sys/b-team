#!/usr/bin/env python3
"""Orbbec Gemini2 RGB-D 相机发布节点。

发布话题:
  /orbbec/color/image_raw  (sensor_msgs/Image)        - RGB 图像
  /orbbec/depth/image_raw  (sensor_msgs/Image)        - 深度图像 (单位 mm, float32)
  /orbbec/camera_info       (sensor_msgs/CameraInfo)  - 相机内参
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

from pyorbbecsdk import (
    AlignFilter,
    Config,
    OBFormat,
    OBSensorType,
    OBStreamType,
    Pipeline,
)


class OrbbecCameraNode(Node):
    def __init__(self):
        super().__init__('orbbec_camera_node')

        # ---- 参数 ----
        self.declare_parameter('color_width', 1280)
        self.declare_parameter('color_height', 720)
        self.declare_parameter('depth_width', 0)
        self.declare_parameter('depth_height', 0)
        self.declare_parameter('min_depth_mm', 20.0)
        self.declare_parameter('max_depth_mm', 10000.0)

        color_w = self.get_parameter('color_width').value
        color_h = self.get_parameter('color_height').value
        depth_w = self.get_parameter('depth_width').value
        depth_h = self.get_parameter('depth_height').value
        self.min_depth_mm = self.get_parameter('min_depth_mm').value
        self.max_depth_mm = self.get_parameter('max_depth_mm').value

        # ---- 初始化 Orbbec Pipeline ----
        self.pipeline = Pipeline()
        config = Config()

        color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)

        # 配置 RGB 流
        color_profile = None
        if color_w > 0 and color_h > 0:
            for profile in color_profiles:
                if (profile.get_width() == color_w and
                        profile.get_height() == color_h and
                        profile.get_format() == OBFormat.RGB):
                    color_profile = profile
                    break
        if color_profile is None:
            color_profile = color_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)

        # 配置 Depth 流
        depth_profile = None
        if depth_w > 0 and depth_h > 0:
            for profile in depth_profiles:
                if (profile.get_width() == depth_w and
                        profile.get_height() == depth_h):
                    depth_profile = profile
                    break
        if depth_profile is None:
            depth_profile = depth_profiles.get_default_video_stream_profile()
        config.enable_stream(depth_profile)

        actual_color_w = color_profile.get_width()
        actual_color_h = color_profile.get_height()
        actual_depth_w = depth_profile.get_width()
        actual_depth_h = depth_profile.get_height()

        self.get_logger().info(
            f'Orbbec 相机已打开: RGB {actual_color_w}x{actual_color_h}, '
            f'Depth {actual_depth_w}x{actual_depth_h}'
        )

        # 启动 Pipeline 和对齐滤波器
        self.pipeline.start(config)
        self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)
        self.get_logger().info('D2C 对齐已启用')

        # 获取内参
        camera_param = self.pipeline.get_camera_param()
        rgb_intrinsic = camera_param.rgb_intrinsic
        scale_x = actual_color_w / float(rgb_intrinsic.width)
        scale_y = actual_color_h / float(rgb_intrinsic.height)
        self.fx = float(rgb_intrinsic.fx) * scale_x
        self.fy = float(rgb_intrinsic.fy) * scale_y
        self.cx = float(rgb_intrinsic.cx) * scale_x
        self.cy = float(rgb_intrinsic.cy) * scale_y

        self.get_logger().info(
            f'内参: fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

        # ---- ROS 发布器 ----
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/orbbec/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/orbbec/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/orbbec/camera_info', 10)

        # CameraInfo
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'orbbec_link'
        self.camera_info.height = actual_color_h
        self.camera_info.width = actual_color_w
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        # 定时器 (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('Orbbec 相机节点已启动')

    def _frame_to_bgr(self, color_frame):
        """将 Orbbec 颜色帧转为 BGR 图像"""
        width = color_frame.get_width()
        height = color_frame.get_height()
        color_format = color_frame.get_format()
        data = np.asanyarray(color_frame.get_data())

        if color_format == OBFormat.RGB:
            image = np.resize(data, (height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif color_format == OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        elif color_format == OBFormat.YUYV:
            image = np.resize(data, (height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
        else:
            return None

    def _extract_depth(self, depth_frame):
        """将 Orbbec 深度帧转为深度矩阵 (mm, float32)"""
        raw_depth = np.frombuffer(
            depth_frame.get_data(),
            dtype=np.uint16,
        ).reshape(
            depth_frame.get_height(),
            depth_frame.get_width(),
        )

        depth_scale = float(depth_frame.get_depth_scale())
        depth_mm = raw_depth.astype(np.float32) * depth_scale

        valid_mask = (depth_mm > self.min_depth_mm) & (depth_mm < self.max_depth_mm)
        depth_mm = np.where(valid_mask, depth_mm, 0.0)
        return depth_mm

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames(1000)
        if not frames:
            return

        # D2C 对齐
        frames = self.align_filter.process(frames)
        if not frames:
            return

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            return

        # RGB -> BGR
        bgr_image = self._frame_to_bgr(color_frame)
        if bgr_image is None:
            return

        # Depth -> 深度矩阵
        depth_map = self._extract_depth(depth_frame)
        if depth_map is None:
            return

        # 确保深度和彩色图尺寸一致
        if depth_map.shape != bgr_image.shape[:2]:
            depth_map = cv2.resize(depth_map, (bgr_image.shape[1], bgr_image.shape[0]),
                                   interpolation=cv2.INTER_NEAREST)

        stamp = self.get_clock().now().to_msg()

        # 发布彩色图像
        color_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
        color_msg.header.frame_id = 'orbbec_link'
        color_msg.header.stamp = stamp
        self.color_pub.publish(color_msg)

        # 发布深度图像 (float32, 单位 mm)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
        depth_msg.header.frame_id = 'orbbec_link'
        depth_msg.header.stamp = stamp
        self.depth_pub.publish(depth_msg)

        # 发布 CameraInfo
        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        try:
            self.pipeline.stop()
            self.get_logger().info('Orbbec Pipeline 已停止')
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
