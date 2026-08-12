#!/usr/bin/env python3
"""USB摄像头发布节点 - 将技彩MF100相机图像发布为ROS2话题。

发布话题:
  /camera/image_raw (sensor_msgs/Image) - 原始图像 (640x480, BGR)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class USBCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        # 参数
        self.declare_parameter('device_id', 2)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        # 相机内参 (来自标定文件, 640x480)
        self.declare_parameter('fx', 657.5551152642361)
        self.declare_parameter('fy', 656.3202788734417)
        self.declare_parameter('cx', 314.7869474017467)
        self.declare_parameter('cy', 191.0288244576825)

        device_id = self.get_parameter('device_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # 打开摄像头
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开 /dev/video{device_id}')
            raise RuntimeError(f'无法打开 /dev/video{device_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'摄像头已打开: /dev/video{device_id}, {actual_w}x{actual_h}, FPS {actual_fps:.1f}'
        )

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # 发布CameraInfo
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_link'
        self.camera_info.height = actual_h
        self.camera_info.width = actual_w
        self.camera_info.distortion_model = 'plumb_bob'
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        self.camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        self.camera_info.d = [0.2552196052871585, -1.0293197399852791,
                              -0.012673932388104929, 0.0027336147522840394,
                              1.5452004344167767]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        # 定时器采集
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('读取帧失败', throttle_duration_sec=1.0)
            return

        # 发布图像
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.frame_id = 'camera_link'
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(img_msg)

        # 发布CameraInfo
        self.camera_info.header.stamp = img_msg.header.stamp
        self.info_pub.publish(self.camera_info)

        self.frame_count += 1

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
