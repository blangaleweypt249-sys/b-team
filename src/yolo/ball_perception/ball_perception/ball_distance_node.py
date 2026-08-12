#!/usr/bin/env python3
"""金色足球相机-雷达融合测距节点。

核心思路:
  1. YOLOv8-seg 检测图像中金色足球, 获取分割mask
  2. 将MID360点云投影到图像平面 (使用相机内参+外参)
  3. 在球mask区域内搜索投影点, 取深度中值作为球的距离
  4. 发布球的3D位置和距离

订阅话题:
  /camera/image_raw    (sensor_msgs/Image)     - 相机图像
  /livox/lidar         (sensor_msgs/PointCloud2) - MID360点云

发布话题:
  /perception/ball_distance  (std_msgs/Float32)        - 球距离(m)
  /perception/ball_position  (geometry_msgs/PointStamped) - 球3D位置(m, 雷达坐标系)
  /perception/ball_overlay   (sensor_msgs/Image)       - 检测+测距叠加图
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class BallDistanceNode(Node):
    def __init__(self):
        super().__init__('ball_distance_node')

        # ---- 参数 ----
        self.declare_parameter('model_path', '/home/husky/Ling_shi/runs/segment/gold_ball/weights/best.pt')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('lidar_frame', 'livox_frame')

        # 相机内参 (640x480)
        self.declare_parameter('fx', 657.5551152642361)
        self.declare_parameter('fy', 656.3202788734417)
        self.declare_parameter('cx', 314.7869474017467)
        self.declare_parameter('cy', 191.0288244576825)

        # 畸变系数
        self.declare_parameter('d0', 0.2552196052871585)
        self.declare_parameter('d1', -1.0293197399852791)
        self.declare_parameter('d2', -0.012673932388104929)
        self.declare_parameter('d3', 0.0027336147522840394)
        self.declare_parameter('d4', 1.5452004344167767)

        # 外参: 相机在雷达右侧198mm, 平行安装
        # 雷达坐标系: X前 Y左 Z上 (ROS标准)
        # 相机坐标系: Z前 X右 Y下 (OpenCV标准)
        # 相机在雷达坐标系中的位置: (0, -0.198, 0)
        self.declare_parameter('extrinsic_x', 0.0)      # 相机在雷达坐标系中的x位置
        self.declare_parameter('extrinsic_y', -0.198)   # 相机在雷达坐标系中的y位置 (右侧为负)
        self.declare_parameter('extrinsic_z', 0.0)      # 相机在雷达坐标系中的z位置
        self.declare_parameter('extrinsic_roll', 0.0)   # 旋转(弧度)
        self.declare_parameter('extrinsic_pitch', 0.0)
        self.declare_parameter('extrinsic_yaw', 0.0)

        # 距离滤波参数
        self.declare_parameter('min_distance', 0.1)     # 最小有效距离(m)
        self.declare_parameter('max_distance', 10.0)    # 最大有效距离(m)
        self.declare_parameter('distance_alpha', 0.7)   # EMA平滑系数

        # 加载模型
        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f'加载YOLO模型: {model_path}')
        self.model = YOLO(model_path)
        self.conf_threshold = self.get_parameter('conf_threshold').value

        # 相机内参
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.dist_coeffs = np.array([
            self.get_parameter('d0').value,
            self.get_parameter('d1').value,
            self.get_parameter('d2').value,
            self.get_parameter('d3').value,
            self.get_parameter('d4').value,
        ], dtype=np.float64)
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1],
        ], dtype=np.float64)

        # 外参: 雷达坐标系 -> 相机坐标系
        ext_x = self.get_parameter('extrinsic_x').value
        ext_y = self.get_parameter('extrinsic_y').value
        ext_z = self.get_parameter('extrinsic_z').value
        roll = self.get_parameter('extrinsic_roll').value
        pitch = self.get_parameter('extrinsic_pitch').value
        yaw = self.get_parameter('extrinsic_yaw').value

        # 旋转矩阵: 雷达坐标系到相机坐标系
        # 先用户自定义旋转, 再做坐标系轴变换
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ])
        R_user = Rx @ Ry @ Rz

        # 坐标系变换: 雷达(X前,Y左,Z上) -> 相机(Z前,X右,Y下)
        R_axis = np.array([
            [0, -1, 0],
            [0, 0, -1],
            [1, 0, 0],
        ], dtype=np.float64)

        R = R_axis @ R_user
        t = np.array([ext_x, ext_y, ext_z], dtype=np.float64)

        # 变换矩阵 (4x4): 雷达坐标系 -> 相机坐标系
        self.T_lidar_to_cam = np.eye(4)
        self.T_lidar_to_cam[:3, :3] = R
        self.T_lidar_to_cam[:3, 3] = -R @ t

        self.get_logger().info(
            f'外参矩阵 (雷达->相机):\n{self.T_lidar_to_cam}\n'
            f'相机位置(雷达系): ({ext_x}, {ext_y}, {ext_z})'
        )

        # 距离滤波
        self.min_dist = self.get_parameter('min_distance').value
        self.max_dist = self.get_parameter('max_distance').value
        self.dist_alpha = self.get_parameter('distance_alpha').value
        self.smoothed_distance = None

        # 数据缓存
        self.latest_image = None
        self.latest_cloud = None
        self.bridge = CvBridge()

        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.cloud_callback, 10)

        # 发布
        self.dist_pub = self.create_publisher(Float32, '/perception/ball_distance', 10)
        self.pos_pub = self.create_publisher(PointStamped, '/perception/ball_position', 10)
        self.overlay_pub = self.create_publisher(Image, '/perception/ball_overlay', 10)

        # 融合定时器 (20Hz)
        self.timer = self.create_timer(0.05, self.fusion_callback)
        self.get_logger().info('球距离融合节点已启动')

    def image_callback(self, msg):
        self.latest_image = msg

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def fusion_callback(self):
        if self.latest_image is None or self.latest_cloud is None:
            return

        # 获取图像
        frame = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        img_h, img_w = frame.shape[:2]

        # YOLO检测
        results = self.model.predict(frame, conf=self.conf_threshold, verbose=False)
        result = results[0]

        # 绘制检测结果
        overlay = result.plot()

        # 检查是否检测到球
        if result.boxes is None or len(result.boxes) == 0:
            cv2.putText(overlay, 'No ball detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
            return

        # 取置信度最高的球
        confs = result.boxes.conf.cpu().numpy()
        best_idx = np.argmax(confs)
        box = result.boxes[best_idx].xyxy[0].cpu().numpy().astype(int)  # [x1, y1, x2, y2]

        # 获取分割mask
        if result.masks is not None:
            mask = result.masks[best_idx].data[0].cpu().numpy()
            mask = cv2.resize(mask, (img_w, img_h))
            mask_bool = mask > 0.5
        else:
            # 退化: 用bbox区域
            mask_bool = np.zeros((img_h, img_w), dtype=bool)
            mask_bool[box[1]:box[3], box[0]:box[2]] = True

        # 球中心像素
        ball_cx = (box[0] + box[2]) // 2
        ball_cy = (box[1] + box[3]) // 2
        ball_radius = max((box[2] - box[0]) // 2, (box[3] - box[1]) // 2)

        # 处理点云
        cloud_msg = self.latest_cloud
        points = np.array([
            [p[0], p[1], p[2]]
            for p in point_cloud2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ], dtype=np.float64)

        if len(points) == 0:
            self.get_logger().warn('点云为空', throttle_duration_sec=1.0)
            return

        # 变换到相机坐标系
        ones = np.ones((len(points), 1))
        points_h = np.hstack([points, ones])  # (N, 4)
        points_cam = (self.T_lidar_to_cam @ points_h.T).T[:, :3]  # (N, 3)

        # 只保留前方的点 (z > 0)
        valid = points_cam[:, 2] > 0.1
        points_cam = points_cam[valid]
        if len(points_cam) == 0:
            return

        # 投影到图像平面
        points_2d, _ = cv2.projectPoints(
            points_cam.astype(np.float32),
            np.zeros(3), np.zeros(3),
            self.camera_matrix,
            self.dist_coeffs,
        )
        points_2d = points_2d.reshape(-1, 2)

        # 在mask区域内搜索点云投影
        px = points_2d[:, 0].astype(int)
        py = points_2d[:, 1].astype(int)

        # 过滤到图像范围内的点
        in_img = (px >= 0) & (px < img_w) & (py >= 0) & (py < img_h)
        px = px[in_img]
        py = py[in_img]
        depths = points_cam[in_img, 2]

        # 在mask区域内找点
        in_mask = mask_bool[py, px]
        mask_depths = depths[in_mask]

        # 在mask区域外扩一圈搜索 (球边缘可能有雷达回波)
        if len(mask_depths) < 3:
            # 扩大搜索范围: 球中心附近 radius*1.5 范围
            search_r = int(ball_radius * 1.5)
            dx = px - ball_cx
            dy = py - ball_cy
            in_range = (dx * dx + dy * dy) < (search_r * search_r)
            mask_depths = depths[in_range]

        if len(mask_depths) < 3:
            cv2.putText(overlay, f'Ball detected, no lidar points', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
            return

        # 距离滤波
        valid_dist = (mask_depths > self.min_dist) & (mask_depths < self.max_dist)
        mask_depths = mask_depths[valid_dist]

        if len(mask_depths) == 0:
            return

        # 取中值 (抗噪声)
        distance = float(np.median(mask_depths))

        # EMA平滑
        if self.smoothed_distance is None:
            self.smoothed_distance = distance
        else:
            self.smoothed_distance = (
                self.dist_alpha * self.smoothed_distance +
                (1 - self.dist_alpha) * distance
            )

        final_dist = self.smoothed_distance

        # 球的3D位置 (相机坐标系)
        ball_3d_cam = np.array([
            (ball_cx - self.cx) * final_dist / self.fx,
            (ball_cy - self.cy) * final_dist / self.fy,
            final_dist,
        ])

        # 转换回雷达坐标系
        T_cam_to_lidar = np.linalg.inv(self.T_lidar_to_cam)
        ball_3d_cam_h = np.array([ball_3d_cam[0], ball_3d_cam[1], ball_3d_cam[2], 1.0])
        ball_3d_lidar = (T_cam_to_lidar @ ball_3d_cam_h)[:3]

        # 发布距离
        dist_msg = Float32()
        dist_msg.data = final_dist
        self.dist_pub.publish(dist_msg)

        # 发布3D位置 (雷达坐标系)
        pos_msg = PointStamped()
        pos_msg.header.frame_id = 'livox_frame'
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.point.x = float(ball_3d_lidar[0])
        pos_msg.point.y = float(ball_3d_lidar[1])
        pos_msg.point.z = float(ball_3d_lidar[2])
        self.pos_pub.publish(pos_msg)

        # 绘制叠加图
        cv2.putText(overlay, f'Distance: {final_dist:.2f} m', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(overlay, f'Points: {len(mask_depths)}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(overlay, f'Pos: ({ball_3d_lidar[0]:.2f}, {ball_3d_lidar[1]:.2f}, {ball_3d_lidar[2]:.2f})',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 标记球中心
        cv2.circle(overlay, (ball_cx, ball_cy), 5, (0, 0, 255), -1)
        cv2.line(overlay, (ball_cx, ball_cy), (ball_cx, ball_cy + 30), (0, 0, 255), 2)

        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = BallDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
