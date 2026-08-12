#!/usr/bin/env python3
"""红蓝块 YOLO 检测 + RGB-D 深度融合测距节点。

核心思路:
  1. YOLOv8-seg 检测图像中红蓝块, 获取分割 mask
  2. 在 mask 区域内提取深度, 取深度主峰中值作为目标距离
  3. 反投影得到相机坐标系三维坐标
  4. 分别发布红块和蓝块的 3D 位置

订阅话题:
  /orbbec/color/image_raw  (sensor_msgs/Image)  - RGB 图像
  /orbbec/depth/image_raw  (sensor_msgs/Image)  - 深度图像 (float32, 单位 mm)
  /orbbec/camera_info       (sensor_msgs/CameraInfo) - 相机内参

发布话题:
  /perception/block_red_position   (geometry_msgs/PointStamped) - 红块 3D 位置 (m)
  /perception/block_blue_position  (geometry_msgs/PointStamped) - 蓝块 3D 位置 (m)
  /perception/block_overlay        (sensor_msgs/Image)          - 检测叠加图

坐标系:
  X: 前方为正
  Y: 右方为正
  Z: 上方为正
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def _default_model_path():
    """优先查找随包安装的模型，fallback 到工作区源码目录。"""
    if get_package_share_directory is not None:
        try:
            pkg = get_package_share_directory('block_perception')
            path = os.path.join(pkg, 'models', 'best.pt')
            if os.path.exists(path):
                return path
        except Exception:
            pass
    # Fallback: 工作区源码相对路径
    src_root = os.path.normpath(os.path.join(
        os.path.dirname(__file__), '..', '..', 'runs', 'segment',
        'blue_red', 'weights', 'best.pt'))
    return src_root


# ---- 深度估计参数 ----
MIN_VALID_DEPTH_POINTS = 20
DEPTH_HIST_BIN_MM = 50.0
DEPTH_CLUSTER_WINDOW_MM = 120.0
SEG_MASK_ERODE_KERNEL = 3


class BlockDistanceNode(Node):
    def __init__(self):
        super().__init__('block_distance_node')

        # ---- 参数 ----
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('red_class_name', 'red')
        self.declare_parameter('blue_class_name', 'blue')
        self.declare_parameter('min_depth_mm', 20.0)
        self.declare_parameter('max_depth_mm', 10000.0)
        self.declare_parameter('distance_alpha', 0.7)

        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f'加载 YOLO 模型: {model_path}')
        self.model = YOLO(model_path)
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value

        self.red_name = self.get_parameter('red_class_name').value.lower()
        self.blue_name = self.get_parameter('blue_class_name').value.lower()
        self.min_depth_mm = self.get_parameter('min_depth_mm').value
        self.max_depth_mm = self.get_parameter('max_depth_mm').value
        self.dist_alpha = self.get_parameter('distance_alpha').value

        # 平滑距离缓存
        self.smoothed_red_dist = None
        self.smoothed_blue_dist = None

        # 相机内参 (从 camera_info 话题动态获取)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # 数据缓存
        self.latest_color = None
        self.latest_depth = None
        self.bridge = CvBridge()

        # 订阅
        self.color_sub = self.create_subscription(
            Image, '/orbbec/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/orbbec/depth/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/orbbec/camera_info', self.info_callback, 10)

        # 发布
        self.red_pub = self.create_publisher(
            PointStamped, '/perception/block_red_position', 10)
        self.blue_pub = self.create_publisher(
            PointStamped, '/perception/block_blue_position', 10)
        self.overlay_pub = self.create_publisher(
            Image, '/perception/block_overlay', 10)

        # 融合定时器 (20Hz)
        self.timer = self.create_timer(0.05, self.fusion_callback)
        self.get_logger().info('红蓝块检测节点已启动')

    def color_callback(self, msg):
        self.latest_color = msg

    def depth_callback(self, msg):
        self.latest_depth = msg

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'内参已获取: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def _estimate_depth_from_mask(self, depth_map, mask):
        """从 mask 区域内估计深度 (mm), 返回深度中值"""
        valid_mask = (
            mask
            & (depth_map > self.min_depth_mm)
            & (depth_map < self.max_depth_mm)
        )

        ys, xs = np.nonzero(valid_mask)
        if xs.size < MIN_VALID_DEPTH_POINTS:
            return None, 0

        depths = depth_map[ys, xs].astype(np.float32)

        # 深度直方图聚类, 取主峰
        bin_ids = np.floor(depths / DEPTH_HIST_BIN_MM).astype(np.int32)
        unique_bins, counts = np.unique(bin_ids, return_counts=True)
        if unique_bins.size == 0:
            return None, 0

        peak_index = int(np.argmax(counts))
        peak_bin = int(unique_bins[peak_index])
        peak_center = (peak_bin + 0.5) * DEPTH_HIST_BIN_MM

        cluster_mask = np.abs(depths - peak_center) <= DEPTH_CLUSTER_WINDOW_MM
        cluster_count = int(np.count_nonzero(cluster_mask))

        if cluster_count >= MIN_VALID_DEPTH_POINTS:
            cluster_depths = depths[cluster_mask]
        else:
            cluster_depths = depths

        return float(np.median(cluster_depths)), int(cluster_depths.size)

    def _deproject_to_3d(self, u, v, depth_mm):
        """像素 + 深度 -> 相机坐标系三维坐标 (X前, Y右, Z上)"""
        z_std = float(depth_mm)
        x_std = (float(u) - self.cx) * z_std / self.fx
        y_std = (float(v) - self.cy) * z_std / self.fy

        # 转换为 X前, Y右, Z上
        x = z_std      # 前方
        y = x_std      # 右方
        z = -y_std     # 上方
        return x, y, z

    def _prepare_mask(self, raw_mask, img_h, img_w):
        """准备分割 mask: resize + 腐蚀边缘"""
        if raw_mask.shape != (img_h, img_w):
            raw_mask = cv2.resize(
                raw_mask.astype(np.float32),
                (img_w, img_h),
                interpolation=cv2.INTER_NEAREST,
            )

        mask = (raw_mask > 0.5).astype(np.uint8)

        if SEG_MASK_ERODE_KERNEL >= 3:
            k = SEG_MASK_ERODE_KERNEL
            if k % 2 == 0:
                k += 1
            kernel = np.ones((k, k), dtype=np.uint8)
            eroded = cv2.erode(mask, kernel, iterations=1)
            if np.count_nonzero(eroded) >= MIN_VALID_DEPTH_POINTS:
                mask = eroded

        return mask.astype(bool)

    def _smooth_distance(self, current_dist, smoothed_var):
        """EMA 平滑"""
        if smoothed_var[0] is None:
            smoothed_var[0] = current_dist
        else:
            smoothed_var[0] = (
                self.dist_alpha * smoothed_var[0]
                + (1.0 - self.dist_alpha) * current_dist
            )
        return smoothed_var[0]

    def fusion_callback(self):
        if self.latest_color is None or self.latest_depth is None:
            return
        if self.fx is None:
            return

        # 获取图像
        frame = self.bridge.imgmsg_to_cv2(self.latest_color, desired_encoding='bgr8')
        depth_map = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding='32FC1')
        img_h, img_w = frame.shape[:2]

        # YOLO 检测
        results = self.model.predict(
            frame, conf=self.conf_threshold, iou=self.iou_threshold, verbose=False,
            retina_masks=True,
        )
        result = results[0]
        overlay = result.plot()

        if result.boxes is None or len(result.boxes) == 0:
            cv2.putText(overlay, 'No block detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
            return

        # 解析检测结果
        boxes = result.boxes
        xyxy = boxes.xyxy.detach().cpu().numpy().astype(int)
        confs = boxes.conf.detach().cpu().numpy()
        cls_ids = boxes.cls.detach().cpu().numpy().astype(int)

        masks = None
        if result.masks is not None:
            masks = result.masks.data.detach().cpu().numpy()

        # 分别找红块和蓝块中置信度最高的
        red_best = None
        blue_best = None

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            x1 = max(0, min(x1, img_w - 1))
            y1 = max(0, min(y1, img_h - 1))
            x2 = max(x1 + 1, min(x2, img_w))
            y2 = max(y1 + 1, min(y2, img_h))

            cls_name = str(result.names.get(cls_ids[i], str(cls_ids[i]))).lower()
            conf = float(confs[i])

            # 准备 mask
            seg_mask = None
            if masks is not None and i < len(masks):
                seg_mask = self._prepare_mask(masks[i], img_h, img_w)
            else:
                seg_mask = np.zeros((img_h, img_w), dtype=bool)
                seg_mask[y1:y2, x1:x2] = True

            # 估计深度
            depth_mm, n_points = self._estimate_depth_from_mask(depth_map, seg_mask)

            if depth_mm is None:
                continue

            # 反投影
            cx_pixel = (x1 + x2) // 2
            cy_pixel = (y1 + y2) // 2
            x_mm, y_mm, z_mm = self._deproject_to_3d(cx_pixel, cy_pixel, depth_mm)

            detection = {
                'cls_name': cls_name,
                'conf': conf,
                'x_mm': x_mm,
                'y_mm': y_mm,
                'z_mm': z_mm,
                'depth_mm': depth_mm,
                'n_points': n_points,
                'bbox': (x1, y1, x2, y2),
            }

            if self.red_name in cls_name:
                if red_best is None or conf > red_best['conf']:
                    red_best = detection
            elif self.blue_name in cls_name:
                if blue_best is None or conf > blue_best['conf']:
                    blue_best = detection

        # 发布红块位置
        if red_best is not None:
            smoothed = self._smooth_distance(red_best['depth_mm'] / 1000.0, [self.smoothed_red_dist])
            self.smoothed_red_dist = smoothed
            self._publish_position(
                self.red_pub, red_best,
                smoothed * 1000.0,
                (0, 0, 255),  # 红色
                overlay,
            )

        # 发布蓝块位置
        if blue_best is not None:
            smoothed = self._smooth_distance(blue_best['depth_mm'] / 1000.0, [self.smoothed_blue_dist])
            self.smoothed_blue_dist = smoothed
            self._publish_position(
                self.blue_pub, blue_best,
                smoothed * 1000.0,
                (255, 0, 0),  # 蓝色
                overlay,
            )

        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))

    def _publish_position(self, publisher, det, depth_mm, color, overlay):
        """发布 3D 位置并绘制叠加信息"""
        x_mm, y_mm, z_mm = det['x_mm'], det['y_mm'], det['z_mm']

        # 使用平滑后的深度重新计算 XYZ
        cx_pixel = (det['bbox'][0] + det['bbox'][2]) // 2
        cy_pixel = (det['bbox'][1] + det['bbox'][3]) // 2
        x_mm, y_mm, z_mm = self._deproject_to_3d(cx_pixel, cy_pixel, depth_mm)

        # 发布 (单位转换为米)
        msg = PointStamped()
        msg.header.frame_id = 'orbbec_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = x_mm / 1000.0
        msg.point.y = y_mm / 1000.0
        msg.point.z = z_mm / 1000.0
        publisher.publish(msg)

        # 绘制叠加信息
        x1, y1, x2, y2 = det['bbox']
        label = f"{det['cls_name']} {det['conf']:.2f} d={depth_mm:.0f}mm"
        cv2.putText(overlay, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(overlay,
                    f'XYZ=({x_mm/1000:.2f}, {y_mm/1000:.2f}, {z_mm/1000:.2f})m',
                    (x1, y2 + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)


def main(args=None):
    rclpy.init(args=args)
    node = BlockDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
