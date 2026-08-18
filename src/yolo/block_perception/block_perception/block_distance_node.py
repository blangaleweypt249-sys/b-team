#!/usr/bin/env python3
"""块 YOLO 检测 + RGB-D 深度融合测距节点(不区分颜色,单块跟踪)。

核心思路:
  1. YOLOv8-seg 检测图像中的块, 获取分割 mask
  2. 在 mask 区域内提取深度, 取深度主峰中值作为目标距离
  3. 反投影得到相机坐标系三维坐标
  4. 转换到夹爪 TCP 坐标系 (P_G = P_C + t_G_C)
  5. 用 IOU 跟踪给每个检测框分配临时 ID, 记录距离历史
     只有"距离持续减小"的 ID 才确认为真正输出目标(机器人正在接近)
     目标消失后(连续失配)自动重新跟踪新的块
  6. 发布单一块 3D 位置(不区分红蓝)

订阅话题:
  /orbbec/color/image_raw  (sensor_msgs/Image)  - RGB 图像
  /orbbec/depth/image_raw  (sensor_msgs/Image)  - 深度图像 (float32, 单位 mm)
  /orbbec/camera_info       (sensor_msgs/CameraInfo) - 相机内参

发布话题:
  /perception/block_position  (geometry_msgs/PointStamped) - 当前跟踪块 3D 位置 (m, gripper frame)
  /perception/block_overlay   (sensor_msgs/Image)          - 检测叠加图

坐标系 (相机 / 夹爪一致):
  X: 前方为正
  Y: 左方为正
  Z: 上方为正

Orbbec -> 夹爪外参 (t_G_C, 固连常量, 尺子量出):
  默认 = (-102, -52.5, 28) mm -> (-0.102, -0.0525, 0.028) m
  即相机光心在 TCP 后方 102mm、右 52.5mm、上 28mm。
"""

import os
from dataclasses import dataclass, field
from typing import List, Tuple

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

# ---- IOU 跟踪参数 ----
# 思路:远处 YOLO 检出一堆候选,机器人向其中一个前进;
# 只把"距离持续减小"的 ID 确认为真正输出目标,其余丢弃。
IOU_THRESHOLD = 0.3              # IOU 匹配阈值,低于此值不视为同一目标
MAX_MISSED_FRAMES = 5            # 连续未匹配帧数上限,超过则删除 track
MIN_HISTORY_TO_CONFIRM = 3       # 确认目标所需的最小距离历史长度
MAX_HISTORY_LEN = 10             # 距离历史最大保留长度
MIN_APPROACH_DELTA_M = 0.10      # 整体接近量(米):首末差需 >= 此值才视为正在接近
MAX_REBOUND_M = 0.05             # 相邻帧距离允许的反弹量(米),超过则视为未在接近


@dataclass
class Track:
    """IOU 跟踪的临时目标,记录历史距离序列用于判定机器人是否正在接近。"""

    track_id: int
    bbox: Tuple[int, int, int, int]
    cls_name: str
    conf: float
    depth_mm: float
    x_mm: float
    y_mm: float
    z_mm: float
    # 距离历史序列(米),用于判断是否持续接近
    distance_history: List[float] = field(default_factory=list)
    # 连续未匹配帧数
    missed: int = 0
    # 是否已确认(距离持续减小)
    confirmed: bool = False


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
        # ---- Orbbec -> 夹爪外参 (t_G_C, 单位 m) ----
        #   默认 (-102, -52.5, 28) mm：相机在 TCP 后方 102mm、右 52.5mm、上 28mm
        self.declare_parameter('output_to_gripper', True)
        self.declare_parameter('t_g_c_x_m', -0.102)
        self.declare_parameter('t_g_c_y_m', -0.0525)
        self.declare_parameter('t_g_c_z_m', 0.028)

        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f'加载 YOLO 模型: {model_path}')
        self.model = YOLO(model_path)
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value

        self.min_depth_mm = self.get_parameter('min_depth_mm').value
        self.max_depth_mm = self.get_parameter('max_depth_mm').value
        self.dist_alpha = self.get_parameter('distance_alpha').value

        # Orbbec -> 夹爪外参
        self.output_to_gripper = self.get_parameter('output_to_gripper').value
        self.t_g_c_x_m = float(self.get_parameter('t_g_c_x_m').value)
        self.t_g_c_y_m = float(self.get_parameter('t_g_c_y_m').value)
        self.t_g_c_z_m = float(self.get_parameter('t_g_c_z_m').value)
        self.get_logger().info(
            f'Orbbec->夹爪: output_to_gripper={self.output_to_gripper}, '
            f't_G_C=({self.t_g_c_x_m*1000:.1f}, {self.t_g_c_y_m*1000:.1f}, {self.t_g_c_z_m*1000:.1f}) mm'
        )

        # IOU 跟踪状态
        self.tracks: List[Track] = []
        self._next_id = 0

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

        # 发布:单一块位置(不区分颜色,只跟踪一个块)
        self.block_pub = self.create_publisher(
            PointStamped, '/perception/block_position', 10)
        self.overlay_pub = self.create_publisher(
            Image, '/perception/block_overlay', 10)

        # 融合定时器 (20Hz)
        self.timer = self.create_timer(0.05, self.fusion_callback)
        self.get_logger().info('块检测节点已启动(不区分颜色,单块跟踪)')

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
        """像素 + 深度 -> 相机坐标系三维坐标 (X前, Y左, Z上)

        标准针孔: x_std=(u-cx)*z/fx 向右为正; y_std=(v-cy)*z/fy 向下为正。
        统一到机器人坐标系（+X前、+Y左、+Z上），与夹爪及 detection.py 一致。
        """
        z_std = float(depth_mm)
        x_std = (float(u) - self.cx) * z_std / self.fx
        y_std = (float(v) - self.cy) * z_std / self.fy

        x = z_std      # 前方
        y = -x_std     # 左方 (x_std 向右为正 -> 取反)
        z = -y_std     # 上方 (y_std 向下为正 -> 取反)
        return x, y, z

    def _to_gripper(self, x_m, y_m, z_m):
        """相机坐标 (m) -> 夹爪 TCP 坐标 (m): P_G = P_C + t_G_C。

        两者坐标系约定一致（+X前、+Y左、+Z上），无旋转，仅平移。
        """
        if not self.output_to_gripper:
            return x_m, y_m, z_m
        return (
            x_m + self.t_g_c_x_m,
            y_m + self.t_g_c_y_m,
            z_m + self.t_g_c_z_m,
        )

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

    # ==================== IOU 跟踪 ====================
    @staticmethod
    def _iou(b1: Tuple[int, int, int, int], b2: Tuple[int, int, int, int]) -> float:
        """计算两个 bbox (x1,y1,x2,y2) 的交并比。"""
        xi1 = max(b1[0], b2[0])
        yi1 = max(b1[1], b2[1])
        xi2 = min(b1[2], b2[2])
        yi2 = min(b1[3], b2[3])
        if xi2 <= xi1 or yi2 <= yi1:
            return 0.0
        inter = (xi2 - xi1) * (yi2 - yi1)
        a1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
        a2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
        return inter / (a1 + a2 - inter + 1e-6)

    def _match_tracks(
        self,
        tracks: List[Track],
        detections: List[dict],
    ):
        """贪心 IOU 匹配:返回 (matched, unmatched_tracks, unmatched_detections)。

        matched 为 (track_idx, det_idx) 列表。
        """
        n_t = len(tracks)
        n_d = len(detections)
        if n_t == 0:
            return [], [], list(range(n_d))
        if n_d == 0:
            return [], list(range(n_t)), []

        # IOU 矩阵
        iou_mat = np.zeros((n_t, n_d), dtype=np.float32)
        for ti in range(n_t):
            for di in range(n_d):
                iou_mat[ti, di] = self._iou(tracks[ti].bbox, detections[di]['bbox'])

        matched: List[Tuple[int, int]] = []
        used_t = set()
        used_d = set()
        # 贪心:每次取全局最大 IOU,达阈值则配对
        while len(used_t) < n_t and len(used_d) < n_d:
            idx = int(np.argmax(iou_mat))
            ti, di = divmod(idx, n_d)
            val = float(iou_mat[ti, di])
            if val < IOU_THRESHOLD:
                break
            matched.append((ti, di))
            used_t.add(ti)
            used_d.add(di)
            # 置零避免重复选取
            iou_mat[ti, :] = 0.0
            iou_mat[:, di] = 0.0

        unmatched_t = [i for i in range(n_t) if i not in used_t]
        unmatched_d = [i for i in range(n_d) if i not in used_d]
        return matched, unmatched_t, unmatched_d

    @staticmethod
    def _is_approaching(history: List[float]) -> bool:
        """判断距离序列是否表明机器人正在持续接近该目标。

        判定条件(需同时满足):
          1. 历史长度 >= MIN_HISTORY_TO_CONFIRM
          2. 末值 < 首值 - MIN_APPROACH_DELTA_M(整体在接近)
          3. 相邻帧距离增大不超过 MAX_REBOUND_M(无明显反弹)
        """
        if len(history) < MIN_HISTORY_TO_CONFIRM:
            return False
        seq = history[-MIN_HISTORY_TO_CONFIRM:]
        # 整体趋势:末值需比首值小至少 MIN_APPROACH_DELTA_M
        if seq[-1] > seq[0] - MIN_APPROACH_DELTA_M:
            return False
        # 过程中不允许大幅反弹
        for i in range(len(seq) - 1):
            if seq[i + 1] > seq[i] + MAX_REBOUND_M:
                return False
        return True

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

        # 解析所有检测框,收集为 detections 列表(红+蓝)
        detections: List[dict] = []
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes
            xyxy = boxes.xyxy.detach().cpu().numpy().astype(int)
            confs = boxes.conf.detach().cpu().numpy()
            cls_ids = boxes.cls.detach().cpu().numpy().astype(int)

            masks = None
            if result.masks is not None:
                masks = result.masks.data.detach().cpu().numpy()

            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                x1 = max(0, min(x1, img_w - 1))
                y1 = max(0, min(y1, img_h - 1))
                x2 = max(x1 + 1, min(x2, img_w))
                y2 = max(y1 + 1, min(y2, img_h))

                cls_name = str(result.names.get(cls_ids[i], str(cls_ids[i]))).lower()
                conf = float(confs[i])

                # 模型只检测块,所有检测框都视为候选块(不区分颜色)

                # 准备 mask
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

                detections.append({
                    'cls_name': cls_name,
                    'conf': conf,
                    'x_mm': x_mm,
                    'y_mm': y_mm,
                    'z_mm': z_mm,
                    'depth_mm': depth_mm,
                    'n_points': n_points,
                    'bbox': (x1, y1, x2, y2),
                })

        # ---- IOU 跟踪更新 ----
        matched, unmatched_t, unmatched_d = self._match_tracks(self.tracks, detections)

        # 1) 已匹配:更新 track 状态并判定是否确认
        for ti, di in matched:
            t = self.tracks[ti]
            d = detections[di]
            t.bbox = d['bbox']
            t.cls_name = d['cls_name']
            t.conf = d['conf']
            t.depth_mm = d['depth_mm']
            t.x_mm = d['x_mm']
            t.y_mm = d['y_mm']
            t.z_mm = d['z_mm']
            t.distance_history.append(d['depth_mm'] / 1000.0)
            if len(t.distance_history) > MAX_HISTORY_LEN:
                t.distance_history.pop(0)
            t.missed = 0
            t.confirmed = self._is_approaching(t.distance_history)

        # 2) 未匹配的 track:missed +1(若曾确认则撤销)
        for ti in unmatched_t:
            self.tracks[ti].missed += 1
            self.tracks[ti].confirmed = False

        # 3) 未匹配的检测框:新建 track
        for di in unmatched_d:
            d = detections[di]
            self._next_id += 1
            self.tracks.append(Track(
                track_id=self._next_id,
                bbox=d['bbox'],
                cls_name=d['cls_name'],
                conf=d['conf'],
                depth_mm=d['depth_mm'],
                x_mm=d['x_mm'],
                y_mm=d['y_mm'],
                z_mm=d['z_mm'],
                distance_history=[d['depth_mm'] / 1000.0],
            ))

        # 4) 删除超时 track(连续 MAX_MISSED_FRAMES 未匹配)
        self.tracks = [t for t in self.tracks if t.missed <= MAX_MISSED_FRAMES]

        # ---- 绘制所有 track(未确认灰色,已确认绿色) ----
        for t in self.tracks:
            color = (0, 255, 0) if t.confirmed else (128, 128, 128)
            thickness = 3 if t.confirmed else 1
            cv2.rectangle(overlay, (t.bbox[0], t.bbox[1]), (t.bbox[2], t.bbox[3]),
                          color, thickness)
            label = f"ID{t.track_id} {t.cls_name} d={t.depth_mm:.0f}mm"
            if t.confirmed:
                label += " [确认]"
            cv2.putText(overlay, label, (t.bbox[0], t.bbox[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # ---- 发布:已确认 track 中选距离最近的一个(不区分颜色) ----
        confirmed_tracks = [t for t in self.tracks if t.confirmed]

        if confirmed_tracks:
            best = min(confirmed_tracks, key=lambda t: t.depth_mm)
            out_depth_mm = self._median_recent_depth(best)
            self._publish_position(self.block_pub, best, out_depth_mm, (0, 255, 0), overlay)

        # 状态提示
        n_confirmed = len(confirmed_tracks)
        cv2.putText(overlay, f'tracks={len(self.tracks)} confirmed={n_confirmed}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))

    @staticmethod
    def _median_recent_depth(track: Track) -> float:
        """取 track 最近 3 个距离的中值作为输出深度(mm),抗深度噪声。"""
        recent = track.distance_history[-3:]
        return float(np.median(recent)) * 1000.0

    def _publish_position(self, publisher, track: Track, depth_mm, color, overlay):
        """发布 3D 位置并绘制叠加信息(默认夹爪 TCP 坐标系)。

        参数 track: 已确认的 Track 对象;depth_mm: 经中值滤波后的输出深度(mm)。
        """
        # 使用输出深度反投影 XYZ(已为 +X前、+Y左、+Z上)
        cx_pixel = (track.bbox[0] + track.bbox[2]) // 2
        cy_pixel = (track.bbox[1] + track.bbox[3]) // 2
        x_mm, y_mm, z_mm = self._deproject_to_3d(cx_pixel, cy_pixel, depth_mm)

        # 转换为夹爪 TCP 坐标 (单位 m)
        x_m = x_mm / 1000.0
        y_m = y_mm / 1000.0
        z_m = z_mm / 1000.0
        gx_m, gy_m, gz_m = self._to_gripper(x_m, y_m, z_m)

        # 发布 (单位 m)
        msg = PointStamped()
        msg.header.frame_id = 'gripper' if self.output_to_gripper else 'orbbec_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = gx_m
        msg.point.y = gy_m
        msg.point.z = gz_m
        publisher.publish(msg)

        # 绘制叠加信息
        x1, y1, x2, y2 = track.bbox
        frame_str = 'gripper' if self.output_to_gripper else 'cam'
        label = (
            f"{track.cls_name} {track.conf:.2f} d={depth_mm:.0f}mm  "
            f"{frame_str}:({gx_m*1000:.0f},{gy_m*1000:.0f},{gz_m*1000:.0f})mm"
        )
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
