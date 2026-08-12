"""

    X：相机前方为正
    Y：相机左方为正
    Z：相机上方为正
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import torch
from ultralytics import YOLO


# ==============================================================================
# 0. 路径
# ==============================================================================

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 优先从脚本所在目录查找 utils.py
sys.path.insert(0, _SCRIPT_DIR)

from utils import frame_to_bgr_image  # noqa: E402

from pyorbbecsdk import (  # noqa: E402
    AlignFilter,
    Config,
    OBFormat,
    OBSensorType,
    OBStreamType,
    Pipeline,
)

import yaml  # noqa: E402


# ==============================================================================
# 1. 默认配置
# ==============================================================================

# 用户训练的蓝红目标检测模型
DEFAULT_MODEL = os.path.join(
    _SCRIPT_DIR,
    "blue_red",
    "runs",
    "seg_train",
    "weights",
    "best.pt",
)

YOLO_IMAGE_SIZE = 640
CONF_THRESHOLD = 0.50
NMS_IOU_THRESHOLD = 0.45
MAX_DETECTIONS = 20

# device：
#   cpu  -> CPU (不依赖CUDA, 可移植到无NVIDIA显卡的电脑)
#   auto -> 有 CUDA 时用 0，否则 CPU
#   0    -> 第一张 NVIDIA GPU
DEFAULT_DEVICE = "cpu"

# ------------------------------------------------------------------------------
# Camera resolution
#
# None = 使用设备默认 Profile
# ------------------------------------------------------------------------------

# 标定配置为 1280x720
COLOR_CAMERA_WIDTH = 1280
COLOR_CAMERA_HEIGHT = 720
DEPTH_CAMERA_WIDTH = None
DEPTH_CAMERA_HEIGHT = None

# ------------------------------------------------------------------------------
# Depth 有效范围，单位 mm
# ------------------------------------------------------------------------------
MIN_DEPTH_MM = 20.0
MAX_DEPTH_MM = 10000.0
# ------------------------------------------------------------------------------
# Detect 模型中心 ROI
#
# 0.50 表示只取 bbox 中心 50% x 50% 区域做深度估计，减少背景干扰。
# ------------------------------------------------------------------------------
CENTER_ROI_RATIO = 0.50
# ------------------------------------------------------------------------------
# Seg 模型 Mask 边缘腐蚀
#
# 分割 Mask 边缘常混入背景深度。
# 这里轻微向内腐蚀，提高深度稳定性。
#
# 0 = 不腐蚀
# 3 = 使用 3x3 kernel 腐蚀一次
# ------------------------------------------------------------------------------
SEG_MASK_ERODE_KERNEL = 3
# ------------------------------------------------------------------------------
# 深度主峰聚类
# ------------------------------------------------------------------------------

# 直方图 bin 宽度：50 mm
DEPTH_HIST_BIN_MM = 50.0

# 只保留主峰中心 +/- 120 mm 的深度点
DEPTH_CLUSTER_WINDOW_MM = 120.0

# 一个区域至少需要多少个有效深度像素
MIN_VALID_DEPTH_POINTS = 20


# ------------------------------------------------------------------------------
# 亮度增强 
# ------------------------------------------------------------------------------

BRIGHTNESS_LOW = 90.0
CLAHE_CLIP_LIMIT = 2.0


# ------------------------------------------------------------------------------
# 相机 -> 夹爪 转换默认配置
#
# t_G_C：相机光心在 {G}（当前夹爪 TCP）中的位置，单位 mm。
#         +X 前、+Y 左、+Z 上。尺子量出，固连常量。
#         默认值 (0, 21, 27.8)：相机在 TCP 左 21mm、上 27.8mm、前后对齐。
#         相机与夹爪固连，P_G = P_C + t_G_C 即物体相对当前 TCP 的位置。
# ------------------------------------------------------------------------------
DEFAULT_T_G_C = (0.0, 21.0, 27.8)


# ------------------------------------------------------------------------------
# GUI
# ------------------------------------------------------------------------------

WINDOW_NAME = "YOLO + Orbbec Gemini2 3D"
WINDOW_INITIAL_WIDTH = 960
WINDOW_INITIAL_HEIGHT = 540

FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.50
FONT_THICKNESS = 1

ESC_KEY = 27

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)

PALETTE = [
    (255, 255, 255),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255),
    (128, 128, 0),
    (128, 0, 128),
    (0, 128, 128),
    (128, 128, 128),
]

# ==============================================================================
# 2. 数据结构
# ==============================================================================

@dataclass
class CameraIntrinsics:
    """RGB 针孔相机内参。"""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

@dataclass
class DepthEstimate:
    """一个目标区域的深度估计结果。"""

    # 最终选中的代表像素
    u: int
    v: int

    # 目标深度中位数
    depth_mm: float

    # 区域中的有效深度点数量
    valid_points: int

    # 主深度簇点数量
    cluster_points: int


@dataclass
class Detection3D:
    """
    一个完整的三维检测结果。

    后续接 ROS 2 时，可以将它转换成自定义 msg 或 Pose/Point 消息。
    """

    class_id: int
    class_name: str
    confidence: float

    # x1, y1, x2, y2
    bbox: Tuple[int, int, int, int]

    # 用于反投影的像素坐标
    pixel_u: Optional[int]
    pixel_v: Optional[int]

    # 深度
    depth_mm: Optional[float]

    # 相机坐标系三维坐标
    x_mm: Optional[float]
    y_mm: Optional[float]
    z_mm: Optional[float]

    # seg_mask / center_roi
    depth_source: str

    # 深度统计信息
    valid_depth_points: int = 0
    cluster_depth_points: int = 0

    # 夹爪坐标系三维坐标（相对当前 TCP = P_C + t_G_C）
    gx_mm: Optional[float] = None
    gy_mm: Optional[float] = None
    gz_mm: Optional[float] = None


# ==============================================================================
# 3. 运行环境 / GPU / GUI 调试
# ==============================================================================

def resolve_device(requested_device: str) -> str:
    """
    把用户指定的 device 解析成 Ultralytics 可使用的字符串。

    auto：
        CUDA 可用 -> "0"
        CUDA 不可用 -> "cpu"
    """

    requested = str(requested_device).strip().lower()

    if requested == "auto":
        return "0" if torch.cuda.is_available() else "cpu"

    return requested


def print_runtime_info(device: str) -> None:
    """打印 PyTorch / CUDA / GPU / DISPLAY 信息。"""

    print("=" * 80)
    print("[Runtime] Environment")
    print("=" * 80)

    print(f"[Runtime] Python executable : {sys.executable}")
    print(f"[Runtime] OpenCV            : {cv2.__version__}")
    print(f"[Runtime] PyTorch           : {torch.__version__}")
    print(f"[Runtime] Torch CUDA build  : {torch.version.cuda}")
    print(f"[Runtime] CUDA available    : {torch.cuda.is_available()}")
    print(f"[Runtime] DISPLAY           : {os.environ.get('DISPLAY', '<empty>')}")
    print(f"[Runtime] Selected device   : {device}")

    if torch.cuda.is_available():
        try:
            print(f"[Runtime] GPU 0             : {torch.cuda.get_device_name(0)}")
        except Exception as error:
            print(f"[Runtime] GPU name failed   : {error}")

    print()


def create_status_image(
    text_lines: List[str],
    width: int = WINDOW_INITIAL_WIDTH,
    height: int = WINDOW_INITIAL_HEIGHT,
) -> np.ndarray:
    """创建简单状态画面，在模型/相机初始化时也能看到 GUI。"""

    image = np.zeros((height, width, 3), dtype=np.uint8)

    title = "YOLO + Orbbec Gemini2"
    cv2.putText(
        image,
        title,
        (40, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        GREEN,
        2,
        cv2.LINE_AA,
    )

    y = 130
    for line in text_lines:
        cv2.putText(
            image,
            line,
            (40, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            WHITE,
            1,
            cv2.LINE_AA,
        )
        y += 38

    return image


def init_display(enabled: bool) -> bool:
    """
    尽早创建 OpenCV 窗口。

    这样可以明确区分：
        - GUI 问题
        - YOLO 第一次推理问题

    注意：DISPLAY 有值并不等于 X11/Qt 一定能正常显示。
    """

    if not enabled:
        print("[GUI] Display disabled by --no_display")
        return False

    print("[GUI] Creating OpenCV window...")

    try:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            WINDOW_NAME,
            WINDOW_INITIAL_WIDTH,
            WINDOW_INITIAL_HEIGHT,
        )

        status = create_status_image(
            [
                "OpenCV window created.",
                "Loading YOLO model...",
            ]
        )

        cv2.imshow(WINDOW_NAME, status)
        cv2.waitKey(1)

        print("[GUI] OpenCV window created successfully.")
        return True

    except cv2.error as error:
        print("[GUI] OpenCV GUI initialization failed:")
        print(error)
        print("[GUI] Re-run with --no_display for headless mode.")
        return False


def show_status(enabled: bool, lines: List[str]) -> None:
    """更新初始化阶段的状态窗口。"""

    if not enabled:
        return

    image = create_status_image(lines)
    cv2.imshow(WINDOW_NAME, image)
    cv2.waitKey(1)


def yolo_warmup(
    model: YOLO,
    device: str,
    imgsz: int,
    conf: float,
    iou: float,
    max_det: int,
    display_enabled: bool,
) -> None:
    """
    在打开相机主循环之前做一次 YOLO warm-up。

    作用：
    1. 第一次 CUDA context / cuDNN 初始化提前完成。
    2. 如果程序卡在第一次 YOLO 推理，会明确停在这里。
    3. 主循环第一次真实帧不会承担全部初始化开销。
    """

    print("=" * 80)
    print("[YOLO] Warm-up")
    print("=" * 80)
    print(f"[YOLO] Warm-up device : {device}")
    print(f"[YOLO] Warm-up imgsz  : {imgsz}")
    print("[YOLO] Warm-up starting...")

    show_status(
        display_enabled,
        [
            f"YOLO model loaded.",
            f"Device: {device}",
            "Running first inference warm-up...",
        ],
    )

    dummy_image = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)

    start_time = time.perf_counter()

    # Ultralytics 自己会进入 inference mode。
    # 这里使用 predict 的真实路径进行 warm-up，避免主循环第一次才初始化。
    _ = model.predict(
        source=dummy_image,
        imgsz=imgsz,
        rect=False,
        conf=conf,
        iou=iou,
        max_det=max_det,
        device=device,
        verbose=True,
        retina_masks=True,
    )

    # 对 CUDA 来说 synchronize 能保证计时包含实际 kernel 执行完成。
    if device != "cpu" and torch.cuda.is_available():
        torch.cuda.synchronize()

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0

    print(f"[YOLO] Warm-up finished: {elapsed_ms:.1f} ms")
    print()

    show_status(
        display_enabled,
        [
            "YOLO warm-up finished.",
            f"Warm-up: {elapsed_ms:.1f} ms",
            "Starting Orbbec camera...",
        ],
    )


# ==============================================================================
# 4. GUI 绘制辅助
# ==============================================================================

def draw_multiline_label(
    image: np.ndarray,
    lines: List[str],
    x: int,
    y: int,
    color: Tuple[int, int, int],
) -> None:
    """绘制多行文字，并为每行绘制黑色背景。"""

    if not lines:
        return

    image_h, image_w = image.shape[:2]
    line_height = 18

    x = max(2, min(x, image_w - 5))
    y = max(line_height, min(y, image_h - 5))

    for index, text in enumerate(lines):
        text_y = y + index * line_height

        (text_width, text_height), baseline = cv2.getTextSize(
            text,
            FONT_FACE,
            FONT_SCALE,
            FONT_THICKNESS,
        )

        if text_y + baseline >= image_h:
            break

        cv2.rectangle(
            image,
            (x, text_y - text_height - 3),
            (
                min(x + text_width + 4, image_w - 1),
                text_y + baseline + 2,
            ),
            BLACK,
            cv2.FILLED,
        )

        cv2.putText(
            image,
            text,
            (x + 2, text_y),
            FONT_FACE,
            FONT_SCALE,
            color,
            FONT_THICKNESS,
            cv2.LINE_AA,
        )


def draw_segmentation_mask(
    image: np.ndarray,
    mask: np.ndarray,
    color: Tuple[int, int, int],
    alpha: float = 0.30,
) -> None:
    """在原图上半透明绘制实例分割 Mask。"""

    if mask is None:
        return

    mask_bool = mask.astype(bool)

    if not np.any(mask_bool):
        return

    color_array = np.array(color, dtype=np.float32)
    original_pixels = image[mask_bool].astype(np.float32)

    blended_pixels = (
        original_pixels * (1.0 - alpha)
        + color_array * alpha
    )

    image[mask_bool] = np.clip(
        blended_pixels,
        0,
        255,
    ).astype(np.uint8)


# ==============================================================================
# 5. Orbbec Depth Frame -> 深度矩阵
# ==============================================================================

def extract_depth_map(depth_frame) -> Optional[np.ndarray]:
    """
    将 Orbbec depth frame 转成二维深度矩阵。

    返回：
        shape = (H, W)
        dtype = float32
        单位 = mm
        无效深度 = 0
    """

    try:
        raw_depth = np.frombuffer(
            depth_frame.get_data(),
            dtype=np.uint16,
        ).reshape(
            depth_frame.get_height(),
            depth_frame.get_width(),
        )

    except ValueError:
        print("[Depth] Failed to reshape depth frame.")
        return None

    # raw depth * depth scale -> mm
    depth_mm = (
        raw_depth.astype(np.float32)
        * float(depth_frame.get_depth_scale())
    )

    valid_mask = (
        (depth_mm > MIN_DEPTH_MM)
        & (depth_mm < MAX_DEPTH_MM)
    )

    depth_mm = np.where(valid_mask, depth_mm, 0.0)

    return depth_mm.astype(np.float32)


# ==============================================================================
# 6. Detect 模型：中心 ROI
# ==============================================================================

def create_center_roi_mask(
    image_shape: Tuple[int, int],
    bbox: Tuple[int, int, int, int],
    ratio: float = CENTER_ROI_RATIO,
) -> np.ndarray:
    """
    在 Bounding Box 中创建中心 ROI Mask。

    ratio=0.5：
        只使用 bbox 中间 50% x 50% 区域测深。
    """

    image_h, image_w = image_shape
    x1, y1, x2, y2 = bbox

    box_width = max(1, x2 - x1)
    box_height = max(1, y2 - y1)

    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0

    roi_width = box_width * ratio
    roi_height = box_height * ratio

    roi_x1 = int(round(center_x - roi_width / 2.0))
    roi_y1 = int(round(center_y - roi_height / 2.0))
    roi_x2 = int(round(center_x + roi_width / 2.0))
    roi_y2 = int(round(center_y + roi_height / 2.0))

    roi_x1 = max(0, min(roi_x1, image_w - 1))
    roi_y1 = max(0, min(roi_y1, image_h - 1))
    roi_x2 = max(roi_x1 + 1, min(roi_x2, image_w))
    roi_y2 = max(roi_y1 + 1, min(roi_y2, image_h))

    mask = np.zeros((image_h, image_w), dtype=bool)
    mask[roi_y1:roi_y2, roi_x1:roi_x2] = True

    return mask


# ==============================================================================
# 7. Seg 模型：Mask 处理
# ==============================================================================

def prepare_segmentation_mask(
    raw_mask: np.ndarray,
    image_shape: Tuple[int, int],
) -> np.ndarray:
    """
    将 Ultralytics Mask 转成与 RGB/Depth 相同大小的 bool mask。

    retina_masks=True 时通常已经是原图尺寸；
    这里仍保留 resize 作为鲁棒性保护。

    另外会轻微腐蚀边缘，减少背景深度混入。
    """

    image_h, image_w = image_shape

    if raw_mask.shape != (image_h, image_w):
        raw_mask = cv2.resize(
            raw_mask.astype(np.float32),
            (image_w, image_h),
            interpolation=cv2.INTER_NEAREST,
        )

    mask = (raw_mask > 0.5).astype(np.uint8)

    if SEG_MASK_ERODE_KERNEL >= 3:
        kernel_size = int(SEG_MASK_ERODE_KERNEL)

        # 强制使用奇数 kernel
        if kernel_size % 2 == 0:
            kernel_size += 1

        kernel = np.ones(
            (kernel_size, kernel_size),
            dtype=np.uint8,
        )

        eroded = cv2.erode(mask, kernel, iterations=1)

        # 如果目标本身很小，腐蚀可能把 mask 全部吃掉。
        # 这时回退到原始 mask。
        if np.count_nonzero(eroded) >= MIN_VALID_DEPTH_POINTS:
            mask = eroded

    return mask.astype(bool)


# ==============================================================================
# 8. 深度主峰聚类
# ==============================================================================

def estimate_depth_from_region(
    depth_map: np.ndarray,
    region_mask: np.ndarray,
) -> Optional[DepthEstimate]:
    """
    从某个目标区域中估计目标深度。

    流程：
        region mask
            -> 有效深度点
            -> 深度直方图
            -> 最大主峰
            -> 主峰附近 +/- window
            -> 深度中位数
            -> 选取一个真实有效的代表像素 (u, v)

    相比“整个 bbox 直接 median”，这种方法能明显减少背景干扰。
    """

    valid_mask = (
        region_mask
        & (depth_map > MIN_DEPTH_MM)
        & (depth_map < MAX_DEPTH_MM)
    )

    ys, xs = np.nonzero(valid_mask)

    if xs.size < MIN_VALID_DEPTH_POINTS:
        return None

    depths = depth_map[ys, xs].astype(np.float32)

    # --------------------------------------------------------------------------
    # 深度直方图
    # --------------------------------------------------------------------------

    bin_ids = np.floor(
        depths / DEPTH_HIST_BIN_MM
    ).astype(np.int32)

    unique_bins, counts = np.unique(
        bin_ids,
        return_counts=True,
    )

    if unique_bins.size == 0:
        return None

    peak_index = int(np.argmax(counts))
    peak_bin = int(unique_bins[peak_index])

    peak_depth_center = (
        peak_bin + 0.5
    ) * DEPTH_HIST_BIN_MM

    cluster_mask = (
        np.abs(depths - peak_depth_center)
        <= DEPTH_CLUSTER_WINDOW_MM
    )

    cluster_count = int(np.count_nonzero(cluster_mask))

    if cluster_count < MIN_VALID_DEPTH_POINTS:
        # 主峰点太少时退化为全部有效区域。
        cluster_depths = depths
        cluster_xs = xs
        cluster_ys = ys
    else:
        cluster_depths = depths[cluster_mask]
        cluster_xs = xs[cluster_mask]
        cluster_ys = ys[cluster_mask]

    if cluster_depths.size == 0:
        return None

    depth_mm = float(np.median(cluster_depths))

    # --------------------------------------------------------------------------
    # 代表像素：
    #
    # 先计算簇内像素坐标的中位数，再在真实簇像素中寻找离该位置最近的点。
    # 这样最终的 (u, v) 一定真的是目标深度簇中的有效像素，
    # 而不是一个可能落在空洞/背景上的“虚构中心点”。
    # --------------------------------------------------------------------------

    median_u = float(np.median(cluster_xs))
    median_v = float(np.median(cluster_ys))

    distance_sq = (
        (cluster_xs.astype(np.float32) - median_u) ** 2
        + (cluster_ys.astype(np.float32) - median_v) ** 2
    )

    representative_index = int(np.argmin(distance_sq))

    u = int(cluster_xs[representative_index])
    v = int(cluster_ys[representative_index])

    return DepthEstimate(
        u=u,
        v=v,
        depth_mm=depth_mm,
        valid_points=int(depths.size),
        cluster_points=int(cluster_depths.size),
    )


# ==============================================================================
# 9. RGB 相机内参
# ==============================================================================

def get_rgb_intrinsics(
    pipeline: Pipeline,
    image_shape: Tuple[int, int],
) -> CameraIntrinsics:
    """从 Orbbec SDK 获取当前实际工作的 RGB 相机内参。"""

    image_h, image_w = image_shape

    camera_param = pipeline.get_camera_param()
    rgb_intrinsic = camera_param.rgb_intrinsic

    original_width = int(rgb_intrinsic.width)
    original_height = int(rgb_intrinsic.height)

    if original_width <= 0 or original_height <= 0:
        raise RuntimeError("Invalid RGB camera intrinsic resolution.")

    scale_x = image_w / original_width
    scale_y = image_h / original_height

    fx = float(rgb_intrinsic.fx) * scale_x
    fy = float(rgb_intrinsic.fy) * scale_y
    cx = float(rgb_intrinsic.cx) * scale_x
    cy = float(rgb_intrinsic.cy) * scale_y

    return CameraIntrinsics(
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        width=image_w,
        height=image_h,
    )


# ==============================================================================
# 10. 像素 + 深度 -> 相机三维坐标
# ==============================================================================

def deproject_pixel_to_3d(
    u: float,
    v: float,
    depth_mm: float,
    intrinsics: CameraIntrinsics,
) -> Tuple[float, float, float]:
    """
    标准针孔模型反投影 + 坐标系转换。

    标准相机坐标系: x_right, y_down, z_forward
  
    """

    z_std = float(depth_mm)

    x_std = (
        (float(u) - intrinsics.cx)
        * z_std
        / intrinsics.fx
    )

    y_std = (
        (float(v) - intrinsics.cy)
        * z_std
        / intrinsics.fy
    )

    # X前, Y左, Z上
    x = z_std    # 前方
    y = -x_std   # 左方
    z = -y_std   # 上方

    return x, y, z


# ==============================================================================
# 10.5 相机 -> 夹爪 转换辅助
# ==============================================================================

def load_t_g_c_from_yaml(path: str) -> Tuple[float, float, float]:
    """从 camera_to_gripper.yaml 读取 t_G_C_mm 字段。"""
    with open(path) as f:
        data = yaml.safe_load(f)
    t = data["t_G_C_mm"]
    return float(t["x"]), float(t["y"]), float(t["z"])


def parse_t_g_c(text: str) -> Tuple[float, float, float]:
    """解析 'cx,cy,cz' 字符串。"""
    parts = [float(v.strip()) for v in text.split(",")]
    if len(parts) != 3:
        raise ValueError(f"t_G_C 需要 'cx,cy,cz'，收到: {text}")
    return parts[0], parts[1], parts[2]


def to_gripper(
    x_c: Optional[float],
    y_c: Optional[float],
    z_c: Optional[float],
    t_g_c: Tuple[float, float, float],
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """相机坐标 P_C -> 夹爪坐标 P_G = P_C + t_G_C（相对当前 TCP）。

    相机与夹爪固连，t_G_C 为固定平移常量。任一输入为 None 时返回全 None。
    """
    if x_c is None or y_c is None or z_c is None:
        return None, None, None
    return x_c + t_g_c[0], y_c + t_g_c[1], z_c + t_g_c[2]


# ==============================================================================
# 11. YOLO Result + Depth -> Detection3D
# ==============================================================================

def get_class_name(names, class_id: int) -> str:
    """兼容 result.names 为 dict 或 list/tuple 的情况。"""

    if isinstance(names, dict):
        return str(names.get(class_id, class_id))

    if 0 <= class_id < len(names):
        return str(names[class_id])

    return str(class_id)


def process_yolo_result(
    image: np.ndarray,
    depth_map: np.ndarray,
    result,
    intrinsics: CameraIntrinsics,
    t_g_c: Optional[Tuple[float, float, float]] = None,
) -> Tuple[np.ndarray, List[Detection3D]]:
    """
    YOLO 结果 + 对齐后的 Depth -> 三维检测结果。

    Detect：
        bbox -> center ROI -> depth cluster -> XYZ

    Segment：
        instance mask -> erosion -> depth cluster -> XYZ
    """

    output_image = image.copy()
    detections_3d: List[Detection3D] = []

    image_h, image_w = image.shape[:2]

    boxes = result.boxes

    if boxes is None or len(boxes) == 0:
        return output_image, detections_3d

    xyxy_array = (
        boxes.xyxy
        .detach()
        .cpu()
        .numpy()
    )

    # 对 YOLO + Ultralytics 高层 API：
    # boxes.conf 就是最终检测置信度，不要再额外乘一次。
    confidence_array = (
        boxes.conf
        .detach()
        .cpu()
        .numpy()
    )

    class_id_array = (
        boxes.cls
        .detach()
        .cpu()
        .numpy()
        .astype(np.int32)
    )

    masks_array = None

    if result.masks is not None:
        masks_array = (
            result.masks.data
            .detach()
            .cpu()
            .numpy()
        )

    for index in range(len(xyxy_array)):
        x1, y1, x2, y2 = (
            xyxy_array[index]
            .astype(np.int32)
            .tolist()
        )

        x1 = max(0, min(x1, image_w - 1))
        y1 = max(0, min(y1, image_h - 1))
        x2 = max(x1 + 1, min(x2, image_w))
        y2 = max(y1 + 1, min(y2, image_h))

        bbox = (x1, y1, x2, y2)

        confidence = float(confidence_array[index])
        class_id = int(class_id_array[index])
        class_name = get_class_name(result.names, class_id)

        color = PALETTE[class_id % len(PALETTE)]

        segmentation_mask = None

        # ----------------------------------------------------------------------
        # Seg 模型：优先使用 instance mask
        # ----------------------------------------------------------------------
        if masks_array is not None and index < len(masks_array):
            segmentation_mask = prepare_segmentation_mask(
                masks_array[index],
                (image_h, image_w),
            )

            if np.count_nonzero(segmentation_mask) >= MIN_VALID_DEPTH_POINTS:
                depth_region = segmentation_mask
                depth_source = "seg_mask"
            else:
                depth_region = create_center_roi_mask(
                    (image_h, image_w),
                    bbox,
                )
                depth_source = "center_roi"

        # ----------------------------------------------------------------------
        # Detect 模型：中心 ROI
        # ----------------------------------------------------------------------
        else:
            depth_region = create_center_roi_mask(
                (image_h, image_w),
                bbox,
            )
            depth_source = "center_roi"

        depth_estimate = estimate_depth_from_region(
            depth_map,
            depth_region,
        )

        if depth_estimate is not None:
            u = depth_estimate.u
            v = depth_estimate.v
            depth_mm = depth_estimate.depth_mm

            x_mm, y_mm, z_mm = deproject_pixel_to_3d(
                u,
                v,
                depth_mm,
                intrinsics,
            )

            valid_points = depth_estimate.valid_points
            cluster_points = depth_estimate.cluster_points

        else:
            u = None
            v = None
            depth_mm = None
            x_mm = None
            y_mm = None
            z_mm = None
            valid_points = 0
            cluster_points = 0

        # 相机坐标 -> 夹爪坐标（P_G = P_C + t_G_C）
        if t_g_c is not None:
            gx_mm, gy_mm, gz_mm = to_gripper(x_mm, y_mm, z_mm, t_g_c)
        else:
            gx_mm = gy_mm = gz_mm = None

        detection = Detection3D(
            class_id=class_id,
            class_name=class_name,
            confidence=confidence,
            bbox=bbox,
            pixel_u=u,
            pixel_v=v,
            depth_mm=depth_mm,
            x_mm=x_mm,
            y_mm=y_mm,
            z_mm=z_mm,
            depth_source=depth_source,
            valid_depth_points=valid_points,
            cluster_depth_points=cluster_points,
            gx_mm=gx_mm,
            gy_mm=gy_mm,
            gz_mm=gz_mm,
        )

        detections_3d.append(detection)

        # ----------------------------------------------------------------------
        # 绘制 Mask
        # ----------------------------------------------------------------------
        if segmentation_mask is not None:
            draw_segmentation_mask(
                output_image,
                segmentation_mask,
                color,
            )

        # Bounding Box
        cv2.rectangle(
            output_image,
            (x1, y1),
            (x2, y2),
            color,
            2,
        )

        # 最终用于 XYZ 的像素点
        if u is not None and v is not None:
            cv2.circle(
                output_image,
                (u, v),
                5,
                RED,
                -1,
            )

        label_lines = [
            f"{class_name}  conf={confidence:.2f}",
        ]

        if depth_mm is not None:
            label_lines.append(
                f"depth={depth_mm:.0f} mm"
            )

            label_lines.append(
                "XYZ="
                f"({x_mm:.0f}, {y_mm:.0f}, {z_mm:.0f}) mm"
            )

            # 夹爪坐标系（相对当前 TCP）
            if gx_mm is not None:
                label_lines.append(
                    "P_G="
                    f"({gx_mm:.0f}, {gy_mm:.0f}, {gz_mm:.0f}) mm"
                )

            label_lines.append(
                f"uv=({u},{v})  {depth_source}"
            )
        else:
            label_lines.append("depth=N/A")
            label_lines.append("XYZ=N/A")

        # 标签优先放在框上方；如果顶部空间不足则放框内。
        label_x = x1
        label_y = max(20, y1 - 5)

        draw_multiline_label(
            output_image,
            label_lines,
            label_x,
            label_y,
            color,
        )

    return output_image, detections_3d


# ==============================================================================
# 12. Camera stream profile
# ==============================================================================

def find_profile(
    profiles,
    width: int,
    height: int,
    fmt=None,
):
    """寻找指定宽高/格式的 stream profile。"""

    for profile in profiles:
        if (
            profile.get_width() == width
            and profile.get_height() == height
        ):
            if fmt is None or profile.get_format() == fmt:
                return profile

    return None


def build_camera_config(
    pipeline: Pipeline,
    color_width=None,
    color_height=None,
    depth_width=None,
    depth_height=None,
) -> Optional[Config]:
    """
    创建 Orbbec Pipeline Config。

    分辨率优先级：
        CLI 参数 > 全局参数 > 设备默认 Profile
    """

    color_width = color_width or COLOR_CAMERA_WIDTH
    color_height = color_height or COLOR_CAMERA_HEIGHT
    depth_width = depth_width or DEPTH_CAMERA_WIDTH
    depth_height = depth_height or DEPTH_CAMERA_HEIGHT

    config = Config()

    try:
        color_profiles = pipeline.get_stream_profile_list(
            OBSensorType.COLOR_SENSOR
        )

        depth_profiles = pipeline.get_stream_profile_list(
            OBSensorType.DEPTH_SENSOR
        )

        # ----------------------------------------------------------------------
        # Color
        # ----------------------------------------------------------------------
        color_profile = None

        if color_width and color_height:
            color_profile = find_profile(
                color_profiles,
                color_width,
                color_height,
                OBFormat.RGB,
            )

            if color_profile is not None:
                print(
                    f"[Camera] Color requested: "
                    f"{color_width}x{color_height}"
                )
            else:
                print(
                    "[Camera] Requested color "
                    f"{color_width}x{color_height} "
                    "not supported, using default."
                )

        if color_profile is None:
            color_profile = (
                color_profiles.get_default_video_stream_profile()
            )

            print(
                "[Camera] Color default: "
                f"{color_profile.get_width()}x"
                f"{color_profile.get_height()}"
            )

        config.enable_stream(color_profile)

        # ----------------------------------------------------------------------
        # Depth
        # ----------------------------------------------------------------------
        depth_profile = None

        if depth_width and depth_height:
            depth_profile = find_profile(
                depth_profiles,
                depth_width,
                depth_height,
            )

            if depth_profile is not None:
                print(
                    f"[Camera] Depth requested: "
                    f"{depth_width}x{depth_height}"
                )
            else:
                print(
                    "[Camera] Requested depth "
                    f"{depth_width}x{depth_height} "
                    "not supported, using default."
                )

        if depth_profile is None:
            depth_profile = (
                depth_profiles.get_default_video_stream_profile()
            )

            print(
                "[Camera] Depth default: "
                f"{depth_profile.get_width()}x"
                f"{depth_profile.get_height()}"
            )

        config.enable_stream(depth_profile)

    except Exception as error:
        print(f"[Camera] Configuration failed: {error}")
        return None

    return config


# ==============================================================================
# 13. Console output
# ==============================================================================

def print_detections(
    detections: List[Detection3D],
) -> None:
    """把三维检测结果打印到终端。"""

    if not detections:
        print("[Detection] No objects.")
        return

    print("-" * 100)

    for detection in detections:
        if detection.z_mm is None:
            print(
                f"{detection.class_name:<15} "
                f"conf={detection.confidence:.3f} "
                "XYZ=N/A"
            )
            continue

        line = (
            f"{detection.class_name:<15} "
            f"conf={detection.confidence:.3f}  "
            f"pixel=({detection.pixel_u}, {detection.pixel_v})  "
            f"XYZ=("
            f"{detection.x_mm:.1f}, "
            f"{detection.y_mm:.1f}, "
            f"{detection.z_mm:.1f}) mm  "
        )

        if detection.gx_mm is not None:
            line += (
                f"P_G=({detection.gx_mm:.1f},"
                f"{detection.gy_mm:.1f},"
                f"{detection.gz_mm:.1f})  "
            )

        line += (
            f"src={detection.depth_source} "
            f"pts={detection.cluster_depth_points}/"
            f"{detection.valid_depth_points}"
        )
        print(line)


# ==============================================================================
# 14. Main
# ==============================================================================

def main() -> int:
    parser = argparse.ArgumentParser(
        description="YOLO + Orbbec Gemini2 RGB-D 3D object detection"
    )

    # --------------------------------------------------------------------------
    # YOLO
    # --------------------------------------------------------------------------
    parser.add_argument(
        "--model",
        type=str,
        default=DEFAULT_MODEL,
        help="YOLO model path, e.g. yolo8n-seg.pt",
    )

    parser.add_argument(
        "--conf",
        type=float,
        default=CONF_THRESHOLD,
        help="YOLO confidence threshold",
    )

    parser.add_argument(
        "--iou",
        type=float,
        default=NMS_IOU_THRESHOLD,
        help="YOLO NMS IoU threshold",
    )

    parser.add_argument(
        "--imgsz",
        type=int,
        default=YOLO_IMAGE_SIZE,
        help="YOLO inference image size",
    )

    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_DEVICE,
        help="auto / cpu / 0 / 1 ...",
    )

    parser.add_argument(
        "--max_det",
        type=int,
        default=MAX_DETECTIONS,
        help="Maximum detections per frame",
    )

    parser.add_argument(
        "--no_warmup",
        action="store_true",
        help="Skip the initial YOLO warm-up inference",
    )

    # --------------------------------------------------------------------------
    # Camera
    # --------------------------------------------------------------------------
    parser.add_argument("--color_width", type=int, default=None)
    parser.add_argument("--color_height", type=int, default=None)
    parser.add_argument("--depth_width", type=int, default=None)
    parser.add_argument("--depth_height", type=int, default=None)

    # --------------------------------------------------------------------------
    # Output
    # --------------------------------------------------------------------------
    parser.add_argument(
        "--print_result",
        action="store_true",
        help="Print 3D detections to terminal",
    )

    parser.add_argument(
        "--print_interval",
        type=float,
        default=1.0,
        help="Terminal print interval in seconds",
    )

    parser.add_argument(
        "--no_display",
        action="store_true",
        help="Disable cv2 GUI for SSH/headless running",
    )

    parser.add_argument(
        "--debug",
        action="store_true",
        help="Print additional first-frame debugging information",
    )

    # ----------------------------------------------------------------------
    # 相机 -> 夹爪 转换
    # ----------------------------------------------------------------------
    parser.add_argument(
        "--t_g_c",
        type=str,
        default=f"{DEFAULT_T_G_C[0]},{DEFAULT_T_G_C[1]},{DEFAULT_T_G_C[2]}",
        help="相机光心在 {G} 中的位置 'cx,cy,cz'（mm），默认 0,21,27.8",
    )
    parser.add_argument(
        "--t_g_c_yaml",
        type=str,
        default=None,
        help="从 camera_to_gripper.yaml 读取 t_G_C_mm（优先级高于 --t_g_c）",
    )

    args = parser.parse_args()

    # --------------------------------------------------------------------------
    # t_G_C 加载（yaml 优先；否则用 --t_g_c）
    # --------------------------------------------------------------------------
    if args.t_g_c_yaml:
        t_g_c = load_t_g_c_from_yaml(args.t_g_c_yaml)
        print(f"[t_G_C] loaded from {args.t_g_c_yaml}: "
              f"cx={t_g_c[0]:.2f} cy={t_g_c[1]:.2f} cz={t_g_c[2]:.2f} mm")
    else:
        t_g_c = parse_t_g_c(args.t_g_c)
        print(f"[t_G_C] cx={t_g_c[0]:.2f} cy={t_g_c[1]:.2f} cz={t_g_c[2]:.2f} mm")

    # --------------------------------------------------------------------------
    # Resolve device
    # --------------------------------------------------------------------------
    device = resolve_device(args.device)

    print_runtime_info(device)

    # CUDA 性能设置：固定输入尺寸时通常有利。
    if device != "cpu" and torch.cuda.is_available():
        torch.backends.cudnn.benchmark = True

    # --------------------------------------------------------------------------
    # GUI：尽可能早地创建窗口
    # --------------------------------------------------------------------------
    display_enabled = init_display(
        enabled=not args.no_display
    )

    # --------------------------------------------------------------------------
    # Validate model
    # --------------------------------------------------------------------------
    print("=" * 80)
    print("[YOLO] Loading model")
    print("=" * 80)
    print(f"[YOLO] Model: {args.model}")

    if not os.path.isfile(args.model):
        print("[Error] YOLO model file does not exist:")
        print(f"        {args.model}")
        print()
        print("Please check --model or DEFAULT_MODEL.")
        return 1

    show_status(
        display_enabled,
        [
            "OpenCV GUI OK.",
            "Loading YOLO model...",
            os.path.basename(args.model),
        ],
    )

    model = YOLO(args.model)

    print(f"[YOLO] Task: {getattr(model, 'task', 'unknown')}")
    print(f"[YOLO] Device requested/resolved: {args.device} -> {device}")
    print()

    # --------------------------------------------------------------------------
    # YOLO warm-up
    # --------------------------------------------------------------------------
    if not args.no_warmup:
        yolo_warmup(
            model=model,
            device=device,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            max_det=args.max_det,
            display_enabled=display_enabled,
        )
    else:
        print("[YOLO] Warm-up skipped by --no_warmup")

    # --------------------------------------------------------------------------
    # Camera
    # --------------------------------------------------------------------------
    print("=" * 80)
    print("[Camera] Initializing")
    print("=" * 80)

    show_status(
        display_enabled,
        [
            "YOLO ready.",
            f"Device: {device}",
            "Starting Orbbec camera...",
        ],
    )

    pipeline = Pipeline()

    config = build_camera_config(
        pipeline,
        args.color_width,
        args.color_height,
        args.depth_width,
        args.depth_height,
    )

    if config is None:
        print("[Error] Could not configure camera.")
        return 1

    pipeline_started = False

    try:
        pipeline.start(config)
        pipeline_started = True

        align_filter = AlignFilter(
            align_to_stream=OBStreamType.COLOR_STREAM
        )

        print("[Camera] Pipeline started.")
        print("[Camera] Software D2C alignment enabled.")
        print()

        intrinsics: Optional[CameraIntrinsics] = None

        previous_time = time.perf_counter()
        last_print_time = 0.0

        first_real_inference = True
        frame_index = 0

        while True:
            frame_index += 1

            # ==================================================================
            # 1. RGB-D Frames
            # ==================================================================
            frames = pipeline.wait_for_frames(1000)

            if not frames:
                continue

            # ==================================================================
            # 2. D2C Alignment
            # ==================================================================
            frames = align_filter.process(frames)

            if not frames:
                continue

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame is None or depth_frame is None:
                continue

            # ==================================================================
            # 3. RGB -> OpenCV
            # ==================================================================
            image_bgr = frame_to_bgr_image(color_frame)

            if image_bgr is None:
                continue

            # ==================================================================
            # 4. Depth
            # ==================================================================
            depth_map = extract_depth_map(depth_frame)

            if depth_map is None:
                continue

            # D2C 后必须与 RGB 尺寸一致，否则不能直接按相同像素索引取深度。
            if depth_map.shape != image_bgr.shape[:2]:
                print(
                    "[Warning] RGB/Depth shape mismatch after D2C: "
                    f"RGB={image_bgr.shape[:2]}, "
                    f"Depth={depth_map.shape}"
                )
                continue

            # ==================================================================
            # 5. Camera intrinsics
            # ==================================================================
            if intrinsics is None:
                intrinsics = get_rgb_intrinsics(
                    pipeline,
                    image_bgr.shape[:2],
                )

                print("=" * 80)
                print("[Camera] RGB Intrinsics")
                print("=" * 80)
                print(
                    f"resolution = "
                    f"{intrinsics.width}x{intrinsics.height}"
                )
                print(f"fx = {intrinsics.fx:.4f}")
                print(f"fy = {intrinsics.fy:.4f}")
                print(f"cx = {intrinsics.cx:.4f}")
                print(f"cy = {intrinsics.cy:.4f}")
                print()

            # ==================================================================
            # 6. 在第一次真实 YOLO 推理前先把原始相机画面显示出来
            #
            # 如果这里已经能看到画面，而随后卡住，问题就在 YOLO/CUDA，
            # 而不是 OpenCV GUI 或相机。
            # ==================================================================
            if display_enabled and first_real_inference:
                preview = image_bgr.copy()

                cv2.putText(
                    preview,
                    "Camera OK - starting first YOLO inference...",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    YELLOW,
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow(WINDOW_NAME, preview)
                cv2.waitKey(1)

            # ==================================================================
            # 7. YOLO inference
            # ==================================================================
            if first_real_inference or args.debug:
                print(
                    "[DEBUG] YOLO input: "
                    f"shape={image_bgr.shape}, "
                    f"dtype={image_bgr.dtype}, "
                    f"device={device}"
                )

                if first_real_inference:
                    print("[DEBUG] First real YOLO inference starting...")

            inference_start = time.perf_counter()

            results = model.predict(
                source=image_bgr,
                imgsz=args.imgsz,

                # Ultralytics 内部完成保持纵横比的 LetterBox。
                rect=False,

                conf=args.conf,
                iou=args.iou,
                max_det=args.max_det,
                device=device,
                verbose=False,

                # Seg 模型让 mask 直接映射回原图尺寸。
                retina_masks=True,
            )

            if device != "cpu" and torch.cuda.is_available():
                # 用于让 inference_ms 更接近真实 GPU 完成时间。
                torch.cuda.synchronize()

            inference_ms = (
                time.perf_counter() - inference_start
            ) * 1000.0

            if first_real_inference:
                print(
                    "[DEBUG] First real YOLO inference finished: "
                    f"{inference_ms:.1f} ms"
                )

            result = results[0]

            # ==================================================================
            # 8. YOLO + Depth -> XYZ
            # ==================================================================
            output_image, detections = process_yolo_result(
                image_bgr,
                depth_map,
                result,
                intrinsics,
                t_g_c=t_g_c,
            )

            if first_real_inference:
                print(
                    "[DEBUG] First post-process finished. "
                    f"objects={len(detections)}"
                )

            # ==================================================================
            # 9. FPS
            # ==================================================================
            current_time = time.perf_counter()

            frame_delta = max(
                current_time - previous_time,
                1e-6,
            )

            fps = 1.0 / frame_delta
            previous_time = current_time

            # ==================================================================
            # 10. 亮度增强 
            # ==================================================================
            gray_mean = float(cv2.cvtColor(output_image, cv2.COLOR_BGR2GRAY).mean())
            if gray_mean < BRIGHTNESS_LOW:
                lab = cv2.cvtColor(output_image, cv2.COLOR_BGR2LAB)
                l_ch, a_ch, b_ch = cv2.split(lab)
                clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=(8, 8))
                l_enhanced = clahe.apply(l_ch)
                lab_enhanced = cv2.merge((l_enhanced, a_ch, b_ch))
                output_image = cv2.cvtColor(lab_enhanced, cv2.COLOR_LAB2BGR)

            # ==================================================================
            # 11. OSD
            # ==================================================================
            cv2.putText(
                output_image,
                (
                    f"Infer: {inference_ms:.1f} ms"
                    f" | FPS: {fps:.1f}"
                    f" | Objects: {len(detections)}"
                    f" | Device: {device}"
                    f" | Bright: {gray_mean:.0f}/255"
                ),
                (10, 22),
                FONT_FACE,
                FONT_SCALE,
                RED,
                FONT_THICKNESS,
                cv2.LINE_AA,
            )

            # 夹爪偏移提示行（t_G_C）
            cv2.putText(
                output_image,
                f"t_G_C=({t_g_c[0]:.0f},{t_g_c[1]:.0f},{t_g_c[2]:.0f}) mm",
                (10, 44),
                FONT_FACE,
                FONT_SCALE,
                YELLOW,
                FONT_THICKNESS,
                cv2.LINE_AA,
            )

            # ==================================================================
            # 11. Terminal structured output
            # ==================================================================
            if args.print_result:
                wall_time = time.time()

                if (
                    wall_time - last_print_time
                    >= args.print_interval
                ):
                    print_detections(detections)
                    last_print_time = wall_time

            # ==================================================================
            # 12. GUI
            # ==================================================================
            if display_enabled:
                if first_real_inference:
                    print("[DEBUG] Calling cv2.imshow() with processed frame...")

                cv2.imshow(
                    WINDOW_NAME,
                    output_image,
                )

                key = cv2.waitKey(1) & 0xFF

                if key in (
                    ESC_KEY,
                    ord("q"),
                    ord("Q"),
                ):
                    print("[Info] Exit key pressed.")
                    break

            first_real_inference = False

    except KeyboardInterrupt:
        print("\n[Info] Interrupted by user.")

    except Exception as error:
        print("\n[Error] Unhandled exception:")
        print(f"        {type(error).__name__}: {error}")
        raise

    finally:
        if display_enabled:
            cv2.destroyAllWindows()

        if pipeline_started:
            try:
                pipeline.stop()
                print("[Camera] Pipeline stopped.")
            except Exception as error:
                print(f"[Camera] pipeline.stop() failed: {error}")

    return 0


# ==============================================================================
# Entry
# ==============================================================================

if __name__ == "__main__":
    sys.exit(main())
    