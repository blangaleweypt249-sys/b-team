"""大地块 RGB-D 检测所需的纯几何与深度处理函数。"""

from typing import Tuple

import numpy as np


def map_color_pixel_to_depth(
    pixel_x: int,
    pixel_y: int,
    color_shape: Tuple[int, ...],
    depth_shape: Tuple[int, ...],
    depth_is_registered: bool,
) -> Tuple[int, int]:
    """将彩色图像像素映射为有效范围内的深度图像像素。"""
    if depth_is_registered:
        depth_x, depth_y = pixel_x, pixel_y
    else:
        depth_x = round(pixel_x * depth_shape[1] / color_shape[1])
        depth_y = round(pixel_y * depth_shape[0] / color_shape[0])
    return (
        max(0, min(depth_shape[1] - 1, depth_x)),
        max(0, min(depth_shape[0] - 1, depth_y)),
    )


def median_depth_m(depth_image: np.ndarray, pixel_x: int, pixel_y: int, window_size: int) -> float:
    """返回目标像素邻域的有效深度中值，输入深度单位为毫米。"""
    half_window = max(1, window_size // 2)
    depth_roi = depth_image[
        max(0, pixel_y - half_window):pixel_y + half_window + 1,
        max(0, pixel_x - half_window):pixel_x + half_window + 1,
    ]
    valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < 20000)]
    return float(np.median(valid_depths)) / 1000.0 if valid_depths.size else 0.0


def pixel_depth_to_robot_camera_point(
    pixel_x: int,
    pixel_y: int,
    depth_m: float,
    focal_x: float,
    focal_y: float,
    principal_x: float,
    principal_y: float,
) -> Tuple[float, float, float]:
    """按 x 前、y 左、z 上约定，将像素与深度反投影为机器人相机坐标。"""
    return (
        depth_m,
        -((pixel_x - principal_x) * depth_m / focal_x),
        -((pixel_y - principal_y) * depth_m / focal_y),
    )