"""比赛场地定位与安全区域判断所需的几何基础类型。"""

from dataclasses import dataclass
from math import cos, pi, sin
from typing import Iterable, Optional


@dataclass(frozen=True)
class Point2D:
    """比赛场地坐标系中的二维点，单位为米。"""

    x: float
    y: float


@dataclass(frozen=True)
class AxisAlignedZone:
    """边与坐标轴平行的命名矩形区域，单位为米。"""

    name: str
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def contains(self, point: Point2D, margin: float = 0.0) -> bool:
        """判断点是否处于区域内；正 margin 会向内收缩有效区域。"""
        return (
            self.min_x + margin <= point.x <= self.max_x - margin
            and self.min_y + margin <= point.y <= self.max_y - margin
        )


def rotate_to_field(point: Point2D, origin: Point2D, yaw_rad: float) -> Point2D:
    """将里程计坐标转换到以 origin 和 yaw_rad 为基准的赛场坐标系。"""
    # 先平移到比赛起点，再消除起步朝向带来的坐标轴偏转。
    delta_x = point.x - origin.x
    delta_y = point.y - origin.y
    return Point2D(
        x=cos(yaw_rad) * delta_x + sin(yaw_rad) * delta_y,
        y=-sin(yaw_rad) * delta_x + cos(yaw_rad) * delta_y,
    )


def start_corner_transform(
    local_point: Point2D,
    start_corner: str,
    field_length_m: float,
    field_width_m: float,
) -> Point2D:
    """将里程计局部坐标映射为以本方下角为原点的第一象限坐标。"""
    transforms = {
        # 两种起点均从本方下角开始：前进和朝赛场中线移动都为正。
        'bottom_left': 0.0,
        'bottom_right': 0.0,
    }
    if start_corner not in transforms:
        raise ValueError(f'未知起始角: {start_corner}')
    rotation = transforms[start_corner]
    return Point2D(
        cos(rotation) * local_point.x - sin(rotation) * local_point.y,
        sin(rotation) * local_point.x + cos(rotation) * local_point.y,
    )


def lidar_to_base_pose(
    lidar_position: Point2D,
    lidar_yaw_rad: float,
    base_to_lidar: Point2D,
    base_to_lidar_yaw_rad: float,
) -> tuple[Point2D, float]:
    """根据 base_link 到雷达的外参，将 LIO 雷达位姿转换为车体中心位姿。"""
    base_yaw_rad = lidar_yaw_rad - base_to_lidar_yaw_rad
    return (
        Point2D(
            lidar_position.x - (
                cos(base_yaw_rad) * base_to_lidar.x - sin(base_yaw_rad) * base_to_lidar.y
            ),
            lidar_position.y - (
                sin(base_yaw_rad) * base_to_lidar.x + cos(base_yaw_rad) * base_to_lidar.y
            ),
        ),
        base_yaw_rad,
    )


def classify_zone(point: Point2D, zones: Iterable[AxisAlignedZone]) -> Optional[str]:
    """返回点命中的第一个配置区域；未命中任何区域时返回 None。"""
    for zone in zones:
        if zone.contains(point):
            return zone.name
    return None