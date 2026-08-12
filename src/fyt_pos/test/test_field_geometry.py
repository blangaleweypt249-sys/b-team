import pytest

from fyt_pos.field_geometry import (
    AxisAlignedZone,
    Point2D,
    classify_zone,
    lidar_to_base_pose,
    rotate_to_field,
    start_corner_transform,
)


def test_rotates_origin_to_zero():
    # 原点在经过平移和旋转后必须保持为坐标零点。
    assert rotate_to_field(Point2D(3.0, 4.0), Point2D(3.0, 4.0), 0.0) == Point2D(0.0, 0.0)


def test_classifies_first_matching_zone():
    # 验证区域命中与未命中两种基本情况。
    zones = [AxisAlignedZone('storage_l1', 0.0, 1.4, 0.0, 0.4)]
    assert classify_zone(Point2D(1.0, 0.2), zones) == 'storage_l1'
    assert classify_zone(Point2D(2.0, 0.2), zones) is None


def test_converts_lidar_pose_to_base_pose_using_extrinsics():
    position, yaw = lidar_to_base_pose(
        Point2D(1.0, 2.0), 1.5707963267948966, Point2D(0.2, 0.0), 0.0
    )
    assert position.x == 1.0
    assert position.y == 1.8
    assert yaw == 1.5707963267948966


def test_maps_each_bottom_start_to_its_local_first_quadrant():
    left_position = start_corner_transform(
        Point2D(2.0, 0.0), 'bottom_right', 11.0, 6.0
    )
    right_position = start_corner_transform(
        Point2D(2.0, 1.0), 'bottom_right', 11.0, 6.0
    )
    assert left_position.x == pytest.approx(2.0)
    assert left_position.y == pytest.approx(0.0)
    assert right_position.x == pytest.approx(2.0)
    assert right_position.y == pytest.approx(1.0)