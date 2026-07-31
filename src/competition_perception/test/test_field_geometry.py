from competition_perception.field_geometry import (
    AxisAlignedZone,
    Point2D,
    classify_zone,
    rotate_to_field,
)


def test_rotates_origin_to_zero():
    # 原点在经过平移和旋转后必须保持为坐标零点。
    assert rotate_to_field(Point2D(3.0, 4.0), Point2D(3.0, 4.0), 0.0) == Point2D(0.0, 0.0)


def test_classifies_first_matching_zone():
    # 验证区域命中与未命中两种基本情况。
    zones = [AxisAlignedZone('storage_l1', 0.0, 1.4, 0.0, 0.4)]
    assert classify_zone(Point2D(1.0, 0.2), zones) == 'storage_l1'
    assert classify_zone(Point2D(2.0, 0.2), zones) is None