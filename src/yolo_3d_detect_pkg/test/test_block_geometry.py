import numpy as np

from yolo_3d_detect_pkg.block_geometry import (
    map_color_pixel_to_depth,
    median_depth_m,
    pixel_depth_to_robot_camera_point,
)


def test_maps_unregistered_color_pixel_to_depth_resolution():
    assert map_color_pixel_to_depth(320, 240, (480, 640, 3), (240, 320), False) == (160, 120)


def test_maps_pixel_within_depth_image_bounds():
    assert map_color_pixel_to_depth(1000, -1, (480, 640, 3), (240, 320), True) == (319, 0)


def test_uses_median_valid_depth_in_meters():
    depth = np.array([[0, 1000, 1000], [1000, 1500, 30000], [1000, 1000, 1000]], dtype=np.uint16)
    assert median_depth_m(depth, 1, 1, 3) == 1.0


def test_projects_pixel_using_robot_camera_convention():
    assert pixel_depth_to_robot_camera_point(420, 140, 2.0, 200.0, 200.0, 320.0, 240.0) == (2.0, -1.0, 1.0)