#!/usr/bin/env python3
"""坐标转换逻辑测试脚本 — 验证 雷达→相机→雷达→夹爪 的转换链路是否正确。

不依赖 ROS2 / YOLO，纯 numpy 计算。

运行方式:
  python3 test_coordinate_transform.py
"""

import numpy as np
from math import cos, sin, radians, degrees


# ==============================================================================
# 1. 参数 (与 ball_distance_node.py 完全一致)
# ==============================================================================

# 相机内参
FX = 657.5551152642361
FY = 656.3202788734417
CX = 314.7869474017467
CY = 191.0288244576825

# 外参: 相机在雷达右侧17.3cm, 平行安装
EXT_X = -0.035   # 后方为负
EXT_Y = -0.173   # 右侧为负
EXT_Z = 0.0
EXT_ROLL = 0.0
EXT_PITCH = 0.0
EXT_YAW = 0.0

# 输出转换: 雷达→夹爪
# 雷达在夹爪后方17.4cm、左方17.3cm, 雷达朝向夹爪右方(-90°)
GRIPPER_X = -0.174    # 后方为负
GRIPPER_Y = 0.173      # 左方为正
GRIPPER_YAW_DEG = -90.0


# ==============================================================================
# 2. 构建变换矩阵 (复制自 ball_distance_node.py)
# ==============================================================================

def build_lidar_to_cam_matrix():
    """构建雷达→相机变换矩阵 (4x4)。"""
    roll, pitch, yaw = EXT_ROLL, EXT_PITCH, EXT_YAW
    Rx = np.array([[1, 0, 0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])
    Ry = np.array([[cos(pitch), 0, sin(pitch)],
                   [0, 1, 0],
                   [-sin(pitch), 0, cos(pitch)]])
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                   [sin(yaw), cos(yaw), 0],
                   [0, 0, 1]])
    R_user = Rx @ Ry @ Rz

    # 坐标轴变换: 雷达(X前,Y左,Z上) -> 相机(Z前,X右,Y下)
    R_axis = np.array([
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0],
    ], dtype=np.float64)

    R = R_axis @ R_user
    t = np.array([EXT_X, EXT_Y, EXT_Z], dtype=np.float64)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ t
    return T


def build_lidar_to_gripper_matrix():
    """构建雷达→夹爪旋转矩阵和平移向量。"""
    gyaw = radians(GRIPPER_YAW_DEG)
    R = np.array([
        [cos(gyaw), -sin(gyaw), 0],
        [sin(gyaw),  cos(gyaw), 0],
        [0, 0, 1],
    ], dtype=np.float64)
    t = np.array([GRIPPER_X, GRIPPER_Y, 0.0], dtype=np.float64)
    return R, t


# ==============================================================================
# 3. 正向转换: 雷达坐标系中的球 → 像素坐标 + 距离 → 夹爪坐标系
# ==============================================================================

def lidar_point_to_gripper(ball_lidar, T_lidar_to_cam, R_l2g, t_l2g):
    """模拟 ball_distance_node 的完整转换链路。

    输入: 球在雷达坐标系中的3D位置 (x前, y左, z上)
    输出: (距离, 像素坐标, 球在夹爪坐标系中的位置)
    """
    # 1. 雷达→相机坐标系
    ball_h = np.array([ball_lidar[0], ball_lidar[1], ball_lidar[2], 1.0])
    ball_cam = (T_lidar_to_cam @ ball_h)[:3]

    # 2. 相机坐标系→像素坐标 (针孔模型)
    z = ball_cam[2]  # 深度 = 距离
    if z <= 0:
        return None, None, None
    px = int(FX * ball_cam[0] / z + CX)
    py = int(FY * ball_cam[1] / z + CY)
    distance = z

    # 3. 像素+距离→3D位置 (相机坐标系, 模拟节点中的逆投影)
    ball_3d_cam = np.array([
        (px - CX) * distance / FX,
        (py - CY) * distance / FY,
        distance,
    ])

    # 4. 相机→雷达坐标系 (逆变换)
    T_cam_to_lidar = np.linalg.inv(T_lidar_to_cam)
    ball_3d_cam_h = np.array([ball_3d_cam[0], ball_3d_cam[1], ball_3d_cam[2], 1.0])
    ball_3d_lidar = (T_cam_to_lidar @ ball_3d_cam_h)[:3]

    # 5. 雷达→夹爪坐标系
    ball_3d_gripper = R_l2g @ ball_3d_lidar + t_l2g

    return distance, (px, py), ball_3d_gripper


# ==============================================================================
# 4. 测试用例
# ==============================================================================

def run_tests():
    T_lidar_to_cam = build_lidar_to_cam_matrix()
    R_l2g, t_l2g = build_lidar_to_gripper_matrix()

    print("=" * 80)
    print("坐标转换逻辑测试")
    print("=" * 80)

    # 打印变换信息
    print(f"\n相机外参: 相机在雷达右侧 {abs(EXT_Y)*100:.1f}cm")
    print(f"输出转换: 雷达在夹爪后方 {abs(GRIPPER_X)*100:.1f}cm, 左方 {GRIPPER_Y*100:.1f}cm, 朝向右侧 {GRIPPER_YAW_DEG}°")
    print()

    # 定义测试用例: 球在雷达坐标系中的位置
    # 雷达坐标系: X前 Y左 Z上
    test_cases = [
        # (名称, 雷达系坐标, 期望说明)
        ("正前方1m",
         np.array([1.0, 0.0, 0.0]),
         "球在雷达正前方1m处"),

        ("正前方2m",
         np.array([2.0, 0.0, 0.0]),
         "球在雷达正前方2m处"),

        ("前方偏左30cm, 1.5m",
         np.array([1.5, 0.3, 0.0]),
         "球在雷达前方1.5m, 左侧30cm"),

        ("前方偏右30cm, 1.5m",
         np.array([1.5, -0.3, 0.0]),
         "球在雷达前方1.5m, 右侧30cm"),

        ("前方偏上20cm, 1m",
         np.array([1.0, 0.0, 0.2]),
         "球在雷达前方1m, 上方20cm"),

        ("近距离50cm",
         np.array([0.5, 0.0, 0.0]),
         "球在雷达前方50cm"),
    ]

    print(f"{'测试场景':<25} {'雷达坐标(x,y,z)':<25} {'距离':>6} {'像素(u,v)':<12} {'夹爪坐标(x,y,z)':<30} {'验证'}")
    print("-" * 120)

    all_pass = True

    for name, ball_lidar, desc in test_cases:
        dist, pixel, ball_gripper = lidar_point_to_gripper(
            ball_lidar, T_lidar_to_cam, R_l2g, t_l2g)

        if dist is None:
            print(f"{name:<25} {'('.join(f'{v:.2f}' for v in ball_lidar)}')  {'失败: 球在相机后方'}")
            all_pass = False
            continue

        lidar_str = f"({ball_lidar[0]:.2f}, {ball_lidar[1]:.2f}, {ball_lidar[2]:.2f})"
        gripper_str = f"({ball_gripper[0]:.2f}, {ball_gripper[1]:.2f}, {ball_gripper[2]:.2f})"
        pixel_str = f"({pixel[0]}, {pixel[1]})"

        # 验证逻辑:
        # 雷达朝向夹爪右侧(-90°), 所以:
        # 夹爪X(前) ≈ 雷达Y(左)
        # 夹爪Y(左) ≈ -雷达X(前)
        expected_gx = ball_lidar[1] + GRIPPER_X  # 雷达Y + 平移
        expected_gy = -ball_lidar[0] + GRIPPER_Y  # -雷达X + 平移
        expected_gz = ball_lidar[2]

        tol = 0.05  # 5cm 容差 (像素量化误差)
        x_ok = abs(ball_gripper[0] - expected_gx) < tol
        y_ok = abs(ball_gripper[1] - expected_gy) < tol
        z_ok = abs(ball_gripper[2] - expected_gz) < tol
        passed = x_ok and y_ok and z_ok

        status = "✅ 通过" if passed else "❌ 失败"
        if not passed:
            all_pass = False

        print(f"{name:<25} {lidar_str:<25} {dist:>5.2f}m {pixel_str:<12} {gripper_str:<30} {status}")

        if not passed:
            print(f"  期望: ({expected_gx:.2f}, {expected_gy:.2f}, {expected_gz:.2f})")
            print(f"  差异: dx={ball_gripper[0]-expected_gx:.3f}, dy={ball_gripper[1]-expected_gy:.3f}, dz={ball_gripper[2]-expected_gz:.3f}")

    # 总结
    print("\n" + "=" * 80)
    if all_pass:
        print("✅ 所有测试通过! 坐标转换逻辑正确。")
    else:
        print("❌ 有测试失败, 请检查转换逻辑。")
    print("=" * 80)

    # 额外: 打印变换矩阵供调试
    print(f"\n雷达→相机变换矩阵:\n{T_lidar_to_cam}")
    print(f"\n雷达→夹爪旋转矩阵:\n{R_l2g}")
    print(f"雷达→夹爪平移: {t_l2g}")

    # 额外: 验证物理意义
    print("\n" + "-" * 40)
    print("物理意义验证:")
    print("-" * 40)

    # 雷达朝向夹爪右侧(-90°), 所以:
    #   雷达X(前) → 夹爪Y的负方向(右方)
    #   雷达Y(左) → 夹爪X的正方向(前方)
    #
    # 球在雷达正前方1m → 夹爪坐标系:
    #   夹爪X(前) = 雷达Y(0) + 平移(-0.174) = -0.174m (后方17.4cm, 因为雷达在夹爪后方)
    #   夹爪Y(左) = -雷达X(1.0) + 平移(0.173) = -0.827m (右方82.7cm, 雷达前方=夹爪右方)
    ball = np.array([1.0, 0.0, 0.0])
    dist, pix, gripper = lidar_point_to_gripper(ball, T_lidar_to_cam, R_l2g, t_l2g)
    print(f"球在雷达正前方1m → 夹爪坐标系: ({gripper[0]:.3f}, {gripper[1]:.3f}, {gripper[2]:.3f})")
    print(f"  解读: 球在夹爪前方 {gripper[0]*100:.1f}cm, 左方 {gripper[1]*100:.1f}cm, 上方 {gripper[2]*100:.1f}cm")
    print(f"  期望: 前方 ≈ -17.4cm (雷达在夹爪后方17.4cm, 球与雷达同高)")
    print(f"  期望: 左方 ≈ -82.7cm (雷达前方1m = 夹爪右方1m, 加上雷达左偏17.3cm → 右方82.7cm)")


if __name__ == '__main__':
    run_tests()
