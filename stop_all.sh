#!/bin/bash
# 停止所有 ROS2 节点(SSH下快速关闭)
cd /home/upre/TR/b-team
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "⚠️  停止所有 ROS2 节点..."
ros2 lifecycle set /  shutdown >/dev/null 2>&1 || true
pkill -9 -f "ros2 launch"  2>/dev/null || true
pkill -9 -f "rclpy"         2>/dev/null || true
pkill -9 -f "fast_lio"      2>/dev/null || true
pkill -9 -f "livox_ros_dr"  2>/dev/null || true
pkill -9 -f "serial_gatewa" 2>/dev/null || true
sleep 1
echo "✅ 已停止所有 ROS2 相关进程"
echo "剩余进程:"
ps aux | grep -E "ros2|rclpy|livox|fast_lio|serial_gateway" | grep -v grep || echo "  (无)"
