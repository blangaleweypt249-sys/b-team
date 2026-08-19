#!/bin/bash
# 查看系统状态 —— SSH下快速查看各话题是否正常
# 用法: bash status_check.sh
cd /home/upre/TR/b-team
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=== ROS2 话题频率(5s采样,超时2s) ==="
for t in /Odometry /competition/field_pose /perception/block_position /perception/ball_position /competition/serial/controller_connected; do
    echo -n "  $t : "
    if ros2 topic info "$t" >/dev/null 2>&1; then
        freq_line=$(timeout 3 ros2 topic hz "$t" -w 1 2>/dev/null | grep "average rate:" | head -1)
        if [ -n "$freq_line" ]; then
            echo "✅ $freq_line"
        else
            echo "⚠️  话题存在但无数据"
        fi
    else
        echo "❌  话题未发布"
    fi
done

echo ""
echo "=== 当前区域 ==="
timeout 2 ros2 topic echo /competition/current_zone --once 2>/dev/null || echo "❌  区域未发布"

echo ""
echo "=== 机器人位置(截断) ==="
timeout 2 ros2 topic echo /competition/field_pose --once --field pose.position 2>/dev/null || echo "❌  位置未发布"

echo ""
echo "=== 控制器在线状态 ==="
timeout 2 ros2 topic echo /competition/serial/controller_connected --once 2>/dev/null || echo "❌  状态未发布"
echo ""
echo "=== 控制器状态(状态机state) ==="
timeout 2 ros2 topic echo /competition/serial/controller_state --once 2>/dev/null || echo "❌  状态未发布"
