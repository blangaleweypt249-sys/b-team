#!/bin/bash
# 蓝方总启动命令(远程SSH/命令行专用,无桌面环境也能直接运行)
# 用法: start_blue   或   bash start_blue.sh   或   ./start_blue.sh [额外参数]
set -e
cd /home/upre/TR/b-team
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=============================================="
echo "  蓝方启动 team=blue  (serial_gateway, X原样)"
echo "=============================================="
exec ros2 launch common_launch_pkg common_launch.py team:=blue "$@"
