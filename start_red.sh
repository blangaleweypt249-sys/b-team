#!/bin/bash
# 红方总启动命令(远程SSH/命令行专用)
# serial_gateway_red — 写死位置帧 field_x_m 取反, 内部坐标系/感知/定位不变
set -e
cd /home/upre/TR/b-team
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=============================================="
echo "  红方启动 team=red  (serial_gateway_red, X取反)"
echo "=============================================="
exec ros2 launch common_launch_pkg common_launch.py team:=red "$@"
