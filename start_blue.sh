#!/bin/bash
# 蓝方总启动命令(远程SSH/命令行专用,无桌面环境也能直接运行)
# 用法: start_blue   或   bash start_blue.sh   或   ./start_blue.sh [额外参数]
set -e

WORKSPACE=/home/upre/TR/b-team
SERIAL_DEV=${SERIAL_DEV:-/dev/ttyACM0}

cd "$WORKSPACE"

echo "============================================================"
echo "  蓝方启动 team=blue  (serial_gateway, X原样)"
echo "  时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "  工作区: $WORKSPACE"
echo "============================================================"

# ---- 前置检查 1: ROS 环境 ----
echo "[检查 1/6] ROS2 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✓ ROS2 Humble 已加载"
else
    echo "  ✗ 未找到 /opt/ros/humble/setup.bash, ROS2 未安装"
    exit 1
fi

# ---- 前置检查 2: 工作区编译 ----
echo "[检查 2/6] 工作区编译产物..."
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    echo "  ✓ install/setup.bash 已加载"
else
    echo "  ✗ 未找到 install/setup.bash, 请先执行: cd $WORKSPACE && colcon build --symlink-install"
    exit 1
fi

# ---- 前置检查 3: common_launch_pkg 包 ----
echo "[检查 3/6] common_launch_pkg 包..."
if ros2 pkg list 2>/dev/null | grep -q "^common_launch_pkg$"; then
    echo "  ✓ common_launch_pkg 已注册"
else
    echo "  ✗ common_launch_pkg 未找到"
    echo "    可能原因: package.dsv 缺少 ament_prefix_path 行"
    echo "    修复方法: 重新编译 → colcon build --symlink-install --packages-select common_launch_pkg"
    exit 1
fi

# ---- 前置检查 4: 串口设备 ----
echo "[检查 4/6] 串口设备 ($SERIAL_DEV)..."
if [ -e "$SERIAL_DEV" ]; then
    if [ -w "$SERIAL_DEV" ]; then
        echo "  ✓ $SERIAL_DEV 存在且可写"
    else
        echo "  ⚠ $SERIAL_DEV 存在但不可写, 尝试: sudo chmod 666 $SERIAL_DEV"
    fi
else
    echo "  ✗ $SERIAL_DEV 不存在"
    echo "    可用串口设备:"
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sed 's/^/      /' || echo "      (无)"
    echo "    提示: STM32 未连接或未上电; 若设备名不同, 启动时指定:"
    echo "      ./start_blue.sh serial_device:=/dev/ttyXXX"
fi

# ---- 前置检查 5: Orbbec 相机 ----
echo "[检查 5/6] Orbbec RGB-D 相机..."
ORBEEC_FOUND=$(lsusb 2>/dev/null | grep -ci "orbbec\|2bc5\|8086:0b4c" || true)
if [ "$ORBEEC_FOUND" -gt 0 ]; then
    echo "  ✓ 检测到 Orbbec 相机 (USB)"
else
    echo "  ⚠ 未检测到 Orbbec 相机"
    echo "    提示: 请检查 USB 连接, 确保插入 USB 3.0 蓝色口"
    echo "    块检测节点将启动但无法获取图像"
fi

# ---- 前置检查 6: MID360 雷达网络 ----
echo "[检查 6/6] MID360 雷达网络..."
if ping -c 1 -W 1 192.168.1.17* &>/dev/null; then
    echo "  ✓ 雷达 IP 192.168.1.17* 可达"
else
    echo "  ⚠ 雷达 IP 192.168.1.17* 不可达"
    echo "    提示: 检查网线连接和静态IP配置 (sudo ip addr add 192.168.1.50/24 dev enp*s0)"
fi

echo "============================================================"
echo "  前置检查完成, 启动 ROS2 launch..."
echo "  队伍: blue (串口X原样)"
echo "  串口: $SERIAL_DEV"
echo "  阶段 1 (0s) :  雷达驱动 + FAST-LIO"
echo "  阶段 2 (6s) :  感知节点 (相机+块检测+定位)"
echo "  阶段 3 (12s):  串口网关 (serial_gateway)"
echo "============================================================"
echo ""

exec ros2 launch common_launch_pkg common_launch.py team:=blue "$@"
