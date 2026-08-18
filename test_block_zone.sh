#!/bin/bash
# ============================================================
# 块检测区域测试脚本(结合雷达定位)
#
# 自动打开 4 个终端标签页:
#   tab1 雷达+FAST-LIO  — 发布 /Odometry
#   tab2 块检测+定位    — 只在 block/special1/special2 区检测块
#   tab3 机器人位置     — 实时显示 field_pose + current_zone
#   tab4 块信息         — 实时显示 block_position + overlay 图像
#
# 用法:
#   ./test_block_zone.sh            # 默认蓝方
#   ./test_block_zone.sh red        # 红方
#
# 前置: Orbbec 相机已连接 USB 3.0,MID360 雷达已上电
# 退出: 关闭弹出的终端窗口即可终止所有节点
# ============================================================

set -e

TEAM=${1:-blue}
WORKSPACE=/home/upre/TR/b-team
SOURCE_CMD="source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash"

# 块区域测试只用一套 test_block_zone.launch.py (定位/检测通用,内部坐标系不变)
# 串口层面才分蓝红:如需串口,手动另开终端启动对应launch
TEST_LAUNCH="test_block_zone.launch.py"
if [ "${TEAM}" = "red" ]; then
    TEAM_LABEL="红方(定位/检测共用一套,需要串口请另启 serial_gateway_red.launch.py)"
else
    TEAM_LABEL="蓝方(定位/检测共用一套,需要串口请另启 serial_gateway.launch.py)"
fi

# 检查工作区是否已构建
if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
    echo "[错误] 未找到 ${WORKSPACE}/install/setup.bash,请先在工作区执行 colcon build"
    exit 1
fi

# 检查终端模拟器
if command -v gnome-terminal &>/dev/null; then
    TERM_BIN="gnome-terminal"
elif command -v xterm &>/dev/null; then
    TERM_BIN="xterm"
else
    echo "[错误] 未找到 gnome-terminal 或 xterm,请安装其一"
    exit 1
fi

echo "========================================"
echo " 块检测区域测试"
echo " 队伍: ${TEAM_LABEL}"
echo " 工作区: ${WORKSPACE}"
echo " 启动文件: common_launch_pkg/${TEST_LAUNCH}"
echo "========================================"

# 启动一个终端标签(自适应 gnome-terminal / xterm)
launch_term() {
    local title="$1"
    local cmd="$2"
    if [ "${TERM_BIN}" = "gnome-terminal" ]; then
        gnome-terminal --tab -t "${title}" -- bash -lc "${cmd}; exec bash"
    else
        xterm -T "${title}" -e "bash -lc '${cmd}; exec bash'" &
    fi
}

# tab1: 雷达驱动 + FAST-LIO
launch_term "雷达+FAST-LIO" \
    "${SOURCE_CMD} && ros2 launch common_launch_pkg lidar_driver.launch.py"

echo "[1/4] 雷达+FAST-LIO 终端已启动,等待 4 秒..."
sleep 4

# tab2: 块检测 + 场地定位 (按队伍直接加载对应launch文件,不传team参数)
launch_term "块检测+定位" \
    "${SOURCE_CMD} && ros2 launch common_launch_pkg ${TEST_LAUNCH}"

echo "[2/4] 块检测+定位终端已启动,等待 5 秒..."
sleep 5

# tab3: 看机器人位置 (field_pose + current_zone)
# current_zone 后台输出,field_pose 前台输出,两个流在同一终端
launch_term "机器人位置" \
    "${SOURCE_CMD} && ros2 topic echo /competition/current_zone & ros2 topic echo /competition/field_pose"

echo "[3/4] 机器人位置终端已启动"

# tab4: 看块信息 (block_position 数据 + overlay 图像)
# rqt_image_view 会弹出独立窗口显示叠加图
launch_term "块信息" \
    "${SOURCE_CMD} && ros2 topic echo /perception/block_position & rqt_image_view /perception/block_overlay"

echo "[4/4] 块信息终端已启动"
echo ""
echo "========================================"
echo " 所有终端已打开"
echo " - 把机器人推到夹块区(block)或特殊点(special1/2)"
echo "   即可看到块检测自动开始"
echo " - overlay 窗口会显示检测叠加图"
echo " - 关闭任意终端窗口即可停止对应节点"
echo "========================================"
