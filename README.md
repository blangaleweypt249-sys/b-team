# 比赛感知工作区

这是面向 2026 年队内赛的独立 ROS 2 源码工作区。项目负责感知信息处理，并为后续串口发送模块提供标准化的定位与视觉数据；当前版本只发布感知结果，不产生底盘或机械臂控制指令。

## 目录说明

- `src/competition_perception/competition_perception/field_localizer.py`：将 FAST-LIO 里程计转换为比赛坐标系中的机器人位姿，并判断所在功能区域。
- `src/competition_perception/competition_perception/vision_detector.py`：检测 RGB-D 图像中的金色灵石，并输出相机坐标系下的三维目标点。
- `src/competition_perception/competition_perception/field_visualizer.py`：在 RViz 中显示赛场边界与机器人实时位置。
- `src/competition_perception/config/field.yaml`：赛场尺寸、区域边界、相机内参与视觉检测阈值。
- `src/competition_perception/launch/competition_stack.launch.py`：统一启动全部新节点。
- `src/competition_gateway/`：C++ 串口网关，独占串口并在 ROS 感知话题与 STM32 状态帧之间转换。
- `docs/development-plan.md`：环境、集成、部署、里程碑与 Git 协作流程。
- `docs/串口通信协议.md`：上位机与 STM32 之间的固定帧格式、校验规则和超时处理要求。

## 接口说明

| 方向 | 话题 | 类型 | 用途 |
| --- | --- | --- | --- |
| 输入 | `/fastlio2/lio_odom` | `nav_msgs/Odometry` | FAST-LIO 里程计输入 |
| 输出 | `/competition/field_pose` | `geometry_msgs/PoseStamped` | `competition_field` 坐标系下的机器人位姿 |
| 输出 | `/competition/current_zone` | `std_msgs/String` | 当前赛场区域或越界告警 |
| 输入 | `/camera/color/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| 输入 | `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| 输出 | `/competition/vision/target` | `geometry_msgs/PointStamped` | 相机坐标约定下的灵石三维点：前、左、上 |
| 输出 | `/competition/vision/overlay` | `sensor_msgs/Image` | 供操作员查看的识别叠加图 |
| 输出 | `/competition/field_markers` | `visualization_msgs/MarkerArray` | RViz 赛场显示 |
| 输出 | `/competition/serial/controller_connected` | `std_msgs/Bool` | STM32 状态帧在线状态 |
| 输出 | `/competition/serial/controller_state` | `std_msgs/UInt8` | STM32 状态机状态 |
| 输出 | `/competition/serial/controller_error` | `std_msgs/UInt8` | STM32 错误码 |

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd /home/husky/下载/last_car-main
colcon build --base-paths new_src --packages-select competition_perception competition_gateway
source install/setup.bash
ros2 launch competition_perception competition_stack.launch.py
```

部署前必须标定 LiDAR 至车体的外参、赛场航向偏置、RGB-D 相机内参，以及 `field.yaml` 中的所有区域边界。

串口设备、波特率、发送周期和超时时间配置在 [serial_gateway.yaml](src/competition_gateway/config/serial_gateway.yaml)。完整帧定义见 [串口通信协议.md](docs/串口通信协议.md)。
