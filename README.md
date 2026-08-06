# 比赛感知工作区

队内赛 B 组感知工作区

- **系统说明**：[系统说明.md](docs/系统说明.md)
- **感知层框架说明**：[感知层框架说明.md](docs/感知层框架说明.md)
- **串口通信协议**：[串口通信协议.md](docs/串口通信协议.md)
- **构建与运行**：见下文

## 快速开始

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select competition_perception competition_gateway ball_perception --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### 启动顺序

1. 启动 FAST-LIO 里程计与 RGB-D 相机
2. 启动 MID360 雷达
3. `ros2 launch competition_perception competition_stack.launch.py` - 比赛感知栈
4. `ros2 launch ball_perception perception_launch.py` - 球感知栈
5. `ros2 run competition_gateway serial_gateway` - 串口网关
