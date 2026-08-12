# 比赛感知工作区

队内赛 B 组感知工作区

- **系统说明**：[系统说明.md](docs/系统说明.md)
- **感知层框架说明**：[感知层框架说明.md](docs/感知层框架说明.md)
- **串口通信协议**：[串口通信协议.md](docs/串口通信协议.md)
- **克隆与部署指南**：[克隆与部署指南.md](docs/克隆与部署指南.md)
- **构建与运行**：见下文

## 快速开始

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### 启动顺序

1. 启动 FAST-LIO 里程计与 RGB-D 相机
2. 启动 MID360 雷达
3. `ros2 launch common_launch_pkg common_launch.py` - 一键启动全部模块
4. `ros2 launch common_launch_pkg lidar_driver.launch.py` - 仅雷达驱动+里程计
5. `ros2 launch common_launch_pkg perception_nodes.launch.py` - 仅感知节点
6. `ros2 launch common_launch_pkg serial_gateway.launch.py` - 仅串口网关
