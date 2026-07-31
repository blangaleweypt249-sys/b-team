# 开发与部署计划

## 环境准备

1. 安装 Ubuntu 22.04、ROS 2 Humble、`ros-humble-cv-bridge`、`ros-humble-message-filters`、OpenCV、NumPy、PCL 与 Livox 驱动。
2. 在启动本工作区前，完成 `livox_ros_driver2` 与 `fastlio2` 的构建和基础验证。
3. 安装深度相机驱动，并通过 `ros2 topic list` 确认彩色与深度图像话题存在。
4. 使用 `colcon build --base-paths new_src --packages-select competition_perception` 构建本包。

## 集成顺序

1. 启动 LiDAR 与 FAST-LIO，确认 `/fastlio2/lio_odom` 的时间戳稳定连续。
2. 将机器人停在启动区，等待 `field_localizer` 收集 30 帧初始化数据。
3. 实测赛场原点和朝向，在 `field.yaml` 中更新 `field_yaw_offset_deg` 与各区域边界。
4. 在存块区、公共区、跨越区与灵石区使用卷尺检查 `/competition/field_pose`。
5. 启动深度相机，填写相机内参，并以已知距离的金色球验证 `/competition/vision/target`。
6. 连接底盘或夹爪控制前，先在 RViz 中检查 `/competition/vision/overlay` 与 `/competition/field_markers`。
7. 只有在定位与目标精度达到验收标准后，才集成独立的任务管理节点。

## 验收标准

- 静态参考点的场地位姿误差小于 100 mm，完整巡航后的误差小于 150 mm。
- L1、L2、公共区、灵石拿取区与得分区的区域切换结果正确。
- 在合法抓取距离和比赛光照下，灵石目标识别成功率不低于 95%。
- 本包不产生运动控制指令；操作员控制和硬件急停必须保持独立的安全链路。

## 里程碑

| 周次 | 交付物 | 完成标准 |
| --- | --- | --- |
| 1 | 传感器启动与场地测绘 | 已记录 LiDAR、里程计、RGB 和深度话题 |
| 2 | 定位标定 | 检查点误差达到 100 mm 目标 |
| 3 | 灵石检测器标定 | 深度与视觉检测验收通过 |
| 4 | RViz 操作员视图与场地区域 | 操作员可快速识别区域和目标 |
| 5 | 任务管理节点集成 | 演练完成 L1/L2 抓取与灵石资格流程 |
| 6 | 全流程比赛演练 | 连续完成 3 次安全的 3 分钟运行 |

## Git 协作流程

仅在本新工作区中初始化版本库：

```bash
cd /home/husky/下载/last_car-main/new_src
git init
git add .
git commit -m "feat: 初始化比赛感知工作区"
```

使用短生命周期分支，例如 `feat/field-calibration` 和 `feat/gem-detector`。合并至 `main` 前必须通过代码审查和场地几何测试；不得提交自动生成的 `build`、`install`、`log` 目录。