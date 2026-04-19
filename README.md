# 踵息

该项目是名为“踵息”的Graph SLAM（Simultaneous Localization and Mapping，即时定位与地图构建）探索系统。本项目旨在展示全栈的自主导航、传感器融合和路径规划的能力。

软件、硬件、设计纯单人实现

开发日志见https://github.com/Sy-LAS/yuanmo

## 硬件配置

- **计算平台**: Jetson Orin Nano Super 8G developer kit
- **激光雷达**: YDLidar X3 Pro
- **摄像头**: C100 USB摄像头
- **移动机构**: 4个麦克纳姆轮 + JGA25-370电机（400RPM, 12V）
- **控制器**: STM32F407VET6开发板
- **驱动器**:  电流驱动板，2个TB6612FNG（各驱动2编码电机）
- **电源管理**: LM2596降压稳压模块 + LTC1871升压稳压模块 + 大功率降压模块

## 软件环境

- **操作系统**: Ubuntu 22.04
- **机器人操作系统**: ROS2 Humble 
- **开发工具**: JetPack 6.2.2, Windows笔记本通过SSH + VSCode远程开发
- **仿真环境**: RViz2，Gazebo Sim 6.16.0
    %%urdf，description文件就绪，未实现真gazebo仿真%%

## 主要功能

1. **智能探索**: 机器人使用前沿点检测算法，自动寻找未探索的边界并规划最短路径前往
2. **SLAM建图**: 使用GraphSLAM算法实时构建环境地图
3. **传感器融合**: 整合激光雷达、摄像头、编码器等多种传感器数据（现在只有激光实机验证）
4. **高效路径规划**: 使用最短路径算法确保以最少的时间和路程覆盖所有可探索区域
5. **强化可视化**: 提供图形化界面，可直接查看小车路径、当前目标和构建的地图
6. **预留导航接口**: 为未来点击地图导航功能预留接口

## 系统架构

- `sensor_interfaces` 包: 处理传感器数据（激光雷达、摄像头、编码器等）
- `graphslam` 包: GraphSLAM算法实现和智能探索节点
- `zhongxi_description` 包: 机器人URDF模型和仿真环境

## 快速启动

```bash
./bin/sensors.sh
```

## 项目目标

本项目主要目标是实现一个功能齐全的简单版本，展示完整的开发流程：

1. 一键启动系统后，小车使用智能路径规划遍历所有可达区域并构建地图
2. 使用前沿点检测和最短路径算法，以最少的时间和路程绘制出可靠的完整地图
3. 预留未来点击地图导航接口，允许用户在RViz2中点击地图任意位置，小车自动规划安全路径前往
4. 展示从硬件选型、软件架构设计到算法实现的完整开发流程
5. 预备未来加上CV、语音模块，实现正态化指令——说：“椅子，3米”，车跑到椅子3m内

## 技术细节

智能探索算法使用前沿点检测技术，自动识别已知地图和未知区域之间的边界点。然后，系统会选择最近的前沿点作为下一个目标，通过路径规划算法前往该点。这种方法确保了机器人能够以最有效的方式覆盖所有可探索区域，而不是盲目地遍历所有地方。

## 现状
1. 实现.bin/sensor.sh启动LiDAR,rviz2，并在rviz2中看到红色激光点云，这也是预期路线
    - 有关于绿色点云指示实际路线的实现，因**电流问题**电机编码器未正常运行
    - 大功率变压器输出电流0.02A,stm32引脚电压有问题
2. 上一个版本有实现camera在rviz2的内嵌（给人看，车不识别），但修改后此功能失去，暂不知原因
3. gazebo未实现

## 注意事项
- 所有一次性的脚本均放置在 `bin` 目录中（现在过于杂乱）
- 本项目重点在于体验开发全流程，而非追求工业级解决方案

## 扩展计划
- 完善点击地图导航功能
- 预备未来加上CV、语音模块，实现正态化指令——说：“椅子，3米”，车跑到椅子3m内

## tutorials
algorithm文件夹中以overview.md为概览，较详细介绍核心算法实现
1. ICP
2. 特征匹配算法 (Feature Matching Algorithm)
3. 差速驱动运动模型 (Differential Drive Motion Model) 
4. 图优化算法 (Graph Optimization Algorithm)
5. 射线追踪算法 (Ray Tracing Algorithm)
6. 特征地图管理算法 (Feature Map Management Algorithm)
7. 运动补偿算法 (Motion Compensation Algorithm)
8. 前沿探索算法 (Frontier Exploration Algorithm)
9. 最短路径覆盖算法 (Shortest Path Coverage Algorithm)
10. 基于距离的回环检测 (Distance-based Loop Detection)
11. 滑动窗口轨迹平滑 (Sliding Window Trajectory Smoothing)

