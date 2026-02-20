#!/bin/bash

# 启动传感器节点的脚本
# 作者: Zhongxi Robot Team
# 日期: 2026-02-18

echo "==========================================="
echo "    Zhongxi Robot - 传感器启动脚本"
echo "==========================================="

# 设置工作目录
WORKSPACE="/home/chuil/Desktop/zhongxi"

# 检查工作目录是否存在
if [ ! -d "$WORKSPACE" ]; then
    echo "错误: 工作目录不存在: $WORKSPACE"
    exit 1
fi

# 检查ROS2环境是否已设置
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS2环境未设置，请先执行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查安装目录
if [ ! -d "$WORKSPACE/install" ]; then
    echo "错误: 安装目录不存在，请先构建项目: colcon build"
    exit 1
fi

echo "设置工作空间..."
cd $WORKSPACE
source install/setup.bash

echo "启动传感器节点..."

# 启动摄像头和激光雷达节点
echo "启动摄像头驱动..."
gnome-terminal --tab --title="Camera Driver" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    ros2 run sensor_interfaces camera_driver;
    exec bash" &

sleep 2

echo "启动激光雷达驱动..."
gnome-terminal --tab --title="Lidar Driver" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    ros2 run sensor_interfaces lidar_driver;
    exec bash" &

echo "==========================================="
echo "传感器节点已启动"
echo "摄像头话题: /camera/image_raw"
echo "激光雷达话题: /scan"
echo "==========================================="

# 显示活跃节点
echo "当前ROS2节点:"
sleep 3
ros2 node list
echo "==========================================="

wait