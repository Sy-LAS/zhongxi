#!/bin/bash

# 设置工作目录
WORKSPACE="/home/chuil/Desktop/zhongxi"

echo "==========================================="
echo "    Zhongxi Robot - 使用FastDDS配置测试"
echo "==========================================="

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

# 设置环境变量，使用自定义FastDDS配置
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/config/fastdds_config.xml"

echo "设置工作空间..."
cd $WORKSPACE
source install/setup.bash

echo "启动激光雷达驱动..."
gnome-terminal --tab --title="Lidar Driver" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/fastdds_config.xml;
    ros2 run sensor_interfaces lidar_driver --ros-args -p port:=/dev/ydlidar;
    exec bash" &

sleep 5

echo "启动RViz2..."
gnome-terminal --tab --title="RViz2" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/fastdds_config.xml;
    rviz2;
    exec bash" &

sleep 3

echo "检查话题列表..."
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/config/fastdds_config.xml
ros2 topic list

echo "==========================================="
echo "系统已启动，使用自定义FastDDS配置"
echo "==========================================="

wait