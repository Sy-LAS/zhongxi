#!/bin/bash

# 设置工作目录
WORKSPACE="/home/chuil/Desktop/zhongxi"

echo "==========================================="
echo "    Zhongxi Robot - 系统启动脚本"
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

# 设置环境变量，禁用FastDDS共享内存传输
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=""
export FASTDDS_SHAREDMEM_ENV=OFF

echo "设置工作空间..."
cd $WORKSPACE
source install/setup.bash

echo "启动传感器节点..."

# 启动摄像头和激光雷达节点
echo "启动摄像头驱动..."
gnome-terminal --tab --title="Camera Driver" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
    ros2 run sensor_interfaces camera_driver;
    exec bash" &

sleep 2

echo "启动激光雷达驱动..."
gnome-terminal --tab --title="Lidar Driver" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
    ros2 run sensor_interfaces lidar_driver;
    exec bash" &

sleep 2

echo "启动RViz2..."
gnome-terminal --tab --title="RViz2" -- bash -c "
    source /opt/ros/humble/setup.bash;
    source install/setup.bash;
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
    rviz2 -d $(realpath $(find $WORKSPACE/src/sensor_interfaces -name '*.rviz' | head -n1));
    exec bash" &

echo "==========================================="
echo "系统已启动"
echo "摄像头话题: /camera/image_raw"
echo "激光雷达话题: /scan"
echo "==========================================="

# 显示活跃节点
echo "当前ROS2节点:"
sleep 3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 node list
echo "==========================================="

wait