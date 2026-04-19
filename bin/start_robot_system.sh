#!/bin/bash

# 启动机器人系统的脚本
# 该脚本将启动传感器、控制板接口和SLAM系统

# 导航到工作空间
cd /home/chuil/Desktop/zhongxi

# 检查ROS2环境是否已设置
if [ -z "$AMENT_PREFIX_PATH" ]; then
    source /opt/ros/humble/setup.bash
fi

# 检查工作空间是否已构建
if [ ! -d "install" ]; then
    echo "工作空间尚未构建，开始构建..."
    colcon build --packages-select sensor_interfaces graphslam
    source install/setup.bash
else
    echo "工作空间已构建，加载环境..."
    source install/setup.bash
fi

echo "启动机器人系统..."

# 启动完整系统
ros2 launch sensor_interfaces full_system.launch.py "$@"

echo "机器人系统已关闭"