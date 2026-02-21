·#!/bin/bash

# 简化版传感器测试脚本，用于验证传感器是否正常工作
echo "启动简化版传感器测试..."

# 检查ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ 已加载ROS2 Humble环境"
else
    echo "✗ 错误：找不到ROS2 Humble环境"
    exit 1
fi

# 检查项目环境
if [ -f "/home/chuil/Desktop/zhongxi/install/setup.bash" ]; then
    source /home/chuil/Desktop/zhongxi/install/setup.bash
    echo "✓ 已加载项目环境"
else
    echo "✗ 错误：找不到项目环境"
    exit 1
fi

# 启动激光雷达驱动（在后台）
echo "启动激光雷达驱动..."
ros2 run sensor_interfaces lidar_driver --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p isSingleChannel:=true -p resolution_fixed:=true -p frequency:=7.0 -p range_max:=8.0 -p range_min:=0.12 &
LIDAR_PID=$!

# 等待几秒让驱动启动
sleep 5

# 检查ROS话题
echo "检查ROS话题..."
TOPICS=$(ros2 topic list)
echo "$TOPICS"

if echo "$TOPICS" | grep -q "/scan"; then
    echo "✓ 激光雷达数据话题(/scan)已找到"
    
    # 检查话题类型
    echo "话题类型:"
    ros2 topic type /scan
    
    # 等待并尝试接收一条消息
    echo "等待激光雷达数据..."
    timeout 10s ros2 topic echo /scan --field header.stamp --once 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "✓ 成功接收到激光雷达数据"
    else
        echo "⚠ 未能接收到激光雷达数据"
    fi
else
    echo "⚠ 激光雷达数据话题(/scan)未找到"
fi

# 清理进程
kill $LIDAR_PID 2>/dev/null
wait $LIDAR_PID 2>/dev/null

echo "简化版传感器测试完成"