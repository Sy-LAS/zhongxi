#!/bin/bash

# 检查激光雷达数据的简短测试脚本
echo "启动激光雷达数据检查..."

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

# 检查是否有scan话题
echo "检查是否有scan话题..."
if ros2 topic list | grep -q "scan"; then
    echo "✓ 找到scan话题，尝试获取一条消息..."
    
    # 尝试获取一条scan消息
    if timeout 10s ros2 topic echo /scan sensor_msgs/msg/LaserScan --field header.stamp --once > /tmp/scan_output 2>&1; then
        echo "✓ 成功获取到激光雷达数据"
        echo "数据详情："
        cat /tmp/scan_output
    else
        echo "⚠ 未能获取到激光雷达数据，但话题存在"
    fi
else
    echo "⚠ 未找到scan话题"
    echo "当前可用话题："
    ros2 topic list
fi

# 清理进程
kill $LIDAR_PID 2>/dev/null
wait $LIDAR_PID 2>/dev/null

echo "激光雷达数据检查完成"