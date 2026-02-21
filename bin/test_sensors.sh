#!/bin/bash

echo "==========================================="
echo "传感器测试脚本"
echo "==========================================="

# 检查ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble环境已加载"
else
    echo "✗ 无法加载ROS2 Humble环境"
    exit 1
fi

# 检查项目环境
if [ -f "/home/chuil/Desktop/zhongxi/install/setup.bash" ]; then
    source /home/chuil/Desktop/zhongxi/install/setup.bash
    echo "✓ 项目环境已加载"
else
    echo "✗ 无法加载项目环境"
    exit 1
fi

# 检查设备权限
if [ -e "/dev/ttyUSB0" ]; then
    echo "✓ /dev/ttyUSB0 存在"
    if [ -r "/dev/ttyUSB0" ] && [ -w "/dev/ttyUSB0" ]; then
        echo "✓ /dev/ttyUSB0 权限正常"
    else
        echo "⚠ /dev/ttyUSB0 权限不足，尝试设置..."
        sudo chmod 666 /dev/ttyUSB0
    fi
else
    echo "✗ /dev/ttyUSB0 不存在"
fi

# 检查激光雷达驱动是否编译成功
if [ -f "/home/chuil/Desktop/zhongxi/install/sensor_interfaces/lib/sensor_interfaces/lidar_driver" ]; then
    echo "✓ 激光雷达驱动已编译"
else
    echo "✗ 激光雷达驱动未找到，重新构建..."
    cd /home/chuil/Desktop/zhongxi && colcon build --packages-select sensor_interfaces --symlink-install
    source /home/chuil/Desktop/zhongxi/install/setup.bash
fi

# 检查摄像头驱动是否编译成功
if [ -f "/home/chuil/Desktop/zhongxi/install/sensor_interfaces/lib/sensor_interfaces/camera_driver" ]; then
    echo "✓ 摄像头驱动已编译"
else
    echo "✗ 摄像头驱动未找到，重新构建..."
    cd /home/chuil/Desktop/zhongxi && colcon build --packages-select sensor_interfaces --symlink-install
    source /home/chuil/Desktop/zhongxi/install/setup.bash
fi

# 检查是否有其他程序占用设备
if command -v lsof &> /dev/null; then
    if [ -e "/dev/ttyUSB0" ]; then
        lsof_result=$(lsof /dev/ttyUSB0 2>/dev/null)
        if [ -n "$lsof_result" ]; then
            echo "⚠ /dev/ttyUSB0 被以下程序占用:"
            echo "$lsof_result"
        else
            echo "✓ /dev/ttyUSB0 未被其他程序占用"
        fi
    fi
fi

echo ""
echo "==========================================="
echo "测试激光雷达驱动（非阻塞模式）"
echo "==========================================="

# 启动激光雷达驱动并等待几秒后停止
timeout 10s ros2 run sensor_interfaces lidar_driver --ros-args -p port:=/dev/ttyUSB0 &
LIDAR_PID=$!

sleep 5

# 检查进程是否仍在运行
if ps -p $LIDAR_PID > /dev/null; then
    echo "激光雷达驱动正在运行，5秒后停止..."
    sleep 5
    kill $LIDAR_PID 2>/dev/null
else
    echo "激光雷达驱动可能已退出或出错"
fi

echo ""
echo "==========================================="
echo "检查ROS2话题"
echo "==========================================="

# 检查话题列表
echo "当前ROS2话题列表:"
ros2 topic list

echo ""
echo "如果scan话题存在，检查其类型:"
if ros2 topic list | grep -q "/scan"; then
    echo "scan话题类型:"
    ros2 topic type /scan
else
    echo "未发现scan话题"
fi

echo ""
echo "==========================================="
echo "传感器测试完成"
echo "==========================================="