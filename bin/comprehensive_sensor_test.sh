#!/bin/bash

# 综合传感器测试脚本
echo "==========================================="
echo "综合传感器测试脚本"
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

# 设置RMW实现为Cyclone DDS以避免FastDDS SHM错误
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "✓ 设置RMW_IMPLEMENTATION为Cyclone DDS"

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
    echo "⚠ /dev/ttyUSB0 不存在"
fi

# 检查激光雷达驱动是否编译成功
if [ -f "/home/chuil/Desktop/zhongxi/install/sensor_interfaces/lib/sensor_interfaces/lidar_driver" ]; then
    echo "✓ 激光雷达驱动已编译"
else
    echo "✗ 激光雷达驱动未找到，重新构建..."
    cd /home/chuil/Desktop/zhongxi && colcon build --packages-select sensor_interfaces --symlink-install
    source /home/chuil/Desktop/zhongxi/install/setup.bash
fi

# 启动激光雷达驱动并等待一段时间收集数据
echo ""
echo "==========================================="
echo "启动激光雷达驱动并测试数据获取..."
echo "==========================================="

# 启动激光雷达驱动
timeout 15s ros2 run sensor_interfaces lidar_driver --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p isSingleChannel:=true \
  -p resolution_fixed:=true \
  -p frequency:=5.0 \
  -p range_max:=12.0 \
  -p range_min:=0.1 \
  -p invalid_range_is_inf:=true \
  -p angle_max:=180.0 \
  -p angle_min:=-180.0 &
  
LIDAR_PID=$!

# 等待几秒让驱动启动
sleep 3

# 检查话题列表
echo ""
echo "==========================================="
echo "检查ROS2话题列表..."
echo "==========================================="
ros2 topic list

# 检查scan话题是否存在
if ros2 topic list | grep -q "/scan"; then
    echo "✓ /scan 话题存在"
    
    # 检查scan话题类型
    echo "话题类型:"
    ros2 topic type /scan
    
    # 尝试获取一些数据
    echo ""
    echo "尝试获取激光雷达数据..."
    timeout 5s ros2 topic echo /scan --field header.stamp --once
    if [ $? -eq 0 ]; then
        echo "✓ 成功接收到激光雷达数据"
    else
        echo "⚠ 未能接收到激光雷达数据"
    fi
else
    echo "⚠ /scan 话题不存在"
fi

# 清理进程
kill $LIDAR_PID 2>/dev/null
wait $LIDAR_PID 2>/dev/null

echo ""
echo "==========================================="
echo "传感器测试完成"
echo "==========================================="

# 检查是否需要重新启动以解决数据获取问题
echo ""
echo "提示: 如果仍然无法获取数据，请尝试以下操作："
echo "1. 断开并重新连接激光雷达设备"
echo "2. 检查供电是否稳定充足"
echo "3. 更换USB线缆"
echo "4. 使用YDLidar官方工具检查设备状态"