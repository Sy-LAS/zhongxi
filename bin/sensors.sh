#!/bin/bash

# 启动仅真实传感器集成测试脚本，用于验证传感器集成和SLAM算法
echo "启动真实传感器集成测试..."

# 检查ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "已加载ROS2 Humble环境"
else
    echo "错误：找不到ROS2 Humble环境"
    exit 1
fi

# 检查项目环境
if [ -f "/home/chuil/Desktop/zhongxi/install/setup.bash" ]; then
    source /home/chuil/Desktop/zhongxi/install/setup.bash
    echo "已加载项目环境"
else
    echo "错误：找不到项目环境"
    exit 1
fi

# 创建地图保存目录
mkdir -p /home/chuil/Desktop/maps

# 启动传感器节点（在后台运行）
echo "启动摄像头驱动..."
ros2 run sensor_interfaces camera_driver &
CAMERA_PID=$!

sleep 2

echo "启动激光雷达驱动..."
ros2 run sensor_interfaces lidar_driver --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p isSingleChannel:=true \
  -p resolution_fixed:=true \
  -p frequency:=5.0 \
  -p range_max:=12.0 \
  -p range_min:=0.1 \
  -p invalid_range_is_inf:=true &
LIDAR_PID=$!

sleep 2

# 检查传感器是否成功启动
echo "检查传感器状态..."
if ps -p $LIDAR_PID > /dev/null; then
    echo "✓ 激光雷达驱动正在运行 (PID: $LIDAR_PID)"
else
    echo "✗ 激光雷达驱动启动失败"
fi

if ps -p $CAMERA_PID > /dev/null; then
    echo "✓ 摄像头驱动正在运行 (PID: $CAMERA_PID)"
else
    echo "✗ 摄像头驱动启动失败"
fi

# 启动RViz2进行可视化（使用带有路径可视化的配置）
echo "启动RViz2..."
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- ros2 run rviz2 rviz2 -d /home/chuil/Desktop/zhongxi/src/graphslam/config/rviz_config_with_path.rviz &
else
    echo "警告：gnome-terminal不可用，跳过RViz2启动"
fi

# 启动SLAM节点
echo "启动GraphSLAM节点..."
ros2 run graphslam graphslam_node &
SLAM_PID=$!

sleep 2

# 检查SLAM节点是否成功启动
if ps -p $SLAM_PID > /dev/null; then
    echo "✓ GraphSLAM节点正在运行 (PID: $SLAM_PID)"
else
    echo "✗ GraphSLAM节点启动失败"
fi

# 启动地图管理节点
echo "启动地图管理节点..."
ros2 run graphslam map_manager_node &
MAP_MGR_PID=$!

sleep 2

# 检查地图管理节点是否成功启动
if ps -p $MAP_MGR_PID > /dev/null; then
    echo "✓ 地图管理节点正在运行 (PID: $MAP_MGR_PID)"
else
    echo "✗ 地图管理节点启动失败"
fi

# 启动地图保存节点
echo "启动地图保存节点..."
ros2 run graphslam map_saver_node &
MAP_SAVER_PID=$!

sleep 2

# 检查地图保存节点是否成功启动
if ps -p $MAP_SAVER_PID > /dev/null; then
    echo "✓ 地图保存节点正在运行 (PID: $MAP_SAVER_PID)"
else
    echo "✗ 地图保存节点启动失败"
fi

echo "所有节点已启动"
echo "按 Ctrl+C 停止所有进程..."

# 等待信号来终止所有子进程
cleanup() {
    echo "停止所有进程..."
    kill $CAMERA_PID $LIDAR_PID $SLAM_PID $MAP_MGR_PID $MAP_SAVER_PID 2>/dev/null
    wait $CAMERA_PID $LIDAR_PID $SLAM_PID $MAP_MGR_PID $MAP_SAVER_PID 2>/dev/null
    echo "所有进程已停止"
    exit 0
}

trap cleanup SIGINT SIGTERM

# 等待所有后台进程结束
wait $CAMERA_PID $LIDAR_PID $SLAM_PID $MAP_MGR_PID $MAP_SAVER_PID