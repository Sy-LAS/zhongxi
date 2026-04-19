#!/bin/bash

# 测试激光雷达数据持续性的脚本
echo "启动激光雷达数据连续性测试..."

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

# 设置RMW实现为Cyclone DDS以避免FastDDS SHM错误
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# 启动激光雷达驱动（在后台）
echo "启动激光雷达驱动..."
ros2 run sensor_interfaces lidar_driver --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p isSingleChannel:=true -p resolution_fixed:=true -p frequency:=7.0 -p range_max:=8.0 -p range_min:=0.12 &
LIDAR_PID=$!

# 等待几秒让驱动启动
sleep 5

# 检查是否有scan话题
echo "检查是否有scan话题..."
if ros2 topic list | grep -q "scan"; then
    echo "✓ 找到scan话题，开始监听数据..."
    
    # 监听scan话题10秒
    timeout 10s ros2 topic echo /scan --field ranges > /tmp/scan_data.log 2>&1 &
    LISTENER_PID=$!
    
    # 等待监听完成
    wait $LISTENER_PID 2>/dev/null
    
    # 检查日志内容
    if [ -f /tmp/scan_data.log ]; then
        DATA_LINES=$(wc -l < /tmp/scan_data.log)
        if [ $DATA_LINES -gt 5 ]; then
            echo "✓ 成功获取到激光雷达数据，共 $DATA_LINES 行"
            
            # 显示最后几行数据
            echo "最近的激光雷达数据片段："
            tail -10 /tmp/scan_data.log
        else
            echo "⚠ 只获取到少量数据或无数据"
            cat /tmp/scan_data.log
        fi
    else
        echo "⚠ 未能创建数据日志文件"
    fi
else
    echo "⚠ 未找到scan话题"
    echo "当前可用话题："
    ros2 topic list
fi

# 清理进程
kill $LIDAR_PID 2>/dev/null
wait $LIDAR_PID 2>/dev/null

echo "激光雷达数据连续性测试完成"