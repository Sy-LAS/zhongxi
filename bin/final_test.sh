#!/bin/bash

echo "==========================================="
echo "YDLidar X3 Pro 综合测试脚本"
echo "==========================================="

echo "1. 检查设备权限..."
if [ -e "/dev/ydlidar" ]; then
    echo "设备存在: /dev/ydlidar"
    ls -la /dev/ydlidar
elif [ -e "/dev/ttyUSB0" ]; then
    echo "设备存在: /dev/ttyUSB0"
    ls -la /dev/ttyUSB0
    # 创建符号链接
    sudo ln -sf /dev/ttyUSB0 /dev/ydlidar
else
    echo "错误: 未找到激光雷达设备"
    echo "请检查激光雷达是否正确连接"
    exit 1
fi

# 设置权限
sudo chmod 666 /dev/ydlidar /dev/ttyUSB0 2>/dev/null || echo "权限设置可能需要root权限"

echo ""
echo "2. 启动传感器系统测试..."
cd /home/chuil/Desktop/zhongxi
source install/setup.bash

echo "启动所有传感器节点..."
timeout 45s ros2 launch sensor_interfaces sensors.launch.py

echo ""
echo "测试完成。"
echo ""
echo "如果激光雷达仍未正常工作，请尝试以下步骤："
echo "1. 确认激光雷达硬件连接是否牢固"
echo "2. 检查激光雷达是否被其他程序占用"
echo "3. 尝试使用官方YDLidar测试程序验证硬件"
echo "4. 检查激光雷达固件是否为最新版本"
echo ""
echo "要使用官方测试程序，请执行："
echo "cd ~/Desktop/zhongxi/src/sensor_interfaces/sdk/samples"
echo "./ydlidar_test"