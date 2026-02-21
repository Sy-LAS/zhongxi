#!/bin/bash

echo "==========================================="
echo "YDLidar X3 Pro 测试脚本"
echo "==========================================="

echo "1. 检查设备权限..."
if [ -e "/dev/ttyUSB0" ]; then
    echo "设备存在: /dev/ttyUSB0"
    ls -la /dev/ttyUSB0
    # 设置权限
    sudo chmod 666 /dev/ttyUSB0
    echo "已设置权限为666"
else
    echo "错误: 未找到 /dev/ttyUSB0 设备"
    echo "请检查激光雷达是否正确连接"
    exit 1
fi

echo ""
echo "2. 运行激光雷达驱动测试..."
cd /home/chuil/Desktop/zhongxi
source install/setup.bash
timeout 30s ./build/sensor_interfaces/lidar_driver --ros-args -p port:=/dev/ttyUSB0 &

LIDAR_PID=$!
sleep 25  # 等待25秒让驱动运行

# 结束进程
kill $LIDAR_PID 2>/dev/null

echo ""
echo "测试完成。如果仍有问题，请尝试："
echo "1. 重新插拔激光雷达"
echo "2. 确保没有其他程序在使用该设备"
echo "3. 运行: sudo usermod -a -G dialout \$USER"
echo "   然后重新登录系统"
echo "4. 检查硬件连接是否稳固"