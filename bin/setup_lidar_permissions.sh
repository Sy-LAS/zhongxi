#!/bin/bash

echo "设置YDLidar X3 Pro设备权限..."

# 检查当前用户是否在dialout组中
if id -nG "$USER" | grep -qw "dialout"; then
    echo "用户已经在dialout组中"
else
    echo "将用户添加到dialout组..."
    sudo usermod -a -G dialout $USER
    echo "请重新登录以应用组权限更改，或运行: newgrp dialout"
fi

# 设置TTY设备权限
echo "设置TTY设备权限..."
sudo chmod 666 /dev/ttyUSB*

# 验证YDLidar设备是否存在
if [ -e "/dev/ttyUSB0" ]; then
    echo "检测到设备: /dev/ttyUSB0"
    ls -la /dev/ttyUSB0
else
    echo "警告: 未检测到 /dev/ttyUSB0 设备"
    echo "可用的TTY设备:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "无USB串口设备"
fi

echo "权限设置完成。"
echo ""
echo "如果仍然遇到问题，请尝试以下步骤："
echo "1. 重新插拔YDLidar设备"
echo "2. 运行: newgrp dialout (或重新登录)"
echo "3. 再次尝试运行激光雷达驱动"