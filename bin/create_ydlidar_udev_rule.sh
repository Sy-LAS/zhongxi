#!/bin/bash

echo "创建YDLidar X3 Pro的udev规则..."

# 创建udev规则文件
sudo tee /etc/udev/rules.d/99-ydlidar-x3pro.rules <<EOF
# YDLidar X3 Pro
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ydlidar", GROUP="dialout", MODE="0666"
EOF

echo "udev规则已创建，重新加载udev规则..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "规则已生效。现在你可以使用 /dev/ydlidar 作为设备路径。"
echo ""
echo "如果你要更新launch文件以使用新路径，请修改launch文件中的参数。"
echo "当前可用的设备路径："
ls -la /dev/ttyUSB* /dev/ydlidar 2>/dev/null || echo "没有找到 /dev/ydlidar 设备"