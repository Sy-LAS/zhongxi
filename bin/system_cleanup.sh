#!/bin/bash

echo "开始系统清理..."

# 清理ROS2相关进程
echo "停止所有ROS2相关进程..."
pkill -f ros
pkill -f rviz
pkill -f gazebo
pkill -f gz
pkill -f ignition
pkill -f graphslam
pkill -f python
pkill -f launch

sleep 2

# 清理僵尸进程
echo "清理僵尸进程..."
sudo pkill -9 -f ros
sudo pkill -9 -f rviz
sudo pkill -9 -f gazebo
sudo pkill -9 -f gz
sudo pkill -9 -f ignition
sudo pkill -9 -f graphslam
sudo pkill -9 -f python
sudo pkill -9 -f launch

# 清理共享内存
echo "清理共享内存..."
sudo rm -rf /dev/shm/ros*
sudo rm -rf /tmp/ros*

# 清理临时文件
echo "清理临时文件..."
sudo rm -rf /tmp/gazebo*
sudo rm -rf /tmp/ignition*
sudo rm -rf /tmp/.X*

# 清理系统缓存
echo "清理系统缓存..."
sudo apt autoremove -y
sudo apt autoclean
sudo apt clean

# 清理重复的ROS包
echo "清理重复的ROS包..."
sudo apt remove --purge '*ros*old*' '*ros*legacy*' 2>/dev/null || true

# 清理内存缓存
echo "清理内存缓存..."
sync
sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'

# 检查GPU内存使用情况
echo "GPU内存使用情况:"
nvidia-smi 2>/dev/null || echo "未检测到NVIDIA GPU"

# 重启桌面服务
echo "重启桌面服务..."
pkill -f xfce4-panel
pkill -f xfconfd
pkill -f xfsettingsd

sleep 2

# 清理日志
echo "清理日志文件..."
sudo find /var/log -name "*.log" -type f -size +100M -delete 2>/dev/null || true
sudo journalctl --vacuum-size=100M 2>/dev/null || true

# 显示系统状态
echo "系统状态:"
free -h
df -h

echo "系统清理完成！"
echo "建议重启桌面会话以确保所有更改生效"
echo "如果需要重启桌面会话，请运行: pkill -f xfce4-session && sleep 5 && startxfce4 &"
