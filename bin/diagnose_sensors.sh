#!/bin/bash

echo "==================================="
echo "传感器系统诊断脚本"
echo "==================================="

echo ""
echo "1. 检查设备权限和连接性"
echo "-----------------------------------"

# 检查激光雷达设备
if [ -e "/dev/ttyUSB0" ]; then
    echo "✓ /dev/ttyUSB0 存在"
    ls -la /dev/ttyUSB0
else
    echo "✗ /dev/ttyUSB0 不存在"
    echo "可用的串口设备:"
    ls -la /dev/tty{USB,S,*ACM*} 2>/dev/null || echo "未找到USB串口设备"
fi

# 检查摄像头设备
if [ -e "/dev/video0" ]; then
    echo "✓ /dev/video0 存在"
    ls -la /dev/video0
    v4l2-ctl --device /dev/video0 --info 2>/dev/null || echo "v4l2-ctl 未安装或摄像头不支持"
else
    echo "✗ /dev/video0 不存在"
    echo "可用的视频设备:"
    ls -la /dev/video* 2>/dev/null || echo "未找到视频设备"
fi

echo ""
echo "2. 检查用户权限"
echo "-----------------------------------"

if groups $USER | grep -q video; then
    echo "✓ 用户 $USER 在 video 组中"
else
    echo "✗ 用户 $USER 不在 video 组中"
    echo "请运行: sudo usermod -a -G video $USER"
fi

if [ -e "/dev/ttyUSB0" ] && [ -r "/dev/ttyUSB0" ] && [ -w "/dev/ttyUSB0" ]; then
    echo "✓ 对 /dev/ttyUSB0 有读写权限"
else
    echo "✗ 对 /dev/ttyUSB0 权限不足"
fi

echo ""
echo "3. 检查ROS2环境"
echo "-----------------------------------"

if command -v ros2 &> /dev/null; then
    echo "✓ ROS2 已安装"
    echo "ROS2 版本: $(ros2 --version)"
    echo "ROS_DISTRO: $ROS_DISTRO"
else
    echo "✗ ROS2 未安装或未正确设置"
fi

echo ""
echo "4. 检查项目构建状态"
echo "-----------------------------------"

if [ -f "/home/chuil/Desktop/zhongxi/install/setup.bash" ]; then
    echo "✓ 项目已构建 (install目录存在)"
else
    echo "✗ 项目未构建，请先运行 colcon build"
fi

echo ""
echo "5. 检查FastDDS共享内存端口错误"
echo "-----------------------------------"

echo "检查共享内存相关错误..."
if command -v dpkg &> /dev/null; then
    installed_shm=$(dpkg -l | grep -i shm | grep -i fastdds)
    if [ -n "$installed_shm" ]; then
        echo "已安装的FastDDS相关包:"
        echo "$installed_shm"
    else
        echo "未找到特定的FastDDS SHM包"
    fi
fi

echo ""
echo "6. 尝试运行单个传感器驱动进行测试"
echo "-----------------------------------"

echo "测试激光雷达驱动 (将在后台运行5秒)..."
timeout 5s bash -c "
    cd /home/chuil/Desktop/zhongxi && \
    source install/setup.bash && \
    LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib ./build/sensor_interfaces/lidar_driver --ros-args -p port:=/dev/ttyUSB0
" &
LIDAR_PID=$!
wait $LIDAR_PID
echo ""

echo "测试摄像头驱动 (将在后台运行5秒)..."
timeout 5s bash -c "
    cd /home/chuil/Desktop/zhongxi && \
    source install/setup.bash && \
    ./build/sensor_interfaces/camera_driver
" &
CAMERA_PID=$!
wait $CAMERA_PID
echo ""

echo ""
echo "7. 检查共享内存限制"
echo "-----------------------------------"
if [ -f /proc/sys/kernel/shmmax ]; then
    shmmax=$(cat /proc/sys/kernel/shmmax)
    shmall=$(cat /proc/sys/kernel/shmall)
    echo "共享内存限制:"
    echo "  shmmax: $shmmax bytes"
    echo "  shmall: $shmall pages"
fi

echo ""
echo "8. 诊断建议"
echo "-----------------------------------"
echo "a) 如果摄像头设备不存在，检查摄像头是否正确连接"
echo "b) 如果串口设备不存在，检查激光雷达是否正确连接"
echo "c) 如果权限不足，尝试: sudo chmod 666 /dev/ttyUSB0"
echo "d) 如果出现RTPS_SHM错误，尝试禁用共享内存传输:"
echo "   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "   或使用 Cyclone DDS:"
echo "   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
echo ""
echo "9. 完整的传感器系统测试"
echo "-----------------------------------"
echo "要测试完整系统，请运行:"
echo "  cd /home/chuil/Desktop/zhongxi && source install/setup.bash && ros2 launch sensor_interfaces sensors.launch.py"
echo ""
echo "要检查ROS2话题，请运行:"
echo "  source /home/chuil/Desktop/zhongxi/install/setup.bash && ros2 topic list"
echo "  source /home/chuil/Desktop/zhongxi/install/setup.bash && ros2 topic echo /scan"
echo ""