#!/bin/bash
# 启动完整的SLAM系统

echo "=== 启动完整SLAM系统 ==="

cd /home/chuil/Desktop/zhongxi

# 检查并source环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "环境已加载"
else
    echo "错误: 未找到install/setup.bash，请先构建项目"
    exit 1
fi

echo "启动完整SLAM系统..."
echo "参数:"
echo "  - use_sim_time:=false"
echo "  - enable_camera:=true"
echo "  - enable_lidar:=true"
echo "  - enable_slam:=true"
echo "  - enable_robot_state_publisher:=true"

# 启动完整系统
ros2 launch sensor_interfaces full_system.launch.py \
    use_sim_time:=false \
    enable_camera:=true \
    enable_lidar:=true \
    enable_slam:=true \
    enable_robot_state_publisher:=true

echo "系统已停止"