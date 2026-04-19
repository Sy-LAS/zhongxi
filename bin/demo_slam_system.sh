#!/bin/bash

# SLAM系统演示脚本 - 包含完整的探索和地图管理功能
echo "==================================="
echo "   GraphSLAM 机器人自主探索系统"
echo "==================================="
echo ""

# 检查ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ 已加载ROS2 Humble环境"
else
    echo "错误：找不到ROS2 Humble环境"
    exit 1
fi

# 进入项目目录
cd /home/chuil/Desktop/zhongxi || { echo "错误：无法进入项目目录"; exit 1; }

# 检查项目环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ 已加载项目环境"
else
    echo "错误：找不到项目环境"
    exit 1
fi

# 设置ROS_DOMAIN_ID以避免与其他ROS2系统冲突
export ROS_DOMAIN_ID=24

# 创建地图保存目录
MAP_DIR="/home/chuil/Desktop/maps"
mkdir -p "$MAP_DIR"
echo "✓ 地图保存目录: $MAP_DIR"

echo ""
echo "硬件配置："
echo "  - Jetson Orin Nano Super 8G"
echo "  - YDLidar X3 Pro 激光雷达"
echo "  - C100 USB摄像头"
echo "  - 4个麦克纳姆轮 + JGA25-370电机"
echo "  - STM32F407VET6 + TB6612驱动板"
echo ""

echo ""
echo "系统启动中..."

# 检查必要的设备是否存在
echo "检查设备连接..."
if [ ! -e "/dev/ydlidar" ] && [ ! -e "/dev/ttyUSB0" ]; then
    echo "⚠️  未找到激光雷达设备，使用模拟模式"
else
    echo "✓ 激光雷达设备已找到"
fi

if [ ! -e "/dev/video0" ]; then
    echo "⚠️  未找到摄像头设备"
else
    echo "✓ 摄像头设备已找到"
fi

if [ ! -e "/dev/ttyUSB0" ]; then
    echo "⚠️  未找到STM32控制板设备"
else
    echo "✓ STM32控制板设备已找到"
fi

echo ""
echo "启动系统组件..."

# 使用独立终端窗口启动各个组件
echo "1. 启动传感器接口..."
gnome-terminal --title="传感器接口" -- ros2 launch sensor_interfaces sensors.launch.py &

sleep 2

echo "2. 启动SLAM后端优化系统..."
gnome-terminal --title="SLAM核心" -- ros2 run graphslam graphslam_node &

sleep 1

echo "3. 启动地图管理节点..."
gnome-terminal --title="地图管理" -- ros2 run graphslam map_manager_node &

sleep 1

echo "4. 启动地图保存节点..."
gnome-terminal --title="地图保存" -- ros2 run graphslam map_saver_node &

sleep 1

echo "5. 启动自主探索节点..."
gnome-terminal --title="自主探索" -- ros2 run graphslam explore_node &

sleep 1

echo "6. 启动机器人状态发布器..."
gnome-terminal --title="机器人状态" -- ros2 run robot_state_publisher robot_state_publisher &

sleep 1

echo "7. 启动坐标变换系统..."
gnome-terminal --title="坐标变换" -- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_link &

sleep 1

echo "8. 启动可视化界面..."
# 启动RViz2
if command -v rviz2 >/dev/null 2>&1; then
    gnome-terminal --title="RViz2" -- rviz2 -d /home/chuil/Desktop/zhongxi/src/graphslam/config/rviz_config.rviz &
    echo "   ✓ RViz2已启动"
else
    echo "   ⚠️  未找到RViz2"
fi

# 启动SLAM可视化界面
if [ -f "/home/chuil/Desktop/zhongxi/install/graphslam/lib/graphslam/graphslam_g2o" ]; then
    sleep 2
    gnome-terminal --title="SLAM可视化" -- /home/chuil/Desktop/zhongxi/install/graphslam/lib/graphslam/graphslam_g2o &
    echo "   ✓ SLAM可视化已启动"
else
    echo "   ⚠️  未找到SLAM可视化界面"
fi

echo ""
echo "==================================="
echo "   系统已启动！"
echo "==================================="
echo ""
echo "重要话题："
echo "  - /scan: 激光雷达数据"
echo "  - /odom: 里程计数据"
echo "  - /map: 生成的地图"
echo "  - /cmd_vel: 速度控制命令"
echo "  - /exploration_status: 探索状态"
echo "  - /saved_maps: 保存的地图列表"
echo ""
echo "操作说明："
echo "  1. 小车将自动开始探索环境"
echo "  2. SLAM系统实时构建地图"
echo "  3. 通过各个终端窗口查看组件状态"
echo "  4. 在RViz2中观察SLAM过程"
echo "  5. 探索完成后，地图将自动保存到 $MAP_DIR"
echo "  6. 关闭任意终端或按 Ctrl+C 停止特定组件"
echo "  7. 运行 pkill -f ros2 停止所有ROS2进程"
echo "==================================="

# 等待用户按键停止（可选）
echo ""
read -p "按任意键继续（系统在后台运行）..." -n1 -s
echo
echo "系统在后台运行中..."
echo "要停止系统，请使用 'pkill -f ros2' 命令"