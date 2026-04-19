#!/bin/bash

# 一键启动GraphSLAM系统脚本
echo "==================================="
echo "   一键启动GraphSLAM系统"
echo "==================================="
echo ""
echo "硬件配置："
echo "  - Jetson Orin Nano Super 8G"
echo "  - YDLidar X3 Pro 激光雷达"
echo "  - C100 USB摄像头"
echo "  - 4个麦克纳姆轮 + JGA25-370电机"
echo "  - STM32F407VET6 + TB6612驱动板"
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
PROJECT_DIR="/home/chuil/Desktop/zhongxi"
cd $PROJECT_DIR || { echo "错误：无法进入项目目录"; exit 1; }

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

echo ""
echo "启动GraphSLAM系统..."

# 启动传感器接口（激光雷达等）
echo "1. 启动传感器接口..."
ros2 launch sensor_interfaces sensors.launch.py &
SENSOR_PID=$!

sleep 3

# 启动机器人状态发布器
echo "2. 启动机器人状态发布器..."
ros2 run robot_state_publisher robot_state_publisher &
ROBOT_STATE_PUB_PID=$!

sleep 1

# 启动SLAM系统
echo "3. 启动GraphSLAM系统..."
ros2 run graphslam graphslam_node &
SLAM_PID=$!

sleep 2

# 启动智能探索节点（使用改进的探索算法）
echo "4. 启动智能探索节点..."
ros2 run graphslam smart_explore_node &
SMART_EXPLORE_PID=$!

sleep 1

# 启动地图管理节点
echo "5. 启动地图管理节点..."
ros2 run graphslam map_manager_node &
MAP_MANAGER_PID=$!

sleep 1

# 启动地图保存节点
echo "6. 启动地图保存节点..."
ros2 run graphslam map_saver_node &
MAP_SAVER_PID=$!

sleep 1

# 启动强化的可视化界面
echo "7. 启动强化可视化界面..."
if command -v graphslam_g2o >/dev/null 2>&1; then
    graphslam_g2o &
    VISUALIZATION_PID=$!
    echo "   ✓ 强化可视化界面已启动"
else
    echo "   ⚠️  未找到graphslam_g2o可视化程序，启动普通RViz2"
    if command -v rviz2 >/dev/null 2>&1; then
        rviz2 &
        RVIZ_PID=$!
        echo "   ✓ RViz2已启动"
    else
        echo "   ⚠️  未找到RViz2"
    fi
fi

echo ""
echo "==================================="
echo "   GraphSLAM系统已启动！"
echo "==================================="
echo ""
echo "系统功能："
echo "  ✓ 传感器接口已启动"
echo "  ✓ 机器人状态发布器已启动"
echo "  ✓ GraphSLAM建图系统已启动"
echo "  ✓ 智能探索节点已启动"
echo "  ✓ 地图管理系统已启动"
echo "  ✓ 地图保存节点已启动"
echo "  ✓ 可视化界面已启动"
echo ""
echo "核心功能："
echo "  1. 小车将使用智能路径规划遍历可探索区域"
echo "  2. SLAM系统实时构建环境地图"
echo "  3. 通过前沿点选择实现最短路径覆盖"
echo "  4. 在可视化界面中观察SLAM过程和地图构建"
echo ""
echo "重要话题："
echo "  - /scan: 激光雷达数据"
echo "  - /odom: 里程计数据"
echo "  - /map: 生成的地图"
echo "  - /cmd_vel: 速度控制命令"
echo "  - /exploration_active: 探索活动状态"
echo "  - /current_goal: 当前目标点（用于可视化）"
echo ""
echo "操作说明："
echo "  - 小车将自动开始探索环境，使用最短路径覆盖尽可能多的区域"
echo "  - 在可视化界面中可查看实时地图构建过程"
echo "  - 探索完成后，地图将自动保存"
echo "  - 按 Ctrl+C 停止系统"
echo "==================================="

# 等待信号终止
trap "echo '停止所有进程...'; kill $SENSOR_PID $ROBOT_STATE_PUB_PID $SLAM_PID $SMART_EXPLORE_PID $MAP_MANAGER_PID $MAP_SAVER_PID $VISUALIZATION_PID $RVIZ_PID 2>/dev/null; exit" SIGINT SIGTERM

# 等待后台进程
wait