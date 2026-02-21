#!/bin/bash
# 启动完整的SLAM系统

echo "=== 启动完整SLAM系统 ==="

# 进入项目目录
cd /home/chuil/Desktop/zhongxi

# 检查并source环境
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "环境已加载"
else
    echo "错误: 未找到install/setup.bash，请先构建项目"
    exit 1
fi

# 设置ROS_DOMAIN_ID以避免与其他ROS2系统冲突
export ROS_DOMAIN_ID=24

echo "1. 启动传感器系统..."
# 启动传感器（激光雷达、摄像头、控制板）
ros2 launch sensor_interfaces sensors.launch.py &
SENSOR_PID=$!

# 等待传感器启动
sleep 5

echo "2. 启动SLAM系统..."
# 启动SLAM节点
ros2 run graphslam graphslam_node &
SLAM_PID=$!

# 等待SLAM节点启动
sleep 3

echo "3. 启动机器人状态发布器..."
# 启动机器人状态发布器
ros2 run robot_state_publisher robot_state_publisher &
ROBOT_STATE_PUB_PID=$!

# 等待状态发布器启动
sleep 2

echo "4. 启动TF转换..."
# 启动TF转换节点
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame &
TF_PID=$!

echo "5. 启动可视化界面..."
# 启动SLAM可视化界面（如果可用）
if command -v ros2 >/dev/null 2>&1 && ros2 pkg list | grep -q graphslam; then
    ros2 run graphslam graphslam_g2o &
    VISUALIZATION_PID=$!
else
    echo "   可视化界面不可用，跳过启动"
    VISUALIZATION_PID=""
fi

echo "6. 启动RViz2..."
# 启动RViz2可视化工具
if command -v rviz2 >/dev/null 2>&1; then
    rviz2 -d /home/chuil/Desktop/zhongxi/src/zhongxi_description/config/rviz_config.rviz &
    RVIZ_PID=$!
else
    echo "   RViz2未安装，跳过启动"
    RVIZ_PID=""
fi

echo "7. 系统已完全启动！"
echo "   - PID $SENSOR_PID: 传感器系统"
echo "   - PID $SLAM_PID: SLAM节点"
echo "   - PID $ROBOT_STATE_PUB_PID: 机器人状态发布器"
echo "   - PID $TF_PID: TF转换"
if [ -n "$VISUALIZATION_PID" ]; then
    echo "   - PID $VISUALIZATION_PID: SLAM可视化界面"
fi
if [ -n "$RVIZ_PID" ]; then
    echo "   - PID $RVIZ_PID: RViz2可视化"
fi
echo ""
echo "当前运行的ROS2节点:"
ros2 node list
echo ""
echo "当前活跃的话题:"
ros2 topic list
echo ""
echo "提示:"
echo "   - 按 Ctrl+C 停止整个系统"
echo "   - 运行 'ros2 topic echo /scan' 查看激光数据"
echo "   - 运行 'ros2 topic echo /odom' 查看里程计数据"
echo "   - 运行 'ros2 topic echo /map' 查看生成的地图"
echo ""

# 等待信号终止
trap "echo '停止所有进程...'; kill $SENSOR_PID $SLAM_PID $ROBOT_STATE_PUB_PID $TF_PID $VISUALIZATION_PID $RVIZ_PID 2>/dev/null; exit" SIGINT SIGTERM

# 等待后台进程
wait