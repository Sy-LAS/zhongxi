#!/bin/bash

# 自主导航探索和建图脚本
echo "=== 启动自主探索和建图系统 ==="

# 进入项目目录
cd /home/chuil/Desktop/zhongxi

# 源码环境设置
source install/setup.bash

echo "1. 启动传感器..."
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

echo "4. 启动自主探索节点..."
# 启动自主探索节点（这里是一个模拟的探索行为，实际实现需要另外开发）
# 我们使用一个简单的脚本来发布随机运动命令
cat > /tmp/explore_commands.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import time
import random

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.explore_callback)
        self.stop_count = 0
        self.moving_forward = True
        self.turn_direction = 1
        self.get_logger().info('Auto Explorer Started')

    def explore_callback(self):
        msg = Twist()
        
        # 随机探索策略：
        # 大部分时间向前移动
        # 遇到障碍物(通过激光数据判断)时转向
        # 定期改变方向以探索更多区域
        
        if self.moving_forward:
            msg.linear.x = 0.3  # 向前移动
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.turn_direction  # 转向
            
        # 发布运动命令
        self.publisher.publish(msg)
        
        # 简单的避障逻辑模拟
        self.stop_count += 1
        if self.stop_count > 50:  # 每5秒改变一次方向
            self.stop_count = 0
            if random.random() < 0.3:  # 30%概率转向
                self.moving_forward = False
                self.turn_direction = 1 if random.random() > 0.5 else -1
            else:
                self.moving_forward = True


def main(args=None):
    rclpy.init(args=args)
    explorer = AutoExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('Shutting down Auto Explorer...')
    finally:
        # 停止机器人
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        explorer.publisher.publish(stop_msg)
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# 启动探索节点
python3 /tmp/explore_commands.py &
EXPLORE_PID=$!

echo "5. 系统已启动，小车开始自主探索环境..."
echo "   - PID $SENSOR_PID: 传感器系统"
echo "   - PID $SLAM_PID: SLAM节点"
echo "   - PID $ROBOT_STATE_PUB_PID: 机器人状态发布器"
echo "   - PID $EXPLORE_PID: 自主导航探索"
echo ""
echo "提示:"
echo "   - 运行 'rviz2' 查看实时地图构建"
echo "   - 运行 'ros2 topic echo /scan' 查看激光数据"
echo "   - 运行 'ros2 topic echo /odom' 查看里程计数据"
echo "   - 按 Ctrl+C 停止系统"
echo ""

# 等待信号终止
trap "echo '停止所有进程...'; kill $SENSOR_PID $SLAM_PID $ROBOT_STATE_PUB_PID $EXPLORE_PID 2>/dev/null; exit" SIGINT SIGTERM

# 等待后台进程
wait