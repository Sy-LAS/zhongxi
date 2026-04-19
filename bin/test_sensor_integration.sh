#!/bin/bash

# 启动仅传感器集成测试脚本，用于验证传感器集成和SLAM算法本身功能
echo "启动传感器集成测试..."

# 检查ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "已加载ROS2 Humble环境"
else
    echo "错误：找不到ROS2 Humble环境"
    exit 1
fi

# 检查项目环境
if [ -f "../install/setup.bash" ]; then
    source ../install/setup.bash
    echo "已加载项目环境"
else
    echo "错误：找不到项目环境"
    exit 1
fi

# 创建地图保存目录
mkdir -p /home/chuil/Desktop/maps

# 启动RViz2进行可视化（后台运行）
echo "启动RViz2..."
gnome-terminal -- ros2 run rviz2 rviz2 -d /home/chuil/Desktop/zhongxi/src/graphslam/config/rviz_config.rviz &

# 启动Gazebo仿真（如果需要）
echo "启动Ignition Gazebo仿真..."
gnome-terminal -- ign gazebo -s &

# 启动SLAM节点
echo "启动GraphSLAM节点..."
gnome-terminal -- ros2 run graphslam graphslam_node &

# 启动地图管理节点
echo "启动地图管理节点..."
gnome-terminal -- ros2 run graphslam map_manager_node &

# 启动地图保存节点
echo "启动地图保存节点..."
gnome-terminal -- ros2 run graphslam map_saver_node &

# 启动模拟运动节点（代替真实的探索节点，用于模拟机器人运动）
echo "启动模拟运动节点..."
cat << 'EOF' > /tmp/mock_motion_publisher.py
#!/usr/bin/env python3

"""
模拟运动发布节点
模拟机器人在环境中移动，用于测试SLAM算法
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class MockMotionPublisher(Node):
    def __init__(self):
        super().__init__('mock_motion_publisher')
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 发布里程计消息
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.update_position)
        
        # 初始位置
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 0.3  # rad/s
        
        # 记录开始时间
        self.start_time = time.time()
        
        self.get_logger().info('模拟运动发布节点已启动')

    def update_position(self):
        # 模拟机器人运动：圆形轨迹 + 随机转向
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # 创建速度消息
        twist_msg = Twist()
        
        # 简单的运动模式：直线运动，偶尔转向
        if int(elapsed) % 5 == 0:  # 每5秒转向
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_velocity
        else:
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0.0
            
        self.cmd_vel_pub.publish(twist_msg)
        
        # 更新位置（简单积分）
        dt = 0.1
        self.x += twist_msg.linear.x * math.cos(self.theta) * dt
        self.y += twist_msg.linear.x * math.sin(self.theta) * dt
        self.theta += twist_msg.angular.z * dt
        
        # 发布里程计
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # 将角度转换为四元数
        from math import sin, cos
        siny_cosp = sin(self.theta/2)
        cosy_cosp = cos(self.theta/2)
        odom_msg.pose.pose.orientation.z = siny_cosp
        odom_msg.pose.pose.orientation.w = cosy_cosp
        
        # 设置速度
        odom_msg.twist.twist.linear.x = twist_msg.linear.x
        odom_msg.twist.twist.angular.z = twist_msg.angular.z
        
        self.odom_pub.publish(odom_msg)
        
        # 发布TF变换
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = siny_cosp
        tf_msg.transform.rotation.w = cosy_cosp
        
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockMotionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到中断信号')
    finally:
        node.destroy_node()
        rclpy.shutdown()

EOF

# 启动模拟运动节点
chmod +x /tmp/mock_motion_publisher.py
gnome-terminal -- python3 /tmp/mock_motion_publisher.py &

echo "传感器集成测试系统已启动！"
echo "请在RViz2中观察SLAM过程"
echo "注意：机器人将以模拟轨迹运动，您可以手持机器人移动来测试激光雷达响应"

# 等待用户按键停止
read -p "按任意键停止所有进程..." -n1 -s
echo
echo "正在停止所有进程..."

# 杀死所有相关的ROS2进程
pkill -f ros2
pkill -f rviz2
pkill -f ign
pkill -f graphslam
pkill -f python3

echo "所有进程已停止"