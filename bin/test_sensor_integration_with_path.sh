#!/bin/bash

# 启动传感器集成测试脚本，包含路径规划和可视化功能
echo "启动传感器集成测试（带路径规划可视化）..."

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

# 启动RViz2进行可视化（使用带有路径可视化的配置）
echo "启动RViz2..."
gnome-terminal -- ros2 run rviz2 rviz2 -d /home/chuil/Desktop/zhongxi/src/graphslam/config/rviz_config_with_path.rviz &

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
cat << 'EOF' > /tmp/mock_motion_publisher_with_path.py
#!/usr/bin/env python3

"""
模拟运动发布节点
模拟机器人在环境中移动，用于测试SLAM算法和路径规划可视化
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time

class MockMotionPublisherWithPaths(Node):
    def __init__(self):
        super().__init__('mock_motion_publisher_with_path')
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 发布里程计消息
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 发布当前路径
        self.path_pub = self.create_publisher(Path, '/graphslam_path', 10)
        
        # 发布计划路径（用于可视化）
        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        
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
        
        # 记录路径
        self.path = Path()
        self.planned_path = Path()
        
        # 记录开始时间
        self.start_time = time.time()
        
        self.get_logger().info('模拟运动发布节点已启动（带路径可视化）')

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
        
        # 更新并发布路径
        self.update_and_publish_paths()
        
    def update_and_publish_paths(self):
        # 更新当前路径
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = 'map'  # 使用map坐标系
        current_pose.pose.position.x = self.x
        current_pose.pose.position.y = self.y
        current_pose.pose.position.z = 0.0
        
        from math import sin, cos
        siny_cosp = sin(self.theta/2)
        cosy_cosp = cos(self.theta/2)
        current_pose.pose.orientation.z = siny_cosp
        current_pose.pose.orientation.w = cosy_cosp
        
        self.path.poses.append(current_pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'map'
        self.path_pub.publish(self.path)
        
        # 生成模拟的计划路径（用于可视化，展示未来可能的路径）
        self.generate_planned_path()

    def generate_planned_path(self):
        # 清空计划路径
        self.planned_path.poses.clear()
        
        # 基于当前位置生成未来路径
        current_x = self.x
        current_y = self.y
        current_theta = self.theta
        
        # 计算未来10个点的路径
        for i in range(10):
            # 简单的前进路径
            next_x = current_x + 0.2 * math.cos(current_theta)
            next_y = current_y + 0.2 * math.sin(current_theta)
            
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = next_x
            pose.pose.position.y = next_y
            pose.pose.position.z = 0.0
            
            from math import sin, cos
            siny_cosp = sin(current_theta/2)
            cosy_cosp = cos(current_theta/2)
            pose.pose.orientation.z = siny_cosp
            pose.pose.orientation.w = cosy_cosp
            
            self.planned_path.poses.append(pose)
            
            # 更新位置
            current_x = next_x
            current_y = next_y
            
        self.planned_path.header.stamp = self.get_clock().now().to_msg()
        self.planned_path.header.frame_id = 'map'
        self.planned_path_pub.publish(self.planned_path)

def main(args=None):
    rclpy.init(args=args)
    node = MockMotionPublisherWithPaths()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('接收到中断信号')
    finally:
        node.destroy_node()
        rclpy.shutdown()

EOF

# 启动模拟运动节点
chmod +x /tmp/mock_motion_publisher_with_path.py
gnome-terminal -- python3 /tmp/mock_motion_publisher_with_path.py &

echo "传感器集成测试系统已启动！"
echo "请在RViz2中观察SLAM过程和路径规划可视化"
echo "注意：机器人将以模拟轨迹运动，您可以手持机器人移动来测试激光雷达响应"
echo "绿色路径：实际路径（由GraphSLAM生成）"
echo "红色路径：计划路径（用于可视化未来路线）"

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