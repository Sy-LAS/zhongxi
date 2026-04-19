#!/usr/bin/env python3

"""
自主探索节点
实现Coverage Path Planning算法，让机器人遍历所有可达区域
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Header
import numpy as np
import math
import time
from threading import Lock

class ExploreNode(Node):
    def __init__(self):
        super().__init__('explore_node')
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅激光雷达数据
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 订阅地图数据
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # 订阅探索完成状态
        self.exploration_status_sub = self.create_subscription(Bool, '/exploration_complete', self.exploration_status_callback, 1)
        
        # 控制参数
        self.linear_speed = 0.3  # 前进速度
        self.angular_speed = 0.5  # 转向速度
        self.safe_distance = 0.5  # 安全距离
        self.min_front_clearance = 0.8  # 前方最小净空
        self.rotation_angle = math.pi / 2  # 旋转角度 (90度)
        
        # 状态变量
        self.obstacle_detected = False
        self.current_scan = None
        self.current_map = None
        self.exploration_complete = False
        self.map_mutex = Lock()
        
        # 探索状态
        self.state = "GRID_SCAN"  # GRID_SCAN, BOUNDARY_FOLLOW, AVOIDING
        self.forward_duration = 0
        self.last_move_time = time.time()
        
        # 用于网格扫描的参数
        self.grid_scan_direction = 1  # 1为正向，-1为反向
        self.grid_scan_row = 0  # 当前行号
        self.grid_scan_col = 0  # 当前列号
        self.grid_size = 1.0  # 网格大小（米）
        self.next_turn_time = 0  # 下次转向时间
        self.turn_direction = 1  # 转向方向
        
        # 用于边界跟踪的参数
        self.following_boundary = False
        self.boundary_start_time = 0
        self.boundary_follow_duration = 5.0  # 边界跟踪持续时间
        
        # 创建定时器，定期发送控制命令
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('自主探索节点已启动，使用网格扫描算法')

    def scan_callback(self, msg):
        """处理激光扫描数据"""
        self.current_scan = msg
        # 检查前方是否有障碍物
        if msg.ranges:
            # 检查前方±30度范围内的障碍物
            front_ranges = []
            for i, angle in enumerate(np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))):
                if abs(angle) <= math.pi / 6:  # ±30度
                    if msg.ranges[i] < msg.range_max and msg.ranges[i] > msg.range_min:
                        front_ranges.append(msg.ranges[i])
            
            if front_ranges:
                min_range = min(front_ranges)
                self.obstacle_detected = min_range < self.safe_distance
            else:
                self.obstacle_detected = False
        else:
            self.obstacle_detected = False

    def map_callback(self, msg):
        """处理地图数据"""
        with self.map_mutex:
            self.current_map = msg

    def exploration_status_callback(self, msg):
        """处理探索完成状态"""
        self.exploration_complete = msg.data
        if self.exploration_complete:
            self.get_logger().info('探索完成！停止移动...')
            # 发送停止命令
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)

    def control_loop(self):
        """主控制循环"""
        if self.exploration_complete:
            # 探索完成，停止移动
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            return

        cmd_msg = Twist()
        
        if self.obstacle_detected:
            # 遇到障碍物，执行避障
            self.execute_avoidance(cmd_msg)
        else:
            # 正常探索
            self.execute_exploration(cmd_msg)
        
        # 发送命令
        self.cmd_vel_pub.publish(cmd_msg)

    def execute_exploration(self, cmd_msg):
        """执行探索行为 - 使用网格扫描算法"""
        if self.state == "GRID_SCAN":
            # 实施网格扫描算法
            cmd_msg.linear.x = self.linear_speed
            cmd_msg.angular.z = 0.0
            self.forward_duration += 0.1
            self.last_move_time = time.time()
            
            # 每前进一定距离就转向
            if self.forward_duration >= self.grid_size / self.linear_speed:
                self.state = "TURNING"
                self.turn_direction = self.grid_scan_direction
                self.next_turn_time = time.time()
                
        elif self.state == "TURNING":
            # 转向以进入下一行
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.angular_speed * self.turn_direction
            
            # 检查是否转了90度
            if time.time() - self.next_turn_time > 1.5:  # 大约90度转向所需时间
                self.state = "GRID_SCAN"
                self.forward_duration = 0
                self.grid_scan_direction *= -1  # 改变下一行的扫描方向
                
        elif self.state == "BOUNDARY_FOLLOW":
            # 边界跟踪行为
            cmd_msg.linear.x = self.linear_speed * 0.5  # 减慢速度以更好地跟踪边界
            cmd_msg.angular.z = self.angular_speed * 0.3  # 轻微转向以跟随边界
            self.last_move_time = time.time()
            
            # 检查是否需要切换回网格扫描
            if time.time() - self.boundary_start_time > self.boundary_follow_duration:
                self.state = "GRID_SCAN"
                self.forward_duration = 0

    def should_change_direction(self):
        """判断是否应该改变方向以探索新区域"""
        # 如果长时间没有移动（卡住了），则改变方向
        if time.time() - self.last_move_time > 10.0:
            return True
            
        # 在网格扫描算法中，我们不再使用随机转向
        # 而是按照固定的网格模式进行系统性扫描
        return False

    def execute_avoidance(self, cmd_msg):
        """执行避障行为"""
        self.state = "AVOIDING"
        
        # 简单的避障：停止前进，转向
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = self.angular_speed  # 转向以避开障碍物
        self.last_move_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    explore_node = ExploreNode()
    
    try:
        rclpy.spin(explore_node)
    except KeyboardInterrupt:
        explore_node.get_logger().info('接收到中断信号，正在停止...')
    finally:
        # 停止机器人
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        explore_node.cmd_vel_pub.publish(stop_msg)
        explore_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()