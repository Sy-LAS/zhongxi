#!/usr/bin/env python3

"""
智能探索节点
实现更高效的探索算法，使用最短路径覆盖尽可能多的区域
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float32
import numpy as np
import math
import time
from threading import Lock
from scipy.spatial import distance

class SmartExploreNode(Node):
    def __init__(self):
        super().__init__('smart_explore_node')
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅激光雷达数据
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 订阅地图数据
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # 订阅里程计数据
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # 发布探索状态
        self.status_pub = self.create_publisher(Bool, '/exploration_active', 10)
        
        # 发布目标点（用于可视化）
        self.goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        
        # 控制参数
        self.linear_speed = 0.3  # 前进速度
        self.angular_speed = 0.5  # 转向速度
        self.safe_distance = 0.5  # 安全距离
        self.min_front_clearance = 0.8  # 前方最小净空
        
        # 状态变量
        self.obstacle_detected = False
        self.current_scan = None
        self.current_map = None
        self.current_odom = None
        self.exploration_active = True
        self.map_mutex = Lock()
        
        # 探索状态
        self.state = "FRONTIER_EXPLORATION"  # FRONTIER_EXPLORATION, PATH_FOLLOWING, OBSTACLE_AVOIDANCE
        self.last_move_time = time.time()
        
        # 用于前沿探索的参数
        self.frontier_points = []  # 存储前沿点
        self.target_frontier = None  # 目标前沿点
        self.has_reached_target = True  # 是否到达目标前沿点
        self.unexplored_ratio = 0.0  # 未探索区域比例
        self.last_frontier_update = 0  # 上次更新前沿的时间
        
        # 机器人位置
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # 创建定时器，定期发送控制命令
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('智能探索节点已启动，使用最短路径覆盖算法')

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
            
            # 计算未探索区域的比例
            total_cells = len(msg.data)
            unexplored_cells = sum(1 for cell in msg.data if cell == -1)  # -1表示未知区域
            self.unexplored_ratio = unexplored_cells / total_cells if total_cells > 0 else 0
            
            # 如果未探索区域少于5%，认为探索完成
            if self.unexplored_ratio < 0.05:
                self.exploration_active = False
                self.get_logger().info(f'探索完成！未探索区域比例: {self.unexplored_ratio:.2f}')
                
                # 发布探索完成消息
                status_msg = Bool()
                status_msg.data = False
                self.status_pub.publish(status_msg)
            else:
                # 检查前沿点，每秒最多更新一次
                current_time = time.time()
                if current_time - self.last_frontier_update > 1.0:
                    self.update_frontiers()
                    self.last_frontier_update = current_time

    def odom_callback(self, msg):
        """处理里程计数据"""
        self.current_odom = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def update_frontiers(self):
        """更新前沿点列表"""
        if not self.current_map:
            return
            
        # 简化的前沿检测：在地图上找到已知/未知边界的点
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        # 获取地图数据
        map_data = np.array(self.current_map.data).reshape((height, width))
        
        # 寻找前沿点（已知自由空间和未知空间的边界）
        frontiers = []
        for y in range(1, height-1):
            for x in range(1, width-1):
                if map_data[y][x] == 0:  # 已知自由空间
                    # 检查周围是否有未知区域
                    neighbors = [
                        map_data[y-1][x-1], map_data[y-1][x], map_data[y-1][x+1],
                        map_data[y][x-1], map_data[y][x+1],
                        map_data[y+1][x-1], map_data[y+1][x], map_data[y+1][x+1]
                    ]
                    
                    if -1 in neighbors:  # 周围有未知区域
                        world_x = origin_x + x * resolution
                        world_y = origin_y + y * resolution
                        frontiers.append((world_x, world_y))
        
        self.frontier_points = frontiers
        self.get_logger().info(f'找到 {len(frontiers)} 个前沿点')
        
        # 选择最近的前沿点作为目标
        if frontiers and self.robot_x is not None and self.robot_y is not None:
            # 计算到每个前沿点的距离
            distances = [math.sqrt((fx - self.robot_x)**2 + (fy - self.robot_y)**2) for fx, fy in frontiers]
            
            # 选择最近的前沿点
            min_idx = distances.index(min(distances))
            self.target_frontier = frontiers[min_idx]
            self.has_reached_target = False
            
            # 发布目标点用于可视化
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = self.target_frontier[0]
            goal_msg.pose.position.y = self.target_frontier[1]
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.w = 1.0
            self.goal_pub.publish(goal_msg)
            
            self.get_logger().info(f'选择前沿点作为目标: ({self.target_frontier[0]:.2f}, {self.target_frontier[1]:.2f})')

    def control_loop(self):
        """主控制循环"""
        if not self.exploration_active:
            # 探索完成，停止移动
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            
            # 发布停止状态
            status_msg = Bool()
            status_msg.data = False
            self.status_pub.publish(status_msg)
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
        
        # 发布探索状态
        status_msg = Bool()
        status_msg.data = self.exploration_active
        self.status_pub.publish(status_msg)

    def execute_exploration(self, cmd_msg):
        """执行探索行为 - 前往最近的前沿点"""
        if self.target_frontier is None:
            # 没有目标前沿点，原地旋转寻找
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.angular_speed
            return
        
        # 计算到目标的角度和距离
        target_x, target_y = self.target_frontier
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        dist_to_target = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # 获取机器人当前朝向
        if self.current_odom:
            quat = self.current_odom.pose.pose.orientation
            # 简化的四元数到欧拉角转换（仅适用于Z轴旋转）
            current_angle = math.atan2(2*(quat.w*quat.z + quat.x*quat.y), 
                                      1 - 2*(quat.y*quat.y + quat.z*quat.z))
        else:
            current_angle = 0.0
        
        # 计算角度差
        angle_diff = target_angle - current_angle
        # 将角度限制在 [-π, π] 范围内
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # 如果距离目标很近，则认为已到达
        if dist_to_target < 0.5:
            self.has_reached_target = True
            # 寻找下一个前沿点
            self.update_frontiers()
            if self.target_frontier is not None:
                target_x, target_y = self.target_frontier
                dx = target_x - self.robot_x
                dy = target_y - self.robot_y
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - current_angle
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
            else:
                # 没有更多前沿点了
                self.exploration_active = False
                return
        
        # 根据角度差调整方向，然后前进
        if abs(angle_diff) > 0.2:  # 11度以上，先转向
            cmd_msg.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            cmd_msg.linear.x = 0.0
        else:  # 方向正确，向前走
            cmd_msg.linear.x = self.linear_speed
            cmd_msg.angular.z = 0.0

    def execute_avoidance(self, cmd_msg):
        """执行避障行为"""
        self.state = "OBSTACLE_AVOIDANCE"
        
        # 避障：稍微后退然后转向
        cmd_msg.linear.x = -0.1  # 缓慢后退
        cmd_msg.angular.z = self.angular_speed  # 转向以避开障碍物
        self.last_move_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    explore_node = SmartExploreNode()
    
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