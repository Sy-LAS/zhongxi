#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.timer = self.create_timer(0.5, self.tick)
        self.i = 0
    def tick(self):
        self.get_logger().info(f'Hello {self.i}')
        self.i += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
