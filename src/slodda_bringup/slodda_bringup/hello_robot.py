#!/usr/bin/env python3
"""Hello Robot — testnode som publiserer cmd_vel for å verifisere oppsett."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HelloRobot(Node):
    def __init__(self):
        super().__init__('hello_robot')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_command)
        self.count = 0
        
        self.get_logger().info('Hello Robot node started!')
        self.get_logger().info('   Publishing to /cmd_vel')
    
    def publish_command(self):
        msg = Twist()
        
        # Enkel oscillerende bevegelse
        if self.count % 4 < 2:
            msg.linear.x = 0.2
        else:
            msg.linear.x = -0.2
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing cmd_vel [count={self.count}]: linear.x={msg.linear.x:.2f}')
        
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Hello Robot node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
