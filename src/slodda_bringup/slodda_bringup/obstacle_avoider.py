#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

# Terskelverdier
OBSTACLE_DIST = 0.25   # meter: under denne = hinder detektert
REVERSE_SPEED = -0.15  # m/s bakover
TURN_SPEED    =  0.6   # rad/s sving
FORWARD_SPEED =  0.2   # m/s fremover
REVERSE_TIME  =  0.8   # sekunder å rygge
TURN_TIME     =  1.0   # sekunder å svinge

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")

        self.sub_left   = self.create_subscription(LaserScan, "/ir_front_left",   self.cb_left,   10)
        self.sub_center = self.create_subscription(LaserScan, "/ir_front_center",  self.cb_center, 10)
        self.sub_right  = self.create_subscription(LaserScan, "/ir_front_right",   self.cb_right,  10)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.dist_left   = 9.9
        self.dist_center = 9.9
        self.dist_right  = 9.9

        # Kontroll-loop: 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = "FORWARD"
        self.state_start = self.get_clock().now()

        self.get_logger().info("ObstacleAvoider startet!")

    def min_range(self, msg):
        ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        return min(ranges) if ranges else 9.9

    def cb_left(self,   msg): self.dist_left   = self.min_range(msg)
    def cb_center(self, msg): self.dist_center  = self.min_range(msg)
    def cb_right(self,  msg): self.dist_right   = self.min_range(msg)

    def elapsed(self):
        return (self.get_clock().now() - self.state_start).nanoseconds / 1e9

    def set_state(self, new_state):
        self.state = new_state
        self.state_start = self.get_clock().now()
        self.get_logger().info(f"State: {new_state}")

    def publish(self, linear, angular):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    # Hoved state machine
    def control_loop(self):
        l = self.dist_left
        c = self.dist_center
        r = self.dist_right

        if self.state == "FORWARD":
            if c < OBSTACLE_DIST:
                self.set_state("REVERSE")
            elif l < OBSTACLE_DIST:
                self.set_state("TURN_RIGHT")
            elif r < OBSTACLE_DIST:
                self.set_state("TURN_LEFT")
            else:
                self.publish(FORWARD_SPEED, 0.0)

        elif self.state == "REVERSE":
            self.publish(REVERSE_SPEED, 0.0)
            if self.elapsed() > REVERSE_TIME:
                # Velg sving-retning basert på hvilken side som er mest åpen
                if l > r:
                    self.set_state("TURN_LEFT")
                else:
                    self.set_state("TURN_RIGHT")

        elif self.state == "TURN_LEFT":
            self.publish(0.0, TURN_SPEED)
            if self.elapsed() > TURN_TIME and c > OBSTACLE_DIST:
                self.set_state("FORWARD")

        elif self.state == "TURN_RIGHT":
            self.publish(0.0, -TURN_SPEED)
            if self.elapsed() > TURN_TIME and c > OBSTACLE_DIST:
                self.set_state("FORWARD")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
