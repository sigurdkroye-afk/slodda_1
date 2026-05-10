#!/usr/bin/env python3
"""Republish /scan to /slam_scan at 1 Hz.

At 10 Hz LiDAR and 5 Hz EKF, scans arrive between EKF updates and queue
in slam_toolbox's tf2 MessageFilter (size 10) waiting for a transform.
After 1 second the queue fills and scans are dropped.

At 1 Hz, each scan arrives well within a single 200 ms EKF window,
so the transform is always available and the queue never fills.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        self.declare_parameter('rate_hz', 1.0)
        rate = self.get_parameter('rate_hz').value
        self._min_interval = 1.0 / rate
        self._last = None
        self._pub = self.create_publisher(LaserScan, '/slam_scan', 10)
        self.create_subscription(LaserScan, '/scan', self._cb, 10)
        self.get_logger().info(f'ScanThrottle: /scan → /slam_scan at {rate:.1f} Hz')

    def _cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last is None or (now - self._last) >= self._min_interval:
            self._pub.publish(msg)
            self._last = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
