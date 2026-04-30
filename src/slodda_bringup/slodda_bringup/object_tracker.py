#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        self.bridge = CvBridge()
        self.enabled = False

        self.K_ang = 0.8
        self.fwd_speed = 0.15
        self.min_area = 300

        # Red wraps around 0/180 in HSV — two ranges needed
        self.lower1 = np.array([0, 120, 70])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 120, 70])
        self.upper2 = np.array([180, 255, 255])

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(Bool, '/track/enable', self.enable_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('ObjectTracker ready. Send /track/enable=true to start.')

    def enable_cb(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f'Tracking: {self.enabled}')
        if not self.enabled:
            self._stop()

    def image_cb(self, msg: Image):
        if not self.enabled:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower1, self.upper1),
            cv2.inRange(hsv, self.lower2, self.upper2)
        )
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._stop()
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            self._stop()
            return

        M = cv2.moments(largest)
        if M['m00'] == 0:
            self._stop()
            return

        cx = M['m10'] / M['m00']
        offset = (cx - w / 2.0) / (w / 2.0)

        twist = Twist()
        twist.linear.x = self.fwd_speed
        twist.angular.z = -self.K_ang * offset
        self.cmd_pub.publish(twist)
        self.get_logger().debug(f'tracking: cx={cx:.0f} offset={offset:.2f}')

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
