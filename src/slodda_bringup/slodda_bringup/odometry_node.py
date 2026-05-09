#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Physical tracked robot — URDF wheel values are simulation-only and wrong here.
# Measure on the actual robot:
#   WHEEL_RADIUS_M → radius of the drive sprocket (where the belt contacts the axle)
#   WHEEL_BASE_M   → center-to-center distance between left and right belt
# These defaults are placeholders — override via ROS params or set correct values.
WHEEL_RADIUS_M = 0.0208  # drive sprocket radius (measured: 20.8 mm)
WHEEL_BASE_M   = 0.256   # belt center-to-center separation (measured: 25.6 cm)
TICKS_PER_REV  = 663     # GB37Y3530: 11PPR × 2 edges × 30.15 ratio (output shaft)

# Tracks slip more than wheels, especially during in-place turns.  Odometry will
# drift faster than on a wheeled robot — IMU fusion via EKF is strongly recommended.

# Diagonal covariance [x, y, z, roll, pitch, yaw] — tune after first hardware run
_POSE_COV = [
    0.01, 0,    0,   0,   0,   0,
    0,    0.01, 0,   0,   0,   0,
    0,    0,    1e6, 0,   0,   0,
    0,    0,    0,   1e6, 0,   0,
    0,    0,    0,   0,   1e6, 0,
    0,    0,    0,   0,   0,   0.05,
]
_TWIST_COV = [
    0.01, 0,   0,   0,   0,   0,
    0,    1e6, 0,   0,   0,   0,
    0,    0,   1e6, 0,   0,   0,
    0,    0,   0,   1e6, 0,   0,
    0,    0,   0,   0,   1e6, 0,
    0,    0,   0,   0,   0,   0.05,
]


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.declare_parameter('wheel_radius_m', WHEEL_RADIUS_M)
        self.declare_parameter('wheel_base_m',   WHEEL_BASE_M)
        self.declare_parameter('ticks_per_rev',  float(TICKS_PER_REV))

        self.declare_parameter('publish_tf', True)  # set False when EKF publishes TF

        self._x = self._y = self._theta = 0.0
        self._prev_left = self._prev_right = None
        self._last_time = self.get_clock().now()

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_bcast = TransformBroadcaster(self)
        self.create_subscription(
            Int32MultiArray, '/encoder_ticks', self._tick_cb, 10)
        self.get_logger().info('OdometryNode ready.')

    def _tick_cb(self, msg: Int32MultiArray):
        left_ticks  = msg.data[0]
        right_ticks = msg.data[1]

        if self._prev_left is None:
            self._prev_left  = left_ticks
            self._prev_right = right_ticks
            self._last_time  = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        r   = self.get_parameter('wheel_radius_m').value
        wb  = self.get_parameter('wheel_base_m').value
        tpr = self.get_parameter('ticks_per_rev').value
        mpt = 2.0 * math.pi * r / tpr  # metres per tick

        dl = (left_ticks  - self._prev_left)  * mpt
        dr = (right_ticks - self._prev_right) * mpt

        self._prev_left  = left_ticks
        self._prev_right = right_ticks
        self._last_time  = now

        dist   = (dl + dr) / 2.0
        dtheta = (dr - dl) / wb

        # Mid-point integration — more accurate than Euler for large dtheta
        self._x     += dist * math.cos(self._theta + dtheta / 2.0)
        self._y     += dist * math.sin(self._theta + dtheta / 2.0)
        self._theta += dtheta

        self._publish(now, v=dist / dt, omega=dtheta / dt)

    def _publish(self, stamp, v: float, omega: float):
        q = _yaw_to_quat(self._theta)

        if self.get_parameter('publish_tf').value:
            tf = TransformStamped()
            tf.header.stamp    = stamp.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_footprint'
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self._tf_bcast.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.pose.covariance  = _POSE_COV
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        odom.twist.covariance = _TWIST_COV
        self._odom_pub.publish(odom)


def _yaw_to_quat(yaw: float):
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
