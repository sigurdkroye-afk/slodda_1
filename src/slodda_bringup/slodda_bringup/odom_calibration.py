#!/usr/bin/env python3
"""Wheel odometry calibration tool.

Requires motors_only.launch.py to be running (motor_driver + odometry_node).
Does NOT need Nav2, SLAM, EKF, or LiDAR.

Usage:
  Terminal 1: ros2 launch slodda_bringup motors_only.launch.py
  Terminal 2: ros2 run slodda_bringup odom_calibration

Protocol:
  1. Mark start position on floor with tape
  2. Run script — press ENTER to drive
  3. Mark where robot stops
  4. Measure physical distance with tape measure
  5. Enter measurement — script prints corrected wheel_radius_m
  6. Update motors.yaml (and hardware.launch.py) with new value
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomCalibration(Node):
    def __init__(self):
        super().__init__('odom_calibration')
        self.declare_parameter('drive_speed_mps',  0.15)   # slow for safety
        self.declare_parameter('drive_duration_s', 7.0)    # ~1.05 m at 0.15 m/s
        self.declare_parameter('wheel_radius_m',   0.0208) # current value for factor calc

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom    = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, '_odom', m), 10)

    def _spin_until_odom(self):
        while self._odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _spin_seconds(self, secs: float):
        t = self.get_clock().now()
        while (self.get_clock().now() - t).nanoseconds / 1e9 < secs:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        speed    = self.get_parameter('drive_speed_mps').value
        duration = self.get_parameter('drive_duration_s').value
        current_r = self.get_parameter('wheel_radius_m').value

        print('\n=== Slodda Wheel Odometry Calibration ===')
        print(f'Drive plan: {speed} m/s × {duration} s = ~{speed*duration:.2f} m')
        print('\nWaiting for /odom ...')
        self._spin_until_odom()

        print('Mark start position on floor. Press ENTER to start drive.')
        input()

        rclpy.spin_once(self, timeout_sec=0.05)
        x0   = self._odom.pose.pose.position.x
        y0   = self._odom.pose.pose.position.y
        yaw0 = _quat_to_yaw(self._odom.pose.pose.orientation)
        print(f'Start → x={x0:.4f} y={y0:.4f} yaw={math.degrees(yaw0):.1f}°')
        print('Driving ...')

        cmd = Twist()
        cmd.linear.x = speed
        t_start = self.get_clock().now()
        while (self.get_clock().now() - t_start).nanoseconds / 1e9 < duration:
            self._cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self._cmd_pub.publish(Twist())  # stop
        print('Stopped. Settling ...')
        self._spin_seconds(0.5)

        x1   = self._odom.pose.pose.position.x
        y1   = self._odom.pose.pose.position.y
        yaw1 = _quat_to_yaw(self._odom.pose.pose.orientation)

        odom_dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        y_drift   = abs(y1 - y0)
        yaw_drift = abs(math.degrees(yaw1 - yaw0))

        print(f'\n{"="*50}')
        print(f'Odom distance : {odom_dist:.4f} m')
        print(f'Y drift       : {y_drift:.4f} m  (want < 0.01 m)')
        print(f'Yaw drift     : {yaw_drift:.2f}°  (want < 2°)')
        print(f'{"="*50}')

        if y_drift > 0.02:
            print('[WARN] Y drift > 2 cm — check if robot drove straight')
        if yaw_drift > 5.0:
            print('[WARN] Yaw drift > 5° — wheel_base_m or encoder asymmetry likely')

        phys = input('\nMark stop position. Measure with tape. Enter physical distance (m): ').strip()
        if phys:
            try:
                phys_m = float(phys)
                factor = phys_m / odom_dist
                new_r  = current_r * factor
                print(f'\n--- Result ---')
                print(f'Physical    : {phys_m:.4f} m')
                print(f'Odom        : {odom_dist:.4f} m')
                print(f'Factor      : {factor:.5f}')
                print(f'Old wheel_radius_m: {current_r:.5f}')
                print(f'New wheel_radius_m: {new_r:.5f}')
                print(f'\nUpdate in config/motors.yaml:')
                print(f'  wheel_radius_m: {new_r:.5f}')
                print(f'\nAlso update hardware.launch.py odometry_node params.')
                if abs(factor - 1.0) > 0.15:
                    print(f'\n[WARNING] Factor {factor:.3f} is > 15% off — double-check measurement.')
            except ValueError:
                print('Invalid input, skipping calculation.')

        print('\nDone. Rotation calibration: spin robot 360° and check IMU yaw in /odom.')


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibration()
    try:
        node.run()
    except KeyboardInterrupt:
        node._cmd_pub.publish(Twist())  # safety stop
    finally:
        node.destroy_node()
        rclpy.shutdown()
