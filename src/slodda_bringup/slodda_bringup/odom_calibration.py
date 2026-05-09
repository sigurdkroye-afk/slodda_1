#!/usr/bin/env python3
"""Wheel odometry calibration tool.

Requires motors_only.launch.py to be running (motor_driver + odometry_node).
Does NOT need Nav2, SLAM, EKF, or LiDAR.

Usage:
  Terminal 1: ros2 launch slodda_bringup motors_only.launch.py
  Terminal 2: ros2 run slodda_bringup odom_calibration \
                --ros-args -p current_wheel_radius_m:=0.0108

IMPORTANT: Pass current_wheel_radius_m = whatever value odometry_node is
using right now (set via ros2 param set or motors.yaml).
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomCalibration(Node):
    def __init__(self):
        super().__init__('odom_calibration')
        self.declare_parameter('drive_speed_mps',      0.15)
        self.declare_parameter('drive_duration_s',     7.0)
        self.declare_parameter('current_wheel_radius_m', 0.0)  # 0 = unknown

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

    def _stop(self):
        try:
            self._cmd_pub.publish(Twist())
        except Exception:
            pass

    def run(self):
        speed     = self.get_parameter('drive_speed_mps').value
        duration  = self.get_parameter('drive_duration_s').value
        current_r = self.get_parameter('current_wheel_radius_m').value

        print('\n=== Slodda Wheel Odometry Calibration ===')
        print(f'Drive plan: {speed} m/s × {duration} s = ~{speed*duration:.2f} m')
        if current_r <= 0:
            print('[NOTE] Pass --ros-args -p current_wheel_radius_m:=<value> for accurate result calc.')
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

        self._stop()
        print('Stopped. Settling ...')
        self._spin_seconds(0.5)

        x1   = self._odom.pose.pose.position.x
        y1   = self._odom.pose.pose.position.y
        yaw1 = _quat_to_yaw(self._odom.pose.pose.orientation)

        odom_dist  = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        # Forward displacement in robot start frame (removes effect of initial yaw)
        fwd  = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        side = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
        yaw_drift = math.degrees(yaw1 - yaw0)

        print(f'\n{"="*50}')
        print(f'Odom distance (Euclidean) : {odom_dist:.4f} m')
        print(f'Forward  (robot frame)   : {fwd:.4f} m')
        print(f'Sideways (robot frame)   : {side:.4f} m  (want ~0)')
        print(f'Yaw drift                : {yaw_drift:.2f}°  (want ~0)')
        print(f'{"="*50}')

        if abs(side) > 0.03:
            print(f'[WARN] {abs(side)*100:.1f} cm sideways drift — encoder or track asymmetry')
        if abs(yaw_drift) > 3.0:
            print(f'[WARN] {abs(yaw_drift):.1f}° yaw drift — left/right speed mismatch')

        raw = input('\nMeasure physical distance with tape. Enter (m): ').strip()
        # Accept "1.2 m" or "1.2"
        raw = raw.replace(' m', '').replace(',', '.')
        if raw:
            try:
                phys_m = float(raw)
                factor = phys_m / odom_dist
                print(f'\n--- Result ---')
                print(f'Physical : {phys_m:.4f} m')
                print(f'Odom     : {odom_dist:.4f} m')
                print(f'Factor   : {factor:.5f}')
                if current_r > 0:
                    new_r = current_r * factor
                    print(f'\nCurrent wheel_radius_m : {current_r:.5f}')
                    print(f'New     wheel_radius_m : {new_r:.5f}')
                    print(f'\nSet in motors.yaml and hardware.launch.py:')
                    print(f'  wheel_radius_m: {new_r:.5f}')
                else:
                    print(f'\nNew wheel_radius_m = current × {factor:.5f}')
                    print('(Re-run with --ros-args -p current_wheel_radius_m:=<current> for exact value)')
                if abs(factor - 1.0) > 0.10:
                    print(f'[WARNING] Factor {factor:.3f} > 10% off — double-check measurement.')
            except ValueError:
                print('Invalid input.')

        print('\nDone.')


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalibration()
    try:
        node.run()
    except KeyboardInterrupt:
        node._stop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
