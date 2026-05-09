#!/usr/bin/env python3
"""Wheel odometry calibration tool.

Requires motors_only.launch.py to be running (motor_driver + odometry_node).

Usage — avstandsbasert (anbefalt):
  ros2 run slodda_bringup odom_calibration \
    --ros-args -p drive_distance_m:=1.0 -p current_wheel_radius_m:=0.0103

  Robot kjører til odom sier 1.0m, stopper. Mål fysisk avstand.

Usage — tidsbasert (original):
  ros2 run slodda_bringup odom_calibration \
    --ros-args -p current_wheel_radius_m:=0.0103
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomCalibration(Node):
    def __init__(self):
        super().__init__('odom_calibration')
        self.declare_parameter('drive_speed_mps',        0.15)
        self.declare_parameter('drive_duration_s',       7.0)   # brukes kun i tidsbasert modus
        self.declare_parameter('drive_distance_m',       0.0)   # >0 → avstandsbasert modus
        self.declare_parameter('current_wheel_radius_m', 0.0)   # 0 = ukjent

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
        speed       = self.get_parameter('drive_speed_mps').value
        duration    = self.get_parameter('drive_duration_s').value
        target_dist = self.get_parameter('drive_distance_m').value
        current_r   = self.get_parameter('current_wheel_radius_m').value

        dist_mode = target_dist > 0

        print('\n=== Slodda Wheel Odometry Calibration ===')
        if dist_mode:
            print(f'Modus: kjør til odom = {target_dist:.2f} m  (speed={speed} m/s)')
        else:
            print(f'Modus: kjør {duration}s ved {speed} m/s = ~{speed*duration:.2f} m')
        if current_r <= 0:
            print('[NOTE] Pass -p current_wheel_radius_m:=<verdi> for korrekt resultatberegning.')
        print('\nVenter på /odom ...')
        self._spin_until_odom()

        print('Merk startposisjon på gulvet. Trykk ENTER for å starte.')
        input()

        rclpy.spin_once(self, timeout_sec=0.05)
        x0   = self._odom.pose.pose.position.x
        y0   = self._odom.pose.pose.position.y
        yaw0 = _quat_to_yaw(self._odom.pose.pose.orientation)
        print(f'Start → x={x0:.4f} y={y0:.4f} yaw={math.degrees(yaw0):.1f}°')
        print('Kjører ...')

        cmd = Twist()
        cmd.linear.x = speed

        if dist_mode:
            while True:
                self._cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.02)
                x = self._odom.pose.pose.position.x
                y = self._odom.pose.pose.position.y
                if math.sqrt((x - x0)**2 + (y - y0)**2) >= target_dist:
                    break
        else:
            t_start = self.get_clock().now()
            while (self.get_clock().now() - t_start).nanoseconds / 1e9 < duration:
                self._cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

        self._stop()
        print('Stoppet. Venter ...')
        self._spin_seconds(0.5)

        x1   = self._odom.pose.pose.position.x
        y1   = self._odom.pose.pose.position.y
        yaw1 = _quat_to_yaw(self._odom.pose.pose.orientation)

        odom_dist  = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        fwd  = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        side = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
        yaw_drift = math.degrees(yaw1 - yaw0)

        print(f'\n{"="*50}')
        print(f'Odom avstand       : {odom_dist:.4f} m')
        print(f'Frem  (robotramme) : {fwd:.4f} m')
        print(f'Side  (robotramme) : {side:.4f} m  (vil ha ~0)')
        print(f'Yaw-drift          : {yaw_drift:.2f}°  (vil ha ~0)')
        print(f'{"="*50}')

        if abs(side) > 0.03:
            print(f'[WARN] {abs(side)*100:.1f} cm sideveis drift — enkoder/belte asymmetri')
        if abs(yaw_drift) > 3.0:
            print(f'[WARN] {abs(yaw_drift):.1f}° yaw-drift — venstre/høyre ubalanse')

        raw = input('\nMål fysisk avstand med målebånd. Skriv inn (m): ').strip()
        raw = raw.replace(' m', '').replace(',', '.')
        if raw:
            try:
                phys_m = float(raw)
                factor = phys_m / odom_dist
                print(f'\n--- Resultat ---')
                print(f'Fysisk : {phys_m:.4f} m')
                print(f'Odom   : {odom_dist:.4f} m')
                print(f'Faktor : {factor:.5f}')
                if current_r > 0:
                    new_r = current_r * factor
                    print(f'\nNåværende wheel_radius_m : {current_r:.5f}')
                    print(f'Ny      wheel_radius_m : {new_r:.5f}')
                    print(f'\nOppdater motors.yaml og hardware.launch.py:')
                    print(f'  wheel_radius_m: {new_r:.5f}')
                else:
                    print(f'\nNy wheel_radius_m = nåværende × {factor:.5f}')
                if abs(factor - 1.0) > 0.05:
                    print(f'[WARN] Faktor {factor:.3f} > 5% — dobbeltsjekk måling.')
            except ValueError:
                print('Ugyldig input.')

        print('\nFerdig.')


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
