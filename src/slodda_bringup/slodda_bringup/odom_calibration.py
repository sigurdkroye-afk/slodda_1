#!/usr/bin/env python3
"""Wheel odometry calibration tool.

Requires motors_only.launch.py to be running (motor_driver + odometry_node).

DISTANSE-modus (wheel_radius_m):
  ros2 run slodda_bringup odom_calibration \
    --ros-args -p drive_distance_m:=1.0 -p current_wheel_radius_m:=0.01021

ROTASJON-modus (wheel_base_m):
  ros2 run slodda_bringup odom_calibration \
    --ros-args -p rotation_angle_deg:=360 -p current_wheel_base_m:=0.256
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class OdomCalibration(Node):
    def __init__(self):
        super().__init__('odom_calibration')
        # Distanse-modus
        self.declare_parameter('drive_speed_mps',        0.15)
        self.declare_parameter('drive_duration_s',       7.0)
        self.declare_parameter('drive_distance_m',       0.0)
        self.declare_parameter('current_wheel_radius_m', 0.0)
        # Rotasjon-modus
        self.declare_parameter('rotation_angle_deg',     0.0)   # >0 → rotasjonsmodus
        self.declare_parameter('rotation_speed_rads',    0.5)   # rad/s
        self.declare_parameter('current_wheel_base_m',   0.0)

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
        rotation_target = self.get_parameter('rotation_angle_deg').value
        if rotation_target > 0:
            self._run_rotation()
        else:
            self._run_distance()

    # Distanse-kalibrering

    def _run_distance(self):
        speed       = self.get_parameter('drive_speed_mps').value
        duration    = self.get_parameter('drive_duration_s').value
        target_dist = self.get_parameter('drive_distance_m').value
        current_r   = self.get_parameter('current_wheel_radius_m').value

        dist_mode = target_dist > 0
        print('\n=== Slodda Wheel Odometry Calibration — DISTANSE ===')
        if dist_mode:
            print(f'Modus: kjør til odom = {target_dist:.2f} m  (speed={speed} m/s)')
        else:
            print(f'Modus: kjør {duration}s ved {speed} m/s = ~{speed*duration:.2f} m')
        if current_r <= 0:
            print('[NOTE] Pass -p current_wheel_radius_m:=<verdi> for beregning.')

        print('\nVenter på /odom ...')
        self._spin_until_odom()
        print('Merk startposisjon. Trykk ENTER for å starte.')
        input()

        rclpy.spin_once(self, timeout_sec=0.05)
        x0, y0 = self._odom.pose.pose.position.x, self._odom.pose.pose.position.y
        yaw0 = _quat_to_yaw(self._odom.pose.pose.orientation)
        print(f'Start → x={x0:.4f} y={y0:.4f} yaw={math.degrees(yaw0):.1f}°')
        print('Kjører ...')

        cmd = Twist()
        cmd.linear.x = speed

        if dist_mode:
            while True:
                self._cmd_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.02)
                x, y = self._odom.pose.pose.position.x, self._odom.pose.pose.position.y
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

        x1, y1 = self._odom.pose.pose.position.x, self._odom.pose.pose.position.y
        yaw1 = _quat_to_yaw(self._odom.pose.pose.orientation)
        odom_dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        fwd  = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
        side = -(x1 - x0) * math.sin(yaw0) + (y1 - y0) * math.cos(yaw0)
        yaw_drift = math.degrees(yaw1 - yaw0)

        print(f'\n{"="*50}')
        print(f'Odom avstand       : {odom_dist:.4f} m')
        print(f'Frem  (robotramme) : {fwd:.4f} m')
        print(f'Side  (robotramme) : {side:.4f} m  (vil ha ~0)')
        print(f'Yaw-drift          : {yaw_drift:.2f}°')
        print(f'{"="*50}')
        if abs(side) > 0.03:
            print(f'[WARN] {abs(side)*100:.1f} cm sideveis — enkoder/belte asymmetri')
        if abs(yaw_drift) > 3.0:
            print(f'[WARN] {abs(yaw_drift):.1f}° yaw — korrigeres av EKF+IMU i fullt stack')

        raw = input('\nMål fysisk avstand. Skriv inn (m): ').strip().replace(' m', '').replace(',', '.')
        if raw:
            try:
                phys_m = float(raw)
                factor = phys_m / odom_dist
                print(f'\n--- Resultat ---')
                print(f'Fysisk : {phys_m:.4f} m  |  Odom : {odom_dist:.4f} m  |  Faktor : {factor:.5f}')
                if current_r > 0:
                    new_r = current_r * factor
                    print(f'Ny wheel_radius_m : {new_r:.5f}')
                    print(f'Oppdater motors.yaml og hardware.launch.py: wheel_radius_m: {new_r:.5f}')
                else:
                    print(f'Ny wheel_radius_m = nåværende × {factor:.5f}')
                if abs(factor - 1.0) > 0.05:
                    print(f'[WARN] Faktor {factor:.3f} > 5% — dobbeltsjekk måling.')
            except ValueError:
                print('Ugyldig input.')
        print('\nFerdig.')

    # Rotasjonskalibrering

    def _run_rotation(self):
        target_deg  = self.get_parameter('rotation_angle_deg').value
        rot_speed   = self.get_parameter('rotation_speed_rads').value
        current_wb  = self.get_parameter('current_wheel_base_m').value
        target_rad  = math.radians(target_deg)

        print('\n=== Slodda Wheel Odometry Calibration — ROTASJON ===')
        print(f'Modus: rotér {target_deg:.0f}° per odom  (speed={rot_speed} rad/s)')
        print('Kalibrer wheel_base_m — påvirker nøyaktigheten til svinger.')
        if current_wb <= 0:
            print('[NOTE] Pass -p current_wheel_base_m:=0.256 for beregning.')

        print('\nVenter på /odom ...')
        self._spin_until_odom()
        print(f'Sett tape-markering på gulvet for å kunne måle rotasjonen.')
        print(f'Trykk ENTER for å starte.')
        input()

        rclpy.spin_once(self, timeout_sec=0.05)
        yaw0 = _quat_to_yaw(self._odom.pose.pose.orientation)
        print(f'Start yaw = {math.degrees(yaw0):.1f}°')
        print('Roterer ...')

        cmd = Twist()
        cmd.angular.z = rot_speed  # positiv = mot klokka (venstre)

        accumulated = 0.0
        prev_yaw = yaw0
        while accumulated < target_rad:
            self._cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
            cur_yaw = _quat_to_yaw(self._odom.pose.pose.orientation)
            delta = _angle_diff(cur_yaw, prev_yaw)
            accumulated += delta
            prev_yaw = cur_yaw

        self._stop()
        print('Stoppet. Venter ...')
        self._spin_seconds(0.5)

        final_yaw = _quat_to_yaw(self._odom.pose.pose.orientation)
        odom_deg  = math.degrees(accumulated)

        print(f'\n{"="*50}')
        print(f'Odom-rotasjon : {odom_deg:.2f}°  (mål: {target_deg:.0f}°)')
        print(f'{"="*50}')

        raw = input(f'\nMål fysisk rotasjon (grader). Skriv inn: ').strip().replace('°', '').replace(',', '.')
        if raw:
            try:
                phys_deg = float(raw)
                factor   = phys_deg / odom_deg
                print(f'\n--- Resultat ---')
                print(f'Fysisk : {phys_deg:.1f}°  |  Odom : {odom_deg:.2f}°  |  Faktor : {factor:.5f}')
                if current_wb > 0:
                    new_wb = current_wb * (odom_deg / phys_deg)
                    print(f'Ny wheel_base_m : {new_wb:.5f}')
                    print(f'Oppdater motors.yaml og hardware.launch.py: wheel_base_m: {new_wb:.5f}')
                else:
                    print(f'Ny wheel_base_m = nåværende × {odom_deg/phys_deg:.5f}')
                if abs(factor - 1.0) > 0.05:
                    print(f'[WARN] Faktor {factor:.3f} > 5% — dobbeltsjekk måling.')
            except ValueError:
                print('Ugyldig input.')
        print('\nFerdig.')


def _quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path angle from b to a, handles wraparound."""
    diff = a - b
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    return diff


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
