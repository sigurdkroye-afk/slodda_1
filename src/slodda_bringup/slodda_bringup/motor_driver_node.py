#!/usr/bin/env python3
"""
Motor driver node for DFRobot GB37Y3530-12V-251R differential-drive robot.

Bridges /cmd_vel → PWM signals via H-bridge motor driver.
Publishes encoder ticks on /encoder_ticks.

Fill in the TODO pin constants before running on hardware.
Swap StubGpioBackend for a concrete GpioBackend subclass once your GPIO
library is chosen (RPi.GPIO, gpiozero, lgpio, pigpio, …).
"""
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from .gpio_backend import LgpioBackend

# Robot geometry
WHEEL_BASE_M          = 0.2316  # m  effective turning wheel base (calibrated, < physical 0.256)
WHEEL_RADIUS_M        = 0.01021 # m  drive sprocket radius (calibrated)
ENCODER_TICKS_PER_REV = 663     # GB37Y3530: 11PPR * 2 edges * 30.15 ratio ≈ 663
METRES_PER_TICK       = 2.0 * math.pi * WHEEL_RADIUS_M / ENCODER_TICKS_PER_REV

# PWM
PWM_FREQUENCY_HZ      = 1000    # Hz
MIN_PWM               = 18.0    # %  — overcomes static friction; tune on hardware
MAX_PWM               = 95.0    # %
MAX_WHEEL_SPEED_MPS   = 0.5     # m/s at 100% duty (full-scale reference)
VELOCITY_DEADBAND_MPS = 0.01    # m/s — below this → PWM=0

# PID — start tuning with Ki=Kd=0 until Kp response looks stable
PID_KP             = 30.0   # %PWM per (m/s error)
PID_KI             = 5.0    # %PWM per (m/s · s)
PID_KD             = 0.5    # %PWM per (m/s / s)
PID_MAX_CORRECTION = 20.0   # % — bounds correction so feedforward dominates

# Loop rates
ENCODER_PUBLISH_HZ    = 20      # Hz
PID_HZ                = 20      # Hz — matches encoder publish rate
WATCHDOG_HZ           = 10      # Hz
CMD_VEL_TIMEOUT_SEC   = 0.5     # s  — stop if no /cmd_vel

# GPIO pins (BCM numbering) — DFR0601, Motor 1 = left, Motor 2 = right
LEFT_PWM_PIN    = 18   # P1
LEFT_DIR_PIN_A  = 23   # A1
LEFT_DIR_PIN_B  = 24   # B1
RIGHT_PWM_PIN   = 19   # P2
RIGHT_DIR_PIN_A = 26   # B2 (swapped to reverse motor 2 polarity)
RIGHT_DIR_PIN_B = 25   # A2
LEFT_ENC_A_PIN  = 17   # Motor 1 Hall A
LEFT_ENC_B_PIN  = 27   # Motor 1 Hall B
RIGHT_ENC_A_PIN = 22   # Motor 2 Hall A
RIGHT_ENC_B_PIN = 6    # Motor 2 Hall B


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class _WheelPID:
    """Velocity PID with anti-windup for one wheel."""

    def __init__(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral = 0.0
        self._prev_err = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_err = 0.0

    def step(self, error: float, dt: float, out_min: float, out_max: float) -> float:
        self._integral += error * dt
        d = (error - self._prev_err) / dt if dt > 0.0 else 0.0
        self._prev_err = error
        out = self.kp * error + self.ki * self._integral + self.kd * d
        clamped = _clamp(out, out_min, out_max)
        if clamped != out:
            self._integral -= error * dt  # anti-windup: undo when saturated
        return clamped


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self._declare_params()

        self.gpio = LgpioBackend()

        self._left_dir  = True  # True = forward
        self._right_dir = True
        self.left_ticks  = 0
        self.right_ticks = 0
        self._tick_lock  = threading.Lock()
        self._running    = threading.Event()
        self._running.set()

        self._cmd_left     = 0.0  # m/s commanded by cmd_vel
        self._cmd_right    = 0.0
        self._target_left  = 0.0  # m/s ramped target fed to PID
        self._target_right = 0.0
        self._pid_left     = _WheelPID(PID_KP, PID_KI, PID_KD)
        self._pid_right    = _WheelPID(PID_KP, PID_KI, PID_KD)
        self._pid_prev_left  = 0
        self._pid_prev_right = 0
        self._pid_last_time  = self.get_clock().now()

        self._setup_pins()
        self._left_pwm  = self.gpio.setup_pwm(LEFT_PWM_PIN,  PWM_FREQUENCY_HZ)
        self._right_pwm = self.gpio.setup_pwm(RIGHT_PWM_PIN, PWM_FREQUENCY_HZ)
        self._left_pwm.start(0.0)
        self._right_pwm.start(0.0)
        self._setup_encoder_interrupts()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.tick_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.create_timer(1.0 / ENCODER_PUBLISH_HZ, self._publish_ticks)
        self.create_timer(1.0 / PID_HZ,             self._pid_step)
        self.create_timer(1.0 / WATCHDOG_HZ,        self._watchdog)

        self.last_cmd_time = self.get_clock().now()
        self.get_logger().info('MotorDriverNode ready.')

    def _declare_params(self):
        self.declare_parameter('wheel_base_m',          WHEEL_BASE_M)
        self.declare_parameter('max_wheel_speed_mps',   MAX_WHEEL_SPEED_MPS)
        self.declare_parameter('min_pwm',               MIN_PWM)
        self.declare_parameter('max_pwm',               MAX_PWM)
        self.declare_parameter('velocity_deadband_mps', VELOCITY_DEADBAND_MPS)
        self.declare_parameter('cmd_vel_timeout_sec',   CMD_VEL_TIMEOUT_SEC)
        self.declare_parameter('pid_kp',                PID_KP)
        self.declare_parameter('pid_ki',                PID_KI)
        self.declare_parameter('pid_kd',                PID_KD)
        self.declare_parameter('left_trim',             1.0)  # scale left target; <1.0 slows left
        self.declare_parameter('max_accel_mps2',        0.2)  # m/s² ramp limit; 0 = disabled

    def _setup_pins(self):
        for pin in [LEFT_DIR_PIN_A, LEFT_DIR_PIN_B,
                    RIGHT_DIR_PIN_A, RIGHT_DIR_PIN_B]:
            if pin is not None:
                self.gpio.setup_output(pin)
        for pin in [LEFT_ENC_A_PIN, LEFT_ENC_B_PIN,
                    RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN]:
            if pin is not None:
                self.gpio.setup_input(pin, pull_up=True)

    def _setup_encoder_interrupts(self):
        # 'both' edges: count rising AND falling on channel A.
        # TICKS_PER_REV=663 = 11PPR × 2 edges × 30.15 gear ratio — requires both edges.
        if LEFT_ENC_A_PIN is not None:
            self.gpio.attach_interrupt(LEFT_ENC_A_PIN, 'both', self._on_left_tick)
        if RIGHT_ENC_A_PIN is not None:
            self.gpio.attach_interrupt(RIGHT_ENC_A_PIN, 'both', self._on_right_tick)

    # cmd_vel handler — stores targets only, PID loop drives PWM

    def _cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        wb   = self.get_parameter('wheel_base_m').value
        trim = self.get_parameter('left_trim').value
        self._cmd_left  = (msg.linear.x - msg.angular.z * wb / 2.0) * trim
        self._cmd_right =  msg.linear.x + msg.angular.z * wb / 2.0

    # PID control loop

    def _pid_step(self):
        now = self.get_clock().now()
        dt  = (now - self._pid_last_time).nanoseconds / 1e9
        self._pid_last_time = now
        if dt <= 0.0:
            return

        # Acceleration ramp: limit how fast target velocity can change
        max_accel = self.get_parameter('max_accel_mps2').value
        if max_accel > 0.0:
            max_delta = max_accel * dt
            for attr_t, attr_c in (('_target_left', '_cmd_left'), ('_target_right', '_cmd_right')):
                t = getattr(self, attr_t)
                c = getattr(self, attr_c)
                delta = _clamp(c - t, -max_delta, max_delta)
                setattr(self, attr_t, t + delta)
        else:
            self._target_left  = self._cmd_left
            self._target_right = self._cmd_right

        with self._tick_lock:
            cur_left  = self.left_ticks
            cur_right = self.right_ticks

        actual_left  = (cur_left  - self._pid_prev_left)  * METRES_PER_TICK / dt
        actual_right = (cur_right - self._pid_prev_right) * METRES_PER_TICK / dt
        self._pid_prev_left  = cur_left
        self._pid_prev_right = cur_right

        self._pid_left.kp  = self._pid_right.kp  = self.get_parameter('pid_kp').value
        self._pid_left.ki  = self._pid_right.ki  = self.get_parameter('pid_ki').value
        self._pid_left.kd  = self._pid_right.kd  = self.get_parameter('pid_kd').value

        for side, target, actual, pid in (
            ('left',  self._target_left,  actual_left,  self._pid_left),
            ('right', self._target_right, actual_right, self._pid_right),
        ):
            ff   = self._vel_to_signed_duty(target)
            corr = pid.step(target - actual, dt, -PID_MAX_CORRECTION, PID_MAX_CORRECTION)
            self._apply_signed_duty(side, ff + corr)

    # Helpers

    def _vel_to_signed_duty(self, v_mps: float) -> float:
        """Feedforward: velocity → signed PWM duty (negative = reverse)."""
        db   = self.get_parameter('velocity_deadband_mps').value
        vmax = self.get_parameter('max_wheel_speed_mps').value
        pmin = self.get_parameter('min_pwm').value
        pmax = self.get_parameter('max_pwm').value
        if abs(v_mps) < db:
            return 0.0
        mag  = _clamp(abs(v_mps) / vmax, 0.0, 1.0)
        duty = pmin + mag * (pmax - pmin)
        return duty if v_mps >= 0.0 else -duty

    def _apply_signed_duty(self, side: str, signed_duty: float):
        """Sign → direction pins, magnitude → PWM duty."""
        pmax    = self.get_parameter('max_pwm').value
        pmin    = self.get_parameter('min_pwm').value
        forward = signed_duty >= 0.0
        duty    = _clamp(abs(signed_duty), 0.0, pmax)
        if duty < pmin:
            duty = 0.0
        if side == 'left':
            self._left_dir = forward
        else:
            self._right_dir = forward
        self._set_direction(side, forward)
        pwm = self._left_pwm if side == 'left' else self._right_pwm
        pwm.set_duty(duty)

    def _set_direction(self, side: str, forward: bool):
        if side == 'left':
            pin_a, pin_b = LEFT_DIR_PIN_A, LEFT_DIR_PIN_B
        else:
            pin_a, pin_b = RIGHT_DIR_PIN_A, RIGHT_DIR_PIN_B
        if pin_a is not None:
            self.gpio.write(pin_a,     forward)
        if pin_b is not None:
            self.gpio.write(pin_b, not forward)

    # Encoder interrupts

    def _on_left_tick(self, *_):
        if not self._running.is_set():
            return
        sign = 1 if self._left_dir else -1
        with self._tick_lock:
            self.left_ticks += sign

    def _on_right_tick(self, *_):
        if not self._running.is_set():
            return
        sign = 1 if self._right_dir else -1
        with self._tick_lock:
            self.right_ticks += sign

    def _publish_ticks(self):
        with self._tick_lock:
            data = [self.left_ticks, self.right_ticks]
        msg = Int32MultiArray()
        msg.data = data
        self.tick_pub.publish(msg)

    # Watchdog

    def _watchdog(self):
        timeout = self.get_parameter('cmd_vel_timeout_sec').value
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > timeout:
            self._target_left  = 0.0
            self._target_right = 0.0
            self._pid_left.reset()
            self._pid_right.reset()
            self._left_pwm.set_duty(0.0)
            self._right_pwm.set_duty(0.0)

    # Shutdown

    def destroy_node(self):
        self._left_pwm.stop()
        self._right_pwm.stop()
        self.gpio.cleanup()
        super().destroy_node()


def main(args=None):
    import signal
    rclpy.init(args=args)
    node = MotorDriverNode()

    def _emergency_stop(signum, frame):
        # Called on SIGTERM/SIGINT — guarantee motors stop before process exits
        node._running.clear()          # stop encoder callbacks first
        try:
            node._left_pwm.set_duty(0.0)
            node._right_pwm.set_duty(0.0)
        except Exception:
            pass
        rclpy.shutdown()               # finally-block handles full cleanup

    signal.signal(signal.SIGTERM, _emergency_stop)
    signal.signal(signal.SIGINT,  _emergency_stop)

    try:
        rclpy.spin(node)
    except Exception:
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
