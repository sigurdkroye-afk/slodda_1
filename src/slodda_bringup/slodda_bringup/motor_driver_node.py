#!/usr/bin/env python3
"""
Motor driver node for DFRobot GB37Y3530-12V-251R differential-drive robot.

Bridges /cmd_vel → PWM signals via H-bridge motor driver.
Publishes encoder ticks on /encoder_ticks.

Fill in the TODO pin constants before running on hardware.
Swap StubGpioBackend for a concrete GpioBackend subclass once your GPIO
library is chosen (RPi.GPIO, gpiozero, lgpio, pigpio, …).
"""
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from .gpio_backend import StubGpioBackend

# ── Robot geometry ─────────────────────────────────────────────────────────────
WHEEL_BASE_M          = 0.18    # m  (distance between wheel centres) — verify!
WHEEL_RADIUS_M        = 0.034   # m  — verify!
ENCODER_TICKS_PER_REV = 663     # GB37Y3530: 11PPR * 2 edges * 30.15 ratio ≈ 663

# ── PWM ────────────────────────────────────────────────────────────────────────
PWM_FREQUENCY_HZ      = 1000    # Hz
MIN_PWM               = 25.0    # %  — overcomes static friction; tune on hardware
MAX_PWM               = 95.0    # %
MAX_WHEEL_SPEED_MPS   = 0.5     # m/s at 100% duty (full-scale reference)
VELOCITY_DEADBAND_MPS = 0.01    # m/s — below this → PWM=0

# ── Loop rates ─────────────────────────────────────────────────────────────────
ENCODER_PUBLISH_HZ    = 20      # Hz
WATCHDOG_HZ           = 10      # Hz
CMD_VEL_TIMEOUT_SEC   = 0.5     # s  — stop if no /cmd_vel

# ── GPIO pins (BCM numbering assumed) — TODO: fill in ─────────────────────────
LEFT_PWM_PIN    = None  # TODO: EN pin for left H-bridge
LEFT_DIR_PIN_A  = None  # TODO: IN1 for left H-bridge
LEFT_DIR_PIN_B  = None  # TODO: IN2 for left H-bridge
RIGHT_PWM_PIN   = None  # TODO: EN pin for right H-bridge
RIGHT_DIR_PIN_A = None  # TODO: IN1 for right H-bridge
RIGHT_DIR_PIN_B = None  # TODO: IN2 for right H-bridge
LEFT_ENC_A_PIN  = None  # TODO: left encoder channel A
LEFT_ENC_B_PIN  = None  # TODO: left encoder channel B (optional, for direction)
RIGHT_ENC_A_PIN = None  # TODO: right encoder channel A
RIGHT_ENC_B_PIN = None  # TODO: right encoder channel B (optional)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self._declare_params()

        self.gpio = StubGpioBackend(log_fn=lambda msg: self.get_logger().debug(msg))
        self.get_logger().warn(
            'StubGpioBackend active — motors will NOT move. '
            'Fill in GPIO pin constants and swap backend for hardware use.')

        self._left_dir  = True  # True = forward
        self._right_dir = True
        self.left_ticks  = 0
        self.right_ticks = 0
        self._tick_lock  = threading.Lock()

        self._setup_pins()
        self._left_pwm  = self.gpio.setup_pwm(LEFT_PWM_PIN,  PWM_FREQUENCY_HZ)
        self._right_pwm = self.gpio.setup_pwm(RIGHT_PWM_PIN, PWM_FREQUENCY_HZ)
        self._left_pwm.start(0.0)
        self._right_pwm.start(0.0)
        self._setup_encoder_interrupts()

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.tick_pub = self.create_publisher(Int32MultiArray, '/encoder_ticks', 10)
        self.create_timer(1.0 / ENCODER_PUBLISH_HZ, self._publish_ticks)
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
        if LEFT_ENC_A_PIN is not None:
            self.gpio.attach_interrupt(LEFT_ENC_A_PIN, 'rising', self._on_left_tick)
        if RIGHT_ENC_A_PIN is not None:
            self.gpio.attach_interrupt(RIGHT_ENC_A_PIN, 'rising', self._on_right_tick)

    # ── cmd_vel handler ────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        wb = self.get_parameter('wheel_base_m').value
        v_left  = msg.linear.x - msg.angular.z * wb / 2.0
        v_right = msg.linear.x + msg.angular.z * wb / 2.0
        self._drive_wheel('left',  v_left)
        self._drive_wheel('right', v_right)

    def _drive_wheel(self, side: str, v_mps: float):
        db   = self.get_parameter('velocity_deadband_mps').value
        vmax = self.get_parameter('max_wheel_speed_mps').value
        pmin = self.get_parameter('min_pwm').value
        pmax = self.get_parameter('max_pwm').value

        forward = v_mps >= 0.0
        if side == 'left':
            self._left_dir = forward
        else:
            self._right_dir = forward

        if abs(v_mps) < db:
            duty = 0.0
        else:
            mag  = _clamp(abs(v_mps) / vmax, 0.0, 1.0)
            duty = pmin + mag * (pmax - pmin)

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

    # ── Encoder interrupts ─────────────────────────────────────────────────────

    def _on_left_tick(self, *_):
        sign = 1 if self._left_dir else -1
        with self._tick_lock:
            self.left_ticks += sign

    def _on_right_tick(self, *_):
        sign = 1 if self._right_dir else -1
        with self._tick_lock:
            self.right_ticks += sign

    def _publish_ticks(self):
        with self._tick_lock:
            data = [self.left_ticks, self.right_ticks]
        msg = Int32MultiArray()
        msg.data = data
        self.tick_pub.publish(msg)

    # ── Watchdog ───────────────────────────────────────────────────────────────

    def _watchdog(self):
        timeout = self.get_parameter('cmd_vel_timeout_sec').value
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > timeout:
            self._left_pwm.set_duty(0.0)
            self._right_pwm.set_duty(0.0)

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._left_pwm.stop()
        self._right_pwm.stop()
        self.gpio.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
