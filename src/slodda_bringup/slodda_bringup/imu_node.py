#!/usr/bin/env python3
"""
BNO085 IMU node — publishes sensor_msgs/Imu on /imu/data.

Requires adafruit_bno08x and adafruit_blinka on the Raspberry Pi:
  pip install adafruit-circuitpython-bno08x adafruit-blinka

Default I2C address: 0x4A (alt: 0x4B if ADR pin is high).

Design: sensor reads run in a background thread at 20 Hz so that I2C
latency/blockage/recovery never stalls the ROS timer callback. The timer
callback only snapshots the latest cached reading and publishes it.

I2C speed note: adafruit-blinka ignores the frequency= argument on Linux
("I2C frequency is not settable in python, ignoring!"). The actual bus
speed is set by the OS. For BNO085 reliability on Pi 4, add this line to
/boot/firmware/config.txt and reboot:
    dtparam=i2c_arm_baudrate=10000
10 kHz eliminates the clock-stretching NACK errors (OSError 123/133).
"""
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import board
    import busio
    from adafruit_bno08x import (
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_LINEAR_ACCELERATION,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False

# BNO085 noise spec (variance = stddev²)
_ORI_VAR   = 4e-6    # rad²
_GYRO_VAR  = 8.1e-5  # rad²/s²
_ACCEL_VAR = 4.4e-4  # m²/s⁴

_REINIT_AFTER_ERRORS    = 50   # consecutive read failures before re-init
_REINIT_COOLDOWN_S      = 5.0  # seconds to wait between re-init attempts
_READ_HZ                = 20   # background read rate; publish_hz ≤ this
_STALE_DATA_TIMEOUT_S   = 2.0  # skip publish if sample older than this (sw I2C is slower)
_POST_REINIT_VALIDATE_S = 2.0  # skip publish for this long after reinit


def _diag9(v: float):
    m = [0.0] * 9
    m[0] = m[4] = m[8] = v
    return m


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('publish_hz',  5.0)
        self.declare_parameter('frame_id',    'imu_link')
        self.declare_parameter('i2c_address', 0x4A)

        self._pub = self.create_publisher(Imu, '/imu/data', 10)

        # Latest reading tuple (qi,qj,qk,qr, gx,gy,gz, ax,ay,az) or None
        self._data: tuple | None = None
        self._data_t             = 0.0   # wall-clock time of last good read
        self._reinit_completed_t = 0.0   # wall-clock time of last reinit OK
        self._lock = threading.Lock()
        self._bno  = None
        self._consecutive_errors = 0
        self._last_reinit_t      = 0.0

        if not _HW_AVAILABLE:
            self.get_logger().error(
                'adafruit_bno08x not installed — '
                'pip install adafruit-circuitpython-bno08x adafruit-blinka')
        else:
            self._init_sensor()
            threading.Thread(target=self._read_loop, daemon=True).start()

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info('ImuNode ready.')

    # ── Sensor init / reinit ─────────────────────────────────────────────────

    def _init_sensor(self) -> bool:
        """Initialize or reinitialize the BNO085. Blocks ~2–5 s."""
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=50_000)
            self._bno = BNO08X_I2C(
                i2c, address=self.get_parameter('i2c_address').value,
                debug=False)
            time.sleep(1.0)
            for feature in (BNO_REPORT_GAME_ROTATION_VECTOR,
                            BNO_REPORT_GYROSCOPE,
                            BNO_REPORT_LINEAR_ACCELERATION):
                for attempt in range(5):
                    try:
                        self._bno.enable_feature(feature)
                        time.sleep(0.3)
                        break
                    except Exception:
                        if attempt == 4:
                            raise
                        time.sleep(0.5)
            self._consecutive_errors = 0
            self._last_reinit_t      = time.time()
            # Warmup: discard initial SHTP advertisement packets (can be 272 bytes)
            # before declaring the sensor ready to read.
            t0 = time.time()
            while time.time() - t0 < 3.0:
                try:
                    q = self._bno.game_quaternion
                    if q is not None and q[3] != 0.0:
                        break
                except Exception:
                    pass
                time.sleep(0.05)
            self._reinit_completed_t = time.time()
            self.get_logger().info('BNO085 (re)initialized OK.')
            return True
        except Exception as e:
            self._bno = None
            self.get_logger().error(
                f'BNO085 init failed: {e}', throttle_duration_sec=10.0)
            self._last_reinit_t = time.time()
            return False

    # ── Background read loop ─────────────────────────────────────────────────

    def _read_loop(self):
        """Reads the sensor at _READ_HZ; reinits on persistent failure."""
        interval = 1.0 / _READ_HZ
        while rclpy.ok():
            if self._bno is None:
                if time.time() - self._last_reinit_t >= _REINIT_COOLDOWN_S:
                    self._init_sensor()
                time.sleep(interval)
                continue

            try:
                q  = self._bno.game_quaternion
                g  = self._bno.gyro
                a  = self._bno.linear_acceleration
                if q is None or g is None or a is None:
                    time.sleep(interval)
                    continue
                qi, qj, qk, qr = q
                gx, gy, gz      = g
                ax, ay, az      = a
                with self._lock:
                    self._data   = (qi, qj, qk, qr, gx, gy, gz, ax, ay, az)
                    self._data_t = time.time()
                self._consecutive_errors = 0
            except Exception as e:
                self._consecutive_errors += 1
                self.get_logger().warn(
                    f'BNO085 read error: {e} (#{self._consecutive_errors})',
                    throttle_duration_sec=5.0)
                if self._consecutive_errors >= _REINIT_AFTER_ERRORS:
                    self.get_logger().error(
                        'BNO085: too many errors — triggering reinit')
                    self._bno = None  # next iteration calls _init_sensor

            time.sleep(interval)

    # ── ROS publish callback ─────────────────────────────────────────────────

    def _publish(self):
        now = time.time()
        with self._lock:
            data   = self._data
            data_t = self._data_t
        if data is None:
            return
        if now - data_t > _STALE_DATA_TIMEOUT_S:
            return  # sensor failing/reiniting — don't feed stale gyro to EKF
        if now - self._reinit_completed_t < _POST_REINIT_VALIDATE_S:
            return  # BNO085 emits garbage briefly after reinit

        qi, qj, qk, qr, gx, gy, gz, ax, ay, az = data

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

        # BNO08x quaternion order: (i, j, k, real) → ROS: (x, y, z, w)
        msg.orientation.x = qi
        msg.orientation.y = qj
        msg.orientation.z = qk
        msg.orientation.w = qr
        msg.orientation_covariance = _diag9(_ORI_VAR)

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = _diag9(_GYRO_VAR)

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = _diag9(_ACCEL_VAR)

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
