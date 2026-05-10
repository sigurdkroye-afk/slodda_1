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

_REINIT_COOLDOWN_S      = 30.0  # rarely reinit — only on hard failures
_READ_HZ                = 5     # 5 Hz: less I2C traffic = fewer corrupt packets
_STALE_DATA_TIMEOUT_S   = 2.0   # skip publish if sample older than this
_POST_REINIT_VALIDATE_S = 0.0   # norm-validation is the safety filter, no time block
_REENABLE_STALE_S       = 2.0   # re-enable features if no valid sample in 2s
_REENABLE_COOLDOWN_S    = 3.0   # min time between re-enable attempts


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
        """Initialize the BNO085. Blocks ~1.5 s."""
        try:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=50_000)
            self._bno = BNO08X_I2C(
                i2c, address=self.get_parameter('i2c_address').value,
                debug=False)
            time.sleep(0.5)  # let sensor process advertisement packet
            for feature in (BNO_REPORT_GAME_ROTATION_VECTOR,
                            BNO_REPORT_GYROSCOPE,
                            BNO_REPORT_LINEAR_ACCELERATION):
                try:
                    self._bno.enable_feature(feature)
                except Exception:
                    time.sleep(0.2)
                    self._bno.enable_feature(feature)  # one retry
            self._consecutive_errors = 0
            self._last_reinit_t      = time.time()
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
        """Reads at _READ_HZ. Re-enables features after sensor self-reset."""
        interval = 1.0 / _READ_HZ
        last_reenable_t = 0.0
        while rclpy.ok():
            if self._bno is None:
                if time.time() - self._last_reinit_t >= _REINIT_COOLDOWN_S:
                    self._init_sensor()
                time.sleep(interval)
                continue

            now = time.time()
            # Sensor self-resets every ~15s on sw I2C. Re-enable features
            # (cheap — no I2C re-open needed) when samples go stale.
            if (self._data_t > 0.0
                    and now - self._data_t > _REENABLE_STALE_S
                    and now - last_reenable_t > _REENABLE_COOLDOWN_S):
                try:
                    for feature in (BNO_REPORT_GAME_ROTATION_VECTOR,
                                    BNO_REPORT_GYROSCOPE,
                                    BNO_REPORT_LINEAR_ACCELERATION):
                        self._bno.enable_feature(feature)
                    last_reenable_t = time.time()
                    self.get_logger().info(
                        'BNO085 features re-enabled after suspected reset.',
                        throttle_duration_sec=5.0)
                except Exception as e:
                    self.get_logger().warn(
                        f'BNO085 re-enable failed: {e}',
                        throttle_duration_sec=5.0)
                time.sleep(interval)
                continue

            try:
                q = self._bno.game_quaternion
                if q is None:
                    self.get_logger().warn(
                        'BNO085 game_quaternion=None (sensor not ready yet)',
                        throttle_duration_sec=5.0)
                    time.sleep(interval)
                    continue

                qi, qj, qk, qr = q
                if not (0.9 < (qi*qi + qj*qj + qk*qk + qr*qr) < 1.1):
                    time.sleep(interval)
                    continue

                try:
                    g = self._bno.gyro
                    gx, gy, gz = g if g is not None else (0.0, 0.0, 0.0)
                except Exception:
                    gx, gy, gz = 0.0, 0.0, 0.0
                try:
                    a = self._bno.linear_acceleration
                    ax, ay, az = a if a is not None else (0.0, 0.0, 0.0)
                except Exception:
                    ax, ay, az = 0.0, 0.0, 0.0

                with self._lock:
                    self._data   = (qi, qj, qk, qr, gx, gy, gz, ax, ay, az)
                    self._data_t = time.time()

            except Exception as e:
                self.get_logger().warn(
                    f'BNO085 sample dropped: {e}', throttle_duration_sec=10.0)

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
