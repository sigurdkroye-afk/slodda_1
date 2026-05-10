#!/usr/bin/env python3
"""
BNO085 IMU node — publishes sensor_msgs/Imu on /imu/data.

Requires adafruit_bno08x and adafruit_blinka on the Raspberry Pi:
  pip install adafruit-circuitpython-bno08x adafruit-blinka

Default I2C address: 0x4A (alt: 0x4B if ADR pin is high).

Requires hardware I2C enabled in /boot/firmware/config.txt:
  dtparam=i2c_arm=on,i2c_arm_baudrate=400000
Hardware I2C (BCM2711 BSC) handles BNO085 SHTP clock-stretching correctly.
Software I2C (i2c-gpio) cannot — it permanently blocks the bus under CPU load.
"""
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

_ORI_VAR   = 4e-6    # rad²
_GYRO_VAR  = 8.1e-5  # rad²/s²
_ACCEL_VAR = 4.4e-4  # m²/s⁴


def _diag9(v: float):
    m = [0.0] * 9
    m[0] = m[4] = m[8] = v
    return m


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('publish_hz',  50.0)
        self.declare_parameter('frame_id',    'imu_link')
        self.declare_parameter('i2c_address', 0x4A)

        self._pub = self.create_publisher(Imu, '/imu/data', 10)
        self._bno = None

        if not _HW_AVAILABLE:
            self.get_logger().error(
                'adafruit_bno08x not installed — '
                'pip install adafruit-circuitpython-bno08x adafruit-blinka')
        else:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self._bno = BNO08X_I2C(
                i2c, address=self.get_parameter('i2c_address').value)
            self._bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
            self._bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self._bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info('ImuNode ready.')

    def _tick(self):
        if self._bno is None:
            return

        q = self._bno.game_quaternion
        if q is None:
            return
        qi, qj, qk, qr = q

        g = self._bno.gyro
        gx, gy, gz = g if g is not None else (0.0, 0.0, 0.0)

        a = self._bno.linear_acceleration
        ax, ay, az = a if a is not None else (0.0, 0.0, 0.0)

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value

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
