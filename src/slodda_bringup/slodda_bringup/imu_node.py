#!/usr/bin/env python3
"""
BNO055 IMU node — publishes sensor_msgs/Imu on /imu/data.

Requires adafruit-circuitpython-bno055 on the Raspberry Pi:
  pip install adafruit-circuitpython-bno055 adafruit-blinka

Uses hardware I2C bus 1 (GPIO 2/3). Requires in /boot/firmware/config.txt:
  dtparam=i2c_arm=on
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import board
    import busio
    import adafruit_bno055
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
        self.declare_parameter('i2c_address', 0x28)

        self._pub = self.create_publisher(Imu, '/imu/data', 10)
        self._bno = None

        if not _HW_AVAILABLE:
            self.get_logger().error(
                'adafruit-circuitpython-bno055 not installed — '
                'pip install adafruit-circuitpython-bno055')
        else:
            try:
                i2c = busio.I2C(board.SCL, board.SDA)
                addr = self.get_parameter('i2c_address').value
                self._bno = adafruit_bno055.BNO055_I2C(i2c, address=addr)
                self.get_logger().info('BNO055 initialisert OK.')
            except Exception as e:
                self.get_logger().error(
                    f'BNO055 init failed — sjekk kobling og adresse: {e}')
                self._bno = None

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info('ImuNode ready.')

    def _tick(self):
        if self._bno is None:
            return
        try:
            q = self._bno.quaternion  # (w, x, y, z)
            if q is None or None in q:
                return

            g = self._bno.gyro                  # (x, y, z) rad/s
            a = self._bno.linear_acceleration   # (x, y, z) m/s²

            w, qx, qy, qz = q
            gx, gy, gz = g if g is not None else (0.0, 0.0, 0.0)
            ax, ay, az = a if a is not None else (0.0, 0.0, 0.0)

            msg = Imu()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = self.get_parameter('frame_id').value

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = w
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
        except Exception as e:
            self.get_logger().warn(
                f'BNO055 lesefeil: {e}', throttle_duration_sec=5.0)


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
