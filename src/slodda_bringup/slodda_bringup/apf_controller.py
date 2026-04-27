import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

GOAL_TOL    = 0.10
MAX_LINEAR  = 0.25
MAX_ANGULAR = 1.2
MIN_LINEAR  = 0.10
IR_STOP     = 0.12   # stopp hvis hindring nærmere enn 12 cm


class ApfController(Node):
    def __init__(self):
        super().__init__('apf_controller')

        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.path       = []
        self.current_x  = 0.0
        self.current_y  = 0.0
        self.current_yaw = 0.0
        self.ir = {'left': 1.0, 'center': 1.0, 'right': 1.0}

        self.create_subscription(Path,      '/planned_path',   self.path_cb,      qos_latched)
        self.create_subscription(Odometry,  '/odom',           self.odom_cb,      10)
        self.create_subscription(LaserScan, '/ir_front_left',  self.ir_left_cb,   10)
        self.create_subscription(LaserScan, '/ir_front_center',self.ir_center_cb, 10)
        self.create_subscription(LaserScan, '/ir_front_right', self.ir_right_cb,  10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('APF controller klar, venter på sti...')

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y)
                     for p in msg.poses]
        self.get_logger().info(f'Mottok sti med {len(self.path)} waypoints')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny  = 2.0 * (q.w * q.z + q.x * q.y)
        cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def ir_left_cb(self,   msg): self.ir['left']   = self._safe_range(msg)
    def ir_center_cb(self, msg): self.ir['center']  = self._safe_range(msg)
    def ir_right_cb(self,  msg): self.ir['right']   = self._safe_range(msg)

    def _safe_range(self, msg):
        ranges = [r for r in msg.ranges if math.isfinite(r) and r > 0.02]
        return min(ranges) if ranges else 1.0

    def _angle_err(self, desired):
        err = desired - self.current_yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi
        return err

    def control_loop(self):
        if not self.path:
            return

        # Fjern nådde waypoints
        while self.path:
            tx, ty = self.path[0]
            dist = math.sqrt((tx - self.current_x)**2 +
                             (ty - self.current_y)**2)
            if dist < GOAL_TOL:
                self.path.pop(0)
                if self.path:
                    self.get_logger().info(
                        f'Waypoint nådd, {len(self.path)} igjen')
                else:
                    self.get_logger().info('MÅL NÅDD!')
                    self.stop()
                    return
            else:
                break

        if not self.path:
            return

        tx, ty = self.path[0]
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.sqrt(dx**2 + dy**2)

        desired_yaw = math.atan2(dy, dx)
        herr = self._angle_err(desired_yaw)

        cmd = Twist()

        # Hindring foran — stopp og drei
        if self.ir['center'] < IR_STOP:
            self.get_logger().warn(
                f'Hindring {self.ir["center"]:.2f}m — snur')
            cmd.linear.x  =  0.0
            cmd.angular.z =  MAX_ANGULAR
            self.cmd_pub.publish(cmd)
            return

        # Heading-first: drei på stedet hvis vi peker mye feil
        if abs(herr) > math.radians(20):
            cmd.linear.x  = 0.0
            cmd.angular.z = max(-MAX_ANGULAR,
                                min(MAX_ANGULAR, 2.0 * herr))
        else:
            # Kjør fremover, korriger kurs lett
            speed = min(MAX_LINEAR, max(MIN_LINEAR, 0.5 * dist))
            cmd.linear.x  = speed
            cmd.angular.z = max(-MAX_ANGULAR,
                                min(MAX_ANGULAR, 1.5 * herr))

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.path = []


def main(args=None):
    rclpy.init(args=args)
    node = ApfController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
