#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray
from rclpy.duration import Duration
import math


class BearMission(Node):
    IDLE = 'IDLE'
    NAVIGATE = 'NAVIGATE_TO_GOAL'
    SEARCH = 'SEARCH'
    APPROACH = 'APPROACH'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('bear_mission')

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('search_timeout', 30.0)
        self.declare_parameter('approach_fwd_speed', 0.15)
        self.declare_parameter('approach_k_ang', 0.005)
        self.declare_parameter('bbox_area_threshold', 0.15)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self.state = self.IDLE
        self.last_detection = None
        self.last_det_time = None
        self.search_start = None
        self._nav_handle = None

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(Bool, '/bear_mission/start', self.start_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.det_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/bear_mission/status', 10)
        self.track_pub = self.create_publisher(Bool, '/track/enable', 10)

        self.create_timer(0.1, self.tick)
        self.get_logger().info('BearMission klar. Publiser /bear_mission/start=true for å starte.')

    def start_cb(self, msg: Bool):
        if msg.data and self.state == self.IDLE:
            self.get_logger().info('Oppdrag startet!')
            self._disable_tracker()
            self._set_state(self.NAVIGATE)
            self._send_nav_goal()

    def det_cb(self, msg: Detection2DArray):
        if msg.detections:
            self.last_detection = msg.detections[0]
            self.last_det_time = self.get_clock().now()

    def tick(self):
        self.status_pub.publish(String(data=self.state))

        if self.state == self.SEARCH:
            self._tick_search()
        elif self.state == self.APPROACH:
            self._tick_approach()

    def _tick_search(self):
        timeout = self.get_parameter('search_timeout').value
        if self.search_start and (self.get_clock().now() - self.search_start).nanoseconds > timeout * 1e9:
            self.get_logger().warn('Søk timeout — ingen teddybjørn funnet.')
            self._finish('FAILED_SEARCH_TIMEOUT')
            return

        # Sjekk om deteksjon er fersk (< 0.5s)
        if self.last_det_time and (self.get_clock().now() - self.last_det_time).nanoseconds < 5e8:
            self.get_logger().info('Teddybjørn detektert! Starter tilnærming.')
            self._stop()
            self._set_state(self.APPROACH)
            return

        # Rotér på stedet
        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)

    def _tick_approach(self):
        W = self.get_parameter('image_width').value
        H = self.get_parameter('image_height').value
        k_ang = self.get_parameter('approach_k_ang').value
        fwd = self.get_parameter('approach_fwd_speed').value
        threshold = self.get_parameter('bbox_area_threshold').value

        # Sjekk at deteksjon er fersk
        if not self.last_det_time or (self.get_clock().now() - self.last_det_time).nanoseconds > 1e9:
            self.get_logger().info('Mistet bjørn — søker på nytt.')
            self._set_state(self.SEARCH)
            self.search_start = self.get_clock().now()
            return

        det = self.last_detection
        cx = det.bbox.center.position.x
        area = det.bbox.size_x * det.bbox.size_y
        area_frac = area / (W * H)

        if area_frac >= threshold:
            self.get_logger().info(f'Nær nok! Areal={area_frac:.2f}. Stopper.')
            self._finish('SUCCESS')
            return

        twist = Twist()
        twist.linear.x = fwd
        twist.angular.z = -k_ang * (cx - W / 2.0)
        self.cmd_pub.publish(twist)

    def _send_nav_goal(self):
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value

        self.get_logger().info(f'Navigerer til ({goal_x}, {goal_y})...')
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 ikke tilgjengelig — avbryter.')
            self._finish('FAILED_NAV_SERVER')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_cb)

    def _nav_goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2-mål avvist.')
            self._finish('FAILED_NAV')
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Målpunkt nådd. Starter søk.')
            self._set_state(self.SEARCH)
            self.search_start = self.get_clock().now()
        else:
            self.get_logger().warn(f'Navigasjon feilet, status={status}.')
            self._finish('FAILED_NAV')

    def _set_state(self, state):
        self.get_logger().info(f'State: {self.state} → {state}')
        self.state = state

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _disable_tracker(self):
        self.track_pub.publish(Bool(data=False))

    def _finish(self, status):
        self._stop()
        self.track_pub.publish(Bool(data=True))
        self._set_state(self.DONE)
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f'Oppdrag avsluttet: {status}')
        # Reset til IDLE etter 2s
        self.create_timer(2.0, self._reset_to_idle)

    def _reset_to_idle(self):
        self._set_state(self.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = BearMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
