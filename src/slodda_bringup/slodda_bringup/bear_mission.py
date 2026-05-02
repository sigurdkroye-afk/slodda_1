#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration
import math

# ── ALIGN_AND_APPROACH ──────────────────────────────────────────────────────
KP_ANGULAR              = 0.005   # rad/s per pixel
MAX_ANGULAR_VEL         = 0.6     # rad/s
PIXEL_ALIGN_THRESHOLD   = 20      # px — alignment complete
DETECTION_TIMEOUT_SEC   = 0.5     # s  — detection considered stale

KP_LINEAR               = 0.4     # (m/s) per m of distance error
MAX_LINEAR_VEL          = 0.20    # m/s
TARGET_DISTANCE_M       = 0.20    # m
DISTANCE_TOLERANCE_M    = 0.02    # m  (±2 cm)
LIDAR_FALLBACK_SPEED    = 0.05    # m/s — used when LiDAR returns inf

PIXEL_DRIFT_THRESHOLD   = 30      # px — re-alignment trigger during approach
KP_ANGULAR_DRIFT        = 0.003   # rad/s per pixel (gentler)
MAX_ANGULAR_DRIFT       = 0.3     # rad/s

LIDAR_FOV_DEG           = 15.0    # ± half-angle in front sector
LIDAR_MIN_RANGE_M       = 0.05    # m
LIDAR_MAX_RANGE_M       = 2.0     # m

LOST_TARGET_FRAMES      = 5       # consecutive missed ticks → LOST_TARGET
LOST_TARGET_TIMEOUT_SEC = 8.0     # s — give up and stop mission
SEARCH_ANGULAR_VEL      = 0.4     # rad/s — lost-target rotation speed


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class BearMission(Node):
    IDLE = 'IDLE'
    NAVIGATE = 'NAVIGATE_TO_GOAL'
    CANCELING = 'CANCELING'
    SEARCH = 'SEARCH'
    APPROACH = 'APPROACH'
    DONE = 'DONE'
    ALIGN_AND_APPROACH = 'ALIGN_AND_APPROACH'
    LOST_TARGET        = 'LOST_TARGET'

    # Ticks (à 100 ms) to hold zero-velocity after canceling Nav2 goal before
    # publishing our own cmd_vel — ensures Nav2's controller_server has stopped.
    CANCEL_HOLD_TICKS = 4

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
        self.declare_parameter('use_align_and_approach', True)

        self.state = self.IDLE
        self.last_detection = None
        self.last_det_time = None
        self.search_start = None
        self._nav_handle = None
        self._cancel_ticks = 0

        # LiDAR
        self.last_scan      = None
        self.distance_m     = float('inf')
        self.too_close      = False

        # detection-loss tracking
        self.missed_frames      = 0
        self.lost_target_start  = None

        # ALIGN_AND_APPROACH sub-phase
        self.aa_phase           = None      # 'ROTATE' | 'APPROACH' | 'COMPLETE'
        self._completion_logged = False

        # image width (used by _pixel_error)
        self.image_width = self.get_parameter('image_width').value

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(Bool, '/bear_mission/start', self.start_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.det_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

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

        if self.state == self.NAVIGATE:
            self._tick_navigate()
        elif self.state == self.CANCELING:
            self._tick_canceling()
        elif self.state == self.SEARCH:
            self._tick_search()
        elif self.state == self.APPROACH:
            self._tick_approach()
        elif self.state == self.ALIGN_AND_APPROACH:
            self._tick_align_and_approach()
        elif self.state == self.LOST_TARGET:
            self._tick_lost_target()

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
            if self.get_parameter('use_align_and_approach').value:
                self.aa_phase = 'ROTATE'
                self.missed_frames = 0
                self._completion_logged = False
                self._set_state(self.ALIGN_AND_APPROACH)
            else:
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
        if self.state != self.NAVIGATE:
            # Bear was seen mid-navigation — we already moved on; cancel the orphan goal.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().warn('Nav2-mål avvist.')
            self._finish('FAILED_NAV')
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        if self.state != self.NAVIGATE:
            return  # Bear was seen mid-navigation — already moved on
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Målpunkt nådd. Starter søk.')
            self._set_state(self.SEARCH)
            self.search_start = self.get_clock().now()
        else:
            self.get_logger().warn(f'Navigasjon feilet, status={status}.')
            self._finish('FAILED_NAV')

    def _tick_navigate(self):
        """During navigation: if bear is visible, cancel Nav2 goal and go direct."""
        if (self.last_det_time and
                (self.get_clock().now() - self.last_det_time).nanoseconds < 5e8):
            self.get_logger().info('Teddybjørn sett under navigasjon — avbryter Nav2-mål!')
            self._cancel_nav_goal()
            self._stop()
            self._cancel_ticks = 0
            self._set_state(self.CANCELING)

    def _tick_canceling(self):
        """Hold zero velocity for CANCEL_HOLD_TICKS (400 ms) after canceling the Nav2
        goal, so controller_server has time to stop publishing cmd_vel before we take
        over.  Then transition to ALIGN_AND_APPROACH."""
        self._stop()
        self._cancel_ticks += 1
        if self._cancel_ticks >= self.CANCEL_HOLD_TICKS:
            if self.get_parameter('use_align_and_approach').value:
                self.aa_phase = 'ROTATE'
                self.missed_frames = 0
                self._completion_logged = False
                self._set_state(self.ALIGN_AND_APPROACH)
            else:
                self._set_state(self.APPROACH)

    def _cancel_nav_goal(self):
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None

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
        self._idle_timer = self.create_timer(2.0, self._reset_to_idle)

    def _reset_to_idle(self):
        self._idle_timer.cancel()
        self._set_state(self.IDLE)

    # ── LiDAR ───────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _compute_front_distance(self):
        if self.last_scan is None:
            self.distance_m = float('inf')
            self.too_close = False
            return
        scan = self.last_scan
        fov = math.radians(LIDAR_FOV_DEG)
        valid = []
        below_min = 0
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(angle), math.cos(angle))
            if -fov <= a <= fov and math.isfinite(r):
                if r < LIDAR_MIN_RANGE_M:
                    below_min += 1
                elif r <= LIDAR_MAX_RANGE_M:
                    valid.append(r)
        self.too_close = below_min > 0
        self.distance_m = min(valid) if valid else float('inf')

    # ── Detection helpers ────────────────────────────────────────────────────────

    def _pixel_error(self):
        """Returns (pixel_error, is_fresh). is_fresh=False means detection is stale."""
        if self.last_detection is None or self.last_det_time is None:
            return 0.0, False
        dt = (self.get_clock().now() - self.last_det_time).nanoseconds / 1e9
        if dt > DETECTION_TIMEOUT_SEC:
            return 0.0, False
        cx = self.last_detection.bbox.center.position.x
        return cx - self.image_width / 2.0, True

    def _publish_twist(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    # ── LOST_TARGET ──────────────────────────────────────────────────────────────

    def _enter_lost_target(self):
        self.get_logger().warn('Mistet bjørn — LOST_TARGET')
        self._set_state(self.LOST_TARGET)
        self.lost_target_start = self.get_clock().now()
        self.missed_frames = 0

    def _tick_lost_target(self):
        elapsed = (self.get_clock().now() - self.lost_target_start).nanoseconds / 1e9
        if elapsed > LOST_TARGET_TIMEOUT_SEC:
            self.get_logger().warn('LOST_TARGET timed out — avbryter oppdrag')
            self._publish_twist(0.0, 0.0)
            self._finish('LOST')
            return
        err, fresh = self._pixel_error()
        if fresh:
            self.get_logger().info('Bjørn gjenfunnet — tilbake til fase 1')
            self._set_state(self.ALIGN_AND_APPROACH)
            self.aa_phase = 'ROTATE'
            self.missed_frames = 0
            return
        self._publish_twist(0.0, SEARCH_ANGULAR_VEL)

    # ── ALIGN_AND_APPROACH phases ────────────────────────────────────────────────

    def _aa_rotate(self):
        """Phase 1: rotate to centre bear in frame (zero linear velocity)."""
        err, fresh = self._pixel_error()
        if not fresh:
            self.missed_frames += 1
            if self.missed_frames >= LOST_TARGET_FRAMES:
                self._enter_lost_target()
            self._publish_twist(0.0, 0.0)
            return
        self.missed_frames = 0
        if abs(err) < PIXEL_ALIGN_THRESHOLD:
            self.get_logger().info('Fase 1 fullført — bjørn sentrert. Starter tilnærming.')
            self.aa_phase = 'APPROACH'
            return
        ang = clamp(-KP_ANGULAR * err, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
        self._publish_twist(0.0, ang)

    def _aa_approach(self):
        """Phase 3: drive forward using LiDAR distance control."""
        err, fresh = self._pixel_error()
        if not fresh:
            self.missed_frames += 1
            if self.missed_frames >= LOST_TARGET_FRAMES:
                self._enter_lost_target()
            self._publish_twist(0.0, 0.0)
            return
        self.missed_frames = 0

        d = self.distance_m
        ang = 0.0
        if abs(err) > PIXEL_DRIFT_THRESHOLD:
            ang = clamp(-KP_ANGULAR_DRIFT * err, -MAX_ANGULAR_DRIFT, MAX_ANGULAR_DRIFT)

        if not math.isfinite(d):
            if self.too_close:
                self.aa_phase = 'COMPLETE'
                return
            self._publish_twist(LIDAR_FALLBACK_SPEED, ang)
            return

        if d <= TARGET_DISTANCE_M + DISTANCE_TOLERANCE_M and abs(err) < PIXEL_ALIGN_THRESHOLD:
            self.aa_phase = 'COMPLETE'
            return

        lin = clamp(KP_LINEAR * (d - TARGET_DISTANCE_M), 0.0, MAX_LINEAR_VEL)
        self._publish_twist(lin, ang)

    def _aa_complete(self):
        """Phase 4: full stop, log, and end mission."""
        if not self._completion_logged:
            d_cm = self.distance_m * 100.0
            self.get_logger().info(
                f'TASK COMPLETE: Aligned and stopped {d_cm:.1f}cm from teddy bear')
            self._completion_logged = True
            self._finish('SUCCESS')

    def _tick_align_and_approach(self):
        self._compute_front_distance()
        if self.aa_phase == 'ROTATE':
            self._aa_rotate()
        elif self.aa_phase == 'APPROACH':
            self._aa_approach()
        elif self.aa_phase == 'COMPLETE':
            self._aa_complete()


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
