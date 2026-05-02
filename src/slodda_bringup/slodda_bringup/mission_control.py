#!/usr/bin/env python3
"""
Bear mission state machine with odom-based path recording and reverse-replay.

States
------
IDLE          waiting for new Nav2 goal — YOLO disarmed
NAVIGATE      Nav2 driving toward goal — YOLO armed (auto-trigger on bear)
CANCELING     holding zero-vel 400 ms after cancel_goal_async
TRACK_BEAR    P-controller: rotate → approach → complete
AT_BEAR       stopped at bear, waiting for APPROVE command
RETURN_HOME   driving backward along reversed odom path — YOLO disarmed

YOLO gating invariant
---------------------
_yolo_enabled == True   iff  state == NAVIGATE
_tracking_active == True  iff  state == TRACK_BEAR
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan

# ── Tracking P-controller ─────────────────────────────────────────────────────
KP_ANG             = 0.005
MAX_ANG            = 0.6
PIXEL_ALIGN_PX     = 20

KP_LIN             = 0.4
MAX_LIN            = 0.20
TARGET_DIST_M      = 0.20
DIST_TOL_M         = 0.02

KP_ANG_DRIFT       = 0.003
MAX_ANG_DRIFT      = 0.3
PIXEL_DRIFT_PX     = 30

LIDAR_FOV_DEG      = 15.0
LIDAR_MIN_M        = 0.05
LIDAR_MAX_M        = 2.0
LIDAR_FALLBACK_SPD = 0.05

DETECT_TIMEOUT_S   = 0.5
LOST_TIMEOUT_S     = 8.0
SEARCH_ANG_VEL     = 0.4

# ── YOLO auto-trigger ─────────────────────────────────────────────────────────
STREAK_REQ         = 3

# ── Nav2 cancel hold ──────────────────────────────────────────────────────────
CANCEL_HOLD_TICKS  = 15   # 1.5 s — gives Nav2 controller time to stop publishing

# ── Path recording ────────────────────────────────────────────────────────────
PATH_STEP_M        = 0.05
PATH_MAX_LEN       = 500

# ── Return-home controller ────────────────────────────────────────────────────
RETURN_SPD         = 0.15
RETURN_WP_M        = 0.06
KP_ANG_RETURN      = 1.2
MAX_ANG_RETURN     = 0.8
RETURN_ALIGN_RAD   = 0.4


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _adiff(a, b):
    """Signed angle difference a − b, normalised to (−π, π]."""
    d = (a - b) % (2 * math.pi)
    return d - 2 * math.pi if d > math.pi else d


class MissionControl(Node):
    IDLE        = 'IDLE'
    NAVIGATE    = 'NAVIGATE'
    CANCELING   = 'CANCELING'
    TRACK_BEAR  = 'TRACK_BEAR'
    AT_BEAR     = 'AT_BEAR'
    RETURN_HOME = 'RETURN_HOME'

    def __init__(self):
        super().__init__('mission_control')

        self.declare_parameter('image_width', 640)

        self.state           = self.IDLE
        self._cancel_ticks   = 0
        self._track_phase    = 'ROTATE'
        self._at_bear_logged = False
        self._lost_start     = None

        # YOLO gate flags
        self._yolo_enabled    = True    # Armed at startup; disarmed after missions
        self._tracking_active = False   # True only in TRACK_BEAR

        # YOLO detection data
        self.last_detection  = None
        self.last_det_time   = None
        self._streak         = 0
        self.image_width     = self.get_parameter('image_width').value

        # LiDAR
        self.last_scan       = None
        self.distance_m      = float('inf')

        # Odometry + path
        self._pose           = (0.0, 0.0, 0.0)
        self._home_odom      = (0.0, 0.0)
        self._path           = []
        self._return_path    = []
        self._return_idx     = 0

        # Nav2
        self._pending_goal: PoseStamped | None = None
        self._nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_handle  = None
        self._cancel_svc  = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_subscription(PoseStamped,      '/goal_pose',       self._goal_pose_cb, 10)
        self.create_subscription(String,           '/mission/cmd',     self._cmd_cb,       10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self._det_cb,       10)
        self.create_subscription(LaserScan,        '/scan',            self._scan_cb,      10)
        self.create_subscription(Odometry,         '/odom',            self._odom_cb,      10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',        10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        self.create_timer(0.1, self._tick)
        self.get_logger().info('MissionControl klar — sett 2D Nav2 Goal i RViz for å starte')

    # ── YOLO gate helpers ─────────────────────────────────────────────────────

    def _yolo_arm(self):
        """Enable YOLO auto-trigger. Call only when entering NAVIGATE."""
        self._streak       = 0
        self._yolo_enabled = True
        self.get_logger().info('YOLO → ARMED')

    def _yolo_disarm(self):
        """Disable YOLO auto-trigger. Preserves last_detection for TRACK_BEAR."""
        self._yolo_enabled = False
        self._streak       = 0
        self.get_logger().info('YOLO → DISARMED')

    # ── State transitions ─────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        old_state = self.state
        if new_state != old_state:
            self.get_logger().info(
                f'{old_state} → {new_state}  [yolo={self._yolo_enabled}]')

        # Defensive backstop: YOLO must not be armed outside NAVIGATE
        if new_state != self.NAVIGATE and self._yolo_enabled:
            self._yolo_disarm()

        # Sync _tracking_active: True during CANCELING and TRACK_BEAR so that
        # YOLO detections stay fresh across the 400 ms Nav2-cancel hold period.
        if new_state in (self.CANCELING, self.TRACK_BEAR):
            self._tracking_active = True
        elif new_state not in (self.CANCELING, self.TRACK_BEAR):
            self._tracking_active = False

        self.state = new_state

    # ── RViz 2D Nav2 Goal ─────────────────────────────────────────────────────

    def _goal_pose_cb(self, msg: PoseStamped):
        if self.state != self.IDLE:
            self.get_logger().warn(f'2D Goal ignorert (state={self.state})')
            return
        self._pending_goal = msg
        self._path.clear()
        self._home_odom = (self._pose[0], self._pose[1])
        self._path.append(self._home_odom)
        self._set_state(self.NAVIGATE)
        self._yolo_arm()
        self._send_nav_goal()

    # ── External commands ─────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'CMD ← {cmd}  [state={self.state}]')

        # RETURN_HOME has highest priority — only ABORT can interrupt it
        if self.state == self.RETURN_HOME:
            if cmd == 'ABORT':
                self._stop()
                self._path.clear()
                self._pending_goal = None
                self._set_state(self.IDLE)
            return

        if cmd == 'APPROVE' and self.state == self.AT_BEAR:
            self._start_return_home()

        elif cmd == 'RETURN_HOME':
            self._cancel_nav_goal()
            self._stop()
            self._start_return_home()

        elif cmd == 'ABORT':
            self._cancel_nav_goal()
            self._stop()
            self._path.clear()
            self._pending_goal = None
            self._set_state(self.IDLE)

    # ── YOLO detections ───────────────────────────────────────────────────────

    def _det_cb(self, msg: Detection2DArray):
        # RETURN_HOME: hard gate — ignore everything
        if self.state == self.RETURN_HOME:
            return

        # Always keep last_detection fresh for TRACK_BEAR visual servoing
        if msg.detections:
            self.last_detection = msg.detections[0]
            self.last_det_time  = self.get_clock().now()

        # Streak and trigger logic only when YOLO is armed (NAVIGATE state)
        if not self._yolo_enabled:
            return

        if msg.detections:
            self._streak = min(self._streak + 1, STREAK_REQ + 1)
        else:
            self._streak = 0

        if self._streak < STREAK_REQ:
            return

        # Bear confirmed — disarm before state change
        self._yolo_disarm()

        if self.state == self.IDLE:
            # Trigger directly from IDLE: record home, skip Nav2 cancel
            self._path.clear()
            self._home_odom = (self._pose[0], self._pose[1])
            self._path.append(self._home_odom)
            self._start_tracking()
        else:
            # Trigger from NAVIGATE: cancel Nav2 first, hold 1.5 s
            self._cancel_nav_goal()
            self._stop()
            self._cancel_ticks = 0
            self._set_state(self.CANCELING)

    # ── Odometry ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._pose = (p.x, p.y, yaw)
        if self.state in (self.NAVIGATE, self.CANCELING, self.TRACK_BEAR):
            self._record(p.x, p.y)

    def _record(self, x, y):
        if self._path:
            lx, ly = self._path[-1]
            if math.hypot(x - lx, y - ly) < PATH_STEP_M:
                return
        self._path.append((x, y))
        if len(self._path) > PATH_MAX_LEN:
            self._path.pop(0)

    # ── LiDAR ─────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _update_dist(self):
        if self.last_scan is None:
            self.distance_m = float('inf')
            return
        s   = self.last_scan
        fov = math.radians(LIDAR_FOV_DEG)
        vals = [
            r for i, r in enumerate(s.ranges)
            if (abs(_adiff(s.angle_min + i * s.angle_increment, 0.0)) <= fov
                and math.isfinite(r)
                and LIDAR_MIN_M <= r <= LIDAR_MAX_M)
        ]
        self.distance_m = min(vals) if vals else float('inf')

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        self.status_pub.publish(String(data=self.state))
        dispatch = {
            self.NAVIGATE:    self._tick_navigate,
            self.CANCELING:   self._tick_canceling,
            self.TRACK_BEAR:  self._tick_track_bear,
            self.AT_BEAR:     self._stop,
            self.RETURN_HOME: self._tick_return_home,
        }
        dispatch.get(self.state, lambda: None)()

    # ── NAVIGATE ──────────────────────────────────────────────────────────────

    def _send_nav_goal(self):
        if self._pending_goal is None:
            self.get_logger().error('Ingen mål — sett 2D Goal i RViz')
            self._set_state(self.IDLE)
            return
        pose = PoseStamped()
        pose.header.frame_id = self._pending_goal.header.frame_id or 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose            = self._pending_goal.pose
        self.get_logger().info(
            f'Nav2 mål ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 ikke tilgjengelig')
            self._set_state(self.IDLE)
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._nav_client.send_goal_async(goal).add_done_callback(self._nav_goal_cb)

    def _nav_goal_cb(self, future):
        h = future.result()
        if not h.accepted:
            self.get_logger().warn('Nav2 mål avvist — holder NAVIGATE')
            return
        if self.state != self.NAVIGATE:
            # YOLO triggered before this callback arrived — cancel the handle now
            h.cancel_goal_async()
            return
        self._nav_handle = h
        h.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        if self.state != self.NAVIGATE:
            return
        status = future.result().status
        if status == 4:
            self.get_logger().info('Nav2 mål nådd — starter tracking')
            self._start_tracking()
        else:
            self.get_logger().warn(f'Navigasjon feilet (status={status})')
            self._set_state(self.IDLE)

    def _cancel_nav_goal(self):
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()
            self._nav_handle = None
        elif self._cancel_svc.service_is_ready():
            # No handle — bt_navigator is navigating on its own. Cancel all goals.
            self._cancel_svc.call_async(CancelGoal.Request())

    def _tick_navigate(self):
        pass  # auto-trigger handled in _det_cb

    # ── CANCELING ─────────────────────────────────────────────────────────────

    def _tick_canceling(self):
        self._stop()
        self._cancel_ticks += 1
        # Re-send cancel every tick — Nav2 may take several hundred ms to stop
        if self._nav_handle is None and self._cancel_svc.service_is_ready():
            self._cancel_svc.call_async(CancelGoal.Request())
        if self._cancel_ticks >= CANCEL_HOLD_TICKS:
            self._start_tracking()

    # ── TRACK_BEAR ────────────────────────────────────────────────────────────

    def _start_tracking(self):
        self._track_phase    = 'ROTATE'
        self._at_bear_logged = False
        self._lost_start     = None
        self._set_state(self.TRACK_BEAR)

    def _pixel_error(self):
        if self.last_detection is None or self.last_det_time is None:
            return 0.0, False
        dt = (self.get_clock().now() - self.last_det_time).nanoseconds / 1e9
        if dt > DETECT_TIMEOUT_S:
            return 0.0, False
        return self.last_detection.bbox.center.position.x - self.image_width / 2.0, True

    def _tick_track_bear(self):
        self._update_dist()
        err, fresh = self._pixel_error()

        if not fresh:
            if self._lost_start is None:
                self._lost_start = self.get_clock().now()
                self.get_logger().warn('Mistet bjørn — søker')
            elapsed = (self.get_clock().now() - self._lost_start).nanoseconds / 1e9
            if elapsed > LOST_TIMEOUT_S:
                self.get_logger().warn('Mistet bjørn for lenge — avbryter')
                self._stop()
                self._set_state(self.IDLE)
                self._lost_start = None
                return
            self._pub_twist(0.0, SEARCH_ANG_VEL)
            return

        if self._lost_start is not None:
            self.get_logger().info('Bjørn gjenfunnet')
            self._lost_start  = None
            self._track_phase = 'ROTATE'

        if self._track_phase == 'ROTATE':
            self._phase_rotate(err)
        elif self._track_phase == 'APPROACH':
            self._phase_approach(err)
        elif self._track_phase == 'COMPLETE':
            self._phase_complete()

    def _phase_rotate(self, err):
        if abs(err) < PIXEL_ALIGN_PX:
            self.get_logger().info('Sentrert — starter tilnærming')
            self._track_phase = 'APPROACH'
            return
        self._pub_twist(0.0, _clamp(-KP_ANG * err, -MAX_ANG, MAX_ANG))

    def _phase_approach(self, err):
        d = self.distance_m
        if not math.isfinite(d):
            self._pub_twist(LIDAR_FALLBACK_SPD, 0.0)
            return
        if abs(d - TARGET_DIST_M) < DIST_TOL_M and abs(err) < PIXEL_ALIGN_PX:
            self._track_phase = 'COMPLETE'
            return
        lin = _clamp(KP_LIN * (d - TARGET_DIST_M), 0.0, MAX_LIN)
        ang = 0.0
        if abs(err) > PIXEL_DRIFT_PX:
            ang = _clamp(-KP_ANG_DRIFT * err, -MAX_ANG_DRIFT, MAX_ANG_DRIFT)
        self._pub_twist(lin, ang)

    def _phase_complete(self):
        if not self._at_bear_logged:
            self._at_bear_logged = True
            d_cm = self.distance_m * 100.0
            self.get_logger().info(
                f'NÅDD BJØRN ({d_cm:.1f} cm) — venter på APPROVE-kommando')
            self._stop()
            self._set_state(self.AT_BEAR)

    # ── RETURN HOME ───────────────────────────────────────────────────────────

    def _start_return_home(self):
        if not self._path:
            self.get_logger().warn('Ingen path å følge — setter IDLE')
            self._set_state(self.IDLE)
            return
        self._lost_start  = None
        self._return_path = list(reversed(self._path))
        self._return_idx  = 0
        self._path.clear()
        # _set_state backstop disarms YOLO and clears _tracking_active
        self._set_state(self.RETURN_HOME)
        self.get_logger().info(f'Return Home — {len(self._return_path)} waypoints')

    def _tick_return_home(self):
        if self._return_idx >= len(self._return_path):
            cx, cy, _ = self._pose
            dist_home = math.hypot(cx - self._home_odom[0], cy - self._home_odom[1])
            self.get_logger().info(f'Hjemme! ({dist_home * 100:.1f} cm fra spawn)')
            self._stop()
            self._set_state(self.IDLE)
            return

        cx, cy, cyaw = self._pose
        tx, ty = self._return_path[self._return_idx]
        dx, dy = tx - cx, ty - cy
        dist   = math.hypot(dx, dy)

        if dist < RETURN_WP_M:
            self._return_idx += 1
            return

        angle_to_wp = math.atan2(dy, dx)
        desired_yaw = angle_to_wp + math.pi
        heading_err = _adiff(desired_yaw, cyaw)

        angular = _clamp(KP_ANG_RETURN * heading_err, -MAX_ANG_RETURN, MAX_ANG_RETURN)
        linear  = -RETURN_SPD if abs(heading_err) < RETURN_ALIGN_RAD else 0.0
        self._pub_twist(linear, angular)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _pub_twist(self, lin: float, ang: float):
        t = Twist()
        t.linear.x  = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def _stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
