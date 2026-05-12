#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from collections import deque
import math

# ALIGN_AND_APPROACH (beholdt)
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
    # State constants
    IDLE               = 'IDLE'
    NAVIGATE           = 'NAVIGATE_TO_GOAL'
    CANCELING          = 'CANCELING'
    SEARCH             = 'SEARCH'
    APPROACH           = 'APPROACH'
    DONE               = 'DONE'
    ALIGN_AND_APPROACH = 'ALIGN_AND_APPROACH'   # beholdt, bypassed i ny flyt
    LOST_TARGET        = 'LOST_TARGET'
    VERIFY_BEAR        = 'VERIFY_BEAR'
    VISUAL_SERVO       = 'VISUAL_SERVO'
    RETURN_HOME        = 'RETURN_HOME'
    FINAL_RELEASE      = 'FINAL_RELEASE'

    # Ticks (à 100 ms) to hold zero-velocity after canceling Nav2 goal before
    # publishing our own cmd_vel — ensures Nav2's controller_server has stopped.
    CANCEL_HOLD_TICKS = 4

    def __init__(self):
        super().__init__('bear_mission')

        # Original params
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('search_timeout', 30.0)
        self.declare_parameter('approach_fwd_speed', 0.15)
        self.declare_parameter('approach_k_ang', 0.005)
        self.declare_parameter('bbox_area_threshold', 0.15)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('use_align_and_approach', True)

        # Visual servo params
        self.declare_parameter('target_cx_offset_norm', -0.15)
        self.declare_parameter('k_p_yaw', 1.5)
        self.declare_parameter('max_lin_servo', 0.05)
        self.declare_parameter('max_ang_servo', 0.5)
        self.declare_parameter('heading_tolerance', 0.10)
        self.declare_parameter('lost_timeout_s', 15.0)
        self.declare_parameter('drive_stale_s',  3.0)   # stop driving if detection older than this
        self.declare_parameter('grab_timeout_s', 60.0)

        # Path replay params
        self.declare_parameter('replay_lin_speed', 0.10)
        self.declare_parameter('replay_xy_tol', 0.15)
        self.declare_parameter('replay_yaw_tol', 0.25)

        # Original state attrs
        self.state              = self.IDLE
        self.last_detection     = None
        self.last_det_time      = None
        self.search_start       = None
        self._nav_handle        = None
        self._cancel_ticks      = 0

        # LiDAR
        self.last_scan      = None
        self.distance_m     = float('inf')
        self.too_close      = False

        # detection-loss tracking
        self.missed_frames      = 0
        self.lost_target_start  = None

        # ALIGN_AND_APPROACH sub-phase
        self.aa_phase           = None
        self._completion_logged = False

        self.image_width = self.get_parameter('image_width').value

        # Arm service clients
        self._arm_clients = {
            'open':   self.create_client(Trigger, '/arm/open'),
            'drive':  self.create_client(Trigger, '/arm/drive'),
            'search': self.create_client(Trigger, '/arm/search'),
            'grab':   self.create_client(Trigger, '/arm/grab'),
            'stop':   self.create_client(Trigger, '/arm/stop'),
        }
        self._last_arm_status = ''
        self._arm_grabbed     = False

        # Odom path-buffer
        self._latest_odom   = None
        self._path_buffer   = deque(maxlen=500)
        self._caching_path  = False
        self._start_pose    = None          # (x, y, yaw) captured at mission start

        # YOLO verification
        # Each entry: (ROS timestamp, confidence, Detection2D.bbox)
        self._yolo_recent       = deque(maxlen=5)
        self._yolo_verify_count = 1
        self._yolo_verify_conf  = 0.40

        # Per-state timers / futures
        self._verify_t0             = None
        self._verify_future         = None
        self._servo_t0              = None
        self._last_yolo_t           = None
        self._lost_search_started   = None
        self._replay_queue          = []
        self._replay_idx            = 0
        self._release_t0            = None
        self._release_future        = None

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.create_subscription(Bool,             '/bear_mission/start', self.start_cb,       10)
        self.create_subscription(Detection2DArray, '/yolo/detections',    self.det_cb,         10)
        self.create_subscription(LaserScan,        '/scan',               self._scan_cb,       10)
        self.create_subscription(String,           '/arm/status',         self._on_arm_status, 10)
        self.create_subscription(Odometry,         '/odom',               self._on_odom,       10)

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',             10)
        self.status_pub = self.create_publisher(String, '/bear_mission/status', 10)
        self.track_pub  = self.create_publisher(Bool,   '/track/enable',        10)

        self.create_timer(0.1, self.tick)
        self.get_logger().info('BearMission klar. Publiser /bear_mission/start=true for å starte.')

    # Callbacks

    def start_cb(self, msg: Bool):
        if msg.data and self.state == self.IDLE:
            if self._latest_odom is None:
                self.get_logger().warn('Ingen /odom ennå — venter 2s og prøver igjen...')
                self.create_timer(2.0, lambda: self.start_cb(msg))
                return
            self.get_logger().info('Oppdrag startet!')
            self._disable_tracker()
            self._start_pose = self._odom_to_tuple(self._latest_odom)
            self._path_buffer.clear()
            self._caching_path = True
            self._call_arm_async('drive')   # deploy arm to drive pos (fire-and-forget)

            if self._bear_recently_seen(window_s=30.0):
                self.get_logger().info(
                    'Bamse allerede synlig ved start — hopper over Nav2, går rett til VERIFY_BEAR.'
                )
                self._caching_path     = False
                self._last_arm_status  = ''
                self._verify_t0        = self.get_clock().now()
                self._verify_future    = self._call_arm_async('search')
                self._set_state(self.VERIFY_BEAR)
                return

            self._set_state(self.NAVIGATE)
            self._send_nav_goal()

    def det_cb(self, msg: Detection2DArray):
        """Update last_detection (class 77, conf≥0.15) and YOLO verify deque (conf≥0.40)."""
        best_score = 0.0
        best_det   = None
        for det in msg.detections:
            for r in det.results:
                if r.hypothesis.class_id == '77' and r.hypothesis.score > best_score:
                    best_score = r.hypothesis.score
                    best_det   = det
        if best_det is None:
            return
        if best_score >= 0.15:
            self.last_detection = best_det
            self.last_det_time  = self.get_clock().now()
        if best_score >= self._yolo_verify_conf:
            self._yolo_recent.append((
                self.get_clock().now(),
                best_score,
                best_det.bbox,
            ))

    def _on_arm_status(self, msg: String):
        self._last_arm_status = msg.data.strip()
        if self._last_arm_status == 'GRABBED':
            self._arm_grabbed = True

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg
        if self._caching_path:
            pos = self._odom_to_tuple(msg)
            if (not self._path_buffer or
                    self._pose_dist(self._path_buffer[-1], pos) >= 0.05):
                self._path_buffer.append(pos)

    # Tick dispatcher

    def tick(self):
        self.status_pub.publish(String(data=self.state))

        if   self.state == self.NAVIGATE:           self._tick_navigate()
        elif self.state == self.CANCELING:          self._tick_canceling()
        elif self.state == self.SEARCH:             self._tick_search()
        elif self.state == self.APPROACH:           self._tick_approach()
        elif self.state == self.ALIGN_AND_APPROACH: self._tick_align_and_approach()
        elif self.state == self.LOST_TARGET:        self._tick_lost_target()
        elif self.state == self.VERIFY_BEAR:        self._tick_verify_bear()
        elif self.state == self.VISUAL_SERVO:       self._tick_visual_servo()
        elif self.state == self.RETURN_HOME:        self._tick_return_home()
        elif self.state == self.FINAL_RELEASE:      self._tick_final_release()

    # NAVIGATE

    def _tick_navigate(self):
        high_conf = self._yolo_verified()
        low_conf  = self._bear_recently_seen(window_s=1.0)
        if high_conf or low_conf:
            reason = 'high-conf' if high_conf else 'low-conf'
            self.get_logger().info(
                f'Bamse sett under NAVIGATE ({reason}) — avbryter Nav2!'
            )
            self._cancel_nav_goal()
            self._stop()
            self._cancel_ticks = 0
            self._set_state(self.CANCELING)

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
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.w = 1.0
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_cb)

    def _nav_goal_cb(self, future):
        handle = future.result()
        if self.state != self.NAVIGATE:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            if self._bear_recently_seen(window_s=30.0):
                self.get_logger().warn(
                    'Nav2-mål avvist, men bamse nylig sett — går til VERIFY_BEAR.'
                )
                self._stop()
                self._caching_path    = False
                self._last_arm_status = ''
                self._verify_t0       = self.get_clock().now()
                self._verify_future   = self._call_arm_async('search')
                self._set_state(self.VERIFY_BEAR)
                return
            self.get_logger().warn('Nav2-mål avvist.')
            self._finish('FAILED_NAV')
            return
        self._nav_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        if self.state != self.NAVIGATE:
            return
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Målpunkt nådd — starter søk.')
            self._caching_path = False
            self._set_state(self.SEARCH)
            self.search_start = self.get_clock().now()
            return

        if self._bear_recently_seen(window_s=3.0):
            self.get_logger().warn(
                f'Nav2 feilet (status={status}), men bamse nylig sett — '
                f'går til VERIFY_BEAR.'
            )
            self._stop()
            self._caching_path    = False
            self._last_arm_status = ''
            self._verify_t0       = self.get_clock().now()
            self._verify_future   = self._call_arm_async('search')
            self._set_state(self.VERIFY_BEAR)
            return

        self.get_logger().warn(f'Navigasjon feilet, status={status}.')
        self._finish('FAILED_NAV')

    # CANCELING

    def _tick_canceling(self):
        self._stop()
        self._cancel_ticks += 1
        if self._cancel_ticks >= self.CANCEL_HOLD_TICKS:
            self._caching_path = False
            self._last_arm_status = ''
            self._verify_t0    = self.get_clock().now()
            self._verify_future = self._call_arm_async('search')
            self._set_state(self.VERIFY_BEAR)

    # SEARCH

    def _tick_search(self):
        timeout = self.get_parameter('search_timeout').value
        if self.search_start and (self.get_clock().now() - self.search_start).nanoseconds > timeout * 1e9:
            self.get_logger().warn('Søk timeout — ingen teddybjørn funnet.')
            self._finish('FAILED_SEARCH_TIMEOUT')
            return

        if self._yolo_verified():
            self.get_logger().info('Bamse verifisert under SEARCH — går til VERIFY_BEAR.')
            self._stop()
            self._caching_path    = False
            self._last_arm_status = ''
            self._verify_t0       = self.get_clock().now()
            self._verify_future   = self._call_arm_async('search')
            self._set_state(self.VERIFY_BEAR)
            return

        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)

    # APPROACH (original, for backward compat)

    def _tick_approach(self):
        W         = self.get_parameter('image_width').value
        H         = self.get_parameter('image_height').value
        k_ang     = self.get_parameter('approach_k_ang').value
        fwd       = self.get_parameter('approach_fwd_speed').value
        threshold = self.get_parameter('bbox_area_threshold').value

        if not self.last_det_time or (self.get_clock().now() - self.last_det_time).nanoseconds > 1e9:
            self.get_logger().info('Mistet bjørn — søker på nytt.')
            self._set_state(self.SEARCH)
            self.search_start = self.get_clock().now()
            return

        det      = self.last_detection
        cx       = det.bbox.center.position.x
        area     = det.bbox.size_x * det.bbox.size_y
        area_frac = area / (W * H)

        if area_frac >= threshold:
            self.get_logger().info(f'Nær nok! Areal={area_frac:.2f}. Stopper.')
            self._finish('SUCCESS')
            return

        twist = Twist()
        twist.linear.x  = fwd
        twist.angular.z = -k_ang * (cx - W / 2.0)
        self.cmd_pub.publish(twist)

    # VERIFY_BEAR

    def _tick_verify_bear(self):
        self._stop()
        now     = self.get_clock().now()
        elapsed = (now - self._verify_t0).nanoseconds * 1e-9

        # Arm rapporterer DONE:2 når arm er i søk-posisjon
        if self._last_arm_status == 'DONE:2':
            self.get_logger().info('Arm i søk-posisjon (DONE:2) — starter VISUAL_SERVO.')
            self._arm_grabbed         = False
            self._servo_t0            = now
            self._last_yolo_t         = now
            self._lost_search_started = None
            self._set_state(self.VISUAL_SERVO)
            return

        if elapsed > 15.0:
            if self._verify_future is None:
                self.get_logger().error('/arm/search ikke tilgjengelig — avbryter.')
            else:
                self.get_logger().error('/arm/search timeout — ingen DONE:2 etter 15s.')
            self._finish('FAILED_VERIFY_TIMEOUT')

    # VISUAL_SERVO

    def _tick_visual_servo(self):
        now = self.get_clock().now()

        # Arm har grepet — returner hjem
        if self._arm_grabbed:
            self._stop()
            self._enter_return_home()
            return

        lost_timeout  = self.get_parameter('lost_timeout_s').value
        drive_stale_s = self.get_parameter('drive_stale_s').value

        # Sjekk ferskeste high-conf YOLO-deteksjon
        dx_norm    = None
        det_age_s  = float('inf')
        if self._yolo_recent:
            t, _score, bbox = self._yolo_recent[-1]
            det_age_s = (now - t).nanoseconds * 1e-9
            if det_age_s <= lost_timeout:
                W             = float(self.get_parameter('image_width').value)
                target_offset = self.get_parameter('target_cx_offset_norm').value
                cx_norm       = (bbox.center.position.x - W / 2.0) / (W / 2.0)
                dx_norm       = cx_norm - target_offset
                self._lost_search_started = None   # tilbakestill sveip

        # Deteksjon for gammel — stopp og vent på neste YOLO-frame
        if dx_norm is not None and det_age_s > drive_stale_s:
            self._publish_twist(0.0, 0.0)
            return

        # Tap av bjørn: sveip-søk ±30° → abort
        if dx_norm is None:
            if self._lost_search_started is None:
                self._lost_search_started = now
            sweep = (now - self._lost_search_started).nanoseconds * 1e-9
            if sweep < 2.0:
                self._publish_twist(0.0,  0.3)
            elif sweep < 4.0:
                self._publish_twist(0.0, -0.3)
            else:
                self.get_logger().error('Bjørn mistet — åpner arm og avbryter.')
                self._call_arm_async('open')
                self._finish('FAILED_LOST_BEAR')
            return

        # P-kontroller: sentrér bjørn med offset (fersk deteksjon ≤ drive_stale_s)
        k_p     = self.get_parameter('k_p_yaw').value
        max_ang = self.get_parameter('max_ang_servo').value
        max_lin = self.get_parameter('max_lin_servo').value
        tol     = self.get_parameter('heading_tolerance').value

        ang_z = clamp(-k_p * dx_norm, -max_ang, max_ang)
        lin_x = max_lin if abs(dx_norm) < tol else max_lin * 0.5
        self._publish_twist(lin_x, ang_z)

        # Grab timeout: gå til SEARCH og prøv igjen
        if (now - self._servo_t0).nanoseconds * 1e-9 > self.get_parameter('grab_timeout_s').value:
            self.get_logger().warn('GRABBED timeout — åpner arm, returnerer til SEARCH.')
            self._stop()
            self._call_arm_async('open')
            self.search_start = self.get_clock().now()
            self._set_state(self.SEARCH)

    # RETURN_HOME

    def _enter_return_home(self):
        self.get_logger().info('Starter retur hjem via path replay (rygging).')
        self._caching_path = False
        # Reversert path + start_pose som garantert siste punkt
        self._replay_queue = list(reversed(self._path_buffer))
        if self._start_pose is not None:
            self._replay_queue.append(self._start_pose)
        self._replay_idx = 0
        self._set_state(self.RETURN_HOME)

    def _tick_return_home(self):
        if self._replay_idx >= len(self._replay_queue):
            self.get_logger().info('Path replay fullført — fremme ved start.')
            self._stop()
            self._enter_final_release()
            return

        if self._latest_odom is None:
            return

        cur    = self._odom_to_tuple(self._latest_odom)
        target = self._replay_queue[self._replay_idx]

        dx   = target[0] - cur[0]
        dy   = target[1] - cur[1]
        dist = math.hypot(dx, dy)

        if dist < self.get_parameter('replay_xy_tol').value:
            self._replay_idx += 1
            return

        # Rygge: snu 180° slik at ryggen peker mot waypoint, kjør baklengs
        desired_heading = math.atan2(dy, dx) + math.pi
        yaw_err         = self._wrap_angle(desired_heading - cur[2])

        yaw_tol   = self.get_parameter('replay_yaw_tol').value
        lin_speed = self.get_parameter('replay_lin_speed').value

        lin_x = -lin_speed if abs(yaw_err) < yaw_tol else 0.0
        ang_z = clamp(1.5 * yaw_err, -0.5, 0.5)
        self._publish_twist(lin_x, ang_z)

    # FINAL_RELEASE

    def _enter_final_release(self):
        self.get_logger().info('FINAL_RELEASE: åpner arm og avslutter oppdraget.')
        self._release_t0     = self.get_clock().now()
        self._release_future = self._call_arm_async('open')
        self._set_state(self.FINAL_RELEASE)

    def _tick_final_release(self):
        self._stop()
        elapsed = (self.get_clock().now() - self._release_t0).nanoseconds * 1e-9

        if self._release_future is None:
            self.get_logger().warn('/arm/open ikke tilgjengelig — fullfører uansett.')
            self._finish('SUCCESS')
            return

        if self._release_future.done():
            self._finish('SUCCESS')
        elif elapsed > 10.0:
            self.get_logger().warn('/arm/open timeout — fullfører uansett.')
            self._finish('SUCCESS')

    # Navigation helpers

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
        self._caching_path = False
        self.track_pub.publish(Bool(data=True))
        self._set_state(self.DONE)
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f'Oppdrag avsluttet: {status}')
        self._idle_timer = self.create_timer(2.0, self._reset_to_idle)

    def _reset_to_idle(self):
        self._idle_timer.cancel()
        self._set_state(self.IDLE)

    # LiDAR

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _compute_front_distance(self):
        if self.last_scan is None:
            self.distance_m = float('inf')
            self.too_close  = False
            return
        scan     = self.last_scan
        fov      = math.radians(LIDAR_FOV_DEG)
        valid    = []
        below_min = 0
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            a = math.atan2(math.sin(angle), math.cos(angle))
            if -fov <= a <= fov and math.isfinite(r):
                if r < LIDAR_MIN_RANGE_M:
                    below_min += 1
                elif r <= LIDAR_MAX_RANGE_M:
                    valid.append(r)
        self.too_close  = below_min > 0
        self.distance_m = min(valid) if valid else float('inf')

    # Detection helpers

    def _pixel_error(self):
        if self.last_detection is None or self.last_det_time is None:
            return 0.0, False
        dt = (self.get_clock().now() - self.last_det_time).nanoseconds / 1e9
        if dt > DETECTION_TIMEOUT_SEC:
            return 0.0, False
        cx = self.last_detection.bbox.center.position.x
        return cx - self.image_width / 2.0, True

    def _publish_twist(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    # LOST_TARGET

    def _enter_lost_target(self):
        self.get_logger().warn('Mistet bjørn — LOST_TARGET')
        self._set_state(self.LOST_TARGET)
        self.lost_target_start = self.get_clock().now()
        self.missed_frames     = 0

    def _tick_lost_target(self):
        elapsed = (self.get_clock().now() - self.lost_target_start).nanoseconds / 1e9
        if elapsed > LOST_TARGET_TIMEOUT_SEC:
            self.get_logger().warn('LOST_TARGET timed out — avbryter oppdrag')
            self._publish_twist(0.0, 0.0)
            self._finish('LOST')
            return
        err, fresh = self._pixel_error()
        if fresh:
            self.get_logger().info('Bjørn gjenfunnet — tilbake til ALIGN_AND_APPROACH')
            self._set_state(self.ALIGN_AND_APPROACH)
            self.aa_phase      = 'ROTATE'
            self.missed_frames = 0
            return
        self._publish_twist(0.0, SEARCH_ANGULAR_VEL)

    # ALIGN_AND_APPROACH (beholdt, bypassed i ny flyt)

    def _aa_rotate(self):
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
        err, fresh = self._pixel_error()
        if not fresh:
            self.missed_frames += 1
            if self.missed_frames >= LOST_TARGET_FRAMES:
                self._enter_lost_target()
            self._publish_twist(0.0, 0.0)
            return
        self.missed_frames = 0
        d   = self.distance_m
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
        if not self._completion_logged:
            d_cm = self.distance_m * 100.0
            self.get_logger().info(
                f'ALIGN_AND_APPROACH COMPLETE: stoppet {d_cm:.1f}cm fra teddybjørn')
            self._completion_logged = True
            self._finish('SUCCESS')

    def _tick_align_and_approach(self):
        self._compute_front_distance()
        if   self.aa_phase == 'ROTATE':   self._aa_rotate()
        elif self.aa_phase == 'APPROACH': self._aa_approach()
        elif self.aa_phase == 'COMPLETE': self._aa_complete()

    # Helpers

    def _call_arm_async(self, name: str):
        """Non-blocking arm service call. Returns future or None if service not ready."""
        client = self._arm_clients[name]
        if not client.service_is_ready():
            self.get_logger().warning(f'/arm/{name} ikke klar — hopper over.')
            return None
        return client.call_async(Trigger.Request())

    def _yolo_verified(self) -> bool:
        """True iff 3+ class-77 detections with conf≥0.40 within the last 1.5s."""
        if len(self._yolo_recent) < self._yolo_verify_count:
            return False
        now    = self.get_clock().now()
        last_n = list(self._yolo_recent)[-self._yolo_verify_count:]
        return (now - last_n[0][0]).nanoseconds * 1e-9 < 1.5

    def _bear_recently_seen(self, window_s: float = 3.0) -> bool:
        """True hvis siste class-77 deteksjon (conf>=0.15) er nyere enn window_s."""
        if self.last_det_time is None:
            return False
        return (self.get_clock().now() - self.last_det_time).nanoseconds * 1e-9 < window_s

    def _yaw_from_quat(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _odom_to_tuple(self, odom) -> tuple:
        p = odom.pose.pose.position
        return (p.x, p.y, self._yaw_from_quat(odom.pose.pose.orientation))

    def _pose_dist(self, a: tuple, b: tuple) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def _wrap_angle(self, a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))


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
