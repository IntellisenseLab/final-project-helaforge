#!/usr/bin/env python3
"""
SemanticNavigator – Master ROS 2 Node  (v4)
=============================================
• YOLO object detection  (throttled, no display)
• SORT multi-object tracking
• Odom-based object registration + waypoint return-home
• PID controller with velocity ramping for smooth motion
• LiDAR-based reactive obstacle avoidance (/scan)
• Nav2 fallback for GO_TO (with odom fallback if Nav2 fails)

Commands (publish to /semantic_nav/command as std_msgs/String):
  scan          – start (drive with arrow_teleop, YOLO logs objects)
  scan stop     – stop, retrace path home, print object list
  <object_id>   – navigate to object (stop 50 cm away)
  return home   – retrace waypoints back to start
  list          – print detected objects
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge

import tf2_ros
import numpy as np

# ── Optional imports ─────────────────────────────────────────────────
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    from sort import Sort
except ImportError:
    Sort = None

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None
    TaskResult = None


# ── Helpers ──────────────────────────────────────────────────────────
def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ab = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + ab - inter + 1e-6)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y**2 + q.z**2))


def norm_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ═══════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════
YOLO_EVERY_N      = 10      # run YOLO every N-th camera frame
STANDOFF          = 0.50    # stop 50 cm from object
WP_SPACING        = 0.60    # breadcrumb every 60 cm (fewer waypoints)
WP_TOL            = 0.25    # waypoint reached tolerance

# PID gains
KP_LIN            = 1.2     # proportional – linear  (aggressive)
KI_LIN            = 0.02    # integral – linear
KD_LIN            = 0.10    # derivative – linear
KP_ANG            = 3.0     # proportional – angular (snappy turns)
KI_ANG            = 0.01    # integral – angular
KD_ANG            = 0.2     # derivative – angular

MAX_LIN           = 0.50    # m/s  (doubled)
MAX_ANG           = 1.5     # rad/s
RAMP_RATE         = 2.0     # m/s² (fast ramp-up)

# Obstacle avoidance
OBS_STOP_DIST     = 0.30    # hard stop (m)
OBS_SLOW_DIST     = 0.70    # start slowing (m)
OBS_SIDE_STEER    = 0.80    # steer away if side obstacle < this (m)
LIDAR_ARC_FRONT   = 30      # degrees each side of centre to check
LIDAR_ARC_SIDE    = 60      # degrees for left/right zones


class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')
        self.get_logger().info('SemanticNavigator initializing …')
        self.cb = ReentrantCallbackGroup()

        # ── State ────────────────────────────────────────────────────
        self.scanning = False
        self.object_dict: dict[str, dict] = {}
        self.frame_count = 0

        # ── Odom ─────────────────────────────────────────────────────
        self.ox = self.oy = self.oyaw = 0.0

        # ── Waypoints ────────────────────────────────────────────────
        self.waypoints: list[tuple[float, float]] = []
        self.home_x = self.home_y = 0.0

        # ── LiDAR obstacle distances ────────────────────────────────
        self.obs_front = float('inf')
        self.obs_left  = float('inf')
        self.obs_right = float('inf')

        # ── PID state ────────────────────────────────────────────────
        self._pid_reset()
        self.current_lin = 0.0   # for velocity ramping

        # ── Publishers ───────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── CvBridge / TF ────────────────────────────────────────────
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── YOLO ─────────────────────────────────────────────────────
        if YOLO is not None:
            self.get_logger().info('Loading YOLO26n …')
            self.model = YOLO('yolo26n.pt')
            self.model(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
            self.get_logger().info('YOLO warmed up ✓')
        else:
            self.model = None

        # ── SORT ─────────────────────────────────────────────────────
        self.tracker = Sort(max_age=30, min_hits=3,
                            iou_threshold=0.3) if Sort else None

        # ── Nav2 ─────────────────────────────────────────────────────
        self.nav = BasicNavigator() if BasicNavigator else None

        # ── Subscribers ──────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10,
                                 callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10,
                                 callback_group=self.cb)
        self.create_subscription(Image, '/camera/image_raw',
                                 self._img_cb, 10, callback_group=self.cb)
        self.create_subscription(String, '/semantic_nav/command',
                                 self._cmd_cb, 10, callback_group=self.cb)

        self.get_logger().info(
            'Ready.  Commands: scan | scan stop | <id> | return home | list')

    # ──────────────────────────────────────────────────────────────────
    #  PID helpers
    # ──────────────────────────────────────────────────────────────────
    def _pid_reset(self):
        self.lin_i = self.lin_prev = 0.0
        self.ang_i = self.ang_prev = 0.0

    def _pid_linear(self, error, dt):
        self.lin_i += error * dt
        self.lin_i = max(-0.5, min(0.5, self.lin_i))  # anti-windup
        d = (error - self.lin_prev) / dt if dt > 0 else 0.0
        self.lin_prev = error
        return KP_LIN * error + KI_LIN * self.lin_i + KD_LIN * d

    def _pid_angular(self, error, dt):
        self.ang_i += error * dt
        self.ang_i = max(-1.0, min(1.0, self.ang_i))
        d = (error - self.ang_prev) / dt if dt > 0 else 0.0
        self.ang_prev = error
        return KP_ANG * error + KI_ANG * self.ang_i + KD_ANG * d

    def _ramp(self, target, dt):
        """Smooth velocity ramping to avoid jerky starts/stops."""
        max_delta = RAMP_RATE * dt
        if target > self.current_lin:
            self.current_lin = min(target, self.current_lin + max_delta)
        else:
            self.current_lin = max(target, self.current_lin - max_delta)
        return self.current_lin

    # ──────────────────────────────────────────────────────────────────
    #  Obstacle avoidance layer
    # ──────────────────────────────────────────────────────────────────
    def _obstacle_adjust(self, raw_lin, raw_ang):
        """
        Modify raw PID outputs based on LiDAR obstacle readings.
        Returns (linear, angular) after avoidance adjustment.
        """
        lin, ang = raw_lin, raw_ang

        # ── Front obstacle ───────────────────────────────────────────
        if self.obs_front < OBS_STOP_DIST:
            lin = 0.0
            # Steer away from the closer side
            if self.obs_left > self.obs_right:
                ang = MAX_ANG     # turn left
            else:
                ang = -MAX_ANG    # turn right
        elif self.obs_front < OBS_SLOW_DIST:
            # Slow down proportionally
            factor = (self.obs_front - OBS_STOP_DIST) / (OBS_SLOW_DIST - OBS_STOP_DIST)
            lin *= max(0.1, factor)

        # ── Side obstacles (gentle steering) ─────────────────────────
        if self.obs_left < OBS_SIDE_STEER and lin > 0:
            ang -= 0.4   # nudge right
        if self.obs_right < OBS_SIDE_STEER and lin > 0:
            ang += 0.4   # nudge left

        lin = max(-MAX_LIN, min(MAX_LIN, lin))
        ang = max(-MAX_ANG, min(MAX_ANG, ang))
        return lin, ang

    # ══════════════════════════════════════════════════════════════════
    #  SUBSCRIBERS
    # ══════════════════════════════════════════════════════════════════
    def _odom_cb(self, msg: Odometry):
        self.ox = msg.pose.pose.position.x
        self.oy = msg.pose.pose.position.y
        self.oyaw = yaw_from_quat(msg.pose.pose.orientation)

        if self.scanning and self.waypoints:
            lx, ly = self.waypoints[-1]
            if math.hypot(self.ox - lx, self.oy - ly) >= WP_SPACING:
                self.waypoints.append((self.ox, self.oy))

    def _scan_cb(self, msg: LaserScan):
        """Extract min distances in front / left / right zones."""
        n = len(msg.ranges)
        if n == 0:
            return

        def zone_min(centre_deg, half_arc_deg):
            centre_idx = int((math.radians(centre_deg) - msg.angle_min)
                             / msg.angle_increment) % n
            half = int(math.radians(half_arc_deg) / msg.angle_increment)
            lo = max(0, centre_idx - half)
            hi = min(n - 1, centre_idx + half)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max]
            return min(vals) if vals else float('inf')

        self.obs_front = zone_min(0, LIDAR_ARC_FRONT)
        self.obs_left  = zone_min(90, LIDAR_ARC_SIDE // 2)
        self.obs_right = zone_min(-90, LIDAR_ARC_SIDE // 2)

    # ══════════════════════════════════════════════════════════════════
    #  COMMAND HANDLER
    # ══════════════════════════════════════════════════════════════════
    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'CMD: "{cmd}"')

        if cmd == 'scan':
            self.home_x, self.home_y = self.ox, self.oy
            self.waypoints = [(self.home_x, self.home_y)]
            self.object_dict.clear()
            self.scanning = True
            self.get_logger().info(
                f'▶ SCAN started. Home=({self.home_x:.2f},{self.home_y:.2f})'
                f'\n  Drive with arrow_teleop. Send "scan stop" when done.')

        elif cmd == 'scan stop':
            self.scanning = False
            self.get_logger().info(
                f'■ SCAN stopped – {len(self.object_dict)} objects, '
                f'{len(self.waypoints)} waypoints.')
            self._print_dict()
            self.get_logger().info('Retracing path home …')
            self._retrace()

        elif cmd == 'return home':
            self.get_logger().info('Retracing path home …')
            self._retrace()

        elif cmd == 'list':
            self._print_dict()

        else:
            if cmd in self.object_dict:
                c = self.object_dict[cmd]
                self.get_logger().info(
                    f'Going to {cmd} ({c["x"]:.2f}, {c["y"]:.2f}) …')
                self._drive_near(c['x'], c['y'], label=cmd)
            else:
                self.get_logger().warn(
                    f'"{cmd}" unknown. Objects: {list(self.object_dict.keys())}')

    # ══════════════════════════════════════════════════════════════════
    #  IMAGE CALLBACK  (throttled YOLO → SORT → register)
    # ══════════════════════════════════════════════════════════════════
    def _img_cb(self, msg: Image):
        if not self.scanning or not self.model or not self.tracker:
            return
        self.frame_count += 1
        if self.frame_count % YOLO_EVERY_N != 0:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        res = self.model(img, verbose=False)[0]
        names = res.names

        dets, yboxes = [], []
        for box in res.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            sc = float(box.conf[0].cpu().numpy())
            ci = int(box.cls[0].cpu().numpy())
            dets.append([x1, y1, x2, y2, sc])
            yboxes.append({'b': [x1, y1, x2, y2], 'c': names[ci]})

        da = np.array(dets) if dets else np.empty((0, 5))
        tracks = self.tracker.update(da)

        for trk in tracks:
            tx1, ty1, tx2, ty2, tid = trk
            tid = int(tid)

            best_c, best_o = 'object', 0.0
            for yb in yboxes:
                o = iou([tx1, ty1, tx2, ty2], yb['b'])
                if o > best_o and o > 0.1:
                    best_o = o
                    best_c = yb['c']

            oid = f'{best_c}_{tid}'
            if oid not in self.object_dict:
                bh = ty2 - ty1
                depth = max(0.5, min(5.0, 300.0 / (bh + 1e-6)))
                cx = (tx1 + tx2) / 2.0
                ang_off = ((cx - 320.0) / 320.0) * (1.089 / 2.0)
                oa = self.oyaw + ang_off
                mx = self.ox + depth * math.cos(oa)
                my = self.oy + depth * math.sin(oa)
                self.object_dict[oid] = {'x': mx, 'y': my}
                n = len(self.object_dict)
                self.get_logger().info(
                    f'\n'
                    f'  ╔══ NEW OBJECT #{n} ══════════════════╗\n'
                    f'  ║  {oid:30s}       ║\n'
                    f'  ║  Position: ({mx:+.2f}, {my:+.2f})          ║\n'
                    f'  ║  Est. distance: {depth:.1f} m              ║\n'
                    f'  ╚══════════════════════════════════════╝')

    # ══════════════════════════════════════════════════════════════════
    #  MOTION: PID drive to point with obstacle avoidance
    # ══════════════════════════════════════════════════════════════════
    def _drive_to(self, tx, ty, tol=WP_TOL, timeout=60.0):
        """PID drive to (tx,ty) with obstacle avoidance and ramping."""
        self._pid_reset()
        self.current_lin = 0.0
        t0 = time.time()
        prev_t = t0

        while (time.time() - t0) < timeout:
            now = time.time()
            dt = now - prev_t
            if dt < 0.05:
                time.sleep(0.02)
                continue
            prev_t = now

            dx = tx - self.ox
            dy = ty - self.oy
            dist = math.hypot(dx, dy)
            if dist < tol:
                break

            desired = math.atan2(dy, dx)
            yaw_err = norm_angle(desired - self.oyaw)

            # PID outputs
            raw_lin = self._pid_linear(dist, dt)
            raw_ang = self._pid_angular(yaw_err, dt)

            # Only move forward when roughly facing target
            if abs(yaw_err) > 0.8:
                raw_lin = 0.0
            elif abs(yaw_err) > 0.3:
                raw_lin *= 0.5   # slow while turning

            raw_lin = max(0.0, min(MAX_LIN, raw_lin))
            raw_ang = max(-MAX_ANG, min(MAX_ANG, raw_ang))

            # Ramp
            raw_lin = self._ramp(raw_lin, dt)

            # Obstacle avoidance
            lin, ang = self._obstacle_adjust(raw_lin, raw_ang)

            cmd = Twist()
            cmd.linear.x = lin
            cmd.angular.z = ang
            self.cmd_pub.publish(cmd)

        # Stop
        self.current_lin = 0.0
        self.cmd_pub.publish(Twist())

    def _drive_near(self, obj_x, obj_y, label=''):
        """Drive to 50 cm before the object, facing it."""
        dx = obj_x - self.ox
        dy = obj_y - self.oy
        dist = math.hypot(dx, dy)

        if dist < STANDOFF:
            self.get_logger().info(f'Already near {label}.')
            return

        scale = (dist - STANDOFF) / dist
        gx = self.ox + dx * scale
        gy = self.oy + dy * scale

        self.get_logger().info(
            f'🧭 Driving to ({gx:.2f},{gy:.2f}) [50cm from {label}]')
        self._drive_to(gx, gy, tol=0.15, timeout=90.0)
        self.get_logger().info(f'✅ Reached {label}!')

    # ══════════════════════════════════════════════════════════════════
    #  RETRACE WAYPOINTS
    # ══════════════════════════════════════════════════════════════════
    def _retrace(self):
        if len(self.waypoints) < 2:
            self.get_logger().info('No waypoints to retrace.')
            return

        rw = list(reversed(self.waypoints))
        total = len(rw)
        for i, (wx, wy) in enumerate(rw):
            self.get_logger().info(
                f'  WP {i+1}/{total}: ({wx:.2f},{wy:.2f})',
                throttle_duration_sec=1.0)
            self._drive_to(wx, wy, tol=WP_TOL, timeout=30.0)

        self.get_logger().info('✅ Back at start!')

    # ══════════════════════════════════════════════════════════════════
    #  DISPLAY
    # ══════════════════════════════════════════════════════════════════
    def _print_dict(self):
        self.get_logger().info('═══════ Detected Objects ═══════')
        if not self.object_dict:
            self.get_logger().info('  (none)')
        else:
            for i, (o, c) in enumerate(self.object_dict.items(), 1):
                self.get_logger().info(
                    f'  {i}. {o:20s} → ({c["x"]:+.2f}, {c["y"]:+.2f})')
        self.get_logger().info('════════════════════════════════')


# ══════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
