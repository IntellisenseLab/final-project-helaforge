"""
SemanticNavigator – v6 (fixed)
==============================
Scan your environment manually with the arrow teleop while YOLO detects and
logs objects. Then navigate to any detected object by voice or typed command.

Works with BOTH:
  • Simulation  (Gazebo camera + sim odom)
  • Physical    (kinect_bridge + kobuki_driver)

Subscribes:
  /odom                  – robot position (from Gazebo bridge OR kobuki_driver)
  /scan                  – LiDAR for obstacle avoidance (optional)
  /camera/image_raw      – RGB for YOLO detection
  /camera/depth/image_raw– Depth for object distance estimation
  /semantic_nav/command  – Text commands (also readable from stdin CLI)

Publishes:
  /cmd_vel               – velocity commands to drive the robot
  /semantic_objects      – RViz MarkerArray of detected objects

Commands (type directly in terminal OR publish to /semantic_nav/command):
  scan        – start scanning; drive with arrow_teleop to map objects
  stop scan   – stop scanning, retrace path back to start
  list        – show all detected objects
  <object_id> – navigate to that object (e.g. "chair_1")
  go to <obj> – alternate syntax
"""

import math
import time
import os
import json
import sys
import re
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import tf2_ros
import numpy as np

# ── Optional: YOLO (ultralytics) ──────────────────────────────────────────────
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None
    print('[semantic_navigator] WARNING: ultralytics not installed. '
          'Run: pip3 install ultralytics --break-system-packages')

# ── Configuration ─────────────────────────────────────────────────────────────
YOLO_MODEL       = 'yolov8n.pt'   # lightweight model; auto-downloads on first run
YOLO_EVERY_N     = 5              # run YOLO every N camera frames
# Confidence threshold — set LOW (0.20) for Gazebo plain-box simulation.
# Plain colored boxes have no texture so YOLO scores are typically 15-30%.
# Raise to 0.45+ when running on a real camera with real objects.
CONF_THRESHOLD   = 0.20
STANDOFF         = 0.30           # stop 30 cm from target object (close approach)
WP_SPACING       = 0.60           # record a waypoint every 0.6 m
WP_TOL           = 0.25           # waypoint reached tolerance
MAX_LIN          = 0.3            # max linear speed  (m/s) – safe for Kobuki
MAX_ANG          = 1.2            # max angular speed (rad/s)
OBS_STOP_DIST    = 0.30           # stop if obstacle within 30 cm (from /scan)
KP_LIN, KI_LIN, KD_LIN = 1.2, 0.02, 0.1
KP_ANG, KI_ANG, KD_ANG = 3.0, 0.01, 0.2

MAP_FILE = os.path.expanduser('~/Documents/ROS_final_project/yolo_semantic_map.json')


# ── Helpers ───────────────────────────────────────────────────────────────────
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ── Main Node ─────────────────────────────────────────────────────────────────
class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')
        self.get_logger().info('SemanticNavigator initializing …')
        self.cb = ReentrantCallbackGroup()

        # State
        self.scanning    = False
        self.object_dict = {}
        self.frame_count = 0
        self.ox = self.oy = self.oyaw = 0.0
        self.waypoints   = []
        self.obs_front   = float('inf')
        self.latest_depth = None
        self._pid_reset()

        # Publishers
        self.cmd_pub    = self.create_publisher(Twist,      '/cmd_vel',          10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_objects', 10)

        # TF
        self.bridge      = CvBridge()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load persisted object map
        os.makedirs(os.path.dirname(MAP_FILE), exist_ok=True)
        self._load_map()

        # YOLO
        if YOLO:
            self.model = YOLO(YOLO_MODEL)
            self.get_logger().info('YOLO loaded ✓')
        else:
            self.model = None

        # Subscriptions
        self.create_subscription(Odometry,  '/odom',                   self._odom_cb,  10, callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan',                   self._scan_cb,  10, callback_group=self.cb)
        self.create_subscription(Image,     '/camera/image_raw',       self._img_cb,   10, callback_group=self.cb)
        self.create_subscription(Image,     '/camera/depth/image_raw', self._depth_cb, 10, callback_group=self.cb)
        self.create_subscription(String,    '/semantic_nav/command',   self._cmd_cb,   10, callback_group=self.cb)

        # Marker timer
        self.create_timer(1.0, self._publish_markers, callback_group=self.cb)

        self.get_logger().info('Ready. Commands: scan | stop scan | list | <object_id>')

    # ── PID ───────────────────────────────────────────────────────────────────
    def _pid_reset(self):
        self.lin_i = self.lin_prev = 0.0
        self.ang_i = self.ang_prev = 0.0

    def _pid_linear(self, e, dt):
        self.lin_i += e * dt
        d = (e - self.lin_prev) / (dt + 1e-6)
        self.lin_prev = e
        return KP_LIN * e + KI_LIN * self.lin_i + KD_LIN * d

    def _pid_angular(self, e, dt):
        self.ang_i += e * dt
        d = (e - self.ang_prev) / (dt + 1e-6)
        self.ang_prev = e
        return KP_ANG * e + KI_ANG * self.ang_i + KD_ANG * d

    # ── Map persistence ───────────────────────────────────────────────────────
    def _load_map(self):
        if os.path.exists(MAP_FILE):
            try:
                with open(MAP_FILE, 'r') as f:
                    self.object_dict = json.load(f)
                self.get_logger().info(f'Loaded {len(self.object_dict)} objects from map.')
            except Exception:
                pass

    def _save_map(self):
        try:
            with open(MAP_FILE, 'w') as f:
                json.dump(self.object_dict, f, indent=2)
            self.get_logger().info(f'Map saved ({len(self.object_dict)} objects).')
        except Exception as e:
            self.get_logger().error(f'Save failed: {e}')

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        # Prefer map→base_footprint TF if SLAM is running, else use raw odom
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.ox   = t.transform.translation.x
            self.oy   = t.transform.translation.y
            self.oyaw = yaw_from_quat(t.transform.rotation)
        except Exception:
            self.ox   = msg.pose.pose.position.x
            self.oy   = msg.pose.pose.position.y
            self.oyaw = yaw_from_quat(msg.pose.pose.orientation)

        # Record waypoints during scan
        if self.scanning:
            if (not self.waypoints or
                    math.hypot(self.ox - self.waypoints[-1][0],
                               self.oy - self.waypoints[-1][1]) > WP_SPACING):
                self.waypoints.append((self.ox, self.oy))

    def _scan_cb(self, msg):
        """Extract minimum front distance from LiDAR for obstacle avoidance."""
        front = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                if abs(norm_angle(angle)) < math.radians(20):
                    front.append(r)
        self.obs_front = min(front) if front else float('inf')

    def _depth_cb(self, msg):
        """Accept both 32FC1 (metres, Kinect) and 16UC1 (mm, Gazebo bridge)."""
        try:
            enc = msg.encoding
            if enc in ('32FC1', '32FC'):
                # Already float metres (Kinect / physical)
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            else:
                # 16UC1 = uint16 millimetres (Gazebo ros_gz_bridge)
                raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                depth_f = raw.astype(np.float32) / 1000.0  # mm → metres
                depth_f[raw == 0] = np.nan                 # 0 mm = no data
                self.latest_depth = depth_f
        except Exception as e:
            self.get_logger().error(f'Depth decode failed ({msg.encoding}): {e}')
            self.latest_depth = None

    def _img_cb(self, msg):
        """Run YOLO on camera frames and register detected objects in the map."""
        if not self.scanning or not self.model:
            return
        self.frame_count += 1
        if self.frame_count % YOLO_EVERY_N != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(img, verbose=False)[0]

        if not results.boxes:
            return

        for box in results.boxes:
            conf     = float(box.conf[0])
            cls_name = results.names[int(box.cls[0])]

            # Skip very low-confidence detections only
            if conf < CONF_THRESHOLD:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # ── FIX C: depth decode handles 16UC1 (Gazebo) and 32FC1 (Kinect)
            depth = -1.0
            if self.latest_depth is None:
                self.get_logger().warn(
                    'Depth image not received — check /camera/depth/image_raw')
                continue

            h, w = self.latest_depth.shape[:2]
            patch = self.latest_depth[
                max(0, cy - 20):min(h, cy + 20),
                max(0, cx - 20):min(w, cx + 20)
            ]
            valid = patch[np.isfinite(patch) & (patch > 0.1)]
            if len(valid) > 0:
                depth = float(np.median(valid))

            if not (0.5 < depth < 5.0):
                self.get_logger().info(
                    f'SKIP {cls_name}: depth={depth:.2f}m out of range 0.5-5.0m')
                continue

            # Project object position into world frame
            ang  = self.oyaw + ((320 - cx) / 320.0) * (1.089 / 2.0)
            mx   = self.ox + depth * math.cos(ang)
            my   = self.oy + depth * math.sin(ang)

            # Spatial deduplication: skip if similar object already registered nearby
            if any(math.hypot(mx - o['x'], my - o['y']) < 1.5
                   for o in self.object_dict.values()):
                continue

            # ── FIX D: per-class counter → chair_1, chair_2, bottle_1 (not chair_1, bed_2)
            class_count = sum(1 for k in self.object_dict if k.startswith(cls_name + '_'))
            new_id = f'{cls_name}_{class_count + 1}'
            while new_id in self.object_dict:           # handles edge cases
                class_count += 1
                new_id = f'{cls_name}_{class_count + 1}'

            bgr = img[cy, cx]
            self.object_dict[new_id] = {
                'x': mx, 'y': my,
                'r': float(bgr[2] / 255),
                'g': float(bgr[1] / 255),
                'b': float(bgr[0] / 255),
            }
            self.get_logger().info(
                f'NEW OBJECT: {new_id}  conf={conf:.2f}  dist={depth:.1f}m  '
                f'pos=({mx:.2f}, {my:.2f})')

    def _cmd_cb(self, msg):
        """Handle commands from /semantic_nav/command (or CLI stdin thread)."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'CMD: {cmd}')

        if cmd == 'scan':
            self.scanning = True
            self.object_dict.clear()
            self.waypoints.clear()
            self.get_logger().info('SCAN STARTED — drive around with arrow_teleop')

        elif cmd in ('stop scan', 'scan stop'):
            self.scanning = False
            self._save_map()
            # Show a clean list of everything found
            if self.object_dict:
                self.get_logger().info(
                    f'Scan complete. Found {len(self.object_dict)} object(s):')
                for i, (k, v) in enumerate(self.object_dict.items(), 1):
                    self.get_logger().info(
                        f'  {i}. {k:20s} at ({v["x"]:+.2f}, {v["y"]:+.2f})')
                self.get_logger().info(
                    'Type an object name above to navigate to it.')
            else:
                self.get_logger().info(
                    'Scan complete. No objects detected — drive closer and scan again.')

        elif cmd == 'list':
            if not self.object_dict:
                self.get_logger().info('No objects detected yet.')
            else:
                for i, (k, v) in enumerate(self.object_dict.items(), 1):
                    self.get_logger().info(
                        f'  {i}. {k:25s} → ({v["x"]:+.2f}, {v["y"]:+.2f})')

        elif cmd == 'return home':
            threading.Thread(target=self._retrace, daemon=True).start()

        else:
            # Case-insensitive key lookup so "Chair_1" works the same as "chair_1"
            matched_key = None
            for k in self.object_dict:
                if k.lower() == cmd:
                    matched_key = k
                    break

            # 'go to <object>' alternate syntax
            if matched_key is None and cmd.startswith('go to '):
                raw = cmd[6:].strip().replace(' ', '_')
                for k in self.object_dict:
                    if k.lower() == raw:
                        matched_key = k
                        break

            if matched_key:
                o = self.object_dict[matched_key]
                dist_to_obj = math.hypot(o['x'] - self.ox, o['y'] - self.oy)
                self.get_logger().info(
                    f'─── {matched_key} ───')
                self.get_logger().info(
                    f'  Stored position : ({o["x"]:+.2f}, {o["y"]:+.2f}) m')
                self.get_logger().info(
                    f'  Robot position  : ({self.ox:+.2f}, {self.oy:+.2f}) m')
                self.get_logger().info(
                    f'  Distance        : {dist_to_obj:.2f} m')
                if dist_to_obj < 0.15:
                    self.get_logger().warn(
                        f'  Object is at robot position — bad depth during scan.'
                        f' Rescan closer to the object.')
                    return
                self.get_logger().info(
                    f'  Navigating … (stop arrow_teleop first if robot is not moving)')
                # run in thread so odom keeps updating during navigation
                threading.Thread(
                    target=self._drive_near,
                    args=(o['x'], o['y']),
                    kwargs={'label': matched_key},
                    daemon=True
                ).start()
            else:
                self.get_logger().warn(f'Unknown command or object not found: "{cmd}"')
                self.get_logger().warn(f'Known objects: {list(self.object_dict.keys())}')

    # ── Navigation ────────────────────────────────────────────────────────────
    def _drive_to(self, tx, ty, tol=WP_TOL, stop_at_obs=None):
        self._pid_reset()
        t0 = time.time()
        i = 0
        while rclpy.ok() and (time.time() - t0) < 45.0:
            i += 1
            dx, dy = tx - self.ox, ty - self.oy
            dist   = math.hypot(dx, dy)
            if dist < tol:
                break

            if i % 5 == 0:
                self.get_logger().info(f'Nav: dist={dist:.2f}m, obs={self.obs_front:.2f}m')

            # ── FIX F: only check standoff when ALREADY close to target ──────
            if (stop_at_obs is not None
                    and dist <= stop_at_obs
                    and self.obs_front <= stop_at_obs):
                break

            yaw_err = norm_angle(math.atan2(dy, dx) - self.oyaw)
            lin = self._pid_linear(dist, 0.1)
            ang = self._pid_angular(yaw_err, 0.1)

            if abs(yaw_err) > 1.2:
                lin = 0.0                            # rotate first, then drive
            if self.obs_front < OBS_STOP_DIST:
                lin = 0.0                            # emergency obstacle stop

            cmd          = Twist()
            cmd.linear.x = max(0.0, min(MAX_LIN, lin))
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, ang))
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)

        self.cmd_pub.publish(Twist())               # stop

    def _drive_near(self, tx, ty, label=''):
        dist_to_target = math.hypot(tx - self.ox, ty - self.oy)
        self.get_logger().info(
            f'Navigating to {label} …  '
            f'target=({tx:.2f},{ty:.2f})  robot=({self.ox:.2f},{self.oy:.2f})  '
            f'dist={dist_to_target:.2f}m')
        if dist_to_target < 0.3:
            self.get_logger().warn(
                f'{label} is nearly at robot position — '
                f'bad depth during scan. Scan again closer to the object.')
        self._drive_to(tx, ty, tol=0.15, stop_at_obs=STANDOFF)
        self.get_logger().info(f'Arrived at {label}')

    def _retrace(self):
        """Drive back through recorded waypoints in reverse order."""
        if len(self.waypoints) < 2:
            self.get_logger().info('No waypoints to retrace.')
            return
        for wx, wy in reversed(self.waypoints):
            self._drive_to(wx, wy)
        self.get_logger().info('Home reached.')

    # ── RViz Markers ──────────────────────────────────────────────────────────
    def _publish_markers(self):
        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (oid, c) in enumerate(self.object_dict.items()):
            # Sphere
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns, m.id, m.type, m.action = 'objects', i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(c['x']), float(c['y']), 0.2
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = float(c.get('r', 0)), float(c.get('g', 1)), float(c.get('b', 0)), 0.8
            ma.markers.append(m)
            # Label
            t = Marker()
            t.header.frame_id = 'map'
            t.header.stamp    = now
            t.ns, t.id, t.type, t.action = 'labels', i + 100, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(c['x']), float(c['y']), 0.6
            t.scale.z = 0.25
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = oid
            ma.markers.append(t)
        if ma.markers:
            self.marker_pub.publish(ma)


# ── CLI stdin thread ───────────────────────────────────────────────────────────
# Strip ANSI/terminal escape sequences so garbage like ^[[?61;1;21;22c
# that some terminals inject into stdin doesn't corrupt typed commands.
_ANSI_ESC = re.compile(r'\x1b(?:\[[0-9;?]*[a-zA-Z]|\][^\x07]*\x07|[^\[\]])')

def _stdin_reader(pub):
    """Reads commands typed in the terminal and publishes to /semantic_nav/command."""
    print('\n=== Semantic Navigator CLI ===')
    print('Commands: scan | stop scan | list | <object_id>\n')
    for line in sys.stdin:
        clean = _ANSI_ESC.sub('', line)                          # strip escape seqs
        clean = ''.join(c for c in clean if c.isprintable() or c == ' ')
        cmd = clean.strip()
        if not cmd:
            continue
        msg      = String()
        msg.data = cmd
        pub.publish(msg)
        print(f'[CLI] Sent: "{cmd}"')


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()

    cli_pub = node.create_publisher(String, '/semantic_nav/command', 10)
    threading.Thread(target=_stdin_reader, args=(cli_pub,), daemon=True).start()

    ex = MultiThreadedExecutor()
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