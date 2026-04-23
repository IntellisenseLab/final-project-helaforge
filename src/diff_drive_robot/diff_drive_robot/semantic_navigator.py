"""
SemanticNavigator – v5
======================
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
STANDOFF         = 1.00           # stop 1 m from target object
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
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def _img_cb(self, msg):
        """Run YOLO on camera frames and register detected objects in the map."""
        if not self.scanning or not self.model:
            return
        self.frame_count += 1
        if self.frame_count % YOLO_EVERY_N != 0:
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(img, verbose=False)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cls_name = results.names[int(box.cls[0])]

            # Estimate depth from Kinect depth image (median patch)
            depth = -1.0
            if self.latest_depth is not None:
                patch = self.latest_depth[max(0, cy-5):cy+5, max(0, cx-5):cx+5]
                valid = patch[np.isfinite(patch) & (patch > 0.3)]
                if len(valid) > 0:
                    depth = float(np.median(valid))

            # Only register objects within 0.5–5.0 m
            if not (0.5 < depth < 5.0):
                continue

            # Project object position into world frame
            ang  = self.oyaw + ((320 - cx) / 320.0) * (1.089 / 2.0)
            mx   = self.ox + depth * math.cos(ang)
            my   = self.oy + depth * math.sin(ang)

            # Spatial deduplication: skip if similar object already registered nearby
            if any(math.hypot(mx - o['x'], my - o['y']) < 1.5
                   for o in self.object_dict.values()):
                continue

            # Get dominant colour for RViz marker
            bgr    = img[cy, cx]
            new_id = f'{cls_name}_{len(self.object_dict) + 1}'
            self.object_dict[new_id] = {
                'x': mx, 'y': my,
                'r': float(bgr[2] / 255),
                'g': float(bgr[1] / 255),
                'b': float(bgr[0] / 255),
            }
            self.get_logger().info(
                f'NEW OBJECT: {new_id}  dist={depth:.1f}m  '
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
            self.get_logger().info('SCAN STOPPED — retracing home …')
            self._retrace()

        elif cmd == 'list':
            if not self.object_dict:
                self.get_logger().info('No objects detected yet.')
            else:
                for i, (k, v) in enumerate(self.object_dict.items(), 1):
                    self.get_logger().info(
                        f'  {i}. {k:25s} → ({v["x"]:+.2f}, {v["y"]:+.2f})')

        elif cmd == 'return home':
            self._retrace()

        elif cmd in self.object_dict:
            o = self.object_dict[cmd]
            self._drive_near(o['x'], o['y'], label=cmd)

        elif cmd.startswith('go to '):
            obj = cmd[6:].strip().replace(' ', '_')
            if obj in self.object_dict:
                o = self.object_dict[obj]
                self._drive_near(o['x'], o['y'], label=obj)
            else:
                self.get_logger().warn(f'Object not found: {obj}')

        else:
            self.get_logger().warn(f'Unknown command: "{cmd}"')

    # ── Navigation ────────────────────────────────────────────────────────────
    def _drive_to(self, tx, ty, tol=WP_TOL, stop_at_obs=None):
        self._pid_reset()
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 45.0:
            dx, dy = tx - self.ox, ty - self.oy
            dist   = math.hypot(dx, dy)
            if dist < tol:
                break
            if stop_at_obs is not None and self.obs_front <= stop_at_obs:
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
        self.get_logger().info(f'Navigating to {label} …')
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


# ── CLI stdin thread ──────────────────────────────────────────────────────────
def _stdin_reader(pub):
    """Reads commands typed in the terminal and publishes to /semantic_nav/command."""
    print('\n=== Semantic Navigator CLI ===')
    print('Commands: scan | stop scan | list | <object_id>\n')
    for line in sys.stdin:
        cmd = line.strip()
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