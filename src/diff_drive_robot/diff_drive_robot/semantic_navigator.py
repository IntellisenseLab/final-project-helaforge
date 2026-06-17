# """
# SemanticNavigator – Master ROS 2 Node  (v4)
# =============================================
# • YOLO object detection  (throttled, no display)
# • SORT multi-object tracking
# • Odom-based object registration + waypoint return-home
# • PID controller with velocity ramping for smooth motion
# • LiDAR-based reactive obstacle avoidance (/scan)
# • Nav2 fallback for GO_TO (with odom fallback if Nav2 fails)

# Commands (publish to /semantic_nav/command as std_msgs/String):
#   scan          – start (drive with arrow_teleop, YOLO logs objects)
#   scan stop     – stop, retrace path home, print object list
#   <object_id>   – navigate to object (stop 50 cm away)
#   return home   – retrace waypoints back to start
#   list          – print detected objects
# """

# import math
# import time
# import os
# import json
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import ReentrantCallbackGroup

# from std_msgs.msg import String
# from sensor_msgs.msg import Image, LaserScan
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, PoseStamped
# from visualization_msgs.msg import Marker, MarkerArray
# from cv_bridge import CvBridge

# import tf2_ros
# import numpy as np

# # ── Optional imports ─────────────────────────────────────────────────
# try:
#     from ultralytics import YOLO
# except ImportError:
#     YOLO = None

# try:
#     from diff_drive_robot.sort import Sort
# except ImportError:
#     Sort = None

# try:
#     from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# except ImportError:
#     BasicNavigator = None
#     TaskResult = None


# # ── Helpers ──────────────────────────────────────────────────────────
# def iou(a, b):
#     x1, y1 = max(a[0], b[0]), max(a[1], b[1])
#     x2, y2 = min(a[2], b[2]), min(a[3], b[3])
#     inter = max(0, x2 - x1) * max(0, y2 - y1)
#     aa = (a[2] - a[0]) * (a[3] - a[1])
#     ab = (b[2] - b[0]) * (b[3] - b[1])
#     return inter / (aa + ab - inter + 1e-6)


# def yaw_from_quat(q):
#     return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
#                       1.0 - 2.0 * (q.y**2 + q.z**2))


# def norm_angle(a):
#     while a > math.pi:  a -= 2 * math.pi
#     while a < -math.pi: a += 2 * math.pi
#     return a


# # ═══════════════════════════════════════════════════════════════
# #  CONFIGURATION
# # ═══════════════════════════════════════════════════════════════
# YOLO_EVERY_N      = 10
# STANDOFF          = 0.50
# WP_SPACING        = 0.60
# WP_TOL            = 0.25

# KP_LIN            = 1.2
# KI_LIN            = 0.02
# KD_LIN            = 0.10
# KP_ANG            = 3.0
# KI_ANG            = 0.01
# KD_ANG            = 0.2

# MAX_LIN           = 0.50
# MAX_ANG           = 1.5
# RAMP_RATE         = 2.0

# OBS_STOP_DIST     = 0.30
# OBS_SLOW_DIST     = 0.70
# OBS_SIDE_STEER    = 0.80
# LIDAR_ARC_FRONT   = 30
# LIDAR_ARC_SIDE    = 60


# class SemanticNavigator(Node):

#     def __init__(self):
#         super().__init__('semantic_navigator')
#         self.get_logger().info('SemanticNavigator initializing …')
#         self.cb = ReentrantCallbackGroup()

#         self.scanning = False
#         self.object_dict: dict[str, dict] = {}
#         self.frame_count = 0

#         self.ox = self.oy = self.oyaw = 0.0

#         self.waypoints: list[tuple[float, float]] = []
#         self.home_x = self.home_y = 0.0

#         self.obs_front = float('inf')
#         self.obs_left  = float('inf')
#         self.obs_right = float('inf')

#         self._pid_reset()
#         self.current_lin = 0.0

#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.bridge = CvBridge()
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
#         self.marker_pub = self.create_publisher(MarkerArray, '/semantic_objects', 10)

#         self.map_file = os.path.expanduser('~/Documents/ROS_final_project/yolo_semantic_map.json')
#         self._load_semantic_map()

#         self.create_timer(1.0, self._publish_markers_cb, callback_group=self.cb)

#         if YOLO is not None:
#             self.get_logger().info('Loading YOLO26n …')
#             self.model = YOLO('yolo26n.pt')
#             self.model(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
#             self.get_logger().info('YOLO warmed up ✓')
#         else:
#             self.model = None

#         self.tracker = Sort(max_age=30, min_hits=3,
#                             iou_threshold=0.3) if Sort else None

#         self.nav = BasicNavigator() if BasicNavigator else None

#         self.create_subscription(Odometry, '/odom', self._odom_cb, 10,
#                                  callback_group=self.cb)
#         self.create_subscription(LaserScan, '/scan', self._scan_cb, 10,
#                                  callback_group=self.cb)
#         self.create_subscription(Image, '/camera/image_raw',
#                                  self._img_cb, 10, callback_group=self.cb)
#         self.latest_depth = None
#         self.create_subscription(Image, '/camera/depth/image_raw',
#                                  self._depth_cb, 10, callback_group=self.cb)
#         self.create_subscription(String, '/semantic_nav/command',
#                                  self._cmd_cb, 10, callback_group=self.cb)

#         self.get_logger().info(
#             'Ready.  Commands: scan | scan stop | <id> | return home | list')

#     def _save_semantic_map(self):
#         try:
#             with open(self.map_file, 'w') as f:
#                 json.dump(self.object_dict, f, indent=2)
#             self.get_logger().info(f'Saved semantic map to {self.map_file}')
#         except Exception as e:
#             self.get_logger().error(f'Failed to save semantic map: {e}')

#     def _load_semantic_map(self):
#         if os.path.exists(self.map_file):
#             try:
#                 with open(self.map_file, 'r') as f:
#                     self.object_dict = json.load(f)
#                 self.get_logger().info(f'Loaded {len(self.object_dict)} objects from semantic map.')
#             except Exception as e:
#                 self.get_logger().error(f'Failed to load semantic map: {e}')

#     def _publish_markers_cb(self):
#         ma = MarkerArray()
#         idx = 0
#         now = self.get_clock().now().to_msg()
#         for obj_id, coords in self.object_dict.items():
#             # Sphere Marker
#             m = Marker()
#             m.header.stamp = now
#             m.header.frame_id = 'map'
#             m.ns = 'objects'
#             m.id = idx
#             m.type = Marker.SPHERE
#             m.action = Marker.ADD
#             m.pose.position.x = float(coords['x'])
#             m.pose.position.y = float(coords['y'])
#             m.pose.position.z = 0.25
#             m.scale.x = 0.3
#             m.scale.y = 0.3
#             m.scale.z = 0.3
#             m.color.r = float(coords.get('r', 0.0))
#             m.color.g = float(coords.get('g', 1.0))
#             m.color.b = float(coords.get('b', 0.0))
#             m.color.a = 0.8
#             ma.markers.append(m)

#             # Text Marker
#             t = Marker()
#             t.header.stamp = now
#             t.header.frame_id = 'map'
#             t.ns = 'labels'
#             t.id = idx + 1000
#             t.type = Marker.TEXT_VIEW_FACING
#             t.action = Marker.ADD
#             t.pose.position.x = float(coords['x'])
#             t.pose.position.y = float(coords['y'])
#             t.pose.position.z = 0.6
#             t.scale.z = 0.2
#             t.color.r = 1.0
#             t.color.g = 1.0
#             t.color.b = 1.0
#             t.color.a = 1.0
#             t.text = obj_id
#             ma.markers.append(t)
            
#             idx += 1
            
#         if ma.markers:
#             self.marker_pub.publish(ma)

#     # ── PID ───────────────────────────────────────────────────────────
#     def _pid_reset(self):
#         self.lin_i = self.lin_prev = 0.0
#         self.ang_i = self.ang_prev = 0.0

#     def _pid_linear(self, error, dt):
#         self.lin_i += error * dt
#         self.lin_i = max(-0.5, min(0.5, self.lin_i))
#         d = (error - self.lin_prev) / dt if dt > 0 else 0.0
#         self.lin_prev = error
#         return KP_LIN * error + KI_LIN * self.lin_i + KD_LIN * d

#     def _pid_angular(self, error, dt):
#         self.ang_i += error * dt
#         self.ang_i = max(-1.0, min(1.0, self.ang_i))
#         d = (error - self.ang_prev) / dt if dt > 0 else 0.0
#         self.ang_prev = error
#         return KP_ANG * error + KI_ANG * self.ang_i + KD_ANG * d

#     def _ramp(self, target, dt):
#         max_delta = RAMP_RATE * dt
#         if target > self.current_lin:
#             self.current_lin = min(target, self.current_lin + max_delta)
#         else:
#             self.current_lin = max(target, self.current_lin - max_delta)
#         return self.current_lin

#     # ── Obstacle avoidance ────────────────────────────────────────────
#     def _obstacle_adjust(self, raw_lin, raw_ang):
#         lin, ang = raw_lin, raw_ang
#         if self.obs_front < OBS_STOP_DIST:
#             lin = 0.0
#             if self.obs_left > self.obs_right:
#                 ang = MAX_ANG
#             else:
#                 ang = -MAX_ANG
#         elif self.obs_front < OBS_SLOW_DIST:
#             factor = (self.obs_front - OBS_STOP_DIST) / (OBS_SLOW_DIST - OBS_STOP_DIST)
#             lin *= max(0.1, factor)
#         if self.obs_left < OBS_SIDE_STEER and lin > 0:
#             ang -= 0.4
#         if self.obs_right < OBS_SIDE_STEER and lin > 0:
#             ang += 0.4
#         lin = max(-MAX_LIN, min(MAX_LIN, lin))
#         ang = max(-MAX_ANG, min(MAX_ANG, ang))
#         return lin, ang

#     # ── Subscribers ───────────────────────────────────────────────────
#     def _depth_cb(self, msg: Image):
#         try:
#             self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
#         except Exception:
#             pass

#     def _odom_cb(self, msg: Odometry):
#         try:
#             # Attempt to get SLAM-corrected position
#             now = rclpy.time.Time()
#             t = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
#             self.ox = t.transform.translation.x
#             self.oy = t.transform.translation.y
#             self.oyaw = yaw_from_quat(t.transform.rotation)
#         except Exception:
#             # Fallback to pure odometry if SLAM isn't running
#             self.ox = msg.pose.pose.position.x
#             self.oy = msg.pose.pose.position.y
#             self.oyaw = yaw_from_quat(msg.pose.pose.orientation)

#         if self.scanning and self.waypoints:
#             lx, ly = self.waypoints[-1]
#             if math.hypot(self.ox - lx, self.oy - ly) >= WP_SPACING:
#                 self.waypoints.append((self.ox, self.oy))

#     def _scan_cb(self, msg: LaserScan):
#         n = len(msg.ranges)
#         if n == 0:
#             return
#         def zone_min(centre_deg, half_arc_deg):
#             centre_idx = int((math.radians(centre_deg) - msg.angle_min)
#                              / msg.angle_increment) % n
#             half = int(math.radians(half_arc_deg) / msg.angle_increment)
#             lo = max(0, centre_idx - half)
#             hi = min(n - 1, centre_idx + half)
#             vals = [r for r in msg.ranges[lo:hi+1]
#                     if msg.range_min < r < msg.range_max]
#             return min(vals) if vals else float('inf')
#         self.obs_front = zone_min(0, LIDAR_ARC_FRONT)
#         self.obs_left  = zone_min(90, LIDAR_ARC_SIDE // 2)
#         self.obs_right = zone_min(-90, LIDAR_ARC_SIDE // 2)

#     # ── Command handler ──────────────────────────────────────────────
#     def _cmd_cb(self, msg: String):
#         cmd = msg.data.strip().lower()
#         self.get_logger().info(f'CMD: "{cmd}"')

#         if cmd == 'scan':
#             self.home_x, self.home_y = self.ox, self.oy
#             self.waypoints = [(self.home_x, self.home_y)]
#             self.object_dict.clear()
#             self.scanning = True
#             self.get_logger().info(
#                 f'▶ SCAN started. Home=({self.home_x:.2f},{self.home_y:.2f})'
#                 f'\n  Drive with arrow_teleop. Send "scan stop" when done.')

#         elif cmd == 'scan stop':
#             self.scanning = False
#             self._save_semantic_map()
#             self.get_logger().info(
#                 f'■ SCAN stopped – {len(self.object_dict)} objects, '
#                 f'{len(self.waypoints)} waypoints.')
#             self._print_dict()
#             self.get_logger().info('Retracing path home …')
#             self._retrace()

#         elif cmd == 'return home':
#             self.get_logger().info('Retracing path home …')
#             self._retrace()

#         elif cmd == 'list':
#             self._print_dict()

#         else:
#             if cmd in self.object_dict:
#                 c = self.object_dict[cmd]
#                 self.get_logger().info(
#                     f'Going to {cmd} ({c["x"]:.2f}, {c["y"]:.2f}) …')
#                 self._drive_near(c['x'], c['y'], label=cmd)
#             else:
#                 self.get_logger().warn(
#                     f'"{cmd}" unknown. Objects: {list(self.object_dict.keys())}')

#     # ── Image callback ────────────────────────────────────────────────
#     def _img_cb(self, msg: Image):
#         if not self.scanning or not self.model or not self.tracker:
#             return
#         self.frame_count += 1
#         if self.frame_count % YOLO_EVERY_N != 0:
#             return

#         try:
#             img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         except Exception:
#             return

#         res = self.model(img, verbose=False)[0]
#         names = res.names

#         dets, yboxes = [], []
#         for box in res.boxes:
#             x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
#             sc = float(box.conf[0].cpu().numpy())
#             ci = int(box.cls[0].cpu().numpy())
#             dets.append([x1, y1, x2, y2, sc])
#             yboxes.append({'b': [x1, y1, x2, y2], 'c': names[ci]})

#         da = np.array(dets) if dets else np.empty((0, 5))
#         tracks = self.tracker.update(da)

#         for trk in tracks:
#             tx1, ty1, tx2, ty2, tid = trk
#             tid = int(tid)

#             best_c, best_o = 'object', 0.0
#             for yb in yboxes:
#                 o = iou([tx1, ty1, tx2, ty2], yb['b'])
#                 if o > best_o and o > 0.1:
#                     best_o = o
#                     best_c = yb['c']

#             cx = int((tx1 + tx2) / 2.0)
#             cy = int((ty1 + ty2) / 2.0)
            
#             # 1. Exact depth from camera using Median Patch
#             depth = -1.0
#             p = 5 
#             if self.latest_depth is not None:
#                 try:
#                     h, w = self.latest_depth.shape
#                     y_min, y_max = max(0, cy-p), min(h, cy+p)
#                     x_min, x_max = max(0, cx-p), min(w, cx+p)
#                     patch = self.latest_depth[y_min:y_max, x_min:x_max]
#                     valid_depths = patch[np.isfinite(patch) & (patch > 0.1)]
#                     if len(valid_depths) > 0:
#                         depth = float(np.median(valid_depths))
#                 except Exception:
#                     pass
            
#             # 2. Fallback heuristic
#             if depth < 0.0:
#                 bh = ty2 - ty1
#                 depth = max(0.5, min(5.0, 300.0 / (bh + 1e-6)))

#             # 3. Get RGB Color for RViz Markers
#             r, g, b = 0.0, 1.0, 0.0
#             try:
#                 bgr = img[cy, cx]
#                 r, g, b = float(bgr[2])/255.0, float(bgr[1])/255.0, float(bgr[0])/255.0
#             except Exception:
#                 pass

#             # Filter: only identify objects that are 0.5 to 3.0 meters away
#             if depth < 0.5 or depth > 3.0:
#                 continue

#             ang_off = ((cx - 320.0) / 320.0) * (1.089 / 2.0)
#             oa = self.oyaw + ang_off
#             mx = self.ox + depth * math.cos(oa)
#             my = self.oy + depth * math.sin(oa)

#             # Spatial deduplication (1.0 meter threshold)
#             threshold = 1.0
#             matched_oid = None
#             for existing_id, existing_coords in self.object_dict.items():
#                 if existing_id.startswith(best_c):
#                     dist = math.hypot(existing_coords['x'] - mx, existing_coords['y'] - my)
#                     if dist < threshold:
#                         matched_oid = existing_id
#                         break
            
#             if matched_oid:
#                 # Update existing using moving average to refine position
#                 self.object_dict[matched_oid]['x'] = (self.object_dict[matched_oid]['x'] * 0.9) + (mx * 0.1)
#                 self.object_dict[matched_oid]['y'] = (self.object_dict[matched_oid]['y'] * 0.9) + (my * 0.1)
#             else:
#                 # Find next available ID for this class
#                 count = 1
#                 while f'{best_c}_{count}' in self.object_dict:
#                     count += 1
#                 new_oid = f'{best_c}_{count}'
#                 self.object_dict[new_oid] = {'x': mx, 'y': my, 'r': r, 'g': g, 'b': b}
#                 n = len(self.object_dict)
#                 self.get_logger().info(
#                     f'\n'
#                     f'  ╔══ NEW OBJECT #{n} ══════════════════╗\n'
#                     f'  ║  {new_oid:30s}       ║\n'
#                     f'  ║  Position: ({mx:+.2f}, {my:+.2f})          ║\n'
#                     f'  ║  Est. distance: {depth:.1f} m              ║\n'
#                     f'  ╚══════════════════════════════════════╝')

#     # ── Motion ────────────────────────────────────────────────────────
#     def _drive_to(self, tx, ty, tol=WP_TOL, timeout=60.0):
#         self._pid_reset()
#         self.current_lin = 0.0
#         t0 = time.time()
#         prev_t = t0

#         while (time.time() - t0) < timeout:
#             now = time.time()
#             dt = now - prev_t
#             if dt < 0.05:
#                 time.sleep(0.02)
#                 continue
#             prev_t = now

#             dx = tx - self.ox
#             dy = ty - self.oy
#             dist = math.hypot(dx, dy)
#             if dist < tol:
#                 break

#             desired = math.atan2(dy, dx)
#             yaw_err = norm_angle(desired - self.oyaw)

#             raw_lin = self._pid_linear(dist, dt)
#             raw_ang = self._pid_angular(yaw_err, dt)

#             if abs(yaw_err) > 0.8:
#                 raw_lin = 0.0
#             elif abs(yaw_err) > 0.3:
#                 raw_lin *= 0.5

#             raw_lin = max(0.0, min(MAX_LIN, raw_lin))
#             raw_ang = max(-MAX_ANG, min(MAX_ANG, raw_ang))
#             raw_lin = self._ramp(raw_lin, dt)
#             lin, ang = self._obstacle_adjust(raw_lin, raw_ang)

#             cmd = Twist()
#             cmd.linear.x = lin
#             cmd.angular.z = ang
#             self.cmd_pub.publish(cmd)

#         self.current_lin = 0.0
#         self.cmd_pub.publish(Twist())

#     def _drive_near(self, obj_x, obj_y, label=''):
#         dx = obj_x - self.ox
#         dy = obj_y - self.oy
#         dist = math.hypot(dx, dy)
#         if dist < STANDOFF:
#             self.get_logger().info(f'Already near {label}.')
#             return
#         scale = (dist - STANDOFF) / dist
#         gx = self.ox + dx * scale
#         gy = self.oy + dy * scale
#         self.get_logger().info(
#             f'Driving to ({gx:.2f},{gy:.2f}) [50cm from {label}]')
#         self._drive_to(gx, gy, tol=0.15, timeout=90.0)
#         self.get_logger().info(f'Reached {label}!')

#     # ── Retrace ───────────────────────────────────────────────────────
#     def _retrace(self):
#         if len(self.waypoints) < 2:
#             self.get_logger().info('No waypoints to retrace.')
#             return
#         rw = list(reversed(self.waypoints))
#         total = len(rw)
#         for i, (wx, wy) in enumerate(rw):
#             self.get_logger().info(
#                 f'  WP {i+1}/{total}: ({wx:.2f},{wy:.2f})',
#                 throttle_duration_sec=1.0)
#             self._drive_to(wx, wy, tol=WP_TOL, timeout=30.0)
#         self.get_logger().info('Back at start!')

#     # ── Display ───────────────────────────────────────────────────────
#     def _print_dict(self):
#         self.get_logger().info('═══════ Detected Objects ═══════')
#         if not self.object_dict:
#             self.get_logger().info('  (none)')
#         else:
#             for i, (o, c) in enumerate(self.object_dict.items(), 1):
#                 self.get_logger().info(
#                     f'  {i}. {o:20s} → ({c["x"]:+.2f}, {c["y"]:+.2f})')
#         self.get_logger().info('════════════════════════════════')


# def main(args=None):
#     rclpy.init(args=args)
#     node = SemanticNavigator()
#     ex = MultiThreadedExecutor(num_threads=4)
#     ex.add_node(node)
#     try:
#         ex.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()


"""
SemanticNavigator – Final Integrated Version (v5)
=================================================
• Median Depth Patch filtering (Fused RGB-D)
• Multi-Object tracking with SORT
• Automatic MarkerArray publishing for RViz
• Enhanced Command Logic (scan, stop scan, go to <obj>, <obj>)
"""

import math
import time
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import tf2_ros
import numpy as np

# Optional imports with fallbacks
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    from diff_drive_robot.sort import Sort
except ImportError:
    Sort = None

# Helpers
def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ab = (b[2] - b[0]) * (b[3] - b[1])
    return inter / (aa + ab - inter + 1e-6)

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))

def norm_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

# Configuration
YOLO_EVERY_N      = 5
STANDOFF          = 1.00
WP_SPACING        = 0.60
WP_TOL            = 0.25
MAX_LIN           = 2.0
MAX_ANG           = 1.2
RAMP_RATE         = 1.5
OBS_STOP_DIST     = 0.20
KP_LIN, KI_LIN, KD_LIN = 1.2, 0.02, 0.1
KP_ANG, KI_ANG, KD_ANG = 3.0, 0.01, 0.2

class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')
        self.get_logger().info('SemanticNavigator initializing …')
        self.cb = ReentrantCallbackGroup()

        self.scanning = False
        self.object_dict = {}
        self.frame_count = 0
        self.ox = self.oy = self.oyaw = 0.0
        self.waypoints = []
        self.obs_front = float('inf')
        self.current_lin = 0.0
        self._pid_reset()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_objects', 10)
        
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_file = os.path.expanduser('~/Documents/ROS_final_project/yolo_semantic_map.json')
        self._load_semantic_map()

        if YOLO:
            self.model = YOLO('yolo26n.pt')
            self.get_logger().info('YOLO loaded ✓')
        self.tracker = Sort(max_age=30) if Sort else None

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10, callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10, callback_group=self.cb)
        self.declare_parameter('rgb_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value

        self.create_subscription(Image, rgb_topic, self._img_cb, 10, callback_group=self.cb)
        self.latest_depth = None
        self.create_subscription(Image, depth_topic, self._depth_cb, 10, callback_group=self.cb)
        self.create_subscription(String, '/semantic_nav/command', self._cmd_cb, 10, callback_group=self.cb)
        
        self.create_timer(1.0, self._publish_markers_cb, callback_group=self.cb)
        self.get_logger().info('Ready. Commands: scan | stop scan | list | <object_id>')

    def _pid_reset(self):
        self.lin_i = self.lin_prev = 0.0
        self.ang_i = self.ang_prev = 0.0

    def _load_semantic_map(self):
        if os.path.exists(self.map_file):
            try:
                with open(self.map_file, 'r') as f: self.object_dict = json.load(f)
                self.get_logger().info(f"Map loaded: {len(self.object_dict)} objects.")
            except: pass

    def _save_semantic_map(self):
        with open(self.map_file, 'w') as f: json.dump(self.object_dict, f, indent=2)

    def _publish_markers_cb(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (oid, c) in enumerate(self.object_dict.items()):
            # Sphere
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', now
            m.ns, m.id, m.type = 'objects', i, Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(c['x']), float(c['y']), 0.2
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = float(c.get('r',0.0)), float(c.get('g',1.0)), float(c.get('b',0.0)), 0.8
            ma.markers.append(m)
            # Text Label
            t = Marker()
            t.header.frame_id, t.header.stamp = 'map', now
            t.ns, t.id, t.type = 'labels', i + 100, Marker.TEXT_VIEW_FACING
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(c['x']), float(c['y']), 0.6
            t.scale.z = 0.25
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = oid
            ma.markers.append(t)
        if ma.markers: self.marker_pub.publish(ma)

    def _odom_cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.ox, self.oy = t.transform.translation.x, t.transform.translation.y
            self.oyaw = yaw_from_quat(t.transform.rotation)
        except:
            self.ox, self.oy = msg.pose.pose.position.x, msg.pose.pose.position.y
            self.oyaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.scanning:
            if not self.waypoints or math.hypot(self.ox - self.waypoints[-1][0], self.oy - self.waypoints[-1][1]) > WP_SPACING:
                self.waypoints.append((self.ox, self.oy))

    def _scan_cb(self, msg):
        front_ranges = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                angle = math.atan2(math.sin(angle), math.cos(angle))
                if abs(angle) < math.radians(20):
                    front_ranges.append(r)
        self.obs_front = min(front_ranges) if front_ranges else float('inf')

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"CMD: {cmd}")
        
        if cmd == 'scan':
            self.scanning = True
            self.object_dict.clear()
            self.get_logger().info("SCAN STARTED")
        elif cmd in ['scan stop', 'stop scan', 'return home']:
            self.scanning = False
            self._save_semantic_map()
            self.get_logger().info("RETURNING HOME")
            self._retrace()
        elif cmd == 'list':
            self.get_logger().info(f"Objects: {list(self.object_dict.keys())}")
        elif cmd in self.object_dict:
            target = self.object_dict[cmd]
            self._drive_near(target['x'], target['y'], label=cmd)
        elif cmd.startswith("go to "):
            obj = cmd.replace("go to ", "").strip()
            if obj in self.object_dict:
                self._drive_near(self.object_dict[obj]['x'], self.object_dict[obj]['y'], label=obj)
        else:
            self.get_logger().warn(f"Unknown command or object: {cmd}")

    def _depth_cb(self, msg):
        try:
            if msg.encoding == '16UC1':
                # Real Kinect depth format (16-bit unsigned in millimeters)
                depth_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                self.latest_depth = depth_img.astype(np.float32) / 1000.0
            else:
                # Gazebo simulator depth format (32-bit float in meters)
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f"Depth cv_bridge error: {e}")

    def _img_cb(self, msg):
        if not self.scanning or not self.model: return
        self.frame_count += 1
        if self.frame_count % YOLO_EVERY_N != 0: return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(img, verbose=False)[0]
        
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = int((x1+x2)/2), int((y1+y2)/2)
            cls_name = results.names[int(box.cls[0])]

            # Median Depth Patch (Fusion)
            depth = -1.0
            if self.latest_depth is not None:
                patch = self.latest_depth[max(0,cy-5):cy+5, max(0,cx-5):cx+5]
                valid = patch[np.isfinite(patch) & (patch > 0.3)]
                if len(valid) > 0: depth = float(np.median(valid))

            # Filter logic (0.5m to 5.0m)
            if 0.5 < depth < 5.0:
                ang = self.oyaw + ((320 - cx) / 320.0) * (1.089 / 2.0)
                mx, my = self.ox + depth * math.cos(ang), self.oy + depth * math.sin(ang)
                
                # Spatial Deduplication
                if not any(math.hypot(mx-obj['x'], my-obj['y']) < 1.5 for obj in self.object_dict.values()):
                    bgr = img[cy, cx]
                    new_id = f"{cls_name}_{len(self.object_dict)+1}"
                    self.object_dict[new_id] = {'x': mx, 'y': my, 'r': float(bgr[2]/255), 'g': float(bgr[1]/255), 'b': float(bgr[0]/255)}
                    self.get_logger().info(f"MAP NEW: {new_id} ({depth:.1f}m)")

    def _drive_to(self, tx, ty, tol=WP_TOL, stop_at_obs=None):
        self._pid_reset()
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < 45.0:
            dx, dy = tx - self.ox, ty - self.oy
            dist = math.hypot(dx, dy)
            if dist < tol: break
            
            if stop_at_obs is not None and self.obs_front <= stop_at_obs:
                break
            
            yaw_err = norm_angle(math.atan2(dy, dx) - self.oyaw)
            lin = self._pid_linear(dist, 0.1)
            ang = self._pid_angular(yaw_err, 0.1)
            
            if abs(yaw_err) > 1.2: lin = 0.0
            if self.obs_front < OBS_STOP_DIST: lin = 0.0
            
            cmd = Twist()
            cmd.linear.x = max(0.0, min(MAX_LIN, lin))
            cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, ang))
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())

    def _drive_near(self, tx, ty, label):
        self._drive_to(tx, ty, tol=0.15, stop_at_obs=STANDOFF)
        self.get_logger().info(f"ARRIVED AT {label} (Distance: {self.obs_front:.2f}m)")

    def _retrace(self):
        for wx, wy in reversed(self.waypoints):
            self._drive_to(wx, wy)
        self.get_logger().info("HOME REACHED")

    def _pid_linear(self, e, dt):
        self.lin_i += e * dt
        d = (e - self.lin_prev) / dt
        self.lin_prev = e
        return KP_LIN * e + KI_LIN * self.lin_i + KD_LIN * d

    def _pid_angular(self, e, dt):
        self.ang_i += e * dt
        d = (e - self.ang_prev) / dt
        self.ang_prev = e
        return KP_ANG * e + KI_ANG * self.ang_i + KD_ANG * d

def main():
    rclpy.init()
    node = SemanticNavigator()
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

if __name__ == '__main__': main()