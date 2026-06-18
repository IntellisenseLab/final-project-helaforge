"""
semantic_navigator.py  (Real Hardware Edition — v5)
====================================================
Master ROS 2 node for Kobuki + Kinect v1/libfreenect RGB-D + RTAB-Map + Nav2.

What changed from v4 (simulation)
-----------------------------------
• Tracker   : SORT → Ultralytics BoT-SORT (model.track, persist=True)
• Depth      : Heuristic 300/bbox_height → real Kinect depth pixel lookup
• 3D coords  : Kinect optical-frame depth + optional LiDAR range refinement → map
• Scan Stop  : Nav2 NavigateThroughPoses route home, smooth PID fallback
• GO_TO      : odom-PID fallback kept; Nav2 action used preferentially
• RTAB-Map   : Mapping mode during scan, localization mode after scan stop
• LiDAR      : /scan refines object distance and feeds obstacle checks

Commands (publish to /semantic_nav/command, or say via voice_commander):
  scan environment – enable mapping mode, teleop + object detection
  scan stop        – stop mapping, list objects, Nav2 route home
  go to <object>   – send goal 5 cm in front of the stored object
  return home      – send Nav2 goal to the scan start pose
  list             – log detected objects

Tracker parameters:
  tracker_cfg  (string, default "botsort.yaml")   – swap to bytetrack.yaml for speed
  yolo_model   (string, default "yolov8n.pt")
  every_n      (int,    default 10)               – run YOLO every N frames
"""

import math
import time
import threading
import numpy as np

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_geometry_msgs   # registers PointStamped transform

# ── Optional imports ──────────────────────────────────────────────────────────
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except ImportError:
    BasicNavigator = None
    TaskResult = None


# ── Helpers ────────────────────────────────────────────────────────────────────
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y**2 + q.z**2))


def norm_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def make_pose_stamped(x: float, y: float, yaw: float, frame: str,
                      stamp) -> PoseStamped:
    """Build a PoseStamped from (x, y, yaw) for Nav2 goals."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp    = stamp
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def image_to_bgr_array(msg: Image) -> np.ndarray:
    """Convert common 8-bit ROS image encodings to a BGR numpy array."""
    encoding = msg.encoding.lower()
    channels_by_encoding = {
        'bgr8': 3,
        'rgb8': 3,
        'bgra8': 4,
        'rgba8': 4,
        'mono8': 1,
        '8uc1': 1,
        '8uc3': 3,
        '8uc4': 4,
    }
    if encoding not in channels_by_encoding:
        raise ValueError(f'unsupported RGB encoding: {msg.encoding}')

    channels = channels_by_encoding[encoding]
    row_pixels = msg.step // channels
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if channels == 1:
        img = raw.reshape(msg.height, msg.step)[:, :msg.width]
        return np.dstack((img, img, img)).copy()

    img = raw.reshape(msg.height, row_pixels, channels)[:, :msg.width, :]
    if encoding in {'rgb8', 'rgba8'}:
        return img[:, :, :3][:, :, ::-1].copy()
    return img[:, :, :3].copy()


def depth_image_to_meters(msg: Image) -> np.ndarray:
    """Convert 32FC1 metres or 16UC1 millimetres depth images to float metres."""
    encoding = msg.encoding.lower()
    if encoding == '32fc1':
        dtype = np.dtype(np.float32)
    elif encoding in {'16uc1', 'mono16'}:
        dtype = np.dtype(np.uint16)
    else:
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')

    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    item_size = dtype.itemsize
    row_items = msg.step // item_size
    raw = np.frombuffer(bytes(msg.data), dtype=dtype)
    depth = raw.reshape(msg.height, row_items)[:, :msg.width].astype(np.float32)

    if encoding in {'16uc1', 'mono16'}:
        depth *= 0.001
    depth[~np.isfinite(depth)] = np.nan
    depth[depth <= 0.0] = np.nan
    return np.ascontiguousarray(depth, dtype=np.float32)


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION  (tune for real Kobuki + Kinect v1/libfreenect)
# ══════════════════════════════════════════════════════════════════════════════
YOLO_EVERY_N      = 10       # run YOLO every N frames
OBJECT_CLEARANCE  = 0.05     # stop with front of robot 5 cm from target object
ROBOT_RADIUS      = 0.20     # Kobuki body radius used for base_footprint goal
WP_SPACING        = 0.60     # minimum distance between recorded waypoints
WP_TOL            = 0.25     # waypoint arrival tolerance (m)
RETURN_MAX_LIN    = 0.10     # smooth return-home speed (m/s)
RETURN_MAX_ANG    = 0.35     # smooth return-home turn rate (rad/s)
RETURN_LIN_RAMP   = 0.18     # m/s^2
RETURN_ANG_RAMP   = 0.55     # rad/s^2
RETURN_WP_TOL     = 0.18
RETURN_HOME_TOL   = 0.10
RETURN_YAW_TOL    = 0.18
NAV_RETURN_TIMEOUT_PER_POSE = 35.0

KP_LIN            = 1.2
KI_LIN            = 0.02
KD_LIN            = 0.10
KP_ANG            = 3.0
KI_ANG            = 0.01
KD_ANG            = 0.2

MAX_LIN           = 0.30     # Kobuki real-hardware safe max (m/s)
MAX_ANG           = 1.0
RAMP_RATE         = 1.5

OBS_STOP_DIST     = 0.30
OBS_SLOW_DIST     = 0.70
OBS_SIDE_STEER    = 0.80
LIDAR_ARC_FRONT   = 30
LIDAR_ARC_SIDE    = 60

# Kinect v1/libfreenect depth image dimensions. CameraInfo replaces these values
# once the real stream is running; they are only used for YOLO warm-up.
DEPTH_WIDTH  = 640
DEPTH_HEIGHT = 480


class SemanticNavigator(Node):

    def __init__(self):
        super().__init__('semantic_navigator')
        self.get_logger().info('SemanticNavigator (v5 — real hardware) initializing …')
        self.cb = ReentrantCallbackGroup()

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter('tracker_cfg', 'botsort.yaml')
        self.declare_parameter('yolo_model',  'yolov8n.pt')
        self.declare_parameter('every_n',     YOLO_EVERY_N)
        self.declare_parameter('return_max_linear', RETURN_MAX_LIN)
        self.declare_parameter('return_max_angular', RETURN_MAX_ANG)
        self.declare_parameter('return_strategy', 'nav2_waypoints')
        self.declare_parameter('navigation_backend', 'nav2')
        self.declare_parameter('rtabmap_mode_services', True)
        self.declare_parameter('object_clearance', OBJECT_CLEARANCE)
        self.declare_parameter('robot_radius', ROBOT_RADIUS)
        self.declare_parameter('object_depth_radius', 5)
        self.declare_parameter('object_lidar_fusion', True)
        self.declare_parameter('object_lidar_window_deg', 4.0)
        self.declare_parameter('object_lidar_max_delta', 0.75)
        self.declare_parameter('laser_frame', 'laser_link')

        self.tracker_cfg = self.get_parameter('tracker_cfg').value
        yolo_model       = self.get_parameter('yolo_model').value
        self.every_n     = self.get_parameter('every_n').value
        self.return_max_linear = float(
            self.get_parameter('return_max_linear').value)
        self.return_max_angular = float(
            self.get_parameter('return_max_angular').value)
        self.return_strategy = str(
            self.get_parameter('return_strategy').value).lower()
        self.navigation_backend = str(
            self.get_parameter('navigation_backend').value).lower()
        self.rtabmap_mode_services = bool(
            self.get_parameter('rtabmap_mode_services').value)
        self.object_clearance = max(
            0.0, float(self.get_parameter('object_clearance').value))
        self.robot_radius = max(
            0.0, float(self.get_parameter('robot_radius').value))
        self.object_standoff = self.robot_radius + self.object_clearance
        self.object_depth_radius = max(
            1, int(self.get_parameter('object_depth_radius').value))
        self.object_lidar_fusion = bool(
            self.get_parameter('object_lidar_fusion').value)
        self.object_lidar_window = math.radians(max(
            0.5, float(self.get_parameter('object_lidar_window_deg').value)))
        self.object_lidar_max_delta = max(
            0.05, float(self.get_parameter('object_lidar_max_delta').value))
        self.laser_frame = str(self.get_parameter('laser_frame').value)

        # ── State ──────────────────────────────────────────────────────────────
        self.scanning    = False
        self.object_dict: dict[str, dict] = {}   # label → {x, y, z, track_id}
        self.frame_count = 0

        # Robot pose (from /odom)
        self.ox = self.oy = self.oyaw = 0.0
        # Robot pose in /map from slam_pose_publisher, matching qbot A* nav.
        self.sx = self.sy = self.syaw = 0.0
        self._slam_pose_received = False

        # Waypoints recorded during scan
        self.waypoints: list[tuple[float, float]] = []       # odom fallback path
        self.map_waypoints: list[tuple[float, float, float]] = []  # Nav2 path
        self.home_x = self.home_y = self.home_yaw = 0.0
        self.home_odom_x = self.home_odom_y = self.home_odom_yaw = 0.0
        self._home_set = False
        self._nav_lock = threading.Lock()

        # Obstacle distances (from /scan if available)
        self.obs_front = float('inf')
        self.obs_left  = float('inf')
        self.obs_right = float('inf')
        self._scan_lock = threading.Lock()
        self._last_scan: LaserScan | None = None

        # Latest depth image (set by _depth_cb)
        self._depth_image: np.ndarray | None = None
        self._depth_lock  = threading.Lock()

        # Kinect v1 camera intrinsics
        # Populated from CameraInfo once received; fall back to typical Kinect v1 values.
        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.camera_frame_id = 'camera_rgb_optical_frame'
        self._cam_info_received = False

        self._pid_reset()
        self.current_lin = 0.0
        self.current_ang = 0.0

        # ── ROS interfaces ──────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ui_goal_pub = self.create_publisher(Point, '/ui_goal', 10)
        teleop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.teleop_pub = self.create_publisher(
            Bool, '/semantic_nav/teleop_enabled', teleop_qos)
        self.status_pub = self.create_publisher(
            String, '/semantic_nav/status', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic_nav/object_markers', 10)
        self._publish_teleop_enabled(False)

        self.tf_buffer  = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── YOLO + BoT-SORT ────────────────────────────────────────────────────
        self.model = None
        if YOLO is not None:
            self.get_logger().info(f'Loading YOLO model: {yolo_model} …')
            try:
                self.model = YOLO(yolo_model)
                # Warm-up pass
                self.model.track(
                    np.zeros((DEPTH_HEIGHT, DEPTH_WIDTH, 3), dtype=np.uint8),
                    tracker=self.tracker_cfg, persist=True, verbose=False)
                self.get_logger().info(
                    f'YOLO warmed up with {self.tracker_cfg} ✓\n'
                    '  (Switch to bytetrack.yaml with param tracker_cfg:=bytetrack.yaml'
                    ' if processing is slow)')
            except Exception as exc:
                self.get_logger().warn(
                    f'YOLO could not start ({exc}). '
                    'Mapping and teleop still work, but object detection is disabled.')
        else:
            self.get_logger().warn(
                'ultralytics not found — object detection disabled.\n'
                '  pip3 install ultralytics --break-system-packages')

        # ── Optional navigation backends ───────────────────────────────────────
        if self.navigation_backend == 'nav2' and BasicNavigator is not None:
            self.nav = BasicNavigator()
        elif self.navigation_backend == 'nav2':
            self.nav = None
            self.get_logger().warn(
                'nav2_simple_commander not found — Nav2 action goals disabled.')
        else:
            self.nav = None
            self.get_logger().info(
                f'Navigation backend "{self.navigation_backend}" selected; '
                'object/home goals will be published to /ui_goal.')

        # ── RTAB-Map mode service clients ─────────────────────────────────────
        self.rtabmap_mapping_cli = self.create_client(
            Empty, '/rtabmap/set_mode_mapping', callback_group=self.cb)
        self.rtabmap_localization_cli = self.create_client(
            Empty, '/rtabmap/set_mode_localization', callback_group=self.cb)

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',
                                 self._odom_cb, 10, callback_group=self.cb)
        self.create_subscription(Odometry, '/slam_pose',
                                 self._slam_pose_cb, 10, callback_group=self.cb)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_cb, 10, callback_group=self.cb)
        self.create_subscription(Image, '/camera/image_raw',
                                 self._img_cb, 10, callback_group=self.cb)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, 10, callback_group=self.cb)
        from sensor_msgs.msg import CameraInfo
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._cam_info_cb, 10, callback_group=self.cb)
        self.create_subscription(String, '/semantic_nav/command',
                                 self._cmd_cb, 10, callback_group=self.cb)

        self.create_timer(1.0, self._publish_object_markers,
                          callback_group=self.cb)

        self.get_logger().info(
            '\n'
            '╔═══════════════════════════════════════════════╗\n'
            '║   SemanticNavigator v5 (real hardware) ready  ║\n'
            '║                                               ║\n'
            '║  Commands:                                    ║\n'
            '║    scan environment – start mapping scan      ║\n'
            '║    scan stop        – list, Nav2 route home   ║\n'
            '║    go to <object>   – navigate to object      ║\n'
            '║    return home      – go to scan start pose   ║\n'
            '║    list          – print detected objects     ║\n'
            '╚═══════════════════════════════════════════════╝')

    # ── PID ────────────────────────────────────────────────────────────────────
    def _pid_reset(self):
        self.lin_i = self.lin_prev = 0.0
        self.ang_i = self.ang_prev = 0.0

    def _pid_linear(self, error, dt):
        self.lin_i += error * dt
        self.lin_i  = max(-0.5, min(0.5, self.lin_i))
        d = (error - self.lin_prev) / dt if dt > 0 else 0.0
        self.lin_prev = error
        return KP_LIN * error + KI_LIN * self.lin_i + KD_LIN * d

    def _pid_angular(self, error, dt):
        self.ang_i += error * dt
        self.ang_i  = max(-1.0, min(1.0, self.ang_i))
        d = (error - self.ang_prev) / dt if dt > 0 else 0.0
        self.ang_prev = error
        return KP_ANG * error + KI_ANG * self.ang_i + KD_ANG * d

    def _ramp(self, target, dt):
        max_delta = RAMP_RATE * dt
        if target > self.current_lin:
            self.current_lin = min(target, self.current_lin + max_delta)
        else:
            self.current_lin = max(target, self.current_lin - max_delta)
        return self.current_lin

    def _ramp_value(self, current: float, target: float,
                    rate: float, dt: float) -> float:
        max_delta = max(rate, 0.01) * max(dt, 0.0)
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    # ── Obstacle avoidance ─────────────────────────────────────────────────────
    def _obstacle_adjust(self, raw_lin, raw_ang):
        lin, ang = raw_lin, raw_ang
        if self.obs_front < OBS_STOP_DIST:
            lin = 0.0
            ang = MAX_ANG if self.obs_left > self.obs_right else -MAX_ANG
        elif self.obs_front < OBS_SLOW_DIST:
            factor = (self.obs_front - OBS_STOP_DIST) / (OBS_SLOW_DIST - OBS_STOP_DIST)
            lin *= max(0.1, factor)
        if self.obs_left < OBS_SIDE_STEER and lin > 0:
            ang -= 0.4
        if self.obs_right < OBS_SIDE_STEER and lin > 0:
            ang += 0.4
        lin = max(-MAX_LIN, min(MAX_LIN, lin))
        ang = max(-MAX_ANG, min(MAX_ANG, ang))
        return lin, ang

    # ── Shared publishers / helpers ───────────────────────────────────────────
    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_teleop_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.teleop_pub.publish(msg)

    def _set_rtabmap_mode(self, mapping: bool):
        if not self.rtabmap_mode_services:
            return
        service_name = 'mapping' if mapping else 'localization'
        client = self.rtabmap_mapping_cli if mapping else self.rtabmap_localization_cli
        if client.service_is_ready():
            client.call_async(Empty.Request())
            self.get_logger().info(f'RTAB-Map switched to {service_name} mode.')
        else:
            self.get_logger().warn(
                f'RTAB-Map {service_name} service not ready; continuing.')

    def _current_map_pose(self) -> tuple[float, float, float]:
        if self._slam_pose_received:
            return (self.sx, self.sy, self.syaw)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.3))
            t = tf_msg.transform.translation
            yaw = yaw_from_quat(tf_msg.transform.rotation)
            return (t.x, t.y, yaw)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'Cannot read map→base_footprint TF yet ({e}); '
                'using odom pose as a temporary fallback.',
                throttle_duration_sec=5.0)
            return (self.ox, self.oy, self.oyaw)

    def _clear_object_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def _publish_object_markers(self):
        if not self.object_dict:
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for i, (label, c) in enumerate(self.object_dict.items()):
            z = c['z'] if math.isfinite(c['z']) else 0.10

            pin = Marker()
            pin.header.frame_id = 'map'
            pin.header.stamp = now
            pin.ns = 'semantic_objects'
            pin.id = i * 2
            pin.type = Marker.SPHERE
            pin.action = Marker.ADD
            pin.pose.position.x = float(c['x'])
            pin.pose.position.y = float(c['y'])
            pin.pose.position.z = float(z)
            pin.pose.orientation.w = 1.0
            pin.scale.x = 0.16
            pin.scale.y = 0.16
            pin.scale.z = 0.16
            pin.color.r = 0.05
            pin.color.g = 0.75
            pin.color.b = 1.0
            pin.color.a = 0.95
            markers.markers.append(pin)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'semantic_object_labels'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(c['x'])
            text.pose.position.y = float(c['y'])
            text.pose.position.z = float(z) + 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.r = 1.0
            text.color.g = 0.92
            text.color.b = 0.15
            text.color.a = 1.0
            text.text = label
            markers.markers.append(text)

        self.marker_pub.publish(markers)

    def _start_nav_thread(self, x: float, y: float, yaw: float,
                          label: str = 'target'):
        threading.Thread(
            target=self._nav2_goto,
            args=(x, y, yaw, label),
            daemon=True).start()

    def _start_pid_return_thread(self, label: str = 'home'):
        threading.Thread(
            target=self._pid_return_home,
            args=(label,),
            daemon=True).start()

    def _start_return_home_thread(self, label: str = 'home'):
        threading.Thread(
            target=self._return_home_nav2_then_pid,
            args=(label,),
            daemon=True).start()

    def _using_qbot_astar(self) -> bool:
        return self.navigation_backend in {
            'qbot_astar', 'qbot', 'astar', 'lidar_astar'
        }

    def _send_qbot_goal(self, x: float, y: float, label: str = 'target'):
        goal = Point()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = 0.0
        self.ui_goal_pub.publish(goal)
        self._publish_status(
            f'A* goal sent for {label}: map({x:.2f}, {y:.2f}).')

    def _return_home_nav2_then_pid(self, label: str = 'home'):
        """Return home using Nav2 waypoints, with smooth PID as fallback."""
        strategy = self.return_strategy
        if strategy in {'pid', 'pid_waypoints', 'odom_pid'}:
            self._pid_return_home(label)
            return

        ok = self._nav2_return_through_waypoints(label)
        if ok:
            return

        self._publish_status(
            'Nav2 return did not complete cleanly; falling back to smooth PID.')
        self._pid_return_home(label)

    def _build_nav2_return_route(self) -> list[PoseStamped]:
        route = list(reversed(self.map_waypoints))
        if route:
            rx, ry, _ = self._current_map_pose()
            if math.hypot(route[0][0] - rx, route[0][1] - ry) < RETURN_WP_TOL:
                route = route[1:]

        # Always finish exactly at the recorded map-frame home pose.
        if not route or math.hypot(
                route[-1][0] - self.home_x,
                route[-1][1] - self.home_y) > RETURN_HOME_TOL:
            route.append((self.home_x, self.home_y, self.home_yaw))
        else:
            route[-1] = (self.home_x, self.home_y, self.home_yaw)

        now = self.get_clock().now().to_msg()
        poses = []
        for i, (x, y, yaw) in enumerate(route):
            if i < len(route) - 1:
                if i + 1 < len(route):
                    nx, ny, _ = route[i + 1]
                    yaw = math.atan2(ny - y, nx - x)
                else:
                    yaw = self.home_yaw
            poses.append(make_pose_stamped(x, y, yaw, 'map', now))
        return poses

    def _nav2_return_through_waypoints(self, label: str = 'home') -> bool:
        with self._nav_lock:
            if self.nav is None:
                self.get_logger().warn(
                    'Nav2 not available for waypoint return.')
                return False

            poses = self._build_nav2_return_route()
            if not poses:
                return False

            try:
                if not self.nav.isTaskComplete():
                    self.nav.cancelTask()
            except Exception:
                pass

            try:
                self.nav.clearAllCostmaps()
            except Exception as exc:
                self.get_logger().warn(
                    f'Could not clear Nav2 costmaps before return: {exc}')

            self._publish_status(
                f'Nav2 return started with {len(poses)} map waypoints.')
            accepted = self.nav.goThroughPoses(poses)
            if not accepted:
                self.get_logger().warn('Nav2 rejected return waypoint route.')
                return False

            timeout = max(
                45.0, NAV_RETURN_TIMEOUT_PER_POSE * float(len(poses)))
            start_t = time.time()
            while not self.nav.isTaskComplete():
                if (time.time() - start_t) > timeout:
                    self.get_logger().warn(
                        f'Nav2 return timeout after {timeout:.1f}s.')
                    try:
                        self.nav.cancelTask()
                    except Exception:
                        pass
                    return False
                time.sleep(0.2)

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self._publish_status(f'Arrived at {label} via Nav2.')
                return True
            if result == TaskResult.CANCELED:
                self.get_logger().warn('Nav2 return was cancelled.')
            else:
                self.get_logger().warn(
                    f'Nav2 return failed with result={result}.')
            return False

    def _pid_return_home(self, label: str = 'home'):
        """Low-speed smooth odom waypoint retrace to the scan-start pose."""
        with self._nav_lock:
            if self.nav is not None:
                try:
                    if not self.nav.isTaskComplete():
                        self.nav.cancelTask()
                except Exception:
                    pass

            self._publish_status(
                f'PID return started at max '
                f'{self.return_max_linear:.2f} m/s, '
                f'{self.return_max_angular:.2f} rad/s.')

            # Skip the current last waypoint if it is already very close.
            route = list(reversed(self.waypoints))
            if route:
                cx, cy = self.ox, self.oy
                if math.hypot(route[0][0] - cx, route[0][1] - cy) < RETURN_WP_TOL:
                    route = route[1:]

            total = len(route)
            for i, (wx, wy) in enumerate(route, 1):
                self._publish_status(
                    f'PID return waypoint {i}/{total}: ({wx:.2f}, {wy:.2f})')
                ok = self._drive_to_smooth(
                    wx, wy, tol=RETURN_WP_TOL, timeout=45.0)
                if not ok:
                    self._publish_status(
                        'PID return waypoint timeout; continuing toward home.')

            self._publish_status(
                f'PID final home approach: '
                f'({self.home_odom_x:.2f}, {self.home_odom_y:.2f})')
            self._drive_to_smooth(
                self.home_odom_x, self.home_odom_y,
                tol=RETURN_HOME_TOL, timeout=90.0)
            self._rotate_to_yaw_smooth(self.home_odom_yaw, timeout=20.0)
            self.cmd_pub.publish(Twist())
            self.current_lin = 0.0
            self.current_ang = 0.0
            self._publish_status(f'Arrived at {label}.')

    # ── Subscribers ────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.ox   = msg.pose.pose.position.x
        self.oy   = msg.pose.pose.position.y
        self.oyaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.scanning and self.waypoints:
            lx, ly = self.waypoints[-1]
            if math.hypot(self.ox - lx, self.oy - ly) >= WP_SPACING:
                self.waypoints.append((self.ox, self.oy))
                mx, my, myaw = self._current_map_pose()
                self.map_waypoints.append((mx, my, myaw))

    def _slam_pose_cb(self, msg: Odometry):
        self.sx = msg.pose.pose.position.x
        self.sy = msg.pose.pose.position.y
        self.syaw = yaw_from_quat(msg.pose.pose.orientation)
        self._slam_pose_received = True

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return
        with self._scan_lock:
            self._last_scan = msg

        def zone_min(centre_deg, half_arc_deg):
            centre_idx = int(
                (math.radians(centre_deg) - msg.angle_min)
                / msg.angle_increment) % n
            half = int(math.radians(half_arc_deg) / msg.angle_increment)
            lo = max(0, centre_idx - half)
            hi = min(n - 1, centre_idx + half)
            vals = [r for r in msg.ranges[lo:hi + 1]
                    if msg.range_min < r < msg.range_max]
            return min(vals) if vals else float('inf')

        self.obs_front = zone_min(0,   LIDAR_ARC_FRONT)
        self.obs_left  = zone_min(90,  LIDAR_ARC_SIDE // 2)
        self.obs_right = zone_min(-90, LIDAR_ARC_SIDE // 2)

    def _laser_range_at_bearing(self, bearing: float) -> float | None:
        """Return nearest valid LiDAR range around a bearing in laser_link."""
        with self._scan_lock:
            msg = self._last_scan
        if msg is None or not msg.ranges or msg.angle_increment == 0.0:
            return None

        n = len(msg.ranges)
        inc = msg.angle_increment
        span = abs(inc) * max(0, n - 1)
        full_scan = span >= (2.0 * math.pi - 2.0 * abs(inc))

        if inc > 0.0:
            lo_angle = msg.angle_min
            hi_angle = msg.angle_min + inc * (n - 1)
            target = bearing
            while target < lo_angle:
                target += 2.0 * math.pi
            while target > hi_angle:
                target -= 2.0 * math.pi
            if not full_scan and not (lo_angle <= target <= hi_angle):
                return None
            centre_idx = int(round((target - msg.angle_min) / inc))
        else:
            hi_angle = msg.angle_min
            lo_angle = msg.angle_min + inc * (n - 1)
            target = bearing
            while target < lo_angle:
                target += 2.0 * math.pi
            while target > hi_angle:
                target -= 2.0 * math.pi
            if not full_scan and not (lo_angle <= target <= hi_angle):
                return None
            centre_idx = int(round((target - msg.angle_min) / inc))

        half = max(1, int(self.object_lidar_window / abs(inc)))
        ranges = []
        for offset in range(-half, half + 1):
            idx = centre_idx + offset
            if full_scan:
                idx %= n
            elif idx < 0 or idx >= n:
                continue
            r = float(msg.ranges[idx])
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                ranges.append(r)
        return min(ranges) if ranges else None

    def _depth_cb(self, msg: Image):
        """Cache the latest Kinect depth image (32FC1, metres)."""
        try:
            depth = depth_image_to_meters(msg)
            with self._depth_lock:
                self._depth_image = depth.copy()
        except Exception as e:
            self.get_logger().warn(f'Depth conversion error: {e}',
                                   throttle_duration_sec=5.0)

    def _cam_info_cb(self, msg):
        """Update camera intrinsics from live CameraInfo (first message only)."""
        if self._cam_info_received:
            return
        k = msg.k  # row-major 3×3
        self.fx = k[0]
        self.fy = k[4]
        self.cx = k[2]
        self.cy = k[5]
        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id
        self._cam_info_received = True
        self.get_logger().info(
            f'Camera intrinsics updated: fx={self.fx:.2f} fy={self.fy:.2f} '
            f'cx={self.cx:.2f} cy={self.cy:.2f} frame={self.camera_frame_id}')

    # ── 3D coordinate extraction ────────────────────────────────────────────────
    def _get_3d_map_coords(
            self, cx_px: float, cy_px: float, stamp,
            radius: int | None = None) -> tuple[float, float, float] | None:
        """
        Back-project pixel (cx_px, cy_px) using real Kinect depth data.
        Samples a small neighbourhood to reduce noise.
        Returns (map_x, map_y, map_z) or None on failure.
        """
        if radius is None:
            radius = self.object_depth_radius

        with self._depth_lock:
            if self._depth_image is None:
                return None
            depth_img = self._depth_image

        u, v = int(cx_px), int(cy_px)
        h, w = depth_img.shape

        # Clamp to image bounds
        u1, u2 = max(0, u - radius), min(w, u + radius + 1)
        v1, v2 = max(0, v - radius), min(h, v + radius + 1)

        patch = depth_img[v1:v2, u1:u2]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            self.get_logger().debug(
                f'No valid depth at pixel ({u},{v}) — object coordinate skipped.')
            return None

        z = float(np.median(valid))   # metres, in camera frame

        # Back-project to camera 3D (depth optical frame, X right, Y down, Z forward)
        x_cam = (cx_px - self.cx) * z / self.fx
        y_cam = (cy_px - self.cy) * z / self.fy
        z_cam = z

        # Build a PointStamped in the registered camera frame.
        pt = PointStamped()
        pt.header.stamp    = stamp
        pt.header.frame_id = self.camera_frame_id
        if self.camera_frame_id.endswith('optical_frame'):
            # Optical frame convention: x right, y down, z forward.
            # Do not convert these values when the frame id is optical.
            pt.point.x = float(x_cam)
            pt.point.y = float(y_cam)
            pt.point.z = float(z_cam)
        else:
            # Standard ROS camera/body convention: x forward, y left, z up.
            pt.point.x = float(z_cam)
            pt.point.y = float(-x_cam)
            pt.point.z = float(-y_cam)

        map_source = pt
        if self.object_lidar_fusion:
            try:
                pt_laser = self.tf_buffer.transform(
                    pt, self.laser_frame, timeout=Duration(seconds=0.1))
                bearing = math.atan2(pt_laser.point.y, pt_laser.point.x)
                cam_range = math.hypot(pt_laser.point.x, pt_laser.point.y)
                laser_range = self._laser_range_at_bearing(bearing)
                if laser_range is not None and cam_range > 0.05:
                    delta = abs(laser_range - cam_range)
                    if delta <= self.object_lidar_max_delta:
                        fused_range = 0.30 * cam_range + 0.70 * laser_range
                        pt_laser.point.x = fused_range * math.cos(bearing)
                        pt_laser.point.y = fused_range * math.sin(bearing)
                        map_source = pt_laser
                    else:
                        self.get_logger().debug(
                            f'LiDAR/object range mismatch: camera={cam_range:.2f}m '
                            f'lidar={laser_range:.2f}m, keeping Kinect depth.')
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug(f'LiDAR object fusion skipped: {e}')

        try:
            pt_map = self.tf_buffer.transform(
                map_source, 'map', timeout=Duration(seconds=0.3))
            return (pt_map.point.x, pt_map.point.y, pt_map.point.z)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF to map failed: {e}')
            return None

    # ── Command handler ────────────────────────────────────────────────────────
    def _normalize_command(self, text: str) -> str:
        cmd = text.strip().lower()
        cmd = ' '.join(cmd.replace('_', ' ').split())

        if cmd in {'scan', 'start scan', 'start scanning',
                   'scan environment', 'scan the environment',
                   'scan room', 'map environment'}:
            return 'scan'
        if cmd in {'scan stop', 'stop scan', 'stop scanning',
                   'end scan', 'finish scan', 'stop mapping'}:
            return 'scan stop'
        if cmd in {'return home', 'go home', 'come home', 'back home'}:
            return 'return home'
        if cmd in {'list', 'show objects', 'list objects',
                   'what objects', 'detected objects'}:
            return 'list'
        if cmd.startswith('go to '):
            return cmd[6:].strip()
        if cmd.startswith('navigate to '):
            return cmd[12:].strip()
        return cmd

    def _resolve_object_target(self, cmd: str) -> tuple[str, dict] | None:
        key = cmd.replace(' ', '_')
        if key in self.object_dict:
            return key, self.object_dict[key]

        class_matches = [
            (label, c) for label, c in self.object_dict.items()
            if c.get('class', '').lower() == cmd
        ]
        if len(class_matches) == 1:
            return class_matches[0]
        if len(class_matches) > 1:
            labels = [label for label, _ in class_matches]
            self.get_logger().warn(
                f'"{cmd}" matches multiple objects: {labels}. '
                'Use the full label from the object list.')
            return None

        prefix_matches = [
            (label, c) for label, c in self.object_dict.items()
            if label.lower().startswith(key)
        ]
        if len(prefix_matches) == 1:
            return prefix_matches[0]
        if len(prefix_matches) > 1:
            labels = [label for label, _ in prefix_matches]
            self.get_logger().warn(
                f'"{cmd}" is ambiguous: {labels}. '
                'Use the full label from the object list.')
            return None
        return None

    def _goal_near_object(self, obj_x: float, obj_y: float) -> tuple[float, float, float]:
        rx, ry, _ = self._current_map_pose()
        dx = obj_x - rx
        dy = obj_y - ry
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if dist > 0.001 else 0.0
        if dist <= self.object_standoff:
            return (rx, ry, yaw)
        scale = max(0.0, (dist - self.object_standoff) / dist)
        return (rx + dx * scale, ry + dy * scale, yaw)

    def _cmd_cb(self, msg: String):
        cmd = self._normalize_command(msg.data)
        self.get_logger().info(f'CMD: "{cmd}"')

        if cmd == 'scan':
            self._start_scan()

        elif cmd == 'scan stop':
            self._stop_scan()

        elif cmd == 'return home':
            if self._using_qbot_astar():
                self._send_qbot_goal(self.home_x, self.home_y, 'home')
            else:
                self._publish_status(
                    f'Nav2 waypoint return home '
                    f'map({self.home_x:.2f}, {self.home_y:.2f}) ...')
                self._start_return_home_thread('home')

        elif cmd == 'list':
            self._print_dict()

        else:
            target = self._resolve_object_target(cmd)
            if target is not None:
                label, c = target
                gx, gy, gyaw = self._goal_near_object(c['x'], c['y'])
                self._publish_status(
                    f'Navigating to {label} near '
                    f'({c["x"]:.2f}, {c["y"]:.2f}, z={c["z"]:.2f}) ...')
                if self._using_qbot_astar():
                    self._send_qbot_goal(gx, gy, label)
                else:
                    self._start_nav_thread(gx, gy, gyaw, label)
            else:
                self.get_logger().warn(
                    f'"{cmd}" unknown. Detected objects: '
                    f'{list(self.object_dict.keys())}')

    def _start_scan(self):
        """Begin scan mode: record home position, enable object detection."""
        self.home_x, self.home_y, self.home_yaw = self._current_map_pose()
        self.home_odom_x = self.ox
        self.home_odom_y = self.oy
        self.home_odom_yaw = self.oyaw
        self._home_set = True
        self.waypoints = [(self.home_odom_x, self.home_odom_y)]
        self.map_waypoints = [(self.home_x, self.home_y, self.home_yaw)]
        self.object_dict.clear()
        self._clear_object_markers()
        self.scanning = True
        self._publish_teleop_enabled(True)
        self._set_rtabmap_mode(mapping=True)

        self._publish_status(
            f'SCAN started. Home map=({self.home_x:.2f}, {self.home_y:.2f}), '
            f'odom=({self.home_odom_x:.2f}, {self.home_odom_y:.2f}). '
            'Teleop enabled; drive with arrow_teleop.')

    def _stop_scan(self):
        """Stop scan mode, switch RTAB-Map to localization, navigate home."""
        self.scanning = False
        self._publish_teleop_enabled(False)
        self._publish_status(
            f'SCAN stopped: {len(self.object_dict)} objects detected, '
            f'{len(self.waypoints)} waypoints recorded. Teleop disabled.')
        self._print_dict()

        # Freeze the map and keep RTAB-Map localizing against the saved graph.
        self._set_rtabmap_mode(mapping=False)

        if self._using_qbot_astar():
            self._send_qbot_goal(self.home_x, self.home_y, 'home')
            return

        # Return home through the recorded map-frame waypoints with Nav2.
        # If Nav2 fails, the thread falls back to smooth odom-PID retracing.
        self._publish_status(
            f'Nav2 waypoint return home '
            f'map({self.home_x:.2f}, {self.home_y:.2f}) ...')
        self._start_return_home_thread('home')

    # ── Nav2 goal sender ────────────────────────────────────────────────────────
    def _nav2_goto(self, target_x: float, target_y: float, target_yaw: float,
                   label: str = 'target'):
        """
        Send a NavigateToPose goal to Nav2.
        Falls back to odom-PID _drive_to() if Nav2 is unavailable.
        """
        with self._nav_lock:
            if self.nav is None:
                self.get_logger().warn(
                    'Nav2 not available — falling back to odom-PID drive.')
                self._drive_to(target_x, target_y, tol=0.15, timeout=90.0)
                self._publish_status(f'Arrived at {label}.')
                return

            if not self.nav.isTaskComplete():
                self.get_logger().info('Nav2 was busy; cancelling old goal.')
                self.nav.cancelTask()

            goal = make_pose_stamped(
                target_x, target_y, target_yaw,
                frame='map',
                stamp=self.get_clock().now().to_msg())

            self.get_logger().info(
                f'Nav2 goal ({label}) → ({target_x:.2f}, {target_y:.2f}, '
                f'yaw={math.degrees(target_yaw):.1f} deg)')

            self.nav.goToPose(goal)

            # Monitor result
            while not self.nav.isTaskComplete():
                time.sleep(0.2)

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self._publish_status(f'Arrived at {label}.')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Nav2: goal to {label} was cancelled.')
            else:
                self.get_logger().warn(
                    f'Nav2: failed to reach {label} (result={result}). '
                    'Falling back to odom-PID drive.')
                self._drive_to(target_x, target_y, tol=0.15, timeout=90.0)
                self._publish_status(f'Arrived at {label}.')

    # ── Image callback (YOLO + BoT-SORT + 3D coord registration) ──────────────
    def _img_cb(self, msg: Image):
        if not self.scanning or not self.model:
            return

        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return

        try:
            img = image_to_bgr_array(msg)
        except Exception:
            return

        stamp = msg.header.stamp

        # ── BoT-SORT tracking ─────────────────────────────────────────────────
        results = self.model.track(
            img,
            tracker=self.tracker_cfg,
            persist=True,
            verbose=False)

        if not results or results[0].boxes is None:
            return

        boxes = results[0].boxes
        if boxes.id is None:
            return   # tracker not yet assigned IDs

        ids     = boxes.id.cpu().numpy().astype(int)
        xyxy    = boxes.xyxy.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy().astype(int)
        names   = results[0].names

        for tid, box, cls_id in zip(ids, xyxy, cls_ids):
            x1, y1, x2, y2 = box
            label = f'{names[cls_id]}_{tid}'

            if label in self.object_dict:
                continue   # already registered

            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0

            # ── Real 3D coordinate from Kinect depth ──────────────────────────
            coords = self._get_3d_map_coords(cx_px, cy_px, stamp)

            if coords is None:
                # Depth lookup failed (NaN pixel or TF not ready)
                # Skip this detection; will retry on the next frame
                self.get_logger().debug(
                    f'Depth unavailable for {label} at ({cx_px:.0f},{cy_px:.0f}) '
                    '— skipping this frame.')
                continue

            mx, my, mz = coords

            self.object_dict[label] = {
                'x': mx, 'y': my, 'z': mz,
                'track_id': tid,
                'class': names[cls_id],
            }
            self._publish_object_markers()

            n = len(self.object_dict)
            self._publish_status(
                f'New object #{n}: {label} at '
                f'map({mx:+.2f}, {my:+.2f}, z={mz:+.2f})')

    def _gentle_obstacle_limit(self, lin_target: float,
                               ang_target: float) -> tuple[float, float]:
        lin = lin_target
        ang = ang_target
        max_ang = self.return_max_angular

        if self.obs_front < OBS_STOP_DIST:
            lin = 0.0
            turn = max_ang if self.obs_left > self.obs_right else -max_ang
            ang = max(-max_ang, min(max_ang, turn))
        elif self.obs_front < OBS_SLOW_DIST:
            factor = (self.obs_front - OBS_STOP_DIST) / (
                OBS_SLOW_DIST - OBS_STOP_DIST)
            lin *= max(0.15, min(1.0, factor))

        if self.obs_left < OBS_SIDE_STEER and lin > 0.0:
            ang -= 0.12
        if self.obs_right < OBS_SIDE_STEER and lin > 0.0:
            ang += 0.12

        return (
            max(-self.return_max_linear, min(self.return_max_linear, lin)),
            max(-max_ang, min(max_ang, ang)))

    def _drive_to_smooth(self, tx: float, ty: float,
                         tol: float = RETURN_HOME_TOL,
                         timeout: float = 90.0) -> bool:
        """Smooth low-speed PID drive in odom coordinates."""
        self.current_lin = 0.0
        self.current_ang = 0.0
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
            if dist <= tol:
                self.cmd_pub.publish(Twist())
                self.current_lin = 0.0
                self.current_ang = 0.0
                return True

            desired = math.atan2(dy, dx)
            yaw_err = norm_angle(desired - self.oyaw)

            lin_target = min(self.return_max_linear, 0.45 * dist)
            if abs(yaw_err) > 1.0:
                lin_target = 0.0
            elif abs(yaw_err) > 0.55:
                lin_target *= 0.25
            elif abs(yaw_err) > 0.25:
                lin_target *= 0.55

            ang_target = max(
                -self.return_max_angular,
                min(self.return_max_angular, 1.25 * yaw_err))

            lin_target, ang_target = self._gentle_obstacle_limit(
                lin_target, ang_target)

            self.current_lin = self._ramp_value(
                self.current_lin, lin_target, RETURN_LIN_RAMP, dt)
            self.current_ang = self._ramp_value(
                self.current_ang, ang_target, RETURN_ANG_RAMP, dt)

            cmd = Twist()
            cmd.linear.x = self.current_lin
            cmd.angular.z = self.current_ang
            self.cmd_pub.publish(cmd)

        self.cmd_pub.publish(Twist())
        self.current_lin = 0.0
        self.current_ang = 0.0
        return False

    def _rotate_to_yaw_smooth(self, target_yaw: float,
                              timeout: float = 20.0) -> bool:
        self.current_lin = 0.0
        self.current_ang = 0.0
        t0 = time.time()
        prev_t = t0

        while (time.time() - t0) < timeout:
            now = time.time()
            dt = now - prev_t
            if dt < 0.05:
                time.sleep(0.02)
                continue
            prev_t = now

            err = norm_angle(target_yaw - self.oyaw)
            if abs(err) <= RETURN_YAW_TOL:
                self.cmd_pub.publish(Twist())
                self.current_ang = 0.0
                return True

            ang_target = max(
                -self.return_max_angular,
                min(self.return_max_angular, 1.0 * err))
            self.current_ang = self._ramp_value(
                self.current_ang, ang_target, RETURN_ANG_RAMP, dt)

            cmd = Twist()
            cmd.angular.z = self.current_ang
            self.cmd_pub.publish(cmd)

        self.cmd_pub.publish(Twist())
        self.current_ang = 0.0
        return False

    # ── odom-PID motion (fallback when Nav2 unavailable) ──────────────────────
    def _drive_to(self, tx, ty, tol=WP_TOL, timeout=60.0):
        self._pid_reset()
        self.current_lin = 0.0
        t0 = time.time()
        prev_t = t0

        while (time.time() - t0) < timeout:
            now = time.time()
            dt  = now - prev_t
            if dt < 0.05:
                time.sleep(0.02)
                continue
            prev_t = now

            dx   = tx - self.ox
            dy   = ty - self.oy
            dist = math.hypot(dx, dy)
            if dist < tol:
                break

            desired = math.atan2(dy, dx)
            yaw_err = norm_angle(desired - self.oyaw)

            raw_lin = self._pid_linear(dist, dt)
            raw_ang = self._pid_angular(yaw_err, dt)

            if abs(yaw_err) > 0.8:
                raw_lin = 0.0
            elif abs(yaw_err) > 0.3:
                raw_lin *= 0.5

            raw_lin = max(0.0, min(MAX_LIN, raw_lin))
            raw_ang = max(-MAX_ANG, min(MAX_ANG, raw_ang))
            raw_lin = self._ramp(raw_lin, dt)
            lin, ang = self._obstacle_adjust(raw_lin, raw_ang)

            cmd = Twist()
            cmd.linear.x  = lin
            cmd.angular.z = ang
            self.cmd_pub.publish(cmd)

        self.current_lin = 0.0
        self.cmd_pub.publish(Twist())

    def _drive_near(self, obj_x, obj_y, label='target'):
        dx   = obj_x - self.ox
        dy   = obj_y - self.oy
        dist = math.hypot(dx, dy)
        if dist < self.object_standoff:
            self.get_logger().info(f'Already near {label}.')
            return
        scale = (dist - self.object_standoff) / dist
        gx    = self.ox + dx * scale
        gy    = self.oy + dy * scale
        self.get_logger().info(
            f'odom-PID: driving to ({gx:.2f},{gy:.2f}) '
            f'[{self.object_clearance:.2f} m front clearance from {label}]')
        self._drive_to(gx, gy, tol=0.15, timeout=90.0)
        self.get_logger().info(f'Reached near {label}!')

    # ── Display ───────────────────────────────────────────────────────────────
    def _print_dict(self):
        self.get_logger().info('═══════ Detected Objects ═══════')
        if not self.object_dict:
            self.get_logger().info('  (none)')
            self._publish_status('Detected objects: none.')
        else:
            summary = []
            for i, (label, c) in enumerate(self.object_dict.items(), 1):
                self.get_logger().info(
                    f'  {i}. {label:30s}'
                    f'→ map({c["x"]:+.2f}, {c["y"]:+.2f}, z={c["z"]:+.2f})')
                summary.append(
                    f'{label}=({c["x"]:+.2f},{c["y"]:+.2f},{c["z"]:+.2f})')
            self._publish_status('Detected objects: ' + '; '.join(summary))
        self.get_logger().info('════════════════════════════════')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    ex   = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # safety stop
        node._publish_teleop_enabled(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
