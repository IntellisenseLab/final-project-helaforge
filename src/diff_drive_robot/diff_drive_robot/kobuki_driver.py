"""
kobuki_driver.py
================
ROS 2 node that bridges a physical Kobuki robot over its serial port.

Based on the kobuki-python library from https://github.com/SudilMin/kobuki-python
which provides the Kobuki serial packet structure. This node adds the missing
implementations: velocity command sending and encoder-based odometry.

Subscribes:
  /cmd_vel  (geometry_msgs/Twist) → converts to Kobuki BaseControl serial command

Publishes:
  /odom     (nav_msgs/Odometry)   → wheel encoder odometry
  TF        odom → base_footprint  → so Nav2 / semantic_navigator can localize
  /kobuki/encoder_debug (std_msgs/String) → raw encoder odometry diagnostics

Usage:
  ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0
  ros2 run diff_drive_robot kobuki_driver --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p wheel_separation:=0.230 \
    -p wheel_diameter:=0.070
"""

import glob
import math
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros

try:
    import serial
except ImportError:
    raise ImportError("Run: pip3 install pyserial --break-system-packages")


# ── Kobuki Hardware Constants ──────────────────────────────────────────────────
WHEEL_DIAMETER  = 0.070          # 70 mm
WHEEL_SEP       = 0.230          # 230 mm between wheels
TICKS_PER_REV   = 2578.33        # encoder ticks per wheel revolution
MAX_SPEED_MM    = 500            # hardware max: 500 mm/s

# ── Kobuki Serial Protocol ─────────────────────────────────────────────────────
HEADER_0        = 0xAA
HEADER_1        = 0x55
CMD_BASE_CTRL   = 0x01           # BaseControl sub-payload ID
CMD_BASE_LEN    = 0x04           # 4 bytes: speed(int16) + radius(int16)
FEEDBACK_BASIC  = 0x01           # BasicSensorData sub-payload ID (feedback)

STRAIGHT_RADIUS = 0x7FFF         # special value = no curve, go straight


def _checksum(payload: bytes) -> int:
    cs = len(payload)
    for b in payload:
        cs ^= b
    return cs & 0xFF


def _build_velocity_packet(linear_x: float, angular_z: float,
                           wheel_separation: float = WHEEL_SEP,
                           max_speed_mm: int = MAX_SPEED_MM) -> bytes:
    """
    Convert Twist to Kobuki BaseControl serial packet.
    Protocol from kobuki-python / kobuki.readthedocs.io
    """
    v = linear_x           # m/s
    w = angular_z          # rad/s

    if abs(v) < 0.001 and abs(w) < 0.001:
        speed_mm  = 0
        radius_mm = 0
    elif abs(v) < 0.001:          # pure rotation in place
        speed_mm  = int(abs(w) * (wheel_separation / 2.0) * 1000.0)
        radius_mm = 1 if w > 0 else -1
    elif abs(w) < 0.001:          # straight line
        speed_mm  = int(v * 1000.0)
        radius_mm = STRAIGHT_RADIUS
    else:                          # curved motion
        radius_m  = v / w
        speed_mm  = int(v * 1000.0)
        radius_mm = int(radius_m * 1000.0)

    # Clamp speed to hardware limit
    speed_mm = max(-max_speed_mm, min(max_speed_mm, speed_mm))
    # Clamp radius to int16 range
    radius_mm = max(-32768, min(32767, radius_mm))

    sub_payload = struct.pack('<BBhh', CMD_BASE_CTRL, CMD_BASE_LEN,
                              speed_mm, radius_mm)
    cs = _checksum(sub_payload)
    packet = bytes([HEADER_0, HEADER_1, len(sub_payload)]) + sub_payload + bytes([cs])
    return packet


def _uint16_from_bytes(lo: int, hi: int) -> int:
    return (hi << 8) | lo


def _auto_kobuki_port(requested: str) -> str:
    if requested and requested != 'auto' and os.path.exists(requested):
        return requested

    patterns = [
        '/dev/serial/by-id/usb-Yujin_Robot_iClebo_Kobuki_kobuki_*-if00-port0',
        '/dev/serial/by-id/*Kobuki*',
    ]
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    return requested if requested and requested != 'auto' else '/dev/ttyUSB0'


class KobukiDriver(Node):
    def __init__(self):
        super().__init__('kobuki_driver')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('wheel_diameter', WHEEL_DIAMETER)
        self.declare_parameter('wheel_separation', WHEEL_SEP)
        self.declare_parameter('ticks_per_rev', TICKS_PER_REV)
        self.declare_parameter('max_speed_mm', MAX_SPEED_MM)
        self.declare_parameter('left_wheel_scale', 1.0)
        self.declare_parameter('right_wheel_scale', 1.0)
        self.declare_parameter('swap_encoders', False)
        self.declare_parameter('invert_left_encoder', False)
        self.declare_parameter('invert_right_encoder', False)
        self.declare_parameter('straight_correction_enabled', True)
        self.declare_parameter('straight_correction_angular_threshold', 0.02)
        self.declare_parameter('straight_correction_delta_threshold', 0.20)
        requested_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        port = _auto_kobuki_port(requested_port)
        if requested_port != port:
            self.get_logger().warn(
                f'Kobuki serial port "{requested_port}" not available; '
                f'using "{port}".')
        self.wheel_diameter = float(self.get_parameter('wheel_diameter').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.max_speed_mm = int(self.get_parameter('max_speed_mm').value)
        self.left_wheel_scale = float(
            self.get_parameter('left_wheel_scale').value)
        self.right_wheel_scale = float(
            self.get_parameter('right_wheel_scale').value)
        self.swap_encoders = bool(self.get_parameter('swap_encoders').value)
        self.invert_left_encoder = bool(
            self.get_parameter('invert_left_encoder').value)
        self.invert_right_encoder = bool(
            self.get_parameter('invert_right_encoder').value)
        self.straight_correction_enabled = bool(
            self.get_parameter('straight_correction_enabled').value)
        self.straight_correction_angular_threshold = float(
            self.get_parameter('straight_correction_angular_threshold').value)
        self.straight_correction_delta_threshold = float(
            self.get_parameter('straight_correction_delta_threshold').value)
        self.ticks_per_meter = (
            self.ticks_per_rev / (math.pi * self.wheel_diameter))

        # ── State ─────────────────────────────────────────────────────────────
        self.x    = 0.0
        self.y    = 0.0
        self.yaw  = 0.0
        self.prev_left  = None
        self.prev_right = None
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_debug_time = 0.0
        self._lock = threading.Lock()

        # ── Serial connection (kobuki-python style: 115200 baud) ──────────────
        try:
            self.ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
            self.get_logger().info(f'Kobuki connected on {port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            self.get_logger().fatal(
                'Tip: check port with "ls /dev/ttyUSB* /dev/ttyACM*"')
            raise

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.encoder_debug_pub = self.create_publisher(
            String, '/kobuki/encoder_debug', 10)
        self.tf_bcast = tf2_ros.TransformBroadcaster(self)

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, best_effort_qos)

        # ── Background thread: continuously reads sensor feedback ─────────────
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._read_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            'KobukiDriver ready.\n'
            '  Subscribing to /cmd_vel\n'
            '  Publishing  /odom + TF(odom→base_footprint)\n'
            '  Publishing  /kobuki/encoder_debug\n'
            f'  wheel_diameter={self.wheel_diameter:.4f} m, '
            f'wheel_separation={self.wheel_separation:.4f} m, '
            f'ticks_per_meter={self.ticks_per_meter:.1f}\n'
            f'  left_wheel_scale={self.left_wheel_scale:.3f}, '
            f'right_wheel_scale={self.right_wheel_scale:.3f}, '
            f'swap_encoders={self.swap_encoders}, '
            f'invert_left={self.invert_left_encoder}, '
            f'invert_right={self.invert_right_encoder}')

    # ── cmd_vel callback ───────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z
        packet = _build_velocity_packet(
            msg.linear.x, msg.angular.z,
            self.wheel_separation, self.max_speed_mm)
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial read loop (runs in background thread) ───────────────────────────
    def _read_loop(self):
        """Reads Kobuki feedback packets and extracts encoder data."""
        buf = bytearray()
        while self._running:
            try:
                buf += self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                time.sleep(0.01)
                continue

            # Scan for 0xAA 0x55 header (kobuki-python protocol)
            while len(buf) >= 4:
                if buf[0] != HEADER_0 or buf[1] != HEADER_1:
                    buf.pop(0)
                    continue

                payload_len = buf[2]
                total = 3 + payload_len + 1       # header(3) + payload + checksum
                if len(buf) < total:
                    break

                payload  = buf[3:3 + payload_len]
                checksum = buf[3 + payload_len]

                if _checksum(payload) != checksum:
                    buf.pop(0)
                    continue

                # Valid packet — parse sub-payloads
                self._parse_feedback(bytes(payload))
                buf = buf[total:]

    def _parse_feedback(self, payload: bytes):
        """
        Parse Kobuki feedback sub-payloads.
        SubPayloadSchemas from kobuki-python defines the structure.
        BasicSensorData (ID=0x01) contains LeftEncoder + RightEncoder.
        """
        i = 0
        while i + 1 < len(payload):
            sub_id  = payload[i]
            sub_len = payload[i + 1]
            data    = payload[i + 2: i + 2 + sub_len]
            i      += 2 + sub_len

            if sub_id == FEEDBACK_BASIC and len(data) >= 9:
                # BasicSensorData layout (from SubPayloadSchemas.py):
                # [0-1]  TimeStamp   UShort
                # [2]    Bumper      Flag1B
                # [3]    WheelDrop   Flag1B
                # [4]    Cliff       Flag1B
                # [5-6]  LeftEncoder UShort  ← we need this
                # [7-8]  RightEncoder UShort ← and this
                # ... rest (PWM, Button, Charger, Battery, OverCurrent)
                left_enc  = _uint16_from_bytes(data[5], data[6])
                right_enc = _uint16_from_bytes(data[7], data[8])
                if self.swap_encoders:
                    left_enc, right_enc = right_enc, left_enc

                bumper = data[2]
                if bumper:
                    # Stop on bumper hit for safety
                    self.ser.write(_build_velocity_packet(
                        0.0, 0.0, self.wheel_separation, self.max_speed_mm))

                self._update_odom(left_enc, right_enc)

    def _update_odom(self, left_enc: int, right_enc: int):
        now = self.get_clock().now()

        with self._lock:
            if self.prev_left is None:
                self.prev_left  = left_enc
                self.prev_right = right_enc
                return

            # Handle 16-bit rollover (kobuki encoder wraps at 65535)
            dl = (left_enc  - self.prev_left)  & 0xFFFF
            dr = (right_enc - self.prev_right) & 0xFFFF
            if dl > 32767: dl -= 65536
            if dr > 32767: dr -= 65536

            if self.invert_left_encoder:
                dl = -dl
            if self.invert_right_encoder:
                dr = -dr

            self.prev_left  = left_enc
            self.prev_right = right_enc

            dl_m = (dl / self.ticks_per_meter) * self.left_wheel_scale
            dr_m = (dr / self.ticks_per_meter) * self.right_wheel_scale

            dc     = (dl_m + dr_m) / 2.0    # centre displacement
            dtheta = (dr_m - dl_m) / self.wheel_separation
            raw_dtheta = dtheta

            if self._should_force_straight(dl_m, dr_m):
                dtheta = 0.0

            self.x   += dc * math.cos(self.yaw + dtheta / 2.0)
            self.y   += dc * math.sin(self.yaw + dtheta / 2.0)
            self.yaw += dtheta

            # Wrap yaw to [-π, π]
            while self.yaw >  math.pi: self.yaw -= 2 * math.pi
            while self.yaw < -math.pi: self.yaw += 2 * math.pi

            self._publish_encoder_debug(
                left_enc, right_enc, dl, dr, dl_m, dr_m, dc,
                raw_dtheta, dtheta)

        # ── Publish /odom ──────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.odom_pub.publish(odom)

        # ── Broadcast TF: odom → base_footprint ───────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_footprint'
        tf_msg.transform.translation.x  = self.x
        tf_msg.transform.translation.y  = self.y
        tf_msg.transform.rotation.z     = math.sin(self.yaw / 2.0)
        tf_msg.transform.rotation.w     = math.cos(self.yaw / 2.0)
        self.tf_bcast.sendTransform(tf_msg)

    def _should_force_straight(self, dl_m: float, dr_m: float) -> bool:
        if not self.straight_correction_enabled:
            return False
        if abs(self.last_cmd_linear) < 0.01:
            return False
        if abs(self.last_cmd_angular) > self.straight_correction_angular_threshold:
            return False
        avg = (abs(dl_m) + abs(dr_m)) / 2.0
        if avg < 1e-5:
            return False
        mismatch = abs(dr_m - dl_m) / avg
        return mismatch <= self.straight_correction_delta_threshold

    def _publish_encoder_debug(self, left_enc, right_enc, dl, dr,
                               dl_m, dr_m, dc, raw_dtheta, used_dtheta):
        now_sec = time.time()
        if now_sec - self.last_debug_time < 0.5:
            return
        self.last_debug_time = now_sec

        avg = (abs(dl_m) + abs(dr_m)) / 2.0
        mismatch = (abs(dr_m - dl_m) / avg) if avg > 1e-6 else 0.0
        msg = String()
        msg.data = (
            f'left_enc={left_enc} right_enc={right_enc} '
            f'dl_ticks={dl} dr_ticks={dr} '
            f'dl_m={dl_m:+.4f} dr_m={dr_m:+.4f} dc={dc:+.4f} '
            f'raw_dtheta={raw_dtheta:+.4f} used_dtheta={used_dtheta:+.4f} '
            f'yaw={self.yaw:+.3f} mismatch={mismatch:.2%} '
            f'cmd=({self.last_cmd_linear:+.2f},{self.last_cmd_angular:+.2f})')
        self.encoder_debug_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        # Send stop command before shutdown
        try:
            self.ser.write(_build_velocity_packet(
                0.0, 0.0, self.wheel_separation, self.max_speed_mm))
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KobukiDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[kobuki_driver] Fatal: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
