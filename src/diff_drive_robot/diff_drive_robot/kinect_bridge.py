"""
kinect_bridge.py
================
Reads RGB and depth frames from a physical Kinect using the libfreenect
Python wrapper and republishes them as ROS 2 sensor_msgs on the same
topics that semantic_navigator subscribes to:

  /camera/image_raw         (sensor_msgs/msg/Image, bgr8)
  /camera/depth/image_raw   (sensor_msgs/msg/Image, 32FC1 in metres)
  /camera/camera_info       (sensor_msgs/msg/CameraInfo)

Usage (physical robot, after setting up libfreenect + Python wrapper):
  ros2 run diff_drive_robot kinect_bridge

This node replaces the Gazebo camera bridge when running on real hardware.
The semantic_navigator.py code remains UNCHANGED.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

# ── freenect import ────────────────────────────────────────────────────
try:
    import freenect
    FREENECT_OK = True
except ImportError:
    FREENECT_OK = False
    print("[kinect_bridge] WARNING: freenect not found. "
          "Install the Python wrapper from libfreenect/wrappers/python.")


# ── Kinect V1 / K4W factory calibration (640×480) ─────────────────────
# These are reasonable defaults; replace with your calibration if you have it.
FX = 525.0
FY = 525.0
CX = 319.5
CY = 239.5
WIDTH  = 640
HEIGHT = 480

# Kinect baseline: the RGB camera is ~25 mm to the right of the IR/depth camera.
# When mapping an RGB pixel (u,v) to the depth image we shift u left by this
# amount (in pixels at the nominal 525 focal length).
KINECT_BASELINE_SHIFT = 16   # pixels at 640×480; tune if needed

CAMERA_INFO_D = [0.0, 0.0, 0.0, 0.0, 0.0]          # distortion (none assumed)
CAMERA_INFO_K = [FX, 0.0, CX,
                 0.0, FY, CY,
                 0.0, 0.0, 1.0]
CAMERA_INFO_R = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
CAMERA_INFO_P = [FX, 0.0, CX, 0.0,
                 0.0, FY, CY, 0.0,
                 0.0, 0.0, 1.0, 0.0]


class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')
        self.bridge = CvBridge()

        self.rgb_pub   = self.create_publisher(Image,      '/camera/image_raw',       10)
        self.depth_pub = self.create_publisher(Image,      '/camera/depth/image_raw', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info',     10)

        # Publish at ~15 Hz to match simulation sensor update_rate
        self.timer = self.create_timer(1.0 / 15.0, self._publish_frame)

        if not FREENECT_OK:
            self.get_logger().error(
                "freenect Python module not installed! "
                "Run: cd ~/libfreenect/wrappers/python && sudo python setup.py install")
            return

        self.get_logger().info("KinectBridge started — publishing RGB + Depth at 15 Hz")

    # ── Camera Info (constant) ─────────────────────────────────────────
    def _make_camera_info(self, stamp: Time) -> CameraInfo:
        ci = CameraInfo()
        ci.header.stamp    = stamp
        ci.header.frame_id = 'camera_depth_optical_frame'
        ci.width  = WIDTH
        ci.height = HEIGHT
        ci.distortion_model = 'plumb_bob'
        ci.d = CAMERA_INFO_D
        ci.k = CAMERA_INFO_K
        ci.r = CAMERA_INFO_R
        ci.p = CAMERA_INFO_P
        return ci

    # ── Main publish callback ──────────────────────────────────────────
    def _publish_frame(self):
        if not FREENECT_OK:
            return

        now = self.get_clock().now().to_msg()

        # ── RGB frame ─────────────────────────────────────────────────
        try:
            rgb_data, _ = freenect.sync_get_video()
            # freenect returns (H, W, 3) uint8 in RGB order → convert to BGR for cv_bridge
            bgr = rgb_data[:, :, ::-1].astype(np.uint8)
            rgb_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            rgb_msg.header.stamp    = now
            rgb_msg.header.frame_id = 'camera_depth_optical_frame'
            self.rgb_pub.publish(rgb_msg)
        except Exception as e:
            self.get_logger().warn(f'RGB grab failed: {e}', throttle_duration_sec=5.0)

        # ── Depth frame ────────────────────────────────────────────────
        try:
            depth_raw, _ = freenect.sync_get_depth()   # uint16, 0-2047 raw disparity

            # ── Correct disparity → metres conversion for Kinect V1 ────
            # freenect returns 11-bit raw disparity (NOT millimetres).
            # Formula from the OpenKinect community calibration:
            #   depth_m = 1.0 / (raw * -0.0030711016 + 3.3309495161)
            # Values of 0 (no data) or 2047 (max/invalid) → NaN
            raw_f = depth_raw.astype(np.float32)
            depth_m = 1.0 / (raw_f * -0.0030711016 + 3.3309495161)
            depth_m[depth_raw == 0]    = np.nan   # no reading
            depth_m[depth_raw == 2047] = np.nan   # out of range
            depth_m[depth_m > 8.0]    = np.nan   # clip > 8 m
            depth_m[depth_m < 0.3]    = np.nan   # clip < 30 cm

            depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding='32FC1')
            depth_msg.header.stamp    = now
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().warn(f'Depth grab failed: {e}', throttle_duration_sec=5.0)

        # ── Camera info ────────────────────────────────────────────────
        self.info_pub.publish(self._make_camera_info(now))


def main(args=None):
    rclpy.init(args=args)
    node = KinectBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
