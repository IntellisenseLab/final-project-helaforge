"""
kinect_bridge.py
================
Reads RGB and depth frames from a physical Kinect using the libfreenect
Python wrapper and republishes them as ROS 2 sensor_msgs on the same
topics that semantic_navigator subscribes to:

  /camera/image_raw         (sensor_msgs/msg/Image, bgr8)
  /camera/depth/image_raw   (sensor_msgs/msg/Image, 32FC1 in metres)
  /camera/camera_info       (sensor_msgs/msg/CameraInfo)

FIX: Added robust retry loop on LIBUSB_ERROR_BUSY (caused by the gspca_kinect
kernel module grabbing the USB interface before freenect).

Pre-requisite (run ONCE before launching):
  sudo modprobe -r gspca_kinect gspca_main 2>/dev/null
  echo 'blacklist gspca_kinect' | sudo tee /etc/modprobe.d/kinect.conf
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

# ── freenect import ────────────────────────────────────────────────────
try:
    # The underlying library module name is 'freenect', not 'libfreenect'
    import freenect
    FREENECT_OK = True
except ImportError:
    FREENECT_OK = False
    print("[kinect_bridge] WARNING: freenect module not found. "
          "Install: cd ~/libfreenect/wrappers/python && sudo python setup.py install")


# ── Kinect V1 / K4W factory calibration (640×480) ─────────────────────
FX = 525.0
FY = 525.0
CX = 319.5
CY = 239.5
WIDTH  = 640
HEIGHT = 480

CAMERA_INFO_D = [0.0, 0.0, 0.0, 0.0, 0.0]
CAMERA_INFO_K = [FX, 0.0, CX,
                 0.0, FY, CY,
                 0.0, 0.0, 1.0]
CAMERA_INFO_R = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
CAMERA_INFO_P = [FX, 0.0, CX, 0.0,
                 0.0, FY, CY, 0.0,
                 0.0, 0.0, 1.0, 0.0]

# How long to wait between retries when the USB device is busy
RETRY_DELAY_S  = 2.0
MAX_RETRIES    = 30        # ~60 s total before giving up


class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')
        self.bridge = CvBridge()
        self._device_ready = False
        self._retry_count  = 0

        self.rgb_pub   = self.create_publisher(Image,      '/camera/image_raw',       10)
        self.depth_pub = self.create_publisher(Image,      '/camera/depth/image_raw', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/camera_info',     10)

        if not FREENECT_OK:
            self.get_logger().error(
                "freenect Python module not installed! "
                "Run: cd ~/libfreenect/wrappers/python && sudo python setup.py install")
            return

        # Try an initial sync grab to verify the device is accessible
        self._try_init()

        # Publish at ~15 Hz
        self.timer = self.create_timer(1.0 / 15.0, self._publish_frame)

    # ── Initial device probe (with retry) ─────────────────────────────
    def _try_init(self):
        """
        freenect.sync_get_video() will raise if the USB device is busy
        (LIBUSB_ERROR_BUSY). This happens when the gspca_kinect kernel
        module has claimed the interface. We retry for up to MAX_RETRIES
        cycles and tell the user what to do.
        """
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                result = freenect.sync_get_video(0)   # index 0 = first Kinect
                # freenect returns None (not an exception) when device can't be opened.
                if result is None:
                    self.get_logger().warn(
                        f"[attempt {attempt}/{MAX_RETRIES}] Kinect returned None — "
                        "device not accessible.\n"
                        "  → Ensure the Kinect is unplugged and replugged AFTER "
                        "running: sudo udevadm trigger\n"
                        "  → Also check: ls -la /dev/bus/usb/001/ | grep '045e'\n"
                        "  → Or try:     sudo python3 -c \"import freenect; "
                        "print(freenect.sync_get_video(0))\"\n"
                        "  Retrying in 2 s …",
                        throttle_duration_sec=10.0)
                    time.sleep(RETRY_DELAY_S)
                    continue
                
                self._device_ready = True
                self.get_logger().info(
                    "KinectBridge started — publishing RGB + Depth at 15 Hz")
                return
            except Exception as e:
                err = str(e)
                if 'BUSY' in err or 'Invalid index' in err or "Can't open" in err or 'init' in err.lower():
                    self.get_logger().warn(
                        f"[attempt {attempt}/{MAX_RETRIES}] Kinect USB busy or uninitialized: {err}\n"
                        "  → Fix: run these commands in a terminal, then relaunch:\n"
                        "      sudo modprobe -r gspca_kinect gspca_main\n"
                        "      sudo sh -c \"echo 'blacklist gspca_kinect' > "
                        "/etc/modprobe.d/kinect.conf\"\n"
                        "  Retrying in 2 s …",
                        throttle_duration_sec=10.0)
                    time.sleep(RETRY_DELAY_S)
                else:
                    self.get_logger().error(f"Kinect init failed with unexpected error: {e}")
                    return
        
        self.get_logger().error(
            "Kinect: gave up after 30 retries. "
            "Make sure the Kinect USB is replugged after udev rules were applied, "
            "and /dev/bus/usb permissions allow non-root access.")

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

        if not self._device_ready:
            # Still retrying in the background
            self._retry_count += 1
            if self._retry_count % 30 == 0:   # every ~2 s at 15 Hz
                self._try_init()
            return

        now = self.get_clock().now().to_msg()

        # ── RGB frame ─────────────────────────────────────────────────
        try:
            result = freenect.sync_get_video(0)
            if result is None:
                self.get_logger().warn(
                    'RGB grab returned None — device lost, triggering re-init',
                    throttle_duration_sec=5.0)
                self._device_ready = False   # trigger re-init on next tick
                return
            rgb_data, _ = result
            # freenect returns (H, W, 3) uint8 in RGB → convert to BGR for cv_bridge
            bgr = rgb_data[:, :, ::-1].astype(np.uint8)
            rgb_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            rgb_msg.header.stamp    = now
            rgb_msg.header.frame_id = 'camera_depth_optical_frame'
            self.rgb_pub.publish(rgb_msg)
        except Exception as e:
            self.get_logger().warn(f'RGB grab failed: {e}', throttle_duration_sec=5.0)
            self._device_ready = False   # trigger re-init on next tick

        # ── Depth frame ────────────────────────────────────────────────
        try:
            depth_result = freenect.sync_get_depth(0)   # uint16, 0-2047 raw disparity
            if depth_result is None:
                self.get_logger().warn(
                    'Depth grab returned None — skipping frame',
                    throttle_duration_sec=5.0)
                return
            depth_raw, _ = depth_result

            # Kinect V1 raw disparity → metres
            raw_f   = depth_raw.astype(np.float32)
            depth_m = 1.0 / (raw_f * -0.0030711016 + 3.3309495161)
            depth_m[depth_raw == 0]    = np.nan   # no reading
            depth_m[depth_raw == 2047] = np.nan   # out of range / invalid
            depth_m[depth_m > 4.5]    = np.nan   # clip > 4.5 m (Kinect V1 practical max)
            depth_m[depth_m < 0.3]    = np.nan   # clip < 30 cm (min range)

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
        # Cleanly shut down freenect sync so the next launch doesn't get BUSY
        if FREENECT_OK:
            try:
                # Need to use the globally scoped module variable
                freenect.sync_stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()