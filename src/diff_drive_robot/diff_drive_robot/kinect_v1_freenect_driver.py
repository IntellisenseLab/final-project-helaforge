"""
kinect_v1_freenect_driver.py
============================
Small ROS 2 publisher for the connected Kinect v1 / Xbox 360 RGB-D sensor.

This is used when the sensor appears as:
  045e:02c2 Kinect motor
  045e:02ad Kinect audio
  045e:02ae Kinect camera

The node reads frames through the Python libfreenect binding and publishes
OpenNI-style topics. kinect_topic_bridge then normalizes these for the rest of
the project:

  /camera/rgb/image_raw
  /camera/depth_registered/image_raw
  /camera/rgb/camera_info
"""

import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    import freenect
except ImportError:
    freenect = None


class KinectV1FreenectDriver(Node):
    def __init__(self):
        super().__init__('kinect_v1_freenect_driver')

        if freenect is None:
            self.get_logger().fatal(
                'Python module "freenect" is not installed. Install it with:\n'
                '  sudo apt install -y freenect libfreenect-dev libfreenect-bin\n'
                '  pip3 install freenect --break-system-packages')
            raise SystemExit(1)

        self.declare_parameter('device_index', 0)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('rgb_topic', '/camera/rgb/image_raw')
        self.declare_parameter(
            'depth_topic', '/camera/depth_registered/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/rgb/camera_info')
        self.declare_parameter('frame_id', 'camera_rgb_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)
        self.declare_parameter('depth_format', 'registered')
        self.declare_parameter('max_consecutive_failures', 5)

        self.device_index = int(self.get_parameter('device_index').value)
        fps = float(self.get_parameter('fps').value)
        self.max_consecutive_failures = int(
            self.get_parameter('max_consecutive_failures').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.depth_format_name = str(
            self.get_parameter('depth_format').value).lower()
        self.depth_format = self._resolve_depth_format(self.depth_format_name)

        self.rgb_pub = self.create_publisher(
            Image, rgb_topic, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            Image, depth_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, info_topic, qos_profile_sensor_data)

        self.period = 1.0 / max(fps, 1.0)
        self.consecutive_failures = 0
        self.timer = self.create_timer(self.period, self._publish_frame)

        self.get_logger().info(
            'Kinect v1 freenect driver ready:\n'
            f'  Device index: {self.device_index}\n'
            f'  RGB   -> {rgb_topic}\n'
            f'  Depth -> {depth_topic}\n'
            f'  Info  -> {info_topic}\n'
            f'  Frame -> {self.frame_id}\n'
            f'  Depth format: {self.depth_format_name}')
        self.get_logger().info(
            'If frames do not appear and libfreenect reports LIBUSB_ERROR_BUSY, '
            'close other Kinect programs and reset/replug the Kinect USB cable.')

    def _resolve_depth_format(self, name: str):
        if name in {'registered', 'depth_registered'}:
            if hasattr(freenect, 'DEPTH_REGISTERED'):
                return freenect.DEPTH_REGISTERED
            self.get_logger().warn(
                'freenect.DEPTH_REGISTERED is unavailable; using DEPTH_MM')

        if name in {'registered', 'depth_registered', 'mm', 'depth_mm'}:
            if hasattr(freenect, 'DEPTH_MM'):
                return freenect.DEPTH_MM
            self.get_logger().warn(
                'freenect.DEPTH_MM is unavailable; using DEPTH_11BIT')

        return getattr(freenect, 'DEPTH_11BIT')

    def _camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        info.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def _publish_frame(self):
        try:
            video_ret = freenect.sync_get_video(
                index=self.device_index, format=freenect.VIDEO_RGB)
            depth_ret = freenect.sync_get_depth(
                index=self.device_index, format=self.depth_format)

            if video_ret is None or depth_ret is None:
                self.get_logger().warn(
                    'No Kinect frames received yet', throttle_duration_sec=5.0)
                return

            rgb, _ = video_ret
            depth, _ = depth_ret

            stamp = self.get_clock().now().to_msg()
            rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
            depth_m = self._depth_to_meters(depth)

            self.rgb_pub.publish(self._image_msg(rgb, 'rgb8', stamp))
            self.depth_pub.publish(self._image_msg(depth_m, '32FC1', stamp))
            self.info_pub.publish(self._camera_info(stamp))
            self.consecutive_failures = 0

        except Exception as exc:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().fatal(
                    f'Kinect v1 frame read failed {self.consecutive_failures} '
                    f'times on device_index={self.device_index}: {exc}\n'
                    'The Kinect camera is not visible to libfreenect or is busy.\n'
                    'Check: lsusb | grep -i microsoft, freenect-glview, '
                    'USB power, direct USB port, and close other Kinect nodes.')
                self.timer.cancel()
                if hasattr(freenect, 'sync_stop'):
                    freenect.sync_stop()
                return

            self.get_logger().warn(
                f'Kinect v1 frame read failed '
                f'({self.consecutive_failures}/'
                f'{self.max_consecutive_failures}): {exc}',
                throttle_duration_sec=2.0)

    def _image_msg(self, array: np.ndarray, encoding: str, stamp):
        array = np.ascontiguousarray(array)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = int(array.strides[0])
        msg.data = array.tobytes()
        return msg

    def _depth_to_meters(self, depth):
        depth = np.asarray(depth)
        depth_f = depth.astype(np.float32)

        if self.depth_format_name in {
                'registered', 'depth_registered', 'mm', 'depth_mm'}:
            depth_f *= 0.001
        else:
            valid = depth_f > 0.0
            depth_m = np.full(depth_f.shape, np.nan, dtype=np.float32)
            depth_m[valid] = 1.0 / (-0.0030711016 * depth_f[valid] +
                                    3.3309495161)
            depth_f = depth_m

        depth_f[~np.isfinite(depth_f)] = np.nan
        depth_f[depth_f <= 0.0] = np.nan
        return np.ascontiguousarray(depth_f, dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinectV1FreenectDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit as exc:
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(exc.code)
    finally:
        if node is not None:
            node.destroy_node()
        if freenect is not None and hasattr(freenect, 'sync_stop'):
            freenect.sync_stop()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
