"""
kinect_topic_bridge.py
======================
Normalizes topics from a Kinect v1/libfreenect or OpenNI RGB-D driver.

The camera driver should publish registered RGB-D data. For the connected
Kinect v1, this normally comes from kinect_v1_freenect_driver. This node
republishes those streams on the stable topics consumed by RTAB-Map, Nav2 and
the Python semantic nodes:

  /camera/image_raw
  /camera/depth/image_raw
  /camera/camera_info

Depth is converted to 32FC1 metres when the driver publishes 16UC1 millimetres.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class KinectTopicBridge(Node):
    def __init__(self):
        super().__init__('kinect_topic_bridge')

        self.declare_parameter('source_rgb_topic', '/camera/rgb/image_raw')
        self.declare_parameter(
            'source_depth_topic', '/camera/depth_registered/image_raw')
        self.declare_parameter('source_camera_info_topic', '/camera/rgb/camera_info')
        self.declare_parameter('target_rgb_topic', '/camera/image_raw')
        self.declare_parameter('target_depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('target_camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_frame_id', 'camera_rgb_optical_frame')

        self.source_rgb_topic = self.get_parameter('source_rgb_topic').value
        self.source_depth_topic = self.get_parameter('source_depth_topic').value
        self.source_camera_info_topic = self.get_parameter(
            'source_camera_info_topic').value
        self.target_rgb_topic = self.get_parameter('target_rgb_topic').value
        self.target_depth_topic = self.get_parameter('target_depth_topic').value
        self.target_camera_info_topic = self.get_parameter(
            'target_camera_info_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE)

        self.rgb_pub = self.create_publisher(
            Image, self.target_rgb_topic, reliable_qos)
        self.depth_pub = self.create_publisher(
            Image, self.target_depth_topic, reliable_qos)
        self.info_pub = self.create_publisher(
            CameraInfo, self.target_camera_info_topic, reliable_qos)

        self.create_subscription(
            Image, self.source_rgb_topic, self._rgb_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.source_depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.source_camera_info_topic, self._info_cb,
            qos_profile_sensor_data)

        self.get_logger().info(
            'Kinect topic bridge ready:\n'
            f'  RGB   {self.source_rgb_topic} -> {self.target_rgb_topic}\n'
            f'  Depth {self.source_depth_topic} -> {self.target_depth_topic}\n'
            f'  Info  {self.source_camera_info_topic} -> '
            f'{self.target_camera_info_topic}\n'
            f'  Output frame: {self.output_frame_id}')

    def _set_header(self, out_msg, in_header):
        out_msg.header.stamp = in_header.stamp
        out_msg.header.frame_id = self.output_frame_id or in_header.frame_id

    def _rgb_cb(self, msg: Image):
        try:
            out = Image()
            self._set_header(out, msg.header)
            out.height = msg.height
            out.width = msg.width
            out.encoding = msg.encoding
            out.is_bigendian = msg.is_bigendian
            out.step = msg.step
            out.data = bytes(msg.data)
            self.rgb_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(
                f'RGB bridge failed: {e}', throttle_duration_sec=5.0)

    def _depth_cb(self, msg: Image):
        try:
            depth = self._depth_image_to_array(msg)

            finite = depth[np.isfinite(depth) & (depth > 0.0)]
            median = float(np.median(finite)) if finite.size else math.nan
            if msg.encoding.lower() in {'16uc1', 'mono16'} or (
                    math.isfinite(median) and median > 20.0):
                depth *= 0.001

            depth[~np.isfinite(depth)] = np.nan
            depth[depth <= 0.0] = np.nan
            depth = np.ascontiguousarray(depth, dtype=np.float32)

            out = Image()
            self._set_header(out, msg.header)
            out.height = int(depth.shape[0])
            out.width = int(depth.shape[1])
            out.encoding = '32FC1'
            out.is_bigendian = 0
            out.step = out.width * 4
            out.data = depth.tobytes()
            self.depth_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(
                f'Depth bridge failed: {e}', throttle_duration_sec=5.0)

    def _depth_image_to_array(self, msg: Image):
        encoding = msg.encoding.lower()
        if encoding == '32fc1':
            dtype = np.dtype(np.float32)
        elif encoding in {'16uc1', 'mono16'}:
            dtype = np.dtype(np.uint16)
        else:
            raise ValueError(f'unsupported depth encoding: {msg.encoding}')

        if msg.is_bigendian:
            dtype = dtype.newbyteorder('>')
        else:
            dtype = dtype.newbyteorder('<')

        item_size = dtype.itemsize
        row_items = msg.step // item_size
        raw = np.frombuffer(bytes(msg.data), dtype=dtype)
        image = raw.reshape(msg.height, row_items)[:, :msg.width]
        return image.astype(np.float32)

    def _info_cb(self, msg: CameraInfo):
        out = CameraInfo()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id or msg.header.frame_id
        out.height = msg.height
        out.width = msg.width
        out.distortion_model = msg.distortion_model
        out.d = list(msg.d)
        out.k = list(msg.k)
        out.r = list(msg.r)
        out.p = list(msg.p)
        out.binning_x = msg.binning_x
        out.binning_y = msg.binning_y
        out.roi = msg.roi
        self.info_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = KinectTopicBridge()
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
