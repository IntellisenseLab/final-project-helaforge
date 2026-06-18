"""
depth_preview.py
================
Convert a floating-point depth image into a display-friendly mono8 image.

RTAB-Map should receive the real /camera/depth/image_raw topic as 32FC1 metres.
This node only creates /camera/depth/preview for image_view.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthPreview(Node):
    def __init__(self):
        super().__init__('depth_preview')

        self.declare_parameter('input_topic', '/camera/depth/image_raw')
        self.declare_parameter('output_topic', '/camera/depth/preview')
        self.declare_parameter('min_depth', 0.5)
        self.declare_parameter('max_depth', 4.0)
        self.declare_parameter('invert', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.invert = bool(self.get_parameter('invert').value)

        self.pub = self.create_publisher(
            Image, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.input_topic, self._depth_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'Depth preview: {self.input_topic} -> {self.output_topic} '
            f'[{self.min_depth:.2f}m, {self.max_depth:.2f}m]')

    def _depth_cb(self, msg: Image):
        try:
            depth = self._depth_image_to_array(msg)
            preview = self._depth_to_mono8(depth)

            out = Image()
            out.header = msg.header
            out.height = int(preview.shape[0])
            out.width = int(preview.shape[1])
            out.encoding = 'mono8'
            out.is_bigendian = 0
            out.step = out.width
            out.data = preview.tobytes()
            self.pub.publish(out)
        except Exception as exc:
            self.get_logger().warn(
                f'Depth preview failed: {exc}', throttle_duration_sec=5.0)

    def _depth_image_to_array(self, msg: Image):
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
        depth = raw.reshape(msg.height, row_items)[:, :msg.width]
        depth = depth.astype(np.float32)

        if encoding in {'16uc1', 'mono16'}:
            depth *= 0.001
        return depth

    def _depth_to_mono8(self, depth: np.ndarray):
        min_d = self.min_depth
        max_d = max(self.max_depth, min_d + 0.01)

        valid = np.isfinite(depth) & (depth >= min_d) & (depth <= max_d)
        clipped = np.clip(depth, min_d, max_d)

        if self.invert:
            scaled = (max_d - clipped) / (max_d - min_d)
        else:
            scaled = (clipped - min_d) / (max_d - min_d)

        preview = np.zeros(depth.shape, dtype=np.uint8)
        preview[valid] = np.asarray(scaled[valid] * 255.0, dtype=np.uint8)
        return np.ascontiguousarray(preview)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPreview()
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
