"""
kinect_depth_grid.py
====================
Project Kinect depth into a local 2D OccupancyGrid.

This is a simple live depth map, not full SLAM. It is useful for verifying that
the Kinect depth stream can create a 2D obstacle map before relying on RTAB-Map.
The grid is published in base_footprint coordinates:
  x = forward from the Kinect/robot
  y = left/right
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class KinectDepthGrid(Node):
    def __init__(self):
        super().__init__('kinect_depth_grid')

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_topic', '/kinect_depth_map')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('camera_height', 0.24)
        self.declare_parameter('camera_pitch', 0.0)
        self.declare_parameter('resolution', 0.07)
        self.declare_parameter('forward_min', 0.50)
        self.declare_parameter('forward_max', 3.50)
        self.declare_parameter('lateral_range', 2.50)
        self.declare_parameter('min_obstacle_height', 0.08)
        self.declare_parameter('max_obstacle_height', 1.40)
        self.declare_parameter('pixel_step', 6)
        self.declare_parameter('obstacle_min_points', 2)
        self.declare_parameter('raytrace_free_space', True)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_pitch = float(self.get_parameter('camera_pitch').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.forward_min = float(self.get_parameter('forward_min').value)
        self.forward_max = float(self.get_parameter('forward_max').value)
        self.lateral_range = float(self.get_parameter('lateral_range').value)
        self.min_obstacle_height = float(
            self.get_parameter('min_obstacle_height').value)
        self.max_obstacle_height = float(
            self.get_parameter('max_obstacle_height').value)
        self.pixel_step = max(1, int(self.get_parameter('pixel_step').value))
        self.obstacle_min_points = max(
            1, int(self.get_parameter('obstacle_min_points').value))
        self.raytrace_free_space = bool(
            self.get_parameter('raytrace_free_space').value)
        self._cos_pitch = math.cos(self.camera_pitch)
        self._sin_pitch = math.sin(self.camera_pitch)

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 319.5
        self.cy = 239.5
        self.have_camera_info = False

        self.width = max(
            1, int(math.ceil((self.forward_max - self.forward_min) /
                             self.resolution)))
        self.height = max(
            1, int(math.ceil((2.0 * self.lateral_range) / self.resolution)))

        self.pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.depth_topic, self._depth_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'Kinect depth grid publishing {self.map_topic} in {self.frame_id}: '
            f'{self.width}x{self.height} @ {self.resolution:.2f} m/cell, '
            f'camera_height={self.camera_height:.2f} m, '
            f'camera_pitch={self.camera_pitch:.3f} rad')

    def _info_cb(self, msg: CameraInfo):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])
            if not self.have_camera_info:
                self.get_logger().info(
                    f'CameraInfo received fx={self.fx:.1f}, fy={self.fy:.1f}, '
                    f'cx={self.cx:.1f}, cy={self.cy:.1f}')
            self.have_camera_info = True

    def _depth_cb(self, msg: Image):
        try:
            depth = self._depth_image_to_array(msg)
            grid = np.full((self.height, self.width), -1, dtype=np.int8)
            hit_counts = np.zeros((self.height, self.width), dtype=np.uint16)

            rows = np.arange(0, depth.shape[0], self.pixel_step)
            cols = np.arange(0, depth.shape[1], self.pixel_step)
            uu, vv = np.meshgrid(cols, rows)
            z = depth[vv, uu]

            x_right = (uu.astype(np.float32) - self.cx) * z / self.fx
            y_down = (vv.astype(np.float32) - self.cy) * z / self.fy

            # Optical frame: X right, Y down, Z forward.
            # Camera/base frame before pitch: X forward, Y left, Z up.
            cam_x = z
            cam_y = -x_right
            cam_z = -y_down

            # Positive camera_pitch means the sensor is tilted downward.
            forward = self._cos_pitch * cam_x + self._sin_pitch * cam_z
            height_above_ground = (
                self.camera_height
                - self._sin_pitch * cam_x
                + self._cos_pitch * cam_z)
            lateral = cam_y

            range_valid = np.isfinite(forward)
            range_valid &= np.isfinite(lateral)
            range_valid &= np.isfinite(height_above_ground)
            range_valid &= forward >= self.forward_min
            range_valid &= forward <= self.forward_max
            range_valid &= lateral >= -self.lateral_range
            range_valid &= lateral <= self.lateral_range

            obstacle_valid = range_valid.copy()
            obstacle_valid &= height_above_ground >= self.min_obstacle_height
            obstacle_valid &= height_above_ground <= self.max_obstacle_height

            if self.raytrace_free_space:
                for fwd, lat in zip(forward[range_valid].flat,
                                    lateral[range_valid].flat):
                    self._mark_free_ray(grid, float(fwd), float(lat))

            for fwd, lat in zip(forward[obstacle_valid].flat,
                                lateral[obstacle_valid].flat):
                cell_x, cell_y = self._world_to_cell(float(fwd), float(lat))
                if cell_x is not None:
                    hit_counts[cell_y, cell_x] += 1

            grid[hit_counts >= self.obstacle_min_points] = 100

            out = OccupancyGrid()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.frame_id
            out.info.map_load_time = self.get_clock().now().to_msg()
            out.info.resolution = self.resolution
            out.info.width = self.width
            out.info.height = self.height
            out.info.origin.position.x = self.forward_min
            out.info.origin.position.y = -self.lateral_range
            out.info.origin.position.z = 0.0
            out.info.origin.orientation.w = 1.0
            out.data = grid.reshape(-1).tolist()
            self.pub.publish(out)

        except Exception as exc:
            self.get_logger().warn(
                f'Kinect depth grid failed: {exc}', throttle_duration_sec=5.0)

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

    def _world_to_cell(self, forward: float, lateral: float):
        if forward < self.forward_min or forward > self.forward_max:
            return None, None
        if lateral < -self.lateral_range or lateral > self.lateral_range:
            return None, None

        cell_x = int((forward - self.forward_min) / self.resolution)
        cell_y = int((lateral + self.lateral_range) / self.resolution)
        if 0 <= cell_x < self.width and 0 <= cell_y < self.height:
            return cell_x, cell_y
        return None, None

    def _mark_free_ray(self, grid, forward: float, lateral: float):
        distance = math.hypot(forward, lateral)
        steps = max(1, int(distance / self.resolution))
        for i in range(steps):
            ratio = float(i) / float(steps)
            cell_x, cell_y = self._world_to_cell(
                forward * ratio, lateral * ratio)
            if cell_x is not None and grid[cell_y, cell_x] != 100:
                grid[cell_y, cell_x] = 0


def main(args=None):
    rclpy.init(args=args)
    node = KinectDepthGrid()
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
