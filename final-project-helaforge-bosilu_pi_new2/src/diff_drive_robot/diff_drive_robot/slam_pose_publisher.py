#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener


class SlamPosePublisher(Node):
    """Publishes map-frame robot pose as /slam_pose for the A* controller."""

    def __init__(self):
        super().__init__('slam_pose_publisher')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(Odometry, '/slam_pose', 10)
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().info(
            f'Publishing SLAM pose {self.map_frame}->{self.base_frame} on /slam_pose')

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPosePublisher()
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
