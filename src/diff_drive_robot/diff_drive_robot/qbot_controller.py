#!/usr/bin/env python3

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from interfaces.action import Navigation
from interfaces.msg import Position
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class QbotControllerNode(Node):
    """Friend-project pure-pursuit controller adapted to publish /cmd_vel."""

    def __init__(self):
        super().__init__('qbot_controller')

        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('max_angular_speed', 0.45)
        self.declare_parameter('pose_topic', '/slam_pose')
        self.declare_parameter('goal_topic', '/ui_goal')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value)
        pose_topic = self.get_parameter('pose_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.current_yaw = 0.0
        self.goal_done = True
        self.mode = 'idle'
        self._goal_handle = None
        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, pose_topic, self.update_yaw_from_odom, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            Point, goal_topic, self.on_ui_goal, 10,
            callback_group=self._cb_group)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '/qbot_nav/status', 10)
        self._action_client = ActionClient(
            self, Navigation, 'navigate', callback_group=self._cb_group)

        self.get_logger().info(
            'QBot LiDAR controller ready '
            f'(goal={goal_topic}, cmd={cmd_vel_topic}, pose={pose_topic})')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def update_yaw_from_odom(self, msg):
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_ui_goal(self, msg):
        self.publish_status(f'UI goal received: ({msg.x:.2f}, {msg.y:.2f})')
        if self.mode == 'navigating' and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        self.mode = 'navigating'
        self.goal_done = False
        self._send_goal(msg.x, msg.y)

    def _send_goal(self, x, y):
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('A* navigation action server is not ready.')
            self.mode = 'idle'
            self.goal_done = True
            return

        goal_msg = Navigation.Goal()
        position = Position()
        position.x = float(x)
        position.y = float(y)
        goal_msg.end_position = position

        self.publish_status(f'A* goal sent: ({x:.2f}, {y:.2f})')
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.handle_navigation_feedback)
        future.add_done_callback(self.handle_navigation_goal_response)

    def handle_navigation_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.publish_status('A* goal rejected.')
                self.mode = 'idle'
                self.goal_done = True
                return
            self.publish_status('A* goal accepted.')
            self._goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.handle_navigation_result)
        except Exception as exc:
            self.publish_status(f'A* goal error: {exc}')
            self.mode = 'idle'
            self.goal_done = True

    def handle_navigation_result(self, future):
        self.goal_done = True
        self.mode = 'idle'
        self._goal_handle = None
        self.cmd_pub.publish(Twist())

        try:
            result = future.result().result
            if result.success:
                self.publish_status('A* goal reached.')
            else:
                self.publish_status('A* goal failed.')
        except Exception as exc:
            self.publish_status(f'A* result error: {exc}')

    def handle_navigation_feedback(self, feedback_msg):
        if self.goal_done or self.mode != 'navigating':
            return

        direction = feedback_msg.feedback.direction
        yaw_error = self.angle_diff(direction, self.current_yaw)
        abs_yaw_error = abs(yaw_error)
        speed_factor = max(0.2, 1.0 - (abs_yaw_error / 1.57))

        cmd = Twist()
        cmd.linear.x = self.linear_speed * speed_factor
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, yaw_error))
        self.cmd_pub.publish(cmd)

    @staticmethod
    def angle_diff(desired, current):
        diff = desired - current
        return math.atan2(math.sin(diff), math.cos(diff))


def main(args=None):
    rclpy.init(args=args)
    node = QbotControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
