#!/usr/bin/env python3

import heapq
import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from interfaces.action import Navigation
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String


class AStarNavigationServer(Node):
    """Friend-project A* planner adapted for this project's frame names."""

    def __init__(self):
        super().__init__('qbot_navigation_server')

        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('inflation_radius', 3)
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('pose_topic', '/slam_pose')
        self.declare_parameter('map_topic', '/map')

        self.waypoint_tolerance = float(
            self.get_parameter('waypoint_tolerance').value)
        self.inflation_radius = int(
            self.get_parameter('inflation_radius').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value)
        pose_topic = self.get_parameter('pose_topic').value
        map_topic = self.get_parameter('map_topic').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.position_received = False

        self.map_data = []
        self.map_info = None
        self.freeze_map = False

        self._cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, pose_topic, self.odom_callback, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            OccupancyGrid, map_topic, self.map_callback, 10,
            callback_group=self._cb_group)

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.status_pub = self.create_publisher(String, '/qbot_nav/status', 10)

        self._action_server = ActionServer(
            self, Navigation, 'navigate', self.execute_callback,
            callback_group=self._cb_group)

        self.get_logger().info(
            'LiDAR A* Navigation Server started '
            f'(pose={pose_topic}, map={map_topic})')

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.position_received = True

    def map_callback(self, msg):
        if not self.freeze_map:
            self.map_info = msg.info
            self.map_data = msg.data

    def world_to_grid(self, wx, wy):
        if not self.map_info:
            return None
        gx = int((wx - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((wy - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        if not self.map_info:
            return None
        wx = (gx * self.map_info.resolution) + self.map_info.origin.position.x
        wy = (gy * self.map_info.resolution) + self.map_info.origin.position.y
        return wx, wy

    def is_valid_cell(self, gx, gy, width, height, grid_data):
        if gx < 0 or gx >= width or gy < 0 or gy >= height:
            return False

        for dy in range(-self.inflation_radius, self.inflation_radius + 1):
            for dx in range(-self.inflation_radius, self.inflation_radius + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    idx = ny * width + nx
                    if grid_data[idx] > 50:
                        return False
        return True

    @staticmethod
    def heuristic(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def calculate_astar_path(self, start_gx, start_gy, goal_gx, goal_gy):
        width = self.map_info.width
        height = self.map_info.height
        grid = self.map_data

        if not self.is_valid_cell(goal_gx, goal_gy, width, height, grid):
            self.get_logger().warn('Goal is too close to a mapped obstacle.')
            return []

        queue = []
        heapq.heappush(queue, (0, 0, start_gx, start_gy))
        g_costs = {(start_gx, start_gy): 0}
        came_from = {}
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (-1, 1), (1, -1), (-1, -1),
        ]
        cells_searched = 0

        while queue:
            _, current_g_cost, cx, cy = heapq.heappop(queue)
            cells_searched += 1

            if cx == goal_gx and cy == goal_gy:
                self.get_logger().info(
                    f'A* path found after searching {cells_searched} cells.')
                path = []
                while (cx, cy) in came_from:
                    path.append(self.grid_to_world(cx, cy))
                    cx, cy = came_from[(cx, cy)]
                path.reverse()
                return path

            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if self.is_valid_cell(nx, ny, width, height, grid):
                    step_cost = 1.414 if dx != 0 and dy != 0 else 1.0
                    new_g_cost = current_g_cost + step_cost

                    if (nx, ny) not in g_costs or new_g_cost < g_costs[(nx, ny)]:
                        g_costs[(nx, ny)] = new_g_cost
                        came_from[(nx, ny)] = (cx, cy)
                        h_cost = self.heuristic(nx, ny, goal_gx, goal_gy)
                        f_cost = new_g_cost + h_cost
                        heapq.heappush(queue, (f_cost, new_g_cost, nx, ny))

        self.get_logger().warn(f'No A* path found after {cells_searched} cells.')
        return []

    @staticmethod
    def smooth_path(path, passes=5):
        if len(path) < 3:
            return path
        smoothed = list(path)
        for _ in range(passes):
            temp = list(smoothed)
            for i in range(1, len(smoothed) - 1):
                smoothed[i] = (
                    (temp[i - 1][0] + temp[i][0] + temp[i + 1][0]) / 3.0,
                    (temp[i - 1][1] + temp[i][1] + temp[i + 1][1]) / 3.0,
                )
        return smoothed

    def publish_path_for_ui(self, waypoints):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for wx, wy in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def execute_callback(self, goal_handle):
        goal = goal_handle.request.end_position
        feedback_msg = Navigation.Feedback()

        if not self.map_info or not self.position_received:
            self.publish_status('A* cannot navigate: waiting for /map and /slam_pose.')
            goal_handle.abort()
            return Navigation.Result(success=False)

        self.freeze_map = True

        start = self.world_to_grid(self.current_x, self.current_y)
        target = self.world_to_grid(goal.x, goal.y)
        if start is None or target is None:
            self.publish_status('A* cannot convert start/goal to grid cells.')
            self.freeze_map = False
            goal_handle.abort()
            return Navigation.Result(success=False)

        start_gx, start_gy = start
        goal_gx, goal_gy = target
        self.publish_status(
            f'A* planning from grid({start_gx}, {start_gy}) '
            f'to grid({goal_gx}, {goal_gy}).')
        waypoints = self.calculate_astar_path(
            start_gx, start_gy, goal_gx, goal_gy)
        if not waypoints:
            self.publish_status('A* failed: no path found.')
            self.freeze_map = False
            goal_handle.abort()
            return Navigation.Result(success=False)

        waypoints = self.smooth_path(waypoints)
        self.publish_path_for_ui(waypoints)
        self.publish_status(f'A* path accepted with {len(waypoints)} waypoints.')

        final_x, final_y = waypoints[-1]
        closest_idx = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_path_for_ui([])
                self.freeze_map = False
                return Navigation.Result(success=False)

            dist_to_goal = math.sqrt(
                (final_x - self.current_x) ** 2
                + (final_y - self.current_y) ** 2)
            if dist_to_goal < self.waypoint_tolerance:
                break

            min_dist = float('inf')
            search_end = min(closest_idx + 30, len(waypoints))
            for i in range(closest_idx, search_end):
                d = math.sqrt(
                    (waypoints[i][0] - self.current_x) ** 2
                    + (waypoints[i][1] - self.current_y) ** 2)
                if d < min_dist:
                    min_dist = d
                    closest_idx = i

            target_x, target_y = final_x, final_y
            for i in range(closest_idx, len(waypoints)):
                d = math.sqrt(
                    (waypoints[i][0] - self.current_x) ** 2
                    + (waypoints[i][1] - self.current_y) ** 2)
                if d >= self.lookahead_distance:
                    target_x, target_y = waypoints[i]
                    break

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            feedback_msg.direction = float(math.atan2(dy, dx))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.05)

        self.publish_status('A* goal reached.')
        self.publish_path_for_ui([])
        self.freeze_map = False
        goal_handle.succeed()
        return Navigation.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = AStarNavigationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
