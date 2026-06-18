"""
Arrow-Key Teleop for ROS 2 – drive with arrow keys, stop on release.
"""
import curses
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

DEFAULT_LINEAR_SPEED = 0.20
DEFAULT_ANGULAR_SPEED = 0.70
DEFAULT_STOP_RAMP_TIME = 0.45
DEFAULT_KEY_TIMEOUT = 0.18


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.declare_parameter('linear_speed', DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('stop_ramp_time', DEFAULT_STOP_RAMP_TIME)
        self.declare_parameter('key_timeout', DEFAULT_KEY_TIMEOUT)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.stop_ramp_time = max(
            0.05, float(self.get_parameter('stop_ramp_time').value))
        self.key_timeout = max(
            0.05, float(self.get_parameter('key_timeout').value))

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enabled = False
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        teleop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, '/semantic_nav/teleop_enabled', self._enabled_cb, teleop_qos)

    def _enabled_cb(self, msg: Bool):
        was_enabled = self.enabled
        self.enabled = msg.data
        if was_enabled and not self.enabled:
            self.stop(immediate=True)

    def _publish(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.pub.publish(t)

    @staticmethod
    def _step_toward(current: float, target: float, max_delta: float) -> float:
        if current < target:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def set_target(self, linear: float, angular: float):
        self.target_linear = linear
        self.target_angular = angular

    def stop(self, immediate: bool = False):
        self.target_linear = 0.0
        self.target_angular = 0.0
        if immediate:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self._publish(0.0, 0.0)

    def update(self, dt: float):
        if not self.enabled:
            return

        old_linear = self.current_linear
        old_angular = self.current_angular
        linear_rate = max(abs(self.linear_speed) / self.stop_ramp_time, 0.02)
        angular_rate = max(abs(self.angular_speed) / self.stop_ramp_time, 0.05)

        self.current_linear = self._step_toward(
            self.current_linear, self.target_linear, linear_rate * dt)
        self.current_angular = self._step_toward(
            self.current_angular, self.target_angular, angular_rate * dt)

        active = (
            abs(self.current_linear) > 1e-4
            or abs(self.current_angular) > 1e-4
            or abs(self.target_linear) > 1e-4
            or abs(self.target_angular) > 1e-4
            or abs(old_linear) > 1e-4
            or abs(old_angular) > 1e-4)
        if active:
            self._publish(self.current_linear, self.current_angular)

    def is_stopped(self) -> bool:
        return (
            abs(self.current_linear) < 1e-4
            and abs(self.current_angular) < 1e-4
            and abs(self.target_linear) < 1e-4
            and abs(self.target_angular) < 1e-4)


def main(args=None):
    rclpy.init(args=args)
    node = ArrowTeleop()

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    try:
        stdscr.addstr(0, 0, '=== Arrow-Key Teleop ===')
        stdscr.addstr(1, 0, 'Up/Down = forward/back   Left/Right = turn')
        stdscr.addstr(2, 0, 'q = quit')
        stdscr.addstr(3, 0, 'Say "Scan Environment" to enable teleop.')
        stdscr.addstr(5, 0, 'Status: DISABLED')

        status = 'DISABLED'
        last_motion_key_time = 0.0
        last_time = time.monotonic()

        while True:
            now = time.monotonic()
            dt = max(0.0, now - last_time)
            last_time = now

            rclpy.spin_once(node, timeout_sec=0.0)
            key = stdscr.getch()
            if key == ord('q'):
                break
            elif not node.enabled:
                node.stop(immediate=True)
                status = 'DISABLED'
            elif key == curses.KEY_UP:
                node.set_target(node.linear_speed, 0.0)
                last_motion_key_time = now
                status = 'FORWARD'
            elif key == curses.KEY_DOWN:
                node.set_target(-node.linear_speed, 0.0)
                last_motion_key_time = now
                status = 'BACKWARD'
            elif key == curses.KEY_LEFT:
                node.set_target(0.0, node.angular_speed)
                last_motion_key_time = now
                status = 'TURN LEFT'
            elif key == curses.KEY_RIGHT:
                node.set_target(0.0, -node.angular_speed)
                last_motion_key_time = now
                status = 'TURN RIGHT'
            elif key != -1 or (now - last_motion_key_time) > node.key_timeout:
                node.stop(immediate=False)
                status = 'STOPPED' if node.is_stopped() else 'SMOOTH STOP'
            else:
                pass

            node.update(dt)

            stdscr.move(5, 0)
            stdscr.clrtoeol()
            stdscr.addstr(5, 0, f'Status: {status}')
            stdscr.refresh()

    except KeyboardInterrupt:
        pass
    finally:
        node.stop(immediate=True)
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
