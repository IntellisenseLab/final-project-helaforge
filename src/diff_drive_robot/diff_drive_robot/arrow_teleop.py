"""
Arrow-Key Teleop for ROS 2 – drive with arrow keys, stop on release.
"""
import curses
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

DEFAULT_LINEAR_SPEED = 0.20
DEFAULT_ANGULAR_SPEED = 0.70


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.declare_parameter('linear_speed', DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enabled = False
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
            self.stop()

    def send(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.pub.publish(t)

    def stop(self):
        self.send(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArrowTeleop()

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    try:
        stdscr.addstr(0, 0, '=== Arrow-Key Teleop ===')
        stdscr.addstr(1, 0, 'Up/Down = forward/back   Left/Right = turn')
        stdscr.addstr(2, 0, 'q = quit')
        stdscr.addstr(3, 0, 'Say "Scan Environment" to enable teleop.')
        stdscr.addstr(5, 0, 'Status: DISABLED')

        while True:
            rclpy.spin_once(node, timeout_sec=0.0)
            key = stdscr.getch()
            if key == ord('q'):
                break
            elif not node.enabled:
                status = 'DISABLED'
            elif key == curses.KEY_UP:
                node.send(node.linear_speed, 0.0)
                status = 'FORWARD'
            elif key == curses.KEY_DOWN:
                node.send(-node.linear_speed, 0.0)
                status = 'BACKWARD'
            elif key == curses.KEY_LEFT:
                node.send(0.0, node.angular_speed)
                status = 'TURN LEFT'
            elif key == curses.KEY_RIGHT:
                node.send(0.0, -node.angular_speed)
                status = 'TURN RIGHT'
            else:
                node.stop()
                status = 'STOPPED'

            stdscr.move(5, 0)
            stdscr.clrtoeol()
            stdscr.addstr(5, 0, f'Status: {status}')
            stdscr.refresh()

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
