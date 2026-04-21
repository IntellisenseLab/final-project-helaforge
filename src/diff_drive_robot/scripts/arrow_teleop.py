#!/usr/bin/env python3
"""
Arrow-Key Teleop for ROS 2
===========================
Drive your robot with arrow keys.  The robot moves only while a key is
held down and stops immediately when released.

Controls:
  ↑  – forward       ↓  – backward
  ←  – turn left     →  – turn right
  q  – quit
"""

import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── Speeds (tune to your robot) ─────────────────────────────────────
LINEAR_SPEED  = 0.3   # m/s
ANGULAR_SPEED = 1.0   # rad/s


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

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
    stdscr.nodelay(True)       # non-blocking getch
    stdscr.timeout(100)        # refresh every 100 ms

    try:
        stdscr.addstr(0, 0, '=== Arrow-Key Teleop ===')
        stdscr.addstr(1, 0, '↑/↓ = forward/back   ←/→ = turn')
        stdscr.addstr(2, 0, 'q = quit')
        stdscr.addstr(4, 0, 'Status: READY')

        while True:
            key = stdscr.getch()

            if key == ord('q'):
                break
            elif key == curses.KEY_UP:
                node.send(LINEAR_SPEED, 0.0)
                status = 'FORWARD ▲'
            elif key == curses.KEY_DOWN:
                node.send(-LINEAR_SPEED, 0.0)
                status = 'BACKWARD ▼'
            elif key == curses.KEY_LEFT:
                node.send(0.0, ANGULAR_SPEED)
                status = 'TURN LEFT ◀'
            elif key == curses.KEY_RIGHT:
                node.send(0.0, -ANGULAR_SPEED)
                status = 'TURN RIGHT ▶'
            else:
                # No key pressed → stop
                node.stop()
                status = 'STOPPED ■'

            stdscr.move(4, 0)
            stdscr.clrtoeol()
            stdscr.addstr(4, 0, f'Status: {status}')
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
