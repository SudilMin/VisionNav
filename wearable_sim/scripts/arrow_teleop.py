#!/usr/bin/env python3
"""
arrow_teleop.py
===============
Drive the wearable human dummy with arrow keys.

  ↑  Forward       ↓  Backward
  ←  Turn left     →  Turn right
  Space / any other key : stop
  Q or Ctrl-C : quit

Usage
-----
  python3 ~/wearable_ws/src/wearable_sim/scripts/arrow_teleop.py
"""

import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_STEP  = 0.10   # m/s increment per keypress
ANGULAR_STEP = 0.15   # rad/s increment per keypress
MAX_LINEAR   = 1.50   # m/s
MAX_ANGULAR  = 2.00   # rad/s


class ArrowTeleop(Node):
    def __init__(self):
        super().__init__("arrow_teleop")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._lin = 0.0
        self._ang = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x  = self._lin
        msg.angular.z = self._ang
        self._pub.publish(msg)

    def stop(self):
        self._lin = 0.0
        self._ang = 0.0
        self.publish()


def main():
    rclpy.init()
    node = ArrowTeleop()

    def _run(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)          # non-blocking getch
        stdscr.timeout(100)           # refresh every 100 ms

        def safe_addstr(y, x, text):
            try:
                stdscr.addstr(y, x, text)
            except curses.error:
                pass

        stdscr.clear()
        safe_addstr(0, 0, "=== Arrow Key Teleop ===")
        safe_addstr(2, 0, "  UP    arrow : forward")
        safe_addstr(3, 0, "  DOWN  arrow : backward")
        safe_addstr(4, 0, "  LEFT  arrow : turn left")
        safe_addstr(5, 0, "  RIGHT arrow : turn right")
        safe_addstr(6, 0, "  SPACE       : stop")
        safe_addstr(7, 0, "  Q / Ctrl-C  : quit")
        safe_addstr(9, 0, "Speed: ")

        while rclpy.ok():
            key = stdscr.getch()

            if key == curses.KEY_UP:
                node._lin = min(node._lin + LINEAR_STEP, MAX_LINEAR)
                node._ang = 0.0
            elif key == curses.KEY_DOWN:
                node._lin = max(node._lin - LINEAR_STEP, -MAX_LINEAR)
                node._ang = 0.0
            elif key == curses.KEY_LEFT:
                node._ang = min(node._ang + ANGULAR_STEP, MAX_ANGULAR)
                node._lin = 0.0
            elif key == curses.KEY_RIGHT:
                node._ang = max(node._ang - ANGULAR_STEP, -MAX_ANGULAR)
                node._lin = 0.0
            elif key == ord(" "):
                node._lin = 0.0
                node._ang = 0.0
            elif key in (ord("q"), ord("Q")):
                break

            node.publish()
            rclpy.spin_once(node, timeout_sec=0)

            # Status line
            safe_addstr(9, 0,
                f"Speed: linear={node._lin:+.2f} m/s   "
                f"angular={node._ang:+.2f} rad/s    ")
            stdscr.refresh()

        node.stop()

    try:
        curses.wrapper(_run)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
