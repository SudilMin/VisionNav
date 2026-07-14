#!/usr/bin/env python3
"""
arrow_teleop.py - Arrow key teleop using curses for reliable key capture.
Publishes Twist to /cmd_vel. Stops instantly when keys are released.
"""
import curses
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('arrow_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.8
        self.turn = 1.2

def ros_spin(node):
    rclpy.spin(node)

def safe_addstr(stdscr, y, x, text):
    """Write text to screen, silently ignoring if terminal is too small."""
    try:
        stdscr.addstr(y, x, text)
    except curses.error:
        pass

def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    rclpy.init()
    node = ArrowTeleop()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    safe_addstr(stdscr, 0, 0, "TELEOP: Arrows=Move  Q=Quit")
    safe_addstr(stdscr, 1, 0, "----------------------------")
    stdscr.refresh()

    try:
        while rclpy.ok():
            key = stdscr.getch()
            twist = Twist()
            status = "STOPPED        "

            if key == curses.KEY_UP:
                twist.linear.x = node.speed
                status = "FORWARD >>>>   "
            elif key == curses.KEY_DOWN:
                twist.linear.x = -node.speed
                status = "<<<< REVERSE   "
            elif key == curses.KEY_LEFT:
                twist.angular.z = node.turn
                status = "<<< TURN LEFT  "
            elif key == curses.KEY_RIGHT:
                twist.angular.z = -node.turn
                status = "TURN RIGHT >>> "
            elif key == ord('q') or key == ord('Q'):
                node.pub.publish(Twist())
                break

            node.pub.publish(twist)
            safe_addstr(stdscr, 2, 0, status)
            stdscr.refresh()

    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    curses.wrapper(main)
