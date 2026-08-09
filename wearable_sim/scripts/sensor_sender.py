#!/usr/bin/env python3
"""
sensor_sender.py — Runs on the Raspberry Pi 5.
DIRECTLY captures camera via OpenCV (bypasses v4l2_camera ROS node)
and forwards LiDAR + Camera to the Laptop via TCP.

Zero-lag architecture:
  - Camera: Direct OpenCV capture → MJPG → TCP (no ROS overhead)
  - LiDAR:  ROS subscription → JSON → TCP

Usage:
  ros2 run wearable_sim sensor_sender.py --ros-args -p laptop_ip:=172.27.50.253
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct
import threading
import json
import time
import cv2
import numpy as np

TOPIC_SCAN = 0
TOPIC_IMAGE = 1


class SensorSender(Node):
    def __init__(self):
        super().__init__('sensor_sender')

        self.declare_parameter('laptop_ip', '172.27.50.253')
        self.declare_parameter('port', 5555)
        self.declare_parameter('video_device', '/dev/video1')

        self.laptop_ip = self.get_parameter('laptop_ip').value
        self.port = self.get_parameter('port').value
        self.video_device = self.get_parameter('video_device').value

        self.sock = None
        self.connected = False
        self.send_lock = threading.Lock()

        # Background threads
        threading.Thread(target=self._connect_loop, daemon=True).start()
        threading.Thread(target=self._camera_loop, daemon=True).start()

        # Subscribe to LiDAR (still via ROS — it's lightweight)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.get_logger().info(
            f'🔌 Sender starting. Target: {self.laptop_ip}:{self.port}'
        )
        self.get_logger().info(
            f'📷 Direct camera capture from {self.video_device}'
        )

    # ── Connection ────────────────────────────────────────────────────────
    def _connect_loop(self):
        while rclpy.ok():
            if not self.connected:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.settimeout(3)
                    s.connect((self.laptop_ip, self.port))
                    s.settimeout(None)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    with self.send_lock:
                        self.sock = s
                        self.connected = True
                    self.get_logger().info(
                        f'✅ Connected to laptop at {self.laptop_ip}:{self.port}'
                    )
                except Exception:
                    time.sleep(1)
            else:
                time.sleep(0.5)

    def _send_packet(self, topic_id: int, data: bytes):
        with self.send_lock:
            if not self.connected or self.sock is None:
                return
            try:
                header = struct.pack('!II', topic_id, len(data))
                self.sock.sendall(header + data)
            except Exception:
                self.get_logger().warn('Connection lost. Reconnecting...')
                self.connected = False
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    # ── LiDAR (via ROS — small data, no lag issue) ────────────────────────
    def _scan_cb(self, msg: LaserScan):
        scan_dict = {
            'frame_id': msg.header.frame_id,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
        }
        self._send_packet(TOPIC_SCAN, json.dumps(scan_dict).encode())

    # ── Camera (DIRECT OpenCV — zero ROS overhead) ────────────────────────
    def _camera_loop(self):
        """Capture directly from hardware, compress, and send.
        This is the fastest possible path — no ROS, no v4l2_camera node."""

        # Wait for connection first
        while rclpy.ok() and not self.connected:
            time.sleep(0.5)

        # Try multiple methods to open the camera (Pi 5 compatibility)
        cap = None
        # Extract device index from path (e.g., /dev/video1 → 1)
        try:
            dev_index = int(self.video_device.replace('/dev/video', ''))
        except ValueError:
            dev_index = 1

        # On Pi 5, OpenCV V4L2 backend requires integer index, not string path.
        # Also, sometimes video1 is metadata and video2 is the actual capture device.
        attempts = [
            (f'v4l2_index_{dev_index}', lambda: cv2.VideoCapture(dev_index, cv2.CAP_V4L2)),
            (f'v4l2_index_{dev_index+1}', lambda: cv2.VideoCapture(dev_index + 1, cv2.CAP_V4L2)),
            (f'v4l2_index_0', lambda: cv2.VideoCapture(0, cv2.CAP_V4L2)),
            ('auto_index', lambda: cv2.VideoCapture(dev_index)),
        ]

        for name, opener in attempts:
            self.get_logger().info(f'Trying camera method: {name}...')
            cap = opener()
            if cap is not None and cap.isOpened():
                # Test if it can actually read a frame (filters out metadata nodes)
                cap.grab()
                ret, _ = cap.retrieve()
                if ret:
                    self.get_logger().info(f'✅ Camera opened and reading frames via method: {name}')
                    break
                else:
                    self.get_logger().warn(f'Method {name} opened but could not read frames.')
            if cap is not None:
                cap.release()
            cap = None

        if cap is None or not cap.isOpened():
            self.get_logger().error('Cannot open camera with any method!')
            return

        # Configure for speed
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.get_logger().info('📷 Camera streaming!')

        target_fps = 15
        interval = 1.0 / target_fps

        while rclpy.ok():
            start = time.monotonic()

            # grab() discards buffered frames, retrieve() gets the latest
            cap.grab()
            ret, frame = cap.retrieve()

            if ret and self.connected:
                _, jpeg = cv2.imencode(
                    '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50]
                )
                jpeg_bytes = jpeg.tobytes()
                meta = struct.pack('!II', frame.shape[1], frame.shape[0])
                self._send_packet(TOPIC_IMAGE, meta + jpeg_bytes)

            elapsed = time.monotonic() - start
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

        cap.release()


def main():
    rclpy.init()
    node = SensorSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
