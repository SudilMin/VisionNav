#!/usr/bin/env python3
"""
sensor_receiver.py — Runs on the Laptop (AI Brain).
Listens for TCP connections from the Pi 5's sensor_sender.py,
receives LiDAR and Camera data, and republishes them as local
ROS 2 topics so SLAM and YOLO can consume them normally.

Uses a "latest frame only" pattern for zero-lag camera feed.

Usage:
  ros2 run wearable_sim sensor_receiver.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Image
import socket
import struct
import threading
import json
import numpy as np
import time

TOPIC_SCAN = 0
TOPIC_IMAGE = 1


class SensorReceiver(Node):
    def __init__(self):
        super().__init__('sensor_receiver')

        self.declare_parameter('port', 5555)
        self.port = self.get_parameter('port').value

        # Realtime QoS: depth=1, best-effort, volatile — zero buffering
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', realtime_qos)

        self.scan_count = 0
        self.image_count = 0

        # Latest-frame-only for camera
        self._latest_image_msg = None
        self._image_lock = threading.Lock()

        # Publish latest image at fixed rate (no buffering)
        self.create_timer(1.0 / 20.0, self._publish_latest_image)  # 20 FPS

        # Status timer
        self.create_timer(5.0, self._status_timer)

        # Start TCP server in background
        threading.Thread(target=self._server_loop, daemon=True).start()

        self.get_logger().info(
            f'📡 TCP Sensor Receiver listening on port {self.port}...'
        )

    def _status_timer(self):
        if self.scan_count > 0 or self.image_count > 0:
            self.get_logger().info(
                f'📊 Bridged: {self.scan_count} scans, {self.image_count} images'
            )

    def _publish_latest_image(self):
        """Timer callback: publish ONLY the most recent image, drop all old ones."""
        with self._image_lock:
            msg = self._latest_image_msg
            self._latest_image_msg = None  # Clear after publishing

        if msg is not None:
            self.image_pub.publish(msg)
            self.image_count += 1

    # ── TCP Server ────────────────────────────────────────────────────────
    def _server_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(1)

        while rclpy.ok():
            self.get_logger().info('🔄 Waiting for Pi 5 to connect...')
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # Small receive buffer to prevent OS-level frame queuing
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
                self.get_logger().info(f'✅ Pi 5 connected from {addr[0]}:{addr[1]}')
                self._handle_connection(conn)
            except Exception as e:
                self.get_logger().warn(f'Server error: {e}')

    def _recv_exact(self, conn, n):
        buf = b''
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('Connection closed')
            buf += chunk
        return buf

    def _handle_connection(self, conn):
        try:
            while rclpy.ok():
                header = self._recv_exact(conn, 8)
                topic_id, payload_len = struct.unpack('!II', header)
                payload = self._recv_exact(conn, payload_len)

                if topic_id == TOPIC_SCAN:
                    self._publish_scan(payload)
                elif topic_id == TOPIC_IMAGE:
                    self._store_image(payload)

        except (ConnectionError, OSError) as e:
            self.get_logger().warn(f'Pi 5 disconnected: {e}')
        finally:
            try:
                conn.close()
            except Exception:
                pass

    # ── Republishers ──────────────────────────────────────────────────────
    def _publish_scan(self, payload: bytes):
        d = json.loads(payload)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = d['frame_id']
        msg.angle_min = float(d['angle_min'])
        msg.angle_max = float(d['angle_max'])
        msg.angle_increment = float(d['angle_increment'])
        msg.time_increment = float(d['time_increment'])
        msg.scan_time = float(d['scan_time'])
        msg.range_min = float(d['range_min'])
        msg.range_max = float(d['range_max'])
        msg.ranges = [float(r) for r in d['ranges']]
        msg.intensities = [float(i) for i in d['intensities']]
        self.scan_pub.publish(msg)
        self.scan_count += 1

    def _store_image(self, payload: bytes):
        """Decode and STORE the image — don't publish immediately.
        The timer callback will grab the latest one."""
        import cv2

        width, height = struct.unpack('!II', payload[:8])
        jpeg_bytes = payload[8:]

        img_bgr = cv2.imdecode(
            np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if img_bgr is None:
            return

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = img_rgb.tobytes()

        # Store latest — old frame is discarded automatically
        with self._image_lock:
            self._latest_image_msg = msg


def main():
    rclpy.init()
    node = SensorReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
