#!/usr/bin/env python3
"""
phone_camera_publisher.py
-------------------------
Streams the chest camera (a USB webcam on the Pi, or an IP webcam stream given in CAMERA_URL) to the laptop
as JPEG on /camera/image_raw/compressed. Runs on the Pi, started by pi_sensors.launch.py.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import os
import time

CAMERA_SLOW_FPS = float(os.environ.get("CAMERA_SLOW_FPS", "10"))  # below this the rate report becomes a warning


class PhoneCameraNode(Node):
    def __init__(self):
        super().__init__('phone_camera_publisher')
        
        # BEST_EFFORT: if the receiver lags, old frames are dropped rather than queued
        realtime_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', realtime_qos)
        self.bridge = CvBridge()
        self.publish_fps = float(os.environ.get("CAMERA_PUBLISH_FPS", "30"))
        # Smaller JPEGs = fewer UDP fragments over Wi-Fi; with BEST_EFFORT one lost fragment drops a frame
        self.jpeg_quality = int(os.environ.get("CAMERA_JPEG_QUALITY", "80"))
        self.publish_interval = 1.0 / max(self.publish_fps, 1.0)
        self._last_publish_time = 0.0
        
        self.cap = None
        
        # An IP webcam stream (CAMERA_URL), else the first working local video device
        camera_url = os.environ.get('CAMERA_URL', '').strip()
        if camera_url:
            self.get_logger().info(f"Connecting to wireless IP Webcam stream at: {camera_url}...")
            candidate = cv2.VideoCapture(camera_url, cv2.CAP_FFMPEG)
            if candidate.isOpened():
                ret, frame = candidate.read()
                if ret and frame is not None:
                    self.cap = candidate
                    self.get_logger().info("✅ Successfully linked to IP Webcam stream!")
            if self.cap is None:
                self.get_logger().error(f"Failed to open IP Webcam stream at {camera_url}. Check Wi-Fi connection and URL.")
                import sys; sys.exit(1)
        else:
            self.get_logger().info("No CAMERA_URL set. Scanning local USB hardware webcams...")
            for index in range(0, 35):
                candidate = cv2.VideoCapture(index, cv2.CAP_V4L2)
                if candidate.isOpened():
                    ret, frame = candidate.read()
                    if ret and frame is not None:
                        self.cap = candidate
                        self.get_logger().info(f"Camera opened at /dev/video{index}")
                        break
                candidate.release()
            
            if self.cap is None:
                self.get_logger().error("Could not open any camera device or IP stream. Set CAMERA_URL or check USB permissions.")
                import sys; sys.exit(1)
            
        # MJPEG from the camera (less USB bandwidth), 640x480 at 30 fps
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Keep only the newest frame in the driver's buffer
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.get_logger().info(f"Phone Camera connected. Publishing freshest frames at up to {self.publish_fps:.0f} FPS "
                               f"(JPEG quality {self.jpeg_quality}).")
        
        # A dedicated thread drains the camera buffer as fast as it fills
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

    def capture_loop(self):
        grabbed_count, published_count, bytes_sent, last_report = 0, 0, 0, time.monotonic()
        while self.running and rclpy.ok():
            grabbed = self.cap.grab()
            if not grabbed:
                time.sleep(0.002)
                continue
            grabbed_count += 1

            now = time.monotonic()
            if now - last_report >= 5.0:
                # If "camera" is low, the webcam itself is slow (often auto-exposure in dim light);
                # if "published" is fine but the laptop receives less, it is Wi-Fi loss.
                # Quiet by default (see it with --ros-args --log-level debug); warns only when slow.
                dt = now - last_report
                report = (f"camera {grabbed_count / dt:.1f} fps, published {published_count / dt:.1f} fps, "
                          f"{bytes_sent / max(published_count, 1) / 1024:.0f} KB/frame")
                if grabbed_count / dt < CAMERA_SLOW_FPS:
                    self.get_logger().warn(f"Camera is slow: {report}")
                else:
                    self.get_logger().debug(report)
                grabbed_count, published_count, bytes_sent, last_report = 0, 0, 0, now
            if (now - self._last_publish_time) < self.publish_interval:
                continue

            ret, frame = self.cap.retrieve()
            if not ret or frame is None:
                continue

            self._last_publish_time = now
            ok, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.format = "jpeg"
            msg.data = jpg.tobytes()
            self.publisher_.publish(msg)
            published_count += 1
            bytes_sent += len(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PhoneCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
