#!/usr/bin/env python3
"""
phone_camera.py
---------------
Reads the live video feed from a USB-connected smartphone (Webcam mode) 
and publishes it to the ROS 2 `/camera/image_raw` topic so the AI can process it.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PhoneCameraNode(Node):
    def __init__(self):
        super().__init__('phone_camera')
        
        # We use BEST_EFFORT so if the AI lags, it just drops old frames 
        # rather than building up a massive backlog.
        realtime_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', realtime_qos)
        self.bridge = CvBridge()
        
        self.cap = None
        for index in [0, 1, 2, 3, 4, 5, 6]:
            candidate = cv2.VideoCapture(index)
            if candidate.isOpened():
                ret, frame = candidate.read()
                if ret and frame is not None:
                    self.cap = candidate
                    self.get_logger().info(f"Camera opened at /dev/video{index}")
                    break
            candidate.release()

        if self.cap is None:
            self.get_logger().error("Could not open any /dev/video0..6 camera. Check USB webcam mode and permissions.")
            import sys; sys.exit(1)
            
        # Optional: Set resolution to 640x480 for faster AI processing
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.get_logger().info("✅ Phone Camera Connected! Broadcasting live to the AI...")
        
        # Publish at roughly 30 FPS
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.step = frame.shape[1] * 3
            
            import numpy as np
            msg.data = np.ascontiguousarray(frame).tobytes()
            self.publisher_.publish(msg)

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
