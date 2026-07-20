#!/usr/bin/env python3
"""
vision_perception.py
====================
ROS 2 Jazzy – Wearable Blind-Assist Vision Node  (OpenCV 5 / ONNX edition)
"""

import os
os.environ["QT_QPA_PLATFORM"]  = "xcb"          # force X11/XWayland
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.fonts=false"

import cv2
import math
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

np.random.seed(42)
CONFIDENCE_THRESHOLD = 0.40
NMS_THRESHOLD        = 0.45   # IoU threshold for Non-Maximum Suppression

INPUT_W = INPUT_H = 640

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_PATH  = os.path.join(_SCRIPT_DIR, "yolov5m.onnx")

class VisionPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_perception")

        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_save_time = 0.0

        cv2.namedWindow("Wearable Vision System", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Wearable Vision System", 800, 600)
        cv2.waitKey(1)

        self.get_logger().info("Loading YOLOv5m ONNX model...")
        self._net = self._load_model()
        self.get_logger().info("Model loaded. Ready for detections.")

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        realtime_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self._image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, realtime_qos,
        )
        self._image_pub = self.create_publisher(Image, "/vision/debug_image", 10)
        
        self._latest_scan = None
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data,
        )
        self._marker_pub = self.create_publisher(MarkerArray, "/semantic_markers", 10)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self._mapped_locations = []
        self._saved_markers = []
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        self._marker_pub.publish(MarkerArray(markers=[delete_marker]))
        
        # Hazard Tracking
        self._hazard_pub = self.create_publisher(String, "/hazard_warning", 10)
        self._hazard_history = {}  # {label_id: (cx, cy, area, time)}
        self._hazard_classes = ["person", "bicycle", "car", "motorcycle", "bus", "truck"]

    def _scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _load_model(self) -> cv2.dnn_Net:
        if not os.path.isfile(MODEL_PATH):
            raise FileNotFoundError(MODEL_PATH)
        net = cv2.dnn.readNetFromONNX(MODEL_PATH)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        return net

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1
        try:
            frame: np.ndarray = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            return

        import time
        now = time.monotonic()
        if now - self._last_save_time >= 5.0:
            cv2.imwrite("/tmp/vision_frame.jpg", frame)
            self._last_save_time = now

        if not hasattr(self, '_cached_boxes'):
            self._cached_boxes = []
            
        if self._frame_count % 10 != 0:
            self._draw_cached_boxes(frame)
            return

        blob = cv2.dnn.blobFromImage(
            frame, scalefactor=1.0 / 255.0, size=(INPUT_W, INPUT_H),
            mean=(0.0, 0.0, 0.0), swapRB=True, crop=False,
        )
        self._net.setInput(blob)
        raw_output = self._net.forward(self._net.getUnconnectedOutLayersNames())[0]

        h, w = frame.shape[:2]
        scale_x = w / INPUT_W
        scale_y = h / INPUT_H
        predictions = raw_output[0]

        boxes, confidences, class_ids = [], [], []

        for det in predictions:
            obj_conf = float(det[4])
            if obj_conf < 0.30:
                continue

            class_scores = det[5:]
            class_id = int(np.argmax(class_scores))
            confidence = obj_conf * float(class_scores[class_id])

            if confidence < CONFIDENCE_THRESHOLD:
                continue

            # All 80 COCO classes are now detected (no filter).
            # Friendly name mapping happens later in the detection loop.

            cx = float(det[0]) * scale_x
            cy = float(det[1]) * scale_y
            bw_ = float(det[2]) * scale_x
            bh_ = float(det[3]) * scale_y

            x1 = max(0, int(cx - bw_ / 2))
            y1 = max(0, int(cy - bh_ / 2))
            bw_ = min(w - x1, int(bw_))
            bh_ = min(h - y1, int(bh_))

            boxes.append([x1, y1, bw_, bh_])
            confidences.append(confidence)
            class_ids.append(class_id)

        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
            if len(indices) > 0:
                indices = indices.flatten()
        else:
            indices = []

        self._cached_boxes = []
        
        # Keep track of current frame hazards
        current_hazards = {}
        
        for idx in indices:
            x1, y1, bw_, bh_ = boxes[idx]
            x2, y2 = x1 + bw_, y1 + bh_
            cid = class_ids[idx]
            conf = confidences[idx]
            raw_label = COCO_CLASSES[cid]  # Original COCO name (for hazard checks)
            
            # Friendly name mapping for voice/display
            FRIENDLY_NAMES = {
                "dining table": "table", "couch": "sofa", "cell phone": "smartphone",
                "potted plant": "plant", "wine glass": "glass", "sports ball": "ball",
                "baseball bat": "bat", "baseball glove": "glove", "tennis racket": "racket",
                "hair drier": "hair dryer", "tv": "television", "fire hydrant": "hydrant",
                "parking meter": "meter", "traffic light": "signal",
            }
            label = FRIENDLY_NAMES.get(raw_label, raw_label)
            
            color = (0, 0, 255) if raw_label in self._hazard_classes else (0, 255, 0)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness=2)
            label_text = f"{label}: {conf * 100:.1f}%"
            (tw, th), bl = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            ly = max(y1, th + bl + 4)
            cv2.rectangle(frame, (x1, ly - th - bl - 4), (x1 + tw, ly), color, cv2.FILLED)
            cv2.putText(frame, label_text, (x1, ly - bl - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

            self._cached_boxes.append((x1, y1, x2, y2, color, label_text))
            
            # ============================================
            # MOVING HAZARD TRACKING (Dynamic Danger Zones)
            # ============================================
            if raw_label in self._hazard_classes:
                cx_box = x1 + bw_ / 2.0
                cy_box = y1 + bh_ / 2.0
                area = bw_ * bh_
                now_time = time.monotonic()
                
                if 200 < cx_box < 440:
                    matched_id = None
                    for h_id, (prev_cx, prev_cy, prev_area, prev_time) in self._hazard_history.items():
                        if h_id.startswith(raw_label):
                            if math.hypot(cx_box - prev_cx, cy_box - prev_cy) < 100:
                                matched_id = h_id
                                break
                    
                    if matched_id:
                        _, _, prev_area, prev_time = self._hazard_history[matched_id]
                        time_diff = now_time - prev_time
                        
                        if area > prev_area * 1.15 and time_diff < 1.0 and area > 10000:
                            msg = String()
                            msg.data = f"EMERGENCY BRAKE! {label.upper()} is rapidly approaching!"
                            self._hazard_pub.publish(msg)
                            self.get_logger().warn(msg.data)
                            color = (0, 0, 255)
                            
                        current_hazards[matched_id] = (cx_box, cy_box, area, now_time)
                    else:
                        new_id = f"{raw_label}_{len(current_hazards)}"
                        current_hazards[new_id] = (cx_box, cy_box, area, now_time)
            
            # Skip physical mapping for hazards (they move!)
            if raw_label in self._hazard_classes:
                continue

            cx = x1 + bw_ / 2.0
            yaw = -((cx - 320.0) / 320.0) * (1.396 / 2.0)
            
            try:
                # ============================================
                # CAMERA-BASED DEPTH ESTIMATION (Pinhole Model)
                # ============================================
                # Instead of blindly trusting LiDAR (which hits walls 
                # between robot and object), estimate depth from the 
                # bounding box size. Larger box = closer object.
                #
                # Formula: depth = (real_height * focal_length) / bbox_height
                # focal_length = image_width / (2 * tan(hfov/2))
                #              = 640 / (2 * tan(0.698)) = 381.4 pixels
                
                FOCAL = 381.4
                # Approximate real-world heights (meters) for ALL 80 COCO classes
                KNOWN_HEIGHTS = {
                    "chair": 0.85, "couch": 0.85, "dining table": 0.75, "bed": 0.6,
                    "toilet": 0.45, "bench": 0.45, "tv": 0.5, "laptop": 0.25, 
                    "cell phone": 0.12, "remote": 0.15, "keyboard": 0.05, "mouse": 0.05,
                    "refrigerator": 1.7, "oven": 0.6, "microwave": 0.35, "toaster": 0.2,
                    "sink": 0.3, "bottle": 0.25, "wine glass": 0.2, "cup": 0.12,
                    "fork": 0.15, "knife": 0.15, "spoon": 0.15, "bowl": 0.10,
                    "potted plant": 0.4, "vase": 0.3, "book": 0.25, "clock": 0.3,
                    "scissors": 0.18, "teddy bear": 0.3, "hair drier": 0.25,
                    "toothbrush": 0.18, "backpack": 0.45, "umbrella": 0.9, "handbag": 0.3,
                    "tie": 0.5, "suitcase": 0.6, "person": 1.7, "bicycle": 1.0, 
                    "car": 1.5, "motorcycle": 1.1, "bus": 3.0, "truck": 2.5, "train": 3.5, 
                    "boat": 1.5, "airplane": 4.0, "bird": 0.2, "cat": 0.25, "dog": 0.5, 
                    "horse": 1.6, "sheep": 0.7, "cow": 1.4, "elephant": 3.0, "bear": 1.5,
                    "zebra": 1.4, "giraffe": 5.0, "frisbee": 0.03, "skis": 0.1, 
                    "snowboard": 0.15, "sports ball": 0.22, "kite": 0.6, "baseball bat": 0.8,
                    "baseball glove": 0.25, "skateboard": 0.1, "surfboard": 0.1,
                    "tennis racket": 0.68, "banana": 0.15, "apple": 0.08, "sandwich": 0.08, 
                    "orange": 0.08, "broccoli": 0.15, "carrot": 0.15, "hot dog": 0.05,
                    "pizza": 0.05, "donut": 0.05, "cake": 0.15, "traffic light": 0.9, 
                    "fire hydrant": 0.6, "stop sign": 0.6, "parking meter": 1.2,
                }
                
                real_h = KNOWN_HEIGHTS.get(raw_label, 0.5)
                camera_depth = (real_h * FOCAL) / max(bh_, 10)
                camera_depth = max(0.5, min(camera_depth, 25.0))
                
                depth = camera_depth
                frame_id = "camera_link"
                stamp = self.get_clock().now().to_msg()
                
                # Cross-check with LiDAR if available
                if self._latest_scan is not None:
                    scan = self._latest_scan
                    frame_id = scan.header.frame_id
                    stamp = scan.header.stamp
                    idx_scan = int((yaw - scan.angle_min) / scan.angle_increment)
                    r_vals = []
                    for di in [-1, 0, 1]:
                        ri = idx_scan + di
                        if 0 <= ri < len(scan.ranges):
                            rv = scan.ranges[ri]
                            if scan.range_min < rv < scan.range_max and not math.isinf(rv) and not math.isnan(rv):
                                r_vals.append(rv)
                    if r_vals:
                        lidar_depth = sum(r_vals) / len(r_vals)
                        # If LiDAR and camera agree within 2m, trust LiDAR (it's more precise)
                        if abs(lidar_depth - camera_depth) < 2.0:
                            depth = lidar_depth
                mx = depth * math.cos(yaw)
                my = depth * math.sin(yaw)
                
                pt_local = PointStamped()
                pt_local.header.frame_id = frame_id
                pt_local.header.stamp = stamp
                pt_local.point.x = mx
                pt_local.point.y = my
                pt_local.point.z = 0.5 + ((cid % 5) * 0.15)
                
                try:
                    pt_global = self._tf_buffer.transform(pt_local, 'map', rclpy.duration.Duration(seconds=0.5))
                except Exception:
                    continue
                px, py = pt_global.point.x, pt_global.point.y
                
                is_duplicate = False
                for i, mapped in enumerate(self._mapped_locations):
                    if len(mapped) == 3: mapped = (*mapped, 0.0, 0, "")
                    if len(mapped) == 5: mapped = (*mapped, "")
                    mapped_cid, m_x, m_y, m_conf, m_idx, m_label = mapped
                    
                    if math.hypot(px - m_x, py - m_y) < 3.0:
                        is_duplicate = True
                        if conf > m_conf and m_idx < len(self._saved_markers):
                            if cid != mapped_cid:
                                if not hasattr(self, '_class_counts'): self._class_counts = {}
                                self._class_counts[cid] = self._class_counts.get(cid, 0) + 1
                                m_label = f"{label.replace(' ', '_')}_{self._class_counts[cid]}"
                                
                            self._mapped_locations[i] = (cid, px, py, conf, m_idx, m_label)
                            self._saved_markers[m_idx].text = m_label
                            self._saved_markers[m_idx].pose.position.x = px
                            self._saved_markers[m_idx].pose.position.y = py
                            if m_idx + 1 < len(self._saved_markers):
                                self._saved_markers[m_idx + 1].pose.position.x = px
                                self._saved_markers[m_idx + 1].pose.position.y = py
                        break
                        
                if is_duplicate:
                    continue
                    
                if not hasattr(self, '_class_counts'): self._class_counts = {}
                self._class_counts[cid] = self._class_counts.get(cid, 0) + 1
                final_label = f"{label.replace(' ', '_')}_{self._class_counts[cid]}"
                
                m_idx = len(self._saved_markers)
                self._mapped_locations.append((cid, px, py, conf, m_idx, final_label))
                
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "yolo_semantic_labels"
                marker.id = len(self._saved_markers) * 2
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = px
                marker.pose.position.y = py
                marker.pose.position.z = pt_global.point.z
                marker.scale.z = 0.2
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                marker.text = final_label
                self._saved_markers.append(marker)
                
                dot_marker = Marker()
                dot_marker.header.frame_id = 'map'
                dot_marker.header.stamp = self.get_clock().now().to_msg()
                dot_marker.ns = "yolo_semantic_anchors"
                dot_marker.id = (len(self._saved_markers) * 2) + 1
                dot_marker.type = Marker.SPHERE
                dot_marker.action = Marker.ADD
                dot_marker.pose.position.x = px
                dot_marker.pose.position.y = py
                dot_marker.pose.position.z = 0.1
                dot_marker.scale.x = 0.15
                dot_marker.scale.y = 0.15
                dot_marker.scale.z = 0.15
                dot_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                self._saved_markers.append(dot_marker)
                    
            except Exception:
                pass

        # Update hazard history for the next frame
        self._hazard_history = current_hazards

        # Prevent Out-Of-Memory (OOM) if odometry is static and markers smear infinitely
        if len(self._saved_markers) > 200:
            self._saved_markers.clear()
            self._mapped_locations.clear()
            if hasattr(self, '_class_counts'):
                self._class_counts.clear()

        if self._saved_markers:
            self._marker_pub.publish(MarkerArray(markers=self._saved_markers))

        self._draw_cached_boxes(frame)

    def _draw_cached_boxes(self, frame):
        for (x1, y1, x2, y2, color, label_text) in getattr(self, '_cached_boxes', []):
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness=2)
            (tw, th), bl = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            ly = max(y1, th + bl + 4)
            cv2.rectangle(frame, (x1, ly - th - bl - 4), (x1 + tw, ly), color, cv2.FILLED)
            cv2.putText(frame, label_text, (x1, ly - bl - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
            
        cv2.imshow("Wearable Vision System", frame)
        cv2.waitKey(1)
        
        try:
            debug_msg = Image()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.height, debug_msg.width = frame.shape[:2]
            debug_msg.encoding = "bgr8"
            debug_msg.step = frame.shape[1] * 3
            debug_msg.data = np.ascontiguousarray(frame).tobytes()
            self._image_pub.publish(debug_msg)
        except Exception:
            pass

def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        rclpy.spin(VisionPerceptionNode())
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
