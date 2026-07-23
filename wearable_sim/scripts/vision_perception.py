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
import time
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
DETECTION_FRAME_STRIDE = max(1, int(os.environ.get("WEARABLE_DETECTION_STRIDE", "3")))

class KalmanTracker:
    def __init__(self, track_id: int, x: float, y: float, conf: float, now: float, is_dynamic: bool = True):
        self.id = track_id
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 5.0
        self.conf = conf
        self.last_seen = now
        self.is_dynamic = is_dynamic
        
        self.F = np.eye(4, dtype=np.float64)
        self.H = np.zeros((2, 4), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        
        if is_dynamic:
            self.R = np.eye(2, dtype=np.float64) * 0.5
            self.Q = np.eye(4, dtype=np.float64) * 0.1
        else:
            self.R = np.eye(2, dtype=np.float64) * 1.0
            self.Q = np.eye(4, dtype=np.float64) * 0.001

    def predict(self, now: float):
        dt = max(0.0, now - self.last_seen)
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, px: float, py: float, conf: float, now: float):
        z = np.array([px, py], dtype=np.float64)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.conf = max(self.conf * 0.95, conf)
        self.last_seen = now


class VisionPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_perception")

        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_save_time = 0.0
        self._last_image_time = time.monotonic()

        cv2.setNumThreads(1)
        self._show_window = os.environ.get("WEARABLE_SHOW_WINDOW", "1") != "0"
        if self._show_window:
            cv2.namedWindow("Wearable Vision System", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Wearable Vision System", 800, 600)
            cv2.waitKey(1)

        self.get_logger().info(f"Loading YOLO ONNX model: {MODEL_PATH}")
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
        self._camera_watchdog = self.create_timer(3.0, self._camera_watchdog_callback)
        self._image_pub = self.create_publisher(Image, "/vision/debug_image", 10)
        
        self._latest_scan = None
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data,
        )
        self._marker_pub = self.create_publisher(MarkerArray, "/semantic_markers", 10)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        self._marker_pub.publish(MarkerArray(markers=[delete_marker]))
        
        # Dynamic object tracking for live map markers and warnings.
        self._hazard_pub = self.create_publisher(String, "/hazard_warning", 10)
        self._hazard_history = {}  # {label_id: (cx, cy, area, time)}
        self._dynamic_classes = {"person", "bicycle", "car", "motorcycle", "bus", "truck"}
        self._hazard_classes = self._dynamic_classes

        self._camera_hfov = math.radians(float(os.environ.get("WEARABLE_CAMERA_HFOV_DEG", "70.0")))
        self._camera_yaw_offset = math.radians(float(os.environ.get("WEARABLE_CAMERA_YAW_OFFSET_DEG", "0.0")))
        self._lidar_yaw_offset = math.radians(float(os.environ.get("WEARABLE_LIDAR_YAW_OFFSET_DEG", "0.0")))
        self._mirror_camera_x = os.environ.get("WEARABLE_CAMERA_MIRROR_X", "0") == "1"
        self._trust_lidar_depth = os.environ.get("WEARABLE_TRUST_LIDAR_DEPTH", "1") != "0"
        self._lidar_camera_max_diff = float(os.environ.get("WEARABLE_LIDAR_CAMERA_MAX_DIFF", "1.75"))
        self._static_lidar_camera_max_diff = float(os.environ.get("WEARABLE_STATIC_LIDAR_CAMERA_MAX_DIFF", "0.75"))
        self._static_tracks = {}
        self._next_static_track_id = {}
        self._static_box_tracks = {}
        self._static_smoothing_alpha = float(os.environ.get("WEARABLE_STATIC_SMOOTHING_ALPHA", "0.05"))
        self._static_bbox_alpha = float(os.environ.get("WEARABLE_STATIC_BBOX_ALPHA", "0.12"))
        self._static_bbox_assoc_px = float(os.environ.get("WEARABLE_STATIC_BBOX_ASSOC_PX", "180.0"))
        self._static_association_distance = float(os.environ.get("WEARABLE_STATIC_ASSOC_DISTANCE", "1.50"))
        self._static_track_timeout = float(os.environ.get("WEARABLE_STATIC_TRACK_TIMEOUT", "15.0"))
        
        self._dynamic_tracks = {}
        self._next_dynamic_track_id = {}
        self._dynamic_box_tracks = {}
        self._dynamic_smoothing_alpha = float(os.environ.get("WEARABLE_DYNAMIC_SMOOTHING_ALPHA", "0.30"))
        self._dynamic_bbox_alpha = float(os.environ.get("WEARABLE_DYNAMIC_BBOX_ALPHA", "0.30"))
        self._dynamic_bbox_assoc_px = float(os.environ.get("WEARABLE_DYNAMIC_BBOX_ASSOC_PX", "250.0"))
        self._dynamic_association_distance = float(os.environ.get("WEARABLE_DYNAMIC_ASSOC_DISTANCE", "2.0"))
        self._dynamic_track_timeout = float(os.environ.get("WEARABLE_DYNAMIC_TRACK_TIMEOUT", "2.0"))
        self.get_logger().info(
            f"Projection calibration: hfov={math.degrees(self._camera_hfov):.1f}deg, "
            f"camera_yaw_offset={math.degrees(self._camera_yaw_offset):.1f}deg, "
            f"lidar_yaw_offset={math.degrees(self._lidar_yaw_offset):.1f}deg, "
            f"mirror_x={self._mirror_camera_x}, trust_lidar={self._trust_lidar_depth}, "
            f"lidar_camera_max_diff={self._lidar_camera_max_diff:.2f}m, "
            f"static_lidar_camera_max_diff={self._static_lidar_camera_max_diff:.2f}m, "
            f"static_alpha={self._static_smoothing_alpha:.2f}, "
            f"static_bbox_alpha={self._static_bbox_alpha:.2f}, "
            f"static_assoc={self._static_association_distance:.2f}m, "
            f"dynamic_alpha={self._dynamic_smoothing_alpha:.2f}, "
            f"dynamic_bbox_alpha={self._dynamic_bbox_alpha:.2f}"
        )

    def _camera_watchdog_callback(self) -> None:
        if self._frame_count == 0 and time.monotonic() - self._last_image_time > 3.0:
            self.get_logger().warn(
                "No /camera/image_raw frames received yet. Start phone_camera.py or check the USB camera index.",
                throttle_duration_sec=6.0,
            )

    def _scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    @staticmethod
    def _angle_wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _estimate_lidar_depth(self, yaw: float, half_width_angle: float) -> float | None:
        if self._latest_scan is None:
            return None

        scan = self._latest_scan
        scan_angle = self._angle_wrap(yaw + self._lidar_yaw_offset)
        angle_min = scan.angle_min
        angle_max = scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1)

        if scan.angle_increment == 0.0:
            return None

        # Some scanners publish [0, 2pi), while camera yaw naturally uses [-pi, pi].
        if scan_angle < angle_min:
            scan_angle += 2.0 * math.pi
        elif scan_angle > angle_max:
            scan_angle -= 2.0 * math.pi

        samples = max(2, int(abs(half_width_angle / scan.angle_increment)))
        center_idx = int(round((scan_angle - angle_min) / scan.angle_increment))
        ranges = []

        for di in range(-samples, samples + 1):
            idx = center_idx + di
            if 0 <= idx < len(scan.ranges):
                rv = scan.ranges[idx]
                if scan.range_min < rv < scan.range_max and not math.isinf(rv) and not math.isnan(rv):
                    ranges.append(rv)

        if ranges:
            ranges.sort()
            return ranges[len(ranges) // 2]

        if angle_min <= scan_angle <= angle_max:
            return None

        return None

    def _stabilize_object(self, label: str, px: float, py: float, conf: float, now: float, is_dynamic: bool) -> tuple[str, float, float]:
        tracks_dict = self._dynamic_tracks if is_dynamic else self._static_tracks
        used_tracks_dict = self._dynamic_tracks_used if is_dynamic else self._static_tracks_used
        timeout = self._dynamic_track_timeout if is_dynamic else self._static_track_timeout
        association_dist = self._dynamic_association_distance if is_dynamic else self._static_association_distance
        next_id_dict = self._next_dynamic_track_id if is_dynamic else self._next_static_track_id
        
        tracks = tracks_dict.setdefault(label, [])
        tracks[:] = [track for track in tracks if now - track.last_seen <= timeout]

        for track in tracks:
            if track.id not in used_tracks_dict.get(label, set()):
                track.predict(now)

        best_track = None
        best_dist = float("inf")
        for track in tracks:
            if track.id in used_tracks_dict.get(label, set()):
                continue
            dist = math.hypot(px - track.x[0], py - track.x[1])
            if dist < best_dist:
                best_dist = dist
                best_track = track

        if best_track is None or best_dist > association_dist:
            track_id = next_id_dict.get(label, 0) + 1
            next_id_dict[label] = track_id
            best_track = KalmanTracker(track_id, px, py, conf, now, is_dynamic=is_dynamic)
            tracks.append(best_track)
        else:
            best_track.update(px, py, conf, now)

        used_tracks_dict.setdefault(label, set()).add(best_track.id)
        final_label = f"{label.replace(' ', '_')}_{best_track.id}"
        return final_label, float(best_track.x[0]), float(best_track.x[1])

    def _stabilize_static_box(
        self, label: str, x1: int, y1: int, bw: int, bh: int, conf: float, now: float
    ) -> tuple[int, int, int, int]:
        tracks = self._static_box_tracks.setdefault(label, [])
        tracks[:] = [track for track in tracks if now - track["last_seen"] <= self._static_track_timeout]

        cx = x1 + bw / 2.0
        cy = y1 + bh / 2.0
        best_track = None
        best_dist = float("inf")

        for track in tracks:
            if track["id"] in self._static_box_tracks_used.get(label, set()):
                continue
            dist = math.hypot(cx - track["cx"], cy - track["cy"])
            if dist < best_dist:
                best_dist = dist
                best_track = track

        if best_track is None or best_dist > self._static_bbox_assoc_px:
            track_id = len(tracks) + 1
            best_track = {
                "id": track_id,
                "cx": cx,
                "cy": cy,
                "bw": float(bw),
                "bh": float(bh),
                "conf": conf,
                "last_seen": now,
            }
            tracks.append(best_track)
        else:
            alpha = max(0.0, min(self._static_bbox_alpha, 1.0))
            best_track["cx"] = (1.0 - alpha) * best_track["cx"] + alpha * cx
            best_track["cy"] = (1.0 - alpha) * best_track["cy"] + alpha * cy
            best_track["bw"] = (1.0 - alpha) * best_track["bw"] + alpha * float(bw)
            best_track["bh"] = (1.0 - alpha) * best_track["bh"] + alpha * float(bh)
            best_track["conf"] = max(best_track["conf"] * 0.95, conf)
            best_track["last_seen"] = now

        self._static_box_tracks_used.setdefault(label, set()).add(best_track["id"])
        stable_x1 = int(round(best_track["cx"] - best_track["bw"] / 2.0))
        stable_y1 = int(round(best_track["cy"] - best_track["bh"] / 2.0))
        stable_bw = int(round(best_track["bw"]))
        stable_bh = int(round(best_track["bh"]))
        return stable_x1, stable_y1, max(stable_bw, 1), max(stable_bh, 1)


    def _stabilize_dynamic_box(
        self, label: str, x1: int, y1: int, bw: int, bh: int, conf: float, now: float
    ) -> tuple[int, int, int, int]:
        tracks = self._dynamic_box_tracks.setdefault(label, [])
        tracks[:] = [track for track in tracks if now - track["last_seen"] <= self._dynamic_track_timeout]

        cx = x1 + bw / 2.0
        cy = y1 + bh / 2.0
        best_track = None
        best_dist = float("inf")

        for track in tracks:
            if track["id"] in self._dynamic_box_tracks_used.get(label, set()):
                continue
            dist = math.hypot(cx - track["cx"], cy - track["cy"])
            if dist < best_dist:
                best_dist = dist
                best_track = track

        if best_track is None or best_dist > self._dynamic_bbox_assoc_px:
            track_id = len(tracks) + 1
            best_track = {
                "id": track_id,
                "cx": cx,
                "cy": cy,
                "bw": float(bw),
                "bh": float(bh),
                "conf": conf,
                "last_seen": now,
            }
            tracks.append(best_track)
        else:
            alpha = max(0.0, min(self._dynamic_bbox_alpha, 1.0))
            best_track["cx"] = (1.0 - alpha) * best_track["cx"] + alpha * cx
            best_track["cy"] = (1.0 - alpha) * best_track["cy"] + alpha * cy
            best_track["bw"] = (1.0 - alpha) * best_track["bw"] + alpha * float(bw)
            best_track["bh"] = (1.0 - alpha) * best_track["bh"] + alpha * float(bh)
            best_track["conf"] = max(best_track["conf"] * 0.95, conf)
            best_track["last_seen"] = now

        self._dynamic_box_tracks_used.setdefault(label, set()).add(best_track["id"])
        stable_x1 = int(round(best_track["cx"] - best_track["bw"] / 2.0))
        stable_y1 = int(round(best_track["cy"] - best_track["bh"] / 2.0))
        stable_bw = int(round(best_track["bw"]))
        stable_bh = int(round(best_track["bh"]))
        return stable_x1, stable_y1, max(stable_bw, 1), max(stable_bh, 1)

    def _load_model(self) -> cv2.dnn_Net:
        if not os.path.isfile(MODEL_PATH):
            raise FileNotFoundError(MODEL_PATH)
        net = cv2.dnn.readNetFromONNX(MODEL_PATH)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        return net

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1
        self._last_image_time = time.monotonic()
        try:
            frame: np.ndarray = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            return

        now = time.monotonic()
        if now - self._last_save_time >= 5.0:
            cv2.imwrite("/tmp/vision_frame.jpg", frame)
            self._last_save_time = now

        if not hasattr(self, '_cached_boxes'):
            self._cached_boxes = []
            
        if self._frame_count % DETECTION_FRAME_STRIDE != 0:
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
        
        # Current-frame markers replace the previous semantic overlay in RViz.
        current_hazards = {}
        current_markers = []
        current_class_counts = {}
        self._static_tracks_used = {}
        self._static_box_tracks_used = {}
        self._dynamic_tracks_used = {}
        self._dynamic_box_tracks_used = {}
        marker_seq = 0
        now_msg = msg.header.stamp
        marker_lifetime = rclpy.duration.Duration(seconds=1.5).to_msg()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        current_markers.append(delete_marker)
        
        for idx in indices:
            raw_x1, raw_y1, raw_bw, raw_bh = boxes[idx]
            x1, y1, bw_, bh_ = raw_x1, raw_y1, raw_bw, raw_bh
            x2, y2 = x1 + bw_, y1 + bh_
            
            is_truncated = (x1 <= 5) or (y1 <= 5) or (x2 >= w - 5) or (y2 >= h - 5)
            
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
            
            is_dynamic = raw_label in self._dynamic_classes
            if not is_dynamic:
                x1, y1, bw_, bh_ = self._stabilize_static_box(label, x1, y1, bw_, bh_, conf, now)
            else:
                x1, y1, bw_, bh_ = self._stabilize_dynamic_box(label, x1, y1, bw_, bh_, conf, now)
                
            x1 = max(0, min(x1, w - 1))
            y1 = max(0, min(y1, h - 1))
            bw_ = max(1, min(bw_, w - x1))
            bh_ = max(1, min(bh_, h - y1))
            x2, y2 = x1 + bw_, y1 + bh_

            color = (0, 0, 255) if is_dynamic else (0, 255, 0)

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
            
            self.get_logger().info(f'LIVE MAP: {label} conf={conf:.2f} bbox=({x1},{y1},{x2},{y2})', throttle_duration_sec=0.5)

            cx = x1 + bw_ / 2.0
            image_center_x = max(w / 2.0, 1.0)
            pixel_offset = (cx - image_center_x) / image_center_x
            yaw_sign = 1.0 if self._mirror_camera_x else -1.0
            yaw = yaw_sign * pixel_offset * (self._camera_hfov / 2.0) + self._camera_yaw_offset
            half_width_angle = max(
                math.radians(1.0),
                (max(float(bw_), 1.0) / max(float(w), 1.0)) * (self._camera_hfov / 2.0),
            )
            
            # ============================================
            # CAMERA-BASED DEPTH ESTIMATION (Pinhole Model)
            # ============================================
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
            focal_px = image_center_x / max(math.tan(self._camera_hfov / 2.0), 0.01)
            camera_depth = (real_h * focal_px) / max(float(bh_), 10.0)
            camera_depth = max(0.5, min(camera_depth, 25.0))

            lidar_depth = self._estimate_lidar_depth(yaw, half_width_angle)
            max_lidar_diff = self._lidar_camera_max_diff if is_dynamic else self._static_lidar_camera_max_diff
            
            if is_truncated and lidar_depth is not None:
                depth = lidar_depth
                depth_source = "lidar_truncated_override"
            elif (
                self._trust_lidar_depth
                and lidar_depth is not None
                and abs(lidar_depth - camera_depth) <= max_lidar_diff
            ):
                depth = lidar_depth
                depth_source = "lidar"
            else:
                depth = camera_depth
                depth_source = "camera" if lidar_depth is None else "camera_lidar_rejected"
                    
            mx = depth * math.cos(yaw)
            my = depth * math.sin(yaw)
            
            # Transform to map frame with fallback
            pt_local = PointStamped()
            pt_local.header.frame_id = "camera_link"
            pt_local.header.stamp = msg.header.stamp
            pt_local.point.x = mx
            pt_local.point.y = my
            pt_local.point.z = 0.5 + ((cid % 5) * 0.15)
            
            pt_global = None
            for target_frame in ['map', 'odom']:
                try:
                    pt_global = self._tf_buffer.transform(pt_local, target_frame, rclpy.duration.Duration(seconds=0.3))
                    break
                except Exception:
                    pass
            
            if pt_global is None:
                # Fallback: use raw local coordinates
                pt_global = pt_local
                pt_global.header.frame_id = 'map'
                self.get_logger().warn(f'TF unavailable, placing {label} with raw coords')
            
            px, py = pt_global.point.x, pt_global.point.y
            self.get_logger().info(
                f'PLACED {label} at ({px:.2f}, {py:.2f}) yaw={math.degrees(yaw):.1f}deg '
                f'depth={depth:.2f} source={depth_source} camera_depth={camera_depth:.2f} '
                f'lidar_depth={lidar_depth if lidar_depth is not None else -1.0:.2f}'
            )
            
            final_label, px, py = self._stabilize_object(label, px, py, conf, now, is_dynamic)

            label_text = final_label
            marker_color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0) if is_dynamic else ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            anchor_scale = 0.35 if is_dynamic else 0.20
            label_height = 0.75 if is_dynamic else 0.55
            marker_ns_prefix = "yolo_dynamic" if is_dynamic else "yolo_static"

            pin_marker = Marker()
            pin_marker.header.frame_id = 'map'
            pin_marker.header.stamp = now_msg
            pin_marker.ns = f"{marker_ns_prefix}_pins"
            pin_marker.id = marker_seq
            marker_seq += 1
            pin_marker.type = Marker.CYLINDER
            pin_marker.action = Marker.ADD
            pin_marker.pose.position.x = px
            pin_marker.pose.position.y = py
            pin_marker.pose.position.z = label_height / 2.0
            pin_marker.scale.x = 0.08 if is_dynamic else 0.05
            pin_marker.scale.y = 0.08 if is_dynamic else 0.05
            pin_marker.scale.z = label_height
            pin_marker.color = marker_color
            pin_marker.lifetime = marker_lifetime
            current_markers.append(pin_marker)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now_msg
            marker.ns = f"{marker_ns_prefix}_labels"
            marker.id = marker_seq
            marker_seq += 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = px
            marker.pose.position.y = py
            marker.pose.position.z = label_height + 0.20
            marker.scale.z = 0.35 if is_dynamic else 0.28
            marker.color = marker_color
            marker.text = label_text
            marker.lifetime = marker_lifetime
            current_markers.append(marker)

            dot_marker = Marker()
            dot_marker.header.frame_id = 'map'
            dot_marker.header.stamp = now_msg
            dot_marker.ns = f"{marker_ns_prefix}_anchors"
            dot_marker.id = marker_seq
            marker_seq += 1
            dot_marker.type = Marker.SPHERE
            dot_marker.action = Marker.ADD
            dot_marker.pose.position.x = px
            dot_marker.pose.position.y = py
            dot_marker.pose.position.z = 0.12
            dot_marker.scale.x = anchor_scale
            dot_marker.scale.y = anchor_scale
            dot_marker.scale.z = anchor_scale
            dot_marker.color = marker_color
            dot_marker.lifetime = marker_lifetime
            current_markers.append(dot_marker)

            self.get_logger().info(
                f'LIVE MARKER: {label_text} at ({px:.2f}, {py:.2f}) dynamic={is_dynamic}',
                throttle_duration_sec=0.5,
            )

        # Update hazard history for the next frame
        self._hazard_history = current_hazards

        self._marker_pub.publish(MarkerArray(markers=current_markers))

        self._draw_cached_boxes(frame)

    def _draw_cached_boxes(self, frame):
        for (x1, y1, x2, y2, color, label_text) in getattr(self, '_cached_boxes', []):
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness=2)
            (tw, th), bl = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            ly = max(y1, th + bl + 4)
            cv2.rectangle(frame, (x1, ly - th - bl - 4), (x1 + tw, ly), color, cv2.FILLED)
            cv2.putText(frame, label_text, (x1, ly - bl - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
            
        if self._show_window:
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
