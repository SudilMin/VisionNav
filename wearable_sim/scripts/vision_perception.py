#!/usr/bin/env python3
"""
vision_perception.py
====================
ROS 2 Jazzy – Wearable Blind-Assist Vision Node  (OpenCV 5 / ONNX edition)

Tesla AI-Grade Dual-Mode Perception System:
  INDOOR  – Persistent spatial memory map, scene recall, object finding
  OUTDOOR – Forward-only collision avoidance, no memory, maximum responsiveness
"""

import os
os.environ["QT_QPA_PLATFORM"]  = "xcb"          # force X11/XWayland
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.fonts=false"

import cv2
import json
import math
import time
import numpy as np
import threading
import copy
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

INPUT_W = INPUT_H = 640

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MODEL_PATH  = os.path.join(_SCRIPT_DIR, "yolov5m.onnx")
DETECTION_FRAME_STRIDE = max(1, int(os.environ.get("WEARABLE_DETECTION_STRIDE", "3")))

NMS_THRESHOLD = 0.45

# ── MODE-SPECIFIC PERCEPTION PARAMETERS ──
# Indoor: high memory, low noise, persistent mapping
# Outdoor: short memory, responsive tracking, collision focus
MODE_PARAMS = {
    "indoor": {
        "conf_threshold":       0.35,
        "static_timeout":       120.0,   # 2 min memory while indoors
        "dynamic_timeout":      5.0,
        "static_assoc":         1.50,
        "dynamic_assoc":        2.00,
        "static_alpha":         0.03,
        "dynamic_alpha":        0.25,
        "static_bbox_alpha":    0.25,
        "dynamic_bbox_alpha":   0.45,
        "danger_distance":      1.5,     # Indoor danger threshold
        "collision_corridor_w": 0.8,     # Narrow indoor corridor
    },
    "outdoor": {
        "conf_threshold":       0.45,    # Higher threshold to reduce false positives
        "static_timeout":       3.0,     # Very short memory outdoors
        "dynamic_timeout":      2.0,
        "static_assoc":         2.00,
        "dynamic_assoc":        2.50,
        "static_alpha":         0.15,
        "dynamic_alpha":        0.35,
        "static_bbox_alpha":    0.40,
        "dynamic_bbox_alpha":   0.55,
        "danger_distance":      2.0,     # Outdoor needs earlier warnings
        "collision_corridor_w": 1.2,     # Shoulder-width walking corridor
    },
}

# Spatial memory persistence path
SPATIAL_MEMORY_DIR  = os.path.expanduser("~/.wearable_nav")
SPATIAL_MEMORY_FILE = os.path.join(SPATIAL_MEMORY_DIR, "indoor_map.json")
SPATIAL_DEDUP_RADIUS = 0.8   # meters — prevents duplicate entries for same object
MEMORY_SAVE_INTERVAL = 30.0  # seconds between auto-saves


class KalmanTracker:
    """Real-time 2D/3D spatial Kalman filter for dynamic & static obstacles.
    Tracks state vector: [X, Y, Vx, Vy] in meters and m/s, plus physical width and height.
    """
    def __init__(self, track_id: int, x: float, y: float, width: float, height: float, conf: float, now: float, is_dynamic: bool = True):
        self.id = track_id
        # State: [X (forward meters), Y (lateral meters), Vx (m/s), Vy (m/s)]
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 2.0
        self.conf = conf
        self.last_seen = now
        self.is_dynamic = is_dynamic
        self.width = max(0.1, float(width))
        self.height = max(0.1, float(height))
        self.velocity = 0.0
        self.is_moving = False
        
        self.F = np.eye(4, dtype=np.float64)
        self.H = np.zeros((2, 4), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        
        if is_dynamic:
            # Responsive process noise for moving people / vehicles
            self.R = np.eye(2, dtype=np.float64) * 0.15
            self.Q = np.eye(4, dtype=np.float64) * 0.08
        else:
            # Low noise for static obstacles (furniture, doors, walls)
            self.R = np.eye(2, dtype=np.float64) * 0.40
            self.Q = np.eye(4, dtype=np.float64) * 0.001

    def predict(self, now: float):
        dt = max(0.001, min(0.5, now - self.last_seen))
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.velocity = float(math.hypot(self.x[2], self.x[3]))

    def update(self, px: float, py: float, width: float, height: float, conf: float, now: float):
        dt = max(0.001, min(0.5, now - self.last_seen))
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        z = np.array([px, py], dtype=np.float64)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
        # Update physical dimensions with smooth exponential filter
        dim_alpha = 0.20 if self.is_dynamic else 0.10
        self.width = (1.0 - dim_alpha) * self.width + dim_alpha * max(0.05, float(width))
        self.height = (1.0 - dim_alpha) * self.height + dim_alpha * max(0.05, float(height))
        
        self.velocity = float(math.hypot(self.x[2], self.x[3]))
        self.is_moving = self.is_dynamic and (self.velocity > 0.20)
        self.conf = max(self.conf * 0.90, conf)
        self.last_seen = now

    @property
    def distance(self) -> float:
        return float(math.hypot(self.x[0], self.x[1]))


class VisionPerceptionNode(Node):
    def __init__(self, mode: str = "indoor") -> None:
        super().__init__("vision_perception")

        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_save_time = 0.0
        self._last_image_time = time.monotonic()
        self._last_memory_save = time.monotonic()

        # ── MODE INITIALIZATION ──
        self._mode = mode.lower()
        if self._mode not in MODE_PARAMS:
            self._mode = "indoor"
        self._apply_mode_params()

        cv2.setNumThreads(1)
        self._window_name = f"VisionNav AI [{self._mode.upper()}]"
        
        # Auto-detect headless environment
        has_display = "DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ
        show_window_env = os.environ.get("WEARABLE_SHOW_WINDOW", "1")
        if show_window_env != "0" and has_display:
            self._show_window = True
        else:
            self._show_window = False
            if show_window_env != "0":
                self.get_logger().warn("No display detected. Forcing WEARABLE_SHOW_WINDOW=0 (Headless Mode).")

        if self._show_window:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 800, 600)
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

        # ── CAMERA ACQUISITION MODE (Direct USB or ROS Wi-Fi) ──
        self._camera_mode = os.environ.get("WEARABLE_CAMERA_MODE", "direct").lower()
        self._camera_pub = self.create_publisher(Image, '/camera/image_raw', realtime_qos)
        self._image_pub = self.create_publisher(Image, "/vision/debug_image", 10)
        
        self._inference_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_stamp = None
        self._inference_results = None
        self._inference_busy = False
        self._inference_thread = threading.Thread(target=self._yolo_worker, daemon=True)
        self._inference_thread.start()
        
        # Thread-safe persistent HUD tracks
        self._hud_lock = threading.Lock()
        self._hud_tracks = {}
        
        self._gui_frame = None
        
        if self._camera_mode == "ros":
            # Distributed Mode: Listen to the Pi 5's camera over Wi-Fi
            self.get_logger().info("📡 Distributed Mode: Subscribing to /camera/image_raw over ROS.")
            self._image_sub = self.create_subscription(
                Image, '/camera/image_raw', self._ros_camera_callback, realtime_qos
            )
        else:
            # Direct Mode: High-speed background USB capture thread
            self._direct_cap = self._open_direct_camera()
            if self._direct_cap is not None:
                self.get_logger().info("✅ Direct camera active! High-speed V4L2 capture enabled.")
                self._cam_drain_thread = threading.Thread(
                    target=self._cam_drain_loop, daemon=True)
                self._cam_drain_thread.start()
            else:
                self.get_logger().error("Failed to open any camera. Check USB connection.")
        
        # Tracking and TF2 timer (runs in background ROS thread)
        self._tracking_timer = self.create_timer(0.05, self._tracking_callback)
        
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
        self._static_bbox_assoc_px = float(os.environ.get("WEARABLE_STATIC_BBOX_ASSOC_PX", "180.0"))
        
        self._dynamic_tracks = {}
        self._next_dynamic_track_id = {}
        self._dynamic_box_tracks = {}
        self._dynamic_bbox_assoc_px = float(os.environ.get("WEARABLE_DYNAMIC_BBOX_ASSOC_PX", "250.0"))

        # ── SPATIAL MEMORY (Indoor Mode) ──
        self._spatial_memory = {}  # key: "label_x_y" -> {class, x, y, w, h, conf, first_seen, last_seen}
        self._memory_lock = threading.Lock()
        if self._mode == "indoor":
            self._load_spatial_memory()

        # ── RUNTIME MODE SWITCHING via /perception_mode topic ──
        self._mode_sub = self.create_subscription(
            String, "/perception_mode", self._mode_callback, 10
        )

        # ── PERIODIC MEMORY SAVE (Indoor) ──
        if self._mode == "indoor":
            self._memory_save_timer = self.create_timer(MEMORY_SAVE_INTERVAL, self._auto_save_memory)

        self.get_logger().info(
            f"═══ VisionNav AI Perception Engine ═══\n"
            f"  Mode:               {self._mode.upper()}\n"
            f"  Confidence Thresh:  {self._conf_threshold:.2f}\n"
            f"  Static Timeout:     {self._static_track_timeout:.0f}s\n"
            f"  Dynamic Timeout:    {self._dynamic_track_timeout:.1f}s\n"
            f"  Danger Distance:    {self._danger_distance:.1f}m\n"
            f"  Corridor Width:     {self._collision_corridor_w:.1f}m\n"
            f"  Spatial Memory:     {'ACTIVE (' + str(len(self._spatial_memory)) + ' objects)' if self._mode == 'indoor' else 'DISABLED'}\n"
            f"  Coordinate Frame:   {'map (global)' if self._mode == 'indoor' else 'base_footprint (local)'}\n"
            f"  FSD HUD:            Active"
        )

    # ── MODE PARAMETER APPLICATION ──
    def _apply_mode_params(self):
        """Apply mode-specific parameters from MODE_PARAMS dict."""
        p = MODE_PARAMS[self._mode]
        self._conf_threshold = p["conf_threshold"]
        self._static_track_timeout = p["static_timeout"]
        self._dynamic_track_timeout = p["dynamic_timeout"]
        self._static_association_distance = p["static_assoc"]
        self._dynamic_association_distance = p["dynamic_assoc"]
        self._static_smoothing_alpha = p["static_alpha"]
        self._dynamic_smoothing_alpha = p["dynamic_alpha"]
        self._static_bbox_alpha = p["static_bbox_alpha"]
        self._dynamic_bbox_alpha = p["dynamic_bbox_alpha"]
        self._danger_distance = p["danger_distance"]
        self._collision_corridor_w = p["collision_corridor_w"]

    def _mode_callback(self, msg: String):
        """Runtime mode switching via /perception_mode topic."""
        new_mode = msg.data.strip().lower()
        if new_mode in MODE_PARAMS and new_mode != self._mode:
            old_mode = self._mode
            # Save indoor memory before switching away
            if old_mode == "indoor":
                self._save_spatial_memory()
            
            self._mode = new_mode
            self._apply_mode_params()
            
            # Load memory if switching to indoor
            if new_mode == "indoor":
                self._load_spatial_memory()
            
            # Clear short-lived tracks when switching modes
            if new_mode == "outdoor":
                self._static_tracks.clear()
                self._dynamic_tracks.clear()
                self._static_box_tracks.clear()
                self._dynamic_box_tracks.clear()
                with self._hud_lock:
                    self._hud_tracks.clear()
            
            # Update window title
            if self._show_window:
                try:
                    cv2.destroyWindow(self._window_name)
                except Exception:
                    pass
                self._window_name = f"VisionNav AI [{self._mode.upper()}]"
                cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self._window_name, 800, 600)
            
            self.get_logger().info(f"🔄 Mode switched: {old_mode.upper()} → {new_mode.upper()}")

    # ── SPATIAL MEMORY PERSISTENCE (Indoor Mode) ──
    def _load_spatial_memory(self):
        """Load persistent spatial memory from JSON file."""
        if not os.path.isfile(SPATIAL_MEMORY_FILE):
            self.get_logger().info("No existing spatial memory found. Starting fresh.")
            return
        try:
            with open(SPATIAL_MEMORY_FILE, 'r') as f:
                data = json.load(f)
            with self._memory_lock:
                self._spatial_memory = data
            self.get_logger().info(f"📂 Loaded {len(data)} objects from spatial memory.")
        except Exception as e:
            self.get_logger().warn(f"Failed to load spatial memory: {e}")

    def _save_spatial_memory(self):
        """Save spatial memory to JSON file."""
        if self._mode != "indoor":
            return
        try:
            os.makedirs(SPATIAL_MEMORY_DIR, exist_ok=True)
            with self._memory_lock:
                data = copy.deepcopy(self._spatial_memory)
            with open(SPATIAL_MEMORY_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"💾 Saved {len(data)} objects to spatial memory.")
        except Exception as e:
            self.get_logger().warn(f"Failed to save spatial memory: {e}")

    def _auto_save_memory(self):
        """Timer callback to auto-save spatial memory periodically."""
        self._save_spatial_memory()

    def _register_to_memory(self, label: str, px: float, py: float, width: float, height: float, conf: float, now_wall: float):
        """Register or update an object in the persistent spatial memory with deduplication."""
        if self._mode != "indoor":
            return
        
        with self._memory_lock:
            # Check for existing nearby object of the same class
            best_key = None
            best_dist = float("inf")
            base_label = label.rsplit('_', 1)[0] if '_' in label else label
            
            for key, entry in self._spatial_memory.items():
                if entry.get("class", "") != base_label:
                    continue
                dist = math.hypot(px - entry["x"], py - entry["y"])
                if dist < best_dist:
                    best_dist = dist
                    best_key = key
            
            if best_key is not None and best_dist <= SPATIAL_DEDUP_RADIUS:
                # Update existing entry (re-confirmation)
                entry = self._spatial_memory[best_key]
                alpha = 0.15
                entry["x"] = (1 - alpha) * entry["x"] + alpha * px
                entry["y"] = (1 - alpha) * entry["y"] + alpha * py
                entry["w"] = (1 - alpha) * entry.get("w", width) + alpha * width
                entry["h"] = (1 - alpha) * entry.get("h", height) + alpha * height
                entry["conf"] = max(entry.get("conf", 0), conf)
                entry["last_seen"] = now_wall
                entry["seen_count"] = entry.get("seen_count", 1) + 1
            else:
                # New object — create entry
                key = f"{base_label}_{px:.1f}_{py:.1f}_{now_wall:.0f}"
                self._spatial_memory[key] = {
                    "class": base_label,
                    "x": px,
                    "y": py,
                    "w": width,
                    "h": height,
                    "conf": conf,
                    "first_seen": now_wall,
                    "last_seen": now_wall,
                    "seen_count": 1,
                }

    def _open_direct_camera(self):
        """Open the USB camera directly with low-latency V4L2 backend."""
        for index in range(0, 10):
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f"Camera opened at /dev/video{index} via V4L2 (640x480 @ 30fps)")
                    return cap
            cap.release()
        return None

    def _cam_drain_loop(self):
        """Dedicated high-speed thread: captures frames with 0ms latency."""
        last_ros_pub = 0.0
        while rclpy.ok():
            if self._direct_cap is None or not self._direct_cap.isOpened():
                time.sleep(0.01)
                continue
            
            grabbed = self._direct_cap.grab()
            if not grabbed:
                time.sleep(0.001)
                continue
            ret, frame = self._direct_cap.retrieve()
            if not ret or frame is None:
                continue
            
            self._frame_count += 1
            now_mono = time.monotonic()
            self._last_image_time = now_mono
            
            # 1. Update GUI display frame for main-thread rendering
            self._gui_frame = frame
            
            # 2. Feed freshest frame to YOLO worker (non-blocking)
            with self._inference_lock:
                if not self._inference_busy:
                    self._latest_frame = frame.copy()
                    self._latest_frame_stamp = self.get_clock().now().to_msg()
            
            # 3. Throttled ROS2 publisher (5Hz, only when subscribed)
            if now_mono - last_ros_pub >= 0.20:
                try:
                    if rclpy.ok() and self._camera_pub.get_subscription_count() > 0:
                        last_ros_pub = now_mono
                        msg = Image()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "camera_link"
                        msg.height, msg.width = frame.shape[:2]
                        msg.encoding = "bgr8"
                        msg.step = frame.shape[1] * 3
                        msg.data = np.ascontiguousarray(frame).tobytes()
                        self._camera_pub.publish(msg)
                except Exception:
                    pass

    def _ros_camera_callback(self, msg: Image) -> None:
        """Callback for Distributed Mode: Receives image over Wi-Fi."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return
            
        self._frame_count += 1
        now_mono = time.monotonic()
        self._last_image_time = now_mono
        self._gui_frame = frame
        
        with self._inference_lock:
            if not self._inference_busy:
                self._latest_frame = frame.copy()
                self._latest_frame_stamp = msg.header.stamp

    def _scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    @staticmethod
    def _angle_wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _estimate_lidar_depth(self, left_angle: float, right_angle: float) -> float | None:
        """Extract foreground surface depth from LiDAR multi-ray scan cone across the bounding box."""
        if self._latest_scan is None:
            return None

        scan = self._latest_scan
        if scan.angle_increment == 0.0:
            return None

        min_ang = self._angle_wrap(min(left_angle, right_angle) + self._lidar_yaw_offset)
        max_ang = self._angle_wrap(max(left_angle, right_angle) + self._lidar_yaw_offset)
        
        angle_min = scan.angle_min
        angle_max = scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1)

        idx1 = int(round((min_ang - angle_min) / scan.angle_increment))
        idx2 = int(round((max_ang - angle_min) / scan.angle_increment))
        
        start_idx = max(0, min(idx1, idx2))
        end_idx = min(len(scan.ranges) - 1, max(idx1, idx2))

        valid_ranges = []
        for i in range(start_idx, end_idx + 1):
            r = scan.ranges[i]
            if scan.range_min <= r <= scan.range_max and not math.isinf(r) and not math.isnan(r):
                valid_ranges.append(r)

        if not valid_ranges:
            return None

        valid_ranges.sort()
        # 15th percentile captures the nearest physical surface of the obstacle
        p15_idx = int(len(valid_ranges) * 0.15)
        return float(valid_ranges[p15_idx])

    def _stabilize_object(
        self, label: str, px: float, py: float, width_m: float, height_m: float, conf: float, now: float, is_dynamic: bool
    ) -> tuple[str, float, float, float, float, float, bool, float]:
        """Continuous Kalman spatial tracking with position, velocity, and size estimation."""
        tracks_dict = self._dynamic_tracks if is_dynamic else self._static_tracks
        used_tracks_dict = self._dynamic_tracks_used if is_dynamic else self._static_tracks_used
        timeout = self._dynamic_track_timeout if is_dynamic else self._static_track_timeout
        association_dist = self._dynamic_association_distance if is_dynamic else self._static_association_distance
        
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
            existing_ids = {t.id for t in tracks}
            track_id = 1
            while track_id in existing_ids:
                track_id += 1
            best_track = KalmanTracker(track_id, px, py, width_m, height_m, conf, now, is_dynamic=is_dynamic)
            tracks.append(best_track)
        else:
            best_track.update(px, py, width_m, height_m, conf, now)

        used_tracks_dict.setdefault(label, set()).add(best_track.id)
        final_label = f"{label.replace(' ', '_')}_{best_track.id}"
        return (
            final_label,
            float(best_track.x[0]),
            float(best_track.x[1]),
            float(best_track.width),
            float(best_track.height),
            float(best_track.distance),
            bool(best_track.is_moving),
            float(best_track.velocity),
        )

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
            if math.hypot(cx - best_track["cx"], cy - best_track["cy"]) < 8.0:
                alpha *= 0.15
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
            if math.hypot(cx - best_track["cx"], cy - best_track["cy"]) < 10.0:
                alpha *= 0.20
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

    def _yolo_worker(self):
        while rclpy.ok():
            frame = None
            stamp = None
            with self._inference_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame
                    stamp = self._latest_frame_stamp
                    self._latest_frame = None
                    self._inference_busy = True
            
            if frame is None:
                time.sleep(0.01)
                continue
                
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

                if confidence < self._conf_threshold:
                    continue

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
                indices = cv2.dnn.NMSBoxes(boxes, confidences, self._conf_threshold, NMS_THRESHOLD)
                if len(indices) > 0:
                    indices = indices.flatten()
            else:
                indices = []
                
            with self._inference_lock:
                self._inference_results = (stamp, boxes, confidences, class_ids, indices, frame)
                self._inference_busy = False

    def _tracking_callback(self):
        now = time.monotonic()
        
        new_results = None
        with self._inference_lock:
            if self._inference_results is not None:
                new_results = self._inference_results
                self._inference_results = None
                
        if new_results is not None:
            stamp, boxes, confidences, class_ids, indices, frame = new_results
            self._process_detections(stamp, boxes, confidences, class_ids, indices, frame, now)
        else:
            # Predict step for smooth interpolation
            for label, tracks in self._dynamic_tracks.items():
                for track in tracks:
                    track.predict(now)
            for label, tracks in self._static_tracks.items():
                for track in tracks:
                    track.predict(now)
                    
        self._publish_markers(now)

    def _process_detections(self, msg_stamp, boxes, confidences, class_ids, indices, frame, now):
        h, w = frame.shape[:2]
        current_hazards = {}
        
        self._static_tracks_used = {}
        self._static_box_tracks_used = {}
        self._dynamic_tracks_used = {}
        self._dynamic_box_tracks_used = {}
        
        # Thread-safe persistent HUD tracks cache with temporal hysteresis
        with self._hud_lock:
            # Purge expired tracks
            hud_timeout_dyn = 1.2 if self._mode == "indoor" else 0.8
            hud_timeout_sta = 3.0 if self._mode == "indoor" else 1.5
            self._hud_tracks = {
                k: v for k, v in self._hud_tracks.items()
                if now - v['last_seen'] <= (hud_timeout_dyn if v['is_dynamic'] else hud_timeout_sta)
            }
        
        # Pinhole Camera Intrinsics
        focal_px = (w / 2.0) / max(math.tan(self._camera_hfov / 2.0), 0.01)
        focal_py = focal_px  # Square pixels standard
        image_center_x = w / 2.0
        image_center_y = h / 2.0

        # Track collision corridor threats (outdoor mode)
        corridor_threats = []

        for idx in indices:
            raw_x1, raw_y1, raw_bw, raw_bh = boxes[idx]
            x1, y1, bw_, bh_ = raw_x1, raw_y1, raw_bw, raw_bh
            x2, y2 = x1 + bw_, y1 + bh_
            
            cid = class_ids[idx]
            conf = confidences[idx]
            raw_label = COCO_CLASSES[cid]
            
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
            
            cx = x1 + bw_ / 2.0
            cy = y1 + bh_ / 2.0

            # Compute angular frustum for LiDAR query
            angle_center = ((cx - image_center_x) / focal_px)
            angle_left = ((x1 - image_center_x) / focal_px)
            angle_right = (((x1 + bw_) - image_center_x) / focal_px)

            if self._mirror_camera_x:
                angle_center = -angle_center
                angle_left = -angle_left
                angle_right = -angle_right

            yaw = self._camera_yaw_offset - angle_center
            yaw_left = self._camera_yaw_offset - angle_left
            yaw_right = self._camera_yaw_offset - angle_right

            # ── 1. SENSOR DEPTH ESTIMATION (LiDAR + Pinhole Optics) ──
            lidar_depth = self._estimate_lidar_depth(yaw_left, yaw_right)
            
            # Optical baseline from pinhole projection
            elevation_rad = math.atan2(max(1.0, y2 - image_center_y), focal_py)
            optical_ground_depth = 1.30 / max(math.tan(elevation_rad), 0.05) if y2 > image_center_y + 20 else 3.5

            box_coverage = (bw_ * bh_) / float(max(w * h, 1))

            if lidar_depth is not None and (0.25 <= lidar_depth <= 16.0):
                if box_coverage > 0.10 and y2 > h * 0.70 and lidar_depth > optical_ground_depth * 1.5:
                    depth = min(lidar_depth, optical_ground_depth)
                else:
                    depth = lidar_depth
            else:
                depth = optical_ground_depth

            depth = max(0.35, min(depth, 15.0))

            # ── 2. ACTUAL PHYSICAL SIZE ESTIMATION (Width & Height in meters) ──
            width_meters = depth * (bw_ / focal_px)
            height_meters = depth * (bh_ / focal_py)
            width_meters = max(0.05, min(width_meters, 5.0))
            height_meters = max(0.05, min(height_meters, 4.0))

            # ── 3. 3D POSITION ──
            mx = depth * math.cos(yaw)
            my = depth * math.sin(yaw)

            # ── 4. SPATIAL KALMAN TRACKING & VELOCITY ESTIMATION ──
            if self._mode == "indoor":
                # Indoor: transform to map frame for persistent spatial registration
                pt_local = PointStamped()
                pt_local.header.frame_id = "base_footprint"
                pt_local.header.stamp = msg_stamp
                pt_local.point.x = mx
                pt_local.point.y = my
                pt_local.point.z = 0.0
                
                pt_global = None
                for target_frame in ['map', 'odom']:
                    try:
                        pt_global = self._tf_buffer.transform(pt_local, target_frame, rclpy.duration.Duration(seconds=0.3))
                        break
                    except Exception:
                        pass
                
                if pt_global is None:
                    continue
                    
                px, py = pt_global.point.x, pt_global.point.y
            else:
                # Outdoor: stay in base_footprint (no TF2 needed, minimum latency)
                px, py = mx, my

            final_label, kx, ky, kw, kh, kdist, is_moving, vel = self._stabilize_object(
                label, px, py, width_meters, height_meters, conf, now, is_dynamic
            )

            # Directional relative position for blind assistance
            lat_offset = my  # +Y is Left, -Y is Right
            if lat_offset > 0.35:
                rel_pos_text = f"LEFT {abs(lat_offset):.1f}m"
            elif lat_offset < -0.35:
                rel_pos_text = f"RIGHT {abs(lat_offset):.1f}m"
            else:
                rel_pos_text = "CENTER"

            motion_text = f"MOVING {vel:.1f}m/s" if is_moving else "STATIONARY"

            # ── OUTDOOR: Collision corridor check ──
            if self._mode == "outdoor":
                half_corridor = self._collision_corridor_w / 2.0
                if abs(my) < half_corridor and mx > 0 and mx < self._danger_distance * 1.5:
                    corridor_threats.append((label, depth, rel_pos_text))

            # ── STORE IN PERSISTENT HUD TRACKS (ZERO BLINKING) ──
            track_key = f"{final_label}"
            with self._hud_lock:
                prev_hud = self._hud_tracks.get(track_key)
                if prev_hud is not None:
                    alpha = 0.35 if is_dynamic else 0.15
                    x1_s = int(round((1 - alpha) * prev_hud['x1'] + alpha * x1))
                    y1_s = int(round((1 - alpha) * prev_hud['y1'] + alpha * y1))
                    x2_s = int(round((1 - alpha) * prev_hud['x2'] + alpha * (x1 + bw_)))
                    y2_s = int(round((1 - alpha) * prev_hud['y2'] + alpha * (y1 + bh_)))
                    depth_s = (1 - alpha) * prev_hud['depth'] + alpha * depth
                else:
                    x1_s, y1_s, x2_s, y2_s, depth_s = x1, y1, x1 + bw_, y1 + bh_, depth

                self._hud_tracks[track_key] = {
                    'x1': x1_s, 'y1': y1_s, 'x2': x2_s, 'y2': y2_s,
                    'label': label, 'conf': conf, 'is_dynamic': is_dynamic,
                    'depth': depth_s, 'cx': cx, 'img_w': w,
                    'kw': kw, 'kh': kh, 'is_moving': is_moving, 'vel': vel,
                    'rel_pos': rel_pos_text, 'last_seen': now
                }

            # ── 5. STRUCTURED ASSISTIVE HAZARD COMMUNICATION ──
            if depth_s < self._danger_distance * 2.0:
                severity = "DANGER" if depth_s < self._danger_distance else "WARNING"
                hazard_msg = f"[{severity}] {label} at {depth_s:.1f}m {rel_pos_text}, size {kw:.1f}x{kh:.1f}m, {motion_text}"
                self._hazard_pub.publish(String(data=hazard_msg))

            # ── 6. INDOOR: Register to persistent spatial memory ──
            if self._mode == "indoor":
                self._register_to_memory(final_label, kx, ky, kw, kh, conf, time.time())

            if raw_label in self._hazard_classes:
                area = bw_ * bh_
                current_hazards[final_label] = (cx, cy, area, now)

        # ── OUTDOOR: Publish corridor collision summary ──
        if self._mode == "outdoor" and corridor_threats:
            closest = min(corridor_threats, key=lambda t: t[1])
            collision_msg = f"[COLLISION] {closest[0]} blocking path at {closest[1]:.1f}m {closest[2]} — STOP or TURN"
            self._hazard_pub.publish(String(data=collision_msg))

        self._hazard_history = current_hazards
        
    def _publish_markers(self, now):
        from visualization_msgs.msg import Marker, MarkerArray
        current_markers = []
        now_msg = self.get_clock().now().to_msg()
        marker_lifetime = rclpy.duration.Duration(seconds=2.5).to_msg()
        
        marker_frame = 'map' if self._mode == 'indoor' else 'base_footprint'
        
        for label, tracks in self._dynamic_tracks.items():
            for track in tracks:
                if now - track.last_seen > self._dynamic_track_timeout:
                    continue
                final_label = f"{label.replace(' ', '_')}_{track.id}"
                self._add_track_marker(
                    current_markers, track, final_label, now_msg, marker_lifetime, True, marker_frame
                )
                
        for label, tracks in self._static_tracks.items():
            for track in tracks:
                if now - track.last_seen > self._static_track_timeout:
                    continue
                final_label = f"{label.replace(' ', '_')}_{track.id}"
                self._add_track_marker(
                    current_markers, track, final_label, now_msg, marker_lifetime, False, marker_frame
                )

        # ── INDOOR: Also publish remembered objects from spatial memory ──
        if self._mode == "indoor":
            mem_lifetime = rclpy.duration.Duration(seconds=5.0).to_msg()
            with self._memory_lock:
                for key, entry in self._spatial_memory.items():
                    # Skip objects that are currently being tracked live
                    # (they already have markers from the loop above)
                    base_class = entry.get("class", "")
                    is_live = False
                    for lbl, tracks in self._static_tracks.items():
                        if lbl == base_class:
                            for t in tracks:
                                if math.hypot(t.x[0] - entry["x"], t.x[1] - entry["y"]) < SPATIAL_DEDUP_RADIUS:
                                    is_live = True
                                    break
                    if is_live:
                        continue
                    
                    # Ghost marker for remembered (not currently visible) objects
                    mem_id = abs(hash(key)) % 1000000
                    
                    # Text label
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = now_msg
                    marker.ns = "memory_labels"
                    marker.id = mem_id
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.action = Marker.ADD
                    marker.pose.position.x = entry["x"]
                    marker.pose.position.y = entry["y"]
                    marker.pose.position.z = 0.35
                    marker.scale.z = 0.20
                    marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.5)  # Translucent grey
                    marker.text = f"[MEM] {base_class}"
                    marker.lifetime = mem_lifetime
                    current_markers.append(marker)
                    
                    # Ghost sphere
                    dot = Marker()
                    dot.header.frame_id = 'map'
                    dot.header.stamp = now_msg
                    dot.ns = "memory_anchors"
                    dot.id = mem_id + 1
                    dot.type = Marker.SPHERE
                    dot.action = Marker.ADD
                    dot.pose.position.x = entry["x"]
                    dot.pose.position.y = entry["y"]
                    dot.pose.position.z = 0.10
                    dot.scale.x = 0.15
                    dot.scale.y = 0.15
                    dot.scale.z = 0.15
                    dot.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.35)
                    dot.lifetime = mem_lifetime
                    current_markers.append(dot)

        self._marker_pub.publish(MarkerArray(markers=current_markers))
        
    def _add_track_marker(self, current_markers, track, label_text, now_msg, marker_lifetime, is_dynamic, frame_id):
        from std_msgs.msg import ColorRGBA
        from visualization_msgs.msg import Marker
        px, py = track.x[0], track.x[1]
        
        # Distance-weighted color: closer objects are more vivid
        if is_dynamic:
            marker_color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=1.0)
        else:
            marker_color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=1.0)
        
        anchor_scale = 0.35 if is_dynamic else 0.20
        marker_ns_prefix = "yolo_dynamic" if is_dynamic else "yolo_static"
        
        class_base = abs(hash(label_text.rsplit('_', 1)[0])) % 100000
        stable_id = int(class_base * 100 + (track.id % 100) * 2)

        # Clean, minimal 3D RViz label (Object name only)
        display_str = label_text

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now_msg
        marker.ns = f"{marker_ns_prefix}_labels"
        marker.id = stable_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = 0.40
        marker.scale.z = 0.25
        marker.color = marker_color
        marker.text = display_str
        marker.lifetime = marker_lifetime
        current_markers.append(marker)

        # Clean 3D Object Sphere on the map
        dot_marker = Marker()
        dot_marker.header.frame_id = frame_id
        dot_marker.header.stamp = now_msg
        dot_marker.ns = f"{marker_ns_prefix}_anchors"
        dot_marker.id = stable_id + 1
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

    def _draw_cached_boxes(self, frame):
        """Tesla FSD-style detection HUD with proximity colors, corner brackets, and assistive telemetry."""
        h, w = frame.shape[:2]
        with self._hud_lock:
            hud_tracks = list(self._hud_tracks.values())
        now = time.monotonic()
        
        def get_proximity_color(depth):
            if depth is None:
                return (200, 200, 200)      # Grey
            if depth < 1.0:
                return (0, 0, 255)          # RED: DANGER (<1m)
            elif depth < 2.0:
                return (0, 128, 255)        # ORANGE: CAUTION (1-2m)
            elif depth < 3.0:
                return (0, 255, 255)        # YELLOW: NEAR (2-3m)
            else:
                return (0, 255, 0)          # GREEN: SAFE (>3m)
        
        danger_count = 0
        moving_count = 0
        static_count = 0
        dynamic_count = 0
        
        # ── OUTDOOR: Draw collision corridor overlay ──
        if self._mode == "outdoor":
            corridor_x_center = w // 2
            corridor_half_px = int(w * (self._collision_corridor_w / 2.0) / 3.0)  # approximate pixels
            overlay = frame.copy()
            cv2.rectangle(overlay,
                          (corridor_x_center - corridor_half_px, h // 3),
                          (corridor_x_center + corridor_half_px, h),
                          (0, 255, 100), cv2.FILLED)
            cv2.addWeighted(overlay, 0.08, frame, 0.92, 0, frame)
            # Corridor edge lines
            cv2.line(frame, (corridor_x_center - corridor_half_px, h // 3),
                     (corridor_x_center - corridor_half_px, h), (0, 255, 100), 1, cv2.LINE_AA)
            cv2.line(frame, (corridor_x_center + corridor_half_px, h // 3),
                     (corridor_x_center + corridor_half_px, h), (0, 255, 100), 1, cv2.LINE_AA)
        
        for track_data in hud_tracks:
            hud_timeout = 1.2 if self._mode == "indoor" else 0.8
            sta_timeout = 2.5 if self._mode == "indoor" else 1.5
            if now - track_data['last_seen'] > (hud_timeout if track_data['is_dynamic'] else sta_timeout):
                continue
                
            x1 = track_data['x1']
            y1 = track_data['y1']
            x2 = track_data['x2']
            y2 = track_data['y2']
            label = track_data['label']
            conf = track_data['conf']
            is_dynamic = track_data['is_dynamic']
            depth = track_data['depth']
            kw = track_data['kw']
            kh = track_data['kh']
            is_moving = track_data['is_moving']
            vel = track_data['vel']
            rel_pos = track_data['rel_pos']
            
            color = get_proximity_color(depth)
            
            if is_dynamic:
                dynamic_count += 1
            else:
                static_count += 1
            
            if depth is not None and depth < self._danger_distance:
                danger_count += 1
            if is_moving:
                moving_count += 1
            
            # ── 1. ULTRA-FAST ROI DANGER OVERLAY ──
            if depth is not None and depth < self._danger_distance:
                y1_i, y2_i = int(max(0, y1)), int(min(h, y2))
                x1_i, x2_i = int(max(0, x1)), int(min(w, x2))
                if y2_i > y1_i and x2_i > x1_i:
                    roi = frame[y1_i:y2_i, x1_i:x2_i]
                    color_rect = np.full_like(roi, color)
                    cv2.addWeighted(color_rect, 0.15, roi, 0.85, 0, roi)
            
            # ── 2. BOUNDING BOX ──
            thickness = 3 if (is_dynamic or is_moving) else 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            
            # ── 3. TESLA CORNER BRACKETS ──
            corner_len = min(20, max(6, (x2 - x1) // 4), max(6, (y2 - y1) // 4))
            cv2.line(frame, (x1, y1), (x1 + corner_len, y1), color, 3)
            cv2.line(frame, (x1, y1), (x1, y1 + corner_len), color, 3)
            cv2.line(frame, (x2, y1), (x2 - corner_len, y1), color, 3)
            cv2.line(frame, (x2, y1), (x2, y1 + corner_len), color, 3)
            cv2.line(frame, (x1, y2), (x1 + corner_len, y2), color, 3)
            cv2.line(frame, (x1, y2), (x1, y2 - corner_len), color, 3)
            cv2.line(frame, (x2, y2), (x2 - corner_len, y2), color, 3)
            cv2.line(frame, (x2, y2), (x2, y2 - corner_len), color, 3)
            
            # ── 4. PRIMARY ASSISTIVE LABEL ──
            type_tag = "DYN" if is_dynamic else "STA"
            motion_tag = f"MOVING {vel:.1f}m/s" if is_moving else "STATIC"
            if depth is not None:
                info_text = f"{label} {depth:.1f}m {rel_pos} [{type_tag}]"
            else:
                info_text = f"{label} {rel_pos} [{type_tag}]"
            
            (tw, th), bl = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            label_y = max(y1, th + bl + 6)
            cv2.rectangle(frame, (x1, label_y - th - bl - 6), (x1 + tw + 8, label_y), color, cv2.FILLED)
            cv2.putText(frame, info_text, (x1 + 4, label_y - bl - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
            
            # ── 5. SECONDARY ASSISTIVE LABEL ──
            size_text = f"W:{kw:.1f}m H:{kh:.1f}m | {motion_tag}"
            (stw, sth), sbl = cv2.getTextSize(size_text, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
            sub_y = label_y + sth + sbl + 6
            if sub_y < y2:
                cv2.rectangle(frame, (x1, label_y + 2), (x1 + stw + 6, sub_y), (30, 30, 30), cv2.FILLED)
                cv2.putText(frame, size_text, (x1 + 3, sub_y - sbl - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255) if is_moving else (220, 220, 220), 1, cv2.LINE_AA)
            
            # ── 6. CONFIDENCE BAR ──
            bar_w = int((x2 - x1) * max(0.0, min(1.0, conf)))
            cv2.rectangle(frame, (x1, y2 + 2), (x1 + bar_w, y2 + 6), color, cv2.FILLED)
        
        # ══════════════════════════════════════════════════════════════════════
        # ── TESLA HUD STATUS BAR (TOP) ──
        # ══════════════════════════════════════════════════════════════════════
        cv2.rectangle(frame, (0, 0), (w, 32), (20, 20, 20), cv2.FILLED)
        
        now = time.monotonic()
        fps = getattr(self, '_display_fps', 0.0)
        last_fps_time = getattr(self, '_last_fps_time', now)
        fps_frame_count = getattr(self, '_fps_frame_count', 0) + 1
        if now - last_fps_time >= 1.0:
            fps = fps_frame_count / (now - last_fps_time)
            self._display_fps = fps
            self._fps_frame_count = 0
            self._last_fps_time = now
        else:
            self._fps_frame_count = fps_frame_count
        cv2.putText(frame, f"FPS: {fps:.0f}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, f"STATIC: {static_count}", (110, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, f"DYNAMIC: {dynamic_count}", (230, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 128, 255), 1, cv2.LINE_AA)
        
        if moving_count > 0:
            cv2.putText(frame, f"MOVING: {moving_count}", (370, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        
        if danger_count > 0:
            alert_text = f"!! DANGER: {danger_count} CLOSE !!"
            cv2.putText(frame, alert_text, (w - 290, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "PATH CLEAR", (w - 130, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
        
        # ── MODE INDICATOR (BOTTOM BAR) ──
        if self._mode == "indoor":
            mem_count = len(self._spatial_memory) if hasattr(self, '_spatial_memory') else 0
            mode_text = f"MODE: INDOOR [MAPPING] | Memory: {mem_count} objects"
            mode_color = (255, 200, 0)
        else:
            mode_text = "MODE: OUTDOOR [NAV] | Collision Avoidance Active"
            mode_color = (0, 200, 255)
        
        cv2.putText(frame, mode_text, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1, cv2.LINE_AA)

def main(args=None) -> None:
    rclpy.init(args=args)
    
    # Parse mode from environment variable
    mode = os.environ.get("WEARABLE_MODE", "indoor").lower()
    if mode not in MODE_PARAMS:
        mode = "indoor"
    
    node = VisionPerceptionNode(mode=mode)
    
    # ── ZERO-LAG ARCHITECTURE: Offload ROS 2 spin to background thread ──
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    # ── MAIN THREAD: High-speed OpenCV GUI loop (maximum FPS, 0ms delay) ──
    try:
        while rclpy.ok():
            frame = node._gui_frame
            if frame is not None:
                display_frame = frame.copy()
                node._draw_cached_boxes(display_frame)
                cv2.imshow(node._window_name, display_frame)
            else:
                time.sleep(0.005)
                continue
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Save spatial memory on shutdown (indoor mode)
        if node._mode == "indoor":
            node._save_spatial_memory()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
