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
from sensor_msgs.msg import Image, LaserScan, CompressedImage
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

NMS_THRESHOLD = 0.15

# ── KNOWN REAL-WORLD MAXIMUM PHYSICAL SIZES (width_m, height_m) ──
# Used to clamp estimated sizes to prevent wildly oversized RViz markers
# when LiDAR depth overshoots past the object.
OBJECT_MAX_SIZES = {
    # Small objects
    "mouse":        (0.12, 0.05),
    "remote":       (0.20, 0.06),
    "cell phone":   (0.10, 0.18),
    "smartphone":   (0.10, 0.18),
    "toothbrush":   (0.03, 0.20),
    "scissors":     (0.10, 0.20),
    "fork":         (0.03, 0.20),
    "knife":        (0.03, 0.25),
    "spoon":        (0.04, 0.20),
    "cup":          (0.12, 0.15),
    "glass":        (0.10, 0.20),
    "bottle":       (0.10, 0.30),
    "apple":        (0.10, 0.10),
    "orange":       (0.10, 0.10),
    "banana":       (0.05, 0.22),
    "book":         (0.25, 0.35),
    "clock":        (0.35, 0.35),
    "vase":         (0.20, 0.40),
    "keyboard":     (0.50, 0.20),
    # Medium objects
    "laptop":       (0.40, 0.30),
    "bowl":         (0.25, 0.12),
    "backpack":     (0.40, 0.55),
    "handbag":      (0.40, 0.35),
    "umbrella":     (0.15, 1.00),
    "teddy bear":   (0.40, 0.50),
    "hair dryer":   (0.15, 0.30),
    "toaster":      (0.30, 0.25),
    "microwave":    (0.55, 0.35),
    "plant":        (0.50, 0.80),
    "potted plant": (0.50, 0.80),
    "toilet":       (0.50, 0.70),
    "sink":         (0.60, 0.30),
    "tv":           (1.20, 0.80),
    "television":   (1.20, 0.80),
    "chair":        (0.60, 1.20),
    # Large objects
    "table":        (1.80, 0.85),
    "dining table": (1.80, 0.85),
    "sofa":         (2.20, 1.00),
    "couch":        (2.20, 1.00),
    "bed":          (2.20, 1.00),
    "refrigerator": (0.80, 1.80),
    "oven":         (0.70, 0.90),
    # Dynamic objects
    "person":       (0.70, 1.90),
    "bicycle":      (1.80, 1.10),
    "car":          (4.50, 1.60),
    "motorcycle":   (2.20, 1.30),
    "bus":          (12.0, 3.50),
    "truck":        (8.00, 3.50),
    "dog":          (0.80, 0.70),
    "cat":          (0.50, 0.35),
}
# Default max size for unknown objects
OBJECT_MAX_SIZE_DEFAULT = (1.50, 1.50)

# ── SEMANTIC 2D ASPECT RATIO LIMITS (Height / Width) ──
# Prevents YOLO from drawing massive vertical boxes around flat objects (like confusing a wall for a laptop)
OBJECT_MAX_ASPECT_RATIOS = {
    "laptop": 0.85, "mouse": 0.60, "keyboard": 0.40, 
    "tv": 0.85, "television": 0.85, "bed": 0.70, 
    "couch": 0.85, "sofa": 0.85, "car": 0.80, 
    "truck": 0.80, "bowl": 0.60, "sink": 0.70,
    "book": 1.50, "cell phone": 2.0
}

# ── SEMANTIC Z-AXIS ELEVATIONS (Meters) ──
# If YOLO doesn't detect the table, we still want desktop objects to hover at table height
# so the 3D map is realistic instead of placing laptops and coffee cups on the floor.
OBJECT_ELEVATIONS = {
    "laptop": 0.75, "mouse": 0.75, "keyboard": 0.75, "cup": 0.75, 
    "bowl": 0.75, "bottle": 0.75, "apple": 0.75, "orange": 0.75, 
    "banana": 0.75, "sandwich": 0.75, "fork": 0.75, "knife": 0.75, 
    "spoon": 0.75, "scissors": 0.75, "book": 0.75, "remote": 0.75, 
    "cell phone": 0.75, "smartphone": 0.75, "vase": 0.75,
    "microwave": 0.90, "toaster": 0.90, "sink": 0.85,
    "tv": 0.60, "television": 0.60, "clock": 1.50
}

# ── MODE-SPECIFIC PERCEPTION PARAMETERS ──
# Indoor: high memory, low noise, persistent mapping
# Outdoor: short memory, responsive tracking, collision focus
MODE_PARAMS = {
    "indoor": {
        "conf_threshold":       0.50,  # Greatly increased to stop random hallucinations
        "static_timeout":       120.0,   # 2 min memory while indoors
        "dynamic_timeout":      5.0,
        "static_assoc":         3.00,    # Increased heavily to merge jittery detections
        "dynamic_assoc":        3.50,
        "static_alpha":         0.25,
        "dynamic_alpha":        0.50,
        "static_bbox_alpha":    0.65,
        "dynamic_bbox_alpha":   0.75,
        "danger_distance":      1.5,     # Indoor danger threshold
        "collision_corridor_w": 0.8,     # Narrow indoor corridor
    },
    "outdoor": {
        "conf_threshold":       0.55,    # Higher threshold to reduce false positives
        "static_timeout":       3.0,     # Very short memory outdoors
        "dynamic_timeout":      2.0,
        "static_assoc":         3.00,
        "dynamic_assoc":        3.50,
        "static_alpha":         0.30,
        "dynamic_alpha":        0.55,
        "static_bbox_alpha":    0.65,
        "dynamic_bbox_alpha":   0.75,
        "danger_distance":      2.0,     # Outdoor needs earlier warnings
        "collision_corridor_w": 1.2,     # Shoulder-width walking corridor
    },
}

# Spatial memory persistence path
SPATIAL_MEMORY_DIR  = os.path.expanduser("~/.wearable_nav")
SPATIAL_MEMORY_FILE = os.path.join(SPATIAL_MEMORY_DIR, "indoor_map.json")
SPATIAL_DEDUP_RADIUS = 2.5   # meters — heavily increased to prevent duplicate ghost markers
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
        self.dist = 0.0  # Distance from camera/system
        self.is_moving = False
        self.update_count = 0  # Track how many times this object has been observed
        
        self.F = np.eye(4, dtype=np.float64)
        self.H = np.zeros((2, 4), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        
        if is_dynamic:
            # Responsive process noise for moving people / vehicles
            self.R = np.eye(2, dtype=np.float64) * 0.15
            self.Q = np.eye(4, dtype=np.float64) * 0.08
        else:
            # Moderate measurement noise for static obstacles — responsive but stable
            self.R = np.eye(2, dtype=np.float64) * 1.0
            self.Q = np.eye(4, dtype=np.float64) * 0.005

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
        
        # --- ENHANCED STABILIZATION LOGIC ("Still Point" Anchor) ---
        if not self.is_dynamic:
            dist_moved = math.hypot(y[0], y[1])
            self.update_count += 1
            
            if self.update_count >= 5:
                # Well-established object: only accept large movements (> 1.5m)
                # that indicate the object was truly moved (e.g., someone picked up a chair).
                # Camera tilt and LiDAR jitter are always < 1.5m.
                if dist_moved < 1.5:
                    y = y * 0.0  # Fully lock position — zero innovation
                # else: let it update normally (object genuinely relocated)
            elif dist_moved < 1.0:
                # New object still stabilizing: heavily dampen small movements
                # Camera tilt typically causes < 1.0m apparent shifts
                y = y * 0.01
            # else: large movement on new object, let it update normally
        else:
            self.update_count += 1
        # ------------------------------------------------------
        
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
        # Update physical dimensions with smooth exponential filter
        # Use very low alpha for static objects so sizes lock quickly
        dim_alpha = 0.40 if self.is_dynamic else 0.25
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
            self.get_logger().info("📡 Distributed Mode: Subscribing to /camera/image_raw/compressed over ROS.")
            self._image_sub = self.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self._ros_camera_callback, realtime_qos
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
        self._mirror_camera_x = os.environ.get("WEARABLE_CAMERA_MIRROR_X", "1") == "1"  # Default ON: ROS camera is mirrored
        self._trust_lidar_depth = os.environ.get("WEARABLE_TRUST_LIDAR_DEPTH", "1") != "0"
        self._lidar_camera_max_diff = float(os.environ.get("WEARABLE_LIDAR_CAMERA_MAX_DIFF", "1.75"))
        self._static_lidar_camera_max_diff = float(os.environ.get("WEARABLE_STATIC_LIDAR_CAMERA_MAX_DIFF", "0.75"))
        self._static_tracks = {}
        self._next_static_track_id = {}
        self._static_box_tracks = {}
        self._static_bbox_assoc_px = float(os.environ.get("WEARABLE_STATIC_BBOX_ASSOC_PX", "180.0"))
        self._next_static_box_id = 0
        
        self._dynamic_tracks = {}
        self._next_dynamic_track_id = {}
        self._dynamic_box_tracks = {}
        self._dynamic_bbox_assoc_px = float(os.environ.get("WEARABLE_DYNAMIC_BBOX_ASSOC_PX", "250.0"))
        self._next_dynamic_box_id = 0

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
        """Spatial memory is now session-only. Cleared on startup."""
        self.get_logger().info("Starting with a fresh spatial memory for this session.")
        with self._memory_lock:
            self._spatial_memory = {}

    def _save_spatial_memory(self):
        """Spatial memory is session-only, no longer saving to disk to prevent stacking."""
        pass

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


    def _ros_camera_callback(self, msg: CompressedImage) -> None:
        """Callback for Distributed Mode: Receives image over Wi-Fi."""
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
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
        
        # Calculate the angular distance. We want to traverse from right to left.
        diff = left_angle - right_angle
        # If difference is negative, we crossed the PI/-PI boundary
        while diff < 0:
            diff += 2.0 * math.pi
        while diff > 2.0 * math.pi:
            diff -= 2.0 * math.pi
            
        # Sample points along the arc to avoid array index wrap-around bugs
        samples = int(max(10, min(100, diff / scan.angle_increment)))
        angle_step = diff / max(1, samples)
        
        valid_ranges = []
        for i in range(samples + 1):
            a = right_angle + i * angle_step
            
            # Normalize 'a' to match the raw scan array bounds
            while a < scan.angle_min: 
                a += 2.0 * math.pi
            while a >= scan.angle_min + 2.0 * math.pi: 
                a -= 2.0 * math.pi
                
            idx = int(round((a - scan.angle_min) / scan.angle_increment))
            if 0 <= idx < len(scan.ranges):
                r = scan.ranges[idx]
                if scan.range_min <= r <= scan.range_max and not math.isinf(r) and not math.isnan(r):
                    valid_ranges.append(r)

        if not valid_ranges:
            return None

        valid_ranges.sort()
        # 15th percentile captures the nearest physical surface of the obstacle
        p15_idx = int(len(valid_ranges) * 0.15)
        return float(valid_ranges[p15_idx])

    def _stabilize_object(
        self, label: str, px: float, py: float, width_m: float, height_m: float, conf: float, now: float, is_dynamic: bool, dist: float = 0.0
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
            assoc_d = math.hypot(px - track.x[0], py - track.x[1])
            if assoc_d < best_dist:
                best_dist = assoc_d
                best_track = track

        if best_track is None or best_dist > association_dist:
            existing_ids = {t.id for t in tracks}
            track_id = 1
            while track_id in existing_ids:
                track_id += 1
            best_track = KalmanTracker(track_id, px, py, width_m, height_m, conf, now, is_dynamic=is_dynamic)
            best_track.dist = dist
            tracks.append(best_track)
        else:
            best_track.update(px, py, width_m, height_m, conf, now)
            best_track.dist = 0.7 * best_track.dist + 0.3 * dist  # Smooth distance updates

        used_tracks_dict.setdefault(label, set()).add(best_track.id)
        final_label = f"{label.replace(' ', '_')}_{best_track.id}"
        return (
            final_label,
            float(best_track.x[0]),
            float(best_track.x[1]),
            float(best_track.width),
            float(best_track.height),
            float(best_track.dist),
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
            self._next_static_box_id += 1
            track_id = self._next_static_box_id
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
            self._next_dynamic_box_id += 1
            track_id = self._next_dynamic_box_id
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

                raw_label = COCO_CLASSES[class_id]
                
                # ── TESLA-STYLE WHITELIST ──
                # Stop AI from detecting shadows/clothes as dogs, cats, surfboards.
                # Only allow structured indoor obstacles through.
                INDOOR_WHITELIST = {
                    "person", "chair", "laptop", "bottle", "cup", "keyboard", "mouse", 
                    "cell phone", "book", "tv", "dining table", "couch", "bed", "door",
                    "refrigerator", "microwave", "oven", "sink", "vase", "clock",
                    "teddy bear", "backpack", "umbrella"
                }
                if raw_label not in INDOOR_WHITELIST:
                    continue

                cx = float(det[0]) * scale_x
                cy = float(det[1]) * scale_y
                bw_ = float(det[2]) * scale_x
                bh_ = float(det[3]) * scale_y

                x1 = max(0, int(cx - bw_ / 2))
                y1 = max(0, int(cy - bh_ / 2))
                bw_ = min(w - x1, int(bw_))
                bh_ = min(h - y1, int(bh_))
                y2_initial = y1 + bh_
                
                # ── SEMANTIC 2D ASPECT RATIO CORRECTION ──
                # Slices off hallucinated tall bounding boxes for flat objects
                # (e.g. YOLO including the pink wall in the laptop bounding box)
                raw_label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else "unknown"
                max_aspect = OBJECT_MAX_ASPECT_RATIOS.get(raw_label, 100.0)
                current_aspect = bh_ / float(max(1, bw_))
                
                if current_aspect > max_aspect:
                    bh_ = int(bw_ * max_aspect)
                    y1 = max(0, int(y2_initial - bh_))  # Anchor to the bottom (desk/floor) and slice the top off!

                boxes.append([x1, y1, bw_, bh_])
                confidences.append(confidence)
                class_ids.append(class_id)

            if boxes:
                indices = cv2.dnn.NMSBoxes(boxes, confidences, self._conf_threshold, NMS_THRESHOLD)
                if len(indices) > 0:
                    indices = indices.flatten()
                else:
                    indices = []
                    
                # ── CROSS-CLASS NMS: Remove duplicate detections of different classes ──
                # e.g. same object detected as both "laptop" AND "television"
                if len(indices) > 1:
                    keep = []
                    suppressed = set()
                    for i_pos, i_idx in enumerate(indices):
                        if i_idx in suppressed:
                            continue
                        keep.append(i_idx)
                        bx1, by1, bw1, bh1 = boxes[i_idx]
                        for j_pos in range(i_pos + 1, len(indices)):
                            j_idx = indices[j_pos]
                            if j_idx in suppressed:
                                continue
                            bx2, by2, bw2, bh2 = boxes[j_idx]
                            # Calculate IoU
                            ix1 = max(bx1, bx2)
                            iy1 = max(by1, by2)
                            ix2 = min(bx1 + bw1, bx2 + bw2)
                            iy2 = min(by1 + bh1, by2 + bh2)
                            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
                            area1 = bw1 * bh1
                            area2 = bw2 * bh2
                            union = area1 + area2 - inter
                            iou = inter / max(union, 1)
                            if iou > 0.35:
                                # Suppress the lower-confidence duplicate
                                suppressed.add(j_idx)
                    indices = keep
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
            # Purge expired tracks (Increased timeouts to prevent flickering on slower CPUs)
            hud_timeout_dyn = 3.0 if self._mode == "indoor" else 2.0
            hud_timeout_sta = 5.0 if self._mode == "indoor" else 3.0
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
                "hair drier": "hair dryer", "tv": "laptop", "fire hydrant": "hydrant",
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
            # CRITICAL FIX: The physical LiDAR is mounted with the cable pointing forward.
            # We rotated the TF map 180 degrees to face forward, which means the LiDAR's internal 
            # local Y-axis is now physically pointing Right instead of Left.
            # We MUST negate the yaw query to correctly align the camera's Left with the physical Left!
            lidar_query_left = -yaw_right
            lidar_query_right = -yaw_left
            lidar_depth = self._estimate_lidar_depth(lidar_query_left, lidar_query_right)
            
            # ── INTELLIGENT OPTICAL DEPTH (Pinhole + Size Priors) ──
            import os
            camera_height = float(os.environ.get("WEARABLE_CAMERA_HEIGHT", "0.80")) # Default 0.8m for testing on a desk
            
            # 1. Ground Plane Intersection (works well if camera is high and object is on floor)
            elevation_rad = math.atan2(max(1.0, y2 - image_center_y), focal_py)
            
            # Table Heuristic: If it's a desktop object, it's not on the floor! 
            # Subtract table height (0.75m) from camera height to get true drop distance.
            table_objects = ["laptop", "mouse", "keyboard", "cup", "bottle", "bowl", "tv", "monitor", "book", "vase", "scissors"]
            if label in table_objects and camera_height > 1.0:
                effective_h = max(0.1, camera_height - 0.75)
            else:
                effective_h = camera_height
                
            optical_ground_depth = effective_h / max(math.tan(elevation_rad), 0.05)
            
            # 2. Known-Size Prior (Crucial for objects on tables where camera is also on table)
            max_w, max_h = OBJECT_MAX_SIZES.get(label, OBJECT_MAX_SIZES.get(raw_label, OBJECT_MAX_SIZE_DEFAULT))
            typ_w = max_w * 0.75  # Assume average object is ~75% of its absolute max size
            typ_h = max_h * 0.75
            depth_from_width = (typ_w * focal_px) / max(1.0, bw_)
            depth_from_height = (typ_h * focal_py) / max(1.0, bh_)
            optical_size_depth = min(depth_from_width, depth_from_height)
            
            # If object is near horizon (y2 is close to center), ground depth goes to infinity.
            # In that case, trust the size prior!
            if y2 < image_center_y + 40:
                optical_depth = optical_size_depth
            else:
                optical_depth = min(optical_ground_depth, optical_size_depth * 1.5)

            box_coverage = (bw_ * bh_) / float(max(w * h, 1))

            if lidar_depth is not None and (0.25 <= lidar_depth <= 16.0):
                # Smart LiDAR/Optical fusion based on object class
                # Small objects: LiDAR beam often misses them and hits the wall behind
                # Large objects: LiDAR reliably hits them
                small_objects = ["bottle", "cup", "mouse", "keyboard", "cell phone", 
                                 "remote", "scissors", "book", "vase", "bowl", "apple", "orange"]
                
                if raw_label in small_objects:
                    # For small objects: trust optical depth primarily (LiDAR often overshoots)
                    depth = min(optical_depth, lidar_depth)
                elif box_coverage > 0.25 and lidar_depth > optical_depth * 2.5:
                    # Object fills the screen but LiDAR hits the wall — trust optical
                    depth = optical_depth
                else:
                    # For large objects (chair, person, door): trust LiDAR
                    depth = lidar_depth
            else:
                depth = optical_depth

            # Hard cap for LiDAR overshoots
            max_physical_width = max_w * 1.5
            max_depth_by_width = max_physical_width * focal_px / max(bw_, 1.0)
            depth = min(depth, max_depth_by_width)

            depth = max(0.35, min(depth, 10.0))

            # ── 2. ACTUAL PHYSICAL SIZE ESTIMATION (Width & Height in meters) ──
            width_meters = depth * (bw_ / focal_px)
            height_meters = depth * (bh_ / focal_py)
            width_meters = max(0.05, min(width_meters, 5.0))
            height_meters = max(0.05, min(height_meters, 4.0))
            # Per-category size clamping using known real-world dimensions
            max_w, max_h = OBJECT_MAX_SIZES.get(label, OBJECT_MAX_SIZES.get(raw_label, OBJECT_MAX_SIZE_DEFAULT))
            width_meters = max(0.05, min(width_meters, max_w))
            height_meters = max(0.05, min(height_meters, max_h))

            # ── 3. 3D POSITION ──
            mx = depth * math.cos(yaw)
            my = depth * math.sin(yaw)

            # ── 4. SPATIAL KALMAN TRACKING & VELOCITY ESTIMATION ──
            if self._mode == "indoor":
                # Indoor: transform to map frame for persistent spatial registration
                pt_local = PointStamped()
                pt_local.header.frame_id = "base_footprint"
                pt_local.header.stamp = rclpy.time.Time().to_msg()  # Use latest available TF to prevent dropped detections
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

            
            # Semantic Size Enforcement (Fixes occlusion shrinking)
            # If a chair is occluded by a desk, its bounding box is small, making it look shorter than a laptop.
            # We force known objects to their typical real-world heights.
            if label in ["chair", "person", "refrigerator", "door"]:
                height_meters = max_h  # Force to 90% of max height if occluded
                
            final_label, kx, ky, kw, kh, kdist, is_moving, vel = self._stabilize_object(
                label, px, py, width_meters, height_meters, conf, now, is_dynamic, dist=depth
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
                    alpha = 0.70 if is_dynamic else 0.55
                    x1_s = int(round((1 - alpha) * prev_hud['x1'] + alpha * x1))
                    y1_s = int(round((1 - alpha) * prev_hud['y1'] + alpha * y1))
                    x2_s = int(round((1 - alpha) * prev_hud['x2'] + alpha * (x1 + bw_)))
                    y2_s = int(round((1 - alpha) * prev_hud['y2'] + alpha * (y1 + bh_)))
                    depth_s = (1 - alpha) * prev_hud['depth'] + alpha * depth
                else:
                    x1_s, y1_s, x2_s, y2_s, depth_s = x1, y1, x1 + bw_, y1 + bh_, depth

                self._hud_tracks[track_key] = {
                    'x1': x1_s, 'y1': y1_s, 'x2': x2_s, 'y2': y2_s,
                    'label': final_label, 'conf': conf, 'is_dynamic': is_dynamic,
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
                    current_markers, track, final_label, now_msg, marker_lifetime, True, marker_frame, distance=track.dist
                )
                
        for label, tracks in self._static_tracks.items():
            for track in tracks:
                if now - track.last_seen > self._static_track_timeout:
                    continue
                final_label = f"{label.replace(' ', '_')}_{track.id}"
                self._add_track_marker(
                    current_markers, track, final_label, now_msg, marker_lifetime, False, marker_frame, distance=track.dist
                )

        # ── INDOOR: Also publish remembered objects from spatial memory ──
        if self._mode == "indoor":
            mem_lifetime = rclpy.duration.Duration(seconds=5.0).to_msg()
            with self._memory_lock:
                keys_to_delete = []
                for key, entry in self._spatial_memory.items():
                    # 1. Garbage Collection: Remove false positives (seen < 5 times and hasn't been seen in 15 seconds)
                    age = time.time() - entry["last_seen"]
                    if age > 15.0 and entry.get("seen_count", 0) < 5:
                        keys_to_delete.append(key)
                        continue
                        
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
                    mem_w = max(0.05, entry.get("w", 0.3))
                    mem_h = max(0.05, entry.get("h", 0.3))
                    
                    # Safety-net for ghost markers (clamps old bad data in JSON)
                    max_w, max_h = OBJECT_MAX_SIZES.get(base_class, OBJECT_MAX_SIZE_DEFAULT)
                    mem_w = min(mem_w, max_w)
                    mem_h = min(mem_h, max_h)
                    
                    # Text label floating above the ghost cube
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = now_msg
                    marker.ns = "memory_labels"
                    marker.id = mem_id
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.action = Marker.ADD
                    base_z = OBJECT_ELEVATIONS.get(base_class, 0.0)
                    marker.pose.position.x = entry["x"]
                    marker.pose.position.y = entry["y"]
                    marker.pose.position.z = base_z + mem_h + 0.10  # Float above the ghost cube
                    marker.scale.z = 0.18
                    marker.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=0.5)  # Translucent grey
                    marker.text = base_class
                    marker.lifetime = mem_lifetime
                    current_markers.append(marker)
                    
                    # Ghost 3D CUBE (semi-transparent, remembered object)
                    dot = Marker()
                    dot.header.frame_id = 'map'
                    dot.header.stamp = now_msg
                    dot.ns = "memory_cubes"
                    dot.id = mem_id + 1
                    dot.type = Marker.CUBE
                    dot.action = Marker.ADD
                    dot.pose.position.x = entry["x"]
                    dot.pose.position.y = entry["y"]
                    dot.pose.position.z = base_z + (mem_h / 2.0)  # Base on semantic elevation
                    dot.scale.x = mem_w
                    dot.scale.y = mem_w
                    dot.scale.z = mem_h
                    dot.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.15)
                    dot.lifetime = mem_lifetime
                    current_markers.append(dot)
                    
                    if base_z > 0.1:
                        desk = Marker()
                        desk.header.frame_id = 'map'
                        desk.header.stamp = now_msg
                        desk.ns = "memory_phantom_desks"
                        desk.id = mem_id + 2
                        desk.type = Marker.CUBE
                        desk.action = Marker.ADD
                        desk.pose.position.x = entry["x"]
                        desk.pose.position.y = entry["y"]
                        desk.pose.position.z = base_z - 0.025
                        desk.scale.x = mem_w * 1.5
                        desk.scale.y = mem_w * 1.5
                        desk.scale.z = 0.05
                        desk.color = ColorRGBA(r=0.85, g=0.85, b=0.85, a=0.2)
                        desk.lifetime = mem_lifetime
                        current_markers.append(desk)


        # Ground grid removed for cleaner RViz view

        self._marker_pub.publish(MarkerArray(markers=current_markers))
        
        # Actually delete the expired memory keys safely
        if self._mode == "indoor":
            with self._memory_lock:
                for key in keys_to_delete:
                    if key in self._spatial_memory:
                        del self._spatial_memory[key]
        
    def _add_track_marker(self, current_markers, track, label_text, now_msg, marker_lifetime, is_dynamic, frame_id, distance=0.0):
        from std_msgs.msg import ColorRGBA
        from visualization_msgs.msg import Marker
        px, py = track.x[0], track.x[1]
        obj_width = max(0.05, float(track.width))
        obj_height = max(0.05, float(track.height))
        
        # Safety-net: clamp marker sizes using known real-world maximum dimensions
        base_label = label_text.rsplit('_', 1)[0].replace('_', ' ') if '_' in label_text else label_text
        max_w, max_h = OBJECT_MAX_SIZES.get(base_label, OBJECT_MAX_SIZE_DEFAULT)
        obj_width = min(obj_width, max_w)
        obj_height = min(obj_height, max_h)
        
        # Distance-weighted color: closer objects are more vivid
        if is_dynamic:
            marker_color = ColorRGBA(r=1.0, g=0.15, b=0.15, a=0.75)
        else:
            marker_color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.65)
        
        marker_ns_prefix = "yolo_dynamic" if is_dynamic else "yolo_static"
        
        class_base = abs(hash(label_text.rsplit('_', 1)[0])) % 100000
        stable_id = int(class_base * 100 + (track.id % 100) * 2)

        # ── 3D Text label floating above the object cube ──
        display_str = f"{label_text} ({distance:.1f}m)"

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now_msg
        marker.ns = f"{marker_ns_prefix}_labels"
        marker.id = stable_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = px
        marker.pose.position.y = py
        base_z = OBJECT_ELEVATIONS.get(base_label, 0.0)
        marker.pose.position.z = base_z + obj_height + 0.15  # Float above the 3D cube
        marker.scale.z = 0.22
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = display_str
        marker.lifetime = marker_lifetime
        current_markers.append(marker)

        # ── 3D Semantic Shape on the map ──
        cube_marker = Marker()
        cube_marker.header.frame_id = frame_id
        cube_marker.header.stamp = now_msg
        cube_marker.ns = f"{marker_ns_prefix}_shapes"
        cube_marker.id = stable_id + 1
        
        # Tesla-style semantic 3D rendering (Cylinders for people, spheres for balls, cubes for furniture)
        if base_label in ["person", "bottle", "vase"]:
            cube_marker.type = Marker.CYLINDER
        elif base_label in ["sports ball", "apple", "orange", "bowl"]:
            cube_marker.type = Marker.SPHERE
        else:
            cube_marker.type = Marker.CUBE
            
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = px
        cube_marker.pose.position.y = py
        cube_marker.pose.position.z = base_z + (obj_height / 2.0)  # Add semantic elevation so laptops sit on tables
        cube_marker.scale.x = obj_width
        cube_marker.scale.y = obj_width
        cube_marker.scale.z = obj_height
        cube_marker.color = marker_color
        cube_marker.lifetime = marker_lifetime
        current_markers.append(cube_marker)
        
        # ── RESTORED: PHANTOM TABLE ──
        # If an object is floating (base_z > 0), draw a sleek table underneath it
        if base_z > 0.1:
            desk = Marker()
            desk.header.frame_id = frame_id
            desk.header.stamp = now_msg
            desk.ns = f"{marker_ns_prefix}_phantom_desks"
            desk.id = stable_id + 2
            desk.type = Marker.CUBE
            desk.action = Marker.ADD
            desk.pose.position.x = px
            desk.pose.position.y = py
            # Draw a sleek 5cm thick table surface right below the object
            desk.pose.position.z = base_z - 0.025
            desk.scale.x = obj_width * 1.5
            desk.scale.y = obj_width * 1.5
            desk.scale.z = 0.05
            # Elegant white/grey table surface
            desk.color = ColorRGBA(r=0.85, g=0.85, b=0.85, a=0.7)
            desk.lifetime = marker_lifetime
            current_markers.append(desk)
            
            # Draw a pedestal leg down to the floor
            leg = Marker()
            leg.header.frame_id = frame_id
            leg.header.stamp = now_msg
            leg.ns = f"{marker_ns_prefix}_phantom_legs"
            leg.id = stable_id + 3
            leg.type = Marker.CYLINDER
            leg.action = Marker.ADD
            leg.pose.position.x = px
            leg.pose.position.y = py
            leg.pose.position.z = (base_z - 0.05) / 2.0
            leg.scale.x = 0.1  # Thin leg
            leg.scale.y = 0.1
            leg.scale.z = base_z - 0.05
            leg.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.5)
            leg.lifetime = marker_lifetime
            current_markers.append(leg)

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
            # Increased timeouts to ensure boxes stay on screen even if YOLO takes >1 second to run
            hud_timeout = 3.0 if self._mode == "indoor" else 2.0
            sta_timeout = 5.0 if self._mode == "indoor" else 3.0
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
                info_text = f"{label} (dist: {depth:.1f}m) {rel_pos} [{type_tag}]"
            else:
                info_text = f"{label} {rel_pos} [{type_tag}]"
            
            (tw, th), bl = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            label_y = max(y1, th + bl + 6)
            
            # Overlay for transparency
            overlay = frame[label_y - th - bl - 6:label_y, x1:x1 + tw + 8].copy()
            cv2.rectangle(overlay, (0, 0), (tw + 8, th + bl + 6), color, cv2.FILLED)
            frame[label_y - th - bl - 6:label_y, x1:x1 + tw + 8] = cv2.addWeighted(overlay, 0.6, frame[label_y - th - bl - 6:label_y, x1:x1 + tw + 8], 0.4, 0)
            
            cv2.putText(frame, info_text, (x1 + 4, label_y - bl - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
            
            # ── 5. SECONDARY ASSISTIVE LABEL ──
            size_text = f"W:{kw:.1f}m H:{kh:.1f}m | {motion_tag}"
            (stw, sth), sbl = cv2.getTextSize(size_text, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
            sub_y = label_y + sth + sbl + 6
            if sub_y < y2:
                # Text outline for readability without solid background block
                cv2.putText(frame, size_text, (x1 + 3, sub_y - sbl - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,0,0), 2, cv2.LINE_AA)
                cv2.putText(frame, size_text, (x1 + 3, sub_y - sbl - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255) if is_moving else (255, 255, 255), 1, cv2.LINE_AA)
            
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
        if node._show_window:
            waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(waiting_frame, "Waiting for camera feed...", (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            while rclpy.ok():
                frame = node._gui_frame
                if frame is not None:
                    display_frame = frame.copy()
                    node._draw_cached_boxes(display_frame)
                    cv2.imshow(node._window_name, display_frame)
                else:
                    cv2.imshow(node._window_name, waiting_frame)
                
                key = cv2.waitKey(30) & 0xFF
                if key == 27 or key == ord('q'):
                    break
        else:
            # Headless mode: no GUI, just let ROS spin handle everything
            node.get_logger().info("Running in HEADLESS mode (no display). Press Ctrl+C to stop.")
            ros_thread.join()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Save spatial memory on shutdown (indoor mode)
        if node._mode == "indoor":
            node._save_spatial_memory()
        if node._show_window:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
