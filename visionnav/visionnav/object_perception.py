#!/usr/bin/env python3
"""
object_perception.py
====================
ROS 2 Jazzy – Wearable Blind-Assist Vision Node  (Ultralytics YOLOE open-vocabulary edition)

Tesla AI-Grade Dual-Mode Perception System:
  INDOOR  – Persistent spatial memory map, scene recall, object finding
  OUTDOOR – Live hazard warnings, no memory (outdoor_awareness.py): its own detector vocabulary and outdoor
            metric depth, LiDAR walking corridor, ground analysis (obstacles, drops, head-height hazards),
            zebra crossings, traffic / pedestrian light colours, approaching vehicles; spoken on /outdoor_alert

Geometry pipeline (per detection):
  1. Camera and LiDAR extrinsics come from TF (sensor_tf.launch.py), so the objects and the
     SLAM map share one calibration — no hard-coded sign flips.
  2. Every LiDAR point is projected into the image. A point only ranges an object if it lands
     inside the object's segmentation mask *at the row where the scan plane crosses it*. The
     chest LiDAR (1.2 m) passes over chairs, tables and desk items; those points hit the wall
     behind and are rejected instead of being used as the object's depth.
  3. Without a LiDAR hit, depth comes from ray/plane geometry (floor or desk contact point,
     table-top edge) and a known-size prior, fused by inverse variance.
  4. Positions go to the map frame using TF at the image timestamp, then into a Kalman
     tracker whose measurement noise matches the depth source, so close LiDAR-ranged
     sightings outweigh distant optical guesses.
"""

import os
os.environ["QT_QPA_PLATFORM"]  = "xcb"          # force X11/XWayland
os.environ["QT_LOGGING_RULES"] = "*.debug=false;qt.qpa.fonts=false"

import cv2
# pip's OpenCV points Qt at a font folder inside its package that ships no fonts (a warning per window).
# It sets this on import, so it is overridden here, before the first window creates the Qt app.
if os.path.isdir("/usr/share/fonts/truetype/dejavu"):
    os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"
import json
import math
import time
import numpy as np
import threading
import zlib
import glob
import hashlib
import shutil
from collections import deque
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from visionnav.grasp_tracker import GraspTracker
from visionnav import outdoor_awareness as oa
from visionnav.lidar_odometry import GyroYaw, ScanOdometry
from nav_msgs.msg import Odometry
from visionnav.model_paths import model_path
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, Imu, LaserScan, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point
import tf2_ros

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:  # greedy fallback below
    linear_sum_assignment = None

MODEL_PATH  = model_path("yoloe-11s-seg.pt")

# ── OPEN-VOCABULARY DETECTION ──
# YOLOE is prompted once with these names (text embeddings are baked into the TensorRT engine, so
# there is no text encoder at run time). Add a word here and the engine is rebuilt automatically.
VOCABULARY = [
    # doors, stairs and building structure
    "door", "wooden door", "doorway", "doorway to another room", "entrance to room", "room entrance",
    "sliding door", "glass door", "door handle", "door knob", "gate", "window",
    "curtain", "window blinds", "staircase", "stairs", "step", "handrail", "railing", "elevator door",
    "escalator", "pillar", "doormat",
    # switches, sockets and other wall-mounted things
    "light switch", "wall switch", "black switch", "white switch", "electric switch", "switch",
    "switch board", "electrical switch panel", "wall socket",
    "power outlet", "plug socket", "power strip", "extension cord", "circuit breaker panel",
    "thermostat", "fire extinguisher", "fire alarm", "smoke detector", "intercom", "doorbell",
    "air conditioner", "radiator", "water heater", "ceiling fan", "ceiling light", "tube light", "lamp",
    "exit sign", "sign", "mirror", "picture frame", "painting", "poster", "whiteboard", "notice board",
    "calendar", "clock", "wall shelf", "hook", "coat hanger", "towel rack",
    # floor hazards
    "hole in floor", "pothole", "ramp", "wet floor sign", "puddle", "cable on floor",
    "wire", "box", "cardboard box", "bag", "shoe", "slippers", "trash can", "dustbin", "bucket", "mop",
    "broom", "rug", "toy", "ball", "laundry basket", "clothes on floor", "obstacle",
    # furniture
    "chair", "office chair", "plastic chair", "table", "desk", "dining table", "coffee table",
    "side table", "couch", "sofa", "bed", "stool", "bench", "cabinet", "cupboard", "wardrobe", "drawer",
    "chest of drawers", "shelf", "bookshelf", "shoe rack", "tv stand", "kitchen counter",
    "dressing table", "crib",
    # kitchen
    "refrigerator", "microwave", "oven", "stove", "gas cylinder", "rice cooker", "blender", "toaster",
    "kettle", "sink", "faucet", "dish rack", "water dispenser", "plate", "bowl", "cup", "mug", "glass",
    "bottle", "water bottle", "jug", "cooking pot", "frying pan", "knife", "spoon", "fork",
    "cutting board", "food", "fruit",
    # bathroom
    "toilet", "bathtub", "shower", "wash basin", "towel", "toothbrush", "soap",
    # electronics and appliances
    "tv", "monitor", "laptop", "computer", "keyboard", "mouse", "printer", "speaker", "router",
    "phone charger", "cell phone", "remote", "tablet", "camera", "headphones", "fan", "table fan",
    "pedestal fan", "washing machine", "iron", "vacuum cleaner",
    # personal items
    "book", "notebook", "pen", "paper", "backpack", "handbag", "wallet", "keys", "glasses", "watch",
    "medicine", "umbrella", "pillow", "blanket", "cushion", "clothes", "hanger", "basket", "vase",
    "potted plant", "flower", "candle", "scissors", "teddy bear",
    # people / pets
    "person", "child", "dog", "cat",
    # outdoor
    "car", "bicycle", "motorcycle", "three-wheeler", "bus", "truck", "traffic light", "stop sign",
    "fire hydrant", "curb",
]
# Negative prompts: things that are not objects but look like one to an open-vocabulary detector. They are
# in the model's vocabulary so they win the box (YOLO keeps one class per box), and are never reported.
# Measured on the rig: the floor past a doorway threshold was a "floor step" in 22-100 % of frames in three
# views; with these (and without the "floor step" prompt) it was a step in none.
NEGATIVE_PROMPTS = ["door threshold", "floor", "tiled floor", "kitchen floor", "floor edge", "kitchen cabinet"]
VOCABULARY = VOCABULARY + NEGATIVE_PROMPTS
# Several prompts for one thing raise recall; they are reported, mapped and navigated to under one
# name. Measured on the rig: a switch scores 0.60 as "black switch", 0.44 "electric switch", 0.38
# "switch", but only 0.01 as "light switch", and without them it was mostly called a "doorbell".
PROMPT_SYNONYMS = {
    "wall switch": "light switch", "black switch": "light switch", "white switch": "light switch",
    "electric switch": "light switch", "switch": "light switch", "switch board": "light switch",
    "electrical switch panel": "light switch",
    "power outlet": "wall socket", "plug socket": "wall socket",
    "staircase": "stairs", "doorway": "door", "sliding door": "door",
    # Measured on the rig: a closed wooden door scored 0.67 as "wardrobe" and only 0.65 as "door" (and was
    # called a wardrobe in 71 % of frames); as "wooden door" it scores 0.89 in every frame. An open doorway
    # was mostly a "mirror" (0.40-0.47); "entrance to room" (0.62) and "room entrance" win over it.
    "wooden door": "door", "doorway to another room": "door", "entrance to room": "door", "room entrance": "door",
    "coffee table": "table", "side table": "table",
    "glass door": "door", "door knob": "door handle", "dustbin": "trash can",
    "cardboard box": "box", "office chair": "chair", "plastic chair": "chair", "desk": "table",
    "dining table": "table", "cupboard": "cabinet", "wall shelf": "shelf",
    "water bottle": "bottle", "mug": "cup", "table fan": "fan", "pedestal fan": "fan",
    "wire": "cable on floor", "wash basin": "sink", "elevator door": "door",
}
_VOCAB_HASH = hashlib.sha1("|".join(VOCABULARY).encode()).hexdigest()[:8]
ENGINE_PATH = model_path(f"yoloe-11s-seg-indoor-{_VOCAB_HASH}.engine")
# Outdoor mode has its own vocabulary (outdoor_awareness.py) and engine: one engine for both would make outdoor
# prompts compete with indoor ones for every box (a filtered-out class still wins the box it takes).
_OUTDOOR_HASH = hashlib.sha1("|".join(oa.OUTDOOR_VOCABULARY).encode()).hexdigest()[:8]
OUTDOOR_ENGINE_PATH = model_path(f"yoloe-11s-seg-outdoor-{_OUTDOOR_HASH}.engine")

INDOOR_CLASSES = set(VOCABULARY) - set(NEGATIVE_PROMPTS) - {
    "bicycle", "motorcycle", "bus", "truck", "car", "traffic light", "stop sign", "fire hydrant",
    "curb", "pothole", "three-wheeler",
}
OUTDOOR_CLASSES = set(oa.OUTDOOR_VOCABULARY) - set(oa.OUTDOOR_NEGATIVE_PROMPTS)
# (vocabulary, engine, reported classes, prompt -> reported name) of each mode's detector
DETECTORS = {
    "indoor": (VOCABULARY, ENGINE_PATH, INDOOR_CLASSES, None),
    "outdoor": (oa.OUTDOOR_VOCABULARY, OUTDOOR_ENGINE_PATH, OUTDOOR_CLASSES, oa.OUTDOOR_SYNONYMS),
}
# Drops: a blind user needs more warning before these than before a chair.
DROP_HAZARDS = {"stairs", "step", "hole in floor", "pothole", "curb", "escalator"}
# Hazards in the floor itself: a measured top (metric depth or LiDAR) higher than this is not one. The edge
# of a kitchen counter seen through a doorway was read as a "floor step" on the rig.
FLOOR_LEVEL_HAZARDS = {"step", "hole in floor", "pothole", "curb"}
FLOOR_HAZARD_MAX_TOP = 0.5   # m above the floor

# Class-specific NMS is done by YOLO. Across classes, only suppress pairs the model
# genuinely confuses — a person sitting on a chair must keep both detections.
# Groups hold both raw and reported names (FRIENDLY_NAMES), since either may be compared.
CONFUSABLE_GROUPS = [
    {"tv", "monitor", "laptop"},
    # a door and the furniture doors that look just like it
    {"door", "wardrobe", "cabinet", "refrigerator"},
    # an open doorway looks like a mirror (a framed view of another room); kept apart from the group above
    # so a mirror and a cabinet are not interchangeable (the kitchen counter through a doorway was renamed
    # a mirror)
    {"door", "mirror"},
    # one table seen as a desk, a coffee table and a table
    {"table", "coffee table", "side table", "desk", "dining table", "tv stand", "dressing table",
     "kitchen counter", "chest of drawers"},
    # small plates on a wall
    {"light switch", "wall socket", "doorbell", "thermostat", "intercom", "fire alarm", "smoke detector"},
    {"couch", "sofa", "bed", "chair"},
    {"cup", "bottle", "vase"},
    {"cell phone", "smartphone", "remote"},
    {"car", "truck", "bus", "three-wheeler"},
    {"bicycle", "motorcycle"},
]
# In a close call between look-alikes, the class that matters for walking wins (a door, not a wardrobe).
PRIORITY_LABELS = {"door", "stairs", "step", "hole in floor", "light switch", "wall socket", "person"}
PRIORITY_BOOST = 1.25
# A track is renamed once another look-alike label has clearly more evidence than its own.
RELABEL_MIN_VOTES = 3.0
RELABEL_RATIO = 1.5
# Small things fixed to a wall
WALL_MOUNTED = {"light switch", "wall socket", "doorbell", "thermostat", "intercom", "fire alarm",
                "smoke detector", "door handle", "door knob", "exit sign", "power strip"}
WALL_MOUNTED_FOOTPRINT = 0.35  # m
# Flat things in or on a wall: drawn as a thin panel along the wall, not a width x width block (a 1.1 m door
# was a 1.1 x 1.1 x 2.1 m cube). The wall direction is fitted to the LiDAR points around the object.
PANEL_OBJECTS = WALL_MOUNTED | {"door", "window", "curtain", "window blinds", "mirror", "picture frame",
                                "painting", "poster", "whiteboard", "notice board", "calendar", "clock",
                                "sign", "air conditioner", "circuit breaker panel"}
PANEL_THICKNESS = 0.05       # m
WALL_FIT_MIN_PTS = 5         # LiDAR points needed to fit the wall line
WALL_FIT_MIN_ELONGATION = 6.0  # variance along / across the fitted line
DOOR_MIN_HEIGHT = 1.9        # m: doors are standard height; a doorway's top is often cut off by the frame
DOOR_MIN_WIDTH = 0.3         # m: a "door" measured narrower than this (and not cut off by the frame edge) is a
                             # sliver of frame or wall corner: one 10 cm wide at 1 m became a phantom door
YOLO_IOU = 0.50
# Small objects are the ones YOLO misnames most (a door handle as a cup, a remote as a phone),
# so they need more confidence than furniture before they are shown or mapped.
SMALL_OBJECT_CONF = 0.55
SMALL_OBJECTS = {"door handle", "door knob", "keys", "pen", "wallet", "glasses", "watch", "phone charger", "cup", "bottle", "cell phone", "mouse", "remote", "book", "vase", "clock",
                 "scissors", "toothbrush", "spoon", "fork", "knife", "wine glass", "sports ball"}
CROSS_CLASS_OVERLAP = 0.70   # intersection / smaller box area
CROSS_CLASS_SAME_BOX_IOU = 0.80  # any two static labels on (almost) the same box are one detection
# A look-alike label on a box where a better-ranked look-alike was detected this recently is the same
# object flickering between names (an open doorway alternated "door" / "mirror" on the rig and spawned a
# second object 7.8 m away, where the LiDAR saw through the opening).
LOOKALIKE_MEMORY_S = 1.0
LOOKALIKE_BOX_AGE = 2.0      # s: a confirmed object's last image box claims look-alike detections this long

FRIENDLY_NAMES = {
    **PROMPT_SYNONYMS,
    "dining table": "table", "couch": "sofa", "cell phone": "smartphone",
    "potted plant": "plant", "wine glass": "glass", "sports ball": "ball",
    "baseball bat": "bat", "baseball glove": "glove", "tennis racket": "racket",
    "hair drier": "hair dryer", "tv": "monitor", "fire hydrant": "hydrant",
    "parking meter": "meter", "traffic light": "signal",
}

# ── KNOWN REAL-WORLD MAXIMUM PHYSICAL SIZES (width_m, height_m) ──
# Used to clamp estimated sizes to prevent wildly oversized RViz markers.
OBJECT_MAX_SIZES = {
    "doorbell": (0.15, 0.20), "thermostat": (0.20, 0.20), "intercom": (0.25, 0.35),
    "fire alarm": (0.25, 0.25), "smoke detector": (0.20, 0.10), "door handle": (0.30, 0.12),
    "exit sign": (0.60, 0.30), "sign": (1.50, 1.00), "picture frame": (1.50, 1.50),
    "painting": (1.50, 1.50), "poster": (1.00, 1.50), "calendar": (0.50, 0.70),
    "window": (3.00, 2.20), "curtain": (4.00, 3.00), "wardrobe": (2.50, 2.40), "cabinet": (2.00, 2.20),
    "door": (1.10, 2.10), "light switch": (0.12, 0.14), "wall socket": (0.12, 0.12),
    "stairs": (1.60, 3.00), "step": (1.60, 0.30), "hole in floor": (1.50, 0.20),
    "pothole": (1.00, 0.20), "obstacle": (1.50, 1.50), "curb": (3.00, 0.25),
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
    "toilet":       (0.50, 0.80),
    "sink":         (0.60, 0.30),
    "tv":           (1.20, 0.80),
    "monitor":      (1.20, 0.80),
    "television":   (1.20, 0.80),
    "chair":        (0.65, 1.10),
    # Large objects
    "table":        (1.80, 0.85),
    "dining table": (1.80, 0.85),
    "sofa":         (2.20, 1.00),
    "couch":        (2.20, 1.00),
    "bed":          (2.20, 1.00),
    "refrigerator": (0.90, 1.90),
    "oven":         (0.70, 0.95),
    # Dynamic / outdoor objects
    "person":       (0.70, 2.00),
    "bicycle":      (1.80, 1.20),
    "car":          (4.80, 1.80),
    "motorcycle":   (2.20, 1.40),
    "bus":          (12.0, 3.60),
    "truck":        (9.00, 3.80),
    "dog":          (1.00, 0.90),
    "cat":          (0.50, 0.40),
    "bench":        (1.80, 1.00),
    "signal":       (0.40, 1.20),
    "stop sign":    (0.80, 0.80),
    "hydrant":      (0.40, 0.90),
    "suitcase":     (0.55, 0.80),
}
for _k, _v in oa.OUTDOOR_MAX_SIZES.items():
    OBJECT_MAX_SIZES.setdefault(_k, _v)
OBJECT_MAX_SIZE_DEFAULT = (1.50, 1.50)

# ── TYPICAL SIZES (width_m, height_m) for the known-size depth prior ──
# Falls back to 60 % of the max size for classes not listed.
OBJECT_TYPICAL_SIZES = {
    "doorbell": (0.08, 0.12), "thermostat": (0.10, 0.10), "intercom": (0.12, 0.20),
    "fire alarm": (0.12, 0.12), "smoke detector": (0.12, 0.05), "door handle": (0.15, 0.05),
    "door knob": (0.07, 0.07), "exit sign": (0.35, 0.15), "sign": (0.40, 0.30),
    "picture frame": (0.40, 0.50), "painting": (0.60, 0.50), "poster": (0.45, 0.65),
    "calendar": (0.30, 0.45), "window": (1.00, 1.20), "curtain": (1.20, 2.00),
    "wardrobe": (1.20, 1.90), "cabinet": (0.80, 0.90),
    "door": (0.90, 2.00), "light switch": (0.08, 0.12), "wall socket": (0.08, 0.08),
    "stairs": (1.20, 2.50), "step": (1.20, 0.20), "hole in floor": (1.00, 0.10),
    "obstacle": (1.00, 1.00),
    "person": (0.45, 1.68), "chair": (0.50, 0.88), "couch": (1.80, 0.85), "sofa": (1.80, 0.85),
    "bed": (1.60, 0.55), "dining table": (1.20, 0.75), "table": (1.20, 0.75),
    "laptop": (0.33, 0.23), "tv": (0.90, 0.55), "monitor": (0.55, 0.35), "keyboard": (0.44, 0.04), "mouse": (0.06, 0.04),
    "bottle": (0.07, 0.24), "cup": (0.08, 0.10), "cell phone": (0.075, 0.15), "smartphone": (0.075, 0.15),
    "book": (0.16, 0.23), "refrigerator": (0.70, 1.75), "microwave": (0.48, 0.28), "oven": (0.60, 0.88),
    "sink": (0.50, 0.20), "vase": (0.14, 0.28), "clock": (0.30, 0.30), "toilet": (0.38, 0.75),
    "backpack": (0.30, 0.45), "teddy bear": (0.28, 0.35), "potted plant": (0.35, 0.60),
    "bicycle": (1.70, 1.05), "car": (4.40, 1.50), "motorcycle": (2.00, 1.15), "bus": (11.0, 3.20),
    "truck": (7.00, 3.00), "dog": (0.60, 0.55), "cat": (0.40, 0.28), "bench": (1.50, 0.80),
    "traffic light": (0.30, 0.90), "stop sign": (0.75, 0.75), "fire hydrant": (0.30, 0.70),
}
for _k, _v in oa.OUTDOOR_TYPICAL_SIZES.items():
    OBJECT_TYPICAL_SIZES.setdefault(_k, _v)
# Objects usually seen from above: their pixel height is mostly foreshortening, so only
# their width says anything about distance.
FLAT_OBJECTS = {"keyboard", "mouse", "cell phone", "smartphone", "book", "laptop", "remote",
                "dining table", "table", "bed"}

# ── SEMANTIC 2D ASPECT RATIO LIMITS (Height / Width) ──
# Prevents YOLO from drawing massive vertical boxes around flat objects (like confusing a wall for a laptop)
# Boxes far outside a class's possible shape are misdetections, e.g. a tall dark door seen as a
# "tv". They are dropped instead of squashed. (Height / Width)
OBJECT_REJECT_ASPECT_RATIOS = {"tv": 1.9, "laptop": 1.8, "microwave": 1.5, "keyboard": 1.2, "bed": 1.5,
                               "couch": 1.6, "dining table": 1.6}

OBJECT_MAX_ASPECT_RATIOS = {
    "laptop": 0.85, "mouse": 0.60, "keyboard": 0.40,
    "tv": 0.85, "television": 0.85, "bed": 0.70,
    "couch": 0.85, "sofa": 0.85, "car": 0.80,
    "truck": 0.80, "bowl": 0.60, "sink": 0.70,
    "book": 1.50, "cell phone": 2.0
}

# ── SUPPORT SURFACES ──
# Objects standing on the floor: their lowest pixel is a floor contact point.
FLOOR_OBJECTS = {"door", "stairs", "step", "table", "obstacle", "hole in floor", "pothole", "trash can",
                 "box", "chair", "couch", "bed", "dining table", "toilet", "refrigerator", "oven",
                 "potted plant", "bench", "suitcase", "person", "bicycle", "car", "motorcycle",
                 "bus", "truck", "dog", "cat", "fire hydrant", "backpack",
                 # tall furniture stands on the floor (a wardrobe was drawn "on a 0.37 m surface")
                 "wardrobe", "cabinet", "shelf", "bookshelf", "chest of drawers", "sofa", "stool",
                 "tv stand", "dressing table", "kitchen counter", "washing machine", "water dispenser",
                 "crib", "shoe rack"}
# Objects that typically sit on tables
DESKTOP_OBJECTS = {
    "laptop", "mouse", "keyboard", "cup", "bottle", "bowl", "tv",
    "monitor", "book", "vase", "scissors", "remote", "cell phone",
    "smartphone", "apple", "orange", "banana", "fork", "knife",
    "spoon", "toaster", "microwave", "sink",
}
DESK_HEIGHT = 0.75
SUPPORT_HEIGHTS = {"microwave": 0.90, "toaster": 0.90, "sink": 0.85}
# Classes whose top surface is at a standard height: the box's top edge gives distance.
OBJECT_TOP_HEIGHTS = {"dining table": 0.75, "table": 0.75}

# ── SENSOR / GEOMETRY PARAMETERS ──
BASE_FRAME = "base_footprint"
CAMERA_FRAME = os.environ.get("WEARABLE_CAMERA_FRAME", "camera_link")
# Chest sway while walking makes the true camera pitch wobble around its mounted value.
PITCH_SIGMA = math.radians(float(os.environ.get("WEARABLE_PITCH_SWAY_DEG", "3.0")))
# How far (px) above/below a box the scan-plane crossing may fall (absorbs pitch sway).
LIDAR_ROW_TOL_PX = int(os.environ.get("WEARABLE_LIDAR_ROW_TOL_PX", "25"))
LIDAR_CLUSTER_GAP = 0.25     # m, range gap separating an object from what is behind it
LIDAR_BODY_RANGE = 0.30      # m, returns closer than this are the wearer's body
LIDAR_SNAP_RANGE = (0.45, 1.15)  # LiDAR range / camera depth accepted when snapping (near objects: depth net reads long)
LIDAR_SNAP_PLANE_MARGIN = 0.15   # m: only objects whose top reaches this close to the scan plane may snap to it
# Snap regardless of height (the old behaviour), for a rig whose mounting heights in TF are wrong
LIDAR_SNAP_ANY_HEIGHT = os.environ.get("WEARABLE_LIDAR_SNAP_ANY_HEIGHT", "0") == "1"
SCAN_MATCH_MAX_DT = 0.25     # s, max image/scan time offset before falling back to latest scan
BOX_EDGE_PX = 4              # a box this close to the border is truncated by the frame
CENTER_BEARING_DEG = 7.0     # objects within this angle of straight ahead are announced as CENTER

# Optical camera frame (z forward, x right, y down) expressed in a level, forward-facing
# body frame (x forward, y left, z up).
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])

# ── MONOCULAR METRIC DEPTH (Depth Anything V2, indoor) ──
# Gives every pixel a distance, so objects the chest LiDAR passes over (chairs, tables, desk items)
# are ranged too. Its scale is corrected every frame against the LiDAR's true ranges.
MONO_DEPTH_ENABLED = os.environ.get("WEARABLE_MONO_DEPTH", "1") == "1"
DEPTH_WEIGHTS = model_path("depth_anything_v2_metric_indoor_vits.pth")
# Outdoor: the model trained on street scenes (Virtual KITTI, up to 80 m). The indoor one tops out at 20 m.
# Fetch: curl -L -o models/depth_anything_v2_metric_outdoor_vits.pth https://huggingface.co/depth-anything/
#        Depth-Anything-V2-Metric-VKITTI-Small/resolve/main/depth_anything_v2_metric_vkitti_vits.pth
DEPTH_WEIGHTS_OUTDOOR = model_path("depth_anything_v2_metric_outdoor_vits.pth")
DEPTH_MAX = {"indoor": 20.0, "outdoor": 80.0}
DEPTH_INPUT_SIZE = int(os.environ.get("WEARABLE_DEPTH_SIZE", "392"))  # short side, multiple of 14
DEPTH_SCALE_MIN_PTS = 15     # LiDAR points needed to (re)calibrate the depth scale
DEPTH_SCALE_ALPHA = 0.2      # smoothing of the per-frame scale estimate
DEPTH_SCALE_MAX_RESID = 0.15 # a frame whose depth/LiDAR ratios spread more than this is not used to calibrate
DEPTH_SCALE_FRESH_S = 3.0    # s: the scale only earns the tight range sigma while calibrated this recently
# Range of the LiDAR scale correction. The outdoor (street-trained) model read a room twice too far on the rig
# (true scale ~0.5, pinned at the old 0.5 floor), so outdoors it may correct more.
DEPTH_SCALE_LIMITS = {"indoor": (0.5, 2.0), "outdoor": (0.3, 3.0)}
MONO_MAX_POINTS = 2000       # object pixels back-projected per detection

# ── TRACKING PARAMETERS ──
MIN_HITS_STATIC = 4          # sightings before a static object is mapped / remembered
MIN_HITS_DYNAMIC = 2
# ── MOVING OBJECTS (people, pets) ──
# Constant-velocity pedestrian model, timed by when each camera frame arrived (not when it was processed:
# processing delay varies and turned into fake speed, 0.5 m/s median for people sitting still on the rig).
DYN_ACCEL_NOISE = 1.5        # m^2/s^3, white-noise acceleration of a walking person
DYN_INIT_VEL_VAR = 1.0       # (m/s)^2 before the first two sightings
DYN_MIN_SIGMA = 0.10         # m: a person's measured centre wobbles this much (arms, torso turning)
DYN_MAX_SPEED = 3.0          # m/s, indoors
DYN_SPEED_MIN_AGE = 0.5      # s of tracking before a speed is reported
DYN_MOVING_ON, DYN_MOVING_OFF = 0.40, 0.25  # m/s: "MOVING" hysteresis
DYN_SHOW_S = 0.4             # s: a moving object is drawn only this long after its last sighting
DYN_BOX_IOU = 0.3            # image-space match: a person's box overlaps their box this recently...
DYN_BOX_MAX_AGE = 0.3        # ...within this many seconds
DYN_EXTRAPOLATE_S = 0.3      # s a moving object's drawn position may run ahead of its last sighting
MONO_RATIO_ALPHA = 0.3       # per-person LiDAR/metric-depth ratio, used when the LiDAR misses them
DYN_REANCHOR_RUN = 3         # consecutive agreeing out-of-gate sightings that move a person's track there
# A static object must be detected in at least this share of the frames in which the camera is looking at
# its spot. A real object is found in nearly every frame (chairs 95-100 % on the rig); a hallucination
# flickers (a "step" on a doorway threshold: 77 % of frames at a mean score of 0.27, so it crossed
# the 0.30 threshold only now and then). Without it a flicker never timed out and even became "reliable".
MIN_DETECTION_RATE = 0.5
RELIABLE_DETECTION_RATE = 0.6  # stricter before an object is remembered for good, so a lucky run of
                               # detections early in a flicker cannot latch it into the map
# ...and its mean detection score must reach this. Real objects on the rig averaged 0.52 (an open doorway)
# to 0.89; the "floor step" hallucination on a doorway threshold came in bursts just over the 0.30
# threshold and latched on detection rate alone.
RELIABLE_MIN_MEAN_CONF = 0.40
# ...and it must have been measured within this range: beyond it the position is a guess (a "mirror" 7.8 m
# away, where the LiDAR looked through an open doorway, latched into memory). Farther objects are
# remembered once the wearer comes closer.
MEMORY_MAX_RANGE = 6.0
# ...and, while metric depth is running, at least this many of its sightings must have been ranged by the
# LiDAR or depth. A position from box size alone moves with every view: roaming the room without depth mapped
# one switch at three places. (Without depth, chairs and tables below the LiDAR plane have nothing else.)
MEMORY_MIN_RANGED_HITS = 5
DETECTION_VIEW_MARGIN = 0.03  # fraction of the frame border ignored when counting a missed detection
DETECTION_VIEW_RANGE = 10.0   # m, objects this close count as in view for the detection rate
UNCONFIRMED_TIMEOUT = 1.5    # s, tentative tracks die fast (kills one-frame hallucinations)
TRACK_DEBUG = os.environ.get("WEARABLE_TRACK_DEBUG", "0") == "1"  # log why each new static object is created
CONFIRMED_TIMEOUT = 30.0    # s a confirmed (MIN_HITS_STATIC) but not yet reliable object survives out of view,
                             # so the next glance at it matches it instead of mapping it again under a new ID
UNSEEN_DROP_S = 1.0          # s, an object the camera is looking at but no longer detects is removed
                             # (not yet reliable objects only; a remembered one needs free-space evidence)
VISIBILITY_MAX_RANGE = 6.0   # m, only apply the rule above to objects this close
# Real-time map: an object is drawn only while it is being detected right now
LIVE_WINDOW = 1.0            # s
LIVE_MIN_HITS = 2            # sightings within LIVE_WINDOW (one stray frame is not enough)
MARKER_LIFETIME = 0.5        # s, RViz removes an object this soon after it stops being published
# PERSISTENT GLOBAL MAP (indoor): once an object has been seen reliably it stays on the map, faded,
# for the rest of the session, and keeps its ID/name when seen again from another angle. Set
# WEARABLE_MEMORY_S (e.g. 8) for the old real-time-only behaviour instead. Walking around the room, the
# detector often misses a remembered object from a new angle (the back of a chair) and SLAM/depth put it
# a few decimetres off, so "not detected" alone never removes it. It is removed only when the depth map
# reads clearly past its whole spot, wide enough to absorb that offset, for FREE_SPACE_CLEAR_RELIABLE_S
# (the object was taken away). A live object of the same class on its spot is merged into it (same ID).
MEMORY_MIN_HITS = 15
MEMORY_MIN_SPAN = 1.0        # s between first and latest sighting
MEMORY_TTL = float(os.environ.get("WEARABLE_MEMORY_S", "inf"))  # s a reliable object outlives its last sighting
BLIND_SPOT_RADIUS = 0.8      # m: a remembered object this close to the wearer is expected to be below
                              # the chest-mounted camera/LiDAR's view, so it is never dropped for going
                              # unseen (memory-anchored terminal navigation needs it to still be there)
OCCLUSION_DEPTH_MARGIN = 0.3  # m: something this much closer in the same pixel hides the object
FREE_SPACE_MARGIN = 1.0      # m: live depth this far beyond a remembered object means its spot is empty
FREE_SPACE_CLEAR_S = 0.4     # s of consistent free-space evidence before it is pruned (rejects depth glitches)
FREE_SPACE_CLEAR_RELIABLE_S = 1.5  # s of it before a remembered object is removed (seen from a new direction)
# TAKEN AWAY: a remembered object is removed quickly once the camera looks at its spot from a direction it was
# detected from before (so "not detected" means something: it is not the back of a chair seen for the first
# time), close enough, with nothing in front, and it is not there. Only the slow rule above applied before, and
# its wide patch and 1 m margin never cleared a cup taken off a table or a chair moved away from a wall.
REMOVE_MAX_RANGE = 4.0       # m
REMOVE_VIEW_BIN_DEG = 30.0   # viewing directions are remembered in bins this wide (+-1 bin counts as the same)
REMOVE_UNSEEN_S = 2.0        # s undetected in plain view from a known direction...
REMOVE_GAP_FACTOR = 3.0      # ...and at least this many times the longest gap it has ever shown between detections
                             # (an open doorway's detection flickers for 1-2 s; a chair's almost never)
REMOVE_VISIBLE_FRACTION = 0.6  # share of an object's projected 3-D box inside the frame for "in plain view" (its
                               # centre in the middle 76 % excluded table-height things low in a chest camera's view)
REMOVE_PERSON_OVERLAP = 0.3  # share of the object's spot covered by a person's box that counts as hiding it
REMOVE_FREE_S = 0.7          # s, if the depth also shows the background behind its spot
REMOVE_FREE_MARGIN = 0.3     # m (+15 % of range) behind the object's range counts as background
FREE_SPACE_SLACK = 0.3       # m: a remembered object's checked patch is widened by this on each side
                             # (SLAM and depth error while walking), and its lowest depths are used
SEE_THROUGH = {"door", "window"}  # open doorways / windows: depth reading past them is expected, not a ghost
MAX_REMEMBERED_OBJECTS = 200  # oldest-seen remembered objects are evicted first past this count
REMEMBERED_ALPHA = 0.30      # remembered (not currently seen) objects are drawn translucent
OBJECTS_PUBLISH_PERIOD = 0.2 # s, /semantic_objects rate (5 Hz)
# A static object's position is the running average of about the last 1.5 s of sightings at 20 Hz. With an
# 8 cm floor and 0.01 m^2/s drift, each sighting moved it 25-60 %: with the rig standing still, objects
# wandered over 22 cm (median) and up to 35 cm on the map. Real shifts (SLAM, a new viewpoint) are handled by
# the revisit prior, image-space association and re-anchoring, not by trusting every frame.
STATIC_POS_Q = 0.0005        # m^2/s, static objects may slowly be re-estimated / moved
STATIC_MIN_VAR = 0.01 ** 2   # m^2: never more certain than this
STATIC_DIM_ALPHA = 0.08      # smoothing of a static object's width, height and elevation per sighting
REVISIT_GAP_S = 2.0          # s unseen after which the next sighting is treated as a revisit
REVISIT_VAR = 0.30 ** 2      # m^2: prior widened to this on a revisit (absorbs SLAM drift / a new viewing angle)
DRIFT_MERGE_S = 1.5          # s a remembered object may go unseen in plain view, while a newer live object of its
                             # class stands within the revisit gate, before the two are merged under the old ID
                             # (a SLAM loop closure or depth jump moved it; it did not become two objects)
CHI2_GATE_2D = 9.21          # 99 % gate for a 2-D innovation
SAME_OBJECT_IOU = 0.3        # footprints overlapping this much (and statistically consistent) are one object
# Two *different* labels on one spot (a cabinet also read as a door and a notice board) are one object when
# the footprints overlap this much, the widths are within this ratio and the height bands overlap this much
# of the shorter one. A bottle on a table (tiny vs large, different heights) stays two objects.
SAME_PLACE_IOU = 0.40
SAME_PLACE_SIZE_RATIO = 1.6
SAME_PLACE_Z_OVERLAP = 0.5
BIG_COST = 1e6
# Image-space association: a static object's box overlapping, this much, the box its track had this recently
# is that object even when its distance estimate jumped (a table's cut-off box was sized 3 m too far and
# became a second table). The jumped position is then weighted by the jump, so it barely moves the object.
BOX_TRACK_IOU = 0.5
BOX_TRACK_MAX_AGE = 1.0      # s
# After this many image-only matches in a row ranged by LiDAR or depth that agree with each other within
# BOX_SNAP_SPREAD, the object really is at the new position (SLAM corrected the map) and is moved there;
# box-size guesses never move it this way. With 3 unchecked sightings, a door jumped 2 m on the rig whenever
# a chair in front of it put LiDAR ranges of 0.8 m and 3.1 m into its outline by turns.
BOX_SNAP_RUN = 10
BOX_SNAP_SPREAD = 0.2        # m
BOX_ONLY_COST = 2.0 * CHI2_GATE_2D  # assignment cost of an image-only match (above the 3-D gate, so 3-D matches win)
HUD_TIMEOUT = 0.7            # s, camera-view boxes vanish this fast once the object is not detected
RECORD_PERIOD_S = 2.0        # s between frames saved with WEARABLE_RECORD_DIR (besides every spoken alert)
OUTDOOR_SHOW_S = 0.3         # s: outdoors an object is drawn in RViz only this long after it was last detected
OUTDOOR_STRUCTURE = {"wall", "fence", "railing", "gate", "bus stop", "construction site"}  # drawn by the occupancy
                             # grid, not as objects (a wall was a 3 m block on the rig)
OUTDOOR_MARKER_LIFETIME = 0.5  # s: RViz drops the outdoor view this soon if frames stop coming
HUD_SYNC_MAX_AGE = 0.3       # s: the window shows the frame the boxes were computed on while it is this fresh
HUD_BOX_ALPHA, HUD_BOX_SCALE_PX = 0.15, 40.0     # static box: weight of a new corner, +1 per this many px moved
HUD_DEPTH_ALPHA, HUD_DEPTH_SCALE_M = 0.1, 1.0    # static distance label: same, per metre changed
LABEL_SCALE = 0.15           # RViz text height (m)
LABEL_CLEAR_XY = 1.0         # m, RViz labels closer than this horizontally are stacked...
LABEL_CLEAR_Z = 0.40         # ...this far apart vertically (a leader line still ties each to its object)
# Distinct colours so each label, its leader line and its object visibly belong together (RGB 0-1)
OBJECT_PALETTE = [
    (0.10, 0.85, 1.00), (1.00, 0.55, 0.10), (0.35, 1.00, 0.35), (1.00, 0.30, 0.75),
    (1.00, 0.95, 0.20), (0.60, 0.45, 1.00), (0.20, 1.00, 0.80), (1.00, 0.35, 0.30),
    (0.75, 1.00, 0.20), (0.95, 0.70, 1.00),
]


# ── OBJECT COLOUR (spoken references: "the red cup", "the brown door") ──
# Named from the object's own mask pixels, with brightness relative to the frame's white level (the chest
# camera runs dark: a white wall reads ~0.8, a red cup 0.28, a brown door 0.09-0.16 of it). Measured on the
# rig: cup 94 % saturated pixels, hue red; door 55-82 %, hue red/orange but dark; chairs <30 %, very dark.
COLOR_MIN_CHROMA = 0.45      # share of saturated pixels for a chromatic colour
COLOR_SAT = 60               # HSV saturation (0-255) of a "saturated" pixel
COLOR_MIN_VREL = 0.08        # ...and its minimum brightness relative to the frame's white level
COLOR_BROWN_VREL = 0.20      # red/orange/yellow darker than this (or weakly saturated) is brown
COLOR_BROWN_SAT = 100
COLOR_HUES = (("red", 0, 8), ("orange", 8, 20), ("yellow", 20, 33), ("green", 33, 85), ("blue", 85, 130),
              ("purple", 130, 150), ("pink", 150, 170), ("red", 170, 181))  # OpenCV hue 0-180
COLOR_MIN_VOTES = 3.0        # confidence-weighted sightings before an object's colour is reported
COLOR_MIN_SHARE = 0.5        # ...and the share of them its colour needs


def _color_name(hsv, vref: float, mask):
    """Everyday colour name of the masked pixels of an HSV frame, or None if too few pixels."""
    if mask.shape[0] > 12 and mask.shape[1] > 12:
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8))  # edge pixels mix object and background
    px = hsv[mask > 0]
    if len(px) < 30:
        return None
    hue, sat = px[:, 0].astype(int), px[:, 1].astype(int)
    val = px[:, 2].astype(float) / vref
    chroma = (sat >= COLOR_SAT) & (val >= COLOR_MIN_VREL)
    if chroma.mean() >= COLOR_MIN_CHROMA:
        h = hue[chroma]
        counts = {}
        for name, lo, hi in COLOR_HUES:
            counts[name] = counts.get(name, 0) + int(np.count_nonzero((h >= lo) & (h < hi)))
        name = max(counts, key=counts.get)
        if name in ("red", "orange", "yellow") and (float(np.median(val[chroma])) < COLOR_BROWN_VREL
                                                    or float(np.median(sat[chroma])) < COLOR_BROWN_SAT):
            return "brown"
        return name
    v = float(np.median(val))
    return "black" if v < 0.25 else "white" if v > 0.75 else "grey"


def _object_color(name: str):
    """Stable per-object colour (same in RViz and the camera window)."""
    return OBJECT_PALETTE[zlib.crc32(name.encode()) % len(OBJECT_PALETTE)]

# ── MODE-SPECIFIC PERCEPTION PARAMETERS ──
# Indoor: high memory, low noise, persistent mapping
# Outdoor: short memory, responsive tracking, collision focus
MODE_PARAMS = {
    "indoor": {
        "conf_threshold":       0.30,    # open-vocabulary scores run low; track confirmation (MIN_HITS) filters hallucinations
        "static_timeout":       120.0,   # 2 min memory while indoors
        "dynamic_timeout":      1.0,     # kept this long unseen (re-found under the same ID), drawn only DYN_SHOW_S
        "static_assoc":         1.50,    # hard association limit (m); main gate is statistical
        "dynamic_assoc":        2.00,
        "danger_distance":      1.5,     # Indoor danger threshold
        "collision_corridor_w": 0.8,     # Narrow indoor corridor
        "max_range":            15.0,    # m, farthest distance an object is placed at
    },
    "outdoor": {
        "conf_threshold":       0.30,
        "static_timeout":       3.0,     # Very short memory outdoors
        "dynamic_timeout":      1.0,
        "static_assoc":         2.00,
        "dynamic_assoc":        2.50,
        "danger_distance":      2.0,     # Outdoor needs earlier warnings
        "collision_corridor_w": 2 * oa.CORRIDOR_HALF,  # shoulder-width walking corridor
        "max_range":            40.0,    # vehicles matter far away
    },
}



def _quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _lookup(table: dict, label: str, raw_label: str, default=None):
    return table.get(label, table.get(raw_label, default))


def _max_size(label: str, raw_label: str):
    return _lookup(OBJECT_MAX_SIZES, label, raw_label, OBJECT_MAX_SIZE_DEFAULT)


def _typical_size(label: str, raw_label: str):
    typ = _lookup(OBJECT_TYPICAL_SIZES, label, raw_label)
    if typ is None:
        max_w, max_h = _max_size(label, raw_label)
        typ = (0.6 * max_w, 0.6 * max_h)
    return typ


def _ranking_score(det) -> float:
    """Confidence, slightly favouring the classes that matter for walking among look-alikes."""
    return det["conf"] * (PRIORITY_BOOST if det["label"] in PRIORITY_LABELS else 1.0)


def _min_separation(label: str) -> float:
    """Two objects of the same class cannot physically stand closer than this (centre to centre)."""
    typ_w, _ = _typical_size(label, label)
    return max(0.35, min(1.0, 0.7 * typ_w))


def _footprint_iou(ax, ay, aw, bx, by, bw) -> float:
    """IoU of two axis-aligned square footprints (side = object width): a cheap stand-in for 3-D box IoU."""
    ix = max(0.0, min(ax + aw / 2, bx + bw / 2) - max(ax - aw / 2, bx - bw / 2))
    iy = max(0.0, min(ay + aw / 2, by + bw / 2) - max(ay - aw / 2, by - bw / 2))
    inter = ix * iy
    return inter / (aw * aw + bw * bw - inter + 1e-9)


def _dedup_width(label: str, width: float) -> float:
    """Footprint used to decide whether two sightings are one object. A switch's measured position
    wobbles by more than its own 8 cm, and two wall plates are never this close together."""
    return max(width, WALL_MOUNTED_FOOTPRINT) if label in WALL_MOUNTED else width


def _same_object(a_xy, a_w, a_var, b_xy, b_w, b_var, min_sep: float = 0.0) -> bool:
    """Same physical object only if the footprints overlap (or the centres are closer than two objects of
    this class can stand, `min_sep`) AND the gap is within the joint position uncertainty. Plain centre
    distance merged two neighbouring chairs into one."""
    d2 = (a_xy[0] - b_xy[0]) ** 2 + (a_xy[1] - b_xy[1]) ** 2
    # A big object's estimated centre shifts with the side it is seen from (its visible face)
    size_var = (0.25 * max(a_w, b_w)) ** 2
    overlap = (_footprint_iou(a_xy[0], a_xy[1], a_w, b_xy[0], b_xy[1], b_w) >= SAME_OBJECT_IOU
               or d2 <= min_sep ** 2)
    return overlap and d2 / max(a_var + b_var + size_var, 1e-4) <= CHI2_GATE_2D


def _same_place(a_xy, a_w, a_z, a_h, b_xy, b_w, b_z, b_h) -> bool:
    """Two different labels for one physical thing: same footprint, similar width, same height band."""
    if max(a_w, b_w) > SAME_PLACE_SIZE_RATIO * max(min(a_w, b_w), 0.05):
        return False
    if _footprint_iou(a_xy[0], a_xy[1], a_w, b_xy[0], b_xy[1], b_w) < SAME_PLACE_IOU:
        return False
    z_overlap = min(a_z + a_h, b_z + b_h) - max(a_z, b_z)
    return z_overlap >= SAME_PLACE_Z_OVERLAP * min(a_h, b_h)


def _box_overlap(a, b) -> float:
    """Intersection over the smaller box."""
    inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return inter / max(min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1])), 1)


def _box_iou(a, b) -> float:
    inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return inter / max((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1)


def _track_var(t) -> float:
    return 0.5 * float(t.P[0, 0] + t.P[1, 1])


def _dedup_radius(label: str) -> float:
    """Two sightings of the same class closer than this are the same physical object."""
    max_w, _ = OBJECT_MAX_SIZES.get(label, OBJECT_MAX_SIZE_DEFAULT)
    return max(0.5, min(1.5, 0.75 * max_w))


class KalmanTracker:
    """Ground-plane Kalman filter with state [X, Y, Vx, Vy, Ax, Ay] (constant acceleration).

    Static objects keep velocity/acceleration at zero, which turns the filter into an
    inverse-variance weighted average of all sightings: a close, LiDAR-ranged sighting
    counts far more than a distant optical estimate. Each update takes the measurement's
    own noise (sigma) instead of a fixed R.
    """
    def __init__(self, track_id: int, x: float, y: float, z: float, width: float, height: float,
                 conf: float, now: float, is_dynamic: bool, sigma: float, dist: float):
        self.id = track_id
        self.x = np.array([x, y, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.P = np.zeros((6, 6), dtype=np.float64)
        self.P[0, 0] = self.P[1, 1] = sigma * sigma
        if is_dynamic:
            self.P[2, 2] = self.P[3, 3] = DYN_INIT_VEL_VAR  # constant velocity: acceleration stays 0
        self.conf = conf
        self.is_dynamic = is_dynamic
        self.first_seen = now
        self.last_seen = now
        self.last_predict = now
        self.width = max(0.05, float(width))
        self.height = max(0.05, float(height))
        self.z = float(z)
        self.dist = float(dist)
        self.velocity = 0.0
        self.is_moving = False
        self.hits = 1
        self.misses = 0  # frames in which the camera looked at it and did not detect it
        self.conf_sum = conf
        self.ranged_hits = 0  # sightings ranged by LiDAR or metric depth (not box size alone)
        self.min_ranged_hits = 0  # set by the node to MEMORY_MIN_RANGED_HITS while metric depth runs
        self.last_box = None  # image box of the latest sighting, and when (image-space association)
        self.box_only_xy = []  # positions of consecutive ranged image-only matches (for re-anchoring)
        self.mono_ratio = None  # (moving objects) LiDAR range / metric depth, for sightings the LiDAR misses
        self.outlier_run, self.outlier_xy = 0, None  # (moving objects) consecutive out-of-gate sightings
        self.last_box_t = -math.inf
        self.is_reliable = False  # latched: once a real object, a bad angle later does not undo it
        self.unseen_in_view = 0.0
        self.free_in_view = 0.0
        self.seen_times = deque([now], maxlen=10)
        self.uid = 0  # globally unique object id (assigned by the node), used for RViz marker ids
        self.votes = {}  # label -> accumulated (priority-weighted) confidence, for look-alike classes
        self.from_memory = False  # reloaded from a saved map, not yet seen in this session
        self.wall_vec = np.zeros(2)  # weighted sum of (cos 2a, sin 2a) of the wall direction a (map)
        self.max_gap = 0.0  # longest time in plain view without a detection before it was seen again
        self.view_bins = set()  # directions (object -> camera, map) it has been detected from, REMOVE_VIEW_BIN_DEG bins
        self.colors = {}  # colour name -> confidence-weighted sightings

        self.H = np.zeros((2, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

    @property
    def detection_rate(self) -> float:
        return self.hits / (self.hits + self.misses)

    @property
    def confirmed(self) -> bool:
        if self.is_dynamic:
            return self.hits >= MIN_HITS_DYNAMIC
        return self.hits >= MIN_HITS_STATIC and (self.is_reliable or self.detection_rate >= MIN_DETECTION_RATE)

    @property
    def reliable(self) -> bool:
        """Seen often enough, over long enough and consistently enough to be a real object worth remembering."""
        if not self.is_reliable:
            self.is_reliable = (self.hits >= MEMORY_MIN_HITS and self.last_seen - self.first_seen >= MEMORY_MIN_SPAN
                                and self.detection_rate >= RELIABLE_DETECTION_RATE
                                and self.conf_sum / self.hits >= RELIABLE_MIN_MEAN_CONF
                                and self.dist <= MEMORY_MAX_RANGE
                                and self.ranged_hits >= self.min_ranged_hits)
        return self.is_reliable

    def remembered(self, now: float) -> bool:
        """Not detected now, but a reliable object seen within MEMORY_TTL."""
        return self.reliable and now - self.last_seen <= MEMORY_TTL

    def live(self, now: float) -> bool:
        """Confirmed and still being detected right now."""
        if self.is_dynamic and now - self.last_seen > DYN_SHOW_S:
            return False
        return self.confirmed and sum(1 for t in self.seen_times if now - t <= LIVE_WINDOW) >= LIVE_MIN_HITS

    def position_at(self, now: float):
        """(x, y) now: a moving object's last estimate carried forward by its velocity (briefly)."""
        if not self.is_dynamic:
            return float(self.x[0]), float(self.x[1])
        dt = max(0.0, min(now - self.last_predict, DYN_EXTRAPOLATE_S))
        return float(self.x[0] + self.x[2] * dt), float(self.x[1] + self.x[3] * dt)

    def predict(self, now: float):
        # dt is measured from the previous predict (not the last sighting), so calling this
        # at 20 Hz between detections no longer compounds the extrapolation.
        dt = now - self.last_predict
        if dt <= 0.0:
            return
        dt = min(dt, 0.5)
        self.last_predict = now

        if not self.is_dynamic:
            self.P[0, 0] += STATIC_POS_Q * dt
            self.P[1, 1] += STATIC_POS_Q * dt
            if now - self.last_seen > REVISIT_GAP_S:
                # Revisit: let the live sighting match the memory and pull it into place
                self.P[0, 0] = max(self.P[0, 0], REVISIT_VAR)
                self.P[1, 1] = max(self.P[1, 1], REVISIT_VAR)
            return

        # Constant velocity: position += velocity * dt, white-noise acceleration (a walking person)
        F = np.eye(6, dtype=np.float64)
        F[0, 2] = F[1, 3] = dt
        q = DYN_ACCEL_NOISE
        Q = np.zeros((6, 6))
        for p, v in ((0, 2), (1, 3)):
            Q[p, p], Q[p, v], Q[v, p], Q[v, v] = q * dt ** 3 / 3, q * dt ** 2 / 2, q * dt ** 2 / 2, q * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Unobserved objects coast to a stop instead of drifting forever
        if now - self.last_seen > 0.3:
            self.x[2:4] *= 0.80 ** (dt / 0.05)

    def mahalanobis_sq(self, px: float, py: float, sigma: float) -> float:
        y = np.array([px - self.x[0], py - self.x[1]])
        S = self.P[:2, :2] + np.eye(2) * sigma * sigma
        return float(y @ np.linalg.solve(S, y))

    def update(self, px: float, py: float, z: float, width: float, height: float, conf: float,
               now: float, sigma: float, dist: float):
        self.predict(now)

        R = np.eye(2, dtype=np.float64) * sigma * sigma
        y = np.array([px, py], dtype=np.float64) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        if not self.is_dynamic:
            self.x[2:6] = 0.0
            self.P[2:, :] = 0.0
            self.P[:, 2:] = 0.0
            self.P[0, 0] = max(self.P[0, 0], STATIC_MIN_VAR)
            self.P[1, 1] = max(self.P[1, 1], STATIC_MIN_VAR)

        # Size / elevation: plain exponential smoothing
        dim_alpha = 0.40 if self.is_dynamic else STATIC_DIM_ALPHA
        self.width = (1.0 - dim_alpha) * self.width + dim_alpha * max(0.05, float(width))
        self.height = (1.0 - dim_alpha) * self.height + dim_alpha * max(0.05, float(height))
        self.z = (1.0 - dim_alpha) * self.z + dim_alpha * float(z)
        self.dist = 0.5 * self.dist + 0.5 * float(dist)

        self.conf = max(conf, self.conf * 0.98)
        self.conf_sum += conf
        self.hits += 1
        self.last_seen = now
        self.seen_times.append(now)
        self.max_gap = max(self.max_gap, self.unseen_in_view)
        self.unseen_in_view = 0.0
        self.free_in_view = 0.0
        if self.is_dynamic:
            speed = float(math.hypot(self.x[2], self.x[3]))
            if speed > DYN_MAX_SPEED:
                self.x[2:4] *= DYN_MAX_SPEED / speed
                speed = DYN_MAX_SPEED
            # A speed from the first few sightings is mostly noise
            self.velocity = speed if now - self.first_seen >= DYN_SPEED_MIN_AGE else 0.0
            self.is_moving = self.velocity > (DYN_MOVING_OFF if self.is_moving else DYN_MOVING_ON)

    @property
    def distance(self) -> float:
        return float(math.hypot(self.x[0], self.x[1]))

    @property
    def color(self):
        """Colour agreed on by most sightings, or None while unsure."""
        total = sum(self.colors.values())
        if total < COLOR_MIN_VOTES:
            return None
        name = max(self.colors, key=self.colors.get)
        return name if self.colors[name] >= COLOR_MIN_SHARE * total else None

    @property
    def wall_yaw(self):
        """Direction (map) of the wall this panel lies in, or None if not known yet."""
        if np.linalg.norm(self.wall_vec) < 1e-6:
            return None
        return 0.5 * math.atan2(self.wall_vec[1], self.wall_vec[0])


class _InferredTable:
    """Render-only stand-in for a table that YOLO missed but desktop objects imply."""
    def __init__(self, idx, x, y, top_z, width, dist):
        self.id = idx
        self.x = np.array([x, y])
        self.z = 0.0
        self.height = top_z
        self.width = width
        self.dist = dist


class ObjectPerceptionNode(Node):
    def __init__(self, mode: str = "indoor") -> None:
        super().__init__("object_perception")

        self._bridge = CvBridge()
        self._frame_count = 0
        self._last_image_time = time.monotonic()

        # ── MODE INITIALIZATION ──
        self._mode = mode.lower()
        if self._mode not in MODE_PARAMS:
            self._mode = "indoor"
        self._apply_mode_params()

        cv2.setNumThreads(1)
        self._window_name = f"VisionNav AI [{self._mode.upper()}]"
        self._window_rename = None  # set by a mode switch; applied by the GUI thread

        # Auto-detect headless environment
        has_display = "DISPLAY" in os.environ or "WAYLAND_DISPLAY" in os.environ
        show_window_env = os.environ.get("WEARABLE_SHOW_WINDOW", "1")
        if show_window_env != "0" and has_display:
            self._show_window = True
        else:
            self._show_window = False
            if show_window_env != "0":
                self.get_logger().warn("No display detected. Forcing WEARABLE_SHOW_WINDOW=0 (Headless Mode).")

        # Press 'l' in the window to toggle the LiDAR-on-camera calibration overlay
        self._show_lidar_overlay = os.environ.get("WEARABLE_SHOW_LIDAR_OVERLAY", "0") == "1"
        # Diagnostics for the LiDAR snap (see _lidar_hits_in_box): logs, once per second per class,
        # why an object did or did not get a LiDAR-anchored position. Turn on to see why a mapped
        # object's distance disagrees with the LiDAR.
        self._lidar_snap_debug = os.environ.get("WEARABLE_LIDAR_SNAP_DEBUG", "0") == "1"
        self._lidar_debug_last_log = {}

        self.get_logger().info(f"Loading YOLO model: {MODEL_PATH}")
        self._load_model()
        self.get_logger().info("Model loaded. Ready for detections.")
        # Opened only now: during a first-start engine build (minutes) the window would sit frozen
        if self._show_window:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self._window_name, 800, 600)
            cv2.waitKey(1)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        realtime_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ── CAMERA ACQUISITION MODE (Direct USB or ROS Wi-Fi) ──
        self._camera_mode = os.environ.get("WEARABLE_CAMERA_MODE", "direct").lower()
        self._camera_pub = self.create_publisher(Image, '/camera/image_raw', realtime_qos)
        # The Pi's ROS camera stream arrives mirrored, so it is flipped back on arrival. From then
        # on the frame shows the world exactly as seen from the chest, and the HUD and all
        # geometry use it as-is. Override with WEARABLE_CAMERA_FLIP=0/1 for other cameras.
        flip_default = "1" if self._camera_mode == "ros" else "0"
        self._flip_input = os.environ.get("WEARABLE_CAMERA_FLIP", flip_default) == "1"

        self._inference_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_stamp = None
        self._latest_frame_rx = 0.0  # monotonic time the frame arrived (measurement time of moving objects)
        self._hud_frame = None       # (frame, time) the HUD boxes were computed on, for a synchronized display
        self._inference_results = None
        self._inference_busy = False
        self._recent_dets = deque()  # (time, detection) kept in the last LOOKALIKE_MEMORY_S (worker thread only)
        self._inference_thread = threading.Thread(target=self._yolo_worker, daemon=True)
        self._inference_thread.start()

        # Thread-safe persistent HUD tracks
        self._hud_lock = threading.Lock()
        self._hud_tracks = {}
        self._lidar_overlay = None

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
        # Finished inference is picked up within 10 ms (was up to 50 ms); prediction and markers at 20 Hz
        self._results_timer = self.create_timer(0.01, self._results_callback)
        self._tracking_timer = self.create_timer(0.05, self._tracking_callback)

        # Recent scans, so each image is fused with the scan taken at the same moment
        self._scan_buffer = deque(maxlen=40)
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data,
        )
        self._marker_pub = self.create_publisher(MarkerArray, "/semantic_markers", 10)
        # Alias requested for other tooling; voice_navigation_assistant.py and the RViz config use /semantic_markers
        self._marker_pub_alias = self.create_publisher(MarkerArray, "/vision_markers", 10)
        # Machine-readable object map for navigation (semantic_costmap.py / semantic_navigator.py)
        self._objects_pub = self.create_publisher(String, "/semantic_objects", 10)
        self._last_objects_pub = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        self._marker_pub.publish(MarkerArray(markers=[delete_marker]))
        self._marker_pub_alias.publish(MarkerArray(markers=[delete_marker]))

        # Dynamic object tracking for live map markers and warnings.
        self._hazard_pub = self.create_publisher(String, "/hazard_warning", 10)
        self._hazard_history = {}  # {label_id: (cx, cy, area, time)}

        # ── OUTDOOR MODE (outdoor_awareness.py): live hazards, no map ──
        # /outdoor_alert: the one sentence to say now (JSON text/level/kind; the assistant speaks it, a critical
        # one interrupting); /outdoor_scene: what is ahead, 2 Hz (JSON; "what is around me", the status)
        self._outdoor_tracker = oa.OutdoorTracker()
        self._alert_policy = oa.AlertPolicy()
        self._outdoor_alert_pub = self.create_publisher(String, "/outdoor_alert", 10)
        self._outdoor_scene_pub = self.create_publisher(String, "/outdoor_scene", 10)
        # RViz (rviz/visionnav_outdoor.rviz, fixed frame base_footprint: the wearer stays at the centre): only
        # what is detected right now. Nothing is remembered: an object that leaves the camera view is gone
        # from RViz on the next frame.
        self._outdoor_marker_pub = self.create_publisher(MarkerArray, "/outdoor_markers", 10)
        # Like a car: its own motion (LiDAR odometry, also on /odom), everything tracked world-fixed
        # (a parked car is still, a car's speed is its own), LiDAR objects tracked 360° and named by the camera,
        # and a live occupancy grid of the last few seconds (/outdoor_occupancy). Nothing is saved.
        self._odo = ScanOdometry()
        # Chest IMU (mpu6050_imu.py on the Pi), when fitted: its gyro gives the odometry's rotation guess
        self._gyro = GyroYaw()
        self.create_subscription(Imu, "/imu/data", self._gyro.add_msg, qos_profile_sensor_data)
        self._odom_enabled = os.environ.get("WEARABLE_OUTDOOR_ODOM", "1") == "1"
        self._ego_hist = deque(maxlen=60)   # (receive time, pose, velocity) per scan
        self._odom_good_run, self._odom_bad_since, self._odom_good = 0, None, False
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._occ = oa.LocalOccupancy()
        self._occ_pub = self.create_publisher(MarkerArray, "/outdoor_occupancy", 10)
        self._last_occ_pub = 0.0
        self._last_scene_pub = 0.0
        self._outdoor_hud = None     # corridor, lanes, crossing, last alert: drawn by the GUI thread
        self._last_alert = None      # (text, level, time)
        # WEARABLE_RECORD_DIR: every spoken alert (and a frame every RECORD_PERIOD_S) is saved there as an
        # annotated image + a line of alerts.jsonl, to review an outdoor walk afterwards
        self._record_dir = os.environ.get("WEARABLE_RECORD_DIR")
        self._last_record = 0.0
        if self._record_dir:
            os.makedirs(self._record_dir, exist_ok=True)

        # ── CAMERA MODEL ──
        # Intrinsics: HFOV-derived pinhole unless calibrated values are given.
        self._camera_hfov = math.radians(float(os.environ.get("WEARABLE_CAMERA_HFOV_DEG", "70.0")))
        self._calib_fx = os.environ.get("WEARABLE_CAMERA_FX")
        self._calib_fy = os.environ.get("WEARABLE_CAMERA_FY")
        self._calib_cx = os.environ.get("WEARABLE_CAMERA_CX")
        self._calib_cy = os.environ.get("WEARABLE_CAMERA_CY")
        # Extrinsics: read from TF (sensor_tf.launch.py). These env values are only a fallback
        # for running without the brain launch.
        fallback_h = float(os.environ.get("WEARABLE_CAMERA_HEIGHT", "1.3"))
        fallback_pitch = math.radians(float(os.environ.get("WEARABLE_CAMERA_PITCH_DEG", "0.0")))
        cp, sp = math.cos(fallback_pitch), math.sin(fallback_pitch)
        R_pitch = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        self._cam_R = R_pitch @ R_BODY_OPTICAL
        self._cam_t = np.array([0.0, 0.0, fallback_h])
        self._cam_from_tf = False
        self._lidar_R = None
        self._lidar_t = None
        self._lidar_frame = None
        self._last_extrinsics_check = 0.0
        self._warned = set()

        # ── TRACKS ──
        self._static_tracks = {}    # label -> [KalmanTracker]
        self._dynamic_tracks = {}
        self._inferred_tables = []
        self._last_process_time = time.monotonic()

        # Global object map: the static tracks themselves (label -> [KalmanTracker], map frame).
        self._next_uid = 1
        self._deleted_uids = []  # objects removed since the last marker publish (need Marker.DELETE)

        # ── RUNTIME MODE SWITCHING via /perception_mode topic ──
        self._mode_sub = self.create_subscription(
            String, "/perception_mode", self._mode_callback, 10
        )
        # The current mode, latched, so the MODE button (voice_navigation_assistant) knows which way to switch
        self._mode_state_pub = self.create_publisher(
            String, "/perception_mode_state",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._mode_state_pub.publish(String(data=self._mode))

        # ── SAVED MAPS (map_manager): reload the objects of a saved home, save them on request ──
        self._objects_file = None
        self._objects_loaded = False
        # Reloaded objects are protected from removal until one of them is seen again: before that
        # the wearer may not be localized in the saved map yet, so "not seen where expected" means nothing.
        self._memory_confirmed = True
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/active_map", self._active_map_callback, latched)
        self.create_subscription(String, "/map_command", self._map_command_callback, 10)

        # ── GRASP MODE: guide the hand to an object ("start cup" / "stop" on /grasp_command) ──
        self._grasp = GraspTracker(self.get_logger())
        self._grasp_status = None
        self._grasp_pub = self.create_publisher(String, "/grasp_offset", 10)
        self.create_subscription(String, "/grasp_command", self._grasp_command_callback, 10)

        self.get_logger().info(
            f"═══ VisionNav AI Perception Engine ═══\n"
            f"  Mode:               {self._mode.upper()}\n"
            f"  Confidence Thresh:  {self._conf_threshold:.2f}\n"
            f"  Static Timeout:     {self._static_track_timeout:.0f}s\n"
            f"  Dynamic Timeout:    {self._dynamic_track_timeout:.1f}s\n"
            f"  Danger Distance:    {self._danger_distance:.1f}m\n"
            f"  Corridor Width:     {self._collision_corridor_w:.1f}m\n"
            f"  Object Map:         {'PERSISTENT for this session' if self._mode == 'indoor' else 'LIVE ONLY'}\n"
            f"  Coordinate Frame:   {'map (global)' if self._mode == 'indoor' else 'base_footprint (local)'}\n"
            f"  FSD HUD:            Active ('l' toggles LiDAR overlay)"
        )

    def _warn_once(self, key: str, msg: str):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(msg)

    # ── MODE PARAMETER APPLICATION ──
    def _apply_mode_params(self):
        """Apply mode-specific parameters from MODE_PARAMS dict."""
        p = MODE_PARAMS[self._mode]
        self._conf_threshold = p["conf_threshold"]
        self._static_track_timeout = p["static_timeout"]
        self._dynamic_track_timeout = p["dynamic_timeout"]
        self._static_association_distance = p["static_assoc"]
        self._dynamic_association_distance = p["dynamic_assoc"]
        self._danger_distance = p["danger_distance"]
        self._collision_corridor_w = p["collision_corridor_w"]
        self._max_range = p["max_range"]
        # Moving things (tracked with velocity). Outdoors also vans, three-wheelers, cows...
        self._dynamic_classes = {"person", "bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat"}
        if self._mode == "outdoor":
            self._dynamic_classes = self._dynamic_classes | oa.MOVERS
        self._hazard_classes = self._dynamic_classes

    def _mode_callback(self, msg: String):
        """Runtime mode switching via /perception_mode topic."""
        new_mode = msg.data.strip().lower()
        if new_mode in MODE_PARAMS and new_mode != self._mode:
            old_mode = self._mode
            self._mode = new_mode
            self._apply_mode_params()

            # Tracks are stored in the frame of the old mode (map vs base_footprint)
            for tracks in list(self._static_tracks.values()) + list(self._dynamic_tracks.values()):
                self._deleted_uids.extend(t.uid for t in tracks)
            self._static_tracks.clear()
            self._dynamic_tracks.clear()
            self._inferred_tables = []
            with self._hud_lock:
                self._hud_tracks.clear()
            self._outdoor_tracker.clear()
            self._alert_policy.reset()
            self._outdoor_hud = None
            clear = Marker()
            clear.action = Marker.DELETEALL
            self._outdoor_marker_pub.publish(MarkerArray(markers=[clear]))
            self._occ_pub.publish(MarkerArray(markers=[clear]))
            self._odo.reset()
            self._ego_hist.clear()
            self._odom_good_run, self._odom_bad_since, self._odom_good = 0, None, False
            self._occ.reset()
            # Each mode has its own depth model, with its own scale
            if getattr(self, "_depth_models", None):
                self._depth_model = self._depth_models.get(self._mode)
                self._depth_scale, self._depth_scale_valid, self._depth_scale_t = 1.0, False, -math.inf

            # New window title: the window is replaced by the GUI (main) thread. Qt windows may only be touched
            # from that thread; doing it here, in a ROS callback, crashed the node on every mode switch.
            if self._show_window:
                self._window_rename = f"VisionNav AI [{self._mode.upper()}]"

            self.get_logger().info(f"🔄 Mode switched: {old_mode.upper()} → {new_mode.upper()}")
        if new_mode in MODE_PARAMS:
            self._mode_state_pub.publish(String(data=self._mode))

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
            if self._flip_input:
                frame = cv2.flip(frame, 1)

            self._frame_count += 1
            now_mono = time.monotonic()
            self._last_image_time = now_mono

            # 1. Update GUI display frame for main-thread rendering
            self._gui_frame = frame

            # 2. Feed freshest frame to YOLO worker (non-blocking)
            # Always the newest frame: the worker takes it the moment it finishes the previous one (a frame was
            # only kept while the worker was idle, so it then waited up to a frame period: 12.6 Hz, not 17)
            with self._inference_lock:
                self._latest_frame = frame.copy()
                self._latest_frame_stamp = self.get_clock().now().to_msg()
                self._latest_frame_rx = now_mono

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

        # Un-mirror the stream so left in the image is the wearer's left
        if self._flip_input:
            frame = cv2.flip(frame, 1)

        self._frame_count += 1
        now_mono = time.monotonic()
        self._last_image_time = now_mono
        self._gui_frame = frame

        # Always the newest frame: the worker takes it the moment it finishes the previous one
        with self._inference_lock:
            self._latest_frame = frame
            self._latest_frame_stamp = msg.header.stamp
            self._latest_frame_rx = now_mono

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_buffer.append(msg)
        if self._mode == "outdoor":
            self._outdoor_scan(msg, time.monotonic())

    # ══════════════════════════════════════════════════════════════════════
    # ── SENSOR GEOMETRY ──
    # ══════════════════════════════════════════════════════════════════════
    def _refresh_extrinsics(self, now: float):
        """Pull camera and LiDAR mounts from TF (retried every 2 s so launch order doesn't matter)."""
        if self._cam_from_tf and self._lidar_R is not None:
            return
        if now - self._last_extrinsics_check < 2.0:
            return
        self._last_extrinsics_check = now

        if not self._cam_from_tf:
            try:
                tf = self._tf_buffer.lookup_transform(BASE_FRAME, CAMERA_FRAME, Time())
                t = tf.transform.translation
                self._cam_t = np.array([t.x, t.y, t.z])
                self._cam_R = _quat_to_rot(tf.transform.rotation)
                self._cam_from_tf = True
                fwd = self._cam_R[:, 2]
                self.get_logger().info(
                    f"📐 Camera extrinsics from TF: height {t.z:.2f} m, "
                    f"pitch {math.degrees(math.asin(-max(-1.0, min(1.0, fwd[2])))):.1f}° down, "
                    f"yaw {math.degrees(math.atan2(fwd[1], fwd[0])):.1f}°"
                )
            except Exception:
                self._warn_once("cam_tf", f"No TF {BASE_FRAME} -> {CAMERA_FRAME} yet; using fallback camera "
                                          f"height {self._cam_t[2]:.2f} m. Start laptop_brain.launch.py.")

        if self._lidar_R is None and self._scan_buffer:
            frame_id = self._scan_buffer[-1].header.frame_id
            try:
                tf = self._tf_buffer.lookup_transform(BASE_FRAME, frame_id, Time())
                t = tf.transform.translation
                self._lidar_t = np.array([t.x, t.y, t.z])
                self._lidar_R = _quat_to_rot(tf.transform.rotation)
                self._lidar_frame = frame_id
                self.get_logger().info(
                    f"📐 LiDAR extrinsics from TF: height {t.z:.2f} m, "
                    f"x-axis yaw {math.degrees(math.atan2(self._lidar_R[1, 0], self._lidar_R[0, 0])):.0f}°"
                )
            except Exception:
                self._warn_once("lidar_tf", f"No TF {BASE_FRAME} -> {frame_id} yet; LiDAR fusion disabled.")

    def _intrinsics(self, w: int, h: int):
        fx = float(self._calib_fx) if self._calib_fx else (w / 2.0) / max(math.tan(self._camera_hfov / 2.0), 0.01)
        fy = float(self._calib_fy) if self._calib_fy else fx
        cx = float(self._calib_cx) if self._calib_cx else w / 2.0
        cy = float(self._calib_cy) if self._calib_cy else h / 2.0
        return fx, fy, cx, cy, w

    def _pixel_ray(self, u: float, v: float, K) -> np.ndarray:
        """Direction (base_footprint) of the ray through pixel (u, v) of the processed frame."""
        fx, fy, cx, cy, _ = K
        return self._cam_R @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

    def _ray_plane_distance(self, ray: np.ndarray, plane_z: float):
        """Horizontal distance from the camera where `ray` meets the plane z = plane_z."""
        if abs(ray[2]) < 1e-6:
            return None
        s = (plane_z - self._cam_t[2]) / ray[2]
        if s <= 0.0:
            return None
        return float(s * math.hypot(ray[0], ray[1]))

    def _height_along_ray(self, ray: np.ndarray, horiz_dist: float) -> float:
        return float(self._cam_t[2] + horiz_dist * ray[2] / max(math.hypot(ray[0], ray[1]), 1e-6))

    def _scan_for_stamp(self, stamp):
        if not self._scan_buffer:
            return None
        latest = self._scan_buffer[-1]
        if stamp is None:
            return latest
        t_img = _stamp_to_sec(stamp)
        best = min(self._scan_buffer, key=lambda s: abs(_stamp_to_sec(s.header.stamp) - t_img))
        if abs(_stamp_to_sec(best.header.stamp) - t_img) > SCAN_MATCH_MAX_DT:
            self._warn_once("scan_sync", "Camera and LiDAR stamps differ by >0.25 s (clock sync between Pi "
                                         "and laptop?). Using the latest scan instead.")
            return latest
        return best

    def _project_scan(self, scan: LaserScan, K, h: int):
        """Project LiDAR returns into the processed image.

        Returns (u, v, xy_base, horiz_dist, cam_depth) for points in front of the camera, or None.
        """
        if scan is None or self._lidar_R is None or scan.header.frame_id != self._lidar_frame:
            return None
        fx, fy, cx, cy, w = K
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (np.isfinite(ranges) & (ranges >= max(scan.range_min, LIDAR_BODY_RANGE))
                 & (ranges <= scan.range_max))
        if not np.any(valid):
            return None
        r, a = ranges[valid], angles[valid]
        pts_laser = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        pts_base = pts_laser @ self._lidar_R.T + self._lidar_t
        pts_cam = (pts_base - self._cam_t) @ self._cam_R
        front = pts_cam[:, 2] > 0.2
        pts_base, pts_cam = pts_base[front], pts_cam[front]
        u = fx * pts_cam[:, 0] / pts_cam[:, 2] + cx
        v = fy * pts_cam[:, 1] / pts_cam[:, 2] + cy
        inside = (u >= 0) & (u < w) & (v > -h) & (v < 2 * h)
        xy = pts_base[inside, :2]
        horiz = np.hypot(xy[:, 0] - self._cam_t[0], xy[:, 1] - self._cam_t[1])
        return u[inside], v[inside], xy, horiz, pts_cam[inside, 2]

    def _lidar_hits_in_box(self, proj, box, mask, depth_hint=None, debug=None):
        """Range the object with LiDAR points that physically land on it.

        A point qualifies if its column is inside the (slightly shrunk) box, the scan plane's
        image row at that range crosses the box, and — with a mask — the object occupies that
        pixel. The nearest range cluster wins (front surface); single stray returns are ignored.
        Fallback when the mounting heights in TF are off (e.g. the rig resting on a table, so the scan
        plane cuts through chair backs but is projected to the wrong image row): LiDAR points inside
        the object's own mask columns whose range agrees with the camera depth estimate
        (LIDAR_SNAP_RANGE x depth_hint). The wall behind an object is always farther than that.
        Returns (horiz_dist, xy_base_of_front_surface, n_points) or None.

        `debug`, if given a dict, is filled with counts explaining the outcome — see
        WEARABLE_LIDAR_SNAP_DEBUG.
        """
        if proj is None:
            if debug is not None:
                debug['reason'] = 'no_scan'
            return None
        u, v, xy, horiz, _ = proj
        x1, y1, x2, y2 = box
        # central 70 % of the box, but never narrower than 10 px (a switch is ~10 px wide)
        half = max(0.35 * (x2 - x1), 5.0)
        in_cols = np.abs(u - 0.5 * (x1 + x2)) <= half
        sel = np.nonzero(in_cols & (v >= y1 - LIDAR_ROW_TOL_PX) & (v <= y2 + LIDAR_ROW_TOL_PX))[0]
        if debug is not None:
            debug['in_cols'] = int(in_cols.sum())
            debug['row_test_pts'] = int(sel.size)
            if in_cols.any():
                debug['in_cols_range_m'] = (round(float(horiz[in_cols].min()), 2),
                                            round(float(horiz[in_cols].max()), 2))
        if sel.size < 2 and depth_hint is not None:
            lo, hi = LIDAR_SNAP_RANGE
            cand = np.nonzero(in_cols & (horiz >= lo * depth_hint) & (horiz <= hi * depth_hint))[0]
            if debug is not None:
                debug['snap_window_m'] = (round(lo * depth_hint, 2), round(hi * depth_hint, 2))
                debug['snap_pts_in_window'] = int(cand.size)
            if mask is not None and cand.size:
                cols = mask.any(axis=0)
                cand = cand[cols[np.clip(u[cand].astype(int), 0, cols.size - 1)]]
            if debug is not None:
                debug['snap_pts_after_mask'] = int(cand.size)
            if cand.size >= 3:
                order = cand[np.argsort(horiz[cand])]
                r = horiz[order]
                if debug is not None:
                    debug['reason'] = 'snap'
                return float(np.median(r)), np.median(xy[order], axis=0), int(order.size)
            if debug is not None:
                debug['reason'] = 'snap_too_few_pts'
        if sel.size < 2:
            if debug is not None and 'reason' not in debug:
                debug['reason'] = 'row_test_too_few_pts'
            return None

        if mask is not None:
            mh, mw = mask.shape
            keep = []
            for i in sel:
                ui, vi = int(u[i]), int(v[i])
                lo, hi = max(0, vi - LIDAR_ROW_TOL_PX), min(mh, vi + LIDAR_ROW_TOL_PX + 1)
                if 0 <= ui < mw and lo < hi and mask[lo:hi, ui].any():
                    keep.append(i)
            sel = np.asarray(keep, dtype=int)
            if debug is not None:
                debug['row_pts_after_mask'] = int(sel.size)
            if sel.size < 2:
                if debug is not None:
                    debug['reason'] = 'row_mask_too_few_pts'
                return None

        order = sel[np.argsort(horiz[sel])]
        r = horiz[order]
        min_pts = max(2, int(0.2 * len(order)))
        start = 0
        for end in list(np.nonzero(np.diff(r) > LIDAR_CLUSTER_GAP)[0]) + [len(r) - 1]:
            if end - start + 1 >= min_pts:
                cluster = order[start:end + 1]
                if debug is not None:
                    debug['reason'] = 'row'
                return float(np.median(r[start:end + 1])), np.median(xy[cluster], axis=0), int(cluster.size)
            start = end + 1
        if debug is not None:
            debug['reason'] = 'row_no_cluster'
        return None

    def _door_frame_range(self, proj, box):
        """Horizontal range (m) of the nearest LiDAR cluster on a door's jambs (the box's side bands)."""
        if proj is None:
            return None
        u, v, _, horiz, _ = proj
        x1, y1, x2, y2 = box
        band = 0.15 * (x2 - x1)
        sides = ((u >= x1 - band) & (u <= x1 + band)) | ((u >= x2 - band) & (u <= x2 + band))
        sel = np.nonzero(sides & (v >= y1 - LIDAR_ROW_TOL_PX) & (v <= y2 + LIDAR_ROW_TOL_PX)
                         & (horiz > LIDAR_BODY_RANGE))[0]
        r = np.sort(horiz[sel])
        start = 0
        for end in list(np.nonzero(np.diff(r) > LIDAR_CLUSTER_GAP)[0]) + [len(r) - 1]:
            if end - start + 1 >= 3:
                return float(np.median(r[start:end + 1]))
            start = end + 1
        return None

    def _wall_angle(self, proj, box, depth: float, ray_xy):
        """Direction (base_footprint, radians mod pi) of the wall a flat object lies in, and its weight.

        Fits a line to the LiDAR returns at the object's range in and beside its image columns (the wall
        around a switch, the jambs and wall either side of a door). Without enough of them, the object is
        assumed to face the camera (weak weight, so a later LiDAR fit wins).
        """
        if proj is not None:
            u, _, xy, horiz, _ = proj
            x1, _, x2, _ = box
            pad = max(20.0, 0.5 * (x2 - x1))
            sel = (u >= x1 - pad) & (u <= x2 + pad) & (np.abs(horiz - depth) < max(0.4, 0.15 * depth))
            if int(sel.sum()) >= WALL_FIT_MIN_PTS:
                pts = xy[sel] - xy[sel].mean(axis=0)
                evals, evecs = np.linalg.eigh(np.cov(pts.T))
                if evals[1] > WALL_FIT_MIN_ELONGATION * max(evals[0], 1e-6):
                    d = evecs[:, 1]
                    return math.atan2(d[1], d[0]), 1.0
        return math.atan2(ray_xy[1], ray_xy[0]) + math.pi / 2, 0.1

    def _log_lidar_snap_debug(self, label: str, depth_hint: float, lidar, info: dict):
        """Rate-limited (WEARABLE_LIDAR_SNAP_DEBUG=1) diagnostic for why an object did or did not get
        a LiDAR-anchored position — check this before touching LIDAR_SNAP_RANGE or the mask/row filters."""
        now = time.monotonic()
        if now - self._lidar_debug_last_log.get(label, 0.0) < 1.0:
            return
        self._lidar_debug_last_log[label] = now
        outcome = f"lidar={lidar[0]:.2f}m/{lidar[2]}pts" if lidar is not None else "none"
        self.get_logger().info(f"🔎 LiDAR snap [{label}] depth_hint={depth_hint:.2f}m -> {outcome} | {info}")

    def _update_depth_scale(self, proj, dmap):
        """Correct the depth network's scale with the LiDAR ranges visible in the same frame."""
        if proj is None or dmap is None:
            return
        u, v, _, _, zc = proj
        H, W = dmap.shape
        ok = (u >= 0) & (u < W) & (v >= 0) & (v < H) & (zc > 0.4) & (zc < 8.0)
        if ok.sum() < DEPTH_SCALE_MIN_PTS:
            return
        d = dmap[v[ok].astype(int), u[ok].astype(int)]
        ratio = zc[ok] / np.maximum(d, 0.05)
        lo, hi = DEPTH_SCALE_LIMITS[self._mode]
        ratio = ratio[(ratio > 0.8 * lo) & (ratio < 1.25 * hi)]
        if ratio.size < DEPTH_SCALE_MIN_PTS:
            return
        r = float(np.median(ratio))
        spread = float(np.median(np.abs(ratio / r - 1.0)))
        if spread > DEPTH_SCALE_MAX_RESID:
            return  # LiDAR and depth disagree in shape this frame (glitch / wrong row): keep the old scale
        self._depth_scale = r if not self._depth_scale_valid else (
            (1 - DEPTH_SCALE_ALPHA) * self._depth_scale + DEPTH_SCALE_ALPHA * r)
        self._depth_scale = max(lo, min(hi, self._depth_scale))
        self._depth_scale_valid = True
        self._depth_scale_resid = spread
        self._depth_scale_t = time.monotonic()

    def _mono_object(self, det, K, dmap):
        """Back-project the object's own pixels with metric depth.

        Returns the object's visible-surface centre, horizontal distance, width and the heights
        of its lowest/highest points in base_footprint, or None.
        """
        if dmap is None:
            return None
        fx, fy, cx, cy, _ = K
        x1, y1, x2, y2 = det["box"]
        bw, bh = x2 - x1, y2 - y1
        if bw < 6 or bh < 6:
            return None
        ys = xs = np.empty(0, dtype=int)
        mask = det.get("mask")
        if mask is not None:
            sub = mask[y1:y2, x1:x2]
            if bw > 12 and bh > 12:
                sub = cv2.erode(sub, np.ones((5, 5), np.uint8))  # edge pixels mix object and background
            ys, xs = np.nonzero(sub)
        if ys.size < 20:  # no usable mask: central part of the box
            yy, xx = np.mgrid[bh // 4: max(3 * bh // 4, bh // 4 + 1), bw // 4: max(3 * bw // 4, bw // 4 + 1)]
            ys, xs = yy.ravel(), xx.ravel()
        if ys.size > MONO_MAX_POINTS:
            pick = np.linspace(0, ys.size - 1, MONO_MAX_POINTS).astype(int)
            ys, xs = ys[pick], xs[pick]
        us, vs = xs + x1, ys + y1
        z = dmap[vs, us] * self._depth_scale
        # Keep the object's own surface (reject background seen through gaps, e.g. chair backs)
        near = float(np.percentile(z, 30))
        keep = (z > near - max(0.25, 0.15 * near)) & (z < near + max(0.5, 0.35 * near))
        if keep.sum() < 10:
            return None
        us, vs, z = us[keep], vs[keep], z[keep]
        p_opt = np.stack([(us - cx) / fx * z, (vs - cy) / fy * z, z], axis=1)
        p_base = p_opt @ self._cam_R.T + self._cam_t
        cam_xy = self._cam_t[:2]
        xy = np.median(p_base[:, :2], axis=0)
        dist = float(np.linalg.norm(xy - cam_xy))
        if dist < 0.2:
            return None
        direction = (xy - cam_xy) / dist
        lateral = (p_base[:, :2] - cam_xy) @ np.array([-direction[1], direction[0]])
        return {
            "dist": dist, "xy": xy,
            "width": float(np.percentile(lateral, 95) - np.percentile(lateral, 5)),
            "z_lo": float(np.percentile(p_base[:, 2], 3)), "z_hi": float(np.percentile(p_base[:, 2], 97)),
        }

    def _measure_detection(self, det, K, h, proj, dmap=None):
        """Turn one 2D detection into a ground-plane measurement in base_footprint."""
        fx, fy, cx0, cy0, w = K
        x1, y1, x2, y2 = det["box"]
        label, raw = det["label"], det["raw_label"]
        is_dynamic = raw in self._dynamic_classes or label in self._dynamic_classes
        outdoor = self._mode == "outdoor"
        max_range = self._max_range
        pix_w, pix_h = max(1.0, x2 - x1), max(1.0, y2 - y1)
        u_mid = 0.5 * (x1 + x2)
        cut_l, cut_r = x1 <= BOX_EDGE_PX, x2 >= w - BOX_EDGE_PX
        cut_t, cut_b = y1 <= BOX_EDGE_PX, y2 >= h - BOX_EDGE_PX
        max_w, max_h = _max_size(label, raw)
        typ_w, typ_h = _typical_size(label, raw)
        # Without a known real size the size prior is a guess: it may not veto LiDAR or metric depth
        known_size = _lookup(OBJECT_TYPICAL_SIZES, label, raw) is not None or \
            _lookup(OBJECT_MAX_SIZES, label, raw) is not None
        size_gate = 3.0 if known_size else 1e6
        cam_z = float(self._cam_t[2])
        on_floor = raw in FLOOR_OBJECTS or label in FLOOR_OBJECTS or is_dynamic or (outdoor and label in oa.OUTDOOR_GROUND)
        on_desk = not outdoor and not on_floor and (raw in DESKTOP_OBJECTS or label in DESKTOP_OBJECTS)

        # ── 1. OPTICAL DEPTH CANDIDATES (depth, sigma) ──
        # Known-size prior, using only box dimensions not cut off by the frame edge
        d_w = typ_w * fx / pix_w
        d_h = typ_h * fy / pix_h
        if label in FLAT_OBJECTS or raw in FLAT_OBJECTS or raw == "person" or (outdoor and label in oa.OUTDOOR_FLAT):
            size_d = d_h if (raw == "person" and not (cut_t or cut_b)) else d_w
        elif not (cut_t or cut_b) and not (cut_l or cut_r):
            size_d = math.sqrt(d_w * d_h)
        elif not (cut_t or cut_b):
            size_d = d_h
        elif not (cut_l or cut_r):
            size_d = d_w
        else:
            size_d = min(d_w, d_h)
        estimates = [(size_d, (0.35 if known_size else 1.5) * size_d + 0.05)]

        def plausible(d):
            return d is not None and 0.3 < d < max_range and size_d / size_gate < d < size_d * size_gate

        # Support-plane contact: the box's bottom edge touches the floor / desk
        support_z = 0.0 if on_floor else (_lookup(SUPPORT_HEIGHTS, label, raw, DESK_HEIGHT) if on_desk else None)
        if support_z is not None and not cut_b:
            dh = cam_z - support_z
            d = self._ray_plane_distance(self._pixel_ray(u_mid, y2, K), support_z)
            if dh > 0.15 and plausible(d):
                sigma = 0.05 + (d * d + dh * dh) / dh * PITCH_SIGMA + (0.0 if on_floor else d / dh * 0.05)
                estimates.append((d, sigma))

        # Standard-height top surface (table tops): the box's top edge is the far edge
        top_z = _lookup(OBJECT_TOP_HEIGHTS, label, raw)
        if top_z is not None and not cut_t and abs(cam_z - top_z) > 0.2:
            dh = abs(cam_z - top_z)
            d = self._ray_plane_distance(self._pixel_ray(u_mid, y1, K), top_z)
            if plausible(d):
                estimates.append((d, 0.05 + (d * d + dh * dh) / dh * PITCH_SIGMA + d / dh * 0.04))

        # Metric depth of the object's own pixels — the main range source below the LiDAR plane
        mono = self._mono_object(det, K, dmap)
        mono_gate = 4.0 if known_size else 1e6
        if mono is not None and not (0.3 < mono["dist"] < max_range and size_d / mono_gate < mono["dist"] < size_d * mono_gate):
            mono = None
        if mono is not None:
            fresh = time.monotonic() - self._depth_scale_t <= DEPTH_SCALE_FRESH_S
            rel = 0.06 if fresh else (0.15 if self._depth_scale_valid else 0.25)
            estimates.append((mono["dist"], 0.05 + rel * mono["dist"]))

        weights = [1.0 / (s * s) for _, s in estimates]
        depth = sum(d * wt for (d, _), wt in zip(estimates, weights)) / sum(weights)
        sigma_d = math.sqrt(1.0 / sum(weights))
        source = "depth" if mono is not None else "optical"

        # ── 2. LIDAR RANGING (overrides optical when the scan plane actually hits the object) ──
        front_xy = None
        lidar_debug = {} if self._lidar_snap_debug else None
        # The LiDAR scans one plane at chest height. An object whose top is clearly below it (a chair,
        # a bottle on a desk) cannot be hit, so it may not borrow the range of whatever is in the same
        # image columns (a table edge, the wall): that gave confident but wrong positions and duplicates.
        snap_hint = depth
        if not LIDAR_SNAP_ANY_HEIGHT and self._lidar_t is not None and not cut_t:
            top_est = self._height_along_ray(self._pixel_ray(u_mid, y1, K), depth)
            if top_est < self._lidar_t[2] - LIDAR_SNAP_PLANE_MARGIN:
                snap_hint = None
        # A wall-mounted plate is flush with its wall: the return from the wall right beside it is its range
        # (its own mask is too small to contain a scan point)
        obj_mask = None if label in WALL_MOUNTED else det.get("mask")
        lidar_info = {} if lidar_debug is None else lidar_debug
        lidar = self._lidar_hits_in_box(proj, det["box"], obj_mask, depth_hint=snap_hint, debug=lidar_info)
        if lidar_debug is not None:
            self._log_lidar_snap_debug(label, depth, lidar, lidar_debug)
        # Returns inside the object's own mask at the scan row are the object: no optical estimate may veto
        # them (a person's cut-off box, sized 2.3 m too far, discarded a correct LiDAR range). Only the weaker
        # column-only "snap" match must agree with the camera.
        lidar_gate = 1e6 if lidar_info.get("reason") == "row" else (4.0 if (known_size or mono is not None) else 1e6)
        if lidar is not None and depth / lidar_gate < lidar[0] < depth * lidar_gate:
            depth, front_xy, _ = lidar
            sigma_d = 0.04 + 0.01 * depth
            source = "lidar"
        # A door sits in its wall: range it by the frame (jambs at the box's sides). Through an open
        # doorway the middle of the box sees the next room, which put the door metres too far away.
        is_door = label == "door"
        if is_door:
            jambs = self._door_frame_range(proj, det["box"])
            if jambs is not None and (source != "lidar" or jambs < depth - 0.5):
                depth, sigma_d, source = jambs, 0.04 + 0.01 * jambs, "lidar"
                ray = self._pixel_ray(u_mid, 0.5 * (y1 + y2), K)
                front_xy = self._cam_t[:2] + depth * ray[:2] / max(np.linalg.norm(ray[:2]), 1e-6)
        depth = max(0.3, min(depth, max_range))

        # ── 3. POSITION (base_footprint) ──
        cam_xy = self._cam_t[:2]
        # Metric-depth points, rescaled to the final range, give the object's real shape
        k = depth / mono["dist"] if mono is not None else 1.0
        if front_xy is None:
            if mono is not None:
                direction = (mono["xy"] - cam_xy) / mono["dist"]
            else:
                ray = self._pixel_ray(u_mid, 0.5 * (y1 + y2), K)
                direction = ray[:2] / max(np.linalg.norm(ray[:2]), 1e-6)
            front_xy = cam_xy + depth * direction
        else:
            direction = (front_xy - cam_xy) / max(np.linalg.norm(front_xy - cam_xy), 1e-6)

        width_px_m = mono["width"] * k if mono is not None else depth * pix_w / fx
        width_m = max(0.05, min(width_px_m, max_w))
        # LiDAR and ground contact measure the nearest face; the marker goes at the centre.
        # Metric-depth points already sit on the visible surface's centre.
        push = not is_door and (source == "lidar" or (mono is None and len(estimates) > 1))
        center_xy = front_xy + direction * (0.5 * min(width_m, 0.6) if push else 0.0)

        # ── 4. ELEVATION AND HEIGHT: from the object's 3D points, else from the box-edge rays ──
        if mono is not None:
            z_top = cam_z + (mono["z_hi"] - cam_z) * k
            z_bot = cam_z + (mono["z_lo"] - cam_z) * k
        else:
            z_top = self._height_along_ray(self._pixel_ray(u_mid, y1, K), depth)
            z_bot = self._height_along_ray(self._pixel_ray(u_mid, y2, K), depth)
        if on_floor:
            base_z = 0.0
        elif on_desk:
            base_z = max(0.3, min(z_bot, cam_z + 0.3)) if not cut_b else DESK_HEIGHT
        else:
            base_z = max(0.0, z_bot) if not cut_b else max(0.0, z_top - typ_h)
        height_m = z_top - base_z
        if cut_t:
            height_m = max(height_m, typ_h)
        if is_door:
            height_m = max(height_m, DOOR_MIN_HEIGHT)
        height_m = max(0.05, min(height_m, max_h))
        wall = self._wall_angle(proj, det["box"], depth, direction) if label in PANEL_OBJECTS else None

        sigma_xy = math.sqrt(sigma_d ** 2 + (0.02 * depth) ** 2)
        return {
            **det,
            "is_dynamic": is_dynamic, "z_top": z_top, "wall": wall,
            "mono_dist": mono["dist"] if mono is not None else None,  # raw metric depth (per-person calibration)
            "bx": float(center_xy[0]), "by": float(center_xy[1]),
            "depth": depth, "sigma": sigma_xy, "source": source,
            "z": base_z, "width_m": width_m, "height_m": height_m,
        }

    def _lookup_base_pose(self, stamp):
        """(x, y, yaw) of base_footprint in the map at the image time (latest as fallback)."""
        times = ([Time.from_msg(stamp)] if stamp is not None else []) + [Time()]
        for target in ("map", "odom"):
            for t in times:
                try:
                    tf = self._tf_buffer.lookup_transform(target, BASE_FRAME, t)
                except Exception:
                    continue
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return tf.transform.translation.x, tf.transform.translation.y, yaw
        return None

    # ══════════════════════════════════════════════════════════════════════
    # ── DETECTION ──
    # ══════════════════════════════════════════════════════════════════════
    def _load_detector(self, torch, mode: str):
        """YOLOE with the mode's offline vocabulary. Returns (model, is_tensorrt_engine).

        Text embeddings are computed once by set_classes() and baked into the TensorRT engine, so
        the per-frame path never runs a text encoder. The engine is built on first start (or after
        the vocabulary changes), before the ROS loop begins.
        """
        from ultralytics import YOLO, YOLOE
        vocabulary, engine_path, _, _ = DETECTORS[mode]
        if torch.cuda.is_available():
            if not os.path.isfile(engine_path):
                self.get_logger().info(f"Building TensorRT engine {os.path.basename(engine_path)} "
                                       f"({len(vocabulary)} classes, one-time, 3-6 minutes; the camera window "
                                       f"opens when it is done — do not close this terminal)...")
                try:
                    model = YOLOE(MODEL_PATH)
                    model.set_classes(vocabulary, model.get_text_pe(vocabulary))
                    # 2 GB workspace: the RTX 2050 has 4 GB, and a 4 GB request ran the GPU out of memory
                    built = model.export(format="engine", half=True, workspace=2, imgsz=640, device=0)
                    shutil.move(str(built), engine_path)
                    for old in glob.glob(model_path(f"yoloe-11s-seg-{mode}-*.engine")):  # older vocabularies
                        if old != engine_path:
                            os.remove(old)
                    stem = os.path.splitext(MODEL_PATH)[0]
                    for onnx in (stem + ".onnx", stem + ".fp16.onnx"):  # export intermediates
                        if os.path.isfile(onnx):
                            os.remove(onnx)
                    del model
                    torch.cuda.empty_cache()
                except Exception as e:
                    self.get_logger().error(f"TensorRT export failed, using PyTorch weights: {e}")
            if os.path.isfile(engine_path):
                model = YOLO(engine_path, task="segment")
                return model, True
        model = YOLOE(MODEL_PATH)
        model.set_classes(vocabulary, model.get_text_pe(vocabulary))
        return model, False

    def _load_model(self):
        import torch
        from ultralytics.cfg import DEFAULT_CFG_DICT
        self._use_half = torch.cuda.is_available()
        # Both modes' detectors stay loaded (~0.2 GB each), so the MODE button switches at once
        self._detectors = {}
        engines = []
        for mode, (_, _, classes, _) in DETECTORS.items():
            model, on_engine = self._load_detector(torch, mode)
            engines.append(on_engine)
            # The TensorRT engine is already fp16; newer Ultralytics replaced half=True with quantize="fp16"
            if on_engine or not self._use_half:
                precision = {}
            elif "quantize" in DEFAULT_CFG_DICT:
                precision = {"quantize": "fp16"}
            else:
                precision = {"half": True}
            names = model.names
            self._detectors[mode] = (model, names, [i for i, n in names.items() if n in classes], precision)
            self.get_logger().info(f"YOLO {mode} detector: {'CUDA fp16' if self._use_half else 'CPU'}"
                                   f"{' (TensorRT)' if on_engine else ''}, {len(classes)} classes")
        self.get_logger().info(f"YOLO device: {'CUDA fp16' if self._use_half else 'CPU'}"
                               f"{' (TensorRT)' if all(engines) else ''}")

        self._depth_model = None
        self._depth_models = {}
        self._depth_scale, self._depth_scale_valid, self._depth_scale_resid = 1.0, False, 0.0
        self._depth_scale_t = -math.inf
        self._last_depth_log = 0.0
        if not MONO_DEPTH_ENABLED:
            return
        if not (torch.cuda.is_available() and os.path.isfile(DEPTH_WEIGHTS)):
            self.get_logger().warn(f"Metric depth disabled (needs CUDA and {DEPTH_WEIGHTS}).")
            return
        try:
            import warnings
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")  # "xFormers not available" is harmless
                from visionnav.depth_anything_v2.dpt import DepthAnythingV2
            for mode, weights in (("indoor", DEPTH_WEIGHTS), ("outdoor", DEPTH_WEIGHTS_OUTDOOR)):
                if not os.path.isfile(weights):
                    self.get_logger().warn(f"No {os.path.basename(weights)}: {mode} mode uses the indoor depth model "
                                           f"(it reads no farther than 20 m).")
                    continue
                model = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384],
                                        max_depth=DEPTH_MAX[mode])
                model.load_state_dict(torch.load(weights, map_location='cpu'))
                self._depth_models[mode] = model.cuda().half().eval()
            self._depth_models.setdefault("outdoor", self._depth_models.get("indoor"))
            self._depth_model = self._depth_models.get(self._mode)
            self._depth_mean = torch.tensor([0.485, 0.456, 0.406], device='cuda').view(1, 3, 1, 1)
            self._depth_std = torch.tensor([0.229, 0.224, 0.225], device='cuda').view(1, 3, 1, 1)
            self.get_logger().info(f"Metric depth: Depth Anything V2 indoor"
                                   f"{' + outdoor' if self._depth_models['outdoor'] is not self._depth_models['indoor'] else ''}"
                                   f" (input {DEPTH_INPUT_SIZE}px, CUDA fp16)")
        except Exception as e:
            self._depth_model = None
            self._depth_models = {}
            self.get_logger().warn(f"Metric depth disabled: {e}")

    def _infer_depth(self, bgr, model=None):
        """Per-pixel metric depth (m, along the optical axis) for the processed frame."""
        model = model or self._depth_model
        import torch
        import torch.nn.functional as F
        h, w = bgr.shape[:2]
        short = DEPTH_INPUT_SIZE
        ih, iw = ((short, int(round(short * w / h / 14)) * 14) if h <= w
                  else (int(round(short * h / w / 14)) * 14, short))
        with torch.no_grad():
            x = torch.from_numpy(bgr).cuda()[..., [2, 1, 0]].permute(2, 0, 1)[None].float().div_(255.0)
            x = F.interpolate(x, (ih, iw), mode='bilinear', align_corners=False)
            x = ((x - self._depth_mean) / self._depth_std).half()
            d = model(x)[:, None].float()
            d = F.interpolate(d, (h, w), mode='bilinear', align_corners=False)[0, 0]
        return d.cpu().numpy()

    @staticmethod
    def _confusable(a: str, b: str) -> bool:
        return any(a in g and b in g for g in CONFUSABLE_GROUPS)

    def _yolo_worker(self):
        while rclpy.ok():
            frame = None
            stamp = None
            with self._inference_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame
                    stamp = self._latest_frame_stamp
                    rx = self._latest_frame_rx
                    self._latest_frame = None
                    self._inference_busy = True

            if frame is None:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            mode = self._mode
            outdoor = mode == "outdoor"
            yolo_model, model_names, class_ids, precision = self._detectors[mode]
            friendly = oa.OUTDOOR_SYNONYMS if outdoor else FRIENDLY_NAMES
            confusable = oa._confusable if outdoor else self._confusable
            try:
                results = yolo_model.predict(
                    frame, conf=self._conf_threshold, iou=YOLO_IOU,
                    classes=class_ids, verbose=False, **precision,
                )[0]
            except Exception as e:
                self.get_logger().error(f"YOLO inference failed: {e}")
                with self._inference_lock:
                    self._inference_busy = False
                continue

            dets = []
            if results.boxes is not None and len(results.boxes) > 0:
                xyxy = results.boxes.xyxy.cpu().numpy()
                confs = results.boxes.conf.cpu().numpy()
                clss = results.boxes.cls.cpu().numpy().astype(int)
                polys = results.masks.xy if results.masks is not None else [None] * len(xyxy)

                for (x1, y1, x2, y2), conf, cid, poly in zip(xyxy, confs, clss, polys):
                    raw_label = model_names.get(int(cid), "unknown")
                    if not outdoor and raw_label in SMALL_OBJECTS and conf < max(SMALL_OBJECT_CONF, self._conf_threshold):
                        continue
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    bw_, bh_ = max(1, x2 - x1), max(1, y2 - y1)

                    # Box-shape corrections tuned on indoor furniture (a car seen head-on is taller than 0.8 x wide)
                    if not outdoor and bh_ / float(bw_) > OBJECT_REJECT_ASPECT_RATIOS.get(raw_label, 1e9):
                        continue
                    max_aspect = 100.0 if outdoor else OBJECT_MAX_ASPECT_RATIOS.get(raw_label, 100.0)
                    if bh_ / float(bw_) > max_aspect:
                        # Anchor to the bottom (desk/floor) and slice the top off
                        y1 = max(0, int(y2 - bw_ * max_aspect))

                    mask = None
                    if poly is not None and len(poly) >= 3:
                        mask = np.zeros((h, w), dtype=np.uint8)
                        cv2.fillPoly(mask, [poly.astype(np.int32)], 1)

                    dets.append({
                        "box": (x1, y1, x2, y2), "conf": float(conf), "raw_label": raw_label,
                        "label": friendly.get(raw_label, raw_label), "mask": mask,
                    })

            dmap = None
            depth_model = self._depth_model
            # Outdoors every frame: the ground analysis sees what the detector has no name for
            if depth_model is not None and (dets or self._grasp.active or outdoor):
                try:
                    dmap = self._infer_depth(frame, depth_model)
                except Exception as e:
                    self._warn_once("depth_fail", f"Metric depth inference failed: {e}")

            # ── CROSS-CLASS SUPPRESSION: same object reported as two classes ──
            # Look-alikes are suppressed on a large overlap; any two static labels only on (almost)
            # the same box (a bed also read as a crib, a pillow and a laundry basket).
            dets.sort(key=_ranking_score, reverse=True)
            kept = []
            for d in dets:
                ax1, ay1, ax2, ay2 = d["box"]
                duplicate = False
                for k in kept:
                    if k["raw_label"] == d["raw_label"]:
                        continue
                    bx1, by1, bx2, by2 = k["box"]
                    inter = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(0, min(ay2, by2) - max(ay1, by1))
                    area_a, area_b = (ax2 - ax1) * (ay2 - ay1), (bx2 - bx1) * (by2 - by1)
                    both_static = (k["raw_label"] not in self._dynamic_classes and k["label"] not in self._dynamic_classes
                                   and d["raw_label"] not in self._dynamic_classes and d["label"] not in self._dynamic_classes)
                    if both_static and inter / max(area_a + area_b - inter, 1) >= CROSS_CLASS_SAME_BOX_IOU:
                        duplicate = True
                        break
                    if not (k["label"] == d["label"] or confusable(k["label"], d["label"])
                            or confusable(k["raw_label"], d["raw_label"])):
                        continue
                    if inter / max(min(area_a, area_b), 1) > CROSS_CLASS_OVERLAP:
                        duplicate = True
                        break
                if not duplicate:
                    kept.append(d)

            # A switch or socket is never on the door leaf itself: there it is the door handle (read as an
            # "electric switch" on the rig), or something seen through an open doorway
            doors = [k["box"] for k in kept if k["label"] == "door"]
            if doors:
                def inside(a, b):
                    inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
                    return inter >= 0.8 * max(1, (a[2] - a[0]) * (a[3] - a[1]))
                kept = [d for d in kept if not (d["label"] in WALL_MOUNTED and d["label"] not in ("door handle", "door knob")
                                                and any(inside(d["box"], b) for b in doors))]

            # Across frames: a label flicker to a worse-ranked look-alike on the same box is dropped
            t_now = time.monotonic()
            while self._recent_dets and t_now - self._recent_dets[0][0] > LOOKALIKE_MEMORY_S:
                self._recent_dets.popleft()

            def flicker(d):
                ax1, ay1, ax2, ay2 = d["box"]
                for _, r in self._recent_dets:
                    if r["label"] == d["label"] or _ranking_score(r) <= _ranking_score(d) or not (
                            confusable(r["label"], d["label"]) or confusable(r["raw_label"], d["raw_label"])):
                        continue
                    bx1, by1, bx2, by2 = r["box"]
                    inter = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(0, min(ay2, by2) - max(ay1, by1))
                    if inter / max(min((ax2 - ax1) * (ay2 - ay1), (bx2 - bx1) * (by2 - by1)), 1) > CROSS_CLASS_OVERLAP:
                        return True
                return False

            kept = [d for d in kept if not flicker(d)]
            self._recent_dets.extend((t_now, d) for d in kept)

            with self._inference_lock:
                self._inference_results = (stamp, kept, frame, dmap, rx, mode)
                self._inference_busy = False

    # ══════════════════════════════════════════════════════════════════════
    # ── TRACKING ──
    # ══════════════════════════════════════════════════════════════════════
    def _results_callback(self):
        """Process a finished inference as soon as it is ready."""
        with self._inference_lock:
            new_results, self._inference_results = self._inference_results, None
        if new_results is not None:
            stamp, dets, frame, dmap, rx, mode = new_results
            if mode != self._mode:
                return  # computed just before a mode switch, with the other mode's detector
            now = time.monotonic()
            self._process_detections(stamp, dets, frame, now, dmap, t_meas=min(rx, now))
            self._publish_markers(now)

    def _tracking_callback(self):
        now = time.monotonic()
        # Static objects only: a moving object's state stays at its last sighting (drawn carried forward)
        for tracks in self._static_tracks.values():
            for track in tracks:
                track.predict(now)
        self._publish_markers(now)

    def _associate(self, label: str, meas: list, is_dynamic: bool, now: float):
        """Globally optimal (Hungarian) matching of this frame's detections of one class to its tracks."""
        tracks_dict = self._dynamic_tracks if is_dynamic else self._static_tracks
        timeout = self._dynamic_track_timeout if is_dynamic else self._static_track_timeout
        max_gate = self._dynamic_association_distance if is_dynamic else self._static_association_distance
        min_sep = _min_separation(label)

        tracks = tracks_dict.setdefault(label, [])
        self._purge_tracks(tracks, is_dynamic, now)
        for t in tracks:
            t.predict(now)

        assigned = [None] * len(meas)
        if tracks:
            cost = np.full((len(meas), len(tracks)), BIG_COST)
            for i, m in enumerate(meas):
                for j, t in enumerate(tracks):
                    if is_dynamic:
                        cost[i, j] = self._dynamic_cost(m, t, now, max_gate)
                        continue
                    e = math.hypot(m["px"] - t.x[0], m["py"] - t.x[1])
                    m2 = t.mahalanobis_sq(m["px"], m["py"], m["sigma"])
                    if e <= max_gate and (m2 <= CHI2_GATE_2D or e <= min_sep):
                        size_penalty = abs(m["width_m"] - t.width) / max(t.width, 0.1)
                        cost[i, j] = m2 + 2.0 * size_penalty
                    elif (t.last_box is not None and now - t.last_box_t <= BOX_TRACK_MAX_AGE
                          and _box_iou(m["box"], t.last_box) >= BOX_TRACK_IOU):
                        cost[i, j] = BOX_ONLY_COST  # same object in the image; worse than any 3-D match
            if linear_sum_assignment is not None:
                rows, cols = linear_sum_assignment(cost)
                pairs = zip(rows, cols)
            else:
                pairs, used = [], set()
                for i in range(len(meas)):
                    j = int(np.argmin(cost[i]))
                    if j not in used:
                        pairs.append((i, j))
                        used.add(j)
            for i, j in pairs:
                if cost[i, j] < BIG_COST:
                    assigned[i] = tracks[j]
                    meas[i]["box_only"] = not is_dynamic and cost[i, j] == BOX_ONLY_COST

        for i, m in enumerate(meas):
            t = assigned[i]
            if t is None:
                # A second detection on top of an existing object is a duplicate, never a new object
                # (not for people: two can stand side by side)
                near = [] if is_dynamic else [tr for tr in tracks if _same_object((m["px"], m["py"]), _dedup_width(label, m["width_m"]),
                                                            m["sigma"] ** 2, tr.x[:2], _dedup_width(label, tr.width),
                                                            _track_var(tr), min_sep)]
                if near:
                    m["track"] = max(near, key=lambda tr: tr.hits)
                    continue
                existing = {tr.id for tr in tracks}
                track_id = 1
                while track_id in existing:
                    track_id += 1
                if TRACK_DEBUG and not is_dynamic:
                    near_txt = ", ".join(
                        f"{label}_{tr.id} d={math.hypot(m['px'] - tr.x[0], m['py'] - tr.x[1]):.2f} "
                        f"m2={tr.mahalanobis_sq(m['px'], m['py'], m['sigma']):.1f} "
                        f"sd={math.sqrt(_track_var(tr)):.2f} hits={tr.hits}"
                        for tr in tracks if math.hypot(m['px'] - tr.x[0], m['py'] - tr.x[1]) < 1.5)
                    self.get_logger().info(
                        f"[track-debug] NEW {label}_{track_id} at ({m['px']:.2f},{m['py']:.2f}) "
                        f"src={m['source']} sigma={m['sigma']:.2f} depth={m['depth']:.2f} "
                        f"w={m['width_m']:.2f} | nearby: {near_txt or 'none'}")
                t = KalmanTracker(track_id, m["px"], m["py"], m["z"], m["width_m"], m["height_m"],
                                  m["conf"], now, is_dynamic, m["sigma"], m["depth"])
                t.min_ranged_hits = MEMORY_MIN_RANGED_HITS if self._depth_model is not None else 0
                t.uid = self._next_uid
                self._next_uid += 1
                tracks.append(t)
            else:
                # Matched in the image only: weight the position by how far it jumped
                sigma = m["sigma"]
                if is_dynamic:
                    sigma = max(sigma, DYN_MIN_SIGMA)
                    # Outside the 99 % gate of where the person should be: the range hit something else (the
                    # wall behind, the person next to them) — keep the track, weight that range by its jump.
                    # Such flips were the fake 1-2 m/s speeds of people standing still.
                    if t.mahalanobis_sq(m["px"], m["py"], sigma) > CHI2_GATE_2D:
                        prev = t.outlier_xy
                        t.outlier_xy = (m["px"], m["py"])
                        t.outlier_run = t.outlier_run + 1 if prev is not None and math.hypot(
                            m["px"] - prev[0], m["py"] - prev[1]) <= 3 * DYN_MIN_SIGMA else 1
                        if t.outlier_run >= DYN_REANCHOR_RUN:
                            # Several sightings agree on the new place: the track was wrong, not them. Re-anchor
                            # there at rest (coasting on a speed made from a range flip gave 1.5 m/s to a
                            # person standing still)
                            t.x[:2], t.x[2:4] = (m["px"], m["py"]), 0.0
                            t.P[:] = 0.0
                            t.P[0, 0] = t.P[1, 1] = sigma * sigma
                            t.P[2, 2] = t.P[3, 3] = DYN_INIT_VEL_VAR
                            t.outlier_run, t.outlier_xy = 0, None
                        else:
                            sigma = max(sigma, math.hypot(m["px"] - t.x[0], m["py"] - t.x[1]))
                    else:
                        t.outlier_run, t.outlier_xy = 0, None
                keep_shape = False
                if not is_dynamic and not m.get("box_only"):
                    t.box_only_xy = []
                elif not is_dynamic:
                    if m["source"] != "optical":
                        t.box_only_xy = (t.box_only_xy + [(m["px"], m["py"])])[-BOX_SNAP_RUN:]
                    run = np.array(t.box_only_xy)
                    if len(run) >= BOX_SNAP_RUN and np.linalg.norm(run - np.median(run, axis=0), axis=1).max() <= BOX_SNAP_SPREAD:
                        t.box_only_xy = []
                        t.x[:2] = np.median(run, axis=0)
                        t.P[0, 0] = t.P[1, 1] = max(sigma * sigma, STATIC_MIN_VAR)
                    else:
                        # A range that disagrees with the map: the object stays put and keeps its shape
                        sigma = max(sigma, math.hypot(m["px"] - t.x[0], m["py"] - t.x[1]))
                        keep_shape = True
                t.update(m["px"], m["py"], t.z if keep_shape else m["z"], t.width if keep_shape else m["width_m"],
                         t.height if keep_shape else m["height_m"], m["conf"], now, sigma, m["depth"])
                if t.from_memory:
                    t.from_memory = False
                    if not self._memory_confirmed:
                        self._memory_confirmed = True
                        self.get_logger().info(f"🏠 Localized: remembered {label}_{t.id} seen again where "
                                               f"it was saved; the saved object map is live")
            m["track"] = t

        if is_dynamic:
            return  # two people sitting side by side are two people, never "duplicates" to merge
        merged_into = self._merge_duplicate_tracks(tracks, label)
        self._deleted_uids.extend(t.uid for t in merged_into.pop("_removed", []))
        for m in meas:
            m["track"] = merged_into.get(id(m["track"]), m["track"])

    def _dynamic_cost(self, m, t, now, max_gate):
        """Matching cost of a moving object's sighting to a track. Image continuity first: people close
        together (two sitting side by side at the same range) are told apart by their boxes, which move on
        smoothly from frame to frame, far better than by 3-D distance."""
        if t.last_box is not None and now - t.last_box_t <= DYN_BOX_MAX_AGE:
            iou = _box_iou(m["box"], t.last_box)
            if iou >= DYN_BOX_IOU:
                return 10.0 * (1.0 - iou)
        e = math.hypot(m["px"] - t.x[0], m["py"] - t.x[1])
        m2 = t.mahalanobis_sq(m["px"], m["py"], max(m["sigma"], DYN_MIN_SIGMA))
        if e <= max_gate and m2 <= CHI2_GATE_2D:
            return 10.0 + m2
        return BIG_COST

    def _lookalike_track(self, m):
        """Label of a static track of another class on this measurement's spot (same physical object):
        a look-alike on an overlapping footprint, or any label on the same footprint, size and height.
        Returns (label, image_only) or (None, False); image_only when it matched only in the image."""
        def same(t, lbl, min_sep=0.0):
            return _same_object((m["px"], m["py"]), _dedup_width(m["label"], m["width_m"]), m["sigma"] ** 2,
                                t.x[:2], _dedup_width(lbl, t.width), _track_var(t), min_sep)

        min_sep = _min_separation(m["label"])
        if any(same(t, m["label"], min_sep) for t in self._static_tracks.get(m["label"], [])):
            return None, False
        # In the image: a look-alike's box where a confirmed object was just seen is that object, whatever
        # the distance says (an open doorway's "mirror" sightings ranged through the opening, 7.8 m away)
        now = time.monotonic()
        best = None
        for lbl, tracks in self._static_tracks.items():
            if lbl == m["label"] or not self._confusable(lbl, m["label"]):
                continue
            for t in tracks:
                if (t.confirmed and t.last_box is not None and now - t.last_box_t <= LOOKALIKE_BOX_AGE
                        and _box_overlap(m["box"], t.last_box) > CROSS_CLASS_OVERLAP
                        and (best is None or t.hits > best[1].hits)):
                    best = (lbl, t)
        if best is not None:
            in_3d = _same_object((m["px"], m["py"]), _dedup_width(m["label"], m["width_m"]), m["sigma"] ** 2,
                                 best[1].x[:2], _dedup_width(best[0], best[1].width), _track_var(best[1]))
            return best[0], not in_3d
        for lbl, tracks in self._static_tracks.items():
            if lbl == m["label"]:
                continue
            confusable = self._confusable(lbl, m["label"])
            for t in tracks:
                if not t.confirmed:  # a flickering hallucination may not claim a real object's sightings
                    continue
                match = same(t, lbl) if confusable else _same_place(
                    (m["px"], m["py"]), _dedup_width(m["label"], m["width_m"]), m["z"], m["height_m"],
                    t.x[:2], _dedup_width(lbl, t.width), t.z, t.height)
                if match and (best is None or t.hits > best[1].hits):
                    best = (lbl, t)
        return (best[0], False) if best else (None, False)

    def _relabel_tracks(self):
        """Rename a track whose evidence clearly favours a look-alike label (wardrobe_1 -> door_1).

        The object keeps its uid (RViz marker) and position; only its class and name change.
        Returns {id(track): new label}.
        """
        moves, renamed = [], {}
        for lbl, tracks in self._static_tracks.items():
            for t in tracks:
                if not t.votes:
                    continue
                best = max(t.votes, key=t.votes.get)
                if (best != lbl and t.votes[best] >= RELABEL_MIN_VOTES
                        and t.votes[best] >= RELABEL_RATIO * t.votes.get(lbl, 0.0)):
                    moves.append((lbl, t, best))
        for old, t, new in moves:
            self._static_tracks[old].remove(t)
            dest = self._static_tracks.setdefault(new, [])
            taken = {tr.id for tr in dest}
            t.id = 1
            while t.id in taken:
                t.id += 1
            dest.append(t)
            renamed[id(t)] = new
            self.get_logger().info(f"Relabelled {old} -> {new}_{t.id} (look-alike evidence {t.votes[new]:.1f})")
        return renamed

    # ── GRASP MODE ──
    def _grasp_command_callback(self, msg: String):
        words = msg.data.strip().lower().split(maxsplit=1)
        if words and words[0] == "start" and len(words) > 1:
            ok = self._grasp.start(words[1])
            if not ok:
                self._grasp_pub.publish(String(data=json.dumps({"target": words[1], "state": "unavailable"})))
        elif words and words[0] == "stop":
            self._grasp.stop()
            self._grasp_status = None

    def _grasp_step(self, frame, dets, dmap, K):
        """Hand-to-target offset for this frame, published on /grasp_offset."""
        if dmap is None:
            status = {"target": self._grasp.target, "state": "no_depth"}
        else:
            fx, fy, cx, cy, _ = K
            dh, dw = dmap.shape

            def depth_at(us, vs):
                us = np.clip(np.asarray(us, dtype=int), 0, dw - 1)
                vs = np.clip(np.asarray(vs, dtype=int), 0, dh - 1)
                return dmap[vs, us] * self._depth_scale

            def to_base(u, v, z):
                return self._cam_R @ np.array([(u - cx) / fx * z, (v - cy) / fy * z, z]) + self._cam_t

            status = self._grasp.update(frame, dets, depth_at, to_base)
        self._grasp_status = status
        self._grasp_pub.publish(String(data=json.dumps(status)))

    def _draw_grasp(self, frame):
        """Target box, hand point and the remaining offset while grasp mode is on."""
        overlay = self._grasp.overlay() if self._grasp.active else None
        if overlay is None:
            return
        (x1, y1, x2, y2), hand = overlay
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
        st = self._grasp_status or {}
        if hand is not None:
            cv2.circle(frame, hand, 7, (0, 255, 255), -1)
            cv2.line(frame, hand, ((x1 + x2) // 2, (y1 + y2) // 2), (0, 255, 255), 2)
        if "right" in st:
            text = (f"GRASP {st['target']}: right {100 * st['right']:+.0f}  up {100 * st['up']:+.0f}  "
                    f"fwd {100 * st['forward']:+.0f} cm  [{st['state']}]")
        else:
            text = f"GRASP {st.get('target', '')}: {st.get('state', '')}"
        cv2.putText(frame, text, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    # ── SAVED MAPS ──
    def _active_map_callback(self, msg: String):
        try:
            info = json.loads(msg.data)
            self._objects_file = os.path.join(info["dir"], f"{info['name']}_objects.json")
        except (ValueError, KeyError, TypeError):
            return
        if info.get("mode") == "localization" and self._mode == "indoor" and not self._objects_loaded:
            self._load_objects()

    def _map_command_callback(self, msg: String):
        if msg.data.strip().lower() == "save" and self._objects_file and self._mode == "indoor":
            self._save_objects()

    def _save_objects(self):
        objects = [{"class": label, "x": round(float(t.x[0]), 3), "y": round(float(t.x[1]), 3),
                    "z": round(float(t.z), 3), "w": round(t.width, 3), "h": round(t.height, 3), "hits": t.hits,
                    **({"color": t.color} if t.color else {})}
                   for label, tracks in self._static_tracks.items() for t in tracks if t.reliable]
        try:
            with open(self._objects_file, "w") as f:
                json.dump(objects, f, indent=1)
            self.get_logger().info(f"💾 Saved {len(objects)} objects to {self._objects_file}")
        except OSError as e:
            self.get_logger().error(f"Could not save objects: {e}")

    def _load_objects(self):
        self._objects_loaded = True
        try:
            with open(self._objects_file) as f:
                objects = json.load(f)
        except (OSError, ValueError):
            return
        now = time.monotonic()
        for o in objects:
            tracks = self._static_tracks.setdefault(o["class"], [])
            taken = {t.id for t in tracks}
            track_id = 1
            while track_id in taken:
                track_id += 1
            t = KalmanTracker(track_id, o["x"], o["y"], o["z"], o["w"], o["h"], 0.5, now,
                              False, math.sqrt(REVISIT_VAR), 0.0)
            # Reliable and remembered, but not live: drawn faded until the camera sees it again
            t.hits = max(int(o.get("hits", 0)), MEMORY_MIN_HITS)
            t.is_reliable = True
            t.conf_sum = 0.5 * t.hits
            t.ranged_hits = t.hits
            t.first_seen = now - MEMORY_MIN_SPAN - REVISIT_GAP_S - 1.0
            t.last_seen = now - REVISIT_GAP_S - 1.0
            t.seen_times.clear()
            t.from_memory = True
            if o.get("color"):
                t.colors = {o["color"]: COLOR_MIN_VOTES}
            t.uid = self._next_uid
            self._next_uid += 1
            tracks.append(t)
        self._memory_confirmed = not objects
        self.get_logger().info(f"🏠 Reloaded {len(objects)} remembered objects from {self._objects_file}")

    def _protected(self, t) -> bool:
        """A reloaded object that must not be removed yet (the wearer may not be localized)."""
        return t.from_memory and not self._memory_confirmed

    def _track_ttl(self, t, is_dynamic: bool) -> float:
        """How long a track may go unseen before it is removed."""
        if is_dynamic:
            return self._dynamic_track_timeout
        if self._mode == "indoor" and t.reliable:
            return MEMORY_TTL
        if t.confirmed:
            return min(self._static_track_timeout, CONFIRMED_TIMEOUT)
        return min(self._static_track_timeout, UNCONFIRMED_TIMEOUT)

    def _purge_tracks(self, tracks: list, is_dynamic: bool, now: float):
        survivors = []
        for t in tracks:
            if now - t.last_seen <= self._track_ttl(t, is_dynamic):
                survivors.append(t)
            else:
                self._deleted_uids.append(t.uid)
        tracks[:] = survivors

    def _shown(self, t, is_dynamic: bool, now: float) -> bool:
        """On the map: detected now, or (indoor, static) remembered for a short while."""
        return t.live(now) or (self._mode == "indoor" and not is_dynamic and t.remembered(now))

    @staticmethod
    def _fold_track(twin, t):
        """Merge track `t` into `twin` (same physical object): inverse-variance position, pooled evidence."""
        wa, wb = 1.0 / max(twin.P[0, 0], 1e-4), 1.0 / max(t.P[0, 0], 1e-4)
        twin.x[:2] = (wa * twin.x[:2] + wb * t.x[:2]) / (wa + wb)
        twin.P[:2, :2] = np.eye(2) / (wa + wb)
        twin.hits += t.hits
        twin.misses += t.misses
        twin.wall_vec = twin.wall_vec + t.wall_vec
        twin.view_bins |= t.view_bins
        twin.max_gap = max(twin.max_gap, t.max_gap)
        for c, v in t.colors.items():
            twin.colors[c] = twin.colors.get(c, 0.0) + v
        twin.conf_sum += t.conf_sum
        twin.ranged_hits += t.ranged_hits
        twin.is_reliable = twin.is_reliable or t.is_reliable
        twin.first_seen = min(twin.first_seen, t.first_seen)
        twin.last_seen = max(twin.last_seen, t.last_seen)
        twin.seen_times.extend(t.seen_times)
        twin.seen_times = deque(sorted(twin.seen_times), maxlen=twin.seen_times.maxlen)
        for lbl, v in t.votes.items():
            twin.votes[lbl] = twin.votes.get(lbl, 0.0) + v

    def _merge_duplicate_tracks(self, tracks: list, label: str):
        """Fold together same-class tracks that drifted onto the same spot (keeps the older ID)."""
        tracks.sort(key=lambda t: (-t.hits, t.id))
        min_sep = _min_separation(label)
        kept, merged_into = [], {}
        for t in tracks:
            twin = next((k for k in kept if _same_object(t.x[:2], _dedup_width(label, t.width), _track_var(t),
                                                         k.x[:2], _dedup_width(label, k.width), _track_var(k),
                                                         min_sep)), None)
            if twin is None:
                kept.append(t)
                continue
            self._fold_track(twin, t)
            merged_into[id(t)] = twin
            merged_into.setdefault("_removed", []).append(t)
        tracks[:] = sorted(kept, key=lambda t: t.id)
        return merged_into

    def _merge_same_place_tracks(self):
        """Fold static tracks of different labels that sit on one spot (one object mapped under several
        names from different views) into the best-supported one. Returns {id(folded track): (kept, label)}."""
        # Unconfirmed tracks take no part: a frequent but flickering misdetection must not absorb a real object
        entries = sorted(((lbl, t) for lbl, tracks in self._static_tracks.items() for t in tracks if t.confirmed),
                         key=lambda e: -e[1].hits)
        kept, folded = [], {}
        for lbl, t in entries:
            host = None
            for klbl, k in kept:
                if klbl == lbl:
                    continue
                if self._confusable(klbl, lbl):
                    same = _same_object(t.x[:2], _dedup_width(lbl, t.width), _track_var(t),
                                        k.x[:2], _dedup_width(klbl, k.width), _track_var(k))
                else:
                    same = _same_place(t.x[:2], _dedup_width(lbl, t.width), t.z, t.height,
                                       k.x[:2], _dedup_width(klbl, k.width), k.z, k.height)
                if same:
                    host = (klbl, k)
                    break
            if host is None:
                kept.append((lbl, t))
                continue
            self._fold_track(host[1], t)
            self._static_tracks[lbl].remove(t)
            self._deleted_uids.append(t.uid)
            folded[id(t)] = host
        return folded

    def _in_view_batch(self, xyz: np.ndarray, pose, K, h: int, margin: float = 0.12,
                       max_range: float = VISIBILITY_MAX_RANGE):
        """Which mapped objects (rows of xyz: x, y, z_mid in the map) should the camera be seeing now?

        One vectorised projection for all objects. Returns (in_view, u, v, cam_depth) arrays: the pixel
        each would project to and its distance along the camera axis, so a caller can check whether
        something closer occludes it (dmap[v, u] < cam_depth).
        """
        fx, fy, cx, cy, w = K
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = xyz[:, 0] - px, xyz[:, 1] - py
        p_base = np.stack([c * dx + s * dy, -s * dx + c * dy, xyz[:, 2]], axis=1)
        pc = (p_base - self._cam_t) @ self._cam_R
        near = np.hypot(p_base[:, 0] - self._cam_t[0], p_base[:, 1] - self._cam_t[1]) <= max_range
        front = pc[:, 2] >= 0.5
        zc = np.where(front, pc[:, 2], 1.0)
        u = fx * pc[:, 0] / zc + cx
        v = fy * pc[:, 1] / zc + cy
        lo, hi = margin, 1.0 - margin
        in_view = near & front & (lo * w < u) & (u < hi * w) & (lo * h < v) & (v < hi * h)
        return in_view, u, v, pc[:, 2]

    def _occluded(self, u: float, v: float, cam_depth: float, dmap, w: int, h: int, proj=None,
                  half_px: float = 10.0) -> bool:
        """True if the metric depth map (or, without one, the LiDAR) shows something clearly closer here."""
        if dmap is None:
            if proj is None:
                return False
            pu, _, _, _, zc = proj
            # Most of the returns in the object's columns must be closer: a door jamb at the edge of an object
            # seen through a doorway does not hide it
            col = zc[np.abs(pu - u) < half_px]
            return col.size >= 3 and float(np.median(col)) < cam_depth - OCCLUSION_DEPTH_MARGIN
        ui, vi = int(u), int(v)
        if not (0 <= ui < w and 0 <= vi < h):
            return False
        d = dmap[vi, ui] * self._depth_scale
        return d > 0.05 and d < cam_depth - OCCLUSION_DEPTH_MARGIN

    @staticmethod
    def _view_bin(obj_xy, cam_xy) -> int:
        a = math.degrees(math.atan2(cam_xy[1] - obj_xy[1], cam_xy[0] - obj_xy[0])) % 360.0
        return int(a // REMOVE_VIEW_BIN_DEG)

    def _seen_from_here(self, t, cam_xy) -> bool:
        n = int(round(360.0 / REMOVE_VIEW_BIN_DEG))
        b = self._view_bin(t.x[:2], cam_xy)
        return any((b + k) % n in t.view_bins for k in (-1, 0, 1))

    def _visible_fraction(self, t, pose, K, w: int, h: int) -> float:
        """Share of the object's projected 3-D box (footprint x height) that falls inside the image."""
        fx, fy, cx, cy, _ = K
        px, py, yaw = pose
        c, s_ = math.cos(yaw), math.sin(yaw)
        half = 0.5 * t.width
        corners = np.array([(t.x[0] + dx, t.x[1] + dy, z) for dx in (-half, half) for dy in (-half, half)
                            for z in (t.z, t.z + t.height)])
        dx, dy = corners[:, 0] - px, corners[:, 1] - py
        p_base = np.stack([c * dx + s_ * dy, -s_ * dx + c * dy, corners[:, 2]], axis=1)
        pc = (p_base - self._cam_t) @ self._cam_R
        if np.any(pc[:, 2] < 0.3):
            return 0.0
        us, vs = fx * pc[:, 0] / pc[:, 2] + cx, fy * pc[:, 1] / pc[:, 2] + cy
        x1, x2, y1, y2 = us.min(), us.max(), vs.min(), vs.max()
        area = max(1.0, (x2 - x1) * (y2 - y1))
        inside = max(0.0, min(x2, w) - max(x1, 0)) * max(0.0, min(y2, h) - max(y1, 0))
        return inside / area

    def _spot_empty(self, u: float, v: float, cam_depth: float, t, K, dmap) -> bool:
        """Does the depth at the object's own spot (its size, not widened) read clearly behind it?"""
        if dmap is None:
            return False
        fx = K[0]
        H, W = dmap.shape
        half = max(3, int(0.3 * fx * t.width / max(cam_depth, 0.1)))
        ui, vi = int(u), int(v)
        patch = dmap[max(0, vi - half):min(H, vi + half + 1), max(0, ui - half):min(W, ui + half + 1)]
        margin = max(REMOVE_FREE_MARGIN, 0.15 * cam_depth)
        return patch.size > 0 and float(np.median(patch)) * self._depth_scale > cam_depth + margin

    def _free_space(self, u: float, v: float, cam_depth: float, t, K, dmap, proj) -> bool:
        """Ray-cast clearing: does live depth show open space well beyond a remembered object's spot?

        Uses the low percentile of a patch of the depth map (or, without depth, the LiDAR points
        crossing the object's columns while the scan plane could physically hit it), so any surface
        still at the object's range keeps it alive. A remembered object's patch is widened by
        FREE_SPACE_SLACK and its 10th percentile used, so a few decimetres of SLAM/depth offset while
        walking cannot make the patch miss the object and read the wall behind it.
        """
        fx, _, _, _, w = K
        if t.reliable:
            half_px = max(3, int(0.5 * fx * (t.width + 2 * FREE_SPACE_SLACK) / max(cam_depth, 0.1)))
            pct = 10
        else:
            half_px = max(3, int(0.3 * fx * t.width / max(cam_depth, 0.1)))
            pct = 25
        limit = cam_depth + FREE_SPACE_MARGIN
        if dmap is not None:
            H, W = dmap.shape
            ui, vi = int(u), int(v)
            patch = dmap[max(0, vi - half_px):min(H, vi + half_px + 1), max(0, ui - half_px):min(W, ui + half_px + 1)]
            return patch.size > 0 and float(np.percentile(patch, pct)) * self._depth_scale > limit
        if proj is not None and self._lidar_t is not None and t.z <= self._lidar_t[2] <= t.z + t.height:
            pu, _, _, _, zc = proj
            col = zc[np.abs(pu - u) < half_px]
            return col.size >= 3 and float(col.min()) > limit
        return False

    def _dynamic_box_match(self, m, t):
        """The moving-object track whose recent image box this measurement's box overlaps most, or None."""
        best, best_iou = None, DYN_BOX_IOU
        for tr in self._dynamic_tracks.get(m["label"], []):
            if tr.last_box is not None and t - tr.last_box_t <= DYN_BOX_MAX_AGE:
                iou = _box_iou(m["box"], tr.last_box)
                if iou >= best_iou:
                    best, best_iou = tr, iou
        return best

    def _rescale_measurement(self, m, depth, pose):
        """Move a measurement along its viewing ray to a corrected range."""
        px0, py0, yaw0 = pose
        cam = self._cam_t[:2]
        k = depth / max(m["depth"], 1e-3)
        m["bx"], m["by"] = cam[0] + (m["bx"] - cam[0]) * k, cam[1] + (m["by"] - cam[1]) * k
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        m["px"], m["py"] = px0 + c * m["bx"] - s_ * m["by"], py0 + s_ * m["bx"] + c * m["by"]
        m["depth"] = depth
        m["sigma"] = 0.05 + 0.05 * depth

    def _process_detections(self, msg_stamp, dets, frame, now, dmap=None, t_meas=None):
        """`t_meas`: when the frame arrived (monotonic) — the time moving objects are tracked at."""
        t_meas = now if t_meas is None else t_meas
        h, w = frame.shape[:2]
        frame_dt = min(0.5, now - self._last_process_time)
        self._last_process_time = now
        current_hazards = {}

        with self._hud_lock:
            # Boxes are in pixel coordinates: once stale they sit over whatever the camera turned
            # to next, so they must disappear quickly.
            self._hud_tracks = {k: v for k, v in self._hud_tracks.items() if now - v['last_seen'] <= HUD_TIMEOUT}

        self._refresh_extrinsics(now)
        K = self._intrinsics(w, h)
        proj = self._project_scan(self._scan_for_stamp(msg_stamp), K, h)
        with self._hud_lock:
            self._lidar_overlay = None if proj is None else (proj[0], proj[1], proj[3])
        self._update_depth_scale(proj, dmap)
        if self._grasp.active:
            self._grasp_step(frame, dets, dmap, K)
        if dmap is not None and now - self._last_depth_log > 10.0:
            self._last_depth_log = now
            if self._depth_scale_valid:
                self.get_logger().info(f"📏 Depth calibrated by LiDAR: scale {self._depth_scale:.2f}, "
                                       f"median error vs LiDAR {100 * self._depth_scale_resid:.0f}%")
            else:
                self.get_logger().warn("📏 Depth not LiDAR-calibrated yet (no scan points in view)")

        if self._mode == "outdoor":
            self._process_outdoor(msg_stamp, dets, frame, now, dmap, proj, K, t_meas)
            self._hud_frame = (frame, now)
            return

        # base_footprint -> map at the moment the image was taken
        if self._mode == "indoor":
            pose = self._lookup_base_pose(msg_stamp)
            if pose is None:
                self._warn_once("map_tf", "No map/odom TF yet — objects are not being mapped. Is SLAM running?")
                return
        else:
            pose = (0.0, 0.0, 0.0)  # Outdoor: stay in base_footprint (no TF2 needed, minimum latency)
        px0, py0, yaw0 = pose
        cyaw, syaw = math.cos(yaw0), math.sin(yaw0)

        # Colour references are relative to the frame's white level (95th percentile of brightness)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) if any(d.get("mask") is not None for d in dets) else None
        vref = max(120.0, float(np.percentile(hsv[..., 2], 95))) if hsv is not None else 255.0

        measurements = []
        for det in dets:
            m = self._measure_detection(det, K, h, proj, dmap)
            if m["label"] in FLOOR_LEVEL_HAZARDS and m["source"] != "optical" and m["z_top"] > FLOOR_HAZARD_MAX_TOP:
                continue  # measured well above the floor: not a step or hole (a counter edge, a shelf)
            if (m["label"] == "door" and m["width_m"] < DOOR_MIN_WIDTH
                    and BOX_EDGE_PX < m["box"][0] and m["box"][2] < w - BOX_EDGE_PX):
                continue
            m["px"] = px0 + cyaw * m["bx"] - syaw * m["by"]
            m["py"] = py0 + syaw * m["bx"] + cyaw * m["by"]
            if m["wall"] is not None:
                m["wall"] = (m["wall"][0] + yaw0, m["wall"][1])
            measurements.append(m)

        # The same physical object seen under a look-alike label (a door read as a wardrobe) updates
        # its existing track instead of starting a second object on the same spot
        for m in measurements:
            m["seen_label"] = m["label"]
            if not m["is_dynamic"]:
                lookalike, image_only = self._lookalike_track(m)
                if lookalike is not None:
                    m["label"] = lookalike
                    if image_only:
                        # Same box, but ranged somewhere else (seen through an open doorway): it is that
                        # object, and no evidence for renaming it
                        m["seen_label"] = lookalike

        # Camera position in the map (viewing directions of objects)
        cam_xy_map = (px0 + cyaw * self._cam_t[0] - syaw * self._cam_t[1], py0 + syaw * self._cam_t[0] + cyaw * self._cam_t[1])

        # A person the LiDAR misses this frame (arm's-length, bent down, between scan points): their metric
        # depth, corrected by their own LiDAR/depth ratio from earlier frames (depth read people 0.37 m long)
        for m in measurements:
            if m["is_dynamic"] and m["source"] != "lidar" and m.get("mono_dist"):
                tr = self._dynamic_box_match(m, t_meas)
                if tr is not None and tr.mono_ratio is not None:
                    self._rescale_measurement(m, tr.mono_ratio * m["mono_dist"], pose)
                    m["source"] = "depth"

        groups = {}
        for m in measurements:
            groups.setdefault((m["label"], m["is_dynamic"]), []).append(m)
        for (label, is_dynamic), group in groups.items():
            self._associate(label, group, is_dynamic, t_meas if is_dynamic else now)
        for m in measurements:
            t_obs = t_meas if m["is_dynamic"] else now
            if m["source"] != "optical" and m["track"].last_seen == t_obs:
                m["track"].ranged_hits += 1
            if m["is_dynamic"] and m["source"] == "lidar" and m.get("mono_dist"):
                r = max(0.5, min(2.0, m["depth"] / m["mono_dist"]))
                tr = m["track"]
                tr.mono_ratio = r if tr.mono_ratio is None else (1 - MONO_RATIO_ALPHA) * tr.mono_ratio + MONO_RATIO_ALPHA * r
            if hsv is not None and not m["is_dynamic"] and m.get("mask") is not None:
                x1, y1, x2, y2 = m["box"]
                color = _color_name(hsv[y1:y2, x1:x2], vref, m["mask"][y1:y2, x1:x2])
                if color is not None:
                    m["track"].colors[color] = m["track"].colors.get(color, 0.0) + m["conf"]
            m["track"].last_box, m["track"].last_box_t = m["box"], t_obs
            if not m["is_dynamic"] and m["track"].last_seen == now:
                m["track"].view_bins.add(self._view_bin(m["track"].x[:2], cam_xy_map))
            if m["wall"] is not None:
                a, wt = m["wall"]
                m["track"].wall_vec = m["track"].wall_vec + wt * np.array([math.cos(2 * a), math.sin(2 * a)])
            votes = m["track"].votes
            boost = PRIORITY_BOOST if m["seen_label"] in PRIORITY_LABELS else 1.0
            votes[m["seen_label"]] = votes.get(m["seen_label"], 0.0) + m["conf"] * boost
        renamed = self._relabel_tracks()
        for m in measurements:
            m["label"] = renamed.get(id(m["track"]), m["label"])
        folded = self._merge_same_place_tracks()
        for m in measurements:
            if id(m["track"]) in folded:
                m["label"], m["track"] = folded[id(m["track"])]

        # Track collision corridor threats (outdoor mode)
        corridor_threats = []

        for m in measurements:
            track = m["track"]
            label, raw_label, conf, is_dynamic = m["label"], m["raw_label"], m["conf"], m["is_dynamic"]
            x1, y1, x2, y2 = m["box"]
            depth = m["depth"]
            final_label = f"{label.replace(' ', '_')}_{track.id}"

            # Directional relative position for blind assistance
            lat_offset = m["by"]  # +Y is Left, -Y is Right
            bearing = math.degrees(math.atan2(m["by"], m["bx"]))
            if bearing > CENTER_BEARING_DEG:
                rel_pos_text = f"LEFT {abs(lat_offset):.1f}m"
            elif bearing < -CENTER_BEARING_DEG:
                rel_pos_text = f"RIGHT {abs(lat_offset):.1f}m"
            else:
                rel_pos_text = "CENTER"

            motion_text = f"MOVING {track.velocity:.1f}m/s" if track.is_moving else "STATIONARY"

            # ── OUTDOOR: Collision corridor check ──
            if self._mode == "outdoor":
                half_corridor = self._collision_corridor_w / 2.0
                if abs(m["by"]) < half_corridor and 0 < m["bx"] < self._danger_distance * 1.5:
                    corridor_threats.append((label, depth, rel_pos_text))

            # ── STORE IN PERSISTENT HUD TRACKS (ZERO BLINKING) ──
            with self._hud_lock:
                prev_hud = self._hud_tracks.get(final_label)
                if prev_hud is not None:
                    # Moving objects: the box is drawn on the frame it was detected in, so smoothing would
                    # only make it trail the person; their distance is smoothed lightly
                    # Static objects: small detector jitter is smoothed strongly, a real move of the box (the
                    # wearer turning) is followed at once — adaptive, like a one-euro filter
                    def smooth(old, new, scale, floor):
                        a = 1.0 if is_dynamic else min(1.0, floor + abs(new - old) / scale)
                        return (1 - a) * old + a * new
                    x1_s, y1_s, x2_s, y2_s = (int(round(smooth(prev_hud[k], v, HUD_BOX_SCALE_PX, HUD_BOX_ALPHA)))
                                              for k, v in (('x1', x1), ('y1', y1), ('x2', x2), ('y2', y2)))
                    depth_s = ((1 - 0.6) * prev_hud['depth'] + 0.6 * depth) if is_dynamic else \
                        smooth(prev_hud['depth'], depth, HUD_DEPTH_SCALE_M, HUD_DEPTH_ALPHA)
                else:
                    x1_s, y1_s, x2_s, y2_s, depth_s = x1, y1, x2, y2, depth

                # Only confirmed objects are drawn: a flickering misdetection never reaches the screen
                if track.confirmed:
                    self._hud_tracks[final_label] = {
                        'x1': x1_s, 'y1': y1_s, 'x2': x2_s, 'y2': y2_s,
                        'label': final_label, 'conf': conf, 'is_dynamic': is_dynamic,
                        'depth': depth_s, 'cx': 0.5 * (x1 + x2), 'img_w': w,
                        'kw': track.width, 'kh': track.height, 'kz': track.z,
                        'is_moving': track.is_moving, 'vel': track.velocity,
                        'rel_pos': rel_pos_text, 'last_seen': now, 'source': m["source"],
                    }

            # ── 5. STRUCTURED ASSISTIVE HAZARD COMMUNICATION ──
            danger_d = self._danger_distance * (2.0 if raw_label in DROP_HAZARDS or label in DROP_HAZARDS else 1.0)
            if track.confirmed and depth_s < danger_d * 2.0:
                severity = "DANGER" if depth_s < danger_d else "WARNING"
                hazard_msg = (f"[{severity}] {label} at {depth_s:.1f}m {rel_pos_text}, "
                              f"size {track.width:.1f}x{track.height:.1f}m, {motion_text}")
                self._hazard_pub.publish(String(data=hazard_msg))

            # ── 6. INDOOR: Register to persistent spatial memory (confirmed static objects only) ──

            if raw_label in self._hazard_classes:
                current_hazards[final_label] = (0.5 * (x1 + x2), 0.5 * (y1 + y2), (x2 - x1) * (y2 - y1), now)

        # ── OUTDOOR: Publish corridor collision summary ──
        if self._mode == "outdoor" and corridor_threats:
            closest = min(corridor_threats, key=lambda t: t[1])
            collision_msg = f"[COLLISION] {closest[0]} blocking path at {closest[1]:.1f}m {closest[2]} — STOP or TURN"
            self._hazard_pub.publish(String(data=collision_msg))

        self._hazard_history = current_hazards

        # Objects within arm's reach: expected to have dropped into the chest sensors' blind spot,
        # so negative evidence below must not remove them.
        near_user = (lambda t: t.reliable and math.hypot(t.x[0] - px0, t.x[1] - py0) < BLIND_SPOT_RADIUS) \
            if self._mode == "indoor" else (lambda t: False)

        # ── CLEAN-UP OF EVERY CLASS (not only those detected this frame) ──
        for tracks in self._dynamic_tracks.values():
            self._purge_tracks(tracks, True, now)
        for label, tracks in self._static_tracks.items():
            self._purge_tracks(tracks, False, now)
            # A remembered object and a live one of the same class on one spot (created while the distance
            # estimate or SLAM jumped) are one object: merged, keeping the better-established ID
            merged_into = self._merge_duplicate_tracks(tracks, label)
            self._deleted_uids.extend(t.uid for t in merged_into.pop("_removed", []))
            for m in measurements:
                m["track"] = merged_into.get(id(m["track"]), m["track"])

        # ── NEGATIVE EVIDENCE: mapped objects that are in plain view but not detected ──
        # Every frame the camera looks at an object's spot (nothing closer in the way) without detecting it
        # counts as a miss for its detection rate (MIN_DETECTION_RATE). A misdetection that is not yet
        # reliable disappears once it goes UNSEEN_DROP_S unseen in plain view. A reliable (remembered)
        # object is only removed on sustained free-space evidence (the depth map reads past its whole
        # spot): not being detected from a new angle while walking around must not erase the room's map.
        # Objects within BLIND_SPOT_RADIUS of the wearer are below the chest sensors' view and left alone.
        matched = {id(m["track"]) for m in measurements}
        all_static = [t for tracks in self._static_tracks.values() for t in tracks]
        view = {}
        if all_static:
            xyz = np.array([(t.x[0], t.x[1], t.z + 0.5 * t.height) for t in all_static])
            vis, us, vs, depths = self._in_view_batch(xyz, pose, K, h)
            vis_loose = self._in_view_batch(xyz, pose, K, h, DETECTION_VIEW_MARGIN, DETECTION_VIEW_RANGE)[0]
            view = {id(t): (bool(vis[i]), bool(vis_loose[i]), us[i], vs[i], float(depths[i]))
                    for i, t in enumerate(all_static)}
        fx_px = K[0]
        # People (and pets) in front of an object hide it from the detector even when its centre pixel is clear
        person_boxes = [m["box"] for m in measurements if m["is_dynamic"]]

        def hidden_by_person(u, v, half_w, half_h):
            spot = (u - half_w, v - half_h, u + half_w, v + half_h)
            area = max(1.0, 4 * half_w * half_h)
            return any(max(0, min(spot[2], b[2]) - max(spot[0], b[0])) * max(0, min(spot[3], b[3]) - max(spot[1], b[1]))
                       >= REMOVE_PERSON_OVERLAP * area for b in person_boxes)

        for label, tracks in self._static_tracks.items():
            survivors = []
            for t in tracks:
                if id(t) not in matched and not near_user(t) and not self._protected(t):
                    in_view, in_view_loose, u, v, cam_depth = view[id(t)]
                    half_px = 0.5 * fx_px * t.width / max(cam_depth, 0.1)
                    clear = (not self._occluded(u, v, cam_depth, dmap, w, h, proj, half_px)
                             and not hidden_by_person(u, v, half_px, 0.5 * fx_px * t.height / max(cam_depth, 0.1)))
                    # In frame: its centre, or (a switch at the image edge, a chair low in the view) most of its box
                    frac = self._visible_fraction(t, pose, K, w, h) if (in_view_loose or t.reliable) else 0.0
                    visible = clear and (in_view_loose or frac >= REMOVE_VISIBLE_FRACTION)
                    if visible:
                        t.misses += 1
                    known_view = (t.reliable and visible and frac >= REMOVE_VISIBLE_FRACTION
                                  and cam_depth <= REMOVE_MAX_RANGE and self._seen_from_here(t, cam_xy_map))
                    if (in_view or known_view) and visible:
                        t.unseen_in_view += frame_dt
                        if known_view:
                            # Looking at its spot from a direction it was seen from: it has been taken away
                            empty = label not in SEE_THROUGH and self._spot_empty(u, v, cam_depth, t, K, dmap)
                            t.free_in_view = t.free_in_view + frame_dt if empty else 0.0
                            patience = max(REMOVE_UNSEEN_S, REMOVE_GAP_FACTOR * t.max_gap)
                            gone = t.free_in_view > REMOVE_FREE_S or t.unseen_in_view > patience
                            why = "its spot is empty" if t.free_in_view > REMOVE_FREE_S else \
                                f"not seen there for {t.unseen_in_view:.1f} s"
                        else:
                            t.free_in_view = (t.free_in_view + frame_dt
                                              if label not in SEE_THROUGH
                                              and self._free_space(u, v, cam_depth, t, K, dmap, proj) else 0.0)
                            if t.reliable:
                                gone = t.free_in_view > FREE_SPACE_CLEAR_RELIABLE_S
                            else:
                                gone = t.unseen_in_view > UNSEEN_DROP_S or t.free_in_view > FREE_SPACE_CLEAR_S
                            why = f"depth reads >{FREE_SPACE_MARGIN:.1f} m past its spot"
                        if gone:
                            if t.reliable:
                                self.get_logger().info(f"🧹 Removed {label}_{t.id} from the map: {why} (taken away?)")
                            self._deleted_uids.append(t.uid)
                            continue
                    else:
                        t.unseen_in_view = 0.0
                        t.free_in_view = 0.0
                survivors.append(t)
            tracks[:] = survivors

        # ── SLAM / DEPTH JUMP: a remembered object not seen where it was, with a newer live object of its class
        # right next to it, is that object. It keeps its old ID and takes the live position. All remembered,
        # unseen objects of a class are matched to the newer live ones at once (Hungarian): a jump moves every
        # object together, so nearest-first pairing handed a door's new sighting to the doorway next to it.
        for label, tracks in self._static_tracks.items():
            old = [t for t in tracks if t.is_reliable and not t.live(now)]
            if not old:
                continue
            new = [l for l in tracks if l.live(now) and l.first_seen > min(t.first_seen for t in old)]
            if not new:
                continue
            cost = np.full((len(old), len(new)), BIG_COST)
            for i, t in enumerate(old):
                for j, l in enumerate(new):
                    d2 = (l.x[0] - t.x[0]) ** 2 + (l.x[1] - t.x[1]) ** 2
                    if (l.first_seen > t.first_seen and d2 <= self._static_association_distance ** 2
                            and d2 <= CHI2_GATE_2D * (REVISIT_VAR + _track_var(l))):
                        cost[i, j] = d2
            if linear_sum_assignment is not None:
                pairs = zip(*linear_sum_assignment(cost))
            else:
                pairs, used = [], set()
                for i in np.argsort(cost.min(axis=1)):
                    j = int(np.argmin(np.where([k in used for k in range(len(new))], BIG_COST, cost[i])))
                    pairs.append((i, j))
                    used.add(j)
            for i, j in pairs:
                t, l = old[i], new[j]
                # Only once the camera has looked at the old spot long enough without seeing it there
                if cost[i, j] >= BIG_COST or t.unseen_in_view <= DRIFT_MERGE_S:
                    continue
                shift = math.hypot(l.x[0] - t.x[0], l.x[1] - t.x[1])
                self._fold_track(t, l)
                t.unseen_in_view = t.free_in_view = 0.0
                tracks.remove(l)
                self._deleted_uids.append(l.uid)
                self.get_logger().info(f"🔗 {label}_{t.id} re-found {shift:.2f} m from where it was remembered "
                                       f"(SLAM/depth shift); keeping its ID")

        # ── MEMORY CAP: evict the oldest-seen remembered objects once over the limit ──
        remembered = sorted((t for tracks in self._static_tracks.values() for t in tracks
                             if not t.live(now) and t.reliable), key=lambda t: t.last_seen)
        if len(remembered) > MAX_REMEMBERED_OBJECTS:
            evict = set(id(t) for t in remembered[:len(remembered) - MAX_REMEMBERED_OBJECTS])
            self._deleted_uids.extend(t.uid for t in remembered if id(t) in evict)
            for tracks in self._static_tracks.values():
                tracks[:] = [t for t in tracks if id(t) not in evict]

        if self._mode == "indoor":
            self._infer_tables(now)
        # The camera window shows this frame with these boxes, so they line up even on a moving person
        self._hud_frame = (frame, now)

    # ══════════════════════════════════════════════════════════════════════
    # ── OUTDOOR: live hazards (outdoor_awareness.py) ──
    # ══════════════════════════════════════════════════════════════════════
    def _scan_xy_base(self, scan):
        """All LiDAR returns (360°) in base_footprint (N,2), the wearer's body excluded, or None."""
        if scan is None or self._lidar_R is None or scan.header.frame_id != self._lidar_frame:
            return None
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges >= max(scan.range_min, LIDAR_BODY_RANGE)) & (ranges <= scan.range_max)
        r, a = ranges[ok], angles[ok]
        pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1) @ self._lidar_R.T + self._lidar_t
        return pts[:, :2]

    def _project_base(self, pts, K):
        """Pixels (u, v) of base_footprint points (N,3); v is NaN behind the camera."""
        fx, fy, cx, cy, _ = K
        pc = (np.asarray(pts, dtype=np.float64) - self._cam_t) @ self._cam_R
        z = np.where(pc[:, 2] > 0.05, pc[:, 2], np.nan)
        return fx * pc[:, 0] / z + cx, fy * pc[:, 1] / z + cy

    def _horizon_row(self, K, h: int) -> int:
        """Image row of the horizon (a point far ahead at camera height), clamped to the frame."""
        u, v = self._project_base(np.array([[100.0, 0.0, self._cam_t[2]]]), K)
        return int(max(0, min(h - 1, v[0]))) if np.isfinite(v[0]) else h // 2

    def _ground_point(self, u: float, v: float, K, ground):
        """base_footprint (x, y) where pixel (u, v) meets the ground (the fitted plane, else z = 0), or None."""
        ray = self._pixel_ray(u, v, K)
        if ground is not None and ground.fitted:
            n, d = ground.plane
            denom = float(n @ ray)
            if abs(denom) < 1e-6:
                return None
            s_ = -(float(n @ self._cam_t) + d) / denom
        else:
            s_ = -self._cam_t[2] / ray[2] if ray[2] < -1e-6 else -1.0
        if s_ <= 0:
            return None
        p = self._cam_t + s_ * ray
        return float(p[0]), float(p[1])

    def _crossing_measurement(self, box, K, ground):
        """A zebra crossing found by its stripes, as a measurement at its nearest edge (bottom of the box)."""
        x1, y1, x2, y2 = box
        near = self._ground_point(0.5 * (x1 + x2), y2, K, ground)
        if near is None or not (0.3 < near[0] < 30.0):
            return None
        # The box must lie on the ground: the stripes' own depth points are at ground height (windows and
        # house siding also form rows of bars)
        if ground is not None and ground.fitted and ground.points is not None:
            us, vs, hgt = ground.points[:3]
            sel = (us >= x1) & (us <= x2) & (vs >= y1) & (vs <= y2)
            if sel.sum() >= 5 and float(np.median(np.abs(hgt[sel]))) > 0.15:
                return None
        far = self._ground_point(0.5 * (x1 + x2), y1, K, ground)
        dist = math.hypot(*near)
        return {"box": tuple(int(v) for v in box), "conf": 0.6, "raw_label": "zebra crossing",
                "label": "zebra crossing", "mask": None, "is_dynamic": False, "bx": near[0], "by": near[1],
                "depth": dist, "sigma": 0.1 + 0.05 * dist, "source": "stripes",
                "z": 0.0, "z_top": 0.0, "width_m": 3.0, "height_m": 0.02,
                "length_m": (far[0] - near[0]) if far is not None else None}

    def _outdoor_scan(self, msg, t_rx):
        """Every LiDAR scan in outdoor mode: the wearer's motion, the occupancy grid, and the objects around
        (360°). Times are when the scan arrived (monotonic), the same clock as the camera frames."""
        self._refresh_extrinsics(t_rx)
        xy = self._scan_xy_base(msg)
        if xy is None:
            return
        ego = None
        if self._odom_enabled:
            t_scan = _stamp_to_sec(msg.header.stamp)
            pose, vel, ok = self._odo.update(xy, t_scan, self._gyro)
            # Healthy after a run of matched scans; lost after 2 s of failures (then everything falls back to
            # the body frame until it recovers)
            if ok:
                self._odom_good_run += 1
                self._odom_bad_since = None
                if self._odom_good_run >= 5 and not self._odom_good:
                    self._odom_good = True
                    self.get_logger().info("🧭 LiDAR odometry locked: objects are tracked world-fixed")
            else:
                self._odom_good_run = 0
                self._odom_bad_since = self._odom_bad_since or t_rx
                if self._odom_good and t_rx - self._odom_bad_since > 2.0:
                    self._odom_good = False
                    self.get_logger().warn("🧭 LiDAR odometry lost (nothing to match): tracking relative to you")
            self._ego_hist.append((t_rx, pose, vel))
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = msg.header.stamp, "odom", BASE_FRAME
            o.pose.pose.position.x, o.pose.pose.position.y = float(pose[0]), float(pose[1])
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
            c, s_ = math.cos(pose[2]), math.sin(pose[2])
            o.twist.twist.linear.x, o.twist.twist.linear.y = float(c * vel[0] + s_ * vel[1]), float(-s_ * vel[0] + c * vel[1])
            o.twist.twist.angular.z = float(vel[2])
            var = 0.0025 if ok else 0.25
            o.pose.covariance[0] = o.pose.covariance[7] = var
            o.pose.covariance[35] = 0.1 * var
            self._odom_pub.publish(o)
            if self._odom_good:
                ego = pose
        self._occ.add_scan(ego, xy, t_rx, sensor_xy=self._lidar_t[:2])
        self._outdoor_tracker.update(oa.lidar_clusters(xy), t_rx, ego)

    def _ego_at(self, t):
        """(pose, velocity) of the wearer (odom) at monotonic time t, interpolated between scans; None when the
        odometry is not healthy or has nothing near that time."""
        if not self._odom_good or not self._ego_hist:
            return None, None
        hist = list(self._ego_hist)
        if t >= hist[-1][0]:
            return (hist[-1][1], hist[-1][2]) if t - hist[-1][0] < 0.3 else (None, None)
        for (t0, p0, v0), (t1, p1, v1) in zip(hist[:-1], hist[1:]):
            if t0 <= t <= t1:
                a = (t - t0) / max(t1 - t0, 1e-6)
                dyaw = (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi
                pose = np.array([p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), p0[2] + a * dyaw])
                return pose, (1 - a) * v0 + a * v1
        return None, None

    def _process_outdoor(self, stamp, dets, frame, now, dmap, proj, K, t_meas):
        """One camera frame in outdoor mode: measure, track (body frame), assess, speak, draw."""
        h, w = frame.shape[:2]
        meas = []
        for det in dets:
            m = self._measure_detection(det, K, h, proj, dmap)
            if m["label"] in oa.SIGNALS:
                m["signal"] = oa.signal_color(frame, m["box"], m.get("mask"))
            meas.append(m)
        dscale = self._depth_scale if self._depth_scale_valid else 1.0
        ground = oa.analyze_ground(dmap, K, self._cam_R, self._cam_t, dscale,
                                   exclude_boxes=[m["box"] for m in meas if m["label"] in oa.MOVERS]) \
            if dmap is not None else None
        # Zebra crossings: the detector rarely finds them; the white-stripe pattern does
        horizon = self._horizon_row(K, h)
        zebra = oa.find_zebra_crossing(frame, horizon)
        if zebra is not None and not any(m["label"] in oa.CROSSINGS and _box_overlap(m["box"], zebra) > 0.3
                                         for m in meas):
            zm = self._crossing_measurement(zebra, K, ground)
            if zm is not None:
                meas.append(zm)
            else:
                zebra = None
        lidar = None
        xy = self._scan_xy_base(self._scan_for_stamp(stamp))
        if xy is not None:
            lidar = oa.lidar_corridor(xy)

        ego, _ = self._ego_at(t_meas)
        self._outdoor_tracker.update(meas, t_meas, ego)
        ego_now, vel_now = self._ego_at(now)
        if ego_now is None and self._odom_good and self._ego_hist:
            ego_now, vel_now = self._ego_hist[-1][1], self._ego_hist[-1][2]
        self._outdoor_tracker.set_view(ego_now, vel_now)
        tracks = self._outdoor_tracker.confirmed()
        # Depth's low obstacles and drops ahead go into the occupancy grid too
        if ground is not None and ground.ok and ground.points is not None:
            _, _, hgt, gx, gy = ground.points
            near = (gx > 0.3) & (gx < oa.LOOK_AHEAD) & (np.abs(gy) < 4.0)
            gxy = np.stack([gx, gy], 1)
            self._occ.add_ground(ego, gxy[near & (hgt > oa.OBSTACLE_MIN_H) & (hgt < oa.HEAD_LOW)],
                                 gxy[near & (hgt < -oa.DROP_MIN_H) & (gx < oa.DROP_MAX_RANGE)], t_meas)
        alerts, lanes = oa.assess(tracks, lidar, ground)
        self._publish_outdoor_markers(tracks, alerts, lanes, lidar, ground, now, vel_now)
        if now - self._last_occ_pub >= 0.2:
            self._last_occ_pub = now
            self._publish_occupancy(ego_now)
        alert = self._alert_policy.pick(alerts)
        if alert is not None:
            payload = alert.as_dict()
            self._outdoor_alert_pub.publish(String(data=json.dumps(payload)))
            tag = {oa.CRITICAL: "DANGER", oa.WARNING: "WARNING"}.get(alert.level, "INFO")
            self._hazard_pub.publish(String(data=f"[{tag}] {alert.text}"))
            self._last_alert = (alert.text, alert.level, now)
            self.get_logger().info(f"🔊 [{tag}] {alert.text}")
        if now - self._last_scene_pub >= 0.5:
            self._last_scene_pub = now
            sig = oa.signal_summary(tracks)
            scene = {
                "frame": BASE_FRAME,
                "objects": [{"class": t.label, "id": t.id, "x": round(float(t.x[0]), 2), "y": round(float(t.x[1]), 2),
                             "dist": round(t.dist, 2), "closing": round(t.closing_speed, 2),
                             "ttc": None if math.isinf(t.ttc) else round(t.ttc, 1), "in_path": bool(t.in_path()),
                             "color": t.color, "source": t.m.get("source"),
                             "speed": None if t.world_speed is None else round(t.world_speed, 2)}
                            for t in tracks if t.label is not None],
                "path": {k: round(v, 2) for k, v in lanes.items()},
                "lidar": lidar is not None, "ground": bool(ground is not None and ground.ok),
                "odometry": self._odom_good,
                "walking_speed": None if vel_now is None else round(float(np.hypot(vel_now[0], vel_now[1])), 2),
                "signal": None if sig is None else {"kind": sig[0], "color": sig[1], "text": sig[2]},
                "alerts": [a.as_dict() for a in alerts[:5]],
                "summary": oa.scene_summary(tracks, lanes, ground),
            }
            self._outdoor_scene_pub.publish(String(data=json.dumps(scene)))

        # ── HUD ──
        hud = {}
        for t in tracks:
            m = t.m
            if t.label is None or m.get("box") is None or t_meas - t.last_cam > 0.3:
                continue  # beside / behind: LiDAR only, not in the camera picture
            x1, y1, x2, y2 = m["box"]
            name = f"{t.label.replace(' ', '_')}_{t.id}"
            detail = f"{t.dist:.1f}m ({ {'lidar': 'LiDAR', 'depth': 'depth', 'stripes': 'stripes'}.get(m.get('source'), 'cam')}) | {oa.where(*t.x[:2])}"
            if t.color:
                detail += f" | {t.color.upper()}"
            if t.world_speed is not None and t.label in oa.MOVERS and t.world_speed > 0.5:
                detail += f" | moving {t.world_speed:.1f}m/s"
            if t.label in oa.MOVERS and abs(t.closing_speed) > 0.5:
                detail += f" | {'closing' if t.closing_speed > 0 else 'leaving'} {abs(t.closing_speed):.1f}m/s"
                if not math.isinf(t.ttc):
                    detail += f" TTC {t.ttc:.1f}s"
            if t.label in oa.OVERHEAD or (t.label not in oa.OUTDOOR_GROUND and m.get("z", 0.0) > 0.3):
                detail += f" | low edge {m.get('z', 0.0):.2f}m"
            hud[name] = {
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'label': name, 'conf': m["conf"],
                'is_dynamic': t.label in oa.MOVERS, 'depth': t.dist, 'cx': 0.5 * (x1 + x2), 'img_w': w,
                'kw': m.get("width_m", 0.5), 'kh': m.get("height_m", 0.5), 'kz': m.get("z", 0.0),
                'is_moving': t.closing_speed > 1.0, 'vel': t.closing_speed, 'rel_pos': oa.where(*t.x[:2]),
                'last_seen': now, 'source': m.get("source"), 'detail': detail,
            }
        with self._hud_lock:
            self._hud_tracks = hud
        self._outdoor_hud = {"K": K, "lanes": lanes, "lidar": lidar, "zebra": zebra, "horizon": horizon,
                             "ground": ground, "alerts": alerts[:3], "time": now, "odom": self._odom_good,
                             "speed": None if vel_now is None else float(np.hypot(vel_now[0], vel_now[1]))}
        self._record(frame, alert, now)

    def _publish_outdoor_markers(self, tracks, alerts, lanes, lidar, ground, now, ego_vel=None):
        """Live outdoor view for RViz, drawn like a self-driving car's display: base_footprint (the wearer at the
        centre, facing +x), a model per object class at its place and heading, predicted paths of moving things,
        and the wearer's own walking path up to where it is blocked.

        Each array starts with DELETEALL, so RViz shows exactly the present: an object is drawn only while a
        sensor sees it (the camera ahead, the LiDAR all around, OUTDOOR_SHOW_S), and nothing stays."""
        if self._outdoor_marker_pub.get_subscription_count() == 0:
            return
        stamp = self.get_clock().now().to_msg()
        life = Duration(seconds=OUTDOOR_MARKER_LIFETIME).to_msg()  # vanish if frames stop coming
        markers = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.append(clear)
        ids = {}

        def mk(ns, mtype, rgba, scale=(1.0, 1.0, 1.0), pos=(0.0, 0.0, 0.0), yaw=0.0):
            m = Marker()
            m.header.frame_id, m.header.stamp, m.lifetime = BASE_FRAME, stamp, life
            ids[ns] = ids.get(ns, -1) + 1
            m.ns, m.id, m.type, m.action = ns, ids[ns], mtype, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
            markers.append(m)
            return m

        def local(x, y, yaw, dx, dy):
            c, s_ = math.cos(yaw), math.sin(yaw)
            return x + c * dx - s_ * dy, y + s_ * dx + c * dy

        # ── The wearer: a person model and a heading chevron ──
        me = (0.35, 0.65, 1.0, 1.0)
        mk("wearer", Marker.CYLINDER, me, (0.42, 0.42, oa.USER_HEIGHT - 0.3), (0, 0, (oa.USER_HEIGHT - 0.3) / 2))
        mk("wearer", Marker.SPHERE, me, (0.26, 0.26, 0.28), (0, 0, oa.USER_HEIGHT - 0.12))
        arrow = mk("wearer", Marker.ARROW, me, (0.07, 0.18, 0.18))
        arrow.points = [Point(x=0.35, y=0.0, z=0.03), Point(x=1.0, y=0.0, z=0.03)]
        if ego_vel is not None and self._odom_good:
            spd = float(np.hypot(ego_vel[0], ego_vel[1]))
            txt = mk("wearer", Marker.TEXT_VIEW_FACING, (0.8, 0.9, 1.0, 1.0), (0, 0, 0.3), (0, 0, oa.USER_HEIGHT + 0.35))
            txt.text = f"you {spd:.1f} m/s"

        # ── Walking path (blue while clear) up to the nearest blockage, and the lanes either side ──
        block = min(lanes.get("center", oa.LOOK_AHEAD), oa.LOOK_AHEAD)
        col = (1.0, 0.15, 0.1) if block < oa.OBSTACLE_CRITICAL_M else (1.0, 0.55, 0.0) if block < oa.OBSTACLE_WARNING_M \
            else (0.2, 0.55, 1.0)
        start = 0.35
        if block > start:
            mk("path", Marker.CUBE, (*col, 0.35), (block - start, 2 * oa.CORRIDOR_HALF, 0.01), ((start + block) / 2, 0, 0.005))
            for edge in (-oa.CORRIDOR_HALF, oa.CORRIDOR_HALF):
                mk("path", Marker.CUBE, (*col, 0.8), (block - start, 0.03, 0.02), ((start + block) / 2, edge, 0.01))
        if block < oa.LOOK_AHEAD:
            mk("path", Marker.CUBE, (*col, 0.9), (0.05, 2 * oa.CORRIDOR_HALF + 0.2, 0.25), (block, 0.0, 0.125))
            t_ = mk("path", Marker.TEXT_VIEW_FACING, (*col, 1.0), (0, 0, 0.35), (block, 0.0, 0.6))
            t_.text = f"{block:.1f} m"
        for name, y0 in (("left", oa.CORRIDOR_HALF + oa.LANE_WIDTH / 2), ("right", -oa.CORRIDOR_HALF - oa.LANE_WIDTH / 2)):
            d = min(lanes.get(name, oa.LOOK_AHEAD), oa.LOOK_AHEAD)
            if d > start:
                lc = (0.2, 0.55, 1.0) if d >= oa.LOOK_AHEAD else (1.0, 0.55, 0.0)
                mk("lanes", Marker.CUBE, (*lc, 0.08), (d - start, oa.LANE_WIDTH, 0.01), ((start + d) / 2, y0, 0.004))

        # ── Objects: a model per class, where and how it stands, only while seen ──
        level = {}
        for a in alerts:
            for pre in ("obj", "veh"):
                if a.key.startswith(pre) and a.key[len(pre):].isdigit():
                    level[int(a.key[len(pre):])] = max(level.get(int(a.key[len(pre):]), 0), a.level)
        ego_yaw = self._ego_hist[-1][1][2] if (self._odom_good and self._ego_hist) else None
        for t in tracks:
            if t.label is None or now - t.last_seen > OUTDOOR_SHOW_S or t.label in OUTDOOR_STRUCTURE:
                continue  # unnamed outlines and walls / fences are in the occupancy view; out of sight = gone
            x, y = float(t.x[0]), float(t.x[1])
            lbl, yaw = t.label, t.yaw_body
            lvl = level.get(t.id, 0)
            base = (1.0, 0.15, 0.1) if lvl >= oa.CRITICAL else (1.0, 0.55, 0.0) if lvl == oa.WARNING else None
            grey = base or (0.82, 0.84, 0.88)
            white = base or (0.96, 0.96, 0.96)
            top = 1.0
            if lbl in ("car", "van", "three-wheeler", "tractor"):
                L, W, H = {"car": (4.4, 1.8, 1.45), "van": (4.8, 1.9, 1.9), "three-wheeler": (2.6, 1.3, 1.7),
                           "tractor": (3.5, 2.0, 2.5)}[lbl]
                mk("objects", Marker.CUBE, (*grey, 0.9), (L, W, 0.55 * H), (x, y, 0.2 + 0.275 * H), yaw)
                cx, cy = local(x, y, yaw, -0.08 * L, 0.0)
                mk("objects", Marker.CUBE, (*grey, 0.75), (0.55 * L, 0.92 * W, 0.4 * H), (cx, cy, 0.2 + 0.75 * H), yaw)
                top = H + 0.2
            elif lbl in ("bus", "truck", "train"):
                L, W, H = {"bus": (11.0, 2.5, 3.2), "truck": (7.5, 2.4, 3.2), "train": (20.0, 3.0, 3.8)}[lbl]
                mk("objects", Marker.CUBE, (*grey, 0.85), (L, W, H - 0.3), (x, y, 0.3 + (H - 0.3) / 2), yaw)
                top = H
            elif lbl in ("motorcycle", "bicycle"):
                mk("objects", Marker.CUBE, (*grey, 0.9), (1.8 if lbl == "motorcycle" else 1.7, 0.35, 0.7), (x, y, 0.4), yaw)
                if t.world_speed is not None and t.world_speed > 1.0:  # ridden
                    mk("objects", Marker.CYLINDER, (*white, 0.9), (0.4, 0.4, 0.9), (x, y, 1.2))
                    mk("objects", Marker.SPHERE, (*white, 0.9), (0.25, 0.25, 0.28), (x, y, 1.8))
                top = 1.9
            elif lbl in oa.PEOPLE:
                h = 1.15 if lbl == "child" else 1.7
                mk("objects", Marker.CYLINDER, (*white, 0.95), (0.45, 0.45, h - 0.25), (x, y, (h - 0.25) / 2))
                mk("objects", Marker.SPHERE, (*white, 0.95), (0.25, 0.25, 0.28), (x, y, h - 0.1))
                top = h
            elif lbl in oa.ANIMALS:
                L, H = {"cow": (2.0, 1.4), "goat": (1.0, 0.75), "dog": (0.9, 0.6), "cat": (0.5, 0.3)}[lbl]
                mk("objects", Marker.CUBE, (*grey, 0.9), (L, 0.35 * L + 0.1, 0.45 * H), (x, y, 0.55 * H + 0.1), yaw)
                hx, hy = local(x, y, yaw, 0.55 * L, 0.0)
                mk("objects", Marker.SPHERE, (*grey, 0.9), (0.3 * H + 0.1,) * 3, (hx, hy, 0.95 * H))
                top = H + 0.1
            elif lbl == "tree":
                mk("objects", Marker.CYLINDER, (0.55, 0.42, 0.3, 0.95) if base is None else (*base, 0.95),
                   (0.35, 0.35, 2.6), (x, y, 1.3))
                mk("objects", Marker.SPHERE, (0.35, 0.7, 0.4, 0.5) if base is None else (*base, 0.5), (3.0, 3.0, 2.4),
                   (x, y, 3.6))
                top = 4.8
            elif lbl == "pole":
                mk("objects", Marker.CYLINDER, (*grey, 0.95), (0.18, 0.18, 3.5), (x, y, 1.75))
                top = 3.5
            elif lbl == "traffic cone":
                mk("objects", Marker.CYLINDER, (1.0, 0.45, 0.05, 0.95), (0.35, 0.35, 0.12), (x, y, 0.06))
                mk("objects", Marker.CYLINDER, (1.0, 0.45, 0.05, 0.95), (0.22, 0.22, 0.6), (x, y, 0.35))
                top = 0.7
            elif lbl in oa.SIGNALS:
                lamp = {"red": (1.0, 0.1, 0.1), "yellow": (1.0, 0.85, 0.0), "green": (0.1, 1.0, 0.3)}.get(t.color, (0.5, 0.5, 0.5))
                mk("objects", Marker.CYLINDER, (0.3, 0.3, 0.3, 1.0), (0.12, 0.12, 2.6), (x, y, 1.3))
                mk("objects", Marker.CUBE, (0.1, 0.1, 0.1, 1.0), (0.25, 0.35, 0.8), (x, y, 2.9))
                mk("objects", Marker.SPHERE, (*lamp, 1.0), (0.3, 0.3, 0.3), (x, y, 3.05 if t.color == "red" else 2.75))
                top = 3.4
            elif lbl in oa.CROSSINGS:
                length = max(1.0, min(t.m.get("length_m") or 3.0, 6.0))
                for k in range(7):
                    sy = -1.8 + k * 0.6
                    px, py = local(x + length / 2, y, 0.0, 0.0, sy)
                    mk("objects", Marker.CUBE, (1.0, 1.0, 1.0, 0.85), (length, 0.35, 0.02), (px, py, 0.01))
                top = 0.1
            elif lbl in oa.DROPS:
                w_ = max(0.3, min(float(t.m.get("width_m", 0.6)), 3.0))
                mk("objects", Marker.CYLINDER, (0.25, 0.0, 0.3, 0.9), (w_, w_, 0.03), (x, y, 0.015))
                mk("objects", Marker.CYLINDER, (1.0, 0.0, 1.0, 0.7) if base is None else (*base, 0.8),
                   (w_ + 0.12, w_ + 0.12, 0.01), (x, y, 0.005))
                top = 0.2
            elif lbl in oa.OVERHEAD:
                z0 = float(t.m.get("z", 1.5))
                mk("objects", Marker.CYLINDER, (0.55, 0.42, 0.3, 0.95) if base is None else (*base, 0.95),
                   (0.12, 0.12, 1.6), (x, y, max(z0, 1.2)))
                top = max(z0, 1.2) + 0.3
                markers[-1].pose.orientation.x, markers[-1].pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
            else:
                w_ = max(0.2, min(float(t.m.get("width_m", 0.5)), 4.0))
                h_ = max(0.2, min(float(t.m.get("height_m") or 1.0), 3.0))
                z0 = float(t.m.get("z", 0.0)) if lbl not in oa.OUTDOOR_GROUND else 0.0
                mk("objects", Marker.CUBE, (*grey, 0.85), (w_, max(0.2, min(w_, 0.8)), h_), (x, y, z0 + h_ / 2), yaw)
                top = z0 + h_
            label = f"{oa.spoken_class(lbl)} {t.dist:.1f} m"
            if t.color:
                label += f" {t.color}"
            if t.world_speed is not None and t.world_speed > 0.5 and lbl in oa.MOVERS:
                label += f"  {t.world_speed:.1f} m/s"
            txt = mk("labels", Marker.TEXT_VIEW_FACING, (*(base or (1.0, 1.0, 1.0)), 1.0), (0, 0, 0.35), (x, y, top + 0.4))
            txt.text = label
            # ── Predicted path of a moving thing (its own motion, 3 s) ──
            if lbl in oa.MOVERS and t.world_speed is not None and ego_yaw is not None and t.world_speed > 0.5:
                c, s_ = math.cos(ego_yaw), math.sin(ego_yaw)
                vbx, vby = c * t.xw[2] + s_ * t.xw[3], -s_ * t.xw[2] + c * t.xw[3]
                line = mk("paths", Marker.LINE_STRIP, (*(base or (0.6, 0.85, 1.0)), 0.9), (0.08, 0, 0))
                line.points = [Point(x=x + vbx * k * 0.25, y=y + vby * k * 0.25, z=0.05) for k in range(13)]
            elif lbl in oa.MOVERS and ego_yaw is None and abs(t.closing_speed) > 0.8:
                line = mk("paths", Marker.LINE_STRIP, (*(base or (0.6, 0.85, 1.0)), 0.9), (0.08, 0, 0))
                line.points = [Point(x=x + t.x[2] * k * 0.25, y=y + t.x[3] * k * 0.25, z=0.05) for k in range(13)]
        self._outdoor_marker_pub.publish(MarkerArray(markers=markers))

    def _publish_occupancy(self, ego):
        """The live occupancy grid (/outdoor_occupancy, 5 Hz): grey columns where the LiDAR finds something
        (anything, named or not), orange low obstacles and magenta drops from depth, and faint walkable ground
        where the LiDAR sees through. The last few seconds only, fading; nothing is saved."""
        if self._occ_pub.get_subscription_count() == 0:
            return
        stamp = self.get_clock().now().to_msg()
        life = Duration(seconds=0.6).to_msg()
        cells = self._occ.cells(ego)
        markers = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.append(clear)
        style = {"occupied": ((0.62, 0.64, 0.7, 0.85), 1.3), "low": ((1.0, 0.55, 0.1, 0.9), 0.35),
                 "drop": ((0.9, 0.0, 0.9, 0.9), 0.03), "free": ((0.25, 0.4, 0.7, 0.18), 0.01)}
        for i, (name, ((r, g, b, a), hgt)) in enumerate(style.items()):
            pts = cells.get(name)
            if pts is None or len(pts) == 0:
                continue
            if name == "free":
                pts = pts[::3]  # a hint of the walkable area is enough, and keeps RViz fast
            m = Marker()
            m.header.frame_id, m.header.stamp, m.lifetime = BASE_FRAME, stamp, life
            m.ns, m.id, m.type, m.action = "occupancy", i, Marker.CUBE_LIST, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = oa.OCC_RES * (1.7 if name == "free" else 1.0)
            m.scale.z = hgt
            m.color = ColorRGBA(r=r, g=g, b=b, a=a)
            m.points = [Point(x=float(px), y=float(py), z=hgt / 2) for px, py in pts]
            markers.append(m)
        self._occ_pub.publish(MarkerArray(markers=markers))

    def _record(self, frame, alert, now):
        """WEARABLE_RECORD_DIR: save the annotated frame of every spoken alert, and one every RECORD_PERIOD_S."""
        if not self._record_dir or (alert is None and now - self._last_record < RECORD_PERIOD_S):
            return
        self._last_record = now
        stamp = time.strftime("%H%M%S") + f"_{int(1000 * (time.time() % 1)):03d}"
        img = frame.copy()
        try:
            self._draw_cached_boxes(img)
            cv2.imwrite(os.path.join(self._record_dir, f"{stamp}{'_alert' if alert else ''}.jpg"), img,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
            if alert is not None:
                with open(os.path.join(self._record_dir, "alerts.jsonl"), "a") as f:
                    f.write(json.dumps({"t": stamp, **alert.as_dict()}) + "\n")
        except Exception as e:
            self._warn_once("record", f"Recording failed: {e}")

    def _draw_outdoor(self, frame):
        """Outdoor HUD layer: the walking corridor on the ground (green clear / orange / red blocked, with the
        blocking distance), the lanes either side, a crossing found by its stripes, and the current alerts."""
        hud = self._outdoor_hud
        if hud is None:
            return
        h, w = frame.shape[:2]
        K, lanes = hud["K"], hud["lanes"]
        if K[4] != w:
            return
        ground = hud["ground"]
        # Ground heights: obstacles red, drops magenta, head-height orange (sampled pixels in the corridor)
        if ground is not None and ground.ok and ground.points is not None:
            us, vs, hgt, xs, ys = ground.points
            half = oa.CORRIDOR_HALF
            inpath = (np.abs(ys) <= half) & (xs < oa.LOOK_AHEAD)
            for sel, col in (((hgt > oa.OBSTACLE_MIN_H) & (hgt < oa.HEAD_LOW) & inpath, (0, 0, 255)),
                             ((hgt < -oa.DROP_MIN_H) & inpath & (xs < oa.DROP_MAX_RANGE), (255, 0, 255)),
                             ((hgt >= oa.HEAD_LOW) & (hgt <= oa.HEAD_HIGH) & inpath, (0, 165, 255))):
                for u, v in zip(us[sel], vs[sel]):
                    cv2.circle(frame, (int(u), int(v)), 2, col, -1)
        # Corridor on the ground, up to where it is blocked
        block = lanes.get("center", oa.LOOK_AHEAD)
        color = (0, 0, 255) if block < oa.OBSTACLE_CRITICAL_M else (0, 140, 255) if block < oa.OBSTACLE_WARNING_M \
            else (0, 220, 120)
        far = min(block, oa.LOOK_AHEAD)
        for lo, hi, col, alpha in ((-oa.CORRIDOR_HALF, oa.CORRIDOR_HALF, color, 0.18),):
            xs = np.linspace(0.8, far, 12)
            left = np.stack([xs, np.full_like(xs, hi), np.zeros_like(xs)], axis=1)
            right = np.stack([xs[::-1], np.full_like(xs, lo), np.zeros_like(xs)], axis=1)
            u, v = self._project_base(np.vstack([left, right]), K)
            ok = np.isfinite(u) & np.isfinite(v)
            if ok.sum() >= 3:
                poly = np.stack([u[ok], v[ok]], axis=1).astype(np.int32)
                poly[:, 1] = np.clip(poly[:, 1], hud["horizon"], h - 1)
                overlay = frame.copy()
                cv2.fillPoly(overlay, [poly], col)
                cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
                cv2.polylines(frame, [poly], True, col, 1, cv2.LINE_AA)
        if block < oa.LOOK_AHEAD:
            u, v = self._project_base(np.array([[block, oa.CORRIDOR_HALF, 0.0], [block, -oa.CORRIDOR_HALF, 0.0]]), K)
            if np.all(np.isfinite(u)) and np.all(np.isfinite(v)):
                p1, p2 = (int(u[0]), int(min(v[0], h - 2))), (int(u[1]), int(min(v[1], h - 2)))
                cv2.line(frame, p1, p2, color, 3, cv2.LINE_AA)
                cv2.putText(frame, f"{block:.1f}m", (p2[0] + 4, p2[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                            cv2.LINE_AA)
        if hud["zebra"] is not None:
            x1, y1, x2, y2 = hud["zebra"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(frame, "ZEBRA", (x1 + 3, y2 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                        cv2.LINE_AA)
        # The alerts right now (most urgent first), above the mode bar
        y = h - 30
        for a in reversed(hud["alerts"]):
            col = (0, 0, 255) if a.level == oa.CRITICAL else (0, 165, 255) if a.level == oa.WARNING else (255, 255, 0)
            (tw, th), _ = cv2.getTextSize(a.text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (6, y - th - 6), (14 + tw, y + 4), (20, 20, 20), cv2.FILLED)
            cv2.putText(frame, a.text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            y -= th + 12

    def _infer_tables(self, now: float):
        """SMART TABLE INFERENCE: desktop objects floating at desk height imply a table YOLO missed."""
        real_tables = [t for lbl in ("table", "dining table") for t in self._static_tracks.get(lbl, [])
                       if t.live(now)]
        desk_items = [t for lbl, tracks in self._static_tracks.items() if lbl in DESKTOP_OBJECTS
                      for t in tracks if t.live(now) and t.z > 0.4]
        clusters = []
        for t in desk_items:
            for c in clusters:
                if any(math.hypot(t.x[0] - o.x[0], t.x[1] - o.x[1]) < 1.2 for o in c):
                    c.append(t)
                    break
            else:
                clusters.append([t])

        inferred = []
        for c in clusters:
            xs = [t.x[0] for t in c]
            ys = [t.x[1] for t in c]
            cx, cy = float(np.mean(xs)), float(np.mean(ys))
            if any(math.hypot(cx - t.x[0], cy - t.x[1]) < 1.5 for t in real_tables):
                continue
            span = max(max(xs) - min(xs), max(ys) - min(ys))
            inferred.append(_InferredTable(len(inferred) + 1, cx, cy, float(np.median([t.z for t in c])),
                                           max(1.0, span + 0.4), float(np.mean([t.dist for t in c]))))
        self._inferred_tables = inferred

    # ══════════════════════════════════════════════════════════════════════
    # ── RVIZ MARKERS ──
    # ══════════════════════════════════════════════════════════════════════
    def _publish_markers(self, now):
        current_markers = []
        labels = []  # (text marker, anchor on top of its object, colour) — leaders drawn after decluttering
        now_msg = self.get_clock().now().to_msg()
        lifetime = Duration(seconds=MARKER_LIFETIME).to_msg()

        marker_frame = 'map' if self._mode == 'indoor' else 'base_footprint'

        # Distances on the labels are from where the wearer is *now*, not where they stood
        # when the object was last seen.
        user_xy = (0.0, 0.0)
        if self._mode == 'indoor':
            pose = self._lookup_base_pose(None)
            if pose is not None:
                user_xy = (pose[0], pose[1])

        def away(x, y):
            return math.hypot(x - user_xy[0], y - user_xy[1])

        # Objects removed since the last publish: delete every marker they own, right away
        for uid in self._deleted_uids:
            for ns in ("labels", "shapes", "phantom_desks", "phantom_legs", "table_legs", "labels_leaders"):
                for prefix in ("yolo_static", "yolo_dynamic"):
                    for k in (range(4, 8) if ns == "table_legs" else (0, 1, 2, 3)):
                        gone = Marker()
                        gone.header.frame_id = marker_frame
                        gone.ns = f"{prefix}_{ns}"
                        gone.id = uid * 10 + k
                        gone.action = Marker.DELETE
                        current_markers.append(gone)
        self._deleted_uids = []

        # Moving objects: only while being detected. Static objects (indoor): everything reliable
        # stays on the global map; objects not in view right now are drawn translucent.
        for tracks_dict, is_dynamic in ((self._dynamic_tracks, True), (self._static_tracks, False)):
            for label, tracks in tracks_dict.items():
                for track in tracks:
                    is_live = track.live(now)
                    if not self._shown(track, is_dynamic, now):
                        continue
                    final_label = f"{label.replace(' ', '_')}_{track.id}"
                    pos = track.position_at(now)
                    self._add_track_marker(
                        current_markers, labels, track, final_label, now_msg, lifetime, is_dynamic, marker_frame,
                        distance=away(*pos), seen_ago=None if is_live else now - track.last_seen, pos=pos,
                    )

        for table in self._inferred_tables:
            self._add_track_marker(
                current_markers, labels, table, f"table_inferred{table.id}", now_msg, lifetime, False,
                marker_frame, distance=away(table.x[0], table.x[1]), base_label="table"
            )

        if now - self._last_objects_pub >= OBJECTS_PUBLISH_PERIOD:
            self._last_objects_pub = now
            self._publish_objects(now, marker_frame)

        self._declutter_labels(labels)
        # Leader line from each label straight down to the top of the object it names
        for text, (ax, ay, az), (r, g, b) in labels:
            leader = Marker()
            leader.header = text.header
            leader.ns = text.ns + "_leaders"
            leader.id = text.id
            leader.type = Marker.LINE_LIST
            leader.action = Marker.ADD
            leader.pose.orientation.w = 1.0
            leader.scale.x = 0.015
            leader.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            label_bottom = text.pose.position.z - 1.1 * LABEL_SCALE
            leader.points = [Point(x=text.pose.position.x, y=text.pose.position.y, z=label_bottom),
                             Point(x=ax, y=ay, z=az)]
            leader.lifetime = text.lifetime
            current_markers.append(leader)

        self._marker_pub.publish(MarkerArray(markers=current_markers))
        self._marker_pub_alias.publish(MarkerArray(markers=current_markers))

    def _publish_objects(self, now: float, frame_id: str):
        """Publish the global object dictionary as JSON (what the map shows, plus velocities)."""
        objects = []
        for tracks_dict, is_dynamic in ((self._dynamic_tracks, True), (self._static_tracks, False)):
            for label, tracks in tracks_dict.items():
                for t in tracks:
                    live = t.live(now)
                    if not self._shown(t, is_dynamic, now):
                        continue
                    x, y = t.position_at(now)
                    objects.append({
                        "name": f"{label.replace(' ', '_')}_{t.id}", "class": label, "uid": t.uid,
                        "x": round(x, 3), "y": round(y, 3), "z": round(float(t.z), 3),
                        "w": round(float(t.width), 3), "h": round(float(t.height), 3),
                        "vx": round(float(t.x[2]), 3), "vy": round(float(t.x[3]), 3),
                        "dynamic": is_dynamic, "live": live, "seen_ago": round(now - t.last_seen, 1),
                    })
                    if t.color is not None:
                        objects[-1]["color"] = t.color
                    if not is_dynamic and label in PANEL_OBJECTS and t.wall_yaw is not None:
                        objects[-1]["yaw"] = round(t.wall_yaw, 3)  # direction of the wall it lies in
        self._objects_pub.publish(String(data=json.dumps({"frame": frame_id, "objects": objects})))

    @staticmethod
    def _declutter_labels(labels):
        """Stack the text labels of objects standing close together so each one stays readable.
        Fixed order (by marker id, i.e. by object): sorting by height reshuffled the stack whenever two
        heights jittered past each other, making labels jump 0.4 m."""
        placed = []
        for text, _, _ in sorted(labels, key=lambda l: (l[0].ns, l[0].id)):
            p = text.pose.position
            for _ in range(20):
                if not any(math.hypot(p.x - q.x, p.y - q.y) < LABEL_CLEAR_XY and abs(p.z - q.z) < LABEL_CLEAR_Z
                           for q in placed):
                    break
                p.z += LABEL_CLEAR_Z
            placed.append(p)

    def _add_track_marker(self, current_markers, labels, track, label_text, now_msg, marker_lifetime, is_dynamic,
                          frame_id, distance=0.0, base_label=None, seen_ago=None, pos=None):
        px, py = pos if pos is not None else (float(track.x[0]), float(track.x[1]))
        obj_width = max(0.05, float(track.width))
        obj_height = max(0.05, float(track.height))

        # Safety-net: clamp marker sizes using known real-world maximum dimensions
        if base_label is None:
            base_label = label_text.rsplit('_', 1)[0].replace('_', ' ') if '_' in label_text else label_text
        max_w, max_h = OBJECT_MAX_SIZES.get(base_label, OBJECT_MAX_SIZE_DEFAULT)
        obj_width = min(obj_width, max_w)
        obj_height = min(obj_height, max_h)

        # One colour per object: its shape, label and leader line all share it
        r, g, b = _object_color(label_text)
        remembered = seen_ago is not None
        marker_color = ColorRGBA(r=r, g=g, b=b, a=REMEMBERED_ALPHA if remembered else (0.75 if is_dynamic else 0.6))

        marker_ns_prefix = "yolo_dynamic" if is_dynamic else "yolo_static"

        # Persistent, globally unique ids: 10 per object (label, shape, desk, legs...). Marker.ADD with
        # the same id modifies the existing marker in RViz instead of stacking a new one.
        stable_id = int(getattr(track, "uid", 0)) * 10 if getattr(track, "uid", 0) else 900000 + track.id * 10

        # Elevation measured from the camera rays (floor objects are anchored at 0)
        base_z = float(getattr(track, 'z', 0.0))
        is_table = base_label in ["table", "dining table"]
        top_z = max(0.1, base_z + obj_height)

        # ── 3D Text label just above the object ──
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = now_msg
        marker.ns = f"{marker_ns_prefix}_labels"
        marker.id = stable_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = px
        marker.pose.position.y = py
        marker.pose.position.z = top_z + 0.20
        marker.scale.z = LABEL_SCALE
        marker.color = ColorRGBA(r=r, g=g, b=b, a=0.55 if remembered else 1.0)
        # "(" separates the name that voice_navigation_assistant.py matches from the details
        # RViz draws spaces in text markers as huge gaps, so the details use none
        details = f"dist:{distance:.1f}m|H:{obj_height:.2f}m"
        if base_z > 0.3:
            details += f"|on:{base_z:.2f}m"
        if remembered:
            details += f"|seen:{seen_ago:.0f}s-ago"
        marker.text = f"{label_text}\n({details})"
        marker.lifetime = marker_lifetime
        current_markers.append(marker)
        labels.append((marker, (px, py, top_z), (r, g, b)))

        # ── 3D Semantic Shape on the map ──
        cube_marker = Marker()
        cube_marker.header.frame_id = frame_id
        cube_marker.header.stamp = now_msg
        cube_marker.ns = f"{marker_ns_prefix}_shapes"
        cube_marker.id = stable_id + 1
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = px
        cube_marker.pose.position.y = py
        cube_marker.pose.orientation.w = 1.0
        cube_marker.lifetime = marker_lifetime

        if is_table:
            # ── TABLE: flat surface at the measured table-top height, in the table's own colour ──
            cube_marker.type = Marker.CUBE
            cube_marker.pose.position.z = top_z - 0.025  # Top surface
            cube_marker.scale.x = max(obj_width, 1.0)  # Tables are at least 1m wide
            cube_marker.scale.y = max(obj_width, 0.8)   # Tables have depth
            cube_marker.scale.z = 0.05  # 5cm thick surface
            cube_marker.color = marker_color
            current_markers.append(cube_marker)

            # Draw table legs
            for leg_i, (lx, ly) in enumerate([(-0.4, -0.3), (0.4, -0.3), (-0.4, 0.3), (0.4, 0.3)]):
                leg = Marker()
                leg.header.frame_id = frame_id
                leg.header.stamp = now_msg
                leg.ns = f"{marker_ns_prefix}_table_legs"
                leg.id = stable_id + 4 + leg_i
                leg.type = Marker.CYLINDER
                leg.action = Marker.ADD
                leg.pose.position.x = px + lx
                leg.pose.position.y = py + ly
                leg.pose.position.z = (top_z - 0.05) / 2.0
                leg.pose.orientation.w = 1.0
                leg.scale.x = 0.06
                leg.scale.y = 0.06
                leg.scale.z = top_z - 0.05
                leg.color = ColorRGBA(r=r * 0.7, g=g * 0.7, b=b * 0.7, a=0.6)
                leg.lifetime = marker_lifetime
                current_markers.append(leg)
            return

        wall_yaw = getattr(track, "wall_yaw", None) if base_label in PANEL_OBJECTS else None
        if wall_yaw is not None:
            # Thin panel along its wall: local y runs along the wall, x is the thickness
            yaw = wall_yaw - math.pi / 2
            cube_marker.type = Marker.CUBE
            cube_marker.pose.position.z = base_z + obj_height / 2.0
            cube_marker.pose.orientation.z, cube_marker.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            cube_marker.scale.x = PANEL_THICKNESS
            cube_marker.scale.y = obj_width
            cube_marker.scale.z = obj_height
            cube_marker.color = marker_color
            current_markers.append(cube_marker)
            return

        # Tesla-style semantic 3D rendering (Cylinders for people, spheres for balls, cubes for furniture)
        if base_label in ["person", "bottle", "vase"]:
            cube_marker.type = Marker.CYLINDER
        elif base_label in ["sports ball", "apple", "orange", "bowl"]:
            cube_marker.type = Marker.SPHERE
        else:
            cube_marker.type = Marker.CUBE
        cube_marker.pose.position.z = base_z + (obj_height / 2.0)
        cube_marker.scale.x = obj_width
        cube_marker.scale.y = obj_width
        cube_marker.scale.z = obj_height
        cube_marker.color = marker_color
        current_markers.append(cube_marker)

        # ── PHANTOM TABLE ──
        # If an object is floating (base_z > 0), draw a thin grey surface and pedestal under it
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
            desk.pose.position.z = base_z - 0.025
            desk.pose.orientation.w = 1.0
            desk.scale.x = obj_width * 1.5
            desk.scale.y = obj_width * 1.5
            desk.scale.z = 0.05
            desk.color = ColorRGBA(r=0.85, g=0.85, b=0.85, a=0.5)
            desk.lifetime = marker_lifetime
            current_markers.append(desk)

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
            leg.pose.orientation.w = 1.0
            leg.scale.x = 0.1  # Thin leg
            leg.scale.y = 0.1
            leg.scale.z = base_z - 0.05
            leg.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.4)
            leg.lifetime = marker_lifetime
            current_markers.append(leg)

    # ══════════════════════════════════════════════════════════════════════
    # ── HUD ──
    # ══════════════════════════════════════════════════════════════════════
    def _draw_lidar_overlay(self, frame):
        """Calibration aid: LiDAR returns drawn where the TF says they are in the image.

        Correct extrinsics put the dots on walls, door frames and people's torsos. Dots on the
        wrong side (mirrored) or behind the wrong object mean the lidar_yaw_deg/lidar_roll_deg
        or camera_* launch arguments need fixing.
        """
        with self._hud_lock:
            overlay = self._lidar_overlay
        if overlay is None:
            cv2.putText(frame, "LIDAR OVERLAY: no scan / TF", (10, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            return
        h, w = frame.shape[:2]
        u, v, horiz = overlay
        for ui, vi, r in zip(u.astype(int), v.astype(int), horiz):
            if 0 <= ui < w and 0 <= vi < h:
                t = min(1.0, r / 6.0)  # red = near, blue = far
                cv2.circle(frame, (int(ui), int(vi)), 2, (int(255 * t), 64, int(255 * (1 - t))), -1)

    def _draw_cached_boxes(self, frame):
        """Tesla FSD-style detection HUD with proximity colors, corner brackets, and assistive telemetry."""
        h, w = frame.shape[:2]
        if self._show_lidar_overlay:
            self._draw_lidar_overlay(frame)
        self._draw_grasp(frame)
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

        # ── OUTDOOR: walking corridor, ground hazards, crossing, alerts ──
        if self._mode == "outdoor":
            self._draw_outdoor(frame)

        labels = []  # (depth, box, lines, color) — laid out after all boxes are drawn
        for track_data in hud_tracks:
            if now - track_data['last_seen'] > HUD_TIMEOUT:
                continue

            x1 = track_data['x1']
            y1 = track_data['y1']
            x2 = track_data['x2']
            y2 = track_data['y2']
            label = track_data['label']
            conf = track_data['conf']
            is_dynamic = track_data['is_dynamic']
            depth = track_data['depth']
            kh = track_data['kh']
            kz = track_data.get('kz', 0.0)
            is_moving = track_data['is_moving']
            vel = track_data['vel']
            rel_pos = track_data['rel_pos']

            # Same colour as this object's RViz marker, so box, label and leader line match
            r, g, b = _object_color(label)
            color = (int(255 * b), int(255 * g), int(255 * r))

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
                    color_rect = np.full_like(roi, get_proximity_color(depth))
                    cv2.addWeighted(color_rect, 0.20, roi, 0.80, 0, roi)

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

            # ── 4. CONFIDENCE BAR ──
            bar_w = int((x2 - x1) * max(0.0, min(1.0, conf)))
            cv2.rectangle(frame, (x1, y2 + 2), (x1 + bar_w, y2 + 6), color, cv2.FILLED)

            # ── 5. ASSISTIVE LABEL: name / distance from the wearer / real height / direction ──
            detail = track_data.get('detail')
            if detail is None:
                src_tag = {"lidar": "LiDAR", "depth": "depth"}.get(track_data.get('source'), "cam")
                detail = f"{depth:.1f}m away ({src_tag}) | {kh:.2f}m tall"
                if kz > 0.3:
                    cls = track_data['label'].rsplit('_', 1)[0].replace('_', ' ')
                    detail += f" | mounted at {kz:.2f}m" if cls in WALL_MOUNTED else f" | on {kz:.2f}m surface"
                detail += f" | {rel_pos}"
                if is_moving:
                    detail += f" | MOVING {vel:.1f}m/s"
            labels.append((depth, (x1, y1, x2, y2), [label, detail], color))

        # ── 6. LABEL LAYOUT: nearest objects first, never overlapping another label ──
        font, scales = cv2.FONT_HERSHEY_SIMPLEX, (0.50, 0.40)
        top_limit, bottom_limit = 34, h - 24  # keep clear of the status and mode bars
        placed = []
        for _, (x1, y1, x2, y2), lines, color in sorted(labels, key=lambda l: l[0]):
            sizes = [cv2.getTextSize(t, font, sc, 1) for t, sc in zip(lines, scales)]
            block_w = max(tw for (tw, _), _ in sizes) + 10
            line_h = [th + bl + 4 for (_, th), bl in sizes]
            block_h = sum(line_h) + 4
            bx = max(0, min(x1, w - block_w))
            start = min(max(top_limit, y1 - block_h), bottom_limit - block_h)

            def collision(y):
                return next((r for r in placed if bx < r[2] and bx + block_w > r[0]
                             and y < r[3] and y + block_h > r[1]), None)

            # Prefer stacking upward (free space above objects), then downward
            by = start
            for step in (-1, 1):
                by = start
                for _ in range(12):
                    hit = collision(by)
                    if hit is None:
                        break
                    by = hit[1] - block_h - 2 if step < 0 else hit[3] + 2
                if collision(by) is None and top_limit <= by <= bottom_limit - block_h:
                    break
            by = max(top_limit, min(by, bottom_limit - block_h))
            placed.append((bx, by, bx + block_w, by + block_h))

            region = frame[by:by + block_h, bx:bx + block_w]
            if region.size:
                bg = np.full_like(region, (25, 25, 25))
                frame[by:by + block_h, bx:bx + block_w] = cv2.addWeighted(bg, 0.75, region, 0.25, 0)
            cv2.rectangle(frame, (bx, by), (bx + block_w, by + block_h), color, 1)
            # Leader line from the label to its box, so it is clear which object it names
            mid = (x1 + x2) // 2
            lead_x = max(bx + 4, min(mid, bx + block_w - 4))
            cv2.line(frame, (lead_x, by + block_h if by < y1 else by), (mid, y1), color, 1, cv2.LINE_AA)
            ty = by + 2
            for text, sc, lh, col in zip(lines, scales, line_h, (color, (255, 255, 255))):
                ty += lh
                cv2.putText(frame, text, (bx + 5, ty - 4), font, sc, col, 1, cv2.LINE_AA)

        # ══════════════════════════════════════════════════════════════════════
        # ── TESLA HUD STATUS BAR (TOP) ──
        # ══════════════════════════════════════════════════════════════════════
        cv2.rectangle(frame, (0, 0), (w, 32), (20, 20, 20), cv2.FILLED)

        now = time.monotonic()
        fps = getattr(self, '_display_fps', 0.0)
        if not hasattr(self, '_last_fps_time'):
            self._last_fps_time = now
        last_fps_time = self._last_fps_time
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
            alert_text = f"!! {danger_count} CLOSE !!"  # short: "MOVING" sits left of it on a 640 px frame
            cv2.putText(frame, alert_text, (w - 150, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(frame, "PATH CLEAR", (w - 130, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

        # ── MODE INDICATOR (BOTTOM BAR) ──
        if self._mode == "indoor":
            now_m = time.monotonic()
            mem_count = sum(self._shown(t, False, now_m) for tracks in list(self._static_tracks.values()) for t in tracks)
            mode_text = f"MODE: INDOOR [MAPPING] | Map: {mem_count} objects"
            mode_color = (255, 200, 0)
        else:
            hud = self._outdoor_hud
            lanes = hud["lanes"] if hud else {}
            mode_text = "MODE: OUTDOOR | path " + (" ".join(
                f"{k[0].upper()} {'clear' if v >= oa.LOOK_AHEAD else f'{v:.1f}m'}" for k, v in
                (("left", lanes.get("left", oa.LOOK_AHEAD)), ("center", lanes.get("center", oa.LOOK_AHEAD)),
                 ("right", lanes.get("right", oa.LOOK_AHEAD)))))
            if hud:
                mode_text += f" | LiDAR {'on' if hud['lidar'] is not None else 'off'}" \
                             f" | ground {'fitted' if hud['ground'] is not None and hud['ground'].ok else '-'}" \
                             f" | odom {'%.1fm/s' % hud['speed'] if hud['odom'] and hud['speed'] is not None else '-'}"
            mode_color = (0, 200, 255)

        cv2.putText(frame, mode_text, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1, cv2.LINE_AA)


def main(args=None) -> None:
    rclpy.init(args=args)

    # Parse mode from environment variable
    mode = os.environ.get("WEARABLE_MODE", "indoor").lower()
    if mode not in MODE_PARAMS:
        mode = "indoor"

    node = ObjectPerceptionNode(mode=mode)

    # ── ZERO-LAG ARCHITECTURE: Offload ROS 2 spin to background thread ──
    def spin():
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass  # Ctrl+C shuts the context down under the spinning thread
        except Exception:
            if rclpy.ok():
                raise  # a real error; anything else is a callback caught mid-shutdown (publish on a dead context)

    ros_thread = threading.Thread(target=spin, daemon=True)
    ros_thread.start()

    # ── MAIN THREAD: High-speed OpenCV GUI loop (maximum FPS, 0ms delay) ──
    try:
        if node._show_window:
            waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(waiting_frame, "Waiting for camera feed...", (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

            while rclpy.ok():
                # The frame the boxes were computed on, so they sit exactly on moving people (the newest
                # camera frame is ~50 ms ahead of them); the raw feed only when processing stalls
                if node._window_rename:
                    try:
                        cv2.destroyWindow(node._window_name)
                    except Exception:
                        pass
                    node._window_name, node._window_rename = node._window_rename, None
                    cv2.namedWindow(node._window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(node._window_name, 800, 600)
                frame = node._gui_frame
                hud_frame = node._hud_frame
                if hud_frame is not None and time.monotonic() - hud_frame[1] < HUD_SYNC_MAX_AGE:
                    frame = hud_frame[0]
                if frame is not None:
                    display_frame = frame.copy()
                    node._draw_cached_boxes(display_frame)
                    cv2.imshow(node._window_name, display_frame)
                else:
                    cv2.imshow(node._window_name, waiting_frame)

                key = cv2.waitKey(30) & 0xFF
                if key == 27 or key == ord('q'):
                    break
                if key == ord('l'):
                    node._show_lidar_overlay = not node._show_lidar_overlay
        else:
            # Headless mode: no GUI, just let ROS spin handle everything
            node.get_logger().info("Running in HEADLESS mode (no display). Press Ctrl+C to stop.")
            ros_thread.join()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node._show_window:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
