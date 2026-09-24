#!/usr/bin/env python3
"""
object_perception.py
====================
ROS 2 Jazzy – Wearable Blind-Assist Vision Node  (Ultralytics YOLOE open-vocabulary edition)

Tesla AI-Grade Dual-Mode Perception System:
  INDOOR  – Persistent spatial memory map, scene recall, object finding
  OUTDOOR – Forward-only collision avoidance, no memory, maximum responsiveness

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
from visionnav.model_paths import model_path
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, LaserScan, CompressedImage
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
    "door", "doorway", "sliding door", "glass door", "door handle", "door knob", "gate", "window",
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
    "hole in floor", "pothole", "floor step", "ramp", "wet floor sign", "puddle", "cable on floor",
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
# Several prompts for one thing raise recall; they are reported, mapped and navigated to under one
# name. Measured on the rig: a switch scores 0.60 as "black switch", 0.44 "electric switch", 0.38
# "switch", but only 0.01 as "light switch", and without them it was mostly called a "doorbell".
PROMPT_SYNONYMS = {
    "wall switch": "light switch", "black switch": "light switch", "white switch": "light switch",
    "electric switch": "light switch", "switch": "light switch", "switch board": "light switch",
    "electrical switch panel": "light switch",
    "power outlet": "wall socket", "plug socket": "wall socket",
    "staircase": "stairs", "floor step": "step", "doorway": "door", "sliding door": "door",
    "glass door": "door", "door knob": "door handle", "dustbin": "trash can",
    "cardboard box": "box", "office chair": "chair", "plastic chair": "chair", "desk": "table",
    "dining table": "table", "cupboard": "cabinet", "wall shelf": "shelf",
    "water bottle": "bottle", "mug": "cup", "table fan": "fan", "pedestal fan": "fan",
    "wire": "cable on floor", "wash basin": "sink", "elevator door": "door",
}
_VOCAB_HASH = hashlib.sha1("|".join(VOCABULARY).encode()).hexdigest()[:8]
ENGINE_PATH = model_path(f"yoloe-11s-seg-indoor-{_VOCAB_HASH}.engine")

INDOOR_CLASSES = set(VOCABULARY) - {
    "bicycle", "motorcycle", "bus", "truck", "car", "traffic light", "stop sign", "fire hydrant",
    "curb", "pothole", "three-wheeler",
}
OUTDOOR_CLASSES = {
    "person", "bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat",
    "traffic light", "stop sign", "fire hydrant", "bench", "chair", "potted plant", "backpack",
    "umbrella", "stairs", "staircase", "step", "curb", "pothole", "hole in floor", "obstacle", "door",
    "trash can", "three-wheeler", "dustbin",
}
# Drops: a blind user needs more warning before these than before a chair.
DROP_HAZARDS = {"stairs", "step", "hole in floor", "pothole", "curb", "escalator"}

# Class-specific NMS is done by YOLO. Across classes, only suppress pairs the model
# genuinely confuses — a person sitting on a chair must keep both detections.
# Groups hold both raw and reported names (FRIENDLY_NAMES), since either may be compared.
CONFUSABLE_GROUPS = [
    {"tv", "monitor", "laptop"},
    # a door and the furniture doors that look just like it
    {"door", "wardrobe", "cabinet", "refrigerator"},
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
YOLO_IOU = 0.50
# Small objects are the ones YOLO misnames most (a door handle as a cup, a remote as a phone),
# so they need more confidence than furniture before they are shown or mapped.
SMALL_OBJECT_CONF = 0.55
SMALL_OBJECTS = {"door handle", "door knob", "keys", "pen", "wallet", "glasses", "watch", "phone charger", "cup", "bottle", "cell phone", "mouse", "remote", "book", "vase", "clock",
                 "scissors", "toothbrush", "spoon", "fork", "knife", "wine glass", "sports ball"}
CROSS_CLASS_OVERLAP = 0.70   # intersection / smaller box area
CROSS_CLASS_SAME_BOX_IOU = 0.80  # any two static labels on (almost) the same box are one detection

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
                 "bus", "truck", "dog", "cat", "fire hydrant", "backpack"}
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
DEPTH_INPUT_SIZE = int(os.environ.get("WEARABLE_DEPTH_SIZE", "392"))  # short side, multiple of 14
DEPTH_SCALE_MIN_PTS = 15     # LiDAR points needed to (re)calibrate the depth scale
DEPTH_SCALE_ALPHA = 0.2      # smoothing of the per-frame scale estimate
DEPTH_SCALE_MAX_RESID = 0.15 # a frame whose depth/LiDAR ratios spread more than this is not used to calibrate
DEPTH_SCALE_FRESH_S = 3.0    # s: the scale only earns the tight range sigma while calibrated this recently
MONO_MAX_POINTS = 2000       # object pixels back-projected per detection

# ── TRACKING PARAMETERS ──
MIN_HITS_STATIC = 4          # sightings before a static object is mapped / remembered
MIN_HITS_DYNAMIC = 2
UNCONFIRMED_TIMEOUT = 1.5    # s, tentative tracks die fast (kills one-frame hallucinations)
TRACK_DEBUG = os.environ.get("WEARABLE_TRACK_DEBUG", "0") == "1"  # log why each new static object is created
CONFIRMED_TIMEOUT = 30.0    # s a confirmed (MIN_HITS_STATIC) but not yet reliable object survives out of view,
                             # so the next glance at it matches it instead of mapping it again under a new ID
UNSEEN_DROP_S = 1.0          # s, an object the camera is looking at but no longer detects is removed
VISIBILITY_MAX_RANGE = 6.0   # m, only apply the rule above to objects this close
# Real-time map: an object is drawn only while it is being detected right now
LIVE_WINDOW = 1.0            # s
LIVE_MIN_HITS = 2            # sightings within LIVE_WINDOW (one stray frame is not enough)
MARKER_LIFETIME = 0.5        # s, RViz removes an object this soon after it stops being published
# PERSISTENT GLOBAL MAP (indoor): once an object has been seen reliably it stays on the map, faded,
# for the rest of the session, and keeps its ID/name when seen again from another angle. Set
# WEARABLE_MEMORY_S (e.g. 8) for the old real-time-only behaviour instead. A remembered object is
# dropped when the camera looks straight at its spot, with nothing closer in the way, and does not
# see it there (UNSEEN_DROP_RELIABLE_S) — or when a live object of the same class stands on it (a
# duplicate, e.g. one created while its distance estimate briefly jumped).
MEMORY_MIN_HITS = 12
MEMORY_MIN_SPAN = 2.0        # s between first and latest sighting
MEMORY_TTL = float(os.environ.get("WEARABLE_MEMORY_S", "inf"))  # s a reliable object outlives its last sighting
UNSEEN_DROP_RELIABLE_S = 3.0  # s a *reliable* object may go unseen-in-view before it is removed
                              # (longer than UNSEEN_DROP_S: a real object deserves more benefit of the
                              # doubt than a fresh, unconfirmed detection)
BLIND_SPOT_RADIUS = 0.8      # m: a remembered object this close to the wearer is expected to be below
                              # the chest-mounted camera/LiDAR's view, so it is never dropped for going
                              # unseen (memory-anchored terminal navigation needs it to still be there)
OCCLUSION_DEPTH_MARGIN = 0.3  # m: something this much closer in the same pixel hides the object
FREE_SPACE_MARGIN = 1.0      # m: live depth this far beyond a remembered object means its spot is empty
FREE_SPACE_CLEAR_S = 0.4     # s of consistent free-space evidence before it is pruned (rejects depth glitches)
SEE_THROUGH = {"door", "window"}  # open doorways / windows: depth reading past them is expected, not a ghost
MAX_REMEMBERED_OBJECTS = 200  # oldest-seen remembered objects are evicted first past this count
REMEMBERED_ALPHA = 0.30      # remembered (not currently seen) objects are drawn translucent
OBJECTS_PUBLISH_PERIOD = 0.2 # s, /semantic_objects rate (5 Hz)
STATIC_POS_Q = 0.01          # m^2/s, static objects may slowly be re-estimated / moved
STATIC_MIN_VAR = 0.08 ** 2   # m^2: a static object's position never gets more certain than this, so each new
                             # sighting keeps real weight (a moving average that favours the live data)
REVISIT_GAP_S = 2.0          # s unseen after which the next sighting is treated as a revisit
REVISIT_VAR = 0.30 ** 2      # m^2: prior widened to this on a revisit (absorbs SLAM drift / a new viewing angle)
CHI2_GATE_2D = 9.21          # 99 % gate for a 2-D innovation
SAME_OBJECT_IOU = 0.3        # footprints overlapping this much (and statistically consistent) are one object
# Two *different* labels on one spot (a cabinet also read as a door and a notice board) are one object when
# the footprints overlap this much, the widths are within this ratio and the height bands overlap this much
# of the shorter one. A bottle on a table (tiny vs large, different heights) stays two objects.
SAME_PLACE_IOU = 0.40
SAME_PLACE_SIZE_RATIO = 1.6
SAME_PLACE_Z_OVERLAP = 0.5
BIG_COST = 1e6
HUD_TIMEOUT = 0.7            # s, camera-view boxes vanish this fast once the object is not detected
HUD_MIN_HITS = 2             # sightings before a box is labelled in the camera view
LABEL_SCALE = 0.15           # RViz text height (m)
LABEL_CLEAR_XY = 1.0         # m, RViz labels closer than this horizontally are stacked...
LABEL_CLEAR_Z = 0.40         # ...this far apart vertically (a leader line still ties each to its object)
# Distinct colours so each label, its leader line and its object visibly belong together (RGB 0-1)
OBJECT_PALETTE = [
    (0.10, 0.85, 1.00), (1.00, 0.55, 0.10), (0.35, 1.00, 0.35), (1.00, 0.30, 0.75),
    (1.00, 0.95, 0.20), (0.60, 0.45, 1.00), (0.20, 1.00, 0.80), (1.00, 0.35, 0.30),
    (0.75, 1.00, 0.20), (0.95, 0.70, 1.00),
]


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
        "dynamic_timeout":      0.5,     # SUPER FAST cleanup for moving objects
        "static_assoc":         1.50,    # hard association limit (m); main gate is statistical
        "dynamic_assoc":        2.00,
        "danger_distance":      1.5,     # Indoor danger threshold
        "collision_corridor_w": 0.8,     # Narrow indoor corridor
    },
    "outdoor": {
        "conf_threshold":       0.30,
        "static_timeout":       3.0,     # Very short memory outdoors
        "dynamic_timeout":      0.5,
        "static_assoc":         2.00,
        "dynamic_assoc":        2.50,
        "danger_distance":      2.0,     # Outdoor needs earlier warnings
        "collision_corridor_w": 1.2,     # Shoulder-width walking corridor
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
            self.P[2, 2] = self.P[3, 3] = 1.0
            self.P[4, 4] = self.P[5, 5] = 1.0
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
        self.unseen_in_view = 0.0
        self.free_in_view = 0.0
        self.seen_times = deque([now], maxlen=10)
        self.uid = 0  # globally unique object id (assigned by the node), used for RViz marker ids
        self.votes = {}  # label -> accumulated (priority-weighted) confidence, for look-alike classes
        self.from_memory = False  # reloaded from a saved map, not yet seen in this session

        self.H = np.zeros((2, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

    @property
    def confirmed(self) -> bool:
        return self.hits >= (MIN_HITS_DYNAMIC if self.is_dynamic else MIN_HITS_STATIC)

    @property
    def reliable(self) -> bool:
        """Seen often enough, over long enough, to be a real object worth remembering."""
        return self.hits >= MEMORY_MIN_HITS and self.last_seen - self.first_seen >= MEMORY_MIN_SPAN

    def remembered(self, now: float) -> bool:
        """Not detected now, but a reliable object seen within MEMORY_TTL."""
        return self.reliable and now - self.last_seen <= MEMORY_TTL

    def live(self, now: float) -> bool:
        """Confirmed and still being detected right now."""
        return self.confirmed and sum(1 for t in self.seen_times if now - t <= LIVE_WINDOW) >= LIVE_MIN_HITS

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

        F = np.eye(6, dtype=np.float64)
        # Position += Velocity * dt + 0.5 * Accel * dt^2 ; Velocity += Accel * dt
        F[0, 2] = F[1, 3] = dt
        F[0, 4] = F[1, 5] = 0.5 * dt * dt
        F[2, 4] = F[3, 5] = dt
        Q = np.diag([0.02, 0.02, 0.5, 0.5, 2.0, 2.0]) * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Unobserved objects coast to a stop instead of drifting forever
        if now - self.last_seen > 0.3:
            self.x[2:6] *= 0.80 ** (dt / 0.05)
        self.velocity = float(math.hypot(self.x[2], self.x[3]))

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
        dim_alpha = 0.40 if self.is_dynamic else 0.25
        self.width = (1.0 - dim_alpha) * self.width + dim_alpha * max(0.05, float(width))
        self.height = (1.0 - dim_alpha) * self.height + dim_alpha * max(0.05, float(height))
        self.z = (1.0 - dim_alpha) * self.z + dim_alpha * float(z)
        self.dist = 0.5 * self.dist + 0.5 * float(dist)

        self.conf = max(conf, self.conf * 0.98)
        self.hits += 1
        self.last_seen = now
        self.seen_times.append(now)
        self.unseen_in_view = 0.0
        self.free_in_view = 0.0
        self.velocity = float(math.hypot(self.x[2], self.x[3]))
        self.is_moving = self.is_dynamic and self.hits >= 4 and self.velocity > 0.30

    @property
    def distance(self) -> float:
        return float(math.hypot(self.x[0], self.x[1]))


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
        self._inference_results = None
        self._inference_busy = False
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
        self._dynamic_classes = {"person", "bicycle", "car", "motorcycle", "bus", "truck", "dog", "cat"}
        self._hazard_classes = self._dynamic_classes

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

        # Un-mirror the stream so left in the image is the wearer's left
        if self._flip_input:
            frame = cv2.flip(frame, 1)

        self._frame_count += 1
        now_mono = time.monotonic()
        self._last_image_time = now_mono
        self._gui_frame = frame

        with self._inference_lock:
            if not self._inference_busy:
                self._latest_frame = frame.copy()
                self._latest_frame_stamp = msg.header.stamp

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_buffer.append(msg)

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
        shrink = 0.15 * (x2 - x1)
        in_cols = (u >= x1 + shrink) & (u <= x2 - shrink)
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
        ratio = ratio[(ratio > 0.4) & (ratio < 2.5)]
        if ratio.size < DEPTH_SCALE_MIN_PTS:
            return
        r = float(np.median(ratio))
        spread = float(np.median(np.abs(ratio / r - 1.0)))
        if spread > DEPTH_SCALE_MAX_RESID:
            return  # LiDAR and depth disagree in shape this frame (glitch / wrong row): keep the old scale
        self._depth_scale = r if not self._depth_scale_valid else (
            (1 - DEPTH_SCALE_ALPHA) * self._depth_scale + DEPTH_SCALE_ALPHA * r)
        self._depth_scale = max(0.5, min(2.0, self._depth_scale))
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
        is_dynamic = raw in self._dynamic_classes
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
        on_floor = raw in FLOOR_OBJECTS or label in FLOOR_OBJECTS or is_dynamic
        on_desk = not on_floor and (raw in DESKTOP_OBJECTS or label in DESKTOP_OBJECTS)

        # ── 1. OPTICAL DEPTH CANDIDATES (depth, sigma) ──
        # Known-size prior, using only box dimensions not cut off by the frame edge
        d_w = typ_w * fx / pix_w
        d_h = typ_h * fy / pix_h
        if label in FLAT_OBJECTS or raw in FLAT_OBJECTS or raw == "person":
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
            return d is not None and 0.3 < d < 15.0 and size_d / size_gate < d < size_d * size_gate

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
        if mono is not None and not (0.3 < mono["dist"] < 15.0 and size_d / mono_gate < mono["dist"] < size_d * mono_gate):
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
        lidar = self._lidar_hits_in_box(proj, det["box"], det.get("mask"), depth_hint=snap_hint, debug=lidar_debug)
        if lidar_debug is not None:
            self._log_lidar_snap_debug(label, depth, lidar, lidar_debug)
        lidar_gate = 4.0 if (known_size or mono is not None) else 1e6
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
        depth = max(0.3, min(depth, 15.0))

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
        height_m = max(0.05, min(height_m, max_h))

        sigma_xy = math.sqrt(sigma_d ** 2 + (0.02 * depth) ** 2)
        return {
            **det,
            "is_dynamic": is_dynamic,
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
    def _load_detector(self, torch):
        """YOLOE with the offline VOCABULARY. Returns (model, is_tensorrt_engine).

        Text embeddings are computed once by set_classes() and baked into the TensorRT engine, so
        the per-frame path never runs a text encoder. The engine is built on first start (or after
        the vocabulary changes), before the ROS loop begins.
        """
        from ultralytics import YOLO, YOLOE
        if torch.cuda.is_available():
            if not os.path.isfile(ENGINE_PATH):
                self.get_logger().info(f"Building TensorRT engine {os.path.basename(ENGINE_PATH)} "
                                       f"({len(VOCABULARY)} classes, one-time, a few minutes)...")
                try:
                    model = YOLOE(MODEL_PATH)
                    model.set_classes(VOCABULARY, model.get_text_pe(VOCABULARY))
                    built = model.export(format="engine", half=True, workspace=4, imgsz=640, device=0)
                    shutil.move(str(built), ENGINE_PATH)
                    for old in glob.glob(model_path("yoloe-11s-seg-indoor-*.engine")):  # older vocabularies
                        if old != ENGINE_PATH:
                            os.remove(old)
                    stem = os.path.splitext(MODEL_PATH)[0]
                    for onnx in (stem + ".onnx", stem + ".fp16.onnx"):  # export intermediates
                        if os.path.isfile(onnx):
                            os.remove(onnx)
                    del model
                    torch.cuda.empty_cache()
                except Exception as e:
                    self.get_logger().error(f"TensorRT export failed, using PyTorch weights: {e}")
            if os.path.isfile(ENGINE_PATH):
                model = YOLO(ENGINE_PATH, task="segment")
                return model, True
        model = YOLOE(MODEL_PATH)
        model.set_classes(VOCABULARY, model.get_text_pe(VOCABULARY))
        return model, False

    def _load_model(self):
        import torch
        from ultralytics.cfg import DEFAULT_CFG_DICT
        self._yolo_model, on_engine = self._load_detector(torch)
        self._use_half = torch.cuda.is_available()
        # The TensorRT engine is already fp16; newer Ultralytics replaced half=True with quantize="fp16"
        if on_engine or not self._use_half:
            self._precision_kwargs = {}
        elif "quantize" in DEFAULT_CFG_DICT:
            self._precision_kwargs = {"quantize": "fp16"}
        else:
            self._precision_kwargs = {"half": True}
        self._model_names = self._yolo_model.names
        self._class_ids = {
            mode: [i for i, n in self._model_names.items() if n in classes]
            for mode, classes in (("indoor", INDOOR_CLASSES), ("outdoor", OUTDOOR_CLASSES))
        }
        self.get_logger().info(f"YOLO device: {'CUDA fp16' if self._use_half else 'CPU'}"
                               f"{' (TensorRT)' if on_engine else ''}")

        self._depth_model = None
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
            model = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384], max_depth=20)
            model.load_state_dict(torch.load(DEPTH_WEIGHTS, map_location='cpu'))
            self._depth_model = model.cuda().half().eval()
            self._depth_mean = torch.tensor([0.485, 0.456, 0.406], device='cuda').view(1, 3, 1, 1)
            self._depth_std = torch.tensor([0.229, 0.224, 0.225], device='cuda').view(1, 3, 1, 1)
            self.get_logger().info(f"Metric depth: Depth Anything V2 indoor (input {DEPTH_INPUT_SIZE}px, CUDA fp16)")
        except Exception as e:
            self.get_logger().warn(f"Metric depth disabled: {e}")

    def _infer_depth(self, bgr):
        """Per-pixel metric depth (m, along the optical axis) for the processed frame."""
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
            d = self._depth_model(x)[:, None].float()
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
                    self._latest_frame = None
                    self._inference_busy = True

            if frame is None:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            try:
                results = self._yolo_model.predict(
                    frame, conf=self._conf_threshold, iou=YOLO_IOU,
                    classes=self._class_ids[self._mode], verbose=False, **self._precision_kwargs,
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
                    raw_label = self._model_names.get(int(cid), "unknown")
                    if raw_label in SMALL_OBJECTS and conf < max(SMALL_OBJECT_CONF, self._conf_threshold):
                        continue
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    bw_, bh_ = max(1, x2 - x1), max(1, y2 - y1)

                    if bh_ / float(bw_) > OBJECT_REJECT_ASPECT_RATIOS.get(raw_label, 1e9):
                        continue
                    max_aspect = OBJECT_MAX_ASPECT_RATIOS.get(raw_label, 100.0)
                    if bh_ / float(bw_) > max_aspect:
                        # Anchor to the bottom (desk/floor) and slice the top off
                        y1 = max(0, int(y2 - bw_ * max_aspect))

                    mask = None
                    if poly is not None and len(poly) >= 3:
                        mask = np.zeros((h, w), dtype=np.uint8)
                        cv2.fillPoly(mask, [poly.astype(np.int32)], 1)

                    dets.append({
                        "box": (x1, y1, x2, y2), "conf": float(conf), "raw_label": raw_label,
                        "label": FRIENDLY_NAMES.get(raw_label, raw_label), "mask": mask,
                    })

            dmap = None
            if self._depth_model is not None and (dets or self._grasp.active):
                try:
                    dmap = self._infer_depth(frame)
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
                    both_static = (k["raw_label"] not in self._dynamic_classes
                                   and d["raw_label"] not in self._dynamic_classes)
                    if both_static and inter / max(area_a + area_b - inter, 1) >= CROSS_CLASS_SAME_BOX_IOU:
                        duplicate = True
                        break
                    if not (k["label"] == d["label"] or self._confusable(k["label"], d["label"])
                            or self._confusable(k["raw_label"], d["raw_label"])):
                        continue
                    if inter / max(min(area_a, area_b), 1) > CROSS_CLASS_OVERLAP:
                        duplicate = True
                        break
                if not duplicate:
                    kept.append(d)

            with self._inference_lock:
                self._inference_results = (stamp, kept, frame, dmap)
                self._inference_busy = False

    # ══════════════════════════════════════════════════════════════════════
    # ── TRACKING ──
    # ══════════════════════════════════════════════════════════════════════
    def _tracking_callback(self):
        now = time.monotonic()

        new_results = None
        with self._inference_lock:
            if self._inference_results is not None:
                new_results = self._inference_results
                self._inference_results = None

        if new_results is not None:
            stamp, dets, frame, dmap = new_results
            self._process_detections(stamp, dets, frame, now, dmap)
        else:
            # Predict step for smooth interpolation
            for tracks in list(self._dynamic_tracks.values()) + list(self._static_tracks.values()):
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
                    e = math.hypot(m["px"] - t.x[0], m["py"] - t.x[1])
                    m2 = t.mahalanobis_sq(m["px"], m["py"], m["sigma"])
                    if e <= max_gate and (m2 <= CHI2_GATE_2D or e <= min_sep):
                        size_penalty = abs(m["width_m"] - t.width) / max(t.width, 0.1)
                        cost[i, j] = m2 + 2.0 * size_penalty
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

        for i, m in enumerate(meas):
            t = assigned[i]
            if t is None:
                # A second detection on top of an existing object is a duplicate, never a new object
                near = [tr for tr in tracks if _same_object((m["px"], m["py"]), _dedup_width(label, m["width_m"]),
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
                t.uid = self._next_uid
                self._next_uid += 1
                tracks.append(t)
            else:
                t.update(m["px"], m["py"], m["z"], m["width_m"], m["height_m"], m["conf"],
                         now, m["sigma"], m["depth"])
                if t.from_memory:
                    t.from_memory = False
                    if not self._memory_confirmed:
                        self._memory_confirmed = True
                        self.get_logger().info(f"🏠 Localized: remembered {label}_{t.id} seen again where "
                                               f"it was saved; the saved object map is live")
            m["track"] = t

        merged_into = self._merge_duplicate_tracks(tracks, label)
        self._deleted_uids.extend(t.uid for t in merged_into.pop("_removed", []))
        for m in meas:
            m["track"] = merged_into.get(id(m["track"]), m["track"])

    def _lookalike_track(self, m):
        """Label of a static track of another class on this measurement's spot (same physical object):
        a look-alike on an overlapping footprint, or any label on the same footprint, size and height."""
        def same(t, lbl, min_sep=0.0):
            return _same_object((m["px"], m["py"]), _dedup_width(m["label"], m["width_m"]), m["sigma"] ** 2,
                                t.x[:2], _dedup_width(lbl, t.width), _track_var(t), min_sep)

        min_sep = _min_separation(m["label"])
        if any(same(t, m["label"], min_sep) for t in self._static_tracks.get(m["label"], [])):
            return None
        best = None
        for lbl, tracks in self._static_tracks.items():
            if lbl == m["label"]:
                continue
            confusable = self._confusable(lbl, m["label"])
            for t in tracks:
                match = same(t, lbl) if confusable else _same_place(
                    (m["px"], m["py"]), _dedup_width(m["label"], m["width_m"]), m["z"], m["height_m"],
                    t.x[:2], _dedup_width(lbl, t.width), t.z, t.height)
                if match and (best is None or t.hits > best[1].hits):
                    best = (lbl, t)
        return best[0] if best else None

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
                    "z": round(float(t.z), 3), "w": round(t.width, 3), "h": round(t.height, 3), "hits": t.hits}
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
            t.first_seen = now - MEMORY_MIN_SPAN - REVISIT_GAP_S - 1.0
            t.last_seen = now - REVISIT_GAP_S - 1.0
            t.seen_times.clear()
            t.from_memory = True
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
        entries = sorted(((lbl, t) for lbl, tracks in self._static_tracks.items() for t in tracks),
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

    def _in_view_batch(self, xyz: np.ndarray, pose, K, h: int):
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
        near = np.hypot(p_base[:, 0] - self._cam_t[0], p_base[:, 1] - self._cam_t[1]) <= VISIBILITY_MAX_RANGE
        front = pc[:, 2] >= 0.5
        zc = np.where(front, pc[:, 2], 1.0)
        u = fx * pc[:, 0] / zc + cx
        v = fy * pc[:, 1] / zc + cy
        in_view = near & front & (0.12 * w < u) & (u < 0.88 * w) & (0.12 * h < v) & (v < 0.88 * h)
        return in_view, u, v, pc[:, 2]

    def _occluded(self, u: float, v: float, cam_depth: float, dmap, w: int, h: int, proj=None,
                  half_px: float = 10.0) -> bool:
        """True if the metric depth map (or, without one, the LiDAR) shows something clearly closer here."""
        if dmap is None:
            if proj is None:
                return False
            pu, _, _, _, zc = proj
            return int(np.count_nonzero((np.abs(pu - u) < half_px) & (zc < cam_depth - OCCLUSION_DEPTH_MARGIN))) >= 2
        ui, vi = int(u), int(v)
        if not (0 <= ui < w and 0 <= vi < h):
            return False
        d = dmap[vi, ui] * self._depth_scale
        return d > 0.05 and d < cam_depth - OCCLUSION_DEPTH_MARGIN

    def _free_space(self, u: float, v: float, cam_depth: float, t, K, dmap, proj) -> bool:
        """Ray-cast clearing: does live depth show open space well beyond a remembered object's spot?

        Uses the low percentile of a patch of the depth map (or, without depth, the LiDAR points
        crossing the object's columns while the scan plane could physically hit it), so any surface
        still at the object's range keeps it alive.
        """
        fx, _, _, _, w = K
        half_px = max(3, int(0.3 * fx * t.width / max(cam_depth, 0.1)))
        limit = cam_depth + FREE_SPACE_MARGIN
        if dmap is not None:
            H, W = dmap.shape
            ui, vi = int(u), int(v)
            patch = dmap[max(0, vi - half_px):min(H, vi + half_px + 1), max(0, ui - half_px):min(W, ui + half_px + 1)]
            return patch.size > 0 and float(np.percentile(patch, 25)) * self._depth_scale > limit
        if proj is not None and self._lidar_t is not None and t.z <= self._lidar_t[2] <= t.z + t.height:
            pu, _, _, _, zc = proj
            col = zc[np.abs(pu - u) < half_px]
            return col.size >= 3 and float(col.min()) > limit
        return False

    def _process_detections(self, msg_stamp, dets, frame, now, dmap=None):
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

        measurements = []
        for det in dets:
            m = self._measure_detection(det, K, h, proj, dmap)
            m["px"] = px0 + cyaw * m["bx"] - syaw * m["by"]
            m["py"] = py0 + syaw * m["bx"] + cyaw * m["by"]
            measurements.append(m)

        # The same physical object seen under a look-alike label (a door read as a wardrobe) updates
        # its existing track instead of starting a second object on the same spot
        for m in measurements:
            m["seen_label"] = m["label"]
            if not m["is_dynamic"]:
                lookalike = self._lookalike_track(m)
                if lookalike is not None:
                    m["label"] = lookalike

        groups = {}
        for m in measurements:
            groups.setdefault((m["label"], m["is_dynamic"]), []).append(m)
        for (label, is_dynamic), group in groups.items():
            self._associate(label, group, is_dynamic, now)
        for m in measurements:
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
                    alpha = 0.70 if is_dynamic else 0.55
                    x1_s = int(round((1 - alpha) * prev_hud['x1'] + alpha * x1))
                    y1_s = int(round((1 - alpha) * prev_hud['y1'] + alpha * y1))
                    x2_s = int(round((1 - alpha) * prev_hud['x2'] + alpha * x2))
                    y2_s = int(round((1 - alpha) * prev_hud['y2'] + alpha * y2))
                    depth_s = (1 - alpha) * prev_hud['depth'] + alpha * depth
                else:
                    x1_s, y1_s, x2_s, y2_s, depth_s = x1, y1, x2, y2, depth

                # A single-frame detection is often a misclassification: label it once seen twice
                if track.hits >= HUD_MIN_HITS:
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
            if depth_s < danger_d * 2.0:
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
        # so negative evidence and the stale-duplicate rule below must not remove them.
        near_user = (lambda t: t.reliable and math.hypot(t.x[0] - px0, t.x[1] - py0) < BLIND_SPOT_RADIUS) \
            if self._mode == "indoor" else (lambda t: False)

        # ── CLEAN-UP OF EVERY CLASS (not only those detected this frame) ──
        for tracks in self._dynamic_tracks.values():
            self._purge_tracks(tracks, True, now)
        for label, tracks in self._static_tracks.items():
            self._purge_tracks(tracks, False, now)
            # A remembered object next to a live one of the same class is a stale duplicate of it,
            # e.g. created while its distance estimate jumped, or out of view below the camera.
            live = [t for t in tracks if t.live(now)]
            stale = [t for t in tracks if not t.live(now) and not near_user(t) and not self._protected(t)
                     and any(_same_object(t.x[:2], t.width, _track_var(t), l.x[:2], l.width, _track_var(l),
                                          _min_separation(label))
                             for l in live)]
            for t in stale:
                self._deleted_uids.append(t.uid)
            tracks[:] = [t for t in tracks if t not in stale]

        # ── NEGATIVE EVIDENCE: mapped objects that are in plain view but no longer detected ──
        # A misdetection disappears as soon as the camera looks at that spot again and sees nothing.
        # A *reliable* (remembered) object gets more benefit of the doubt (UNSEEN_DROP_RELIABLE_S) and
        # is not counted as "not there" if something closer occludes its spot in the depth map, or if
        # it is within BLIND_SPOT_RADIUS of the wearer (below the chest sensors' view, not truly gone).
        matched = {id(m["track"]) for m in measurements}
        all_static = [t for tracks in self._static_tracks.values() for t in tracks]
        view = {}
        if all_static:
            xyz = np.array([(t.x[0], t.x[1], t.z + 0.5 * t.height) for t in all_static])
            vis, us, vs, depths = self._in_view_batch(xyz, pose, K, h)
            view = {id(t): (bool(vis[i]), us[i], vs[i], float(depths[i])) for i, t in enumerate(all_static)}
        fx_px = K[0]
        for label, tracks in self._static_tracks.items():
            survivors = []
            for t in tracks:
                if id(t) not in matched and not near_user(t) and not self._protected(t):
                    in_view, u, v, cam_depth = view[id(t)]
                    half_px = 0.5 * fx_px * t.width / max(cam_depth, 0.1)
                    if in_view and not self._occluded(u, v, cam_depth, dmap, w, h, proj, half_px):
                        t.unseen_in_view += frame_dt
                        t.free_in_view = (t.free_in_view + frame_dt
                                          if label not in SEE_THROUGH
                                          and self._free_space(u, v, cam_depth, t, K, dmap, proj) else 0.0)
                        drop_after = UNSEEN_DROP_RELIABLE_S if t.reliable else UNSEEN_DROP_S
                        if t.unseen_in_view > drop_after or t.free_in_view > FREE_SPACE_CLEAR_S:
                            if t.free_in_view > FREE_SPACE_CLEAR_S:
                                self.get_logger().info(f"🧹 Cleared ghost {label}_{t.id}: "
                                                       f"depth reads >{FREE_SPACE_MARGIN:.1f} m past it")
                            self._deleted_uids.append(t.uid)
                            continue
                    else:
                        t.unseen_in_view = 0.0
                        t.free_in_view = 0.0
                survivors.append(t)
            tracks[:] = survivors

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
                    self._add_track_marker(
                        current_markers, labels, track, final_label, now_msg, lifetime, is_dynamic, marker_frame,
                        distance=away(track.x[0], track.x[1]),
                        seen_ago=None if is_live else now - track.last_seen,
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
                    objects.append({
                        "name": f"{label.replace(' ', '_')}_{t.id}", "class": label, "uid": t.uid,
                        "x": round(float(t.x[0]), 3), "y": round(float(t.x[1]), 3), "z": round(float(t.z), 3),
                        "w": round(float(t.width), 3), "h": round(float(t.height), 3),
                        "vx": round(float(t.x[2]), 3), "vy": round(float(t.x[3]), 3),
                        "dynamic": is_dynamic, "live": live, "seen_ago": round(now - t.last_seen, 1),
                    })
        self._objects_pub.publish(String(data=json.dumps({"frame": frame_id, "objects": objects})))

    @staticmethod
    def _declutter_labels(labels):
        """Stack the text labels of objects standing close together so each one stays readable."""
        placed = []
        for text, _, _ in sorted(labels, key=lambda l: l[0].pose.position.z):
            p = text.pose.position
            for _ in range(20):
                if not any(math.hypot(p.x - q.x, p.y - q.y) < LABEL_CLEAR_XY and abs(p.z - q.z) < LABEL_CLEAR_Z
                           for q in placed):
                    break
                p.z += LABEL_CLEAR_Z
            placed.append(p)

    def _add_track_marker(self, current_markers, labels, track, label_text, now_msg, marker_lifetime, is_dynamic,
                          frame_id, distance=0.0, base_label=None, seen_ago=None):
        px, py = float(track.x[0]), float(track.x[1])
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
            alert_text = f"!! DANGER: {danger_count} CLOSE !!"
            cv2.putText(frame, alert_text, (w - 290, 22),
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

    node = ObjectPerceptionNode(mode=mode)

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
