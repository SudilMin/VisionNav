#!/usr/bin/env python3
"""
outdoor_awareness.py
====================
Outdoor mode for object_perception.py: live hazard awareness from the chest camera, metric depth and the
LiDAR. Nothing is mapped or remembered; every warning comes from what is in front of the wearer right now.

The open-vocabulary detector alone misses much of what matters outdoors (zebra crossings, trees on a footpath,
most potholes), so each hazard also has a detector that does not depend on the class name:

  what                                how
  ─────────────────────────────────── ──────────────────────────────────────────────────────────────────────
  people, vehicles, animals, signs,   YOLOE with OUTDOOR_VOCABULARY (own TensorRT engine), tracked in the
  cones, poles, trees, potholes       body frame with a constant-velocity filter: speed toward the wearer
                                      and time to collision
  anything at chest height in the     LiDAR corridor: nearest return in the walking corridor, and how far
  path (wall, trunk, pole, person)    the left and right sides are clear (for "step left / right")
  anything low in the path (rock,     metric depth -> 3-D points -> ground plane fitted every frame (absorbs
  bollard, kerb up), drops (ditch,    chest sway) -> heights above the ground in the corridor
  kerb down), head-height hazards     head height with free space below it = a branch or a sign to duck
  zebra crossing                      white-stripe pattern on the ground (rows or columns of regular bright
                                      bars), besides the detector's own "zebra crossing"
  traffic / pedestrian light colour   lit, saturated pixels of the light's box (red / yellow / green)

AlertPolicy turns this into short spoken sentences in feet and clock directions ("Pole ahead, 6 feet. Step
right."): one at a time, about the most urgent thing only, without repeating itself or cutting itself off
however many objects are in view. Nothing behind the wearer is tracked or said (FRONT_LIMIT_DEG). Everything
here is plain Python/NumPy (no ROS), so it can be tested on recorded frames.
"""

import math
import os
import time
from collections import deque

import cv2
import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    linear_sum_assignment = None

# ── VOCABULARY (prompts for the outdoor YOLOE engine) ──
OUTDOOR_VOCABULARY = [
    # people and animals
    "person", "child", "person in wheelchair", "baby stroller", "dog", "cat", "cow", "goat",
    # vehicles
    "car", "bus", "truck", "van", "motorcycle", "scooter", "bicycle", "three-wheeler", "auto rickshaw",
    "tuk tuk", "train", "tractor",
    # traffic control
    "traffic light", "pedestrian traffic light", "stop sign", "road sign", "traffic cone", "road barrier",
    "barricade", "bollard", "zebra crossing", "crosswalk", "pedestrian crossing", "belisha beacon", "speed bump",
    # street furniture (plastic chairs and tables outside shops)
    "chair", "plastic chair", "table", "stool", "box", "bag",
    "pole", "lamp post", "electric pole", "sign post", "fire hydrant", "bench", "trash can", "fence", "wall",
    "gate", "railing", "parking meter", "mailbox", "bus stop",
    # nature
    "tree", "tree trunk", "tree branch", "low hanging branch", "bush", "rock", "puddle", "potted plant",
    # the ground
    "pothole", "hole in the ground", "open drain", "manhole", "curb", "stairs", "step", "ramp",
    "construction site", "debris",
]
# Things that look like an object to an open-vocabulary detector but are not one; they win the box and are never
# reported (as the indoor NEGATIVE_PROMPTS; without "planter" a raised flower bed reads as a "hole in the ground").
OUTDOOR_NEGATIVE_PROMPTS = ["sky", "building", "shadow", "road marking", "road", "sidewalk", "grass", "planter",
                            "shop sign", "window"]
OUTDOOR_VOCABULARY = OUTDOOR_VOCABULARY + OUTDOOR_NEGATIVE_PROMPTS
# Several prompts for one thing, reported under one name
OUTDOOR_SYNONYMS = {
    "auto rickshaw": "three-wheeler", "tuk tuk": "three-wheeler", "scooter": "motorcycle",
    "lamp post": "pole", "electric pole": "pole", "sign post": "pole",
    "tree trunk": "tree", "tree branch": "branch", "low hanging branch": "branch",
    "crosswalk": "zebra crossing", "pedestrian crossing": "zebra crossing",
    # the flashing yellow globes on poles at a zebra crossing (read as a yellow "traffic light" otherwise)
    "belisha beacon": "zebra crossing",
    "barricade": "barrier", "road barrier": "barrier", "hole in the ground": "hole",
    "pedestrian traffic light": "walk signal", "person in wheelchair": "wheelchair",
    "baby stroller": "pram", "potted plant": "plant pot", "plastic chair": "chair",
}

# ── CATEGORIES (reported names) ──
PEOPLE = {"person", "child", "wheelchair", "pram"}
ANIMALS = {"dog", "cat", "cow", "goat"}
VEHICLES = {"car", "bus", "truck", "van", "motorcycle", "bicycle", "three-wheeler", "train", "tractor"}
MOVERS = PEOPLE | ANIMALS | VEHICLES
SIGNALS = {"traffic light", "walk signal"}
CROSSINGS = {"zebra crossing"}
# In the ground: needs more warning (and a different sentence) than something to walk around
DROPS = {"pothole", "hole", "open drain", "manhole", "curb", "stairs", "step", "puddle"}
# Hanging at head height over the path: seen late by a cane user, so announced early
OVERHEAD = {"branch"}
# Informational only, never "in the way"
INFO_ONLY = SIGNALS | CROSSINGS | {"road sign", "stop sign", "bus stop", "speed bump", "ramp", "construction site"}
# Standing on the ground (their lowest pixel is a ground contact point)
OUTDOOR_GROUND = (MOVERS | DROPS | CROSSINGS | {
    "traffic cone", "barrier", "bollard", "pole", "fire hydrant", "chair", "table", "stool", "box", "bag", "bench", "trash can", "fence", "wall", "gate",
    "railing", "parking meter", "mailbox", "bus stop", "tree", "bush", "rock", "plant pot", "speed bump", "ramp",
    "construction site", "debris"}) - {"traffic light", "walk signal"}
# Flat in the ground plane: only the box width says anything about distance
OUTDOOR_FLAT = DROPS | CROSSINGS | {"speed bump", "ramp", "debris"}
# Look-alikes: one box read as two of these is one object
OUTDOOR_CONFUSABLE = [
    {"car", "van", "truck", "bus", "three-wheeler", "tractor"},
    {"motorcycle", "bicycle"},
    {"pole", "tree", "bollard"},
    {"traffic light", "walk signal"},
    {"pothole", "hole", "open drain", "manhole", "puddle"},
    {"barrier", "fence", "railing", "gate", "wall"},
    {"person", "child", "wheelchair"},
    {"dog", "cat", "goat"},
]

# ── REAL SIZES (width_m, height_m): typical (the known-size range prior) and maximum ──
OUTDOOR_TYPICAL_SIZES = {
    "child": (0.35, 1.15), "wheelchair": (0.65, 1.30), "pram": (0.55, 1.00), "cow": (0.70, 1.40),
    "goat": (0.35, 0.75), "van": (1.90, 1.95), "three-wheeler": (1.30, 1.70), "train": (3.00, 3.80),
    "tractor": (2.00, 2.50), "walk signal": (0.30, 0.45), "road sign": (0.60, 0.60),
    "traffic cone": (0.35, 0.70), "barrier": (1.50, 1.00), "bollard": (0.20, 0.90), "pole": (0.25, 3.00),
    "fence": (2.00, 1.20), "wall": (3.00, 1.80), "gate": (1.50, 1.60), "railing": (2.00, 1.00),
    "parking meter": (0.25, 1.40), "mailbox": (0.45, 1.20), "bus stop": (3.00, 2.50), "tree": (0.40, 4.00),
    "branch": (1.50, 0.50), "bush": (1.00, 1.00), "rock": (0.40, 0.30), "plant pot": (0.50, 0.60),
    "pothole": (0.60, 0.10), "hole": (0.80, 0.20), "open drain": (0.60, 0.20), "manhole": (0.70, 0.05),
    "curb": (2.50, 0.15), "puddle": (1.00, 0.02), "speed bump": (3.00, 0.10), "ramp": (1.50, 0.20),
    "zebra crossing": (4.00, 0.02), "construction site": (4.00, 2.00), "debris": (0.60, 0.20),
}
OUTDOOR_MAX_SIZES = {k: (3.0 * w, 2.5 * h) for k, (w, h) in OUTDOOR_TYPICAL_SIZES.items()}
OUTDOOR_MAX_SIZES.update({"tree": (2.0, 25.0), "pole": (0.8, 15.0), "wall": (30.0, 6.0), "fence": (30.0, 3.0),
                          "zebra crossing": (20.0, 0.1), "branch": (6.0, 3.0), "construction site": (30.0, 6.0)})

# ── GEOMETRY ──
USER_HEIGHT = float(os.environ.get("WEARABLE_USER_HEIGHT", "1.75"))  # m, the wearer's height (head clearance)
CORRIDOR_HALF = float(os.environ.get("WEARABLE_CORRIDOR_HALF", "0.45"))  # m: shoulders plus a margin, each side
LANE_WIDTH = 0.9             # m, the lane beside the corridor that "step left / right" leads into
LOOK_AHEAD = 8.0             # m of path watched for obstacles
LIDAR_MIN_X = 0.40           # m: closer returns in front are the wearer's own arms / chest mount

# ── GROUND ANALYSIS (metric depth) ──
GROUND_STRIDE = 8            # px between sampled depth pixels
GROUND_RANSAC_ITERS = 80
GROUND_INLIER_M = 0.06       # m from the plane counts as ground
GROUND_MIN_INLIERS = 0.30    # share of the candidate ground points the fitted plane must explain
GROUND_MAX_TILT_DEG = 15.0   # a "ground" plane steeper than this is a wall or a car side, not the ground
GROUND_MAX_OFFSET = 0.30     # m: the plane must pass this close to the floor under the wearer (else it is a seat,
                             # a table top or a car bonnet, and the real floor below it would read as a hole)
GROUND_MAX_RANGE = 10.0      # m: depth beyond this is too coarse for heights
BIN_M = 0.25                 # m, corridor bins along the walking direction
OBSTACLE_MIN_H = 0.20        # m above the ground: something to walk into (a kerb up is ~0.15: only near)
DROP_MIN_H = 0.25            # m below the ground plane: a hole, ditch or kerb down
HEAD_LOW = 1.20              # m: an overhead hazard's lowest point is between this...
HEAD_HIGH = USER_HEIGHT + 0.15  # ...and just above the wearer's head
BIN_MIN_PTS = 4              # sampled points in a bin before it counts (one noisy pixel is not an obstacle)
BIN_MIN_SHARE = 0.25         # ...and at least this share of the bin's points
DROP_MAX_RANGE = 5.0         # m: holes are only trusted this close (depth errors grow with range)

# ── TRACKING (body frame, everything moves relative to a walking wearer) ──
TRACK_ACCEL = {"vehicle": 8.0, "mover": 3.0, "static": 1.5}  # m^2/s^3 white-noise acceleration
TRACK_MAX_GAP_S = 0.8        # s a track survives unseen
TRACK_CONFIRM_HITS = 3       # camera sightings before it is reported (kills one- and two-frame hallucinations)
# What the camera named is kept only while the camera keeps seeing it (nearby LiDAR returns alone do not keep it)
CAM_KEEP_IN_VIEW_S = 1.0     # s a named object in the camera's view lasts without the camera seeing it
CAM_KEEP_BESIDE_S = 4.0      # s once out of the camera's view (beside the wearer; the LiDAR still warns, unnamed)
DROP_CONFIRM_HITS = 3        # ground hazards are noisier (texture): one more sighting
ANIMAL_CONFIRM_HITS = 5      # animals are often hallucinated in dark texture (a chair's weave read as a "cat")...
ANIMAL_MIN_CONF = 0.5        # ...so they need more sightings and a higher mean score
HAZARD_CONFIRM = (3, 5)      # a drop / head-height hazard from depth counts once seen in 3 of the last 5 frames...
HAZARD_CONFIRM_M = 0.5       # ...at about the same distance
TRACK_GATE_M = {"vehicle": 4.0, "mover": 1.5, "static": 1.2}  # m association gate (+ speed * dt)
SPEED_MIN_AGE_S = 0.5        # s tracked before its speed is trusted
APPROACH_MIN_HITS = 5        # sightings before anything is said to be coming toward the wearer
LIDAR_CONFIRM_HITS = 4       # scans before an unnamed LiDAR object counts (it is never drawn or named, only
                             # used for "something coming on your left / right")
CLUSTER_GATE_M = {"vehicle": 2.5, "mover": 0.8, "static": 0.6}  # m, LiDAR cluster to object

# ── ALERTS ──
CRITICAL, WARNING, INFO = 3, 2, 1
VEHICLE_TTC_CRITICAL = 3.0   # s
VEHICLE_TTC_WARNING = 6.0
VEHICLE_APPROACH_MPS = 1.5   # m/s toward the wearer faster than walking into it: the vehicle is moving
OBSTACLE_CRITICAL_M = 1.0    # m, something in the path this close: stop
OBSTACLE_WARNING_M = 2.0
DROP_CRITICAL_M = 1.8
DROP_WARNING_M = 4.5
OVERHEAD_CRITICAL_M = 1.5
OVERHEAD_WARNING_M = 3.5
PERSON_WARNING_M = 2.0       # a person in the path this close (and not walking away)
CAMERA_HALF_FOV_DEG = 26.0   # beyond this bearing only the LiDAR sees an object (camera: 52 deg wide)
FRONT_LIMIT_DEG = 100.0      # farther round than this from straight ahead is behind the wearer: not tracked, drawn
                             # or said
SIDE_MIN_SPEED = 1.2         # m/s of its own: something coming from the side (needs odometry)
SIDE_TTC = 4.0               # s
SIDE_RANGE = 10.0            # m
SIDE_MIN_HITS = 12           # LiDAR sightings, and...
SIDE_MIN_AGE_S = 1.5         # ...seconds tracked, before something unseen by the camera is said to be coming
SIDE_TURN_RATE = math.radians(30.0)  # rad/s: turning faster than this, the odometry slips and walls beside the
SIDE_TURN_HOLD_S = 1.5       # wearer seem to move: no "coming" alerts while turning and for this long after
ANIMAL_WARNING_M = 3.0
STOP_ADVICE_M = 1.0          # m: "Stop." (no free side to step to) is only said this close
NAME_LATERAL_M = 0.5         # m: a detection names what blocks the path only if it is on the same side
# Speaking: one sentence at a time about the most urgent thing. The same thing is said again only when it becomes
# more urgent (a warning turning into a danger) or after REPEAT_S, never just because it came a little closer.
REPEAT_S = {CRITICAL: 6.0, WARNING: 15.0, INFO: 30.0}  # s before the same thing is said again at the same level,
REPEAT_MAX_S = 30.0          # ...doubling each time while nothing has changed (standing in front of a table)
FARTHER_M = 1.0              # m: what is in the path is this much farther than what was announced = another thing
CHANGE_S = 5.0               # s before another thing in the same place is announced
STICK_M = 0.5                # m: of several things in the path, the one announced stays the one spoken about
                             # until another is this much nearer
PAUSE_S = {WARNING: 2.5, INFO: 5.0}  # s of silence after a sentence before the next warning / information
PATH_CLEAR_AFTER_S = 4.0     # s the path must stay clear before "Path clear" (after a blocking warning)
GENERIC = "obstacle"         # what the LiDAR / depth found but the camera did not name
BLOCKING_SLOTS = ("path", "vehicle")

UNITS = os.environ.get("WEARABLE_UNITS", "feet").lower()  # "feet" (VisionNav default) or "metric"


# ══════════════════════════════════════════════════════════════════════
# ── SPOKEN LANGUAGE ──
# ══════════════════════════════════════════════════════════════════════
def say_distance(d: float) -> str:
    """"6 feet" / "12 feet" / "35 feet" (or metres with WEARABLE_UNITS=metric)."""
    if UNITS.startswith("m"):
        if d < 10:
            v = max(0.5, round(d * 2) / 2)
            return f"{v:g} meter" + ("" if v == 1 else "s")
        return f"{int(round(d))} meters"
    ft = d * 3.28084
    ft = max(1, int(round(ft))) if ft < 20 else int(round(ft / 5.0)) * 5
    return f"{ft} foot" if ft == 1 else f"{ft} feet"


def clock(x: float, y: float) -> int:
    """Clock direction of a point in the body frame (x forward, y left): 12 ahead, 3 right, 9 left."""
    ang = math.atan2(y, x)  # + left
    return int(round(12 - ang * 6 / math.pi)) % 12 or 12


def where(x: float, y: float) -> str:
    """"ahead" / "on your left" / "at 2 o'clock"."""
    c = clock(x, y)
    if c in (12,):
        return "ahead"
    if c in (11, 1):
        return f"slightly {'left' if c == 11 else 'right'}"
    if c in (9, 10):
        return "on your left"
    if c in (2, 3):
        return "on your right"
    return f"at {c} o'clock"


def side_of(y: float) -> str:
    return "left" if y > 0 else "right"


def spoken_class(label: str) -> str:
    return {"curb": "kerb", "walk signal": "pedestrian signal"}.get(label, label)


def behind(x: float, y: float) -> bool:
    """Behind the wearer (body frame, x forward, y left): past FRONT_LIMIT_DEG from straight ahead."""
    return abs(math.degrees(math.atan2(y, x))) > FRONT_LIMIT_DEG


def say_time(text: str) -> float:
    """Seconds the assistant takes to say a sentence (Piper on the laptop: 2.5 s for "Chair ahead, 1 foot. Step
    right.", 0.9 s for "Path clear.")."""
    return 0.4 + len(text) / 14.0


# ══════════════════════════════════════════════════════════════════════
# ── SIGNAL COLOUR ──
# ══════════════════════════════════════════════════════════════════════
def signal_color(bgr: np.ndarray, box, mask=None):
    """Colour of the lit lamp of a traffic or pedestrian light: "red", "yellow", "green" or None.

    A lit lamp is the brightest, strongly saturated part of the housing. Red is also the upper lamp and green
    the lower one, which settles a close vote (a red lamp can bloom orange).
    """
    x1, y1, x2, y2 = [int(v) for v in box]
    H, W = bgr.shape[:2]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
    if x2 - x1 < 3 or y2 - y1 < 5:
        return None
    hsv = cv2.cvtColor(bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0].astype(int), hsv[..., 1].astype(int), hsv[..., 2].astype(int)
    vmax = int(np.percentile(val, 99))
    lit = (val >= max(110, int(0.70 * vmax))) & (sat >= 80)
    if mask is not None:
        sub = mask[y1:y2, x1:x2] > 0
        if sub.shape == lit.shape and sub.any():
            # the mask often leaves out the glowing lamp itself (its halo is outside the housing): grow it
            lit &= cv2.dilate(sub.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    n = int(lit.sum())
    if n < max(4, int(0.004 * lit.size)):
        return None
    h = hue[lit]
    rows = np.nonzero(lit)[0] / max(1, lit.shape[0] - 1)  # 0 top .. 1 bottom
    votes = {
        "red": float(np.sum((h <= 8) | (h >= 165))),
        "yellow": float(np.sum((h > 12) & (h <= 34))),
        "green": float(np.sum((h >= 45) & (h <= 100))),
    }
    # Position: red lamps sit in the upper part, green in the lower part
    for name, sel in (("red", (h <= 8) | (h >= 165)), ("green", (h >= 45) & (h <= 100))):
        if sel.any():
            pos = float(np.median(rows[sel]))
            votes[name] *= 1.3 if (name == "red" and pos < 0.5) or (name == "green" and pos > 0.5) else 1.0
    best = max(votes, key=votes.get)
    total = sum(votes.values())
    if total < 4 or votes[best] < 0.55 * total:
        return None
    return best


# ══════════════════════════════════════════════════════════════════════
# ── ZEBRA CROSSING (stripe pattern) ──
# ══════════════════════════════════════════════════════════════════════
ZEBRA_MIN_ELONGATION = 2.5   # a painted bar is at least this many times longer than thick
ZEBRA_MIN_FILL = 0.55        # share of its rotated bounding box the bar fills (solid paint, not a scribble)
ZEBRA_MIN_BARS = 3
ZEBRA_MIN_TOTAL_LEN = 0.6    # summed bar length, in image widths


def _paint_mask(bgr: np.ndarray) -> np.ndarray:
    """White road paint: much brighter than the asphalt around it (local top-hat), bright overall and
    unsaturated (a yellow centre line touching the bars would merge them into one blob)."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    sat = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[..., 1]
    k = max(15, (gray.shape[1] // 16) | 1)
    background = cv2.blur(gray, (k, k))
    paint = ((gray.astype(np.int16) - background) > 25) & (gray > np.percentile(gray, 55)) & (sat < 70)
    return cv2.morphologyEx(paint.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def find_zebra_crossing(bgr: np.ndarray, horizon_v: int):
    """Box (x1, y1, x2, y2) of a zebra crossing on the ground below `horizon_v`, or None.

    White road paint is much brighter than the asphalt next to it. A crossing is >= 3 solid, elongated painted
    bars lying side by side: similar direction and length, offset across their length (not along it). Seen
    from the kerb the bars run away from the wearer and converge; seen side-on they are stacked horizontal
    bars — both are side by side. A dashed lane line (bars end to end), a single stop line and paving tiles
    (no solid bright bars) are not. Windows and siding above the ground are left to the caller to reject
    (the node checks the box lies on the fitted ground).
    """
    H, W = bgr.shape[:2]
    top = int(max(0, min(H - 20, horizon_v)))
    if H - top < 20:
        return None
    paint = _paint_mask(bgr[top:])
    contours, _ = cv2.findContours(paint, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(40.0, 0.00015 * H * W)
    bars = []  # (centre, unit direction, length, thickness, contour)
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(c)
        length, thick = max(rw, rh), max(1.0, min(rw, rh))
        if length / thick < ZEBRA_MIN_ELONGATION or area < ZEBRA_MIN_FILL * rw * rh:
            continue
        a = math.radians(ang if rw >= rh else ang + 90.0)
        bars.append((np.array([cx, cy]), np.array([math.cos(a), math.sin(a)]), length, thick, c))
    if len(bars) < ZEBRA_MIN_BARS:
        return None
    parent = list(range(len(bars)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(bars)):
        ci, ui, li, ti, _ = bars[i]
        for j in range(i + 1, len(bars)):
            cj, uj, lj, tj, _ = bars[j]
            if abs(float(ui @ uj)) < math.cos(math.radians(20)) or max(li, lj) > 2.5 * min(li, lj):
                continue
            u = ui if float(ui @ uj) >= 0 else -ui
            u = (u + uj) / np.linalg.norm(u + uj)
            d = cj - ci
            along, perp = abs(float(d @ u)), abs(float(d[0] * -u[1] + d[1] * u[0]))
            thick = max(ti, tj)
            # side by side (a dashed lane line's bars are end to end: no sideways offset); seen obliquely the
            # bars of a crossing are also shifted along their length, and one may be hidden by a person
            if 0.8 * thick <= perp <= 6.0 * thick + 6 and along <= max(li, lj):
                parent[root(i)] = root(j)
    groups = {}
    for i in range(len(bars)):
        groups.setdefault(root(i), []).append(i)
    best = max(groups.values(), key=len)
    # A crossing near enough to matter spans much of the view (real ones: 6-7 bars over 1.2-1.6 image widths;
    # paving slabs, siding and kerb paint: 3-4 bars under 0.3)
    if len(best) < ZEBRA_MIN_BARS + 1 or sum(bars[i][2] for i in best) < ZEBRA_MIN_TOTAL_LEN * W:
        return None
    pts = np.vstack([bars[i][4].reshape(-1, 2) for i in best])
    x1, y1 = pts.min(axis=0)
    x2, y2 = pts.max(axis=0)
    return int(x1), int(y1) + top, int(x2), int(y2) + top


# ══════════════════════════════════════════════════════════════════════
# ── GROUND ANALYSIS (metric depth) ──
# ══════════════════════════════════════════════════════════════════════
class GroundAnalysis:
    """Result of one frame: the fitted ground and what sticks up from / down into it along the path."""
    def __init__(self):
        self.ok = False
        self.plane = None          # (n, d): n . p + d = height above the ground, base_footprint
        self.fitted = False        # the plane came from this frame's depth (else the nominal floor z = 0)
        self.obstacle = None       # (distance, lateral y, top height) of the nearest thing in the path
        self.drop = None           # (distance, lateral y, depth below ground) of the nearest hole / drop
        self.overhead = None       # (distance, lateral y, lowest height) of the nearest head-height hazard
        self.lanes = {}            # "left"/"center"/"right" -> free distance (m) along that lane
        self.visible_from = None   # m: nearest ground distance the camera sees (closer is below its view)
        self.points = None         # (u, v, height, x, y) of the sampled pixels, for the HUD


def fit_ground(P: np.ndarray, rng=None):
    """RANSAC plane through near-floor points (N,3 base_footprint). Returns (n, d) with n up, or None."""
    if P.shape[0] < 30:
        return None
    rng = rng or np.random.default_rng(0)
    best, best_n = None, 0
    cos_tilt = math.cos(math.radians(GROUND_MAX_TILT_DEG))
    for _ in range(GROUND_RANSAC_ITERS):
        a, b, c = P[rng.choice(P.shape[0], 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-6:
            continue
        n = n / norm
        if n[2] < 0:
            n = -n
        if n[2] < cos_tilt:
            continue
        d = -float(n @ a)
        cnt = int(np.sum(np.abs(P @ n + d) < GROUND_INLIER_M))
        if cnt > best_n:
            best, best_n = (n, d), cnt
    if best is None or best_n < GROUND_MIN_INLIERS * P.shape[0]:
        return None
    # Refine: least squares through the inliers
    n, d = best
    inl = P[np.abs(P @ n + d) < GROUND_INLIER_M]
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c)
    n = vt[2] if vt[2][2] > 0 else -vt[2]
    if n[2] < cos_tilt:
        return best
    return n, -float(n @ c)


def analyze_ground(dmap: np.ndarray, K, cam_R: np.ndarray, cam_t: np.ndarray, scale: float = 1.0,
                   exclude_boxes=()) -> GroundAnalysis:
    """Heights above the ground of the depth map's pixels, and the nearest obstacle / drop / overhead
    hazard in the walking corridor. `exclude_boxes`: image boxes of detected movers (people, vehicles),
    handled by the tracker, so a person walking beside the wearer is not also a nameless "obstacle"."""
    res = GroundAnalysis()
    if dmap is None:
        return res
    fx, fy, cx, cy, _ = K
    H, W = dmap.shape
    vs, us = np.mgrid[GROUND_STRIDE // 2:H:GROUND_STRIDE, GROUND_STRIDE // 2:W:GROUND_STRIDE]
    us, vs = us.ravel(), vs.ravel()
    z = dmap[vs, us].astype(np.float64) * scale
    ok = (z > 0.3) & (z < GROUND_MAX_RANGE)
    us, vs, z = us[ok], vs[ok], z[ok]
    if z.size < 50:
        return res
    p_opt = np.stack([(us - cx) / fx * z, (vs - cy) / fy * z, z], axis=1)
    P = p_opt @ cam_R.T + cam_t  # base_footprint: x forward, y left, z up
    # Ground candidates: the lower part of the view (mostly ground for a forward-looking chest camera), in front,
    # up to 12 m, and below the camera. Not "near z = 0": a wrong pitch in TF (or chest sway) tilts the whole
    # cloud, and the fitted plane is what corrects it.
    cand = (vs > 0.55 * H) & (P[:, 0] > 0.5) & (P[:, 0] < 12.0) & (P[:, 2] < cam_t[2] - 0.5)
    plane = fit_ground(P[cand]) if cand.sum() >= 30 else None
    if plane is not None and abs((plane[1] + plane[0][0] * cam_t[0] + plane[0][1] * cam_t[1]) / plane[0][2]) \
            > GROUND_MAX_OFFSET:
        plane = None  # not the floor the wearer stands on
    res.fitted = plane is not None
    if plane is None:
        # No ground in view (a wall or a car side, or a pitch far from TF): heights against the nominal floor are
        # mostly noise, so nothing is reported from depth this frame
        return res
    n, d = plane
    res.plane = plane
    hgt = P @ n + d
    x, y = P[:, 0], P[:, 1]
    res.points = (us, vs, hgt, x, y)
    ground = np.abs(hgt) < GROUND_INLIER_M * 2
    if ground.any():
        res.visible_from = float(np.percentile(x[ground], 2))
    if exclude_boxes:
        keep = np.ones(us.size, dtype=bool)
        for (bx1, by1, bx2, by2) in exclude_boxes:
            keep &= ~((us >= bx1) & (us <= bx2) & (vs >= by1) & (vs <= by2))
        x, y, hgt = x[keep], y[keep], hgt[keep]

    def nearest(sel, in_lane, kind):
        """Nearest bin of the lane where `sel` points are dense enough: (dist, y, value) or None."""
        xs = x[sel]
        if xs.size < BIN_MIN_PTS:
            return None
        bins = np.floor(xs / BIN_M).astype(int)
        allb = np.floor(x[in_lane] / BIN_M).astype(int)
        total = np.bincount(allb[allb >= 0], minlength=int(LOOK_AHEAD / BIN_M) + 2)
        cnt = np.bincount(bins[bins >= 0], minlength=total.size)
        for b in range(min(cnt.size, total.size)):
            if cnt[b] >= BIN_MIN_PTS and cnt[b] >= BIN_MIN_SHARE * max(1, total[b]):
                pick = sel & (np.floor(x / BIN_M).astype(int) == b)
                if kind == "drop":
                    val = float(np.percentile(hgt[pick], 20))
                elif kind == "overhead":
                    val = float(np.percentile(hgt[pick], 10))
                else:
                    val = float(np.percentile(hgt[pick], 90))
                return float(np.percentile(x[pick], 10)), float(np.median(y[pick])), val
        return None

    lanes = {"center": (-CORRIDOR_HALF, CORRIDOR_HALF),
             "left": (CORRIDOR_HALF, CORRIDOR_HALF + LANE_WIDTH),
             "right": (-CORRIDOR_HALF - LANE_WIDTH, -CORRIDOR_HALF)}
    for name, (lo, hi) in lanes.items():
        in_lane = (y >= lo) & (y <= hi) & (x > 0.3) & (x < LOOK_AHEAD)
        body = in_lane & (hgt > OBSTACLE_MIN_H) & (hgt < HEAD_LOW)
        hit = nearest(body, in_lane, "obstacle")
        res.lanes[name] = hit[0] if hit is not None else LOOK_AHEAD
        if name != "center":
            continue
        res.obstacle = hit
        # A drop: points clearly below the ground, close enough for depth to be trusted
        res.drop = nearest(in_lane & (hgt < -DROP_MIN_H) & (x < DROP_MAX_RANGE), in_lane, "drop")
        # Overhead: something at head height with free space below it in the same bin
        head = in_lane & (hgt >= HEAD_LOW) & (hgt <= HEAD_HIGH)
        hit = nearest(head, in_lane, "overhead")
        if hit is not None and (res.obstacle is None or res.obstacle[0] > hit[0] + 0.5):
            res.overhead = hit
    res.ok = True
    return res


class HazardConfirm:
    """Depth's drops and head-height hazards count only once seen in HAZARD_CONFIRM[0] of the last
    HAZARD_CONFIRM[1] frames at about the same distance: a single frame's depth error is never said."""

    def __init__(self):
        k, n = HAZARD_CONFIRM
        self._need = k
        self._hist = {"drop": deque(maxlen=n), "overhead": deque(maxlen=n)}

    def apply(self, ground):
        """Clear ground.drop / ground.overhead that are not confirmed yet. Returns `ground`."""
        for kind, hist in self._hist.items():
            hit = getattr(ground, kind, None) if ground is not None and ground.ok else None
            hist.append(None if hit is None else hit[0])
            if hit is not None and sum(1 for d in hist if d is not None
                                       and abs(d - hit[0]) <= HAZARD_CONFIRM_M) < self._need:
                setattr(ground, kind, None)
        return ground

    def reset(self):
        for hist in self._hist.values():
            hist.clear()


# ══════════════════════════════════════════════════════════════════════
# ── LIDAR CORRIDOR ──
# ══════════════════════════════════════════════════════════════════════
def lidar_corridor(xy: np.ndarray):
    """Free distance (m) along the path and the lanes beside it, from LiDAR returns in base_footprint (N,2).

    Returns {"center": d, "left": d, "right": d, "center_y": y of the nearest return in the path} with
    LOOK_AHEAD for a clear lane."""
    out = {"center": LOOK_AHEAD, "left": LOOK_AHEAD, "right": LOOK_AHEAD, "center_y": 0.0}
    if xy is None or len(xy) == 0:
        return out
    x, y = xy[:, 0], xy[:, 1]
    front = (x > LIDAR_MIN_X) & (x < LOOK_AHEAD)
    for name, (lo, hi) in (("center", (-CORRIDOR_HALF, CORRIDOR_HALF)),
                           ("left", (CORRIDOR_HALF, CORRIDOR_HALF + LANE_WIDTH)),
                           ("right", (-CORRIDOR_HALF - LANE_WIDTH, -CORRIDOR_HALF))):
        sel = front & (y >= lo) & (y <= hi)
        if int(sel.sum()) >= 2:  # a single return is noise (rain, an insect)
            xs = np.sort(x[sel])
            out[name] = float(xs[1])
            if name == "center":
                out["center_y"] = float(np.median(y[sel][np.argsort(x[sel])[:3]]))
    return out


# ══════════════════════════════════════════════════════════════════════
# ── LIDAR OBJECTS (ahead and beside the wearer) ──
# ══════════════════════════════════════════════════════════════════════
CLUSTER_GAP = 0.20           # m (+ CLUSTER_GAP_PER_M x range) between neighbouring returns of one object
CLUSTER_GAP_PER_M = 0.03
CLUSTER_MIN_PTS = 3
CLUSTER_MAX_NEW = 1.2        # m: a cluster this compact may start an (unnamed) object: a person, pole, trunk, bike
CLUSTER_MAX_LEN = 5.5        # m: longer is structure (a wall, a fence, a hedge): occupancy, never an object


def lidar_clusters(xy: np.ndarray):
    """Objects in one scan (base_footprint points in scan order): a run of returns with no gap between them.
    Returns measurements like the camera's, but unnamed (label None): centre, extent and direction. Only ahead
    of and beside the wearer: what is behind them is not an object here."""
    if xy is None or len(xy) < CLUSTER_MIN_PTS:
        return []
    r = np.hypot(xy[:, 0], xy[:, 1])
    gaps = np.hypot(*np.diff(xy, axis=0).T)
    breaks = np.nonzero(gaps > CLUSTER_GAP + CLUSTER_GAP_PER_M * r[1:])[0] + 1
    out = []
    for seg in np.split(np.arange(len(xy)), breaks):
        if len(seg) < CLUSTER_MIN_PTS:
            continue
        p = xy[seg]
        c = p.mean(axis=0)
        if len(p) >= 3:
            ev, V = np.linalg.eigh(np.cov((p - c).T) + 1e-9 * np.eye(2))
            axis = V[:, 1]
            length = float(np.ptp((p - c) @ axis)) + 0.05
            width = float(np.ptp((p - c) @ V[:, 0])) + 0.05
        else:
            axis, length, width = np.array([1.0, 0.0]), 0.1, 0.1
        if length > CLUSTER_MAX_LEN or behind(c[0], c[1]):
            continue
        # The LiDAR sees the near face: the centre is about half a thickness behind it (small objects)
        dist = float(np.hypot(*c))
        centre = c + (c / max(dist, 1e-6)) * min(0.15, 0.5 * width)
        out.append({"label": None, "raw_label": None, "bx": float(centre[0]), "by": float(centre[1]),
                    "sigma": 0.06 + 0.01 * dist, "box": None, "width_m": max(length, 0.1), "height_m": None,
                    "z": 0.0, "conf": 0.5, "source": "lidar", "cluster": True,
                    "yaw": float(math.atan2(axis[1], axis[0])), "n": len(p), "compact": length <= CLUSTER_MAX_NEW})
    return out


# ══════════════════════════════════════════════════════════════════════
# ── LIVE OCCUPANCY (LiDAR + depth, the last few seconds) ──
# ══════════════════════════════════════════════════════════════════════
OCC_RES = 0.10               # m per cell
OCC_CELLS = 300              # 30 x 30 m around the wearer
OCC_RECENTER_M = 3.0         # the grid is shifted (whole cells) once the wearer is this far from its centre
OCC_HALF_LIFE_S = 2.0        # evidence halves this fast: the grid shows the last few seconds, nothing older
OCC_L_OCC, OCC_L_FREE = 0.9, -0.35
OCC_L_MIN, OCC_L_MAX = -2.0, 3.5
OCC_SHOW = 1.0               # log-odds above which a cell is drawn occupied (~0.73 probability)
OCC_FREE_SHOW = -0.8         # ...below which it is walkable free space


class LocalOccupancy:
    """What is occupied and free around the wearer, from the last few seconds of LiDAR (ahead and beside, chest
    height) and depth (ahead: low obstacles and drops). World-fixed while the wearer's motion is known, so a wall
    stays put while they walk; without odometry, just the latest scan in the body frame. It forgets on its own
    (OCC_HALF_LIFE_S) and is never saved: a live picture, not a map."""

    def __init__(self):
        self.L = np.zeros((OCC_CELLS, OCC_CELLS), np.float32)      # LiDAR (chest height)
        self.low = np.zeros_like(self.L)                          # depth: low obstacle ahead
        self.drop = np.zeros_like(self.L)                         # depth: hole / drop ahead
        self.origin = None   # world (x, y) of cell (0, 0)'s corner
        self.t = None
        self.world = False

    def reset(self):
        self.__init__()

    def _index(self, xy):
        ij = np.floor((xy - self.origin) / OCC_RES).astype(np.int64)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < OCC_CELLS) & (ij[:, 1] >= 0) & (ij[:, 1] < OCC_CELLS)
        return ij[ok, 0] * OCC_CELLS + ij[ok, 1]

    def _prepare(self, ego, t):
        world = ego is not None
        if world != self.world:
            self.reset()
            self.world = world
        centre = np.asarray(ego[:2]) if world else np.zeros(2)
        if self.origin is None or not world:
            self.origin = centre - OCC_CELLS * OCC_RES / 2
            if not world:
                self.L[:] = 0.0  # no motion known: only the latest scan is meaningful
        elif np.abs(centre - (self.origin + OCC_CELLS * OCC_RES / 2)).max() > OCC_RECENTER_M:
            shift = np.round((centre - (self.origin + OCC_CELLS * OCC_RES / 2)) / OCC_RES).astype(int)
            for layer in (self.L, self.low, self.drop):
                moved = np.zeros_like(layer)
                sx, sy = shift
                src = layer[max(0, sx):OCC_CELLS + min(0, sx), max(0, sy):OCC_CELLS + min(0, sy)]
                moved[max(0, -sx):max(0, -sx) + src.shape[0], max(0, -sy):max(0, -sy) + src.shape[1]] = src
                layer[:] = moved
            self.origin = self.origin + shift * OCC_RES
        if self.t is not None and t > self.t:
            f = np.float32(0.5 ** ((t - self.t) / OCC_HALF_LIFE_S))
            self.L *= f
            self.low *= f
            self.drop *= f
        self.t = t if self.t is None else max(self.t, t)

    def add_scan(self, ego, xy_body, t, sensor_xy=(0.0, 0.0)):
        """One LiDAR scan (base_footprint points), `ego` the wearer's pose (odom) at the scan, or None."""
        self._prepare(ego, t)
        if xy_body is None or len(xy_body) == 0:
            return
        pose = ego if ego is not None else (0.0, 0.0, 0.0)
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        R = np.array([[c, -s_], [s_, c]])
        pts = xy_body @ R.T + np.asarray(pose[:2])
        o = np.asarray(sensor_xy) @ R.T + np.asarray(pose[:2])
        # Free along each ray (sampled every cell), up to just before the return
        d = pts - o
        rng = np.hypot(d[:, 0], d[:, 1])
        n = np.maximum(0, ((rng - 0.15) / OCC_RES).astype(int))
        if n.sum() > 0:
            ray = np.repeat(np.arange(len(pts)), n)
            k = np.concatenate([np.arange(v) for v in n]) if len(n) else np.zeros(0, int)
            frac = (k * OCC_RES) / np.maximum(rng[ray], 1e-6)
            free = np.unique(self._index(o + d[ray] * frac[:, None]))
            self.L.flat[free] += OCC_L_FREE
        occ = np.unique(self._index(pts))
        self.L.flat[occ] += OCC_L_OCC + (-OCC_L_FREE)  # undo the free step a cell may have got from another ray
        np.clip(self.L, OCC_L_MIN, OCC_L_MAX, out=self.L)

    def add_ground(self, ego, obstacle_xy, drop_xy, t):
        """Depth's low obstacles and drops ahead (base_footprint points at the frame's time)."""
        self._prepare(ego, t)
        pose = ego if ego is not None else (0.0, 0.0, 0.0)
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        R = np.array([[c, -s_], [s_, c]])
        for layer, xy in ((self.low, obstacle_xy), (self.drop, drop_xy)):
            if xy is not None and len(xy):
                idx = np.unique(self._index(xy @ R.T + np.asarray(pose[:2])))
                layer.flat[idx] = np.minimum(layer.flat[idx] + 1.0, 3.0)

    def cells(self, ego, radius=12.0, free_radius=6.0):
        """Body-frame centres of cells to draw: occupied (LiDAR), low obstacles, drops and walkable free."""
        if self.origin is None:
            return {}
        pose = ego if (ego is not None and self.world) else (0.0, 0.0, 0.0)
        c, s_ = math.cos(pose[2]), math.sin(pose[2])
        Rt = np.array([[c, s_], [-s_, c]])
        out = {}
        for name, mask, rad in (("occupied", self.L > OCC_SHOW, radius), ("low", self.low > 1.5, radius),
                                ("drop", self.drop > 1.5, radius), ("free", self.L < OCC_FREE_SHOW, free_radius)):
            ij = np.argwhere(mask)
            if len(ij) == 0:
                out[name] = np.zeros((0, 2))
                continue
            w = self.origin + (ij + 0.5) * OCC_RES
            b = (w - np.asarray(pose[:2])) @ Rt.T
            out[name] = b[np.hypot(b[:, 0], b[:, 1]) < rad]
        return out


# ══════════════════════════════════════════════════════════════════════
# ── TRACKING (world frame when the wearer's motion is known) ──
# ══════════════════════════════════════════════════════════════════════
def kind_of(label) -> str:
    if label is None:
        return "mover"   # unnamed LiDAR object: may be a person or a bike
    return "vehicle" if label in VEHICLES else "mover" if label in MOVERS else "static"


def _to_world(ego, x, y):
    c, s_ = math.cos(ego[2]), math.sin(ego[2])
    return ego[0] + c * x - s_ * y, ego[1] + s_ * x + c * y


class OutdoorTrack:
    """Constant-velocity Kalman filter [x, y, vx, vy].

    With the wearer's motion known (LiDAR odometry) it runs in the world-fixed odom frame, so a pole or a
    parked car is still and a car's speed is its own. Without it, in the body frame (everything moves toward a
    walking wearer). Either way, set_view() gives `x` / `P` as seen from the wearer now: body-frame position and
    velocity relative to the wearer, which is what the hazard rules use.

    Fed by the camera (named: "car") and by LiDAR clusters (unnamed, ahead and beside). A LiDAR object takes the
    name of the camera detection it is matched with and keeps it while the LiDAR still sees it beside the wearer:
    still live, not remembered. Once behind them it is dropped."""
    _next_id = 1

    def __init__(self, m, now, ego=None):
        self.id = OutdoorTrack._next_id
        OutdoorTrack._next_id += 1
        self.label = m["label"]
        wx, wy = _to_world(ego, m["bx"], m["by"]) if ego is not None else (m["bx"], m["by"])
        self.xw = np.array([wx, wy, 0.0, 0.0])
        s2 = max(m["sigma"], 0.1) ** 2
        v0 = 4.0 if self.kind == "vehicle" else 1.0
        self.Pw = np.diag([s2, s2, v0, v0])
        self.first_seen = self.last_seen = self.t = now
        self.hits = 1
        self.cam_hits = 0 if m.get("cluster") else 1
        self.conf_sum = 0.0 if m.get("cluster") else float(m.get("conf", 0.0))  # summed camera scores
        self.last_cam = -math.inf if m.get("cluster") else now
        self.m = m                  # latest camera measurement (box, sizes, source); a cluster's until one comes
        self.cm = m if m.get("cluster") else None  # latest LiDAR cluster
        self.color = None           # signal colour (traffic / pedestrian lights), voted
        self.color_votes = {}
        self.world = ego is not None
        self.x, self.P = self.xw.copy(), self.Pw.copy()

    @property
    def kind(self):
        return kind_of(self.label)

    @property
    def confirmed(self):
        if self.label is None:
            return self.hits >= LIDAR_CONFIRM_HITS
        if self.label in ANIMALS:
            return self.cam_hits >= ANIMAL_CONFIRM_HITS and self.conf_sum / self.cam_hits >= ANIMAL_MIN_CONF
        need = DROP_CONFIRM_HITS if self.label in DROPS or self.label in CROSSINGS else TRACK_CONFIRM_HITS
        return self.cam_hits >= need

    def predict(self, now):
        dt = min(0.5, max(0.0, now - self.t))
        if dt <= 0:
            return
        self.t = now
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = TRACK_ACCEL[self.kind] if self.world or self.kind != "static" else TRACK_ACCEL["mover"]
        if self.world and self.kind == "static":
            q = 0.05  # a pole does not move: only the odometry's small error
        Q = np.zeros((4, 4))
        for p, v in ((0, 2), (1, 3)):
            Q[p, p], Q[p, v], Q[v, p], Q[v, v] = q * dt ** 3 / 3, q * dt ** 2 / 2, q * dt ** 2 / 2, q * dt
        self.xw = F @ self.xw
        self.Pw = F @ self.Pw @ F.T + Q

    def update(self, m, now, ego=None):
        self.predict(now)
        wx, wy = _to_world(ego, m["bx"], m["by"]) if ego is not None else (m["bx"], m["by"])
        Hm = np.zeros((2, 4))
        Hm[0, 0] = Hm[1, 1] = 1.0
        R = np.eye(2) * max(m["sigma"], 0.06) ** 2
        yv = np.array([wx, wy]) - Hm @ self.xw
        S = Hm @ self.Pw @ Hm.T + R
        Kg = self.Pw @ Hm.T @ np.linalg.inv(S)
        self.xw = self.xw + Kg @ yv
        self.Pw = (np.eye(4) - Kg @ Hm) @ self.Pw
        self.hits += 1
        self.last_seen = max(self.last_seen, now)
        if m.get("cluster"):
            self.cm = m
            if self.cam_hits == 0:
                self.m = m
            return
        if self.label is None or (m["label"] != self.label and self.cam_hits < 3):
            self.label = m["label"]  # the camera names what the LiDAR found (or corrects an early name)
        self.cam_hits += 1
        self.conf_sum += float(m.get("conf", 0.0))
        self.last_cam = now
        self.m = m
        if m.get("signal") is not None:
            self.color_votes[m["signal"]] = self.color_votes.get(m["signal"], 0.0) + 1.0
        for c in self.color_votes:
            self.color_votes[c] *= 0.7  # recent frames count most (a light changes)
        if self.color_votes:
            best = max(self.color_votes, key=self.color_votes.get)
            self.color = best if self.color_votes[best] >= 1.2 else self.color

    def body_xy(self, ego=None):
        """Position relative to the wearer at pose `ego` (x forward, y left); tracked in the body frame: as it is."""
        if ego is None or not self.world:
            return float(self.xw[0]), float(self.xw[1])
        c, s_ = math.cos(ego[2]), math.sin(ego[2])
        dx, dy = self.xw[0] - ego[0], self.xw[1] - ego[1]
        return float(c * dx + s_ * dy), float(-s_ * dx + c * dy)

    def set_view(self, ego=None, ego_vel=None):
        """x / P as seen from the wearer now (body frame, velocity relative to the wearer walking straight on)."""
        if ego is None or not self.world:
            self.x, self.P = self.xw.copy(), self.Pw.copy()
            return
        c, s_ = math.cos(ego[2]), math.sin(ego[2])
        Rt = np.array([[c, s_], [-s_, c]])  # world -> body
        pos = Rt @ (self.xw[:2] - ego[:2])
        vel = Rt @ (self.xw[2:4] - (ego_vel[:2] if ego_vel is not None else 0.0))
        self.x = np.array([pos[0], pos[1], vel[0], vel[1]])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = Rt @ self.Pw[:2, :2] @ Rt.T
        self.P[2:, 2:] = Rt @ self.Pw[2:, 2:] @ Rt.T

    def moving(self, speed: float) -> bool:
        """Confidently moving by itself (world frame) faster than `speed` m/s. Without odometry: approaching."""
        if not self.world:
            return self.approaching(speed)
        if self.hits < APPROACH_MIN_HITS:
            return False
        v = self.xw[2:4]
        sp = float(np.hypot(*v))
        u = v / max(sp, 1e-6)
        sigma = math.sqrt(max(0.0, float(u @ self.Pw[2:4, 2:4] @ u)))
        return sp - 2.0 * sigma > speed

    @property
    def world_speed(self):
        return float(np.hypot(*self.xw[2:4])) if self.world else None

    @property
    def dist(self):
        return float(math.hypot(self.x[0], self.x[1]))

    @property
    def closing_speed(self):
        """m/s toward the wearer (positive = approaching); 0 until the track is old enough."""
        if self.t - self.first_seen < SPEED_MIN_AGE_S:
            return 0.0
        r = max(self.dist, 1e-3)
        return float(-(self.x[0] * self.x[2] + self.x[1] * self.x[3]) / r)

    def approaching(self, speed: float) -> bool:
        """Confidently coming toward the wearer faster than `speed` (m/s): the closing speed minus two standard
        deviations of its own uncertainty (monocular ranges jitter by tens of centimetres per frame)."""
        if self.hits < APPROACH_MIN_HITS:
            return False
        r = max(self.dist, 1e-3)
        u = -self.x[:2] / r
        sigma = math.sqrt(max(0.0, float(u @ self.P[2:4, 2:4] @ u)))
        return self.closing_speed - 2.0 * sigma > speed

    @property
    def ttc(self):
        v = self.closing_speed
        return self.dist / v if v > 0.3 else math.inf

    def lateral_at_arrival(self):
        """Where across the path (y) it will be when it reaches the wearer's position along x."""
        vx = self.x[2]
        if vx >= -0.2 or self.x[0] <= 0:
            return float(self.x[1])
        t = self.x[0] / -vx
        return float(self.x[1] + self.x[3] * min(t, 5.0))

    def in_path(self, margin=0.0):
        half = CORRIDOR_HALF + 0.5 * min(self.m.get("width_m", 0.5), 3.0) + margin
        return self.x[0] > 0 and abs(self.x[1]) <= half

    def will_cross_path(self):
        half = CORRIDOR_HALF + 0.5 * min(self.m.get("width_m", 0.5), 3.0)
        return self.x[0] > 0 and abs(self.lateral_at_arrival()) <= half

    @property
    def yaw_body(self):
        """Heading of the object as seen from the wearer (for drawing): its motion, else its LiDAR outline."""
        if self.world_speed is not None and self.world_speed > 0.8:
            return float(math.atan2(self.x[3], self.x[2])) if np.hypot(self.x[2], self.x[3]) > 0.3 else 0.0
        if self.cm is not None and self.cm.get("yaw") is not None:
            return float(self.cm["yaw"])  # body frame of that scan: close enough for a drawing
        return 0.0


class OutdoorTracker:
    def __init__(self):
        self.tracks = []

    def update(self, meas, now, ego=None):
        """Camera detections or LiDAR clusters (base_footprint) of one moment; `ego` the wearer's pose then
        (odom), or None to track in the body frame."""
        for t in self.tracks:
            if t.world != (ego is not None):
                self.tracks = []  # odometry came or went: frames do not mix
                break
        for t in self.tracks:
            t.predict(now)
        pts = [(_to_world(ego, m["bx"], m["by"]) if ego is not None else (m["bx"], m["by"])) for m in meas]
        free = list(range(len(meas)))
        if self.tracks and meas:
            cost = np.full((len(meas), len(self.tracks)), 1e6)
            for i, m in enumerate(meas):
                for j, t in enumerate(self.tracks):
                    cluster = m.get("cluster")
                    if not cluster and t.label is not None and t.label != m["label"] and not _confusable(t.label, m["label"]):
                        continue
                    e = math.hypot(pts[i][0] - t.xw[0], pts[i][1] - t.xw[1])
                    kind = "vehicle" if cluster and t.kind == "vehicle" else t.kind
                    gate = (TRACK_GATE_M[kind] if not cluster else CLUSTER_GATE_M.get(kind, 0.8)) + 0.1 * t.dist
                    iou = 0.0 if cluster or t.m.get("box") is None else _box_iou(m["box"], t.m["box"])
                    if e <= gate or iou >= 0.4:
                        cost[i, j] = e / gate - iou + (0.0 if cluster or t.label == m["label"] else 0.3)
            if linear_sum_assignment is not None:
                pairs = zip(*linear_sum_assignment(cost))
            else:
                pairs, used = [], set()
                for i in np.argsort(cost.min(axis=1)):
                    j = int(np.argmin(cost[i]))
                    if j not in used:
                        pairs.append((i, j))
                        used.add(j)
            for i, j in pairs:
                if cost[i, j] < 1e5:
                    self.tracks[j].update(meas[i], now, ego)
                    meas[i]["track"] = self.tracks[j]
                    free.remove(i)
        for i in free:
            m = meas[i]
            if m.get("cluster") and not m.get("compact"):
                continue  # a long outline (wall, fence, car side) only updates what the camera named
            t = OutdoorTrack(m, now, ego)
            m["track"] = t
            self.tracks.append(t)
        # Gone: not seen for a moment, or now behind the wearer (they walked past it, or turned round), or named by
        # the camera that no longer sees it (CAM_KEEP_*)
        self.tracks = [t for t in self.tracks
                       if now - t.last_seen <= TRACK_MAX_GAP_S and not behind(*t.body_xy(ego))
                       and (t.label is None or t.cam_hits == 0 or now - t.last_cam <= self._cam_keep(t, ego))]
        return [t for t in self.tracks if t.confirmed]

    @staticmethod
    def _cam_keep(t, ego):
        x, y = t.body_xy(ego)
        in_view = x > 0.3 and abs(math.degrees(math.atan2(y, x))) < 0.85 * CAMERA_HALF_FOV_DEG
        return CAM_KEEP_IN_VIEW_S if in_view else CAM_KEEP_BESIDE_S

    def confirmed(self):
        """Confirmed objects ahead of and beside the wearer (as of the last set_view)."""
        return [t for t in self.tracks if t.confirmed and not behind(t.x[0], t.x[1])]

    def set_view(self, ego=None, ego_vel=None):
        for t in self.tracks:
            t.set_view(ego, ego_vel)

    def clear(self):
        self.tracks = []


def _confusable(a, b):
    return any(a in g and b in g for g in OUTDOOR_CONFUSABLE)


def _box_iou(a, b):
    inter = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return inter / max((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter, 1)


# ══════════════════════════════════════════════════════════════════════
# ── HAZARD ASSESSMENT AND SPOKEN ALERTS ──
# ══════════════════════════════════════════════════════════════════════
class Alert:
    def __init__(self, key, level, text, dist=None, kind="obstacle", slot=None, name=None):
        self.key, self.level, self.text, self.dist, self.kind = key, level, text, dist, kind
        # Alerts sharing a slot are one thing to say: only the most urgent of them is spoken (AlertPolicy).
        # Everything the wearer would walk into is in "path", a vehicle driving toward them in "vehicle".
        self.slot = slot or key
        self.name = name  # what it is ("chair"): tells another thing in the path from the same one again

    @property
    def rank(self):
        """What may cut off what: a danger over a warning, a vehicle coming over any other danger."""
        return self.level + (1 if self.kind == "vehicle" and self.level >= CRITICAL else 0)

    def as_dict(self):
        return {"key": self.key, "level": self.level, "text": self.text, "rank": self.rank,
                "dist": None if self.dist is None else round(self.dist, 2), "kind": self.kind}


def _step_advice(lanes, block_d):
    """"Step left." / "Step right." / "" — toward the side that stays clear well past the obstacle."""
    need = block_d + 1.5
    left, right = lanes.get("left", 0.0), lanes.get("right", 0.0)
    if max(left, right) < need:
        # Both sides blocked too: "Stop." only once it is close (not in the middle of a room)
        return " Stop." if block_d < STOP_ADVICE_M else ""
    if left >= need and (left > right + 0.5 or right < need):
        return " Step left."
    if right >= need and (right > left + 0.5 or left < need):
        return " Step right."
    return " Step left or right."


def assess(tracks, lidar, ground, turning=False):
    """All hazards in front of the wearer now, most urgent first. `tracks`: confirmed OutdoorTracks,
    `lidar`: lidar_corridor() or None, `ground`: GroundAnalysis or None, `turning`: the wearer is turning (or
    just turned) fast, so the LiDAR-only motion of things beside them is not trusted."""
    alerts = []

    def in_path(key, level, text, d, kind, name):
        alerts.append(Alert(key, level, text, d, kind, "path", name))
    # Lanes for "step left / right": the nearer blockage of LiDAR (chest height) and depth (low things)
    lanes = {}
    for name in ("left", "center", "right"):
        vals = [LOOK_AHEAD]
        if lidar is not None:
            vals.append(lidar[name])
        if ground is not None and ground.ok:
            vals.append(ground.lanes.get(name, LOOK_AHEAD))
        lanes[name] = min(vals)

    named_in_path = []  # (dist, track) of detected things in the corridor, to name what the LiDAR / depth sees
    for t in tracks:
        lbl, d = t.label, t.dist
        bearing = math.degrees(math.atan2(t.x[1], t.x[0]))
        if behind(t.x[0], t.x[1]):
            continue  # nothing behind the wearer is said
        # Out of the camera's view (beside): only the LiDAR sees it. With the wearer's own motion known,
        # something moving by itself toward them (a cyclist from a side road) is worth a warning, named or not.
        if abs(bearing) > CAMERA_HALF_FOV_DEG and (lbl is None or lbl in MOVERS):
            if (not turning and t.world and t.hits >= SIDE_MIN_HITS and t.t - t.first_seen >= SIDE_MIN_AGE_S
                    and t.moving(SIDE_MIN_SPEED) and t.approaching(1.0) and t.ttc < SIDE_TTC and d < SIDE_RANGE):
                side = "on your left" if bearing > 0 else "on your right"
                what = spoken_class(lbl).capitalize() if lbl else "Something"
                # Keyed by side, not by track: someone moving about beside the wearer breaks into new tracks
                alerts.append(Alert(f"side:{side}", WARNING, f"{what} coming {side}, {say_distance(d)}.", d, "side"))
            continue
        if lbl is None:
            continue  # unnamed LiDAR object in front: the corridor check below covers it
        name = spoken_class(lbl)
        if lbl in SIGNALS or lbl in CROSSINGS:
            continue  # reported by the signal / crossing alerts below
        if lbl in VEHICLES:
            ttc = t.ttc
            # Driving (not parked): with odometry its own speed, else faster toward the wearer than walking
            moving = (t.moving(1.0) and t.approaching(0.5)) if t.world else t.approaching(VEHICLE_APPROACH_MPS)
            if moving and (t.in_path(0.8) or t.will_cross_path()) and ttc < VEHICLE_TTC_CRITICAL:
                alerts.append(Alert(f"veh{t.id}", CRITICAL, f"Stop. {name.capitalize()} coming {where(*t.x[:2])}, "
                                    f"{say_distance(d)}.", d, "vehicle", "vehicle", name))
            elif moving and ttc < VEHICLE_TTC_WARNING and d < 25:
                alerts.append(Alert(f"veh{t.id}", WARNING, f"{name.capitalize()} approaching {where(*t.x[:2])}, "
                                    f"{say_distance(d)}.", d, "vehicle", "vehicle", name))
            elif t.in_path() and d < OBSTACLE_WARNING_M * 1.6:
                named_in_path.append((d, t))
                in_path(f"obj{t.id}", CRITICAL if d < OBSTACLE_CRITICAL_M else WARNING,
                        f"Parked {name} ahead, {say_distance(d)}.", d, "obstacle", name)
            continue
        if lbl in PEOPLE or lbl in ANIMALS:
            coming = (t.moving(0.5) and t.approaching(0.5)) if t.world else t.approaching(1.0)
            limit = ANIMAL_WARNING_M if lbl in ANIMALS else PERSON_WARNING_M
            if t.in_path() or (coming and t.will_cross_path()):
                named_in_path.append((d, t))
                if d < limit or (coming and t.ttc < 3.0):
                    what = f"{name.capitalize()} {'coming toward you' if coming else 'ahead'}"
                    in_path(f"obj{t.id}", CRITICAL if d < OBSTACLE_CRITICAL_M else WARNING,
                            f"{what}, {say_distance(d)}.", d, "mover", name)
            elif lbl in ANIMALS and d < limit:
                alerts.append(Alert(f"obj{t.id}", WARNING, f"{name.capitalize()} {where(*t.x[:2])}, "
                                    f"{say_distance(d)}.", d, "mover"))
            continue
        if lbl in DROPS:
            if t.in_path(0.3) and d < DROP_WARNING_M * 1.5:
                level = CRITICAL if d < DROP_CRITICAL_M else WARNING
                verb = {"stairs": "Stairs", "step": "Step", "curb": "Kerb"}.get(lbl, name.capitalize())
                in_path(f"obj{t.id}", level, f"{verb} ahead, {say_distance(d)}.", d, "drop", name)
            continue
        if lbl in OVERHEAD:
            low = t.m.get("z", 0.0)
            if t.in_path(0.2) and HEAD_LOW - 0.3 <= low <= HEAD_HIGH and d < OVERHEAD_WARNING_M * 1.5:
                level = CRITICAL if d < OVERHEAD_CRITICAL_M else WARNING
                in_path(f"obj{t.id}", level, f"Low {name} at head height, {say_distance(d)} ahead. Duck.", d,
                        "overhead", name)
            continue
        if lbl in INFO_ONLY:
            if lbl == "stop sign" and d < 12:
                alerts.append(Alert("stop sign", INFO, f"Stop sign {where(*t.x[:2])}.", d, "info"))
            continue
        if t.in_path() and d < OBSTACLE_WARNING_M * 1.6:
            named_in_path.append((d, t))

    # ── WHAT IS IN THE PATH (LiDAR at chest height, depth for low things), named after a detection there ──
    blockers = []
    if lidar is not None and lidar["center"] < LOOK_AHEAD:
        blockers.append((lidar["center"], lidar["center_y"], "lidar"))
    if ground is not None and ground.ok and ground.obstacle is not None:
        blockers.append((ground.obstacle[0], ground.obstacle[1], "depth"))
    if blockers:
        d, y, src = min(blockers)
        match = min((nt for nt in named_in_path if abs(nt[0] - d) < max(0.8, 0.25 * d)
                     and abs(nt[1].x[1] - y) <= NAME_LATERAL_M + 0.5 * min(nt[1].m.get("width_m", 0.5), 3.0)),
                    key=lambda nt: abs(nt[0] - d), default=None)
        # A moving person / vehicle is announced by its own alert above
        if match is None or match[1].label not in MOVERS:
            name = spoken_class(match[1].label) if match is not None else GENERIC
            key = f"obj{match[1].id}" if match is not None else "path"
            if d < OBSTACLE_WARNING_M:
                level = CRITICAL if d < OBSTACLE_CRITICAL_M else WARNING
                alerts[:] = [a for a in alerts if a.key != key]
                in_path(key, level, f"{name.capitalize()} ahead, {say_distance(d)}." + _step_advice(lanes, d), d,
                        "obstacle", name)
    else:
        # Detected static things in the path that neither LiDAR nor depth confirmed (below the scan plane,
        # no depth): still worth a warning when close
        for d, t in named_in_path:
            if t.label not in MOVERS and d < OBSTACLE_WARNING_M:
                in_path(f"obj{t.id}", CRITICAL if d < OBSTACLE_CRITICAL_M else WARNING,
                        f"{spoken_class(t.label).capitalize()} ahead, {say_distance(d)}." + _step_advice(lanes, d),
                        d, "obstacle", spoken_class(t.label))

    if ground is not None and ground.ok:
        if ground.drop is not None and not any(a.kind == "drop" for a in alerts):
            d = ground.drop[0]
            if d < DROP_WARNING_M:
                in_path("drop", CRITICAL if d < DROP_CRITICAL_M else WARNING,
                        f"Drop or hole ahead, {say_distance(d)}.", d, "drop", "drop")
        if ground.overhead is not None and not any(a.kind == "overhead" for a in alerts):
            d = ground.overhead[0]
            if d < OVERHEAD_WARNING_M:
                in_path("overhead", CRITICAL if d < OVERHEAD_CRITICAL_M else WARNING,
                        f"Something at head height, {say_distance(d)} ahead. Duck.", d, "overhead", "overhead")

    # ── CROSSINGS AND SIGNALS (informational) ──
    for t in tracks:
        if t.label in CROSSINGS and t.dist < 15:
            alerts.append(Alert("crossing", INFO, f"Zebra crossing {where(*t.x[:2])}, {say_distance(t.dist)}.",
                                t.dist, "crossing"))
    sig = signal_summary(tracks)
    if sig is not None:
        alerts.append(Alert(f"signal:{sig[0]}:{sig[1]}", INFO, sig[2], None, "signal"))

    alerts.sort(key=lambda a: (-a.level, a.dist if a.dist is not None else 1e9))
    return alerts, lanes


def signal_summary(tracks):
    """(kind, colour, sentence) for the most relevant light ahead, or None. A pedestrian signal wins over a
    traffic light (it is the one the wearer obeys)."""
    best = None
    for t in tracks:
        if t.label not in SIGNALS or t.color is None or t.dist > 40 or abs(math.degrees(math.atan2(t.x[1], t.x[0]))) > 40:
            continue
        rank = (0 if t.label == "walk signal" else 1, t.dist)
        if best is None or rank < best[0]:
            best = (rank, t)
    if best is None:
        return None
    t = best[1]
    if t.label == "walk signal":
        text = {"red": "Pedestrian signal is red. Wait.", "green": "Pedestrian signal is green.",
                "yellow": "Pedestrian signal is changing."}[t.color]
    else:
        text = {"red": "Traffic light ahead is red.", "green": "Traffic light ahead is green.",
                "yellow": "Traffic light ahead is yellow."}[t.color]
    return t.label, t.color, text


class AlertPolicy:
    """Decides which alert is spoken now, so the voice stays one clear sentence at a time however much is in view.

    Everything the wearer would walk into shares the slot "path": only the most urgent is spoken about, and the
    one announced stays the subject while it is about as near (STICK_M). It is said again when it has become more
    urgent than ever said before, when it is another thing (CHANGE_S), or after REPEAT_S, doubling while nothing
    changes. No sentence starts while the last one is still being spoken (say_time) or within PAUSE_S after it,
    except a danger greater than what is being said: a danger over a warning, a vehicle coming over any other
    danger. A signal is announced when its colour changes."""
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._said = {}          # slot -> what was last said about it: t, level, dist, key, name, repeats, gone
        self._busy_until = -1e9  # when the sentence being spoken ends
        self._busy_rank = 0
        self._last_signal = None
        self._clear_since = None
        self._announced_block = False

    def _may_speak(self, a, now):
        if now < self._busy_until:
            return a.level >= CRITICAL and a.rank > self._busy_rank
        return a.level >= CRITICAL or now >= self._busy_until + PAUSE_S[a.level]

    def _speak(self, a, now):
        self._busy_until, self._busy_rank = now + say_time(a.text), a.rank
        return a

    @staticmethod
    def _subject(group, prev):
        """The alert of a slot to speak about: the most urgent, or the one said last while it is still as urgent
        and about as near."""
        top = group[0]
        if prev is not None:
            for a in group:
                if a.key == prev["key"] and a.level >= top.level and (
                        a.dist is None or top.dist is None or a.dist <= top.dist + STICK_M):
                    return a
        return top

    @staticmethod
    def _fresh(a, prev, now):
        """Why `a` is worth saying after `prev` ("new" / "repeat"), or None."""
        if prev is None or a.level > prev["level"]:
            return "new"
        since = now - prev["t"]
        if a.dist is not None and prev["dist"] is not None:
            if a.dist > prev["dist"] + FARTHER_M and since >= CHANGE_S:
                return "new"  # what was announced is out of the way: this is the next thing
        # Another named thing. Not "obstacle" <-> "chair": the camera naming what the LiDAR found is no news
        if a.name != prev["name"] and GENERIC not in (a.name, prev["name"]) and since >= CHANGE_S:
            return "new"
        if since >= min(REPEAT_MAX_S, REPEAT_S[a.level] * 2 ** prev["repeats"]):
            return "repeat"
        return None

    def pick(self, alerts):
        now = self._clock()
        groups = {}
        for a in alerts:  # most urgent first
            groups.setdefault(a.slot, []).append(a)
        # Forget what is no longer there: the path once it has been clear a while (the next thing in it is new),
        # the rest after a minute
        for slot, prev in list(self._said.items()):
            if slot in groups:
                prev["gone"] = None
                continue
            if prev["gone"] is None:
                prev["gone"] = now
            if now - prev["gone"] >= (PATH_CLEAR_AFTER_S if slot in BLOCKING_SLOTS else 60.0):
                del self._said[slot]
        # "Path clear" after a blocking warning, once the path has stayed clear for a while
        if any(a.slot in BLOCKING_SLOTS and a.level >= WARNING for a in alerts):
            self._clear_since = None
        elif self._announced_block:
            if self._clear_since is None:
                self._clear_since = now
            if now - self._clear_since >= PATH_CLEAR_AFTER_S and now >= self._busy_until + PAUSE_S[INFO]:
                self._announced_block = False
                return self._speak(Alert("clear", INFO, "Path clear.", None, "clear"), now)
        for slot, group in groups.items():
            if group[0].kind == "signal":
                a = group[0]
                if a.key != self._last_signal and self._may_speak(a, now):
                    self._last_signal = a.key
                    return self._speak(a, now)
                continue
            prev = self._said.get(slot)
            a = self._subject(group, prev)
            why = self._fresh(a, prev, now)
            if why is None or not self._may_speak(a, now):
                continue
            level = max(a.level, prev["level"]) if prev is not None and prev["key"] == a.key else a.level
            self._said[slot] = {"t": now, "level": level, "dist": a.dist, "key": a.key, "name": a.name,
                                "repeats": prev["repeats"] + 1 if why == "repeat" else 0, "gone": None}
            if slot in BLOCKING_SLOTS and a.level >= WARNING:
                self._announced_block = True
            return self._speak(a, now)
        return None

    def reset(self):
        self.__init__(self._clock)


def scene_summary(tracks, lanes, ground=None, max_items=5):
    """What is ahead, in one or two sentences ("what is around me" in outdoor mode)."""
    parts = []
    for t in sorted((t for t in tracks if t.label is not None), key=lambda t: t.dist)[:max_items]:
        if t.label in SIGNALS:
            if t.color:
                parts.append(f"a {spoken_class(t.label)} showing {t.color} {where(*t.x[:2])}")
            continue
        motion = ""
        if t.label in MOVERS and (t.moving(0.5) if t.world else True) and t.approaching(0.5 if t.world else 1.0):
            motion = " coming toward you"
        parts.append(f"a {spoken_class(t.label)}{motion} {where(*t.x[:2])}, {say_distance(t.dist)}")
    path = lanes.get("center", LOOK_AHEAD) if lanes else LOOK_AHEAD
    if path >= LOOK_AHEAD:
        path_txt = f"The path ahead is clear for at least {say_distance(LOOK_AHEAD)}."
    else:
        path_txt = f"The path is blocked in {say_distance(path)}."
    if not parts:
        return "I see nothing around you. " + path_txt
    return "Ahead: " + "; ".join(parts) + ". " + path_txt
