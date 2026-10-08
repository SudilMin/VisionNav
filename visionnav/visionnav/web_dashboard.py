#!/usr/bin/env python3
"""
web_dashboard.py
================
A web page to start VisionNav and watch what it does, for the people around the wearer (a helper, a demo, a
test on the rig). The wearer never needs it: the buttons and the voice do everything.

    ros2 run visionnav web_dashboard          then open http://localhost:8080

  top     START / STOP the assistant (voice_navigation_assistant, which starts everything else itself), the
          state of the Pi's sensors, the mode, the navigation, the laptop's free memory, which programs run
  left    the 3D view: what RViz showed, drawn in the page (web/scene3d.js) from the same topics, plus the live
          LiDAR scan. Indoors (rviz/visionnav.rviz) the SLAM map, the objects, the walls, your tracked path, the
          route and the TF frames; outdoors (rviz/visionnav_outdoor.rviz) the live occupancy and the objects ahead
  right   the camera: the camera AI's picture (boxes, distances), or the plain camera while the vision AI looks
  bottom  the voice: everything said and heard, and every press of the rig's buttons as it happens
The map is on while the camera AI runs (a mode is on); the camera while the camera AI runs or the vision AI is
answering a LOOK; otherwise they are off ("Camera closed", MODE hold, sensors off).

The assistant started here opens no desktop window (WEARABLE_WINDOWS=0: no RViz, no camera window; the page shows
both), writes to ~/.visionnav/logs/assistant.log and stops, with everything it started, when STOP is pressed or
this program exits: Ctrl+C stops it cleanly (up to STOP_CLEAN_S), a second Ctrl+C at once. An assistant started
elsewhere (at login) is used as it is; STOP asks it to exit. Settings: WEARABLE_WEB_PORT (8080), WEARABLE_WEB_HOST
(127.0.0.1; 0.0.0.0 to open it from a phone on the same Wi-Fi: anyone on that network can then use it).
"""

import base64
import collections
import gzip
import json
import math
import os
import signal
import subprocess
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from visionnav.outdoor_awareness import FRONT_LIMIT_DEG
from visionnav.system_manager import LOG_DIR, _share

PORT = int(os.environ.get("WEARABLE_WEB_PORT", "8080"))
HOST = os.environ.get("WEARABLE_WEB_HOST", "127.0.0.1")
ASSISTANT_NODE = "voice_navigation_assistant"
ASSISTANT_LOG = os.path.join(LOG_DIR, "assistant.log")
# Programs shown as running / not running, by their ROS node
PARTS = [("Assistant", ASSISTANT_NODE), ("Pi buttons", "pi_button_panel"), ("LiDAR", "sllidar_node"),
         ("Camera", "phone_camera_publisher"), ("IMU", "mpu6050_imu"), ("Map (SLAM)", "cartographer_node"),
         ("Camera AI", "object_perception"), ("Vision AI", "scene_describer")]
RVIZ_NODES = ("rviz2", "outdoor_rviz")  # RViz on the desktop: opened by an assistant started outside this page
CAMERA_NODE = "object_perception"  # the camera AI: the page's map is on only while it runs
VISION_NODE = "scene_describer"    # the vision AI (LOOK): it looks through the camera too
LOOK_MAX_S, LOOK_AFTER_S = 60.0, 5.0  # s the camera shows for a LOOK: until its answer (at most this), and after it
BUTTON_EVENTS = {"tap": "tap", "hold_start": "hold", "hold_end": "released", "double": "double press"}
BUTTON_SHOW_S = 3.0  # s a button press stays in the page's top bar
CAMERA_HZ = 8.0
VIEW_FRESH_S = 1.5   # s: the camera AI's picture is shown while this fresh, else the plain camera
FLIP_RAW = os.environ.get("WEARABLE_CAMERA_FLIP", "1") == "1"  # the Pi's stream arrives mirrored (object_perception)
LOG_KEEP = 400
STOP_CLEAN_S = 20.0  # s the assistant gets to stop the programs it started (8 s each, all at once) before it is killed
MEM_LOW_MB = 1500    # free memory (MemAvailable) shown as low: the laptop starts swapping and slows down
START_NEEDS_MB = 3500  # about what the assistant and indoor mode take (Whisper, the map, the camera AI): START asks
                       # first below it (7.6 GB laptop: a full browser and VS Code leave less, and earlyoom then
                       # closes programs, the browser first)

# The 3D view: RViz's displays (rviz/visionnav.rviz indoors, visionnav_outdoor.rviz outdoors), from the same topics
MARKER_TOPICS = {"indoor": ("/semantic_markers", "/structure_markers", "/trajectory_node_list"),
                 "outdoor": ("/outdoor_markers", "/outdoor_occupancy")}
SCENE_LAYERS = {"indoor": ("frame", "tf", "map", "scan", "route"), "outdoor": ("frame", "tf", "scan")}
LATCHED_TOPICS = ("/structure_markers",)
SCENE_HZ = 10.0      # what changed in the 3D view is sent to each page at most this often
TF_HZ = 10.0
IDENTITY = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)  # a pose: x y z qx qy qz qw
STATIC = {  # the page's script files (share/visionnav/web), sent gzipped
    "scene3d.js": "scene3d.js", "vendor/three.module.js": "vendor/three.module.js",
    "vendor/three.core.js": "vendor/three.core.js", "vendor/OrbitControls.js": "vendor/OrbitControls.js"}

BG = (30, 24, 20)  # placeholder pictures, dark like the page
STOP = threading.Event()  # the dashboard is closing: the pages' streams end


def _map_palette():
    """RViz's "map" colours by cell value as uint8 (-1 is 255), in BGR for cv2: 0 free white … 100 occupied
    black, 101-127 green, 128-254 red to yellow, -1 unknown grey-green. The page draws it at RViz's alpha 0.7."""
    lut = np.zeros((256, 3), np.uint8)
    lut[:101] = (255 - (np.arange(101) * 255) // 100)[:, None]
    lut[101:128] = (0, 255, 0)
    lut[128:255, 1] = (255 * (np.arange(128, 255) - 128)) // 126
    lut[128:255, 2] = 255
    lut[255] = (0x86, 0x89, 0x70)
    return lut


MAP_LUT = _map_palette()


def _print(*args):
    """print, also once the terminal is gone (closing it raises on every write, which broke off the clean-up)."""
    try:
        print(*args, flush=True)
    except (OSError, ValueError):
        pass


def _placeholder(text, sub=""):
    img = np.full((480, 640, 3), BG, np.uint8)
    cv2.putText(img, text, (40, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(img, sub, (40, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 1, cv2.LINE_AA)
    return cv2.imencode(".jpg", img)[1].tobytes()


def _memory():
    """MB of memory free to use (MemAvailable), of swap free and of RAM in all."""
    try:
        with open("/proc/meminfo") as f:
            kb = {k: int(v.split()[0]) for k, v in (line.split(":", 1) for line in f if ":" in line)}
        return {"avail": kb["MemAvailable"] // 1024, "swap": kb.get("SwapFree", 0) // 1024,
                "total": kb["MemTotal"] // 1024}
    except (OSError, KeyError, ValueError, IndexError):
        return None


# ── poses (x y z qx qy qz qw) ──
def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _compose(a, b):
    """Pose b, given in a frame whose pose is a, in a's own frame."""
    x, y, z, w = a[3:]
    vx, vy, vz = b[:3]
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (a[0] + vx + w * tx + y * tz - z * ty, a[1] + vy + w * ty + z * tx - x * tz,
            a[2] + vz + w * tz + x * ty - y * tx) + _qmul(a[3:], b[3:])


def _apply(pose, pts):
    """N x 3 points given in a frame whose pose is `pose`, in that pose's frame."""
    x, y, z, w = pose[3:]
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return pts @ rot.T + np.asarray(pose[:3])


def _msg_pose(p):
    """geometry_msgs/Pose as a pose; a zero quaternion is no rotation (as RViz draws it)."""
    q = p.orientation
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return (p.position.x, p.position.y, p.position.z) + (
        (q.x / n, q.y / n, q.z / n, q.w / n) if n > 1e-9 else (0.0, 0.0, 0.0, 1.0))


def _r(values, digits=3):
    return [round(float(v), digits) for v in values]


def _flat(rows, digits=3):
    return np.round(np.asarray(rows, np.float64), digits).ravel().tolist()


def _marker_json(m, frame_pose):
    """One marker as the page draws it (scene3d.js): its pose in the fixed frame; points stay in its own."""
    d = {"k": f"{m.ns}|{m.id}", "t": m.type, "p": _r(_compose(frame_pose, _msg_pose(m.pose)), 4),
         "s": _r((m.scale.x, m.scale.y, m.scale.z)), "c": _r((m.color.r, m.color.g, m.color.b, m.color.a))}
    if m.points:
        d["pts"] = _flat([(p.x, p.y, p.z) for p in m.points])
    if m.colors:
        d["cols"] = _flat([(c.r, c.g, c.b, c.a) for c in m.colors])
    if m.type == Marker.TEXT_VIEW_FACING:
        d["txt"] = m.text
    return d


# ── processes (the assistant started here, and everything it started) ──
def _processes():
    """pid -> (parent pid, start time, state) of every process, from /proc."""
    table = {}
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                with open(f"/proc/{d}/stat") as f:
                    fields = f.read().rsplit(")", 1)[1].split()
                table[int(d)] = (int(fields[1]), int(fields[19]), fields[0])
            except (OSError, ValueError, IndexError):
                continue
    return table


def _descendants(pid):
    """pid's children, their children …, as pid -> start time (a reused pid is another program)."""
    table = _processes()
    children = collections.defaultdict(list)
    for p, (ppid, _, _) in table.items():
        children[ppid].append(p)
    found, todo = {}, [pid]
    while todo:
        for c in children.get(todo.pop(), ()):
            if c not in found:
                found[c] = table[c][1]
                todo.append(c)
    return found


def _still_running(procs):
    table = _processes()
    return [p for p, start in procs.items() if p in table and table[p][1] == start and table[p][2] != "Z"]


def _names(pids):
    """What these processes run: their program, or what `ros2 run` / `ros2 launch` starts."""
    names = set()
    for p in pids:
        try:
            with open(f"/proc/{p}/cmdline", "rb") as f:
                argv = [os.path.basename(a.decode(errors="replace")) for a in f.read().split(b"\0") if a]
        except OSError:
            continue
        if len(argv) > 1 and argv[0].startswith("python"):
            argv = argv[1:]
        if len(argv) > 3 and argv[0] == "ros2":
            argv = argv[3:]
        if argv:
            names.add(argv[0])
    return ", ".join(sorted(names)[:8])


def _signal(pid, sig):
    """sig to pid's process group (a launch and its nodes, or `ros2 run` and its program), then to pid."""
    for send in (os.killpg, os.kill):
        try:
            send(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


class Dashboard(Node):
    def __init__(self):
        super().__init__("web_dashboard")
        self._lock = threading.Lock()
        self._latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._objects = 0
        self._outdoor = (None, 0.0)  # (/outdoor_scene, when)
        self._view = (None, 0.0)     # the camera AI's picture (JPEG bytes, when)
        self._raw = (None, 0.0)      # the plain camera
        self._button = (None, 0.0)   # the last button press (text, when)
        self._status = {"sensors": None, "mode": None, "nav": None, "buttons": None}
        self._log = collections.deque(maxlen=LOG_KEEP)
        self._log_id = 0
        self._nodes = set()
        self._watchers = {"camera": 0, "scene": 0}  # pages showing the camera, the 3D view
        self.create_subscription(String, "/semantic_objects", self._on_objects, 10)
        self.create_subscription(String, "/outdoor_scene", lambda m: self._set_outdoor(m.data), 10)
        self.create_subscription(String, "/voice_log", self._on_voice, 50)
        self.create_subscription(String, "/pi_sensors_state", lambda m: self._set("sensors", m.data), self._latched)
        self.create_subscription(String, "/pi_button_status", lambda m: self._set("buttons", m.data), self._latched)
        self.create_subscription(String, "/perception_mode_state", lambda m: self._set("mode", m.data),
                                 self._latched)
        self.create_subscription(String, "/semantic_nav_status", self._on_nav, 10)
        self.create_subscription(String, "/button_event", self._on_button, 10)
        # The vision AI's request and answer: the camera shows while it looks (it stays on after the camera AI closes)
        self._looking_until = 0.0
        self.create_subscription(String, "/describe_command", lambda m: self._look(LOOK_MAX_S), 10)
        self.create_subscription(String, "/scene_description", lambda m: self._look(LOOK_AFTER_S), 10)
        # The route is published once per navigation: always listened to, so a page opened later still shows it
        self.create_subscription(Path, "/object_path", self._on_route, 10)
        self._command_pub = self.create_publisher(String, "/voice_command", 10)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        # The 3D view's data, by layer: a version for each, raised when it changes, so each page's stream sends
        # only what changed; the JSON is made once per version, for every page
        self._scene_lock = threading.Lock()
        self._ver = collections.Counter()
        self._cache = {}             # layer -> (version, JSON)
        self._markers = {t: {} for ts in MARKER_TOPICS.values() for t in ts}  # topic -> (ns, id) -> (Marker, when)
        self._grid = self._grid_sum = None
        self._scan, self._scan_t = None, 0.0
        self._route = None
        self._frame = (None, None)   # the 3D view's (fixed frame, mode)
        self._tf_frames = set()
        self._last_expiry = 0.0
        # The streams are subscribed only while a page shows them: the plain camera and the LiDAR come over Wi-Fi
        # from the Pi once per subscriber, and the camera AI draws its picture and the outdoor markers only while
        # someone watches
        self._view_sub = self._raw_sub = self._scan_sub = None
        self._scan_topic = None
        self._scene_subs = []
        self.create_timer(1.0, self._watch_streams)
        self.create_timer(2.0, self._refresh_nodes)
        self.create_timer(1.0 / TF_HZ, self._tf_tick)
        self._proc = None            # the assistant, when started here
        self._phase = None           # "starting" / "stopping" while that is under way
        self._stopper = None         # the thread stopping it
        self._stop_t = 0.0           # when that began
        self._stopped_t = -math.inf  # when it was stopped (its node lingers in the network's list for a while)
        self._force = threading.Event()

    # ── ROS inputs ──
    def _set(self, key, value):
        with self._lock:
            self._status[key] = value

    def _on_objects(self, msg):
        try:
            objects = json.loads(msg.data).get("objects", [])
        except (ValueError, AttributeError):
            return
        with self._lock:
            self._objects = len(objects)

    def _set_outdoor(self, data):
        try:
            scene = json.loads(data)
        except ValueError:
            return
        with self._lock:
            self._outdoor = (scene, time.monotonic())

    def _on_nav(self, msg):
        try:
            nav = json.loads(msg.data)
            text = nav.get("status") or nav.get("state") or nav.get("message") if isinstance(nav, dict) else nav
        except ValueError:
            text = msg.data
        self._set("nav", str(text) if text is not None else None)

    def _on_voice(self, msg):
        try:
            entry = json.loads(msg.data)
        except ValueError:
            entry = {"kind": "said", "text": msg.data}
        self.add_log(entry.get("kind", "said"), entry.get("text", ""), entry.get("t"))

    def _on_button(self, msg):
        """A press of the rig's buttons (pi_button_panel.py), shown as it happens."""
        try:
            press = json.loads(msg.data)
            text = f"{str(press['button']).upper()} {BUTTON_EVENTS.get(press['event'], press['event'])}"
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            self._button = (text, time.monotonic())
        self.add_log("button", text, press.get("t"))

    def add_log(self, kind, text, t=None):
        with self._lock:
            self._log_id += 1
            self._log.append({"id": self._log_id, "kind": kind, "text": text, "t": t or time.time()})

    def _on_view(self, msg):
        self._view = (bytes(msg.data), time.monotonic())

    def _look(self, secs):
        self._looking_until = time.monotonic() + secs

    def _on_raw(self, msg):
        self._raw = (msg, time.monotonic())

    def watch(self, what, delta):
        """A page started (+1) or stopped (-1) showing the camera / the 3D view."""
        with self._lock:
            self._watchers[what] += delta

    def _watch_streams(self):
        with self._lock:
            camera, scene = self._watchers["camera"] > 0, self._watchers["scene"] > 0
        if camera and self._view_sub is None:
            self._view_sub = self.create_subscription(CompressedImage, "/perception/view/compressed", self._on_view,
                                                      qos_profile_sensor_data)
        elif not camera and self._view_sub is not None:
            self.destroy_subscription(self._view_sub)
            self._view_sub = None
        # The plain camera (over Wi-Fi from the Pi) only while the camera AI's picture is missing (vision AI only)
        want_raw = camera and time.monotonic() - self._view[1] > VIEW_FRESH_S
        if want_raw and self._raw_sub is None:
            self._raw_sub = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self._on_raw,
                                                     qos_profile_sensor_data)
        elif not want_raw and self._raw_sub is not None:
            self.destroy_subscription(self._raw_sub)
            self._raw_sub = None
        self._scene_subscriptions(scene)
        if scene:
            self._forget_stopped()
            self._update_frame()

    # ── the 3D view ──
    def _scene_subscriptions(self, scene):
        if scene and not self._scene_subs:
            self._scene_subs.append(self.create_subscription(OccupancyGrid, "/map", self._on_map, self._latched))
            for topic in self._markers:
                self._scene_subs.append(self.create_subscription(
                    MarkerArray, topic, lambda m, t=topic: self._on_markers(t, m),
                    self._latched if topic in LATCHED_TOPICS else 10))
        elif not scene and self._scene_subs:
            for sub in self._scene_subs:
                self.destroy_subscription(sub)
            self._scene_subs = []
            with self._scene_lock:
                for store in self._markers.values():
                    store.clear()
                self._grid = self._grid_sum = None
                for layer in list(self._ver):
                    self._ver[layer] += 1
        # The LiDAR: the brain's /scan_filtered (the wearer's own body taken out) while it runs, else the Pi's /scan
        topic = None
        if scene:
            topic = "/scan_filtered" if self.count_publishers("/scan_filtered") else "/scan"
        if topic != self._scan_topic:
            if self._scan_sub is not None:
                self.destroy_subscription(self._scan_sub)
                self._scan_sub = None
            if topic:
                self._scan_sub = self.create_subscription(LaserScan, topic, self._on_scan, qos_profile_sensor_data)
            self._scan_topic = topic
            with self._scene_lock:
                self._scan = None
                self._ver["scan"] += 1

    def _forget_stopped(self):
        """What a program drew goes when it stops, as RViz's window went with the session (a MODE hold closes the
        map): its map, its markers, its route. The latched ones would otherwise stay on the page."""
        gone = {t for t in list(self._markers) + ["/map", "/object_path"] if self.count_publishers(t) == 0}
        with self._scene_lock:
            if self._scan is not None and time.monotonic() - self._scan_t > 2.0:  # the LiDAR stopped
                self._scan = None
                self._ver["scan"] += 1
            if "/map" in gone and self._grid is not None:
                self._grid = self._grid_sum = None
                self._ver["map"] += 1
            if "/object_path" in gone and self._route is not None:
                self._route = None
                self._ver["route"] += 1
            for topic in gone & set(self._markers):
                if self._markers[topic]:
                    self._markers[topic].clear()
                    self._ver["m:" + topic] += 1

    def _on_map(self, msg):
        info = msg.info
        checksum = (zlib.crc32(msg.data), info.width, info.height, info.resolution, info.origin.position.x,
                    info.origin.position.y, msg.header.frame_id)
        with self._scene_lock:
            if checksum != self._grid_sum:  # Cartographer sends the map every second, mostly unchanged
                self._grid, self._grid_sum = msg, checksum
                self._ver["map"] += 1

    def _on_scan(self, msg):
        with self._scene_lock:
            self._scan, self._scan_t = msg, time.monotonic()
            self._ver["scan"] += 1

    def _on_route(self, msg):
        with self._scene_lock:
            self._route = msg
            self._ver["route"] += 1

    def _on_markers(self, topic, msg):
        """RViz's MarkerArray rules: ADD / MODIFY put a marker (ns, id) in place, DELETE takes one away, DELETEALL
        all of the topic's."""
        now = time.monotonic()
        with self._scene_lock:
            store = self._markers[topic]
            for m in msg.markers:
                if m.action == Marker.DELETEALL:
                    store.clear()
                elif m.action == Marker.DELETE:
                    store.pop((m.ns, m.id), None)
                else:
                    store[(m.ns, m.id)] = (m, now)
            self._ver["m:" + topic] += 1

    def _expire_markers(self):
        """A marker with a lifetime goes once it is that old (RViz's rule: the outdoor ones vanish when the camera
        AI stops sending)."""
        now = time.monotonic()
        if now - self._last_expiry < 0.1:
            return
        self._last_expiry = now
        with self._scene_lock:
            for topic, store in self._markers.items():
                gone = [k for k, (m, t) in store.items()
                        if (m.lifetime.sec or m.lifetime.nanosec)
                        and now - t > m.lifetime.sec + m.lifetime.nanosec * 1e-9]
                for k in gone:
                    del store[k]
                if gone:
                    self._ver["m:" + topic] += 1

    def _update_frame(self):
        """The 3D view's fixed frame, as RViz's: the SLAM map indoors while it is made (else the wearer), the
        wearer (base_footprint) outdoors, the LiDAR's own frame while nothing publishes TF (only the Pi is on). A
        stopped map's frame lingers in TF, frozen: it is not used."""
        try:
            tree = yaml.safe_load(self._tf.all_frames_as_yaml())
        except Exception:
            tree = None
        if not isinstance(tree, dict):
            tree = {}
        frames = set(tree) | {v.get("parent") for v in tree.values() if isinstance(v, dict)}
        frames.discard(None)
        self._tf_frames = frames
        mode = self._status["mode"] if self._status["mode"] in MARKER_TOPICS else "indoor"
        scan = self._scan
        if mode == "indoor" and "map" in frames and self.count_publishers("/map"):
            fixed = "map"
        elif "base_footprint" in frames:
            fixed = "base_footprint"
        elif scan is not None:
            fixed = scan.header.frame_id
        else:
            return
        if (fixed, mode) != self._frame:
            with self._scene_lock:
                self._frame = (fixed, mode)
                for layer in list(self._ver):  # everything again, in the new frame
                    self._ver[layer] += 1
                self._ver["frame"] += 1

    def _tf_tick(self):
        """Every TF frame and the wearer, in the fixed frame, for the 3D view (its TF display)."""
        fixed = self._frame[0]
        with self._lock:
            watched = self._watchers["scene"] > 0
        if not watched or fixed is None:
            return
        cache = {}
        frames = {}
        for name in sorted(self._tf_frames)[:40]:
            pose = self._frame_pose(fixed, name, cache)
            if pose is not None:
                frames[name] = _r(pose, 3)
        payload = json.dumps({"frames": frames, "you": frames.get("base_footprint")}, separators=(",", ":"))
        with self._scene_lock:
            cached = self._cache.get("tf")
            if cached is None or cached != (self._ver["tf"], payload):  # moved, or everything is sent again
                self._ver["tf"] += 1
                self._cache["tf"] = (self._ver["tf"], payload)

    def _frame_pose(self, target, source, cache):
        """Pose of frame `source` in frame `target` (the newest TF), or None while TF does not join them."""
        source = (source or target).lstrip("/")
        if source == target:
            return IDENTITY
        key = (target, source)
        if key not in cache:
            try:
                t = self._tf.lookup_transform(target, source, Time())
            except Exception:
                cache[key] = None
            else:
                tr, q = t.transform.translation, t.transform.rotation
                cache[key] = (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
        return cache[key]

    def scene_updates(self, sent):
        """(event, JSON) for every layer of the 3D view that changed since `sent` (layer -> version, updated)."""
        self._expire_markers()
        with self._scene_lock:
            versions = dict(self._ver)
        mode = self._frame[1] or "indoor"
        out = []
        for layer in SCENE_LAYERS[mode] + tuple("m:" + t for t in MARKER_TOPICS[mode]):
            version = versions.get(layer, 0)
            if sent.get(layer) == version:
                continue
            sent[layer] = version
            payload = self._layer_json(layer, version)
            if payload is not None:
                out.append(("markers" if layer.startswith("m:") else layer, payload))
        return out

    def _layer_json(self, layer, version):
        with self._scene_lock:
            cached = self._cache.get(layer)
        if cached is not None and cached[0] == version:
            return cached[1]
        if layer == "tf":
            return None  # made by _tf_tick
        data = self._layer(layer)
        payload = None if data is None else json.dumps(data, separators=(",", ":"))
        with self._scene_lock:
            self._cache[layer] = (version, payload)
        return payload

    def _layer(self, layer):
        fixed, mode = self._frame
        if fixed is None:
            return None
        cache = {}
        if layer == "frame":
            return {"fixed": fixed, "mode": mode}
        if layer == "map":
            return self._map_json(fixed, cache)
        if layer == "scan":
            return self._scan_json(fixed, mode, cache)
        if layer == "route":
            return self._route_json(fixed, cache)
        topic = layer[2:]
        with self._scene_lock:
            markers = [m for m, _ in self._markers[topic].values()]
        out = []
        for m in markers:
            pose = self._frame_pose(fixed, m.header.frame_id, cache)
            if pose is not None:  # its frame not in TF (yet): not drawn, as in RViz
                out.append(_marker_json(m, pose))
        return {"topic": topic, "markers": out}

    def _map_json(self, fixed, cache):
        """The SLAM map as a PNG in RViz's colours (one pixel per cell, row 0 at the origin) and where it lies."""
        grid = self._grid
        if grid is None:
            return {"png": None}  # none (any more): the page drops the one it shows
        info = grid.info
        pose = self._frame_pose(fixed, grid.header.frame_id or "map", cache)
        if pose is None or not info.width or not info.height:
            return None
        cells = np.asarray(grid.data, np.int8).view(np.uint8).reshape(info.height, info.width)
        ok, png = cv2.imencode(".png", MAP_LUT[cells])
        if not ok:
            return None
        return {"png": base64.b64encode(png.tobytes()).decode("ascii"), "w": info.width, "h": info.height,
                "res": info.resolution, "p": _r(_compose(pose, _msg_pose(info.origin)), 4)}

    def _scan_json(self, fixed, mode, cache):
        """The LiDAR's points in the fixed frame. Outdoors only what is ahead and beside the wearer: nothing behind
        them is drawn (outdoor_awareness.FRONT_LIMIT_DEG)."""
        scan = self._scan
        if scan is None:
            return {"pts": [], "topic": self._scan_topic}
        r = np.asarray(scan.ranges, np.float32)
        a = scan.angle_min + scan.angle_increment * np.arange(len(r), dtype=np.float32)
        ok = np.isfinite(r) & (r >= max(scan.range_min, 0.05)) & (r <= scan.range_max)
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(int(ok.sum()), np.float32)], 1)
        frame = scan.header.frame_id
        if mode == "outdoor":
            to_base = self._frame_pose("base_footprint", frame, cache)
            if to_base is None:
                return None
            pts = _apply(to_base, pts)
            pts = pts[np.abs(np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))) <= FRONT_LIMIT_DEG]
            frame = "base_footprint"
        pose = self._frame_pose(fixed, frame, cache)
        if pose is None:
            return None
        return {"pts": _flat(_apply(pose, pts), 2), "topic": self._scan_topic}

    def _route_json(self, fixed, cache):
        path = self._route
        if path is None or not path.poses:
            return {"pts": []}
        pose = self._frame_pose(fixed, path.header.frame_id or "map", cache)
        if pose is None:
            return {"pts": []}
        pts = np.array([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in path.poses])
        return {"pts": _flat(_apply(pose, pts))}

    def _refresh_nodes(self):
        try:
            nodes = set(self.get_node_names())
        except Exception:
            return
        with self._lock:
            self._nodes = nodes

    # ── the assistant ──
    def assistant_state(self):
        with self._lock:
            on_network = ASSISTANT_NODE in self._nodes
        own = self._proc is not None and self._proc.poll() is None
        if self._phase:
            return self._phase, own
        if not own and time.monotonic() - self._stopped_t < 15.0:
            on_network = False  # stopped here moments ago: what the network still lists is stale
        if own or on_network:
            return ("running" if on_network else "starting"), own
        return "stopped", False

    def launch(self, mode):
        state, _ = self.assistant_state()
        if state != "stopped":
            return f"The assistant is already {state}."
        # No RViz and no camera window: this page shows the map, the LiDAR and the camera
        env = dict(os.environ, PYTHONUNBUFFERED="1", WEARABLE_WINDOWS="0")
        if mode in ("indoor", "outdoor"):
            env["WEARABLE_MODE"] = mode
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(ASSISTANT_LOG, "w") as log:
            # Its own process group: STOP's Ctrl+C reaches it (and it stops the programs it started)
            self._proc = subprocess.Popen(["ros2", "run", "visionnav", ASSISTANT_NODE], stdin=subprocess.DEVNULL,
                                          stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        self._force.clear()
        self._phase = "starting"
        self.add_log("system", f"Starting the assistant ({mode or 'indoor'} mode)…")
        threading.Thread(target=self._watch_start, args=(self._proc,), daemon=True).start()
        return "Starting."

    def _watch_start(self, proc):
        t0 = time.monotonic()
        while self._phase == "starting" and time.monotonic() - t0 < 120:
            if proc.poll() is not None:
                self.add_log("system", f"The assistant exited at start: see the System log ({ASSISTANT_LOG}).")
                break
            with self._lock:  # (the node of one stopped moments ago may still be listed)
                up = ASSISTANT_NODE in self._nodes and time.monotonic() - self._stopped_t > 15.0
            if up:
                self.add_log("system", "Assistant running. Press SENSORS on the rig to start the mode.")
                break
            time.sleep(0.5)
        with self._lock:
            if self._phase == "starting":
                self._phase = None

    def stop(self, force=False):
        state, own = self.assistant_state()
        if not own:
            if state == "stopped":
                return "The assistant is not running."
            # Started elsewhere (at login): asked to exit as a typed "exit" would
            self._command_pub.publish(String(data="exit"))
            self.add_log("system", "Asked the assistant to exit.")
            return "Asked to exit."
        if force:
            self._force.set()
        with self._lock:
            if self._stopper is None or not self._stopper.is_alive():
                self._phase, self._stop_t = "stopping", time.monotonic()
                self._stopper = threading.Thread(target=self._stop_own, args=(self._proc,), daemon=True)
                self._stopper.start()
        return "Stopping."

    def force_stop(self):
        """A second Ctrl+C: what is still running of the assistant started here is killed now (also when it comes
        before the stop has begun)."""
        if not self._force.is_set():
            self._force.set()
            if self.owns_assistant():
                self._note("Stopping at once.")

    def _note(self, text):
        self.add_log("system", text)
        _print(text)

    def _stop_own(self, proc):
        """Ctrl+C to the assistant: it stops the programs it started itself (8 s each, all at once). After
        STOP_CLEAN_S, or at once when forced, whatever is left of it, its programs included, is killed: nothing
        it started keeps running (and filling the laptop's memory) after a STOP."""
        self._note(f"Stopping the assistant and the programs it started (up to {STOP_CLEAN_S:.0f} s; "
                   f"Ctrl+C again or Force stop: at once)…")
        tree = _descendants(proc.pid)  # before it exits: then its programs no longer show as its children
        _signal(proc.pid, signal.SIGINT)
        t0 = last_note = time.monotonic()
        while proc.poll() is None and not self._force.is_set() and time.monotonic() - t0 < STOP_CLEAN_S:
            tree.update(_descendants(proc.pid))
            if time.monotonic() - last_note > 6.0:
                last_note = time.monotonic()
                self._note(f"Still stopping: {_names(_still_running(tree)) or 'the assistant'}…")
            time.sleep(0.2)
        left = _still_running(tree)
        if proc.poll() is None or left:
            self._note(f"Killing what did not stop: {_names(left) or 'the assistant'}.")
            for pid in [proc.pid] + left:
                _signal(pid, signal.SIGKILL)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
        with self._lock:
            self._phase, self._stopped_t = None, time.monotonic()
        self._note("Assistant stopped.")

    def owns_assistant(self):
        return self._proc is not None and self._proc.poll() is None

    def shutdown_own(self):
        """At exit: the assistant started here stops too."""
        if self.owns_assistant():
            self.stop()
        stopper = self._stopper
        if stopper is not None:
            stopper.join()

    # ── what the page shows ──
    def state(self, since):
        with self._lock:
            nodes = self._nodes
            status = dict(self._status)
            log = [e for e in self._log if e["id"] > since]
            objects = self._objects
            scene = self._outdoor[0] if time.monotonic() - self._outdoor[1] < 3 else None
            button = self._button[0] if time.monotonic() - self._button[1] < BUTTON_SHOW_S else None
        assistant, own = self.assistant_state()
        return {"assistant": assistant, "own": own, **status, "objects": objects,
                "parts": [{"name": n, "on": node in nodes} for n, node in PARTS],
                "rviz": any(n in nodes for n in RVIZ_NODES), "map_on": CAMERA_NODE in nodes,
                "camera_on": CAMERA_NODE in nodes or (VISION_NODE in nodes
                                                      and time.monotonic() < self._looking_until),
                "button": button,
                "stopping_s": round(time.monotonic() - self._stop_t, 1) if assistant == "stopping" else 0,
                "mem": _memory(), "mem_low": MEM_LOW_MB, "start_needs": START_NEEDS_MB,
                "outdoor": scene and {"summary": scene.get("summary"), "signal": scene.get("signal")},
                "log": log}

    def log_tail(self, n_bytes=24000):
        try:
            with open(ASSISTANT_LOG, "rb") as f:
                f.seek(0, os.SEEK_END)
                f.seek(max(0, f.tell() - n_bytes))
                return f.read().decode("utf-8", "replace")
        except OSError:
            return ""

    def camera_jpeg(self):
        view, t_view = self._view
        now = time.monotonic()
        if view is not None and now - t_view < VIEW_FRESH_S:
            return view
        raw, t_raw = self._raw
        if raw is not None and now - t_raw < 2.0:
            img = cv2.imdecode(np.frombuffer(bytes(raw.data), np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                if FLIP_RAW:
                    img = cv2.flip(img, 1)
                return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        return _placeholder("Waiting for the camera picture")


class Handler(BaseHTTPRequestHandler):
    node = None
    page = None
    files = {}     # STATIC name -> gzipped bytes
    timeout = 30   # s: a page that went away mid-stream (Wi-Fi lost) frees its thread

    def log_message(self, *args):
        pass

    def _send(self, body, ctype="application/json", code=200, headers=()):
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        for key, value in headers or (("Cache-Control", "no-store"),):
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        q = parse_qs(url.query)
        if url.path in ("/", "/index.html"):
            self._send(self.page, "text/html; charset=utf-8")
        elif url.path == "/api/state":
            self._send(self.node.state(int(q.get("since", ["0"])[0] or 0)))
        elif url.path == "/api/log":
            self._send(self.node.log_tail(), "text/plain; charset=utf-8")
        elif url.path == "/api/scene":
            self._scene()
        elif url.path == "/stream/camera.mjpg":
            self.node.watch("camera", +1)
            try:
                self._stream(self.node.camera_jpeg, CAMERA_HZ)
            finally:
                self.node.watch("camera", -1)
        elif url.path.startswith("/static/") and url.path[8:] in self.files:
            name = url.path[8:]
            body = self.files[name]
            if "gzip" in self.headers.get("Accept-Encoding", ""):
                headers = (("Content-Encoding", "gzip"),)
            else:
                body, headers = gzip.decompress(body), ()
            # The three.js files never change; scene3d.js does with an update
            cache = "no-cache" if name == "scene3d.js" else "max-age=86400"
            self._send(body, "text/javascript; charset=utf-8", headers=headers + (("Cache-Control", cache),))
        else:
            self._send({"error": "not found"}, code=404)

    def do_POST(self):
        try:
            data = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0) or 0)) or b"{}")
        except ValueError:
            data = {}
        path = urlparse(self.path).path
        if path == "/api/launch":
            self._send({"message": self.node.launch(data.get("mode"))})
        elif path == "/api/stop":
            self._send({"message": self.node.stop(force=bool(data.get("force")))})
        else:
            self._send({"error": "not found"}, code=404)

    def _stream(self, frame, hz):
        """MJPEG: the browser shows each new JPEG as it comes (an <img> needs no script)."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while not STOP.is_set():
                jpg = frame()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1.0 / hz)
        except OSError:  # the page closed (broken pipe, reset, timeout)
            pass

    def _scene(self):
        """The 3D view as server-sent events: each layer when it changes (scene3d.js draws them)."""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.node.watch("scene", +1)
        sent, quiet_since = {}, time.monotonic()
        try:
            self.wfile.write(b"retry: 2000\n\n")
            while not STOP.is_set():
                for event, payload in self.node.scene_updates(sent):
                    self.wfile.write(f"event: {event}\ndata: {payload}\n\n".encode())
                    quiet_since = time.monotonic()
                if time.monotonic() - quiet_since > 10:
                    self.wfile.write(b": nothing new\n\n")  # tells a page that went away (the write fails)
                    quiet_since = time.monotonic()
                time.sleep(1.0 / SCENE_HZ)
        except OSError:
            pass
        finally:
            self.node.watch("scene", -1)


def main(args=None):
    # Ctrl+C is this program's, not rclpy's (that one ends ROS at once, under the clean-up): the first stops the
    # assistant started here cleanly, a second one at once
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Dashboard()
    with open(_share("web", "dashboard.html"), encoding="utf-8") as f:
        Handler.page = f.read()
    for name, rel in STATIC.items():
        with open(_share("web", rel), "rb") as f:
            Handler.files[name] = gzip.compress(f.read(), 6)
    Handler.node = node
    # Its own executor, stopped before the node is destroyed (rclpy.shutdown() under a spinning one aborts)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        server = ThreadingHTTPServer((HOST, PORT), Handler)
    except OSError as e:
        _print(f"Cannot open port {PORT} ({e.strerror}): is a dashboard already running? Close it, or use another "
               f"port: WEARABLE_WEB_PORT=8090 ros2 run visionnav web_dashboard")
        server = None
    if server is not None:
        server.daemon_threads = True
        shown = "localhost" if HOST in ("127.0.0.1", "0.0.0.0") else HOST
        _print(f"VisionNav dashboard: http://{shown}:{PORT}  (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}). "
               f"Ctrl+C to quit.")

        def on_signal(signum, _frame):
            if STOP.is_set():  # a second Ctrl+C (or the terminal closed meanwhile): no more waiting
                node.force_stop()
                return
            STOP.set()
            threading.Thread(target=server.shutdown, daemon=True).start()

        # Ctrl+C, closing the terminal (SIGHUP) and a kill (SIGTERM, e.g. earlyoom) all stop what it started
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, on_signal)
        try:
            server.serve_forever()
        finally:
            STOP.set()
            node.shutdown_own()
            server.server_close()
    executor.shutdown(timeout_sec=2.0)
    spinner.join(timeout=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
