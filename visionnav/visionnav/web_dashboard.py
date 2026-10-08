#!/usr/bin/env python3
"""
web_dashboard.py
================
A web page to start VisionNav and watch what it does, for the people around the wearer (a helper, a demo, a
test on the rig). The wearer never needs it: the buttons and the voice do everything.

    ros2 run visionnav web_dashboard          then open http://localhost:8080

  top     START / STOP the assistant (voice_navigation_assistant, which starts everything else itself), the
          state of the Pi's sensors, the mode, the navigation and which programs are running
  left    the map: the indoor SLAM map with you, the objects and the route; outdoors, what is ahead of you
  right   the camera: the camera AI's picture (boxes, distances), or the plain camera while that is off
  bottom  the voice: everything said and heard, typed commands, and the LOOK / MODE / HAND / TALK buttons
          (click = tap, hold = hold: speak into the laptop's microphone while holding, double click = double)

The assistant started here writes to ~/.visionnav/logs/assistant.log and stops (with everything it started) when
STOP is pressed or this program exits. An assistant started elsewhere (at login) is used as it is; STOP asks it to
exit. Settings: WEARABLE_WEB_PORT (8080), WEARABLE_WEB_HOST (127.0.0.1; 0.0.0.0 to open it from a phone on the
same Wi-Fi: anyone on that network can then use it).
"""

import collections
import json
import math
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from visionnav.system_manager import LOG_DIR, _share

PORT = int(os.environ.get("WEARABLE_WEB_PORT", "8080"))
HOST = os.environ.get("WEARABLE_WEB_HOST", "127.0.0.1")
ASSISTANT_NODE = "voice_navigation_assistant"
ASSISTANT_LOG = os.path.join(LOG_DIR, "assistant.log")
# Programs shown as running / not running, by their ROS node
PARTS = [("Assistant", ASSISTANT_NODE), ("Pi buttons", "pi_button_panel"), ("LiDAR", "sllidar_node"),
         ("Camera", "phone_camera_publisher"), ("Map (SLAM)", "cartographer_node"),
         ("Camera AI", "object_perception"), ("Vision AI", "scene_describer"), ("Map window", "rviz2")]
HOLD_S, DOUBLE_TAP_S = 0.6, 0.4  # as the Pi's buttons (pi_button_panel.py)
WEB_BUTTONS = ("look", "mode", "hand", "talk")  # SENSORS is the Pi's own: it powers the camera and LiDAR
MAP_PX = 720         # the map picture's longer side
MAP_HZ, CAMERA_HZ = 2.0, 8.0
VIEW_FRESH_S = 1.5   # s: the camera AI's picture is used while this fresh, else the plain camera
OUTDOOR_RANGE_M = 20.0
FLIP_RAW = os.environ.get("WEARABLE_CAMERA_FLIP", "1") == "1"  # the Pi's stream arrives mirrored (object_perception)
LOG_KEEP = 400

# Colours (BGR) of the map picture, dark like the page
BG = (30, 24, 20)
UNKNOWN, FREE, WALL = (40, 33, 28), (70, 60, 52), (230, 230, 230)
YOU, ROUTE, PERSON, OBJECT = (255, 160, 60), (120, 220, 90), (60, 150, 255), (90, 210, 250)


def _placeholder(text, sub=""):
    img = np.full((480, 640, 3), BG, np.uint8)
    cv2.putText(img, text, (40, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2, cv2.LINE_AA)
    if sub:
        cv2.putText(img, sub, (40, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (140, 140, 140), 1, cv2.LINE_AA)
    return cv2.imencode(".jpg", img)[1].tobytes()


def _label(img, text, x, y, colour):
    cv2.putText(img, text, (x + 7, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x + 7, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)


class Dashboard(Node):
    def __init__(self):
        super().__init__("web_dashboard")
        self._lock = threading.Lock()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map = None             # OccupancyGrid
        self._map_img = None         # (base picture, its grid's stamp, mapped area): drawn once per new map
        self._objects = []
        self._route = []
        self._outdoor = (None, 0.0)  # (/outdoor_scene, when)
        self._view = (None, 0.0)     # the camera AI's picture (JPEG bytes, when)
        self._raw = (None, 0.0)      # the plain camera
        self._status = {"sensors": None, "mode": None, "nav": None, "buttons": None}
        self._log = collections.deque(maxlen=LOG_KEEP)
        self._log_id = 0
        self._nodes = set()
        self.camera_clients = 0
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(String, "/semantic_objects", self._on_objects, 10)
        self.create_subscription(Path, "/object_path", self._on_route, 10)
        self.create_subscription(String, "/outdoor_scene", lambda m: self._set_outdoor(m.data), 10)
        self.create_subscription(String, "/voice_log", self._on_voice, 50)
        self.create_subscription(String, "/pi_sensors_state", lambda m: self._set("sensors", m.data), latched)
        self.create_subscription(String, "/pi_button_status", lambda m: self._set("buttons", m.data), latched)
        self.create_subscription(String, "/perception_mode_state", lambda m: self._set("mode", m.data), latched)
        self.create_subscription(String, "/semantic_nav_status", self._on_nav, 10)
        self._command_pub = self.create_publisher(String, "/voice_command", 10)
        self._button_pub = self.create_publisher(String, "/button_event", 10)
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        # The camera streams are subscribed only while a page shows them: the plain camera comes over Wi-Fi from the
        # Pi once per subscriber, and the camera AI encodes its picture only while someone watches
        self._view_sub = self._raw_sub = None
        self.create_timer(1.0, self._camera_subscriptions)
        self.create_timer(2.0, self._refresh_nodes)
        # Web buttons: the Pi's press rules (hold after HOLD_S, double within DOUBLE_TAP_S)
        self._down = {}
        self._held = {b: False for b in WEB_BUTTONS}
        self._last_tap = {b: 0.0 for b in WEB_BUTTONS}
        self._proc = None            # the assistant, when started here
        self._proc_state = None      # "starting" / "stopping" while that is under way

    # ── ROS inputs ──
    def _set(self, key, value):
        with self._lock:
            self._status[key] = value

    def _on_map(self, msg):
        with self._lock:
            self._map = msg

    def _on_objects(self, msg):
        try:
            objects = json.loads(msg.data).get("objects", [])
        except (ValueError, AttributeError):
            return
        with self._lock:
            self._objects = objects

    def _on_route(self, msg):
        with self._lock:
            self._route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

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

    def add_log(self, kind, text, t=None):
        with self._lock:
            self._log_id += 1
            self._log.append({"id": self._log_id, "kind": kind, "text": text, "t": t or time.time()})

    def _on_view(self, msg):
        self._view = (bytes(msg.data), time.monotonic())

    def _on_raw(self, msg):
        self._raw = (msg, time.monotonic())

    def _camera_subscriptions(self):
        watching = self.camera_clients > 0
        if watching and self._view_sub is None:
            self._view_sub = self.create_subscription(CompressedImage, "/perception/view/compressed", self._on_view,
                                                      qos_profile_sensor_data)
        elif not watching and self._view_sub is not None:
            self.destroy_subscription(self._view_sub)
            self._view_sub = None
        want_raw = watching and time.monotonic() - self._view[1] > VIEW_FRESH_S
        if want_raw and self._raw_sub is None:
            self._raw_sub = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self._on_raw,
                                                     qos_profile_sensor_data)
        elif not want_raw and self._raw_sub is not None:
            self.destroy_subscription(self._raw_sub)
            self._raw_sub = None

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
        if self._proc_state:
            return self._proc_state, own
        if own or on_network:
            return ("running" if on_network else "starting"), own
        return "stopped", False

    def launch(self, mode):
        state, _ = self.assistant_state()
        if state != "stopped":
            return f"The assistant is already {state}."
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        if mode in ("indoor", "outdoor"):
            env["WEARABLE_MODE"] = mode
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(ASSISTANT_LOG, "w") as log:
            # Its own process group: STOP's Ctrl+C reaches it (and it stops the programs it started)
            self._proc = subprocess.Popen(["ros2", "run", "visionnav", ASSISTANT_NODE], stdin=subprocess.DEVNULL,
                                          stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        self.add_log("system", f"Starting the assistant ({mode or 'indoor'} mode)…")
        threading.Thread(target=self._watch_start, daemon=True).start()
        return "Starting."

    def _watch_start(self):
        self._proc_state = "starting"
        t0 = time.monotonic()
        while time.monotonic() - t0 < 60:
            if self._proc is None or self._proc.poll() is not None:
                self.add_log("system", f"The assistant exited at start: see {ASSISTANT_LOG}")
                break
            with self._lock:
                if ASSISTANT_NODE in self._nodes:
                    self.add_log("system", "Assistant running. Press SENSORS on the rig to start the mode.")
                    break
            time.sleep(0.5)
        self._proc_state = None

    def stop(self):
        state, own = self.assistant_state()
        if state == "stopped":
            return "The assistant is not running."
        if not own:
            # Started elsewhere (at login): asked to exit as a typed "exit" would
            self._command_pub.publish(String(data="exit"))
            self.add_log("system", "Asked the assistant to exit.")
            return "Asked to exit."
        threading.Thread(target=self._stop_own, daemon=True).start()
        return "Stopping."

    def _stop_own(self):
        proc = self._proc
        self._proc_state = "stopping"
        self.add_log("system", "Stopping the assistant and the programs it started…")
        for sig, wait_s in ((signal.SIGINT, 30), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(proc.pid, sig)
                proc.wait(timeout=wait_s)
                break
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                break
        self.add_log("system", "Assistant stopped.")
        self._proc_state = None

    def shutdown_own(self):
        if self._proc is not None and self._proc.poll() is None:
            self._stop_own()

    # ── commands and buttons from the page ──
    def command(self, text):
        text = text.strip()
        if text:
            self._command_pub.publish(String(data=text))

    def button(self, name, action):
        """The page's button went down or up: tap / hold_start / hold_end / double, as the Pi decides them."""
        if name not in WEB_BUTTONS:
            return
        if action == "down":
            token = object()
            self._down[name] = token
            threading.Timer(HOLD_S, self._hold, args=(name, token)).start()
        elif action == "up" and self._down.pop(name, None) is not None:
            if self._held[name]:
                self._held[name] = False
                self._emit(name, "hold_end")
                return
            self._emit(name, "tap")
            now = time.monotonic()
            if now - self._last_tap[name] <= DOUBLE_TAP_S:
                self._emit(name, "double")
                self._last_tap[name] = 0.0
            else:
                self._last_tap[name] = now

    def _hold(self, name, token):
        if self._down.get(name) is token:
            self._held[name] = True
            self._emit(name, "hold_start")

    def _emit(self, name, event):
        self._button_pub.publish(String(data=json.dumps({"button": name, "event": event, "t": round(time.time(), 3),
                                                          "source": "web"})))

    # ── what the page shows ──
    def state(self, since):
        with self._lock:
            nodes = self._nodes
            status = dict(self._status)
            log = [e for e in self._log if e["id"] > since]
            objects = len(self._objects)
            scene = self._outdoor[0] if time.monotonic() - self._outdoor[1] < 3 else None
        assistant, own = self.assistant_state()
        return {"assistant": assistant, "own": own, **status, "objects": objects,
                "parts": [{"name": n, "on": node in nodes} for n, node in PARTS],
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
                cv2.putText(img, "camera (camera AI off)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255),
                            2, cv2.LINE_AA)
                return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        return _placeholder("No camera picture", "Start the assistant, then press SENSORS on the rig.")

    def map_jpeg(self):
        with self._lock:
            grid, objects, route = self._map, list(self._objects), list(self._route)
            mode = self._status["mode"]
            scene = self._outdoor[0] if time.monotonic() - self._outdoor[1] < 3 else None
        if mode == "outdoor" and scene is not None:
            return self._outdoor_jpeg(scene)
        if grid is None:
            return _placeholder("No map yet", "Indoor mode maps the place as you walk.")
        return self._indoor_jpeg(grid, objects, route)

    def _pose(self):
        try:
            t = self._tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _indoor_jpeg(self, grid, objects, route):
        info = grid.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        stamp = (grid.header.stamp.sec, grid.header.stamp.nanosec, w, h)
        if self._map_img is None or self._map_img[1] != stamp:
            cells = np.asarray(grid.data, np.int8).reshape(h, w)
            img = np.empty((h, w, 3), np.uint8)
            img[:] = UNKNOWN
            known = cells >= 0
            occ = np.clip(cells.astype(np.float32) / 100.0, 0, 1)[..., None]
            shade = (np.array(FREE, np.float32) * (1 - occ) + np.array(WALL, np.float32) * occ).astype(np.uint8)
            img[known] = shade[known]
            rows, cols = np.nonzero(np.flipud(known))
            bbox = (cols.min(), cols.max(), rows.min(), rows.max()) if len(rows) else (0, w - 1, 0, h - 1)
            self._map_img = (np.flipud(img), stamp, bbox)
        base, _, (c0, c1, r0, r1) = self._map_img
        pose = self._pose()

        def cell(x, y):
            return (x - ox) / res, h - 1 - (y - oy) / res

        # Crop to what is mapped (and you), with a margin, so a small room is not a speck in a big grid
        if pose is not None:
            pc, pr = cell(pose[0], pose[1])
            c0, c1, r0, r1 = min(c0, pc), max(c1, pc), min(r0, pr), max(r1, pr)
        margin = 2.0 / res
        c0, r0 = int(max(0, c0 - margin)), int(max(0, r0 - margin))
        c1, r1 = int(min(w - 1, c1 + margin)), int(min(h - 1, r1 + margin))
        crop = base[r0:r1 + 1, c0:c1 + 1]
        scale = MAP_PX / max(crop.shape[:2])
        img = cv2.resize(crop, (max(1, int(crop.shape[1] * scale)), max(1, int(crop.shape[0] * scale))),
                         interpolation=cv2.INTER_NEAREST)

        def px(x, y):
            c, r = cell(x, y)
            return int((c - c0 + 0.5) * scale), int((r - r0 + 0.5) * scale)

        # 1 m grid scale bar
        bar = int(1.0 / res * scale)
        cv2.line(img, (12, img.shape[0] - 14), (12 + bar, img.shape[0] - 14), (200, 200, 200), 2)
        _label(img, "1 m", 12 + bar, img.shape[0] - 4, (200, 200, 200))
        if len(route) > 1:
            cv2.polylines(img, [np.array([px(x, y) for x, y in route], np.int32)], False, ROUTE, 3, cv2.LINE_AA)
        for o in objects:
            p = px(o["x"], o["y"])
            colour = PERSON if o.get("dynamic") else OBJECT
            cv2.circle(img, p, 6, colour, -1 if o.get("live", True) else 2, cv2.LINE_AA)
            _label(img, o.get("name", o.get("class", "")), *p, colour)
        if pose is not None:
            x, y, yaw = pose
            p = px(x, y)
            tip = px(x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw))
            cv2.circle(img, p, 9, YOU, -1, cv2.LINE_AA)
            cv2.arrowedLine(img, p, tip, YOU, 3, cv2.LINE_AA, tipLength=0.4)
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()

    def _outdoor_jpeg(self, scene):
        """You at the bottom, facing up; only what is ahead and beside you (nothing behind is tracked)."""
        img = np.full((MAP_PX, MAP_PX, 3), BG, np.uint8)
        cx, cy = MAP_PX // 2, int(MAP_PX * 0.85)
        s = (MAP_PX * 0.8) / OUTDOOR_RANGE_M
        for r in (2, 5, 10, 20):
            cv2.ellipse(img, (cx, cy), (int(r * s), int(r * s)), 0, -190, 10, (70, 60, 52), 1, cv2.LINE_AA)
            _label(img, f"{r} m", cx + int(r * s * math.cos(math.radians(-80))) - 20,
                   cy - int(r * s * math.sin(math.radians(80))), (140, 140, 140))
        for o in scene.get("objects", []):
            x, y = o.get("x", 0.0), o.get("y", 0.0)  # x forward, y left (base_footprint)
            if abs(math.degrees(math.atan2(y, x))) > 100:
                continue
            p = (int(cx - y * s), int(cy - x * s))
            ttc = o.get("ttc")
            colour = (60, 60, 255) if ttc is not None and ttc < 3 else \
                     (40, 160, 255) if (ttc is not None and ttc < 6) or o.get("in_path") else OBJECT
            cv2.circle(img, p, 8, colour, -1, cv2.LINE_AA)
            _label(img, f"{o.get('class', '')} {o.get('dist', 0):.1f} m", *p, colour)
        cv2.circle(img, (cx, cy), 10, YOU, -1, cv2.LINE_AA)
        cv2.arrowedLine(img, (cx, cy), (cx, cy - 40), YOU, 3, cv2.LINE_AA, tipLength=0.4)
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


class Handler(BaseHTTPRequestHandler):
    node = None
    page = None

    def log_message(self, *args):
        pass

    def _send(self, body, ctype="application/json", code=200):
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
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
        elif url.path == "/stream/map.mjpg":
            self._stream(self.node.map_jpeg, MAP_HZ)
        elif url.path == "/stream/camera.mjpg":
            self.node.camera_clients += 1
            try:
                self._stream(self.node.camera_jpeg, CAMERA_HZ)
            finally:
                self.node.camera_clients -= 1
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
            self._send({"message": self.node.stop()})
        elif path == "/api/command":
            self.node.command(str(data.get("text", "")))
            self._send({"ok": True})
        elif path == "/api/button":
            self.node.button(str(data.get("button", "")), str(data.get("action", "")))
            self._send({"ok": True})
        else:
            self._send({"error": "not found"}, code=404)

    def _stream(self, frame, hz):
        """MJPEG: the browser shows each new JPEG as it comes (an <img> needs no script)."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while rclpy.ok():
                jpg = frame()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1.0 / hz)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Dashboard()
    with open(_share("web", "dashboard.html"), encoding="utf-8") as f:
        Handler.page = f.read()
    Handler.node = node
    # Its own executor, stopped before the node is destroyed (rclpy.shutdown() under a spinning one aborts)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.daemon_threads = True
    shown = "localhost" if HOST in ("127.0.0.1", "0.0.0.0") else HOST
    print(f"VisionNav dashboard: http://{shown}:{PORT}  (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')})")
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # a second Ctrl+C must not break off the clean-up
        print("Stopping the assistant this dashboard started (if any)...")
        node.shutdown_own()
        server.server_close()
        executor.shutdown()
        spinner.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
