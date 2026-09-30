#!/usr/bin/env python3
"""
system_manager.py
=================
Starts and stops the parts of VisionNav when the wearer asks for them with a button. On the laptop,
voice_navigation_assistant.py owns one; on the Pi, pi_button_panel.py owns one for the sensors. The wearer
never needs a terminal:

  brain       laptop_brain.launch.py (sensor TFs, Cartographer SLAM, Nav2, walls)         indoor
  map_view    RViz with the indoor map (rviz/visionnav.rviz)                              indoor
  perception  object_perception (YOLOE, depth, maps objects, hands for grasp mode)         indoor + outdoor
  vision_ai   scene_describer (Qwen3-VL via Ollama)                                      on demand (LOOK)
  outdoor_tf  outdoor_sensors.launch.py (camera and LiDAR mounts on the rig, live RViz view)  outdoor
  pi_sensors  pi_sensors.launch.py buttons:=false (LiDAR + camera)          on the Pi, SENSORS button

A part is "running" when its ROS node is on the network — so a part started by hand in a terminal is used
as it is and never started twice (and never stopped by the manager either: only parts it started itself).
Parts an earlier assistant started and left running (it was killed, or crashed) are adopted at start-up, so
the MODE button can stop them: an orphaned indoor brain kept its map window open in outdoor mode.
A part is "ready" when its node appears (the scene describer only creates its node after Qwen3-VL is loaded
on the GPU, so ready really means it can answer). Output of each part goes to ~/.visionnav/logs/<part>.log.
"""

import math
import os
import shlex
import shutil
import signal
import subprocess
import tempfile
import threading
import time

LOG_DIR = os.path.expanduser("~/.visionnav/logs")


def _share(*path):
    """A file of the installed package (share/visionnav), else of the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("visionnav"), *path)
    except Exception:
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), *path)


PARTS = {
    # WEARABLE_BRAIN_ARGS: the rig's measured geometry, e.g. "camera_height:=1.32 camera_pitch_deg:=12".
    # Without its RViz: the map window is a part of its own (map_view). localize:=false: every indoor session
    # maps the place it is in afresh (never the map of another day, which put the wearer in the wrong place
    # anywhere else); see start()
    "brain": {"cmd": ["ros2", "launch", "visionnav", "laptop_brain.launch.py", "use_rviz:=false", "localize:=false"]
                     + shlex.split(os.environ.get("WEARABLE_BRAIN_ARGS", "")), "node": "cartographer_node",
              "env": {"LIBGL_ALWAYS_SOFTWARE": "1"}, "timeout": 40, "name": "the map"},
    # The map window. Inside the brain's launch, a window that had been closed (or a brain left running by an
    # earlier session without it) stayed closed: "Indoor mode activated." and no map on the screen. As a part
    # of its own it is opened again whenever the mode starts and it is not there.
    "map_view": {"cmd": ["ros2", "run", "rviz2", "rviz2", "-d", _share("rviz", "visionnav.rviz"),
                         "--ros-args", "-r", "__node:=rviz2"], "node": "rviz2",
                 "env": {"LIBGL_ALWAYS_SOFTWARE": "1"}, "timeout": 30, "name": "the map window"},
    # object_perception's node exists before its models are loaded; its mode publisher only after
    "perception": {"cmd": ["ros2", "run", "visionnav", "object_perception"], "node": "object_perception",
                   "topic": "/perception_mode_state",
                   "env": {"WEARABLE_CAMERA_MODE": "ros"}, "timeout": 240, "name": "the camera AI"},
    "vision_ai": {"cmd": ["ros2", "run", "visionnav", "scene_describer"], "node": "scene_describer",
                  "env": {}, "timeout": 120, "name": "the vision AI"},
    # Same rig geometry as the brain (WEARABLE_BRAIN_ARGS), for the camera AI outdoors
    "outdoor_tf": {"cmd": ["ros2", "launch", "visionnav", "outdoor_sensors.launch.py"]
                          + shlex.split(os.environ.get("WEARABLE_BRAIN_ARGS", "")),
                   "node": "outdoor_camera_optical", "nodes": ["outdoor_camera_optical", "outdoor_lidar_mount"],
                   "env": {"LIBGL_ALWAYS_SOFTWARE": "1"}, "timeout": 20, "name": "the sensor geometry"},
}
PARTS["pi_sensors"] = {
    # buttons:=false: the button panel is the one starting it (a second panel would fight over the GPIO pins)
    "cmd": ["ros2", "launch", "visionnav", "pi_sensors.launch.py", "buttons:=false"],
    "node": "sllidar_node", "nodes": ["sllidar_node", "phone_camera_publisher"], "env": {}, "timeout": 30,
    "name": "the camera and LiDAR",
}
MODE_PARTS = {"indoor": ["brain", "map_view", "perception"],
              "outdoor": ["outdoor_tf", "perception"]}
IMU_UP_MAX_DEG = 45.0  # the IMU's gravity, turned by its mount in the TF, must be this close to straight up (a
                       # wrong mount is ~90 off; a wearer leaning 30 deg forward reads ~34)
STOP_GRACE_S = 8.0   # s after Ctrl+C before a part is terminated
STALE_S = 20.0       # s a stopped part's node may linger in the network's node list (DDS forgets it slowly)


class _Adopted:
    """A part started by an earlier manager: the Popen calls stop() and running() use, by pid."""
    def __init__(self, pid):
        self.pid = pid
        self.returncode = None

    def poll(self):
        try:
            with open(f"/proc/{self.pid}/stat") as f:
                if f.read().rsplit(")", 1)[1].split()[0] == "Z":
                    raise FileNotFoundError
            return None
        except (OSError, IndexError):
            self.returncode = 0
            return 0

    def wait(self, timeout=None):
        t0 = time.time()
        while self.poll() is None:
            if timeout is not None and time.time() - t0 > timeout:
                raise subprocess.TimeoutExpired(str(self.pid), timeout)
            time.sleep(0.1)
        return self.returncode


class SystemManager:
    def __init__(self, node, log=print):
        self._node = node          # an rclpy Node (to see which nodes are on the network)
        self._log = log
        self._procs = {}           # part -> Popen, for the parts this manager started
        self.imu_note = ""         # said with "mode activated" when the brain had to start without the IMU
        self._session_dir = None   # the running brain's folder for this session's places and names
        self._stopped = {}         # part -> when this manager stopped it
        self._lock = threading.Lock()
        os.makedirs(LOG_DIR, exist_ok=True)
        self._adopt_orphans()

    def _adopt_orphans(self):
        """Take over parts a previous manager started and left behind. They are recognised by how start() runs
        them: each is the leader of its own session (start_new_session), with the part's command line, and the
        program that started it is gone. A program run by hand in a terminal is never a session leader (its
        shell is), so it is still never touched."""
        for pid_s in os.listdir("/proc"):
            if not pid_s.isdigit():
                continue
            pid = int(pid_s)
            try:
                if os.getsid(pid) != pid:
                    continue
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
                with open(f"/proc/{pid}/stat") as f:
                    ppid = int(f.read().rsplit(")", 1)[1].split()[1])
                with open(f"/proc/{ppid}/cmdline", "rb") as f:
                    parent = f.read().replace(b"\0", b" ").decode(errors="replace")
            except (OSError, ValueError, IndexError):
                continue
            if "voice_navigation_assistant" in parent or "pi_button_panel" in parent:
                continue  # its manager is alive (another assistant): not an orphan
            i = next((k for k, a in enumerate(argv) if os.path.basename(a) == "ros2"), None)
            if i is None:
                continue
            for part, spec in PARTS.items():
                if part not in self._procs and argv[i + 1:i + 4] == spec["cmd"][1:4]:
                    self._procs[part] = _Adopted(pid)
                    self._log(f"adopted {part} (pid {pid}), left running by an earlier session")
                    break

    # ── state ──
    def _node_names(self):
        try:
            return {n for n, _ in self._node.get_node_names_and_namespaces()}
        except Exception:
            return set()

    def running(self, part) -> bool:
        """Ready on the network (started by this manager or by hand): its node, or its ready topic, is there."""
        spec = PARTS[part]
        proc = self._procs.get(part)
        if proc is not None and proc.poll() is not None:
            return False  # started here and has exited
        if proc is None and time.time() - self._stopped.get(part, -1e9) < STALE_S:
            return False  # stopped here moments ago: what the network still lists is stale
        if "topic" in spec:
            try:
                return self._node.count_publishers(spec["topic"]) > 0
            except Exception:
                return False
        return set(spec.get("nodes", [spec["node"]])) <= self._node_names()

    def starting(self, part) -> bool:
        p = self._procs.get(part)
        return p is not None and p.poll() is None and not self.running(part)

    # ── start / stop ──
    def start(self, part, wait=True) -> bool:
        """Start a part unless it is running; with `wait`, block until it is ready. True when ready."""
        spec = PARTS[part]
        # Stopped moments ago: its node lingers in the network's list for ~10 s, and a restart would look
        # "ready" at once (indoor mode was announced 1 s after restarting the SLAM brain). Wait for it to go.
        t0 = time.time()
        while (time.time() - self._stopped.get(part, -1e9) < STALE_S and spec["node"] in self._node_names()
               and time.time() - t0 < 15.0):
            time.sleep(0.3)
        with self._lock:
            if self.running(part):
                return True
            proc = self._procs.get(part)
            if proc is None or proc.poll() is not None:
                cmd = list(spec["cmd"])
                # The chest IMU, when the Pi publishes it (mpu6050_imu advertises /imu/data only once the chip
                # answers): Cartographer waits for every sensor it is given, so never without that publisher
                self.imu_note = ""
                if (part == "brain" and os.environ.get("WEARABLE_IMU", "auto") != "0"
                        and self._node.count_publishers("/imu/data") > 0):
                    ok, why = self._imu_mount_ok()
                    if ok:
                        cmd.append("imu:=true")
                    else:
                        self.imu_note = "The map runs without the IMU: its mount setting does not match the board."
                        self._log(f"NOT using the IMU: {why}")
                env = dict(os.environ, PYTHONUNBUFFERED="1", **spec["env"])  # logs written as they happen
                # OpenCV's pip wheel, once imported here (LOOK hold loads the vision AI's module), points Qt at its
                # own plugins through os.environ: every RViz started afterwards died with "Could not load the Qt
                # platform plugin xcb", and neither the map nor the outdoor view appeared
                for key in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
                    if "cv2" in env.get(key, ""):
                        del env[key]
                if part == "brain":
                    # The session's memory lives only as long as the map: map_manager (named places), the camera
                    # AI and the assistant (the wearer's names for objects) find this folder on /active_map; it
                    # starts empty and is deleted when the map stops. Nothing of an earlier session is loaded.
                    self._drop_session()
                    self._session_dir = tempfile.mkdtemp(prefix="visionnav_session_")
                    env["VISIONNAV_MAPS_DIR"] = self._session_dir
                log = open(os.path.join(LOG_DIR, f"{part}.log"), "ab")
                self._stopped.pop(part, None)
                self._procs[part] = proc = subprocess.Popen(
                    cmd, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)  # own process group: stop() ends the whole launch
                self._log(f"started {part} (pid {proc.pid}){' with the IMU' if 'imu:=true' in cmd else ''}, "
                          f"log {LOG_DIR}/{part}.log")
        if not wait:
            return False
        return self.wait_ready(part)

    def wait_ready(self, part) -> bool:
        spec, t0 = PARTS[part], time.time()
        while time.time() - t0 < spec["timeout"]:
            if self.running(part):
                return True
            proc = self._procs.get(part)
            if proc is not None and proc.poll() is not None:
                self._log(f"{part} exited (code {proc.returncode}); see {LOG_DIR}/{part}.log")
                return False
            if proc is None and self._stopped.get(part, -1e9) >= t0:
                return False  # stopped while starting (LOOK hold during the vision AI's start): no 2-min wait
            time.sleep(0.5)
        return self.running(part)

    def stop(self, part) -> bool:
        """Stop a part this manager started. Parts started by hand are left alone (returns False)."""
        with self._lock:
            proc = self._procs.pop(part, None)
        if proc is None or proc.poll() is not None:
            return False
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=STOP_GRACE_S)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            os.killpg(proc.pid, signal.SIGTERM)  # anything of it still running after its parent exited
        except ProcessLookupError:
            pass
        self._stopped[part] = time.time()
        self._log(f"stopped {part}")
        if part == "brain":
            self._drop_session()
        return True

    def _drop_session(self):
        if self._session_dir:
            shutil.rmtree(self._session_dir, ignore_errors=True)
            self._session_dir = None

    def _imu_mount_ok(self):
        """Whether the IMU mount in the TF (imu_*_deg: WEARABLE_BRAIN_ARGS, else sensor_tf.launch.py's defaults)
        turns the measured gravity upward. It once did not (the board lay flat, the TF had it upright), and
        Cartographer levelled every scan 90 deg wrong: the saved map never matched, the wearer's position and every
        object placed from it were wrong. Returns (ok, reason)."""
        try:
            import importlib.util
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("visionnav"), "launch", "sensor_tf.launch.py")
            spec = importlib.util.spec_from_file_location("visionnav_sensor_tf", path)
            tf_launch = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(tf_launch)
            mount = {k: float(tf_launch.ARGS[k][0]) for k in ("imu_roll_deg", "imu_pitch_deg", "imu_yaw_deg")}
            for arg in shlex.split(os.environ.get("WEARABLE_BRAIN_ARGS", "")):
                key, _, value = arg.partition(":=")
                if key in mount:
                    mount[key] = float(value)
        except Exception as e:
            return True, f"mount not read ({e})"  # cannot check: as before
        from sensor_msgs.msg import Imu
        samples = []
        sub = self._node.create_subscription(
            Imu, "/imu/data", lambda m: samples.append((m.linear_acceleration.x, m.linear_acceleration.y,
                                                         m.linear_acceleration.z)), 50)
        t0 = time.time()
        while len(samples) < 20 and time.time() - t0 < 3.0:
            time.sleep(0.05)
        self._node.destroy_subscription(sub)
        if not samples:
            return False, "no /imu/data message in 3 s"
        ax, ay, az = (sum(s[i] for s in samples) / len(samples) for i in range(3))
        # Into base_footprint: R = Rz(yaw) Ry(pitch) Rx(roll), as static_transform_publisher applies roll/pitch/yaw
        r, p, y = (math.radians(mount[k]) for k in ("imu_roll_deg", "imu_pitch_deg", "imu_yaw_deg"))
        ay, az = ay * math.cos(r) - az * math.sin(r), ay * math.sin(r) + az * math.cos(r)
        ax, az = ax * math.cos(p) + az * math.sin(p), -ax * math.sin(p) + az * math.cos(p)
        norm = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, az / norm))))
        roll_pitch_yaw = " ".join(f"{k}:={v:g}" for k, v in mount.items())
        if tilt <= IMU_UP_MAX_DEG:
            return True, f"gravity {tilt:.0f} deg from vertical with {roll_pitch_yaw}"
        return False, (f"with {roll_pitch_yaw} the IMU's gravity points {tilt:.0f} deg away from up. Run "
                       f"`setup_pi.sh imu` on the Pi and put the imu_* numbers it prints in WEARABLE_BRAIN_ARGS")

    def owned(self, part) -> bool:
        p = self._procs.get(part)
        return p is not None and p.poll() is None

    def stop_all(self):
        """All parts at once: one by one, a slow one (8 s grace each) made Ctrl+C take half a minute, and a second
        Ctrl+C meanwhile left the rest running on their own."""
        threads = [threading.Thread(target=self.stop, args=(part,)) for part in list(self._procs)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def name(self, part) -> str:
        return PARTS[part]["name"]
