#!/usr/bin/env python3
"""
system_manager.py
=================
Starts and stops the parts of VisionNav when the wearer asks for them with a button. On the laptop,
voice_navigation_assistant.py owns one; on the Pi, pi_button_panel.py owns one for the sensors. The wearer
never needs a terminal:

  brain       laptop_brain.launch.py (sensor TFs, Cartographer SLAM, Nav2, walls, RViz)   indoor
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

import os
import shlex
import signal
import subprocess
import threading
import time

LOG_DIR = os.path.expanduser("~/.visionnav/logs")

PARTS = {
    # WEARABLE_BRAIN_ARGS: the rig's measured geometry, e.g. "camera_height:=1.32 camera_pitch_deg:=12"
    "brain": {"cmd": ["ros2", "launch", "visionnav", "laptop_brain.launch.py"]
                     + shlex.split(os.environ.get("WEARABLE_BRAIN_ARGS", "")), "node": "cartographer_node",
              "env": {"LIBGL_ALWAYS_SOFTWARE": "1"}, "timeout": 40, "name": "the map"},
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
MODE_PARTS = {"indoor": ["brain", "perception"],
              "outdoor": ["outdoor_tf", "perception"]}
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
                env = dict(os.environ, PYTHONUNBUFFERED="1", **spec["env"])  # logs written as they happen
                log = open(os.path.join(LOG_DIR, f"{part}.log"), "ab")
                self._stopped.pop(part, None)
                self._procs[part] = proc = subprocess.Popen(
                    spec["cmd"], env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)  # own process group: stop() ends the whole launch
                self._log(f"started {part} (pid {proc.pid}), log {LOG_DIR}/{part}.log")
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
        return True

    def owned(self, part) -> bool:
        p = self._procs.get(part)
        return p is not None and p.poll() is None

    def stop_all(self):
        for part in list(self._procs):
            self.stop(part)

    def name(self, part) -> str:
        return PARTS[part]["name"]
