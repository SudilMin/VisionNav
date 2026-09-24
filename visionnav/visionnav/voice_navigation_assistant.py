#!/usr/bin/env python3
"""
voice_navigation_assistant.py
-----------------------------
Voice-guided semantic navigation node — like Google Maps for blind users.

Flow:
  1. User drives robot with arrow_teleop → YOLO detects objects
  2. User types: find chair
  3. System SPEAKS: "Chair detected! Say go to chair."
  4. User types: go to chair
  5. System calculates A* path, draws it on RViz
  6. System gives CONTINUOUS turn-by-turn voice navigation:
     "Go straight... Turn left now... Keep going, 12 feet... You have arrived."
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import threading
import math
import heapq
import subprocess
import os
import time
import queue
import json
import sys
import contextlib
import tempfile
import speech_recognition as sr
from faster_whisper import WhisperModel
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from visionnav.model_paths import model_path

TMP_DIR = tempfile.gettempdir()  # scratch audio/images
TTS_MODEL = model_path("en_US-lessac-medium.onnx")
# Memory-anchored terminal navigation: the target's map position is locked when navigation starts,
# because a low object (chair) drops below the chest-mounted camera/LiDAR in the last metre.
# Arrival is decided purely by the SLAM (TF) distance to that locked point.
ARRIVAL_RADIUS = 0.6  # m from the locked object centre
HAPTIC_TOPIC = '/haptic_command'
# Grasp mode: guide the hand to an object ("grasp cup"), also started on arrival at an object
GRASP_COMMANDS = ("grasp ", "grab ", "pick up ", "reach for ")
GRASP_TOL = 0.05          # m left/right or up/down still worth a cue
GRASP_FORWARD_TOL = 0.07  # m forward/back
GRASP_TIMEOUT_S = 90.0
GRASP_CUE_GAP_S = 1.2     # s between spoken cues
# "save this place as kitchen" and its variants: remember the current position under a name
PLACE_COMMANDS = ("save this place as ", "remember this place as ", "save place ", "mark ")

@contextlib.contextmanager
def suppress_stderr():
    """Suppress C-level stderr (e.g. ALSA/JACK errors from PyAudio)."""
    fd = sys.stderr.fileno()
    old_fd = os.dup(fd)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, fd)
    try:
        yield
    finally:
        os.dup2(old_fd, fd)
        os.close(old_fd)
        os.close(devnull)

class FindObjectNode(Node):
    def __init__(self):
        super().__init__('voice_navigation_assistant')
        
        self._marker_names = {}  # (ns, id) -> object name, so Marker.DELETE can forget the object
        self._marker_sub = self.create_subscription(MarkerArray, '/semantic_markers', self._marker_callback, 10)
        
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self._path_pub = self.create_publisher(Path, '/object_path', 10)
        self._describe_cmd_pub = self.create_publisher(String, '/describe_command', 10)
        self._haptic_pub = self.create_publisher(String, HAPTIC_TOPIC, 10)  # "last inch" wristband cue

        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.saved_objects = {}
        self.map_data = None
        self.last_found_object = None
        self.navigating = False  # True when actively guiding user
        self._using_nav2 = False
        self.last_hazard_time = 0.0
        
        # Nav2 semantic navigation (semantic_navigator.py): send the target, receive path + status
        self._semantic_goal_pub = self.create_publisher(String, '/semantic_goal', 10)
        self._nav2_status = None
        self._nav2_path = []  # [(x, y)] in map frame
        self.create_subscription(String, '/semantic_nav_status', self._nav2_status_callback, 10)
        self.create_subscription(Path, '/object_path', self._nav2_path_callback, 10)

        # Saved maps and named places (map_manager.py)
        self.named_places = {}  # {"kitchen": {"x": .., "y": ..}}
        self._map_cmd_pub = self.create_publisher(String, '/map_command', 10)
        self.create_subscription(String, '/named_places', self._places_callback, map_qos)
        self.create_subscription(String, '/map_command_result', lambda m: self.speak(m.data), 10)

        # Grasp mode (object_perception tracks the hand and the object, this node speaks the cues)
        self.grasping = False
        self._grasp_status = None
        self._grasp_cmd_pub = self.create_publisher(String, '/grasp_command', 10)
        self.create_subscription(String, '/grasp_offset', self._grasp_callback, 10)

        # Listen for Emergency Hazards from vision node
        self._hazard_sub = self.create_subscription(String, '/hazard_warning', self._hazard_callback, 10)
        
        self.get_logger().info("Find Object Node Started! Waiting for AI to map objects...")
        
        # --- Keyboard Integration (Voice Temporarily Disabled) ---
        self.command_queue = queue.Queue()
        
        self.thread = threading.Thread(target=self.main_logic_loop)
        self.thread.daemon = True
        self.thread.start()

    def speak(self, text):
        """Speak text aloud using Piper TTS female voice."""
        print(f"🔊 Speaking: '{text}'")
        model_path = TTS_MODEL
        wav_path = os.path.join(TMP_DIR, "temp_voice.wav")
        command = f"echo '{text}' | piper --model {model_path} --output_file {wav_path} 2>/dev/null && aplay {wav_path} -q 2>/dev/null"
        subprocess.run(command, shell=True)

    def _hazard_callback(self, msg: String):
        """Hazard warnings are disabled here so they don't interrupt your typing."""
        pass

    def _marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.action == 3:
                self.saved_objects.clear()
                self._marker_names.clear()
                continue
            key = (marker.ns, marker.id)
            if marker.action == 2:  # DELETE: the perception node removed this object from the map
                name = self._marker_names.pop(key, None)
                if name is not None:
                    self.saved_objects.pop(name, None)
                continue
            if marker.text:
                # Strip out the details like "(dist:1.2m|H:0.9m)" and "[MEM]" tags so we just get "chair_1"
                obj_name = marker.text.split('(')[0].replace("[MEM]", "").strip().lower()
                self.saved_objects[obj_name] = marker.pose.position
                self._marker_names[key] = obj_name

    def _grasp_callback(self, msg: String):
        try:
            self._grasp_status = json.loads(msg.data)
        except ValueError:
            pass

    def start_grasp(self, obj_class):
        """Guide the hand to the object in a background thread (stops any grasp already running)."""
        self.grasping = False
        time.sleep(0.2)
        threading.Thread(target=self._grasp_loop, args=(obj_class,), daemon=True).start()

    @staticmethod
    def _grasp_cue(status, obj):
        """One short spoken cue: left/right first, then height, then forward."""
        state = status.get("state")
        if state == "no_target":
            return f"I can't see the {obj}. Step back a little, or turn toward it."
        if state == "no_hand":
            return f"Reach out your hand toward the {obj}."
        if state != "tracking":
            return None

        def cm(v):
            return max(5, int(round(abs(v) * 100 / 5.0)) * 5)
        right, up, forward = status["right"], status["up"], status["forward"]
        if abs(right) > GRASP_TOL:
            return f"{'Right' if right > 0 else 'Left'} {cm(right)} centimetres."
        if abs(up) > GRASP_TOL:
            return f"{'Higher' if up > 0 else 'Lower'} {cm(up)} centimetres."
        if abs(forward) > GRASP_FORWARD_TOL:
            return f"{'Forward' if forward > 0 else 'Back'} {cm(forward)} centimetres."
        return None

    def _grasp_loop(self, obj):
        self.grasping, self._grasp_status = True, None
        self._grasp_cmd_pub.publish(String(data=f"start {obj}"))
        print(f"\n✋ GRASP MODE: {obj}   — type 'stop' to cancel")
        self.speak(f"Reach out your hand toward the {obj}.")
        start, last_cue, last_time = time.time(), None, 0.0
        while rclpy.ok() and self.grasping:
            status = self._grasp_status or {}
            state = status.get("state")
            if state == "unavailable":
                self.speak("Hand tracking is not available.")
                break
            if state == "reached":
                self._haptic_pub.publish(String(data="arrived"))
                self.speak(f"Stop. The {obj} is at your hand.")
                break
            if time.time() - start > GRASP_TIMEOUT_S:
                self.speak("Grasp mode stopped.")
                break
            cue = self._grasp_cue(status, obj)
            now = time.time()
            if cue and now - last_time > GRASP_CUE_GAP_S and (cue != last_cue or now - last_time > 4.0):
                print(f"  ✋ {cue}")
                self.speak(cue)
                last_cue, last_time = cue, time.time()
            time.sleep(0.1)
        self._grasp_cmd_pub.publish(String(data="stop"))
        self.grasping = False

    def _places_callback(self, msg: String):
        try:
            self.named_places = json.loads(msg.data)
        except ValueError:
            pass

    def _where_am_i(self):
        pose = self.get_robot_pose()
        if pose is None or not self.named_places:
            self.speak("I don't know yet. Save places with: save this place as kitchen.")
            return
        name, p = min(self.named_places.items(), key=lambda kv: math.hypot(kv[1]['x'] - pose[0], kv[1]['y'] - pose[1]))
        dist = math.hypot(p['x'] - pose[0], p['y'] - pose[1])
        self.speak(f"You are at the {name}." if dist < 1.5 else f"You are {int(dist * 3.28084)} feet from the {name}.")

    def _nav2_status_callback(self, msg: String):
        try:
            self._nav2_status = json.loads(msg.data)
        except ValueError:
            pass

    def _nav2_path_callback(self, msg: Path):
        if self._using_nav2:
            self._nav2_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _nav2_available(self):
        """semantic_navigator.py is running (it subscribes to /semantic_goal)."""
        backend = os.environ.get("WEARABLE_NAV_BACKEND", "auto").lower()
        if backend == "astar":
            return False
        return self.count_subscribers('/semantic_goal') > 0

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def find_match(self, search_term):
        search_key = search_term.replace(" ", "_")
        if search_key in self.saved_objects:
            return search_key
        for key in self.saved_objects.keys():
            if key.startswith(f"{search_key}_"):
                return key
        return None

    def get_robot_pose(self):
        """Get current robot position and heading from TF."""
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return rx, ry, yaw
        except Exception:
            return None

    def get_relative_direction(self, robot_yaw, target_x, target_y, robot_x, robot_y):
        """Calculate clock-face direction and turn instruction."""
        target_angle = math.atan2(target_y - robot_y, target_x - robot_x)
        rel_angle = target_angle - robot_yaw
        while rel_angle > math.pi: rel_angle -= 2 * math.pi
        while rel_angle < -math.pi: rel_angle += 2 * math.pi
        
        clock_hr = int(round(12 - (rel_angle * 6 / math.pi))) % 12
        if clock_hr == 0: clock_hr = 12
        
        return rel_angle, clock_hr

    def voice_listener_loop(self):
        try:
            with suppress_stderr():
                mic = sr.Microphone()
            with mic as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
                while rclpy.ok():
                    try:
                        audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                    except sr.WaitTimeoutError:
                        continue
                    
                    with open("temp_mic.wav", "wb") as f:
                        f.write(audio.get_wav_data())

                    segments, _ = self.whisper_model.transcribe("temp_mic.wav", beam_size=5)
                    text = "".join([segment.text for segment in segments]).strip().lower()
                    text = text.replace(".", "").replace(",", "").replace("?", "")
                    
                    if text:
                        print(f"\n🎤 Voice recognized: '{text}'")
                        self.command_queue.put(text)
        except Exception as e:
            print(f"Microphone error: {e}")

    def keyboard_listener_loop(self):
        import select
        import sys
        while rclpy.ok():
            # Wait up to 1 second for keyboard input without blocking permanently
            i, o, e = select.select([sys.stdin], [], [], 1.0)
            if i:
                cmd = sys.stdin.readline().strip().lower()
                if cmd:
                    self.command_queue.put(cmd)

    def main_logic_loop(self):
        time.sleep(2)
        self.speak("System ready. You can type commands in the terminal.")
        print("\n" + "=" * 50)
        print("⌨️  READY FOR COMMANDS")
        print("  - Type in this terminal (voice temporarily disabled).")
        print("  - Commands: 'find [object]', 'go to [object or place]', 'describe...'")
        print("              'save map', 'save this place as [name]', 'where am i', 'forget place [name]'")
        print("              'grasp [object]' (hand guidance; also starts on arrival at an object)")
        print("=" * 50 + "\n")
        
        # Start input threads
        # threading.Thread(target=self.voice_listener_loop, daemon=True).start()
        threading.Thread(target=self.keyboard_listener_loop, daemon=True).start()
        
        while rclpy.ok():
            try:
                # Get command from either voice or keyboard
                target = self.command_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            
            if target == 'exit' or target == 'stop navigation' or target == 'stop':
                if self.grasping:
                    self.grasping = False
                    self.speak("Grasp mode off.")
                elif self.navigating:
                    self.navigating = False
                    if self._using_nav2:
                        self._semantic_goal_pub.publish(String(data="stop"))
                    self.speak("Navigation stopped.")
                    cancel_path = Path()
                    cancel_path.header.frame_id = "map"
                    cancel_path.header.stamp = self.get_clock().now().to_msg()
                    self._path_pub.publish(cancel_path)
                else:
                    self.speak("Shutting down.")
                    rclpy.shutdown()
                    break
                
            elif target == "describe" or target.startswith("describe ") or target.startswith("what ") or target.startswith("read "):
                self.speak("Asking the vision AI...")
                msg = String()
                if target == "describe":
                    msg.data = "Describe what you see in this image in one sentence."
                else:
                    msg.data = target
                self._describe_cmd_pub.publish(msg)
                
            elif target.startswith(GRASP_COMMANDS):
                prefix = next(p for p in GRASP_COMMANDS if target.startswith(p))
                obj = target[len(prefix):].strip()
                obj = obj[4:] if obj.startswith("the ") else obj
                if obj:
                    self.start_grasp(obj)

            elif target in ("save map", "save the map"):
                self._map_cmd_pub.publish(String(data="save"))

            elif target.startswith(PLACE_COMMANDS):
                prefix = next(p for p in PLACE_COMMANDS if target.startswith(p))
                name = target[len(prefix):].strip()
                if name:
                    self._map_cmd_pub.publish(String(data=f"place {name}"))

            elif target.startswith("forget place "):
                self._map_cmd_pub.publish(String(data=f"forget {target[len('forget place '):].strip()}"))

            elif target in ("where am i", "where am i?"):
                self._where_am_i()

            elif target.startswith("find "):
                search_term = target.replace("find ", "").strip()
                matched = self.find_match(search_term)
                if matched:
                    friendly_name = ''.join(c for c in matched if not c.isdigit()).replace("_", "").strip()
                    self.speak(f"{friendly_name} detected! Say go to {search_term}.")
                    self.last_found_object = matched
                else:
                    self.speak(f"{search_term} has not been seen yet. Keep walking.")
                    
            elif target.startswith("go to "):
                dest_term = target.replace("go to ", "").strip()
                matched = None
                if self.last_found_object and dest_term.replace(" ", "_") in self.last_found_object:
                    matched = self.last_found_object
                else:
                    matched = self.find_match(dest_term)
                
                place = self.named_places.get(dest_term)
                if place is not None and not matched:
                    self.speak(f"Starting navigation to the {dest_term}.")
                    threading.Thread(target=self.navigate_to, args=(dest_term, (place['x'], place['y'])),
                                     daemon=True).start()
                elif matched:
                    friendly_name = matched.replace("_", " ")
                    self.speak(f"Starting navigation to {friendly_name}.")
                    # Launch navigation in a separate thread so input loop stays free
                    nav_thread = threading.Thread(target=self.navigate_to, args=(matched,))
                    nav_thread.daemon = True
                    nav_thread.start()
                    self.last_found_object = None
                else:
                    self.speak(f"I don't know where {dest_term} is.")
            else:
                print("❓ Use: find <object>, go to <object or place>, save map, "
                      "save this place as <name>, where am i")

    # =============================================
    # CONTINUOUS TURN-BY-TURN NAVIGATION (like Google Maps)
    # =============================================
    def navigate_to(self, target_name, place=None):
        """Continuously guide the user to an object, or to a named place (place = (x, y)), by voice."""
        if self._nav2_available():
            self._navigate_nav2(target_name, place)
            return
        self.navigating = True
        friendly_name = target_name if place else \
            ''.join(c for c in target_name if not c.isdigit()).replace("_", "").strip()
        
        if self.map_data is None:
            self.speak("No map available yet.")
            self.navigating = False
            return
        
        if place:
            tx, ty = place
        else:
            target_pos = self.saved_objects.get(target_name)
            if not target_pos:
                self.speak(f"Lost track of {friendly_name}.")
                self.navigating = False
                return
            # Locked once: the route must not follow (or lose) the live detection near the object
            tx, ty = target_pos.x, target_pos.y

        last_instruction = ""
        last_speech_time = 0
        recalc_counter = 0
        current_grid_path = None
        
        print("\n" + "=" * 50)
        print(f"🧭 NAVIGATING TO: {friendly_name}")
        print("   Type 'stop' to cancel navigation")
        print("=" * 50)
        
        while rclpy.ok() and self.navigating:
            pose = self.get_robot_pose()
            if pose is None:
                time.sleep(0.1)
                continue
                
            rx, ry, robot_yaw = pose
            dist_to_target = math.hypot(tx - rx, ty - ry)
            dist_ft = dist_to_target * 3.28084
            
            # ---- ARRIVAL CHECK (SLAM distance only: the object is in the blind spot by now) ----
            if dist_to_target < ARRIVAL_RADIUS:
                self._arrive(friendly_name, place is not None, target_name.rsplit('_', 1)[0].replace('_', ' '))
                break
                
            # ---- RECALCULATE A* PATH every 1.0 second (10 cycles) ----
            if recalc_counter % 10 == 0:
                new_path = self._calculate_path(rx, ry, tx, ty)
                if new_path:
                    current_grid_path = new_path
            recalc_counter += 1
            
            # ---- DYNAMIC PATH PUBLISHING (10 Hz) ----
            # Strip waypoints that we have already passed
            if current_grid_path:
                # Find the closest waypoint to the robot
                min_idx = 0
                min_d = float('inf')
                for i, (gx, gy) in enumerate(current_grid_path):
                    wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
                    d = math.hypot(wx - rx, wy - ry)
                    if d < min_d:
                        min_d = d
                        min_idx = i
                # Keep only waypoints from the closest one onwards
                current_grid_path = current_grid_path[min_idx:]
                # Publish perfectly anchored to the robot's real-time position
                self._publish_path(current_grid_path, rx, ry, tx, ty)
            
            # ---- PURE PURSUIT LOOKAHEAD STEERING ----
            lookahead_dist = max(1.5, min(3.0, dist_to_target * 0.3))
            waypoint_x, waypoint_y = tx, ty
            if current_grid_path and len(current_grid_path) > 1:
                accumulated_dist = 0.0
                prev_wx, prev_wy = self.grid_to_world(current_grid_path[0][0], current_grid_path[0][1], self.map_data.info)
                for i in range(1, len(current_grid_path)):
                    curr_wx, curr_wy = self.grid_to_world(current_grid_path[i][0], current_grid_path[i][1], self.map_data.info)
                    segment_dist = math.hypot(curr_wx - prev_wx, curr_wy - prev_wy)
                    
                    if accumulated_dist + segment_dist >= lookahead_dist:
                        ratio = (lookahead_dist - accumulated_dist) / segment_dist if segment_dist > 0 else 0
                        waypoint_x = prev_wx + ratio * (curr_wx - prev_wx)
                        waypoint_y = prev_wy + ratio * (curr_wy - prev_wy)
                        break
                        
                    accumulated_dist += segment_dist
                    prev_wx, prev_wy = curr_wx, curr_wy
            
            # ---- CALCULATE DIRECTION ----
            rel_angle, clock_hr = self.get_relative_direction(robot_yaw, waypoint_x, waypoint_y, rx, ry)
            abs_angle_deg = abs(math.degrees(rel_angle))
            
            # ---- GENERATE INSTRUCTION ----
            instruction = self._instruction(rel_angle, clock_hr, dist_ft)

            # ---- SPEAK INSTRUCTION ----
            instruction_type = instruction.split(".")[0]
            now = time.time()
            if instruction_type != last_instruction or (now - last_speech_time > 8.0):
                self.speak(instruction)
                last_instruction = instruction_type
                last_speech_time = now
                print(f"  📍 {instruction}")
            
            time.sleep(0.1)
        
        self.navigating = False
        print("\n✅ Navigation ended.\n")

    def _arrive(self, friendly_name, is_place=False, obj_class=None):
        """Arrival cue: haptic 'last inch' event, voice, and clear the drawn path. At an object,
        grasp mode then guides the hand to it."""
        self._haptic_pub.publish(String(data="arrived"))
        self.speak(f"You have arrived at the {friendly_name}." if is_place else
                   f"You have arrived at the {friendly_name}. It is within reach.")
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        if not is_place and obj_class:
            self.start_grasp(obj_class)

    @staticmethod
    def _instruction(rel_angle, clock_hr, dist_ft):
        """Spoken turn-by-turn instruction toward the lookahead point."""
        abs_angle_deg = abs(math.degrees(rel_angle))
        if abs_angle_deg > 45:
            direction = "left" if rel_angle > 0 else "right"
            if dist_ft > 15:
                return f"Turn {direction} to your {clock_hr} o clock. {int(dist_ft)} feet remaining."
            return f"Turn {direction} now. Almost there."
        if abs_angle_deg > 15:
            direction = "slightly left" if rel_angle > 0 else "slightly right"
            if dist_ft > 15:
                return f"Bear {direction}. {int(dist_ft)} feet to go."
            return f"Bear {direction}. Almost there."
        if dist_ft > 30:
            return f"Keep going straight. {int(dist_ft)} feet remaining."
        if dist_ft > 10:
            return f"Continue straight. {int(dist_ft)} feet to go."
        return f"Almost there. {int(dist_ft)} feet."

    def _navigate_nav2(self, target_name, place=None):
        """Guide the user along the smooth Nav2 path from semantic_navigator.py (re-planned every second)."""
        if place:
            friendly_name, (tx, ty) = target_name, place
        else:
            friendly_name = ''.join(c for c in target_name if not c.isdigit()).replace("_", " ").strip()
            target_pos = self.saved_objects.get(target_name)
            if not target_pos:
                self.speak(f"Lost track of {friendly_name}.")
                return
            # Lock the goal in the map frame: Nav2 routes to this fixed point, not the live detection
            tx, ty = target_pos.x, target_pos.y
        self.navigating, self._using_nav2 = True, True
        self._nav2_status, self._nav2_path = None, []
        goal = {"name": target_name.replace(" ", "_"), "x": tx, "y": ty}
        if place:
            goal["place"] = True  # walk to the point itself (no object to stop in front of)
        self._semantic_goal_pub.publish(String(data=json.dumps(goal)))
        print(f"\n🧭 NAVIGATING (Nav2) TO: {friendly_name}   — type 'stop' to cancel")
        last_instruction, last_speech_time, last_problem = "", 0.0, None
        while rclpy.ok() and self.navigating:
            status = self._nav2_status or {}
            state = status.get("state")
            if state == "arrived":
                self._arrive(friendly_name, place is not None, target_name.rsplit('_', 1)[0].replace('_', ' '))
                break
            if state in ("no_path", "blocked", "not_found", "no_planner") and state != last_problem:
                last_problem = state
                self.speak({"no_path": "The way is blocked right now. Please wait.",
                            "blocked": f"There is no free space next to the {friendly_name}.",
                            "not_found": f"I can't see the {friendly_name} on the map any more.",
                            "no_planner": "The path planner is not running."}[state])
            pose = self.get_robot_pose()
            if pose is not None and math.hypot(tx - pose[0], ty - pose[1]) < ARRIVAL_RADIUS:
                self._arrive(friendly_name, place is not None, target_name.rsplit('_', 1)[0].replace('_', ' '))
                break
            path = self._nav2_path
            if pose is None or len(path) < 2:
                time.sleep(0.1)
                continue
            rx, ry, robot_yaw = pose
            # Drop the part of the path already walked, then look ahead along the rest
            nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - rx, path[i][1] - ry))
            ahead = path[nearest:]
            remaining = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(ahead, ahead[1:]))
            lookahead = max(1.0, min(2.5, remaining * 0.3))
            wx, wy = ahead[-1]
            travelled = 0.0
            for a, b in zip(ahead, ahead[1:]):
                seg = math.hypot(b[0] - a[0], b[1] - a[1])
                if travelled + seg >= lookahead:
                    k = (lookahead - travelled) / seg if seg > 0 else 0.0
                    wx, wy = a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1])
                    break
                travelled += seg
            rel_angle, clock_hr = self.get_relative_direction(robot_yaw, wx, wy, rx, ry)
            instruction = self._instruction(rel_angle, clock_hr, remaining * 3.28084)
            kind = instruction.split(".")[0]
            now = time.time()
            if kind != last_instruction or now - last_speech_time > 8.0:
                self.speak(instruction)
                print(f"  📍 {instruction}")
                last_instruction, last_speech_time = kind, now
            time.sleep(0.1)
        self._semantic_goal_pub.publish(String(data="stop"))
        self.navigating, self._using_nav2 = False, False
        print("\n✅ Navigation ended.\n")

    def smooth_path_chaikin(self, path, iterations=3):
        """Smooth a list of points using Chaikin's corner cutting algorithm (Tesla-like curves)."""
        if len(path) <= 2:
            return path
        for _ in range(iterations):
            new_path = [path[0]]
            for i in range(len(path) - 1):
                p0 = path[i]
                p1 = path[i+1]
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                new_path.extend([q, r])
            new_path.append(path[-1])
            path = new_path
        return path

    def _calculate_path(self, rx, ry, tx, ty):
        """Calculate A* path from robot to target."""
        if self.map_data is None:
            return None
        start_grid = self.world_to_grid(rx, ry, self.map_data.info)
        goal_grid = self.world_to_grid(tx, ty, self.map_data.info)
        raw_path = self.a_star(start_grid, goal_grid, self.map_data)
        if raw_path:
            return self.smooth_path_chaikin(raw_path, iterations=3)
        return [start_grid, goal_grid]  # Fallback to straight line if A* fails

    def _publish_path(self, grid_path, rx, ry, tx, ty):
        """Publish the path to RViz, perfectly anchored to the robot and target."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add exact robot position
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(rx)
        pose.pose.position.y = float(ry)
        path.poses.append(pose)
        
        # Add grid waypoints
        for (gx, gy) in grid_path:
            wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            path.poses.append(pose)
            
        # Add exact target position
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(tx)
        pose.pose.position.y = float(ty)
        path.poses.append(pose)
        
        self._path_pub.publish(path)

    def world_to_grid(self, x, y, map_info):
        gx = int((x - map_info.origin.position.x) / map_info.resolution)
        gy = int((y - map_info.origin.position.y) / map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, map_info):
        wx = (gx + 0.5) * map_info.resolution + map_info.origin.position.x
        wy = (gy + 0.5) * map_info.resolution + map_info.origin.position.y
        return wx, wy

    def a_star(self, start_idx, goal_idx, map_msg):
        w = map_msg.info.width
        h = map_msg.info.height
        data = map_msg.data
        
        def is_free(gx, gy, radius=4):
            if gx < radius or gx >= w - radius or gy < radius or gy >= h - radius: return False
            # Check bounding box (square is faster than circle in Python)
            for dy in range(-radius, radius + 1):
                row_idx = (gy + dy) * w
                for dx in range(-radius, radius + 1):
                    val = data[row_idx + gx + dx]
                    if val >= 50:  # Allow -1 (unknown space) to be traversed
                        return False
            return True
            
        def get_cost(gx, gy):
            if not is_free(gx, gy, radius=2):
                return float('inf')
            
            if not is_free(gx, gy, radius=3):
                base_cost = 5.0
            elif not is_free(gx, gy, radius=4):
                base_cost = 3.0
            elif not is_free(gx, gy, radius=5):
                base_cost = 2.0
            elif not is_free(gx, gy, radius=6):
                base_cost = 1.5
            else:
                base_cost = 1.0

            near_landmark = False
            for obj_name, pos in self.saved_objects.items():
                ox, oy = self.world_to_grid(pos.x, pos.y, map_msg.info)
                dist_cells = math.hypot(gx - ox, gy - oy)
                
                is_dynamic = any(k in obj_name for k in ["dynamic", "person", "human"])
                if is_dynamic:
                    if dist_cells < 40:
                        base_cost += 20.0 / (dist_cells + 1.0)
                else:
                    if dist_cells <= 30:
                        near_landmark = True
                        
            if near_landmark:
                base_cost *= 0.70
                
            return base_cost
            
        def line_of_sight(p1, p2):
            x0, y0 = p1
            x1, y1 = p2
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy
            
            while True:
                if not is_free(x0, y0, radius=3): return False
                if x0 == x1 and y0 == y1: break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
            return True

        sx, sy = start_idx
        gx, gy = goal_idx
        
        # Try to find a nearby free spot if start is stuck in a wall
        if not is_free(sx, sy, 3):
            found_free = False
            for r in range(1, 10):
                for dx in range(-r, r+1):
                    for dy in range(-r, r+1):
                        if is_free(sx+dx, sy+dy, 3):
                            sx += dx
                            sy += dy
                            found_free = True
                            break
                    if found_free: break
                if found_free: break
        
        open_set = []
        import heapq
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0}
        best_node = (sx, sy)
        min_dist = math.hypot(sx - gx, sy - gy)

        raw_path = []
        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            dist = math.hypot(cx - gx, cy - gy)
            
            if dist < min_dist:
                min_dist = dist
                best_node = (cx, cy)
                
            if dist <= 3:
                curr = (cx, cy)
                while curr in came_from:
                    raw_path.append(curr)
                    curr = came_from[curr]
                raw_path.reverse()
                break
                
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                nx, ny = cx + dx, cy + dy
                if not is_free(nx, ny, radius=4):
                    continue
                cost = (1.414 if dx != 0 and dy != 0 else 1.0) * get_cost(nx, ny)
                if cost == float('inf'):
                    continue
                
                parent = came_from.get((cx, cy))
                if parent and line_of_sight(parent, (nx, ny)):
                    tentative_g = g_score[parent] + math.hypot(parent[0] - nx, parent[1] - ny)
                    came_from_node = parent
                else:
                    tentative_g = g_score[(cx, cy)] + cost
                    came_from_node = (cx, cy)
                    
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = came_from_node
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_set, (f_score, nx, ny))
                    
        if not raw_path:
            curr = best_node
            while curr in came_from:
                raw_path.append(curr)
                curr = came_from[curr]
            raw_path.reverse()
            
        if len(raw_path) <= 2:
            return raw_path
            
        # Path Smoothing (String Pulling)
        smoothed_path = [raw_path[0]]
        current = raw_path[0]
        for i in range(1, len(raw_path)):
            if not line_of_sight(current, raw_path[i]):
                smoothed_path.append(raw_path[i-1])
                current = raw_path[i-1]
        smoothed_path.append(raw_path[-1])
        
        return smoothed_path

def main(args=None):
    rclpy.init(args=args)
    node = FindObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
