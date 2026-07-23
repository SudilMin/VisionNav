#!/usr/bin/env python3
"""
find_object.py
--------------
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
import sys
import contextlib
import speech_recognition as sr
from faster_whisper import WhisperModel
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

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
        super().__init__('find_object_node')
        
        self._marker_sub = self.create_subscription(MarkerArray, '/semantic_markers', self._marker_callback, 10)
        
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self._path_pub = self.create_publisher(Path, '/object_path', 10)
        self._describe_cmd_pub = self.create_publisher(String, '/describe_command', 10)
        
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.saved_objects = {}
        self.map_data = None
        self.last_found_object = None
        self.navigating = False  # True when actively guiding user
        self.last_hazard_time = 0.0
        
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
        model_path = os.path.join(SCRIPT_DIR, "en_US-lessac-medium.onnx")
        wav_path = os.path.join(SCRIPT_DIR, "temp_voice.wav")
        command = f"echo '{text}' | piper --model {model_path} --output_file {wav_path} 2>/dev/null && aplay {wav_path} -q 2>/dev/null"
        subprocess.run(command, shell=True)

    def _hazard_callback(self, msg: String):
        """Emergency interrupt when a moving hazard is detected."""
        now = time.time()
        # Prevent TTS spam (only alert once every 3 seconds)
        if now - self.last_hazard_time > 3.0:
            self.last_hazard_time = now
            # Hard stop any current navigation
            if self.navigating:
                self.navigating = False
                print("\n🛑 NAVIGATION CANCELED DUE TO EMERGENCY HAZARD 🛑")
                # Clear path from RViz
                self._path_pub.publish(Path(header=PoseStamped().header))
            
            # Yell the warning
            self.speak(msg.data)

    def _marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.action == 3:
                self.saved_objects.clear()
                continue
            if marker.text:
                obj_name = marker.text.lower()
                self.saved_objects[obj_name] = marker.pose.position

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
        print("  - Commands: 'find [object]', 'go to [object]', 'describe...'")
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
                if self.navigating:
                    self.navigating = False
                    self.speak("Navigation stopped.")
                    self._path_pub.publish(Path(header=PoseStamped().header))
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
                
            elif target.startswith("find "):
                search_term = target.replace("find ", "").strip()
                matched = self.find_match(search_term)
                if matched:
                    friendly_name = matched.replace("_", " ")
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
                
                if matched:
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
                print("❓ Use: find <object> or go to <object>")

    # =============================================
    # CONTINUOUS TURN-BY-TURN NAVIGATION (like Google Maps)
    # =============================================
    def navigate_to(self, target_name):
        """Continuously guide the user to the target with voice instructions."""
        self.navigating = True
        friendly_name = target_name.replace("_", " ")
        
        if self.map_data is None:
            self.speak("No map available yet.")
            self.navigating = False
            return
        
        target_pos = self.saved_objects.get(target_name)
        if not target_pos:
            self.speak(f"Lost track of {friendly_name}.")
            self.navigating = False
            return
            
        tx, ty = target_pos.x, target_pos.y
        
        last_instruction = ""
        last_speech_time = 0
        recalc_counter = 0
        arrival_threshold = 0.8  # meters — stop closer to the object
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
            
            # ---- ARRIVAL CHECK ----
            if dist_to_target < arrival_threshold:
                self.speak(f"You have arrived at {friendly_name}. It should be within reach.")
                empty_path = Path()
                empty_path.header.frame_id = 'map'
                self._path_pub.publish(empty_path)
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
            
            # ---- FIND NEXT WAYPOINT (~2m ahead on path) ----
            waypoint_x, waypoint_y = tx, ty
            if current_grid_path:
                for (gx, gy) in current_grid_path:
                    wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
                    if math.hypot(wx - rx, wy - ry) >= 2.0:
                        waypoint_x, waypoint_y = wx, wy
                        break
            
            # ---- CALCULATE DIRECTION ----
            rel_angle, clock_hr = self.get_relative_direction(robot_yaw, waypoint_x, waypoint_y, rx, ry)
            abs_angle_deg = abs(math.degrees(rel_angle))
            
            # ---- GENERATE INSTRUCTION ----
            if abs_angle_deg > 45:
                direction = "left" if rel_angle > 0 else "right"
                if dist_ft > 15:
                    instruction = f"Turn {direction} to your {clock_hr} o clock. {int(dist_ft)} feet remaining."
                else:
                    instruction = f"Turn {direction} now. Almost there."
            elif abs_angle_deg > 15:
                direction = "slightly left" if rel_angle > 0 else "slightly right"
                if dist_ft > 15:
                    instruction = f"Bear {direction}. {int(dist_ft)} feet to go."
                else:
                    instruction = f"Bear {direction}. Almost there."
            else:
                if dist_ft > 30:
                    instruction = f"Keep going straight. {int(dist_ft)} feet remaining."
                elif dist_ft > 10:
                    instruction = f"Continue straight. {int(dist_ft)} feet to go."
                else:
                    instruction = f"Almost there. {int(dist_ft)} feet."
            
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
        return None

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
                    if val >= 50 or val == -1:
                        return False
            return True
            
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
                cost = 1.414 if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[(cx, cy)] + cost
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = (cx, cy)
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
