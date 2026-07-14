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
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

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
        
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.saved_objects = {}
        self.map_data = None
        self.last_found_object = None
        self.navigating = False  # True when actively guiding user
        
        self.get_logger().info("Find Object Node Started! Waiting for AI to map objects...")
        
        self.thread = threading.Thread(target=self.input_loop)
        self.thread.daemon = True
        self.thread.start()

    def speak(self, text):
        """Speak text aloud using Piper TTS female voice."""
        print(f"🔊 Speaking: '{text}'")
        model_path = os.path.join(SCRIPT_DIR, "en_US-lessac-medium.onnx")
        wav_path = os.path.join(SCRIPT_DIR, "temp_voice.wav")
        command = f"echo '{text}' | piper --model {model_path} --output_file {wav_path} 2>/dev/null && aplay {wav_path} -q 2>/dev/null"
        subprocess.run(command, shell=True)

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

    def input_loop(self):
        time.sleep(2)
        self.speak("System ready. Drive around to detect objects.")
        
        while rclpy.ok():
            if not self.saved_objects:
                time.sleep(1)
                continue
            
            if self.navigating:
                time.sleep(0.5)
                continue
                
            print("\n" + "=" * 40)
            print("🔍 DETECTED OBJECTS:")
            for obj in self.saved_objects.keys():
                print(f"  ✅ {obj}")
            print("=" * 40)
            
            target = input("\n🗣️ Command (find / go to / exit): ").strip().lower()
            
            if target == 'exit':
                self.speak("Shutting down.")
                rclpy.shutdown()
                break
                
            if target.startswith("find "):
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
        recalc_counter = 0
        arrival_threshold = 1.5  # meters — "you have arrived"
        
        print("\n" + "=" * 50)
        print(f"🧭 NAVIGATING TO: {friendly_name}")
        print("   Type 'stop' to cancel navigation")
        print("=" * 50)
        
        while rclpy.ok() and self.navigating:
            pose = self.get_robot_pose()
            if pose is None:
                time.sleep(0.5)
                continue
                
            rx, ry, robot_yaw = pose
            dist_to_target = math.hypot(tx - rx, ty - ry)
            dist_ft = dist_to_target * 3.28084
            
            # ---- ARRIVAL CHECK ----
            if dist_to_target < arrival_threshold:
                self.speak(f"You have arrived at {friendly_name}. It should be within reach.")
                self._path_pub.publish(Path(header=PoseStamped().header))  # Clear path
                empty_path = Path()
                empty_path.header.frame_id = 'map'
                self._path_pub.publish(empty_path)
                break
            
            # ---- RECALCULATE A* PATH every 3 cycles (1.5 seconds) ----
            recalc_counter += 1
            if recalc_counter % 3 == 1:
                grid_path = self._calculate_path(rx, ry, tx, ty)
                if grid_path:
                    self._publish_path(grid_path)
            
            # ---- FIND NEXT WAYPOINT (~2m ahead on path) ----
            if grid_path:
                waypoint_x, waypoint_y = tx, ty
                for (gx, gy) in grid_path:
                    wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
                    if math.hypot(wx - rx, wy - ry) >= 2.0:
                        waypoint_x, waypoint_y = wx, wy
                        break
            else:
                waypoint_x, waypoint_y = tx, ty
            
            # ---- CALCULATE DIRECTION ----
            rel_angle, clock_hr = self.get_relative_direction(robot_yaw, waypoint_x, waypoint_y, rx, ry)
            abs_angle_deg = abs(math.degrees(rel_angle))
            
            # ---- GENERATE INSTRUCTION ----
            if abs_angle_deg > 45:
                # Big turn needed
                direction = "left" if rel_angle > 0 else "right"
                if dist_ft > 15:
                    instruction = f"Turn {direction} to your {clock_hr} o clock. {int(dist_ft)} feet remaining."
                else:
                    instruction = f"Turn {direction} now. Almost there."
            elif abs_angle_deg > 15:
                # Slight correction
                direction = "slightly left" if rel_angle > 0 else "slightly right"
                if dist_ft > 15:
                    instruction = f"Bear {direction}. {int(dist_ft)} feet to go."
                else:
                    instruction = f"Bear {direction}. Almost there."
            else:
                # Going straight
                if dist_ft > 30:
                    instruction = f"Keep going straight. {int(dist_ft)} feet remaining."
                elif dist_ft > 10:
                    instruction = f"Continue straight. {int(dist_ft)} feet to go."
                else:
                    instruction = f"Almost there. {int(dist_ft)} feet."
            
            # ---- SPEAK ONLY WHEN INSTRUCTION CHANGES ----
            # (Don't spam "go straight" every 0.5 seconds)
            instruction_type = instruction.split(".")[0]  # Compare just the first part
            if instruction_type != last_instruction:
                self.speak(instruction)
                last_instruction = instruction_type
            else:
                # Print silently so terminal shows progress
                print(f"  📍 {instruction}")
            
            time.sleep(0.5)
        
        self.navigating = False
        print("\n✅ Navigation ended.\n")

    def _calculate_path(self, rx, ry, tx, ty):
        """Calculate A* path from robot to target."""
        if self.map_data is None:
            return None
        start_grid = self.world_to_grid(rx, ry, self.map_data.info)
        goal_grid = self.world_to_grid(tx, ty, self.map_data.info)
        return self.a_star(start_grid, goal_grid, self.map_data)

    def _publish_path(self, grid_path):
        """Publish the path to RViz."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for (gx, gy) in grid_path:
            wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
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
        
        def is_free(gx, gy):
            if gx < 0 or gx >= w or gy < 0 or gy >= h: return False
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    if dx*dx + dy*dy > 4:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        val = data[ny * w + nx]
                        if val >= 50 or val == -1:
                            return False
            return True
            
        sx, sy = start_idx
        gx, gy = goal_idx
        
        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0}
        best_node = (sx, sy)
        min_dist = math.hypot(sx - gx, sy - gy)

        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            dist = math.hypot(cx - gx, cy - gy)
            
            if dist < min_dist:
                min_dist = dist
                best_node = (cx, cy)
                
            if dist <= 3:
                path = []
                curr = (cx, cy)
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.reverse()
                return path
                
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                nx, ny = cx + dx, cy + dy
                if not is_free(nx, ny):
                    continue
                cost = 1.414 if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[(cx, cy)] + cost
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_set, (f_score, nx, ny))
                    
        path = []
        curr = best_node
        while curr in came_from:
            path.append(curr)
            curr = came_from[curr]
        path.reverse()
        return path

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
