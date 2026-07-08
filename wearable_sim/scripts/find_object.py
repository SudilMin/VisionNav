#!/usr/bin/env python3
"""
find_object.py
--------------
Interactive semantic navigation node. It listens for permanent AI object markers
and uses the A* algorithm on the 2D SLAM OccupancyGrid map to calculate the 
shortest collision-free path to the requested object!
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import threading
import math
import heapq
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

class FindObjectNode(Node):
    def __init__(self):
        super().__init__('find_object_node')
        
        self._marker_sub = self.create_subscription(MarkerArray, '/semantic_markers', self._marker_callback, 10)
        
        # SLAM publishes map as Transient Local. We MUST match this!
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self._path_pub = self.create_publisher(Path, '/object_path', 10)
        
        # Publish an empty path immediately so RViz discovers the topic in the "By topic" menu!
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.saved_objects = {}
        self.map_data = None
        
        self.get_logger().info("Find Object Node Started! Waiting for AI to map objects...")
        
        self.thread = threading.Thread(target=self.input_loop)
        self.thread.daemon = True
        self.thread.start()

    def _marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            # If the vision node sends a DELETEALL, wipe our memory
            if marker.action == 3: # Marker.DELETEALL
                self.saved_objects.clear()
                continue
                
            if marker.text:
                obj_name = marker.text.lower()
                self.saved_objects[obj_name] = marker.pose.position

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def input_loop(self):
        import time
        time.sleep(2)
        while rclpy.ok():
            if not self.saved_objects:
                continue
                
            print("\n==============================")
            print("🔍 PERMANENT OBJECTS MAPPED:")
            for obj in self.saved_objects.keys():
                print(f"  - {obj}")
            print("==============================")
            
            target = input("Enter the name of an object to find (or 'exit'): ").strip().lower()
            if target == 'exit':
                rclpy.shutdown()
                break
                
            # If the user types "dining table", convert it to "dining_table" to match our RViz marker text!
            search_key = target.replace(" ", "_")
                
            if search_key in self.saved_objects:
                print(f"✅ Route calculating for '{target}' using A* Obstacle Avoidance...")
                self.draw_path_to(search_key)
            else:
                print(f"❌ '{target}' has not been seen yet! Drive around to map it.")

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
        
        # We dilate obstacles slightly by expanding the cost to avoid wall scraping
        def is_free(gx, gy):
            if gx < 0 or gx >= w or gy < 0 or gy >= h: return False
            
            # Inflate obstacles by 2 cells (0.1 meters) to gently avoid walls without trapping the robot
            for dx in range(-2, 3):
                for dy in range(-2, 3):
                    # Check if within a circle to make smooth corners
                    if dx*dx + dy*dy > 4:
                        continue
                        
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        val = data[ny * w + nx]
                        # >= 50 is an obstacle. -1 is unknown space (grey area).
                        if val >= 50 or val == -1:
                            return False
            return True
            
        sx, sy = start_idx
        gx, gy = goal_idx
        
        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        
        came_from = {}
        g_score = {(sx, sy): 0}
        
        # Track the closest node we've ever seen, so if we fail, we return the closest possible path!
        best_node = (sx, sy)
        min_dist = math.hypot(sx - gx, sy - gy)

        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            
            # Since we inflated walls by 2 cells (0.1m), objects near walls are technically "blocked".
            # Stop pathfinding safely 3 cells (0.15m) in front of the object!
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
                
            # Use 8-connected routing for smooth, optimal, diagonal paths.
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
                    
        # If we exhausted all options and failed, return the path to the closest node we found!
        print("⚠️ Object is trapped inside a wall. Returning closest possible path!")
        path = []
        curr = best_node
        while curr in came_from:
            path.append(curr)
            curr = came_from[curr]
        path.reverse()
        return path

    def draw_path_to(self, target_name):
        if self.map_data is None:
            print("⚠️ No SLAM map received yet!")
            return
            
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            
            target_pos = self.saved_objects[target_name]
            tx, ty = target_pos.x, target_pos.y
            
            start_grid = self.world_to_grid(rx, ry, self.map_data.info)
            goal_grid = self.world_to_grid(tx, ty, self.map_data.info)
            
            grid_path = self.a_star(start_grid, goal_grid, self.map_data)
            
            if not grid_path:
                print("❌ A* Algorithm Failed! The object is completely blocked by walls.")
                return
                
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
            
            # --- NATURAL LANGUAGE NAVIGATION GUIDE ---
            # 1. Calculate robot's current heading (yaw)
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # 2. Pick a local target ~1.5m ahead on the path to guide them towards
            target_wx, target_wy = rx, ry
            for (gx, gy) in grid_path:
                wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
                if math.hypot(wx - rx, wy - ry) >= 1.5:
                    target_wx, target_wy = wx, wy
                    break
            else:
                target_wx, target_wy = tx, ty  # default to the actual object
                
            # 3. Calculate relative angle
            target_angle = math.atan2(target_wy - ry, target_wx - rx)
            rel_angle = target_angle - robot_yaw
            while rel_angle > math.pi: rel_angle -= 2 * math.pi
            while rel_angle < -math.pi: rel_angle += 2 * math.pi
            
            # 4. Convert to clock face (12 = straight, 9 = left, 3 = right)
            clock_hr = int(round(12 - (rel_angle * 6 / math.pi))) % 12
            if clock_hr == 0: clock_hr = 12
            
            final_dist_m = math.hypot(tx - rx, ty - ry)
            final_dist_ft = final_dist_m * 3.28084
            
            print(f"🚀 Optimal A* Path generated! Target is {final_dist_ft:.1f}ft away.")
            print("\n==============================")
            print("🔊 VOICE NAVIGATION GUIDE:")
            if abs(rel_angle) < 0.4:
                print(f"  -> 'Go straight ahead for {final_dist_ft:.1f} feet.'")
            else:
                dir_str = "left" if rel_angle > 0 else "right"
                print(f"  -> 'Turn {dir_str} to your {clock_hr} o'clock direction, then walk {final_dist_ft:.1f} feet.'")
            print("==============================\n")
            
        except Exception as e:
            print(f"⚠️ Could not calculate A* path. Error: {e}")

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
