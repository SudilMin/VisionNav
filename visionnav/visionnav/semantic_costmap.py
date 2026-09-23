#!/usr/bin/env python3
"""
semantic_costmap.py
===================
Projects the 3D semantic object map down into a 2D cost grid for Nav2.

Subscribes: /semantic_objects (JSON from vision_perception.py), /map (Cartographer)
Publishes:  /semantic_map (nav_msgs/OccupancyGrid): the SLAM map with the semantic costs merged in

Nav2's global costmap reads /semantic_map with its single StaticLayer (trinary_costmap: false), so the
cell value 0..100 becomes a graded cost 0..254. (A second StaticLayer for the semantic costs does not
work: in a fixed-size costmap it wipes out the first one.) The merge:
  * SLAM cells: >= 65 % occupied -> wall (100, lethal), < 50 % -> free (0), in between -> 50,
    unknown stays unknown. (Nav2's default only treats exactly 100 as lethal, which would let
    Cartographer's 65-99 % wall cells through.)
  * each object's footprint is lethal (a blind user must never be routed through a chair),
  * a class-specific halo around it adds moderate cost, so paths keep a comfortable clearance
    but can still squeeze past when that is the only way,
  * people are projected along their tracked velocity (Vx, Vy) for a few seconds, so the
    planner routes around where they are going, not only where they are now.
Walls do not need a YOLO class: they come from the SLAM map and get Nav2's inflation layer.
"""

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# class -> (halo radius m, halo cost 0-100). Footprints are always 100 (lethal).
CLASS_COSTS = {
    # Furniture: Nav2's inflation layer already keeps clearance, so only a thin extra margin
    "chair": (0.15, 50), "sofa": (0.15, 50), "couch": (0.15, 50), "bed": (0.15, 50),
    "table": (0.20, 60), "dining table": (0.20, 60), "toilet": (0.15, 50),
    "refrigerator": (0.15, 50), "oven": (0.15, 50), "plant": (0.20, 60), "potted plant": (0.20, 60),
    # Trip hazards left on the floor: wide, high-cost halo
    "backpack": (0.40, 85), "suitcase": (0.40, 85), "bottle": (0.35, 85), "cup": (0.35, 85),
    "handbag": (0.40, 85), "umbrella": (0.40, 85), "book": (0.35, 80),
    # Pets move unpredictably
    "dog": (0.60, 80), "cat": (0.50, 80),
    # Traversable openings (handled in _open_door)
    "door": (0.0, 0),
    # Drops: lethal footprint plus a wide, near-lethal halo
    "stairs": (0.60, 95), "step": (0.60, 95), "hole in floor": (0.60, 95), "pothole": (0.60, 95),
    "curb": (0.40, 90),
}
DOOR_PASSABLE_COST = 30        # a closed door reads as wall to the LiDAR; make it passable but not preferred
DOOR_MIN_HALF_WIDTH = 0.45     # m
DEFAULT_COST = (0.15, 50)
PERSON_CORE_RADIUS = 0.35      # m, lethal
PERSON_SPACE = (0.55, 55)      # personal space around a standing person
PREDICT_HORIZON = 2.0          # s, how far ahead a walking person's path is projected
PREDICT_STEP = 0.25            # s
MIN_MOVING_SPEED = 0.25        # m/s
ELEVATED_Z = 0.40              # objects standing on a table/shelf are not floor obstacles
PUBLISH_PERIOD = 0.5           # s
SLAM_OCCUPIED = 65             # Cartographer probability (%) treated as a wall
SLAM_FREE = 50                 # below this: free floor


class SemanticCostmap(Node):
    def __init__(self):
        super().__init__('semantic_costmap')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_info = None
        self._slam = None
        self._objects = []
        self._last_shape = None
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched)
        self.create_subscription(String, '/semantic_objects', self._objects_cb, 10)
        self._pub = self.create_publisher(OccupancyGrid, '/semantic_map', latched)
        self.create_timer(PUBLISH_PERIOD, self._publish)
        self.get_logger().info('Semantic costmap ready (/map + /semantic_objects -> /semantic_map)')

    def _map_cb(self, msg: OccupancyGrid):
        self._map_info = msg.info
        slam = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        occupied = slam >= SLAM_OCCUPIED
        # A lone occupied cell with no occupied neighbour is scan noise (walls are lines of cells);
        # inflated, each one would block a 1 m disc, so it only gets a moderate cost.
        padded = np.pad(occupied, 1)
        neighbours = sum(padded[1 + dy:padded.shape[0] - 1 + dy, 1 + dx:padded.shape[1] - 1 + dx]
                         for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx)
        self._slam = np.where(slam < 0, -1,
                              np.where(occupied & (neighbours > 0), 100,
                                       np.where(slam < SLAM_FREE, 0, 50))).astype(np.int8)
        shape = (msg.info.width, msg.info.height, msg.info.resolution,
                 round(msg.info.origin.position.x, 3), round(msg.info.origin.position.y, 3))
        if shape != self._last_shape:
            # Nav2's static layer resizes the costmap to this grid: follow /map immediately
            self._last_shape = shape
            self._publish()

    def _objects_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get('frame', 'map') == 'map':
            self._objects = data.get('objects', [])

    def _paint_disc(self, grid, x, y, radius, value):
        info = self._map_info
        res = info.resolution
        cx = (x - info.origin.position.x) / res
        cy = (y - info.origin.position.y) / res
        r = radius / res
        x0, x1 = max(0, int(cx - r)), min(info.width, int(cx + r) + 1)
        y0, y1 = max(0, int(cy - r)), min(info.height, int(cy + r) + 1)
        if x0 >= x1 or y0 >= y1:
            return
        yy, xx = np.mgrid[y0:y1, x0:x1]
        inside = (xx + 0.5 - cx) ** 2 + (yy + 0.5 - cy) ** 2 <= r * r
        view = grid[y0:y1, x0:x1]
        np.maximum(view, np.where(inside, value, -1).astype(np.int8), out=view)

    def _open_door(self, grid, obj):
        """Wall cells inside a detected door's footprint become passable (never touching free cells)."""
        info = self._map_info
        res = info.resolution
        r = max(DOOR_MIN_HALF_WIDTH, 0.5 * obj.get('w', 0.9)) / res
        cx = (obj['x'] - info.origin.position.x) / res
        cy = (obj['y'] - info.origin.position.y) / res
        x0, x1 = max(0, int(cx - r)), min(info.width, int(cx + r) + 1)
        y0, y1 = max(0, int(cy - r)), min(info.height, int(cy + r) + 1)
        if x0 >= x1 or y0 >= y1:
            return
        yy, xx = np.mgrid[y0:y1, x0:x1]
        inside = (xx + 0.5 - cx) ** 2 + (yy + 0.5 - cy) ** 2 <= r * r
        view = grid[y0:y1, x0:x1]
        view[inside & (view >= 50)] = DOOR_PASSABLE_COST

    def _paint_object(self, grid, obj):
        cls = obj.get('class', '')
        if cls == 'door':
            self._open_door(grid, obj)
            return
        x, y = obj['x'], obj['y']
        foot = max(0.10, 0.5 * obj.get('w', 0.3))
        if obj.get('dynamic'):
            speed = math.hypot(obj.get('vx', 0.0), obj.get('vy', 0.0))
            self._paint_disc(grid, x, y, PERSON_CORE_RADIUS, 100)
            self._paint_disc(grid, x, y, PERSON_SPACE[0], PERSON_SPACE[1])
            if speed >= MIN_MOVING_SPEED:
                # Where this person will be over the next seconds, fading with uncertainty
                steps = int(PREDICT_HORIZON / PREDICT_STEP)
                for k in range(1, steps + 1):
                    t = k * PREDICT_STEP
                    cost = int(95 - 50 * t / PREDICT_HORIZON)
                    self._paint_disc(grid, x + obj['vx'] * t, y + obj['vy'] * t,
                                     PERSON_CORE_RADIUS + 0.1 * t, cost)
            return
        if obj.get('z', 0.0) > ELEVATED_Z:
            return  # sits on furniture that is itself on the map
        halo_r, halo_cost = CLASS_COSTS.get(cls, DEFAULT_COST)
        if halo_r > 0:
            self._paint_disc(grid, x, y, foot + halo_r, halo_cost)
        if halo_cost > 0:
            self._paint_disc(grid, x, y, foot, 100)

    def _publish(self):
        if self._map_info is None:
            return
        info = self._map_info
        grid = self._slam.copy()
        for obj in self._objects:
            try:
                self._paint_object(grid, obj)
            except (KeyError, TypeError, ValueError):
                continue
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = info
        msg.data = grid.ravel().tolist()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmap()
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
