#!/usr/bin/env python3
"""
structure_mapper.py
===================
Completes the 3D indoor map with the structure YOLO cannot see: walls and other fixed barriers.

The LiDAR SLAM map (/map from Cartographer) already contains every wall, but only as 2D occupied
cells. This node turns them into 3D geometry for RViz, next to the YOLO objects from
vision_perception.py:
  1. Take the occupied cells (Cartographer probability >= 65 %), drop single-cell scan noise, and
     drop cells belonging to YOLO objects (a chair back the LiDAR hits is already drawn as a chair).
  2. Fit straight wall segments with sequential RANSAC, split where a line has gaps (doorways).
  3. Extrude them: segments of WALL_MIN_LENGTH or more become full-height walls; shorter segments
     and leftover blobs become obstacle blocks OBSTACLE_HEIGHT tall (the LiDAR hit them at its
     scan height, so they reach at least that high).

Runs at map rate (every RECOMPUTE_PERIOD s), separately from the camera-rate vision node.

Subscribes: /map (nav_msgs/OccupancyGrid), /semantic_objects (JSON from vision_perception.py)
Publishes:  /structure_markers (visualization_msgs/MarkerArray, frame map)
"""

import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

RECOMPUTE_PERIOD = 2.0       # s
OCCUPIED = 65                # Cartographer probability (%) treated as structure (as semantic_costmap.py)
OBJECT_MASK_MARGIN = 0.15    # m around a YOLO object's footprint whose cells belong to the object
DOOR_MIN_HALF_WIDTH = 0.45   # m: a detected door is at least this wide (half)
ELEVATED_Z = 0.40            # objects standing on furniture do not mask floor-level structure

# Sequential RANSAC line extraction
RANSAC_HYPOTHESES = 200
INLIER_DIST = 0.06           # m from the line
MIN_SEGMENT_POINTS = 6       # cells
MIN_SEGMENT_LENGTH = 0.30    # m
MAX_GAP = 0.25               # m; a larger gap along a line splits it (doorways, openings)
MAX_SEGMENTS = 60

# Extrusion
WALL_MIN_LENGTH = 0.6        # m: at scan height (~1.2 m) anything this long is a wall, door or cupboard
WALL_HEIGHT = 2.4            # m
WALL_THICKNESS = 0.10        # m
OBSTACLE_HEIGHT = 1.3        # m: short structure the LiDAR (scan plane ~1.2 m) hit
MIN_BLOB_CELLS = 2
WALL_COLOR = ColorRGBA(r=0.78, g=0.80, b=0.86, a=0.85)
OBSTACLE_COLOR = ColorRGBA(r=0.50, g=0.52, b=0.58, a=0.85)


def extract_segments(points, rng):
    """Sequential RANSAC: returns ([(p_start, p_end), ...], leftover_mask over points)."""
    remaining = np.arange(len(points))
    used = np.zeros(len(points), dtype=bool)
    segments = []
    for _ in range(MAX_SEGMENTS):
        if remaining.size < MIN_SEGMENT_POINTS:
            break
        pts = points[remaining]
        i = rng.integers(0, len(pts), RANSAC_HYPOTHESES)
        j = rng.integers(0, len(pts), RANSAC_HYPOTHESES)
        d = pts[j] - pts[i]
        length = np.hypot(d[:, 0], d[:, 1])
        valid = length > 0.1
        length[~valid] = 1.0
        nx, ny = -d[:, 1] / length, d[:, 0] / length
        dist = np.abs((pts[None, :, 0] - pts[i, None, 0]) * nx[:, None]
                      + (pts[None, :, 1] - pts[i, None, 1]) * ny[:, None])
        counts = (dist < INLIER_DIST).sum(axis=1)
        counts[~valid] = 0
        best = int(np.argmax(counts))
        if counts[best] < MIN_SEGMENT_POINTS:
            break
        inliers = np.nonzero(dist[best] < INLIER_DIST)[0]
        line_pts = pts[inliers]

        # Refine the direction with PCA, then split the line into gap-free runs
        centre = line_pts.mean(axis=0)
        direction = np.linalg.svd(line_pts - centre)[2][0]
        t = (line_pts - centre) @ direction
        order = np.argsort(t)
        ts = t[order]
        breaks = list(np.nonzero(np.diff(ts) > MAX_GAP)[0])
        start = 0
        for end in breaks + [len(ts) - 1]:
            if end - start + 1 >= MIN_SEGMENT_POINTS and ts[end] - ts[start] >= MIN_SEGMENT_LENGTH:
                segments.append((centre + ts[start] * direction, centre + ts[end] * direction))
                used[remaining[inliers[order[start:end + 1]]]] = True
            start = end + 1
        # This line's points are no longer candidates for other lines (unused ones stay leftovers)
        keep = np.ones(len(remaining), dtype=bool)
        keep[inliers] = False
        remaining = remaining[keep]
    return segments, ~used


class StructureMapper(Node):
    def __init__(self):
        super().__init__('structure_mapper')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map = None
        self._map_changed = False
        self._objects = []
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, latched)
        self.create_subscription(String, '/semantic_objects', self._objects_cb, 10)
        self._pub = self.create_publisher(MarkerArray, '/structure_markers', latched)
        self.create_timer(RECOMPUTE_PERIOD, self._update)
        self.get_logger().info('Structure mapper ready: /map -> 3D walls on /structure_markers')

    def _map_cb(self, msg):
        self._map = msg
        self._map_changed = True

    def _objects_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get('frame', 'map') == 'map':
            self._objects = data.get('objects', [])

    def _occupied_cells(self):
        m = self._map
        info = m.info
        grid = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)
        occ = grid >= OCCUPIED
        # Drop single cells with no occupied neighbour: scan noise, not structure
        padded = np.pad(occ, 1)
        neighbours = sum(padded[1 + dy:padded.shape[0] - 1 + dy, 1 + dx:padded.shape[1] - 1 + dx]
                         for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx)
        occ &= neighbours > 0
        # Cells under YOLO objects belong to the object, not to the building
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        for obj in self._objects:
            is_door = obj.get('class') == 'door'
            # A door is part of the wall to the LiDAR: always carve it out (whatever height its base
            # reads) so no solid 2.4 m wall is extruded across it and routes can pass through.
            if obj.get('z', 0.0) > ELEVATED_Z and not is_door:
                continue
            half_w = max(DOOR_MIN_HALF_WIDTH, 0.5 * obj.get('w', 0.4)) if is_door else 0.5 * obj.get('w', 0.4)
            r = (half_w + OBJECT_MASK_MARGIN) / res
            cx, cy = (obj['x'] - ox) / res, (obj['y'] - oy) / res
            cv2.circle(occ.view(np.uint8), (int(round(cx)), int(round(cy))), int(math.ceil(r)), 0, -1)
        return occ

    def _update(self):
        if self._map is None or not self._map_changed:
            return
        self._map_changed = False
        info = self._map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        occ = self._occupied_cells()
        rows, cols = np.nonzero(occ)
        points = np.stack([ox + (cols + 0.5) * res, oy + (rows + 0.5) * res], axis=1)

        # Deterministic RANSAC: the same map gives the same walls (no flicker between updates)
        segments, leftover = extract_segments(points, np.random.default_rng(0)) if len(points) else ([], None)

        markers = [Marker(action=Marker.DELETEALL)]
        now = self.get_clock().now().to_msg()
        n_walls = 0
        for k, (a, b) in enumerate(segments):
            length = float(np.hypot(*(b - a)))
            is_wall = length >= WALL_MIN_LENGTH
            n_walls += is_wall
            markers.append(self._box(now, 'walls' if is_wall else 'obstacles', k,
                                     (a + b) / 2.0, math.atan2(b[1] - a[1], b[0] - a[0]),
                                     length + res, WALL_THICKNESS if is_wall else max(WALL_THICKNESS, res),
                                     WALL_HEIGHT if is_wall else OBSTACLE_HEIGHT,
                                     WALL_COLOR if is_wall else OBSTACLE_COLOR))

        # Leftover occupied cells that fit no line: small blobs -> obstacle blocks
        n_blobs = 0
        if leftover is not None and leftover.any():
            mask = np.zeros_like(occ, dtype=np.uint8)
            mask[rows[leftover], cols[leftover]] = 1
            count, labels = cv2.connectedComponents(mask, connectivity=8)
            for lab in range(1, count):
                r, c = np.nonzero(labels == lab)
                if r.size < MIN_BLOB_CELLS:
                    continue
                pts = np.stack([ox + (c + 0.5) * res, oy + (r + 0.5) * res], axis=1).astype(np.float32)
                (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
                markers.append(self._box(now, 'obstacles', 1000 + lab, np.array([cx, cy]), math.radians(ang),
                                         max(w, res), max(h, res), OBSTACLE_HEIGHT, OBSTACLE_COLOR))
                n_blobs += 1

        self._pub.publish(MarkerArray(markers=markers))
        self.get_logger().debug(f'{n_walls} walls, {len(segments) - n_walls} short segments, {n_blobs} blobs')

    @staticmethod
    def _box(stamp, ns, mid, centre, yaw, sx, sy, height, color):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns, m.id = ns, int(mid)
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose.position = Point(x=float(centre[0]), y=float(centre[1]), z=height / 2.0)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(height)
        m.color = color
        return m


def main(args=None):
    rclpy.init(args=args)
    node = StructureMapper()
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
