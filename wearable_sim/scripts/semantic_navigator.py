#!/usr/bin/env python3
"""
semantic_navigator.py
=====================
Semantic goal routing for a walking user: "chair" -> a smooth, walkable path to just in front of it.

Why not Nav2's NavigateToPose: that action drives a robot base through a local controller
(cmd_vel) and aborts when the "robot" does not follow its velocity commands. Here the "robot" is a
person, so this node uses the parts of Nav2 that fit a human:
  * Straight line first: when the floor between the user and the stopping point in front of the
    object is clear the whole way, that line is walked directly - no planner, no curve.
  * planner_server  (ComputePathToPose)  - Theta* over map + live scan + semantic costs
  * smoother_server (SmoothPath)          - SimpleSmoother, rounds corners, no car-like turning radius
  used only when something other than the target itself blocks the direct line.
and re-plans every second. The chosen goal has hysteresis (moves only when the object moves, the old
spot gets blocked, or a straight line newly opens up), so the path does not flip between two valid
sides of the object. People's predicted paths are in the semantic costmap, so a Theta* re-plan curves
around where they are walking (what a TEB local planner would do for a robot).
find_object.py turns the resulting path into spoken turn-by-turn guidance.

Subscribes: /semantic_goal (String: "chair", "chair_2", "stop", or JSON {"name","x","y"} to pin the
            goal to a locked map coordinate instead of the live detection), /semantic_objects (JSON),
            /global_costmap/costmap (to pick a reachable approach point)
Publishes:  /object_path (nav_msgs/Path), /semantic_nav_status (String, JSON)
"""

import json
import math

import rclpy
import tf2_ros
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, SmoothPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

# The user should end within arm's reach: about 0.25 m between their body and the object's edge.
APPROACH_DIST = 0.45        # m from the object's centre to where the user stops, along the user->object line
MIN_EDGE_CLEARANCE = 0.25   # m from the object's edge (decides for wide objects: tables, sofas)
ARRIVED_RADIUS = 0.15       # m from the stopping point (was 0.35: "arrived" fired ~1 m from a chair)
REPLAN_PERIOD = 1.0         # s
MAX_APPROACH_COST = 90      # costmap value (0-100): anything below Nav2's "inscribed" (99) is standable
APPROACH_ANGLES = sorted(range(-180, 180, 15), key=abs)  # 0 = straight between user and object
ANGLE_PENALTY = 0.4         # cost points per degree away from the side facing the user
PLANNER_ID = 'GridBased'
SMOOTHER_ID = 'human'

# Straight-line approach (see _approach_pose / _line_clear / _choose_goal)
RAY_STEP = 0.05              # m, resolution when walking a line across the costmap
CLEAR_COST = 70              # costmap value (0-100): below this, a line needs no route planning at all
POSITION_ERROR_ALLOWANCE = 0.8  # m a camera-only (no LiDAR hit) sighting can be off by (seen live)
BODY_CLEARANCE = 0.30        # m from an obstacle's surface where Nav2's cost first reaches "blocked"
                             # (body radius 0.25 + inflation), i.e. where a walking ray stops
STOP_MARGIN = 0.05           # m to back off from a blocking cell that turned out to be the target
                             # (the block is already ~0.27 m from its edge: Nav2's body radius + margin)
GOAL_MOVE_TOLERANCE = 0.3    # m: recompute the chosen goal once the object has moved this much


class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')
        self._objects = {}
        self._costmap = None
        self._target = None        # requested name, e.g. "chair" or "chair_2"
        self._target_obj = None    # resolved object dict (sticks to one object once chosen)
        self._pinned = None        # locked {'name','class','x','y','w'} from a memory-anchored goal
        self._busy = False
        self._last_goal = None
        self._last_goal_obj_xy = None
        self._last_goal_straight = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._smoother = ActionClient(self, SmoothPath, 'smooth_path')

        self.create_subscription(String, '/semantic_goal', self._goal_cb, 10)
        self.create_subscription(String, '/semantic_objects', self._objects_cb, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, 1)
        self._path_pub = self.create_publisher(Path, '/object_path', 10)
        self._status_pub = self.create_publisher(String, '/semantic_nav_status', 10)
        self.create_timer(REPLAN_PERIOD, self._tick)
        self.get_logger().info('Semantic navigator ready: publish a class or name on /semantic_goal')

    # ── inputs ──
    def _objects_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get('frame', 'map') == 'map':
            self._objects = {o['name']: o for o in data.get('objects', []) if not o.get('dynamic')}

    def _costmap_cb(self, msg):
        self._costmap = msg

    def _goal_cb(self, msg):
        raw = msg.data.strip()
        pinned = None
        if raw.startswith('{'):
            # Memory-anchored goal: the caller locked the target's map position already (it may be
            # about to leave the chest-mounted sensors' view), so route to that fixed point instead
            # of re-resolving the live detection every tick.
            try:
                data = json.loads(raw)
                name = str(data['name']).strip().lower().replace(' ', '_')
                x, y = float(data['x']), float(data['y'])
            except (ValueError, KeyError, TypeError):
                self._status('requested', 'Bad pinned goal')
                return
            width = self._objects.get(name, {}).get('w', 0.4)
            pinned = {'name': name, 'class': name.rsplit('_', 1)[0], 'x': x, 'y': y, 'w': width}
            request = name
        else:
            request = raw.lower().replace(' ', '_')
            if request in ('', 'stop', 'cancel'):
                self._stop('stopped')
                return
        self._target, self._target_obj, self._pinned = request, None, pinned
        self._status('requested', f'Looking for {request}')
        self._tick()

    # ── helpers ──
    def _status(self, state, text='', **extra):
        self._status_pub.publish(String(data=json.dumps(
            {'state': state, 'target': self._target, 'text': text, **extra})))

    def _stop(self, state, text='', **extra):
        if self._target is not None:
            self._status(state, text, **extra)
        self._target, self._target_obj, self._pinned, self._last_goal = None, None, None, None
        self._last_goal_obj_xy, self._last_goal_straight = None, False
        empty = Path()
        empty.header.frame_id = 'map'
        self._path_pub.publish(empty)

    def _user_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _resolve_target(self, ux, uy):
        """Exact name ("chair_2") or nearest object of the class ("chair"); stays on the same object."""
        if self._pinned is not None:
            return self._pinned
        if self._target_obj is not None and self._target_obj['name'] in self._objects:
            return self._objects[self._target_obj['name']]
        if self._target in self._objects:
            return self._objects[self._target]
        matches = [o for o in self._objects.values()
                   if o['class'].replace(' ', '_') == self._target or o['name'].startswith(self._target + '_')]
        if not matches:
            return None
        return min(matches, key=lambda o: math.hypot(o['x'] - ux, o['y'] - uy))

    def _cost_at(self, x, y):
        cm = self._costmap
        if cm is None:
            return 0
        i = int((x - cm.info.origin.position.x) / cm.info.resolution)
        j = int((y - cm.info.origin.position.y) / cm.info.resolution)
        if not (0 <= i < cm.info.width and 0 <= j < cm.info.height):
            return 100
        v = cm.data[j * cm.info.width + i]
        return 100 if v < 0 else v

    def _approach_pose(self, obj, ux, uy):
        """Where the user should stop: on the line from the object towards the user, facing the object.

        Walks from the user toward the object in short steps (reusing _cost_at) instead of only
        checking the single point APPROACH_DIST from the object's centre. That accepts a straight
        approach even when the object's mapped position is a little off (the ray reaches the
        object's real footprint before the mapped centre); it only treats something else on the
        line - outside the object's _target_zone - as a real obstacle,
        which falls back to the scored ring search around the object.
        """
        radius = max(APPROACH_DIST, 0.5 * obj.get('w', 0.4) + MIN_EDGE_CLEARANCE)
        base = math.atan2(uy - obj['y'], ux - obj['x'])  # object -> user
        dist_to_obj = math.hypot(ux - obj['x'], uy - obj['y'])
        if dist_to_obj <= radius:
            # Already closer than the stopping distance: stop where the user is, facing the object
            return self._make_pose(ux, uy, base + math.pi)

        walk_dist = dist_to_obj - radius  # user -> standoff ring, along the direct user->object line
        dx, dy = (obj['x'] - ux) / dist_to_obj, (obj['y'] - uy) / dist_to_obj  # unit vector, user->object
        steps = max(1, int(round(walk_dist / RAY_STEP)))
        block = None
        for i in range(1, steps + 1):
            d = min(i * RAY_STEP, walk_dist)
            px, py = ux + dx * d, uy + dy * d
            if self._cost_at(px, py) >= MAX_APPROACH_COST:
                block = (px, py, d)
                break

        if block is None:
            # Reached the standoff ring with a clear line the whole way: the direct approach point
            gx, gy = ux + dx * walk_dist, uy + dy * walk_dist
            return self._make_pose(gx, gy, base + math.pi)

        bx, by, bd = block
        if math.hypot(bx - obj['x'], by - obj['y']) <= self._target_zone(obj):
            # The block is the object itself (its mapped position can be a little off) - stop short
            stop_d = max(0.0, bd - STOP_MARGIN)
            gx, gy = ux + dx * stop_d, uy + dy * stop_d
            return self._make_pose(gx, gy, base + math.pi)

        # Something else genuinely blocks the direct line: fall back to the scored ring search
        best = None
        for deg in APPROACH_ANGLES:
            a = base + math.radians(deg)
            gx, gy = obj['x'] + radius * math.cos(a), obj['y'] + radius * math.sin(a)
            cost = self._cost_at(gx, gy)
            if cost > MAX_APPROACH_COST:
                continue
            score = cost + ANGLE_PENALTY * abs(deg)
            if best is None or score < best[0]:
                best = (score, a, gx, gy)
        if best is None:
            return None
        _, a, gx, gy = best
        return self._make_pose(gx, gy, a + math.pi)

    @staticmethod
    def _target_zone(obj):
        """Radius around the object's mapped centre within which a blocking cell is the object itself:
        mapped-position error + half its width + the blocked band in front of its surface."""
        return POSITION_ERROR_ALLOWANCE + 0.5 * obj.get('w', 0.4) + BODY_CLEARANCE

    def _line_clear(self, ax, ay, bx, by, obj=None):
        """True if the segment a->b is a plain walkable floor line that needs no route planning.

        Points must stay under CLEAR_COST, except inside the target object's zone
        (_target_zone), where only MAX_APPROACH_COST applies: the last stretch of
        an approach necessarily enters the target's own safety zone, and that must not push a
        straight approach back through the planner.
        """
        def limit(x, y):
            if obj is not None and math.hypot(x - obj['x'], y - obj['y']) <= self._target_zone(obj):
                return MAX_APPROACH_COST
            return CLEAR_COST

        dist = math.hypot(bx - ax, by - ay)
        if dist < 1e-6:
            return self._cost_at(ax, ay) < limit(ax, ay)
        steps = max(1, int(round(dist / RAY_STEP)))
        for i in range(steps + 1):
            t = i / steps
            x, y = ax + (bx - ax) * t, ay + (by - ay) * t
            if self._cost_at(x, y) >= limit(x, y):
                return False
        return True

    def _choose_goal(self, obj, ux, uy):
        """This tick's approach goal, with hysteresis against the previous one.

        _approach_pose is recomputed fresh every tick (cheap: a few dozen cost lookups), but a
        freshly different goal is only accepted when the old one is no longer any good - it became
        blocked, the object moved more than GOAL_MOVE_TOLERANCE, or a straight line has newly opened
        up where the old goal needed a detour. Otherwise the old goal is kept, so the path does not
        flip between two valid spots (e.g. two sides of the same chair) every re-plan.
        Returns (pose_or_None, straight) where straight means "walk directly, no planner needed".
        """
        pose = self._approach_pose(obj, ux, uy)
        if pose is None:
            return None, False
        gx, gy = pose.pose.position.x, pose.pose.position.y
        straight = self._line_clear(ux, uy, gx, gy, obj)

        prev = self._last_goal
        if prev is not None and self._last_goal_obj_xy is not None:
            lgx, lgy = prev
            moved = math.hypot(obj['x'] - self._last_goal_obj_xy[0], obj['y'] - self._last_goal_obj_xy[1])
            same_spot = math.hypot(gx - lgx, gy - lgy) < 0.05
            old_still_ok = self._cost_at(lgx, lgy) < MAX_APPROACH_COST
            prefer_switch = (moved > GOAL_MOVE_TOLERANCE                       # (b) object moved
                             or not old_still_ok                               # (a) old spot now blocked
                             or (straight and not self._last_goal_straight))   # (c) a straight goal opened up
            if not same_spot and old_still_ok and not prefer_switch:
                base = math.atan2(uy - obj['y'], ux - obj['x'])
                gx, gy = lgx, lgy
                pose = self._make_pose(gx, gy, base + math.pi)
                straight = self._line_clear(ux, uy, gx, gy, obj)

        self._last_goal = (gx, gy)
        self._last_goal_obj_xy = (obj['x'], obj['y'])
        self._last_goal_straight = straight
        return pose, straight

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return pose

    # ── planning loop ──
    def _tick(self):
        if self._target is None or self._busy:
            return
        pose = self._user_pose()
        if pose is None:
            self._status('waiting', 'No map pose yet (is SLAM running?)')
            return
        ux, uy, _ = pose
        obj = self._resolve_target(ux, uy)
        if obj is None:
            self._status('not_found', f'{self._target} is not on the map yet')
            return
        self._target_obj = obj
        goal, straight = self._choose_goal(obj, ux, uy)
        if goal is None:
            self._status('blocked', f'No free space to stand next to {obj["name"]}')
            return
        gx, gy = goal.pose.position.x, goal.pose.position.y
        if math.hypot(gx - ux, gy - uy) < ARRIVED_RADIUS:
            self._stop('arrived', f'You have arrived at {obj["name"]}', object=obj['name'])
            return
        if straight:
            # The floor between here and the goal is clear the whole way: walk it directly instead
            # of round-tripping through the grid planner and smoother.
            self._publish_straight_path(ux, uy, gx, gy)
            return
        if not self._planner.server_is_ready():
            self._status('no_planner', 'Nav2 planner_server is not running')
            return
        self._busy = True
        request = ComputePathToPose.Goal(goal=goal, planner_id=PLANNER_ID, use_start=False)
        self._planner.send_goal_async(request).add_done_callback(self._plan_accepted)

    def _publish_straight_path(self, ux, uy, gx, gy):
        """Build and publish a plain straight-line path from the user to the goal."""
        dist = math.hypot(gx - ux, gy - uy)
        yaw = math.atan2(gy - uy, gx - ux)
        path = Path()
        n = max(1, int(dist / 0.1))
        for i in range(n + 1):
            t = i / n
            path.poses.append(self._make_pose(ux + (gx - ux) * t, uy + (gy - uy) * t, yaw))
        self._publish_path(path)

    def _plan_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self._busy = False
            return
        handle.get_result_async().add_done_callback(self._plan_done)

    def _plan_done(self, future):
        result = future.result().result
        if not result.path.poses:
            self._busy = False
            self._status('no_path', f'No safe path to {self._target} right now', error=result.error_msg)
            return
        if not self._smoother.server_is_ready():
            self._publish_path(result.path)
            return
        request = SmoothPath.Goal(path=result.path, smoother_id=SMOOTHER_ID,
                                  max_smoothing_duration=Duration(sec=0, nanosec=300_000_000),
                                  check_for_collisions=True)
        raw = result.path
        self._smoother.send_goal_async(request).add_done_callback(lambda f: self._smooth_accepted(f, raw))

    def _smooth_accepted(self, future, raw):
        handle = future.result()
        if not handle.accepted:
            self._publish_path(raw)
            return
        handle.get_result_async().add_done_callback(lambda f: self._smooth_done(f, raw))

    def _smooth_done(self, future, raw):
        """Use the smoothed path only if the smoother reports success; otherwise the planner's raw path
        (already collision-free) is safer than a smoothed one that clips an obstacle (error 503)."""
        result = future.result().result
        ok = result.error_code == 0 and len(result.path.poses) >= 2
        if not ok:
            self.get_logger().debug(f"Smoothing rejected ({result.error_code}), using the planner's path")
        self._publish_path(result.path if ok else raw)

    def _publish_path(self, path):
        self._busy = False
        if self._target is None:
            return  # cancelled while planning
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(path)
        length = sum(math.hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y)
                     for a, b in zip(path.poses, path.poses[1:]))
        self._status('navigating', '', object=self._target_obj['name'], goal=list(self._last_goal),
                     length=round(length, 2))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
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
