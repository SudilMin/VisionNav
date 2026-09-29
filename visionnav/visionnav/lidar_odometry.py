#!/usr/bin/env python3
"""
lidar_odometry.py
=================
Outdoor mode's ego-motion: how the wearer moves, from the chest LiDAR alone (no map is saved). Like a car
knowing its own motion, it lets outdoor perception tell a parked car (still) from one driving (moving),
give people and vehicles their true speed instead of a speed relative to the walking wearer, and keep a
short-lived occupancy grid around the wearer steady while they walk and turn.

With the chest IMU (mpu6050_imu.py, /imu/data) the gyro's turn between two scans is the match's rotation guess
(GyroYaw) instead of the last turn rate: a torso turn starting between two scans no longer sends ICP searching
from the wrong heading, and the heading keeps following the gyro while no scan can be matched.

Each scan is matched against a small local submap of the last few keyframe scans (point-to-line ICP with a
robust Huber loss): a scan is matched to where the walls, trunks and poles were a moment ago. Keyframes older
than the last KEYFRAMES are forgotten, so nothing accumulates. In a direction the scene does not constrain
(a long corridor, an open square with one wall) the match keeps the constant-velocity prediction there
instead of sliding.

Publishes nav_msgs/Odometry on /odom (frame odom, child base_footprint), for inspection (ros2 topic echo,
RViz Odometry display). It does not publish TF: outdoors everything is drawn around the wearer (base_footprint).

Indoors Cartographer does this job. Outdoor mode runs ScanOdometry inside object_perception; this node runs it
on its own (ros2 run visionnav lidar_odometry), never together with outdoor mode (both publish /odom).
"""

import math
import threading
from collections import deque

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover
    cKDTree = None

# ── PARAMETERS ──
VOXEL = 0.05              # m, scan and submap downsampling
MIN_RANGE = 0.35          # m: closer returns are the wearer's arms and body
MAX_RANGE = 15.0
KEYFRAME_DIST = 0.25      # m moved, or...
KEYFRAME_YAW = math.radians(8.0)  # ...turned, before the scan becomes a keyframe of the submap
KEYFRAMES = 12            # submap = the last this many keyframes (a few metres of walk), then forgotten
ICP_ITERS = 20
ICP_MAX_DIST = (0.6, 0.3, 0.15)  # m correspondence gate per stage (coarse to fine)
HUBER = 0.05              # m
NORMAL_K = 8              # neighbours for a submap point's line fit
LINEARITY = 0.15          # smallest / largest eigenvalue below this: a line (point-to-line), else point-to-point
DEGENERATE_EIG = 40.0     # translational information (1/m^2, summed weights) below this is unconstrained
MIN_MATCHES = 40          # correspondences for a trusted match
MAX_SPEED = 3.0           # m/s: a match implying faster walking than this is rejected
MAX_YAW_RATE = math.radians(240.0)
VEL_ALPHA = 0.5           # smoothing of the reported velocity
GYRO_KEEP_S = 2.0         # s of gyro samples kept
GYRO_MAX_GAP = 0.1        # s: a longer gap in the gyro stream (Wi-Fi) and the scan falls back to constant velocity


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def voxel_downsample(pts: np.ndarray, voxel: float = VOXEL) -> np.ndarray:
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys[:, 0] * 1_000_003 + keys[:, 1], return_index=True)
    return pts[np.sort(idx)]


class Submap:
    """Points of the recent keyframes (odom frame), a KD-tree, and per-point line normals."""
    def __init__(self, pts: np.ndarray):
        self.pts = voxel_downsample(pts)
        self.tree = cKDTree(self.pts)
        k = min(NORMAL_K, len(self.pts))
        _, nb = self.tree.query(self.pts, k=k)
        nbp = self.pts[nb]                                  # (N, k, 2)
        c = nbp - nbp.mean(axis=1, keepdims=True)
        cov = np.einsum('nki,nkj->nij', c, c) / k
        evals, evecs = np.linalg.eigh(cov)                  # ascending
        self.normals = evecs[:, :, 0]                       # direction of least spread
        self.linear = evals[:, 0] < LINEARITY * np.maximum(evals[:, 1], 1e-9)


class GyroYaw:
    """Turn rate about the vertical from sensor_msgs/Imu, integrated between two scan times.

    The turn about the vertical is the gyro vector projected on the up direction (from the IMU's orientation,
    which knows roll and pitch from gravity), so it needs no mount transform: the board may sit at any angle."""

    def __init__(self):
        self._s = deque()  # (t, yaw rate)
        self._lock = threading.Lock()

    def add_msg(self, msg):
        q, w = msg.orientation, msg.angular_velocity
        up = np.array([2 * (q.x * q.z - q.w * q.y), 2 * (q.w * q.x + q.y * q.z),
                       q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z])
        self.add(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, float(up @ [w.x, w.y, w.z]))

    def add(self, t, yaw_rate):
        with self._lock:
            if self._s and t <= self._s[-1][0]:
                if t < self._s[-1][0] - 1.0:
                    self._s.clear()  # the IMU restarted (clock went back)
                else:
                    return
            self._s.append((t, yaw_rate))
            while self._s[0][0] < t - GYRO_KEEP_S:
                self._s.popleft()

    def delta(self, t0, t1):
        """Turn (rad, left positive) from t0 to t1, or None when the gyro does not cover that time."""
        with self._lock:
            s = np.array(self._s) if len(self._s) >= 2 else None
        if s is None or s[0, 0] > t0 or s[-1, 0] < t1 - GYRO_MAX_GAP:
            return None
        ts, w = s[:, 0], s[:, 1]
        i0, i1 = np.searchsorted(ts, t0), np.searchsorted(ts, t1)
        if np.diff(ts[max(i0 - 1, 0):i1 + 1]).max(initial=0.0) > GYRO_MAX_GAP:
            return None
        area = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(ts))])

        def integral(t):  # past the last sample (it is still on its way): the last rate held
            return float(np.interp(t, ts, area)) + (w[-1] * (t - ts[-1]) if t > ts[-1] else 0.0)
        return integral(t1) - integral(t0)


class ScanOdometry:
    """2-D LiDAR odometry. update(points_in_base_footprint, t, gyro=None) -> (pose (x, y, yaw), velocity, ok).
    gyro: a GyroYaw (same clock as the scan stamps), for the rotation guess."""

    def __init__(self):
        self.pose = np.zeros(3)
        self.vel = np.zeros(3)          # vx, vy (odom), yaw rate
        self.t = None
        self.keyframes = deque(maxlen=KEYFRAMES)
        self.kf_pose = None
        self.submap = None
        self.last_ok = False
        self.degenerate = False
        self.matches = 0

    def reset(self):
        self.__init__()

    def _add_keyframe(self, pts_base):
        R, t = _rot(self.pose[2]), self.pose[:2]
        self.keyframes.append(pts_base @ R.T + t)
        self.kf_pose = self.pose.copy()
        self.submap = Submap(np.vstack(list(self.keyframes)))

    def update(self, pts_base: np.ndarray, t: float, gyro: "GyroYaw | None" = None):
        pts = pts_base[np.hypot(pts_base[:, 0], pts_base[:, 1]) > MIN_RANGE]
        pts = voxel_downsample(pts[np.hypot(pts[:, 0], pts[:, 1]) < MAX_RANGE])
        if self.t is None or self.submap is None:
            self.t = t
            if len(pts) >= MIN_MATCHES:
                self._add_keyframe(pts)
            return self.pose.copy(), self.vel.copy(), False
        dt = t - self.t
        if dt <= 0 or dt > 1.0:
            # out of order, or a long gap (Wi-Fi): start again from here rather than trust a stale prediction
            self.t = t
            self.vel[:] = 0.0
            return self.pose.copy(), self.vel.copy(), False
        guess = self.pose + self.vel * dt
        dyaw = gyro.delta(self.t, t) if gyro is not None else None
        if dyaw is not None:
            guess[2] = self.pose[2] + dyaw
        guess[2] = _wrap(guess[2])
        est, ok = self._icp(pts, guess) if len(pts) >= MIN_MATCHES else (guess, False)
        step = est - self.pose
        step[2] = _wrap(step[2])
        if ok and (math.hypot(step[0], step[1]) / dt > MAX_SPEED or abs(step[2]) / dt > MAX_YAW_RATE):
            ok = False  # a match that jumped (a person filling the view, a mismatch): keep the prediction
            est = guess
            step = est - self.pose
            step[2] = _wrap(step[2])
        new_vel = step / dt
        if not ok:
            new_vel *= 0.8  # coast to a stop rather than drift on
        self.vel = (1 - VEL_ALPHA) * self.vel + VEL_ALPHA * new_vel
        self.pose = est
        self.pose[2] = _wrap(self.pose[2])
        self.t = t
        self.last_ok = ok
        if ok:
            d = math.hypot(*(self.pose[:2] - self.kf_pose[:2]))
            if d > KEYFRAME_DIST or abs(_wrap(self.pose[2] - self.kf_pose[2])) > KEYFRAME_YAW:
                self._add_keyframe(pts)
        return self.pose.copy(), self.vel.copy(), ok

    def _icp(self, src: np.ndarray, guess: np.ndarray):
        x = guess.copy()
        sm = self.submap
        H = None
        for stage, max_d in enumerate(ICP_MAX_DIST):
            for _ in range(ICP_ITERS // len(ICP_MAX_DIST) + 1):
                R, t = _rot(x[2]), x[:2]
                q = src @ R.T + t
                d, j = sm.tree.query(q, distance_upper_bound=max_d)
                m = np.isfinite(d)
                if m.sum() < MIN_MATCHES:
                    self.matches = int(m.sum())
                    return guess, False
                qm, pm, jm = q[m], sm.pts[j[m]], j[m]
                lin = sm.linear[jm]
                # point-to-line rows for points on a line, point-to-point (x and y) rows for the rest
                n = sm.normals[jm]
                rows_J, rows_r = [], []
                dq_dth = np.stack([-(qm[:, 1] - t[1]), qm[:, 0] - t[0]], axis=1)
                if lin.any():
                    nl = n[lin]
                    rows_r.append(np.einsum('ij,ij->i', nl, qm[lin] - pm[lin]))
                    rows_J.append(np.column_stack([nl[:, 0], nl[:, 1], np.einsum('ij,ij->i', nl, dq_dth[lin])]))
                if (~lin).any():
                    e = qm[~lin] - pm[~lin]
                    dd = dq_dth[~lin]
                    one, zero = np.ones(len(e)), np.zeros(len(e))
                    rows_r += [e[:, 0], e[:, 1]]
                    rows_J += [np.column_stack([one, zero, dd[:, 0]]), np.column_stack([zero, one, dd[:, 1]])]
                r = np.concatenate(rows_r)
                J = np.vstack(rows_J)
                w = np.where(np.abs(r) <= HUBER, 1.0, HUBER / np.maximum(np.abs(r), 1e-9))
                H = J.T @ (J * w[:, None])
                g = J.T @ (w * r)
                try:
                    dx = -np.linalg.solve(H + 1e-6 * np.eye(3), g)
                except np.linalg.LinAlgError:
                    return guess, False
                # Unconstrained translation directions keep the prediction (solution remapping)
                ev, V = np.linalg.eigh(H[:2, :2])
                self.degenerate = bool(ev[0] < DEGENERATE_EIG)
                if self.degenerate:
                    keep = V[:, 1:2] @ V[:, 1:2].T if ev[1] >= DEGENERATE_EIG else np.zeros((2, 2))
                    dx[:2] = keep @ dx[:2]
                x = x + dx
                x[2] = _wrap(x[2])
                if np.abs(dx[:2]).max() < 1e-4 and abs(dx[2]) < 1e-4:
                    break
        self.matches = int(m.sum())
        return x, True


# ══════════════════════════════════════════════════════════════════════
# ── ROS NODE ──
# ══════════════════════════════════════════════════════════════════════
def main(args=None):
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan

    class LidarOdometryNode(Node):
        def __init__(self):
            super().__init__('lidar_odometry')
            self._odo = ScanOdometry()
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            self._mount = None  # (R 2x2, t 2) laser -> base_footprint in the scan plane
            self._pub = self.create_publisher(Odometry, '/odom', 10)
            self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
            self._gyro = GyroYaw()
            self.create_subscription(Imu, '/imu/data', self._gyro.add_msg, qos_profile_sensor_data)
            self._n = 0
            self._bad = 0
            self.get_logger().info("LiDAR odometry started (publishes /odom; no map is kept).")

        def _mount_for(self, frame):
            if self._mount is None:
                try:
                    tf = self._tf_buffer.lookup_transform('base_footprint', frame, Time())
                except Exception:
                    return None
                q = tf.transform.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                R3 = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                               [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                               [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                self._mount = (R3[:2, :2], np.array([tf.transform.translation.x, tf.transform.translation.y]))
                self.get_logger().info(f"LiDAR mount from TF: yaw {math.degrees(math.atan2(R3[1, 0], R3[0, 0])):.0f}°")
            return self._mount

        def _scan_cb(self, msg):
            mount = self._mount_for(msg.header.frame_id)
            if mount is None:
                return
            r = np.asarray(msg.ranges, dtype=np.float64)
            a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
            ok = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
            pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1) @ mount[0].T + mount[1]
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pose, vel, good = self._odo.update(pts, t, self._gyro)
            self._n += 1
            self._bad += 0 if good else 1
            if self._n % 300 == 0:
                self.get_logger().info(f"odometry: {100 * (1 - self._bad / 300):.0f}% scans matched, "
                                       f"at ({pose[0]:.1f}, {pose[1]:.1f}) m, {math.degrees(pose[2]):.0f}°"
                                       f"{', corridor-like (one direction unconstrained)' if self._odo.degenerate else ''}")
                self._bad = 0
            o = Odometry()
            o.header.stamp = msg.header.stamp
            o.header.frame_id = 'odom'
            o.child_frame_id = 'base_footprint'
            o.pose.pose.position.x, o.pose.pose.position.y = float(pose[0]), float(pose[1])
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            o.twist.twist.linear.x = float(c * vel[0] + s * vel[1])   # body frame, as nav_msgs expects
            o.twist.twist.linear.y = float(-s * vel[0] + c * vel[1])
            o.twist.twist.angular.z = float(vel[2])
            var = 0.0025 if good else 0.25
            o.pose.covariance[0] = o.pose.covariance[7] = var
            o.pose.covariance[35] = var * 0.1
            o.twist.covariance[0] = o.twist.covariance[7] = var * 4
            o.twist.covariance[35] = var
            self._pub.publish(o)

    rclpy.init(args=args)
    node = LidarOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
