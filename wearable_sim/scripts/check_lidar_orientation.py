#!/usr/bin/env python3
"""
check_lidar_orientation.py
==========================
Measures how the LiDAR is mounted on the rig, so the map moves the way you move.

If the LiDAR mounting in sensor_tf.launch.py (lidar_yaw_deg / lidar_roll_deg) is wrong, SLAM moves
you the wrong way: e.g. walking backward shows as walking forward in the map.

Guided walk (default, unambiguous), with the Pi's LiDAR running, wearing the rig:
    ros2 run wearable_sim check_lidar_orientation.py
  1. stand still,
  2. walk straight FORWARD about 1 m and stop: the direction the LiDAR moved in its own frame is
     your forward direction -> lidar_yaw_deg,
  3. turn LEFT about 90 deg on the spot: if the LiDAR sees a left turn it is mounted normally,
     if it sees a right turn it is upside down -> lidar_roll_deg 180.
The LiDAR's own motion comes from scan-to-scan ICP; returns closer than MIN_RANGE (your body) are
ignored.

    ros2 run wearable_sim check_lidar_orientation.py --static
compares camera depth with LiDAR ranges for a still rig instead. In a small room several mountings
can fit a still scene about equally well, so use it only as a sanity check.
"""

import math
import os
import sys
import time
import warnings

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
MIN_RANGE = 0.5          # m: closer returns are the wearer's body (they move with the LiDAR)
MAX_RANGE = 8.0
STILL_S, WALK_S, TURN_S = 3.0, 8.0, 8.0
MIN_WALK = 0.4           # m of measured travel needed for a trustworthy heading
MIN_TURN = math.radians(30)


# ── Scan geometry and ICP (pure functions, no ROS) ──
def scan_points(ranges, angle_min, angle_inc):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r > MIN_RANGE) & (r < MAX_RANGE)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def icp(src, dst, iters=40):
    """Rigid 2D transform (R, t) that maps points of scan `src` into the frame of scan `dst`,
    i.e. the pose of the src sensor expressed in the dst sensor frame."""
    R, t = np.eye(2), np.zeros(2)
    if len(src) < 20 or len(dst) < 20:
        return R, t, np.inf
    tree = cKDTree(dst)
    err = np.inf
    for _ in range(iters):
        p = src @ R.T + t
        d, idx = tree.query(p, distance_upper_bound=0.5)
        ok = np.isfinite(d)
        if ok.sum() < 20:
            break
        keep = np.nonzero(ok)[0]
        keep = keep[d[keep] <= np.percentile(d[keep], 80)]      # trimmed ICP: ignore worst matches
        A, B = p[keep], dst[idx[keep]]
        ma, mb = A.mean(0), B.mean(0)
        U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb))
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:
            Vt[1] *= -1
            dR = Vt.T @ U.T
        R, t = dR @ R, dR @ t + (mb - dR @ ma)
        new_err = float(np.sqrt(np.mean(np.sum((A @ dR.T + (mb - dR @ ma) - B) ** 2, axis=1))))
        if abs(err - new_err) < 1e-5:
            err = new_err
            break
        err = new_err
    return R, t, err


def accumulate(scans):
    """LiDAR motion over a list of point sets: (total translation, total rotation) expressed in the
    frame of the first scan."""
    R_tot, t_tot = np.eye(2), np.zeros(2)
    for prev, cur in zip(scans, scans[1:]):
        R, t, _ = icp(cur, prev)
        t_tot = R_tot @ t + t_tot
        R_tot = R_tot @ R
    return t_tot, math.atan2(R_tot[1, 0], R_tot[0, 0])


def mounting_from_motion(walk_translation, turn_angle):
    """Mounting that turns the measured LiDAR-frame motion into 'forward' and 'left turn'.

    Normal mount (yaw psi): walking forward moves the LiDAR along (cos -psi, sin -psi) in its own frame.
    Upside down (roll 180, yaw psi): along (cos psi, sin psi), and a left turn is seen as a right turn.
    """
    heading = math.atan2(walk_translation[1], walk_translation[0])
    mirrored = turn_angle < 0
    yaw = heading if mirrored else -heading
    return round(math.degrees(yaw)) % 360, 180 if mirrored else 0


# ── ROS capture ──
class ScanRecorder(Node):
    def __init__(self):
        super().__init__('check_lidar_orientation')
        self.scans = []
        self.create_subscription(LaserScan, '/scan', self.scans.append, qos_profile_sensor_data)

    def record(self, seconds):
        start = len(self.scans)
        end_t = time.time() + seconds
        while rclpy.ok() and time.time() < end_t:
            rclpy.spin_once(self, timeout_sec=0.05)
        return [scan_points(s.ranges, s.angle_min, s.angle_increment) for s in self.scans[start:]]


def prompt(text, seconds):
    print(f"\n>>> {text}")
    for k in range(3, 0, -1):
        print(f"    starting in {k} ...", flush=True)
        time.sleep(1.0)
    print(f"    GO  (recording {seconds:.0f} s)", flush=True)


def guided_walk():
    rclpy.init()
    node = ScanRecorder()
    t0 = time.time()
    while rclpy.ok() and not node.scans and time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.scans:
        print("No /scan received: is the Pi LiDAR (pi_sensors.launch.py) running?")
        return
    prompt("Stand still, facing forward (the way you normally walk).", STILL_S)
    still = node.record(STILL_S)
    prompt("Walk straight FORWARD about 1 metre, then stop and stand still.", WALK_S)
    walk = node.record(WALK_S)
    prompt("Now turn LEFT (anticlockwise) about 90 degrees on the spot, then stand still.", TURN_S)
    turn = node.record(TURN_S)
    rclpy.shutdown()

    drift, _ = accumulate(still)
    t_walk, _ = accumulate(walk)
    _, a_turn = accumulate(turn)
    print(f"\nMeasured: standing drift {np.hypot(*drift):.2f} m, walked {np.hypot(*t_walk):.2f} m, "
          f"turned {math.degrees(a_turn):+.0f} deg (LiDAR frame, + = anticlockwise)")
    if np.hypot(*t_walk) < MIN_WALK or abs(a_turn) < MIN_TURN:
        print("Not enough motion measured (walk ~1 m, turn ~90 deg, in a room with walls in range). Re-run.")
        return
    yaw, roll = mounting_from_motion(t_walk, a_turn)
    print(f"\nLiDAR mounting:  lidar_yaw_deg:={yaw} lidar_roll_deg:={roll}"
          f"  ({'upside down' if roll else 'upright'})")
    print(f"Start the brain with:  ros2 launch wearable_sim laptop_brain.launch.py "
          f"lidar_yaw_deg:={yaw} lidar_roll_deg:={roll}")
    print("(or make it the default in launch/sensor_tf.launch.py)")


# ── Static camera-vs-LiDAR sanity check ──
def static_check():
    import cv2
    import torch
    import torch.nn.functional as F
    from cv_bridge import CvBridge
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    sys.path.insert(0, _SCRIPT_DIR)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        from depth_anything_v2.dpt import DepthAnythingV2

    hfov = math.radians(float(os.environ.get("WEARABLE_CAMERA_HFOV_DEG", "70.0")))
    flip = os.environ.get("WEARABLE_CAMERA_FLIP", "1") == "1"
    rclpy.init()
    node = Node('check_lidar_orientation')
    bridge, scans, pairs = CvBridge(), [], []
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)

    def on_image(msg):
        if scans and len(pairs) < 4 and (not pairs or time.time() - pairs[-1][2] > 1.0):
            frame = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            pairs.append((cv2.flip(frame, 1) if flip else frame, scans[-1], time.time()))
    node.create_subscription(CompressedImage, '/camera/image_raw/compressed', on_image,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                        history=HistoryPolicy.KEEP_LAST))
    t0 = time.time()
    while rclpy.ok() and len(pairs) < 4 and time.time() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
    if len(pairs) < 4:
        print("Not enough data: are the Pi camera and LiDAR running?")
        return

    model = DepthAnythingV2(encoder='vits', features=64, out_channels=[48, 96, 192, 384], max_depth=20)
    model.load_state_dict(torch.load(os.path.join(_SCRIPT_DIR, 'depth_anything_v2_metric_indoor_vits.pth'),
                                     map_location='cpu'))
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = model.to(dev).eval()
    mean = torch.tensor([0.485, 0.456, 0.406], device=dev).view(1, 3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device=dev).view(1, 3, 1, 1)
    data = []
    for frame, scan, _ in pairs:
        h, w = frame.shape[:2]
        x = torch.from_numpy(frame).to(dev)[..., [2, 1, 0]].permute(2, 0, 1)[None].float() / 255.0
        x = F.interpolate(x, (392, int(round(392 * w / h / 14)) * 14), mode='bilinear', align_corners=False)
        with torch.no_grad():
            dm = F.interpolate(model((x - mean) / std)[:, None], (h, w), mode='bilinear',
                               align_corners=False)[0, 0].cpu().numpy()
        fx = (w / 2) / math.tan(hfov / 2)
        cols = np.arange(8, w - 8, 8)
        theta = np.arctan((w / 2 - cols) / fx)
        cam = np.array([np.median(dm[h // 2 - 10:h // 2 + 60, c]) for c in cols])
        data.append((theta, cam, np.asarray(scan.ranges, float), scan.angle_min, scan.angle_increment))

    def score(sign, yaw_deg):
        out = []
        for theta, cam, ranges, a0, inc in data:
            a = sign * theta - math.radians(yaw_deg)
            idx = np.round((np.arctan2(np.sin(a), np.cos(a)) - a0) / inc).astype(int) % len(ranges)
            lid = np.array([np.median(w_[np.isfinite(w_) & (w_ > 0.3)]) * math.cos(th)
                            if np.any(np.isfinite(w_ := ranges[max(0, i - 2):i + 3]) & (w_ > 0.3)) else np.nan
                            for i, th in zip(idx, theta)])
            ok = np.isfinite(lid) & (cam > 0.2)
            if ok.sum() > 10:
                out.append(np.corrcoef(np.log(cam[ok]), np.log(lid[ok]))[0, 1])
        return float(np.mean(out)) if out else -1.0

    ranked = sorted(((score(s, y), s, y) for s in (1, -1) for y in range(0, 360, 2)), reverse=True)
    print("\nStill-scene fit (camera depth vs LiDAR range; several can fit a small room — "
          "the guided walk decides):")
    shown = []
    for sc, s, y in ranked:
        if all(s != s2 or min(abs(y - y2), 360 - abs(y - y2)) > 20 for _, s2, y2 in shown):
            shown.append((sc, s, y))
        if len(shown) == 4:
            break
    for sc, s, y in shown:
        print(f"  lidar_yaw_deg:={y:3d} lidar_roll_deg:={0 if s == 1 else 180}  correlation {sc:+.2f}")


if __name__ == '__main__':
    static_check() if '--static' in sys.argv else guided_walk()
