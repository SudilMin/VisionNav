"""
grasp_tracker.py
================
Grasp Mode: lock onto a target object and track the wearer's hand as it reaches for it.

Used by object_perception (which owns the camera frame, the detections and the LiDAR-calibrated
metric depth map). Each frame it:
  1. keeps the target locked: the detection of the target class that overlaps the previous box
     (the largest one at the start); while the hand covers it, the last position is held,
  2. finds the hand with MediaPipe Hands (21 landmarks); the grasp point is between the thumb and
     index fingertips,
  3. puts both in 3D (pixel + metric depth -> base_footprint: x forward, y left, z up) and reports
     how far the hand must still move: right, up and forward, in metres.

Needs `mediapipe` (pip, installed with --no-deps) and models/hand_landmarker.task.
"""

import math
import time

import numpy as np

from visionnav.model_paths import model_path

HAND_MODEL = model_path("hand_landmarker.task")
TARGET_HOLD_S = 3.0      # keep the last target position this long when it is not detected (the hand covers it)
HAND_HOLD_S = 0.5        # a hand missing for a moment keeps its last position
REACH_TOL_SIDE = 0.05    # m: hand is on target left/right and up/down within this...
REACH_TOL_FORWARD = 0.07  # m: ...and this close in depth
SMOOTH = 0.5             # exponential smoothing of the reported offset
GRASP_LANDMARKS = (4, 8)            # thumb tip, index fingertip: the grasp point is between them
DEPTH_LANDMARKS = (0, 5, 9, 13, 17, 4, 8)  # wrist, knuckles and the two tips: their depths are more reliable together


def _iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class GraspTracker:
    def __init__(self, logger):
        self._log = logger
        self._landmarker = None
        self._load_error = None
        self.target = None          # class name being reached for
        self._reset()

    def _reset(self):
        self._box = None            # locked target box (pixels)
        self._target_xyz = None     # target centre in base_footprint
        self._target_seen = 0.0
        self._hand_xyz = None
        self._hand_px = None
        self._hand_seen = 0.0
        self._offset = None
        self._t0 = time.monotonic()

    @property
    def active(self) -> bool:
        return self.target is not None

    def start(self, target: str) -> bool:
        if not self._ensure_landmarker():
            return False
        self.target = target.strip().lower()
        self._reset()
        self._log.info(f"✋ Grasp mode: reaching for the {self.target}")
        return True

    def stop(self):
        if self.target is not None:
            self._log.info("✋ Grasp mode off")
        self.target = None
        self._reset()

    def _ensure_landmarker(self) -> bool:
        if self._landmarker is not None:
            return True
        try:
            from mediapipe.tasks.python import BaseOptions
            from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
            options = HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=HAND_MODEL),
                running_mode=RunningMode.VIDEO, num_hands=2,
                min_hand_detection_confidence=0.5, min_tracking_confidence=0.5)
            self._landmarker = HandLandmarker.create_from_options(options)
            return True
        except Exception as e:  # mediapipe missing, or the model file
            self._load_error = str(e)
            self._log.error(f"Grasp mode unavailable (hand tracker): {e}")
            return False

    # ── per frame ──
    def update(self, frame, dets, depth_at, to_base):
        """One frame. depth_at(us, vs) -> metric optical depths (m) at pixels (nan where unknown);
        to_base(u, v, z) -> 3D point in base_footprint. Returns the status dict to publish."""
        now = time.monotonic()
        h, w = frame.shape[:2]
        self._update_target(dets, depth_at, to_base, now)
        self._update_hand(frame, depth_at, to_base, now, w, h)

        status = {"target": self.target, "t": round(now - self._t0, 2)}
        target_ok = self._target_xyz is not None and now - self._target_seen <= TARGET_HOLD_S
        hand_ok = self._hand_xyz is not None and now - self._hand_seen <= HAND_HOLD_S
        if not target_ok:
            status["state"] = "no_target"
            return status
        if not hand_ok:
            status["state"] = "no_hand"
            status["target_distance"] = round(float(math.hypot(*self._target_xyz[:2])), 2)
            return status

        d = self._target_xyz - self._hand_xyz
        raw = np.array([-d[1], d[2], d[0]])  # right, up, forward
        self._offset = raw if self._offset is None else SMOOTH * raw + (1 - SMOOTH) * self._offset
        right, up, forward = (float(v) for v in self._offset)
        in_box = self._box is not None and self._hand_px is not None and \
            self._box[0] <= self._hand_px[0] <= self._box[2] and self._box[1] <= self._hand_px[1] <= self._box[3]
        reached = abs(forward) < REACH_TOL_FORWARD and (
            in_box or (abs(right) < REACH_TOL_SIDE and abs(up) < REACH_TOL_SIDE))
        status.update({"state": "reached" if reached else "tracking",
                       "right": round(right, 3), "up": round(up, 3), "forward": round(forward, 3),
                       "distance": round(float(np.linalg.norm(self._offset)), 3)})
        return status

    def overlay(self):
        """(target box, hand pixel) for the HUD, or None."""
        if self._box is None:
            return None
        return self._box, self._hand_px

    def _update_target(self, dets, depth_at, to_base, now):
        cands = [d for d in dets if self.target in (d["label"], d["raw_label"])]
        if not cands:
            return  # hold the last position (the hand in front of the object hides it)
        if self._box is not None:
            best = max(cands, key=lambda d: _iou(d["box"], self._box))
            if _iou(best["box"], self._box) < 0.05:
                bx = 0.5 * (self._box[0] + self._box[2])
                best = min(cands, key=lambda d: abs(0.5 * (d["box"][0] + d["box"][2]) - bx))
        else:
            best = max(cands, key=lambda d: (d["box"][2] - d["box"][0]) * (d["box"][3] - d["box"][1]))
        x1, y1, x2, y2 = best["box"]
        mask = best.get("mask")
        if mask is not None:
            ys, xs = np.nonzero(mask[y1:y2, x1:x2])
            us, vs = xs + x1, ys + y1
        else:
            yy, xx = np.mgrid[y1 + (y2 - y1) // 4:y2 - (y2 - y1) // 4, x1 + (x2 - x1) // 4:x2 - (x2 - x1) // 4]
            us, vs = xx.ravel(), yy.ravel()
        if us.size == 0:
            return
        if us.size > 800:
            pick = np.linspace(0, us.size - 1, 800).astype(int)
            us, vs = us[pick], vs[pick]
        z = depth_at(us, vs)
        z = z[np.isfinite(z)]
        if z.size < 5:
            return
        # Nearest surface of the object (percentile), at the box centre: where the hand closes around it
        self._box = best["box"]
        self._target_xyz = to_base(0.5 * (x1 + x2), 0.5 * (y1 + y2), float(np.percentile(z, 30)))
        self._target_seen = now

    def _update_hand(self, frame, depth_at, to_base, now, w, h):
        import mediapipe as mp
        rgb = np.ascontiguousarray(frame[:, :, ::-1])
        result = self._landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                                   int(now * 1000))
        if not result.hand_landmarks:
            return
        # The hand closest (in the image) to the target is the one reaching for it
        ref = None if self._box is None else (0.5 * (self._box[0] + self._box[2]), 0.5 * (self._box[1] + self._box[3]))
        hands = []
        for lm in result.hand_landmarks:
            gx = np.mean([lm[i].x for i in GRASP_LANDMARKS]) * w
            gy = np.mean([lm[i].y for i in GRASP_LANDMARKS]) * h
            hands.append((gx, gy, lm))
        gx, gy, lm = hands[0] if ref is None else min(hands, key=lambda t: math.hypot(t[0] - ref[0], t[1] - ref[1]))
        us = np.clip(np.array([lm[i].x * w for i in DEPTH_LANDMARKS]).astype(int), 0, w - 1)
        vs = np.clip(np.array([lm[i].y * h for i in DEPTH_LANDMARKS]).astype(int), 0, h - 1)
        z = depth_at(us, vs)
        z = z[np.isfinite(z)]
        if z.size < 3:
            return
        self._hand_px = (int(gx), int(gy))
        self._hand_xyz = to_base(gx, gy, float(np.median(z)))
        self._hand_seen = now
