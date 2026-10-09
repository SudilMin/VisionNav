#!/usr/bin/env python3
"""
face_memory.py
==============
Remembers the faces of people the wearer names, and says who is in front of the chest camera (the HAND
button's face mode, voice_navigation_assistant.py). Offline, on the laptop's CPU, ~25 ms a picture:

  YuNet  (models/face_detection_yunet_2023mar.onnx)    finds the faces and their five landmarks
  SFace  (models/face_recognition_sface_2021dec.onnx)  turns an aligned face into 128 numbers; two pictures of
                                                       one person score a cosine of ~0.9, two people ~0.1

Both come from the OpenCV model zoo (https://github.com/opencv/opencv_zoo) and run with OpenCV's own
FaceDetectorYN / FaceRecognizerSF. Unlike the indoor map, faces are kept for another day: the wearer names a
person once, and the next visit they are recognised. Stored only on this laptop, in
~/.visionnav/faces/faces.json ({"Kamal": [[128 numbers], ...]}); "forget face Kamal" deletes one.

A chest camera's pictures are blurred, dim and at an angle, so a wrong name is worse than "I don't know":
  - remembering keeps several clear pictures of the person (sharp, facing the camera, big enough), never one
  - recognising looks at several pictures in a row and needs the best name to be close enough AND clearly ahead
    of the next one; a near miss is said as "might be", not as the name
"""

import json
import math
import os
from collections import namedtuple

import cv2
import numpy as np

from visionnav.model_paths import model_path

DETECTOR = model_path("face_detection_yunet_2023mar.onnx")
RECOGNIZER = model_path("face_recognition_sface_2021dec.onnx")
STORE = os.path.expanduser(os.environ.get("VISIONNAV_FACES", "~/.visionnav/faces/faces.json"))
DETECT_SCORE = 0.8        # YuNet face confidence, to recognise
ENROLL_SCORE = 0.9        # ... and to remember: only a clear face is kept
MIN_FACE_PX = 48          # smaller faces (far away) are too blurred to recognise
ENROLL_FACE_PX = 64       # ... or to remember
MIN_SHARPNESS = 25.0      # variance of the Laplacian of the aligned face (contrast-stretched): less is blurred
MAX_YAW = 0.3             # nose this far off the middle of the eyes (share of the eye distance): turned away
MATCH_COSINE = 0.45       # sure it is this person (SFace's 0.363 is for sharp photographs: it named strangers)
MATCH_MARGIN = 0.06       # ... and the best name beats the next one by this much
GUESS_COSINE = 0.363      # below sure but above this: "might be Kamal"
NOT_THIS_COSINE = 0.3     # "no, not Jordan": Jordan's pictures this close to the face in front are dropped
SAME_PERSON = 0.5         # the pictures taken while remembering must all be of one person
TOP_K = 3                 # a name scores the mean of its closest pictures: one odd picture can't name a stranger
SAMPLES_PER_HOLD = 5      # clear pictures kept each time the wearer names a person
SAMPLES_PER_NAME = 20     # remembering a person again adds pictures of them, up to this many
SAME_PLACE = 0.15         # one face in pictures taken in a row moves less than this (share of the width)

Face = namedtuple("Face", "row feat cx crop")  # YuNet output row, unit feature, centre x in 0..1, aligned face


def _unit(v):
    return v / max(np.linalg.norm(v), 1e-9)


def _area(face):
    return face.row[2] * face.row[3]


def _sharpness(crop):
    g = cv2.normalize(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), None, 0, 255, cv2.NORM_MINMAX)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def _yaw(row):
    """How far the face is turned: the nose tip's offset from the eyes' midpoint along the eye line, in eye
    distances (0 facing the camera, ~0.5 in profile). YuNet landmarks: right eye, left eye, nose, mouth x2."""
    r, l, nose = np.asarray(row[4:6]), np.asarray(row[6:8]), np.asarray(row[8:10])
    eye = l - r
    dist = math.hypot(*eye)
    return abs(float((nose - (r + l) / 2) @ eye)) / max(dist * dist, 1e-6)


def _clear(face):
    """Good enough to remember: confident, big, facing the camera and sharp."""
    return (face.row[14] >= ENROLL_SCORE and min(face.row[2], face.row[3]) >= ENROLL_FACE_PX
            and _yaw(face.row) <= MAX_YAW and _sharpness(face.crop) >= MIN_SHARPNESS)


class FaceMemory:
    def __init__(self, store=STORE):
        self._store = store
        self._det = cv2.FaceDetectorYN.create(DETECTOR, "", (320, 320), DETECT_SCORE, 0.3, 20)
        self._rec = cv2.FaceRecognizerSF.create(RECOGNIZER, "")
        self._people = {}
        try:
            with open(store) as f:
                self._people = {n: [np.asarray(v, np.float32) for v in vs] for n, vs in json.load(f).items()}
        except (OSError, ValueError):
            pass

    def names(self):
        return sorted(self._people)

    def _faces(self, bgr):
        """Faces big enough to use, largest first."""
        h, w = bgr.shape[:2]
        self._det.setInputSize((w, h))
        _, found = self._det.detect(bgr)
        out = []
        for f in found if found is not None else []:
            if min(f[2], f[3]) < MIN_FACE_PX:
                continue
            crop = self._rec.alignCrop(bgr, f)
            out.append(Face(f, _unit(self._rec.feature(crop).ravel()), float((f[0] + f[2] / 2) / w), crop))
        return sorted(out, key=lambda o: -_area(o))

    def _scores(self, feat):
        """{name: mean cosine of its TOP_K pictures closest to this face}."""
        out = {}
        for name, samples in self._people.items():
            close = sorted((float(feat @ v) for v in samples), reverse=True)[:TOP_K]
            out[name] = sum(close) / len(close)
        return out

    @staticmethod
    def _decide(scores):
        """(name if sure else None, best cosine, best name if it is only a guess else None)."""
        ranked = sorted(scores.items(), key=lambda s: -s[1])
        if not ranked:
            return None, -1.0, None
        name, best = ranked[0]
        second = ranked[1][1] if len(ranked) > 1 else -1.0
        if best >= MATCH_COSINE and best - second >= MATCH_MARGIN:
            return name, best, None
        return None, best, name if best >= GUESS_COSINE else None

    def identify(self, frames):
        """Everyone in the pictures (one, or several taken in a row), left to right:
        [(name or None, cosine, centre x 0..1, "might be" name or None)]. Each face's scores are averaged over
        the pictures it is in, so one blurred picture does not decide."""
        if isinstance(frames, np.ndarray):
            frames = [frames]
        seen = [self._faces(img) for img in frames]
        if not any(seen):
            return []
        anchor = max(range(len(seen)), key=lambda i: (len(seen[i]), i))  # the picture showing the most faces
        tracks = [[f] for f in seen[anchor]]
        for i, faces in enumerate(seen):
            if i == anchor:
                continue
            for f in faces:
                j = min(range(len(tracks)), key=lambda j: abs(tracks[j][0].cx - f.cx))
                if abs(tracks[j][0].cx - f.cx) < SAME_PLACE:
                    tracks[j].append(f)
        people = []
        for track in tracks:
            scores = {}
            for f in track:
                for name, s in self._scores(f.feat).items():
                    scores[name] = scores.get(name, 0.0) + s / len(track)
            name, cos, guess = self._decide(scores)
            people.append((name, cos, track[0].cx, guess))
        return sorted(people, key=lambda p: p[2])

    def _main_faces(self, frames, clear):
        """Features of the one face the wearer means (the largest) in the pictures, and a status: "ok", "no_face",
        "several" (two faces of about the same size: which one is meant is not clear) or "unclear" (with
        `clear`: only small, blurred or turned-away faces). At most SAMPLES_PER_HOLD, sharpest first."""
        if isinstance(frames, np.ndarray):
            frames = [frames]
        good, why = [], "no_face"
        for img in frames:
            faces = self._faces(img)
            if not faces:
                continue
            if len(faces) > 1 and _area(faces[1]) > 0.6 * _area(faces[0]):
                why = "several"
                continue
            if clear and not _clear(faces[0]):
                why = why if why == "several" else "unclear"
                continue
            good.append(faces[0])
        if not good:
            return [], why
        # All of one person: a picture unlike the others (someone walked past) is dropped
        mean = _unit(sum(f.feat for f in good))
        good = sorted((f for f in good if float(f.feat @ mean) >= SAME_PERSON), key=lambda f: -_sharpness(f.crop))
        feats = []
        for f in good:
            if all(float(f.feat @ k) < 0.97 for k in feats):  # near-copies add nothing
                feats.append(f.feat)
        return (feats[:SAMPLES_PER_HOLD], "ok") if feats else ([], "several")

    def _take_face(self, feat, names, cosine):
        """Remove the pictures of `names` this close to this face; a name left without pictures is forgotten.
        Returns the names that had it."""
        had = []
        for n in list(names):
            keep = [v for v in self._people[n] if float(feat @ v) < cosine]
            if len(keep) < len(self._people[n]):
                had.append(n)
                if keep:
                    self._people[n] = keep
                else:
                    del self._people[n]
        return had

    def remember(self, name, frames):
        """Remember the largest face in the pictures (one, or several taken in a row) as `name`. A face can have
        only one name: the same face kept under another name (a misheard name) is taken from it. Returns
        (status, [names that had this face]), status "ok", "no_face", "several" or "unclear"."""
        feats, status = self._main_faces(frames, clear=True)
        if not feats:
            return status, []
        key = next((n for n in self._people if n.lower() == name.lower()), name)
        replaced = self._take_face(_unit(sum(feats)), [n for n in self._people if n != key], MATCH_COSINE)
        samples = self._people.setdefault(key, [])
        samples.extend(feats)
        del samples[:-SAMPLES_PER_NAME]
        self._save()
        return "ok", replaced

    def not_this(self, name, frames):
        """"No, not Jordan": the face in front is not `name`. Returns "ok", "unknown_name", "no_match" (it was
        not kept under that name), "no_face" or "several"."""
        key = next((n for n in self._people if n.lower() == name.lower()), None)
        if key is None:
            return "unknown_name"
        feats, status = self._main_faces(frames, clear=False)
        if not feats:
            return status
        if not self._take_face(_unit(sum(feats)), [key], NOT_THIS_COSINE):
            return "no_match"
        self._save()
        return "ok"

    def forget(self, name):
        key = next((n for n in self._people if n.lower() == name.lower()), None)
        if key is None:
            return False
        del self._people[key]
        self._save()
        return True

    def _save(self):
        os.makedirs(os.path.dirname(self._store), exist_ok=True)
        tmp = self._store + ".tmp"
        with open(tmp, "w") as f:
            json.dump({n: [[round(float(x), 5) for x in v] for v in vs] for n, vs in self._people.items()}, f)
        os.replace(tmp, self._store)
