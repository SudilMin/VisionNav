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
"""

import json
import os

import cv2
import numpy as np

from visionnav.model_paths import model_path

DETECTOR = model_path("face_detection_yunet_2023mar.onnx")
RECOGNIZER = model_path("face_recognition_sface_2021dec.onnx")
STORE = os.path.expanduser(os.environ.get("VISIONNAV_FACES", "~/.visionnav/faces/faces.json"))
MATCH_COSINE = 0.363      # SFace's own threshold for "same person" (cosine of the 128-number features)
DETECT_SCORE = 0.8        # YuNet face confidence
MIN_FACE_PX = 40          # smaller faces (far away) are too blurred to recognise or to remember
SAMPLES_PER_NAME = 10     # remembering a person again adds another picture of them, up to this many


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
        """Faces big enough to use, largest first: (row of YuNet output, unit feature, centre x in 0..1)."""
        h, w = bgr.shape[:2]
        self._det.setInputSize((w, h))
        _, found = self._det.detect(bgr)
        out = []
        for f in found if found is not None else []:
            if min(f[2], f[3]) < MIN_FACE_PX:
                continue
            feat = self._rec.feature(self._rec.alignCrop(bgr, f)).ravel()
            out.append((f, feat / max(np.linalg.norm(feat), 1e-9), float((f[0] + f[2] / 2) / w)))
        return sorted(out, key=lambda o: -o[0][2] * o[0][3])

    def _best(self, feat):
        """(name, cosine) of the closest remembered person, or (None, best cosine)."""
        best, score = None, -1.0
        for name, samples in self._people.items():
            s = max(float(feat @ v) for v in samples)
            if s > score:
                best, score = name, s
        return (best, score) if score >= MATCH_COSINE else (None, score)

    def identify(self, bgr):
        """Everyone in the picture, left to right: [(name or None, cosine, centre x 0..1)]."""
        people = [(*self._best(feat), cx) for _, feat, cx in self._faces(bgr)]
        return sorted(people, key=lambda p: p[2])

    def _main_face(self, bgr):
        """Feature of the one face the wearer means (the largest), or "no_face" / "several" (more than one face
        of about the same size: which one is meant is not clear)."""
        faces = self._faces(bgr)
        if not faces:
            return "no_face"
        if len(faces) > 1 and faces[1][0][2] * faces[1][0][3] > 0.6 * faces[0][0][2] * faces[0][0][3]:
            return "several"
        return faces[0][1]

    def _take_face(self, feat, names):
        """Remove the samples of `names` that match this face; a name left without samples is forgotten.
        Returns the names that had it."""
        had = []
        for n in list(names):
            keep = [v for v in self._people[n] if float(feat @ v) < MATCH_COSINE]
            if len(keep) < len(self._people[n]):
                had.append(n)
                if keep:
                    self._people[n] = keep
                else:
                    del self._people[n]
        return had

    def remember(self, name, bgr):
        """Remember the largest face in the picture as `name`. A face can have only one name: the same face kept
        under another name (a misheard name) is taken from it. Returns (status, [names that had this face]),
        status "ok", "no_face" or "several"."""
        feat = self._main_face(bgr)
        if isinstance(feat, str):
            return feat, []
        key = next((n for n in self._people if n.lower() == name.lower()), name)
        replaced = self._take_face(feat, [n for n in self._people if n != key])
        samples = self._people.setdefault(key, [])
        samples.append(feat)
        del samples[:-SAMPLES_PER_NAME]
        self._save()
        return "ok", replaced

    def not_this(self, name, bgr):
        """"No, not Jordan": the face in front is not `name`. Returns "ok", "unknown_name", "no_match" (it was
        not kept under that name), "no_face" or "several"."""
        key = next((n for n in self._people if n.lower() == name.lower()), None)
        if key is None:
            return "unknown_name"
        feat = self._main_face(bgr)
        if isinstance(feat, str):
            return feat
        if not self._take_face(feat, [key]):
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
