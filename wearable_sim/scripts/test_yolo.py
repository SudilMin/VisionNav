#!/usr/bin/env python3
"""
test_yolo.py – Standalone YOLOv5n detection test (no ROS needed).

Loads the ONNX model and runs inference on:
  1. /tmp/vision_frame.jpg  (saved by vision_perception.py from the live camera)
  2. A synthetically generated image (coloured shapes) as a fallback

Usage:
  python3 ~/wearable_ws/src/wearable_sim/scripts/test_yolo.py
"""

import os
import sys
import cv2
import numpy as np

# ── Model path ────────────────────────────────────────────────────────────────
MODEL = os.path.expanduser(
    "~/wearable_ws/install/wearable_sim/lib/wearable_sim/yolov5n.onnx"
)

# ── COCO classes ──────────────────────────────────────────────────────────────
CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

CONF_THRESH = 0.25
NMS_THRESH  = 0.45
INPUT_SIZE  = 640

# ── Load model ────────────────────────────────────────────────────────────────
print(f"OpenCV version : {cv2.__version__}")
print(f"Loading model  : {MODEL}")
if not os.path.isfile(MODEL):
    sys.exit(f"ERROR: model not found at {MODEL}")

net = cv2.dnn.readNetFromONNX(MODEL)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
print("Model loaded OK\n")

# ── Load test image ───────────────────────────────────────────────────────────
CAMERA_FRAME = "/tmp/vision_frame.jpg"
RESULT_PATH  = "/tmp/yolo_result.jpg"

img = cv2.imread(CAMERA_FRAME)
if img is None:
    print(f"[WARN] {CAMERA_FRAME} not found – generating a synthetic test image.")
    # Draw coloured shapes on a white background; YOLO won't detect them as
    # real objects but proves the forward-pass runs without crashing.
    img = np.full((480, 640, 3), 220, dtype=np.uint8)
    cv2.circle(img,  (200, 240), 80,  (0,   128,  0),  -1)   # green circle
    cv2.rectangle(img, (350, 150), (580, 400), (180, 80, 20), -1)  # brown rect
    cv2.putText(img, "Synthetic test (no camera frame)",
                (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
else:
    print(f"Loaded: {CAMERA_FRAME}  shape={img.shape}")

h, w = img.shape[:2]

# ── Preprocess ────────────────────────────────────────────────────────────────
blob = cv2.dnn.blobFromImage(
    img, 1.0 / 255.0, (INPUT_SIZE, INPUT_SIZE),
    mean=(0, 0, 0), swapRB=True, crop=False,
)
net.setInput(blob)

# ── Forward pass ─────────────────────────────────────────────────────────────
print("Running forward pass …")
raw = net.forward(net.getUnconnectedOutLayersNames())[0][0]  # (25200, 85)
print(f"Output shape: {raw.shape}  (25200 anchors × 85 values)")

# ── Decode detections ─────────────────────────────────────────────────────────
scale_x, scale_y = w / INPUT_SIZE, h / INPUT_SIZE
boxes, confs, ids = [], [], []

for det in raw:
    obj_conf = float(det[4])
    if obj_conf < 0.20:
        continue
    cid  = int(np.argmax(det[5:]))
    conf = obj_conf * float(det[5 + cid])
    if conf < CONF_THRESH:
        continue
    cx, cy, bw, bh = (float(det[0]) * scale_x, float(det[1]) * scale_y,
                      float(det[2]) * scale_x, float(det[3]) * scale_y)
    x1, y1 = max(0, int(cx - bw / 2)), max(0, int(cy - bh / 2))
    boxes.append([x1, y1, max(1, int(bw)), max(1, int(bh))])
    confs.append(conf)
    ids.append(cid)

# ── NMS ───────────────────────────────────────────────────────────────────────
indices = []
if boxes:
    idx = cv2.dnn.NMSBoxes(boxes, confs, CONF_THRESH, NMS_THRESH)
    if len(idx) > 0:
        indices = idx.flatten()

print(f"\nDetections after NMS: {len(indices)}")
if not indices:
    print("  (none above threshold – expected for an empty/synthetic scene)")

for i in indices:
    x1, y1, bw, bh = boxes[i]
    label = f"{CLASSES[ids[i]]}: {confs[i]:.0%}"
    print(f"  ✓ {label}  box=[{x1},{y1},{x1+bw},{y1+bh}]")
    cv2.rectangle(img, (x1, y1), (x1 + bw, y1 + bh), (0, 255, 0), 2)
    cv2.putText(img, label, (x1, max(y1 - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

# ── Save result ───────────────────────────────────────────────────────────────
cv2.imwrite(RESULT_PATH, img)
print(f"\nResult saved → {RESULT_PATH}")
print("Open with:  xdg-open /tmp/yolo_result.jpg")
