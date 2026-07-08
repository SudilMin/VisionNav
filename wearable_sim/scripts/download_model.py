#!/usr/bin/env python3
import os
import urllib.request
import sys

def download_yolov5_onnx():
    url = "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5m.onnx"
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "yolov5m.onnx")
    
    if os.path.exists(model_path):
        print(f"[✅] YOLOv5m already exists at: {model_path}")
        return

    print(f"[⏳] Downloading Highly-Accurate YOLOv5m from Ultralytics...")
    print(f"     URL: {url}")
    
    try:
        urllib.request.urlretrieve(url, model_path)
        print(f"[✅] Successfully downloaded YOLOv5m to: {model_path}")
    except Exception as e:
        print(f"[❌] Failed to download model: {e}")
        sys.exit(1)

if __name__ == '__main__':
    download_yolov5_onnx()
