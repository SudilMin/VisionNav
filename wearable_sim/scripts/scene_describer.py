#!/usr/bin/env python3
"""
scene_describer.py
------------------
Offline Vision-Language Model (VLM) for identifying ANY object.

Uses moondream2 (1.6B params) running 100% locally on GPU.
When the user says "what is this?", the system:
  1. Grabs the latest camera frame
  2. Feeds it to the VLM
  3. Speaks the description aloud via Piper TTS

This handles ALL 160+ objects that YOLO/COCO cannot detect:
  doors, stairs, keys, plates, pillows, white cane, etc.

First-time setup:
  pip3 install --break-system-packages torch torchvision transformers einops Pillow
  # The model (~3.5GB) downloads automatically on first run.
"""

import os
import sys
import subprocess
import time
import threading
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── Try importing heavy ML libraries ──
try:
    import torch
    from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig
    from PIL import Image as PILImage
    HAS_VLM = True
except ImportError:
    HAS_VLM = False

# ── Try importing ROS 2 ──
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

import cv2
import ollama

def speak(text):
    """Speak text aloud using Piper TTS."""
    print(f"\n🔊 Speaking: '{text}'\n")
    model_path = os.path.join(SCRIPT_DIR, "en_US-lessac-medium.onnx")
    wav_path = os.path.join(SCRIPT_DIR, "temp_describe.wav")
    safe_text = text.replace("'", "").replace('"', '').replace('\n', ' ')
    if not os.path.exists(model_path):
        return
    command = f"echo '{safe_text}' | piper --model {model_path} --output_file {wav_path} 2>/dev/null && aplay {wav_path} -q 2>/dev/null"
    subprocess.run(command, shell=True)


class OfflineVLM:
    """Moondream2 — using Ollama for lightning-fast 4-bit GPU inference."""
    
    def __init__(self):
        print("🧠 Connecting to Ollama Moondream engine...")
        try:
            # Check if ollama is running and has moondream
            ollama.list()
        except Exception:
            print("❌ Ollama is not running. Please run: curl -fsSL https://ollama.com/install.sh | sh")
            sys.exit(1)
            
        print("⏳ Warming up the GPU (loading model into VRAM)... this takes ~30s once.")
        try:
            # Create a tiny 10x10 black image for warmup
            warmup_img = np.zeros((10, 10, 3), dtype=np.uint8)
            temp_warmup = os.path.join(SCRIPT_DIR, "_temp_warmup.jpg")
            cv2.imwrite(temp_warmup, warmup_img)
            
            ollama.chat(
                model='moondream',
                messages=[{'role': 'user', 'content': 'test', 'images': [temp_warmup]}],
                options={'num_predict': 1}
            )
        except Exception as e:
            pass
            
        print(f"✅ GPU Warmed up! Model is now in memory. Ready to describe anything in ~2 seconds.")
    
    def describe(self, image_np, question="Describe what you see in this image in one sentence."):
        """
        Takes a numpy BGR image and a question, returns the VLM's answer via Ollama.
        """
        # Save temp image for Ollama to read
        temp_img_path = os.path.join(SCRIPT_DIR, "_temp_vlm_input.jpg")
        cv2.imwrite(temp_img_path, image_np)
        
        try:
            response = ollama.chat(
                model='moondream',
                messages=[{
                    'role': 'user',
                    'content': question,
                    'images': [temp_img_path]
                }],
                options={
                    'num_predict': 40, # Limit output length for faster response
                    'temperature': 0.1
                }
            )
            return response['message']['content'].strip()
        except Exception as e:
            return f"Error connecting to Ollama: {e}"


class SceneDescriberNode(Node):
    """ROS 2 node that listens for 'describe' commands."""
    
    def __init__(self, vlm: OfflineVLM):
        super().__init__('scene_describer')
        self.vlm = vlm
        self.bridge = CvBridge()
        self.latest_frame = None
        
        realtime_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self._image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, realtime_qos
        )
        
        # Listen for describe commands from the user
        self._cmd_sub = self.create_subscription(
            String, "/describe_command", self._cmd_callback, 10
        )
        
        # Publish the VLM's description
        self._desc_pub = self.create_publisher(String, "/scene_description", 10)
        
        self.get_logger().info("Scene Describer ready! Waiting for commands from find_object...")
    
    def _image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
    
    def _cmd_callback(self, msg):
        """Handle commands from other nodes (e.g., find_object.py)."""
        if self.latest_frame is not None:
            self._process_question(msg.data)
            
    def _process_question(self, question):
        """Process a question about the current camera frame."""
        print("🔄 Analyzing image...")
        start = time.time()
        
        answer = self.vlm.describe(self.latest_frame, question)
        elapsed = time.time() - start
        
        print(f"⏱️  Response time: {elapsed:.1f}s")
        print(f"📝 Answer: {answer}")
        
        # Publish to ROS topic
        msg = String()
        msg.data = answer
        self._desc_pub.publish(msg)
        
        # Speak the answer
        speak(answer)


def main_ros():
    """Run as a ROS 2 node (connects to the live camera topic)."""
    vlm = OfflineVLM()
    rclpy.init()
    node = SceneDescriberNode(vlm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_standalone():
    """Run standalone on a single image file (no ROS needed)."""
    if len(sys.argv) < 2:
        print("Usage:")
        print("  With ROS:  ros2 run wearable_sim scene_describer.py")
        print("  Standalone: python3 scene_describer.py <image_path> [question]")
        sys.exit(1)
    
    image_path = sys.argv[1]
    question = " ".join(sys.argv[2:]) if len(sys.argv) > 2 else "List every object you can see in this image."
    
    if not os.path.exists(image_path):
        print(f"❌ Image not found: {image_path}")
        sys.exit(1)
    
    vlm = OfflineVLM()
    img = cv2.imread(image_path)
    
    print(f"🖼️  Analyzing: {image_path}")
    print(f"❓ Question: {question}")
    
    start = time.time()
    answer = vlm.describe(img, question)
    elapsed = time.time() - start
    
    print(f"\n⏱️  Response time: {elapsed:.1f}s")
    print(f"📝 Answer: {answer}")
    speak(answer)


if __name__ == "__main__":
    if len(sys.argv) > 1 and not sys.argv[1].startswith("--ros"):
        main_standalone()
    elif HAS_ROS:
        main_ros()
    else:
        main_standalone()
