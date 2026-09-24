#!/usr/bin/env python3
"""
scene_describer.py
------------------
Offline Vision-Language Model (VLM): answers any question about what the chest camera sees.

Uses Qwen3-VL 2B *instruct* running 100% locally on the GPU via Ollama. When the user asks a
question ("what colour is the door?", "is there a light switch?"), the node:
  1. Grabs the latest camera frame (un-mirrored, so left/right match the wearer)
  2. Asks the VLM, with instructions to answer briefly and only from what is visible
  3. Speaks the answer aloud via Piper TTS

This is the system's only VLM. The instruct variant answers directly; the plain `qwen3-vl:2b` tag is
the *thinking* variant, which spent its token budget on hidden reasoning and gave empty answers.

First-time setup:
  ollama pull qwen3-vl:2b-instruct
"""

import os
import re
import sys
import subprocess
import tempfile
import time
import threading
import numpy as np

from visionnav.model_paths import model_path

TMP_DIR = tempfile.gettempdir()  # scratch audio
TTS_MODEL = model_path("en_US-lessac-medium.onnx")


# ── Try importing ROS 2 ──
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CompressedImage
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
    model_path = TTS_MODEL
    wav_path = os.path.join(TMP_DIR, "temp_describe.wav")
    safe_text = text.replace("'", "").replace('"', '').replace('\n', ' ')
    if not os.path.exists(model_path):
        return
    command = f"echo '{safe_text}' | piper --model {model_path} --output_file {wav_path} 2>/dev/null && aplay {wav_path} -q 2>/dev/null"
    subprocess.run(command, shell=True)


# ── VLM Model Configuration ──
VLM_MODEL = "qwen3-vl:2b-instruct"
SYSTEM_PROMPT = (
    "You are the eyes of a blind person, looking through a camera on their chest. Answer their "
    "question about this image in one to three short, complete spoken sentences. Only mention what "
    "is clearly visible; if you cannot tell, say so. Give left and right from the wearer's point of "
    "view, and mention anything in their way when it matters for walking."
)
ANSWER_TOKENS = 256
KEEP_ALIVE = "30m"   # keep the model in VRAM between questions
# The Pi's ROS camera stream arrives mirrored (as in object_perception): flip it back so that
# "left" in the answer is the wearer's left. Override with WEARABLE_CAMERA_FLIP=0/1.
FLIP_INPUT = os.environ.get("WEARABLE_CAMERA_FLIP", "1") == "1"
DEFAULT_QUESTION = "Describe what is in front of me."


class OfflineVLM:
    """A vision-language model served by Ollama (4-bit, on the GPU)."""
    
    def __init__(self):
        print(f"🧠 Connecting to Ollama VLM engine (model: {VLM_MODEL})...")
        try:
            # Check if ollama is running
            installed = [m.model for m in ollama.list().models]
        except Exception:
            print("❌ Ollama is not running. Please run: curl -fsSL https://ollama.com/install.sh | sh")
            sys.exit(1)
        if VLM_MODEL not in installed:
            print(f"❌ {VLM_MODEL} is not installed. Please run: ollama pull {VLM_MODEL}")
            sys.exit(1)
            
        print("⏳ Warming up the GPU (loading model into VRAM)... this takes ~30s once.")
        try:
            self._chat([{'role': 'user', 'content': 'test',
                         'images': [self._jpeg(np.zeros((10, 10, 3), dtype=np.uint8))]}], num_predict=1)
        except Exception:
            pass
            
        print(f"✅ GPU Warmed up! Model is now in memory. Ready to describe anything in ~2 seconds.")
    
    @staticmethod
    def _jpeg(image_np) -> bytes:
        return cv2.imencode('.jpg', image_np, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()

    def _chat(self, messages, num_predict=ANSWER_TOKENS):
        """One Ollama chat call with reasoning off (`think` is a top-level argument, not an option)."""
        return ollama.chat(model=VLM_MODEL, messages=messages, keep_alive=KEEP_ALIVE, think=False,
                           options={'num_predict': num_predict, 'temperature': 0.2})

    def describe(self, image_np, question=DEFAULT_QUESTION):
        """Answer a question about a BGR image."""
        try:
            response = self._chat([
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': question, 'images': [self._jpeg(image_np)]},
            ])
        except Exception as e:
            return f"Error connecting to Ollama: {e}"
        answer = re.sub(r'<think>.*?</think>', '', response['message']['content'], flags=re.DOTALL).strip()
        return answer or "I can see the scene but could not answer that. Please ask again."


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
            CompressedImage, "/camera/image_raw/compressed", self._image_callback, realtime_qos
        )
        
        # Listen for describe commands from the user
        self._cmd_sub = self.create_subscription(
            String, "/describe_command", self._cmd_callback, 10
        )
        
        # Publish the VLM's description
        self._desc_pub = self.create_publisher(String, "/scene_description", 10)
        
        self.get_logger().info("Scene Describer ready! Waiting for commands from voice_navigation_assistant...")
    
    def _image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.latest_frame = cv2.flip(frame, 1) if FLIP_INPUT else frame
        except Exception:
            pass
    
    def _cmd_callback(self, msg):
        """Handle commands from other nodes (e.g., voice_navigation_assistant.py)."""
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
    """Run as a ROS 2 node (connects to the live camera topic and allows terminal triggers)."""
    vlm = OfflineVLM()
    rclpy.init()
    node = SceneDescriberNode(vlm)
    
    # Background thread allowing instant frame capture and description by simply pressing ENTER!
    def keyboard_trigger_loop():
        time.sleep(1)
        print("\n" + "="*65)
        print(f"🤖 SCENE DESCRIBER READY (model: {VLM_MODEL})")
        print("   Type a question about what the camera sees and press Enter.")
        print("   (Or just press Enter without typing to get a general description)")
        print("="*65 + "\n")
        while rclpy.ok():
            try:
                user_q = input("\n👉 Ask a question: ")
                if not user_q.strip():
                    user_q = DEFAULT_QUESTION
                
                if node.latest_frame is not None:
                    node._process_question(user_q)
                else:
                    print("⚠️ No camera frame received yet! Check the camera stream.")
            except EOFError:
                break
            except Exception as e:
                print(f"Error: {e}")

    kb_thread = threading.Thread(target=keyboard_trigger_loop, daemon=True)
    kb_thread.start()

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
        print("  With ROS:  ros2 run visionnav scene_describer")
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


def main():
    if len(sys.argv) > 1 and not sys.argv[1].startswith("--ros"):
        main_standalone()
    elif HAS_ROS:
        main_ros()
    else:
        main_standalone()


if __name__ == "__main__":
    main()
