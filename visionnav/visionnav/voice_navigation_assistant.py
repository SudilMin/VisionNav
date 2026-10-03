#!/usr/bin/env python3
"""
voice_navigation_assistant.py
-----------------------------
The laptop's main program: it talks to the wearer and acts on the Pi's buttons and on spoken commands.

  * Buttons (pi_button_panel.py on the Pi, /button_event): mode switching, push-to-talk (Whisper, offline), vision
    AI questions (scene_describer.py), hand guidance (grasp), face memory (face_memory.py).
  * Starts and stops the other programs itself (system_manager.py).
  * Indoor: finds objects on this session's map (object_language.py) and guides the wearer to them turn by turn,
    along the Nav2 path of semantic_navigator.py (built-in A* as a fallback).
  * Outdoor: speaks the hazard alerts of object_perception.py / outdoor_awareness.py.
Speech output: Piper TTS.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import threading
import math
import heapq
import subprocess
import os
import time
import queue
import json
import re
import sys
import contextlib
import tempfile
from faster_whisper import WhisperModel
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from visionnav.model_paths import model_path
from visionnav import object_language as lang
from visionnav import speech_fix
from visionnav.system_manager import SystemManager, MODE_PARTS

TMP_DIR = tempfile.gettempdir()  # scratch audio/images
TTS_MODEL = model_path("en_US-lessac-medium.onnx")
# Memory-anchored arrival: the target's map position is locked when navigation starts (a low object drops below the
# chest sensors in the last metre), and arrival is decided by the SLAM distance to that point.
ARRIVAL_RADIUS = 0.6  # m from the locked point: a named place
OBJECT_ARRIVAL_RADIUS = 0.45  # m from an object's centre
# Grasp mode: guide the hand to an object ("grasp cup"), also started on arrival at an object
GRASP_COMMANDS = ("grasp ", "grab ", "pick up ", "reach for ")
GRASP_TOL = 0.05          # m left/right or up/down still worth a cue
GRASP_FORWARD_TOL = 0.07  # m forward/back
GRASP_TIMEOUT_S = 90.0
GRASP_CUE_GAP_S = 1.2     # s between spoken cues
# "save this place as kitchen" and its variants: remember the current position under a name
PLACE_COMMANDS = ("save this place as ", "remember this place as ", "save place ", "mark ")
# Spoken object requests (object_language.py resolves "the table with the red cup on it"; IDs are never spoken)
FIND_COMMANDS = ("find ", "where is ", "where s ", "wheres ", "locate ", "is there ", "search for ", "look for ")
GO_COMMANDS = ("go to ", "take me to ", "guide me to ", "navigate to ", "bring me to ", "lead me to ", "walk me to ")
GO_THERE = ("go there", "take me there", "go", "guide me there", "lets go", "navigate there", "yes take me there")
AROUND_COMMANDS = ("what is around me", "what s around me", "whats around me", "what is near me", "what s near me",
                   "whats near me", "what is here", "what can you see around me")
MAX_OFFERED = 3           # candidates read out for "list them"
AROUND_MAX = 5            # objects listed for "what is around me"
LIST_WORDS = ("list them", "list all", "list", "read them all", "all of them", "which ones", "what are they")
# The user's own names for objects: "call this my chair", then "go to my chair" (kept for the session)
NAME_COMMANDS = ("call this ", "name this ", "this is ", "label this ", "call it ", "name it ", "call that ")
NAME_MATCH_RADIUS = 1.2   # m: a named object is the one of its class this close to where it was named
NAME_TARGET_RANGE = 2.5   # m: without a just-found object, "this" is the nearest one ahead within this range
# Physical buttons (pi_button_panel.py on the Pi): LOOK / MODE / HAND / TALK
DESCRIBE_PROMPT = "Describe what is in front of me."
VISION_ANSWER_S = 60.0  # s: a question unanswered this long is not waited for (its answer was lost)
# A HAND or LOOK tap waits this long for a second one (HAND double press: face mode on/off; LOOK: vision AI off)
HAND_DOUBLE_WAIT_S = 0.45
FRAME_MAX_AGE_S = 1.5     # a camera picture older than this is not used to recognise anyone
WHO_COMMANDS = ("who is this", "who is it", "who is here", "who is in front of me", "who is there",
                "who s this", "whos this", "who s there", "whos there")
NAME_LEADS = ("his name is ", "her name is ", "their name is ", "this is ", "it is ", "it s ", "name ")
# The Pi's camera stream arrives mirrored (as for object_perception and the vision AI): flipped back so that
# "on your left" is the wearer's left
FLIP_CAMERA = os.environ.get("WEARABLE_CAMERA_FLIP", "1") == "1"
HAND_REACH_M = 1.0        # m: an object this close is within reach, hand guidance starts at once
HAND_SEARCH_RANGE = 2.5   # m: without a found object, HAND picks the nearest object ahead within this range
SENSOR_SETTLE_S = 15.0    # s after "Camera and LiDAR on." in which their streams appearing is not announced
LOOK_REPEAT_S = 1.5       # a second LOOK tap this soon after the first is the same press: one description
# Words of the commands themselves (never "corrected" into the name of an object or a person)
COMMAND_WORDS = ("go take bring walk lead guide navigate find locate search look describe read grasp grab pick reach "
                 "save remember mark call label forget place name face status help vision off quiet warnings mute "
                 "unmute cross crossing light signal ahead around near here where who what list another stop cancel "
                 "exit shutdown describe colour color is there").split()
# Questions for the camera that are not commands, sent to the vision AI ("how many people are here")
QUESTION_STARTS = ("how ", "is ", "are ", "does ", "can ", "who ", "which ", "why ", "tell me ")
NUMBER_WORDS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
                "ten": 10, "eleven": 11, "twelve": 12, "won": 1}  # "won": how Whisper often hears "one"
STATUS_COMMANDS = ("status", "system status", "what is the status", "what s the status", "whats the status")
# The vision AI's off switch: said with a LOOK hold (or a TALK hold in indoor mode)
VISION_OFF_COMMANDS = ("vision off", "vision ai off", "turn off the vision ai", "turn off vision", "stop the vision ai",
                       "close the vision ai", "close vision", "turn the vision ai off", "turn vision off")
# When to start the current mode's parts (SLAM brain, camera AI, ...): "sensors" (default) once the Pi's camera and
# LiDAR are on (SENSORS button), "now" at start-up, "off" never (start them by hand). Parts already started in a
# terminal are used, never doubled.
AUTOSTART = {"1": "now", "true": "now", "0": "off", "false": "off"}.get(
    os.environ.get("WEARABLE_AUTOSTART", "sensors").lower(), os.environ.get("WEARABLE_AUTOSTART", "sensors").lower())
PI_NODE = "pi_button_panel"   # the Pi's button program: seen on the network = the Pi is connected
WATCH_S = 2.0                 # how often the Pi connection and sensors are checked
HELP_TEXT = ("Five buttons. Sensor button: press to turn the camera and LiDAR on; hold it to turn them off. Look: describe what is in front; "
             "hold it to ask the camera a question or say a command; press it twice to turn the vision AI off. Mode: indoor or outdoor; hold it to close the map "
             "and the camera and forget this place. Hand: guide your hand to an object; hold it to stop. Talk, in "
             "indoor mode: hold it and say where to go, for example chair; press it again to end the navigation. "
             "Press hand twice for face mode: then press it to hear who is in front, or hold it and say a name to "
             "remember that person. "
             "Tap talk twice for what is around you. "
             "You can say: find the table with the cup, go to the chair, what is around me, call this my chair, "
             "status, vision off.")
HELP_TEXT_OUTDOOR = ("Outdoor mode warns you about obstacles, holes, low branches and vehicles, with their distance. "
                     "Talk: tap twice for what is ahead. "
                     "Hold look and say: what is ahead, what colour is the light, can I cross, quiet warnings, "
                     "warnings on.")
OUTDOOR_QUIET = ("quiet", "quiet warnings", "mute", "mute warnings", "be quiet", "silence")
OUTDOOR_LOUD = ("warnings on", "unmute", "unmute warnings", "talk to me", "resume warnings")
HELP_COMMANDS = ("help", "what can i do", "what can i say", "how does it work")
STOP_COMMANDS = ("stop", "stop navigation", "cancel", "stop it")
# Outdoor mode (object_perception + outdoor_awareness.py): spoken hazard alerts on /outdoor_alert
ALERT_MAX_AGE_S = 2.0     # an alert whose turn to be spoken comes after this long is out of date (the wearer has moved on)
ALERT_MUTE_TAP_S = 6.0    # stopping everything quiets non-critical alerts this long (critical ones are always spoken)
ALERT_MUTE_SAID_S = 120.0 # "quiet warnings": non-critical alerts off this long (or until "warnings on")
SCENE_MAX_AGE_S = 3.0
OUTDOOR_AHEAD = ("what is ahead", "what s ahead", "whats ahead", "what is in front of me", "what s in front of me",
                 "whats in front of me", "what is in front", "is the path clear", "is the way clear")
# The Pi's sensors (pi_sensors.launch.py): said aloud when one is missing, lost or back
SENSORS = {"/camera/image_raw/compressed": "camera", "/scan": "LiDAR"}
# Colours are not spoken unless asked for (someone blind from birth may not know them); set 1 for low vision
SPEAK_COLORS = os.environ.get("WEARABLE_SPEAK_COLORS", "0") == "1"

# Push-to-talk (TALK, LOOK and HAND holds): the laptop microphone records while the button is held and Whisper
# (faster-whisper, offline, on the CPU: the GPU holds the camera AI and the vision AI) transcribes it. small.en
# understood 92 % of the system's own commands and is the most robust to noise and accents; base.en is nearly as
# accurate and faster (WEARABLE_WHISPER_MODEL=base.en).
WHISPER_MODEL = os.environ.get("WEARABLE_WHISPER_MODEL", "small.en")
WHISPER_THREADS = int(os.environ.get("WEARABLE_WHISPER_THREADS", "8"))
PTT_RATE = 16000
PTT_MAX_S = 12.0
PTT_SKIP_S = 0.15         # the "speak now" beep plays as the microphone opens: cut off, it is not speech
# What the wearer says, given to Whisper as a hint (it then prefers these words over sound-alikes: "chair" not
# "share", "vision off" not "visual"); the names of people, places and the objects on the map are added
PTT_VOCABULARY = ("Go to the chair, table, door, sofa, bed, toilet, kitchen, laptop, cup, bottle. Status, help, "
                  "vision off, what is around me, where am I, save this place as, call this my, forget face, find, "
                  "grasp, describe, read.")


class PushToTalk:
    """Record the microphone between start() and stop(), then transcribe."""

    def __init__(self):
        self._model = None
        self._pa = None
        self._frames, self._on, self._thread = [], False, None
        threading.Thread(target=self._load, daemon=True).start()  # ready before the first press

    def _load(self):
        # PyAudio takes ~2 s to start: created once here so a press starts recording at once
        try:
            import pyaudio
            with suppress_stderr():
                self._pa = pyaudio.PyAudio()
        except Exception as e:
            print(f"Microphone unavailable: {e}")
        try:
            kw = {"device": "cpu", "compute_type": "int8", "cpu_threads": WHISPER_THREADS}
            try:
                model = WhisperModel(WHISPER_MODEL, local_files_only=True, **kw)
            except Exception:
                print(f"Downloading the speech recognition model {WHISPER_MODEL} (once)...")
                model = WhisperModel(WHISPER_MODEL, **kw)
            # The first transcription is slow (it sets everything up): done now, not on the first press
            import numpy as np
            list(model.transcribe(np.zeros(PTT_RATE, np.float32), language="en")[0])
            self._model = model
            print(f"Speech recognition ready ({WHISPER_MODEL})")
        except Exception as e:
            print(f"Speech recognition unavailable ({WHISPER_MODEL}): {e}")

    def start(self):
        if self._on:
            return
        self._frames, self._on = [], True
        self._thread = threading.Thread(target=self._record, daemon=True)
        self._thread.start()

    def _record(self):
        import pyaudio
        if self._pa is None:
            return
        stream = self._pa.open(format=pyaudio.paInt16, channels=1, rate=PTT_RATE, input=True, frames_per_buffer=1024)
        t0 = time.time()
        try:
            while self._on and time.time() - t0 < PTT_MAX_S:
                self._frames.append(stream.read(1024, exception_on_overflow=False))
        finally:
            stream.stop_stream()
            stream.close()

    def stop(self, words=()) -> str:
        """Stop recording and return what was said ("" if nothing). `words`: names that may be said (people,
        places, objects on the map), added to the hint."""
        self._on = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._model is None or len(self._frames) < 5:
            return ""
        import numpy as np
        audio = np.frombuffer(b"".join(self._frames), np.int16).astype(np.float32) / 32768.0
        audio = audio[int(PTT_SKIP_S * PTT_RATE):]
        hint = PTT_VOCABULARY + (" " + ", ".join(dict.fromkeys(words)) + "." if words else "")
        segments, _ = self._model.transcribe(audio, beam_size=5, language="en", vad_filter=True,
                                             initial_prompt=hint, condition_on_previous_text=False)
        return "".join(seg.text for seg in segments).strip()


def _click_wav():
    """A 25 ms tick played on every button tap."""
    import wave
    import struct
    path = os.path.join(TMP_DIR, "visionnav_click.wav")
    if not os.path.exists(path):
        rate, n = 16000, 400
        with wave.open(path, "wb") as f:
            f.setnchannels(1)
            f.setsampwidth(2)
            f.setframerate(rate)
            f.writeframes(b"".join(struct.pack("<h", int(7000 * math.sin(2 * math.pi * 1500 * i / rate)
                                                        * (1 - i / n))) for i in range(n)))
    return path


def _beep_wav():
    """A short 880 Hz tone: "speak now" (a spoken prompt would be recorded along with the command)."""
    import wave
    import struct
    path = os.path.join(TMP_DIR, "visionnav_beep.wav")
    if not os.path.exists(path):
        rate, n = 16000, 1600
        with wave.open(path, "wb") as f:
            f.setnchannels(1)
            f.setsampwidth(2)
            f.setframerate(rate)
            f.writeframes(b"".join(struct.pack("<h", int(9000 * math.sin(2 * math.pi * 880 * i / rate)
                                                        * min(1.0, (n - i) / 400))) for i in range(n)))
    return path


@contextlib.contextmanager
def suppress_stderr():
    """Suppress C-level stderr (e.g. ALSA/JACK errors from PyAudio)."""
    fd = sys.stderr.fileno()
    old_fd = os.dup(fd)
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, fd)
    try:
        yield
    finally:
        os.dup2(old_fd, fd)
        os.close(old_fd)
        os.close(devnull)

def lang_vehicles():
    from visionnav.outdoor_awareness import VEHICLES
    return VEHICLES


class FindObjectNode(Node):
    def __init__(self):
        super().__init__('voice_navigation_assistant')
        
        self._marker_names = {}  # (ns, id) -> object name, so Marker.DELETE can forget the object
        # Speech: one sentence at a time, interruptible by the STOP button
        self._speech_lock = threading.Lock()
        self._speech_gen = 0
        self._speech_proc = None
        self._speaking_rank = 0  # urgency of the sentence being spoken (0: not a warning): what may cut it off
        self._voice = None       # Piper, kept loaded (see _load_voice)
        threading.Thread(target=self._load_voice, daemon=True).start()
        self._marker_sub = self.create_subscription(MarkerArray, '/semantic_markers', self._marker_callback, 10)
        
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self._path_pub = self.create_publisher(Path, '/object_path', 10)
        self._describe_cmd_pub = self.create_publisher(String, '/describe_command', 10)

        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.saved_objects = {}
        # Object map with classes, sizes and colours (object_perception /semantic_objects), for spoken requests
        self.objects = []
        self._pending = None  # [verb, [candidates nearest first], index offered] for "another one" / "the first one"
        # Buttons: perception mode (object_perception publishes it, latched) and push-to-talk
        self._mode = os.environ.get("WEARABLE_MODE", "indoor").lower()
        self._map_info = None
        self._mode_pub = self.create_publisher(String, '/perception_mode', 10)
        self._ptt = PushToTalk()
        self._sys = SystemManager(self, log=lambda m: print(f"⚙️  {m}"))
        self._switching = False     # a mode switch (starting/stopping parts) is under way
        self._off = False           # all modes turned off with a MODE hold while the sensors stay on: they stay
                                    # off until a MODE tap
        self._turning_off = False   # a MODE hold is under way: it says what is off, once
        self._look_last = -math.inf  # when LOOK was last tapped
        self._vision_queue = []     # questions waiting for the vision AI to start
        self._vision_asked = []     # [when sent, still wanted] of each question unanswered (answered in order)
        self._vision_gen = 0        # +1 at every "vision off": questions waiting for a start that was cancelled
        self._hand_busy = False     # a HAND press is starting the camera AI
        self._hand_timer = None     # a HAND tap waiting to see whether it is a double press
        self._look_timer = None     # a LOOK tap waiting to see whether it is a double press (vision AI off)
        self._face_mode = False     # HAND double press: taps recognise faces, a hold remembers one
        self._face_ptt = False      # a HAND hold in face mode is recording a name
        self._faces = None          # face_memory.FaceMemory, loaded at the first use
        self._cam_sub = None        # the camera stream, subscribed only while faces are wanted
        self._frame = (None, 0.0)   # the latest camera picture (compressed) and when it came
        self._talk_refused = False  # the TALK hold under way was refused (not in indoor mode): no recording
        self._nav_gen = 0           # +1 at every navigation start: an older guidance loop ends without a word
        self._grasp_on_arrival = False  # guide the hand after arriving (HAND button), not after a TALK navigation
        # The vision AI's answers are said here, in turn with everything else
        self.create_subscription(String, '/scene_description', self._description_callback, 10)
        self.create_subscription(String, '/button_event', self._button_callback, 10)
        self._active = False  # the current mode's parts are running
        if AUTOSTART == "now":
            threading.Thread(target=self._switch_mode, args=(self._mode, True), daemon=True).start()
        self._sensor_ok = {}
        self._pi_sensors = None  # the Pi's SENSORS button state (pi_button_panel, latched)
        threading.Thread(target=self._system_watch, daemon=True).start()
        self.create_subscription(String, '/pi_button_status', self._pi_button_status_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/pi_sensors_state', self._pi_sensors_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/perception_mode_state', self._mode_state_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._names = {}      # the user's names: {"my chair": {"class": "chair", "x": .., "y": ..}}
        self._names_file = None
        self.create_subscription(String, '/active_map', self._active_map_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/semantic_objects', self._objects_callback, 10)
        self.map_data = None
        self.last_found_object = None
        self.navigating = False  # True when actively guiding user
        self._using_nav2 = False
        
        # Nav2 semantic navigation (semantic_navigator.py): send the target, receive path + status
        self._semantic_goal_pub = self.create_publisher(String, '/semantic_goal', 10)
        self._nav2_status = None
        self._nav2_path = []  # [(x, y)] in map frame
        self.create_subscription(String, '/semantic_nav_status', self._nav2_status_callback, 10)
        self.create_subscription(Path, '/object_path', self._nav2_path_callback, 10)

        # Named places of this session (map_manager.py)
        self.named_places = {}  # {"kitchen": {"x": .., "y": ..}}
        self._map_cmd_pub = self.create_publisher(String, '/map_command', 10)
        self.create_subscription(String, '/named_places', self._places_callback, map_qos)
        self.create_subscription(String, '/map_command_result', lambda m: self.speak(m.data), 10)

        # Grasp mode (object_perception tracks the hand and the object, this node speaks the cues)
        self.grasping = False
        self._grasp_status = None
        self._grasp_cmd_pub = self.create_publisher(String, '/grasp_command', 10)
        self.create_subscription(String, '/grasp_offset', self._grasp_callback, 10)

        # Outdoor mode: one alert at a time, the most urgent; a critical one cuts off whatever is being said
        self._alert_lock = threading.Lock()
        self._alert_pending = None
        self._alert_event = threading.Event()
        self._alerts_muted_until = 0.0
        self._listening = False  # push-to-talk recording: nothing but critical alerts is said meanwhile
        self._outdoor_scene = None
        self.create_subscription(String, '/outdoor_alert', self._outdoor_alert_callback, 10)
        self.create_subscription(String, '/outdoor_scene', self._outdoor_scene_callback, 10)
        threading.Thread(target=self._alert_loop, daemon=True).start()
        
        self.get_logger().info("Find Object Node Started! Waiting for AI to map objects...")
        
        # Commands (typed, or spoken with push-to-talk) are handled one at a time by main_logic_loop
        self.command_queue = queue.Queue()
        
        self.thread = threading.Thread(target=self.main_logic_loop)
        self.thread.daemon = True
        self.thread.start()

    def _load_voice(self):
        """Keep the Piper voice loaded: starting the `piper` program for every sentence reloads the model each time
        (~1.2 s of silence before each sentence instead of ~0.1 s)."""
        try:
            from piper import PiperVoice
            self._voice = PiperVoice.load(TTS_MODEL)
        except Exception as e:
            print(f"Piper voice not kept loaded ({e}): each sentence starts the piper program instead")

    def _synthesize(self, text, wav_path):
        voice = self._voice
        if voice is not None:
            try:
                import wave
                with wave.open(wav_path, "wb") as f:
                    voice.synthesize_wav(text, f)
                return
            except Exception as e:
                print(f"Piper voice failed ({e}): using the piper program")
                self._voice = None
        # The text goes to Piper on stdin (no shell quoting to break on "12 o'clock")
        subprocess.run(["piper", "--model", TTS_MODEL, "--output_file", wav_path], input=text.encode(),
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def speak(self, text, expires=None, rank=0):
        """Speak text aloud (Piper TTS). One sentence at a time, in order; stop_speech() cuts it off.

        `expires` (monotonic time): a warning still waiting for its turn by then is dropped, not said late.
        `rank`: how urgent it is, for what may cut it off (_outdoor_alert_callback)."""
        print(f"🔊 Speaking: '{text}'")
        gen = self._speech_gen
        with self._speech_lock:
            if gen != self._speech_gen:  # STOP was pressed while this was waiting its turn
                return
            if expires is not None and time.monotonic() > expires:
                return
            self._speaking_rank = rank
            try:
                wav_path = os.path.join(TMP_DIR, "visionnav_voice.wav")
                self._synthesize(text, wav_path)
                if gen != self._speech_gen:
                    return
                self._speech_proc = subprocess.Popen(["aplay", "-q", wav_path], stderr=subprocess.DEVNULL)
                self._speech_proc.wait()
                self._speech_proc = None
            finally:
                self._speaking_rank = 0

    def stop_speech(self):
        """Cut off what is being said and drop what is waiting to be said."""
        self._speech_gen += 1
        proc = self._speech_proc
        if proc is not None and proc.poll() is None:
            proc.terminate()

    # ── OUTDOOR MODE: spoken hazard alerts (object_perception decides what and when; this node says it) ──
    def _outdoor_alert_callback(self, msg: String):
        if self._mode != "outdoor" or not self._active:
            return
        try:
            a = json.loads(msg.data)
            a["level"] = int(a.get("level", 1))
        except (ValueError, TypeError):
            return
        now = time.monotonic()
        if a["level"] < 3 and (now < self._alerts_muted_until or self._listening):
            return
        a["t"] = now
        a["rank"] = int(a.get("rank", a["level"]))
        # A danger cuts off what is being said, unless that is just as urgent (only a vehicle coming outranks another
        # danger), so every warning is heard to its end
        if a["level"] >= 3 and a["rank"] > self._speaking_rank:
            self.stop_speech()
        with self._alert_lock:
            if self._alert_pending is None or a["rank"] >= self._alert_pending["rank"]:
                self._alert_pending = a
        self._alert_event.set()

    def _alert_loop(self):
        while rclpy.ok():
            self._alert_event.wait(0.5)
            self._alert_event.clear()
            with self._alert_lock:
                a, self._alert_pending = self._alert_pending, None
            if a is None or time.monotonic() - a["t"] > ALERT_MAX_AGE_S or self._mode != "outdoor":
                continue
            # Its age is checked again when its turn comes: behind a long sentence it would be said too late
            self.speak(a["text"], expires=a["t"] + ALERT_MAX_AGE_S, rank=a["rank"])

    def _outdoor_scene_callback(self, msg: String):
        try:
            self._outdoor_scene = json.loads(msg.data)
            self._outdoor_scene["_t"] = time.monotonic()
        except ValueError:
            pass

    def _scene(self):
        sc = self._outdoor_scene
        return sc if sc is not None and time.monotonic() - sc["_t"] <= SCENE_MAX_AGE_S else None

    def _outdoor_command(self, target) -> bool:
        """Outdoor questions, answered from what the camera AI sees now. True when handled."""
        if target in HELP_COMMANDS:
            self.speak(HELP_TEXT_OUTDOOR)
        elif target in OUTDOOR_QUIET:
            self._alerts_muted_until = time.monotonic() + ALERT_MUTE_SAID_S
            self.speak("Warnings quiet for two minutes. I will still warn you of immediate danger.")
        elif target in OUTDOOR_LOUD:
            self._alerts_muted_until = 0.0
            self.speak("Warnings on.")
        elif target in AROUND_COMMANDS or target in OUTDOOR_AHEAD:
            sc = self._scene()
            self.speak(sc["summary"] if sc else "The camera AI is not seeing anything yet.")
        elif " cross" in f" {target}" or "zebra" in target:
            self.speak(self._crossing_answer())
        elif ("light" in target or "signal" in target) and not target.startswith("describe"):
            sc = self._scene()
            sig = (sc or {}).get("signal")
            self.speak(sig["text"] if sig else "I do not see a traffic light or pedestrian signal ahead.")
        else:
            return False
        return True

    def _crossing_answer(self):
        """Where the crossing is, what its signal shows and which vehicles are coming. Never says it is safe:
        the camera cannot see everything (a car hidden by a bus, a bike behind)."""
        sc = self._scene()
        if sc is None:
            return "The camera AI is not seeing anything yet."
        parts = []
        cross = min((o for o in sc["objects"] if o["class"] == "zebra crossing"), key=lambda o: o["dist"], default=None)
        if cross is not None:
            parts.append(f"Zebra crossing {int(round(cross['dist'] * 3.28084))} feet ahead.")
        if sc.get("signal"):
            parts.append(sc["signal"]["text"])
        coming = [o for o in sc["objects"] if o["class"] in lang_vehicles() and (o.get("closing") or 0) > 1.5]
        if coming:
            o = min(coming, key=lambda o: o["dist"])
            side = "on your left" if o["y"] > 1 else "on your right" if o["y"] < -1 else "ahead"
            parts.append(f"A {o['class']} is coming {side}, {int(round(o['dist'] * 3.28084))} feet away."
                         + (f" {len(coming) - 1} more vehicles are moving." if len(coming) > 1 else ""))
        else:
            parts.append("I see no vehicle coming.")
        parts.append("Listen for traffic before you cross.")
        return " ".join(parts)

    def _marker_callback(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.action == 3:
                self.saved_objects.clear()
                self._marker_names.clear()
                continue
            key = (marker.ns, marker.id)
            if marker.action == 2:  # DELETE: the perception node removed this object from the map
                name = self._marker_names.pop(key, None)
                if name is not None:
                    self.saved_objects.pop(name, None)
                continue
            if marker.text:
                # Strip out the details like "(dist:1.2m|H:0.9m)" and "[MEM]" tags so we just get "chair_1"
                obj_name = marker.text.split('(')[0].replace("[MEM]", "").strip().lower()
                self.saved_objects[obj_name] = marker.pose.position
                self._marker_names[key] = obj_name

    def _objects_callback(self, msg: String):
        try:
            self.objects = json.loads(msg.data).get("objects", [])
        except (ValueError, AttributeError):
            pass

    # ── PHYSICAL BUTTONS ──
    def _mode_state_callback(self, msg: String):
        self._mode = msg.data.strip().lower() or self._mode

    def _button_callback(self, msg: String):
        try:
            ev = json.loads(msg.data)
            button, event = ev["button"], ev["event"]
        except (ValueError, KeyError, TypeError):
            return
        print(f"🔘 {button.upper()} {event}")
        if event == "tap" or (event == "hold_start" and button in ("mode", "sensors")) or (
                event == "hold_start" and button == "hand" and not self._face_mode):
            # A click at once, so the wearer knows the press was heard
            subprocess.Popen(["aplay", "-q", _click_wav()], stderr=subprocess.DEVNULL)
        if button == "look" and event in ("tap", "double"):
            # A tap waits a moment, as HAND's: two quick presses turn the vision AI off instead of describing
            if self._look_timer is not None:
                self._look_timer.cancel()
                self._look_timer = None
            if event == "double":
                threading.Thread(target=self._vision_off, daemon=True).start()
            else:
                self._look_timer = threading.Timer(HAND_DOUBLE_WAIT_S, self._look_tap)
                self._look_timer.start()
            return
        if button == "hand" and event in ("tap", "double"):
            # A tap waits a moment: two quick presses switch face mode on or off instead
            if self._hand_timer is not None:
                self._hand_timer.cancel()
                self._hand_timer = None
            if event == "double":
                threading.Thread(target=self._toggle_face_mode, daemon=True).start()
            else:
                self._hand_timer = threading.Timer(HAND_DOUBLE_WAIT_S, self._hand_tap)
                self._hand_timer.start()
            return
        if button == "hand" and event == "hold_start" and self._face_mode:
            self._face_ptt = True  # recording the name of the person in front
        if button == "hand" and event == "hold_end" and self._face_ptt:
            self._face_ptt = False
            threading.Thread(target=self._ptt_finish, args=("hand",), daemon=True).start()
            return
        if button == "talk" and event == "hold_start" and not (self._active and self._mode == "indoor"):
            # TALK is the navigation button: only in indoor mode (the map is indoor mode's)
            self._talk_refused = True
            threading.Thread(target=self._no_navigation, daemon=True).start()
            return
        if button == "talk" and event == "hold_end" and self._talk_refused:
            self._talk_refused = False
            return
        if (button in ("talk", "look") or self._face_ptt) and event == "hold_start":
            # Push-to-talk: silence the speaker (it would be recorded), beep, record while held. TALK: where to go
            # (or a command); LOOK: a question for the camera ("what colour is the door") or a command
            self.stop_speech()
            if button == "look":
                self._drop_description()
            self._listening = True
            subprocess.Popen(["aplay", "-q", _beep_wav()], stderr=subprocess.DEVNULL)
            self._ptt.start()
        elif button in ("talk", "look") and event == "hold_end":
            threading.Thread(target=self._ptt_finish, args=(button,), daemon=True).start()
        else:
            threading.Thread(target=self._on_button, args=(button, event), daemon=True).start()

    @staticmethod
    def _command_phrases():
        """Whole short commands, for speech_fix.fix_command."""
        return set(STOP_COMMANDS + STATUS_COMMANDS + VISION_OFF_COMMANDS + HELP_COMMANDS + AROUND_COMMANDS
                   + WHO_COMMANDS + OUTDOOR_QUIET + OUTDOOR_LOUD + OUTDOOR_AHEAD + ("where am i", "describe"))

    def _ptt_words(self):
        """Names the wearer may say now: remembered people and places, their names for objects, what is on the map."""
        words = list(self._faces.names()) if self._faces is not None else []
        words += [p.title() for p in self.named_places] + list(self._names)
        words += sorted({o["class"] for o in self.objects})
        return words[:60]

    def _ptt_finish(self, button):
        text = self._ptt.stop(self._ptt_words())
        self._listening = False
        if not text:
            self.speak("I did not catch that.")
            return
        print(f"🎤 Heard: '{text}'")
        target = " ".join(re.sub(r"[^a-z0-9_ ]", " ", text.lower()).split())
        # Sound-alikes of what this system knows: "top" -> "stop", "share one" -> "chair one" (speech_fix.py)
        phrases = self._command_phrases()
        fixed = speech_fix.fix_command(target, phrases)
        if button != "hand" and fixed not in phrases:  # a new name is never "corrected" into one already known
            fixed = speech_fix.fix_words(fixed, self._ptt_words() + list(lang.FURNITURE),
                                         protect=list(phrases) + [PTT_VOCABULARY.lower()] + list(COMMAND_WORDS))
        if fixed != target:
            print(f"🎤 Understood: '{fixed}'")
            target = text = fixed
        if button == "hand":
            self._remember_face(target)
        elif button == "look" and not self._outdoor_words(target) and self._camera_question(target):
            self._ask_vision(text)  # LOOK is the camera's button: "is there a chair" asks the camera, not the map
        elif self._is_command(target):
            self.command_queue.put(text)  # "status", "vision off", "find the cup", "go to the table", ...
        elif button == "look":
            self._ask_vision(text)  # a question for the camera (Qwen3-VL), started first if it is off
        else:
            # TALK: just the destination ("chair", "the table with the cup", "my chair"): go there
            self.command_queue.put(f"go to {target}")

    # ── FACES (HAND button: double press = face mode; there a tap says who it is, a hold remembers a name) ──
    def _hand_tap(self):
        self._hand_timer = None
        if self._face_mode:
            self._recognize()
        else:
            self._on_button("hand", "tap")  # hand guidance

    def _toggle_face_mode(self):
        if self._face_mode:
            self._face_mode_off()
            self.speak("Face mode off.")
            return
        if self._load_faces() is None:
            return
        self._face_mode = True
        self._watch_camera(True)
        self.speak("Face mode. Press to recognise a person. Hold and say a name to remember them.")

    def _face_mode_off(self):
        self._face_mode = False
        self._watch_camera(False)

    def _load_faces(self):
        if self._faces is None:
            try:
                from visionnav.face_memory import FaceMemory
                self._faces = FaceMemory()
            except Exception as e:  # models missing (models/face_*.onnx), OpenCV without the face module
                print(f"Face recognition unavailable: {e}")
                self.speak("Face recognition is not available on this laptop.")
        return self._faces

    def _watch_camera(self, on):
        """The camera stream, only while faces are wanted (another subscriber doubles the Pi's Wi-Fi traffic)."""
        if on and self._cam_sub is None:
            self._cam_sub = self.create_subscription(
                CompressedImage, "/camera/image_raw/compressed",
                lambda m: setattr(self, "_frame", (m, time.monotonic())),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        elif not on and self._cam_sub is not None:
            self.destroy_subscription(self._cam_sub)
            self._cam_sub, self._frame = None, (None, 0.0)

    def _picture(self):
        """The latest camera picture (BGR, as the wearer sees it) or None. Outside face mode ("who is this"
        said with LOOK) the stream is watched just for this."""
        import cv2
        import numpy as np
        temporary = self._cam_sub is None
        self._watch_camera(True)
        t0 = time.monotonic()
        while time.monotonic() - self._frame[1] > FRAME_MAX_AGE_S and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        msg, t = self._frame
        if temporary:
            self._watch_camera(False)
        if msg is None or time.monotonic() - t > FRAME_MAX_AGE_S + 2.0:
            return None
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        return cv2.flip(img, 1) if img is not None and FLIP_CAMERA else img

    def _recognize(self):
        """Who is in front, left to right: "This is Kamal." / "I see 2 people: Kamal on your left, and someone I
        don't know on your right." """
        if self._load_faces() is None:
            return
        img = self._picture()
        if img is None:
            self.speak("The camera is not sending pictures. Press the sensor button.")
            return
        people = self._faces.identify(img)
        if not people:
            self.speak("I don't see a face.")
            return
        if len(people) == 1:
            name = people[0][0]
            self.speak(f"This is {name}." if name else
                       "I don't know this person. Hold the hand button and say their name to remember them.")
            return

        def where(cx):
            return "on your left" if cx < 0.35 else "on your right" if cx > 0.65 else "ahead"
        said = [f"{name or 'someone I do not know'} {where(cx)}" for name, _, cx in people]
        self.speak(f"I see {len(people)} people: {', '.join(said[:-1])}, and {said[-1]}.")

    def _remember_face(self, target):
        """HAND hold in face mode: the name just said goes to the face in front."""
        for lead in NAME_LEADS:
            if target.startswith(lead):
                target = target[len(lead):]
        name = " ".join(w.capitalize() for w in target.split())
        if not name:
            self.speak("I did not catch the name.")
            return
        if self._load_faces() is None:
            return
        img = self._picture()
        if img is None:
            self.speak("The camera is not sending pictures. Press the sensor button.")
            return
        result = self._faces.remember(name, img)
        self.speak({"ok": f"I will remember {name}.",
                    "no_face": "I don't see a face. Point the camera at the person, then try again.",
                    "several": f"I see more than one face. Only {name} should be in front of the camera."}[result])

    def _outdoor_words(self, t) -> bool:
        """An outdoor question or setting (answered from the hazard view, _outdoor_command)."""
        return self._mode == "outdoor" and (t in OUTDOOR_QUIET or t in OUTDOOR_LOUD or t in OUTDOOR_AHEAD
                                            or " cross" in f" {t}" or "zebra" in t
                                            or (("light" in t or "signal" in t) and not t.startswith("describe")))

    @staticmethod
    def _camera_question(t) -> bool:
        """A question about what the camera sees ("what colour is the door", "is there a chair"), not one the map
        or the system answers ("what is around me", "what is on the table", "status")."""
        return (t.startswith(QUESTION_STARTS + ("what ", "read ")) or t == "describe" or t.startswith("describe ")) \
            and t not in AROUND_COMMANDS and t not in STATUS_COMMANDS and t not in HELP_COMMANDS \
            and t not in WHO_COMMANDS \
            and not t.startswith(("what is on ", "what s on ", "whats on "))

    def _is_command(self, t) -> bool:
        """A sentence main_logic_loop knows as a command (as it tests them), rather than a destination or a
        question for the camera."""
        return bool(
            self._outdoor_words(t) or t in ("exit", "shut down", "shutdown") or t in STOP_COMMANDS or t in STATUS_COMMANDS
            or t in VISION_OFF_COMMANDS or t in HELP_COMMANDS or t in AROUND_COMMANDS or t in GO_THERE
            or t in ("save map", "save the map", "where am i") or t == "describe" or t in WHO_COMMANDS
            or t.startswith(("describe ", "what ", "read ", "forget ")) or t.startswith(GRASP_COMMANDS)
            or t.startswith(PLACE_COMMANDS) or t.startswith(NAME_COMMANDS) or t.startswith(FIND_COMMANDS)
            or t.startswith(GO_COMMANDS) or t.startswith(QUESTION_STARTS)
            or (self._pending and (lang.is_next(t) or t in LIST_WORDS or lang.is_selection(t))))

    def _no_navigation(self):
        """TALK hold outside indoor mode."""
        if self._switching and self._mode == "indoor":
            self.speak("Indoor mode is still starting. Please wait.")
        else:
            self.speak("You can't use navigation now. It works only in indoor mode.")

    def _end_navigation(self):
        """TALK press while being guided: the navigation ends."""
        self._stop_nav()
        cancel_path = Path()
        cancel_path.header.frame_id = "map"
        self._path_pub.publish(cancel_path)
        self.stop_speech()
        self.speak("Navigation stopped.")

    def _description_callback(self, msg: String):
        """The vision AI's answer (scene_describer), said here in turn with everything else so STOP can cut it off."""
        # Only the answer to the newest question is said
        if self._vision_asked:
            _, wanted = self._vision_asked.pop(0)
            if self._vision_asked or not wanted:
                return
        threading.Thread(target=self.speak, args=(msg.data,), daemon=True).start()

    def _on_button(self, button, event):
        if button == "mode" and event == "tap":
            new = "outdoor" if self._mode == "indoor" else "indoor"
            if self._switching:
                self.speak("Still switching modes. Please wait.")
            elif self._off:
                # Turned off with a MODE hold: a tap starts the same mode again (at once, or with the sensors)
                self._off = False
                if AUTOSTART == "sensors" and self._missing_sensors():
                    self._say_selected(self._mode)
                else:
                    self._switch_mode(self._mode, True)
            elif not self._active and AUTOSTART == "sensors":
                # Paused (sensors off): just choose the mode; it starts with the camera and LiDAR
                self._mode = new
                self._say_selected(new)
            else:
                self._switch_mode(new)
        elif button == "mode" and event == "hold_start":
            self._all_off()
        elif button == "hand" and event == "tap":
            self._hand_button()
        elif button == "hand" and event == "hold_start":
            self._hand_off()
        elif button == "talk" and event == "tap":
            if self.navigating:
                self._end_navigation()  # one press while guided: the navigation ends (otherwise a tap does nothing)
        elif button == "talk" and event == "double":
            if self.navigating:
                self._end_navigation()
            else:
                self._around()

    def _say_selected(self, mode):
        """The mode is chosen but waits for the sensors. Names the one that is missing: the Pi says "on" when the
        LiDAR started even if the camera publisher exited."""
        missing = self._missing_sensors() or [name for name in SENSORS.values()]
        self.speak(f"{mode.capitalize()} mode selected. It starts when the "
                   f"{' and the '.join(missing)} {'is' if len(missing) == 1 else 'are'} on. "
                   + self._sensor_sentence(self._missing_sensors()))

    def _stop_all(self, say="Stopped."):
        """Stop talking, navigating and hand guidance at once (the "stop" command, HAND and MODE holds). Outdoors,
        non-critical warnings are also quiet for a few seconds."""
        self.stop_speech()
        if self._mode == "outdoor":
            self._alerts_muted_until = max(self._alerts_muted_until, time.monotonic() + ALERT_MUTE_TAP_S)
        self._pending = None
        if self.grasping:
            self.grasping = False
        if self.navigating:
            self._stop_nav()
            cancel_path = Path()
            cancel_path.header.frame_id = "map"
            self._path_pub.publish(cancel_path)
        if say:
            self.speak(say)

    def _all_off(self):
        """MODE hold: close the map and the camera feed — the programs of both modes (the map with its window, the
        camera AI with the camera window, the outdoor view) — and stop any guidance. The session ends: its map, its
        objects and the places and names given in it are forgotten (nothing is kept for another day). The camera
        and LiDAR stay on (the SENSORS button's job); the modes stay off until MODE is tapped. The vision AI has its
        own off switch (say "vision off")."""
        if self._switching:
            self.speak("Still switching modes. Please wait.")
            return
        parts = list(dict.fromkeys(p for mode_parts in MODE_PARTS.values() for p in mode_parts))
        self._switching = self._turning_off = True
        try:
            self._stop_all(say=None)
            if not self._active and not any(self._sys.owned(p) for p in parts):
                by_hand = any(self._sys.running(p) for p in parts)
                self.speak("The modes were started in a terminal. Close them there." if by_hand and not self._off
                           else "All modes are already off.")
                return
            for part in parts:
                self._sys.stop(part)
            self._active = False
            self._off = self._pi_sensors not in ("stopping", "off")  # sensors off: they start it all again
            self._forget_session()
            self._face_mode_off()
            msg = "Map and camera closed. Press mode to start again."
            by_hand = [self._sys.name(p) for p in parts if self._sys.running(p)]
            if by_hand:  # started in a terminal, not by the assistant: left alone
                msg += (f" {' and '.join(by_hand).capitalize()} {'was' if len(by_hand) == 1 else 'were'} started in "
                        f"a terminal. Close {'it' if len(by_hand) == 1 else 'them'} there.")
            self.speak(msg)
        finally:
            self._switching = self._turning_off = False

    def _forget_session(self):
        """The session's map is gone (MODE hold, the sensors off, indoor mode left): forget what was learned in it
        here too — the objects, the found one, the wearer's names and places. They were positions in that map."""
        self.saved_objects.clear()
        self._marker_names.clear()
        self.objects = []
        self._pending = None
        self.last_found_object = None
        self._names, self._names_file = {}, None
        self.named_places = {}
        self._map_info = None

    # ── PARTS OF THE SYSTEM (system_manager.py): started and stopped by the buttons ──
    def _switch_mode(self, mode, startup=False):
        """Start the parts the mode needs, stop the other mode's, and say when it is active.
        Indoor: SLAM brain (map, Nav2, walls) + camera AI. Outdoor: sensor mounts and the live RViz view + camera AI
        (hazard warnings). The camera AI switches mode without a restart."""
        self._switching = True
        self._off = False
        try:
            # Said once, when it is ready (the button's click already said the press was heard)
            print(f"⚙️  {'starting' if startup else 'switching to'} {mode} mode")
            leaving = [p for m, parts in MODE_PARTS.items() if m != mode for p in parts if p not in MODE_PARTS[mode]]
            for part in leaving:
                self._sys.stop(part)
            if "brain" in leaving:
                self._forget_session()  # the indoor map is gone: the next indoor mode maps the place it is in
            wanted = list(MODE_PARTS[mode])
            for part in wanted:
                self._sys.start(part, wait=False)  # all at once; then wait for each
            failed = [p for p in wanted if not self._sys.wait_ready(p)]
            self._mode = mode
            self._active = True
            self._mode_pub.publish(String(data=mode))
            msg = f"{mode.capitalize()} mode activated."
            if failed:
                msg += " " + " ".join(f"{self._sys.name(p).capitalize()} did not start." for p in failed)
            if getattr(self._sys, "imu_note", "") and "brain" in wanted:
                msg += " " + self._sys.imu_note
            missing = self._missing_sensors()
            if missing:
                msg += " " + self._sensor_sentence(missing)
            # The switch is done: the next MODE press works while this is still being said
            self._switching = False
            self.speak(msg)
        finally:
            self._switching = False

    def _missing_sensors(self):
        """Names of the Pi sensors nobody is publishing (the stream's publisher is gone, no Wi-Fi traffic used)."""
        return [name for topic, name in SENSORS.items() if self.count_publishers(topic) == 0]

    def _sensor_sentence(self, missing):
        if not missing:
            return ""
        what = " and the ".join(missing)
        if self._pi_sensors == "on":  # the Pi started them but one exited (camera not plugged in): a tap restarts
            return (f"The {what} {'is' if len(missing) == 1 else 'are'} not running. "
                    f"Check {'its' if len(missing) == 1 else 'their'} cable, then press the sensor button.")
        return (f"The {what} {'is' if len(missing) == 1 else 'are'} not running. "
                f"Press the sensor button on the Pi to turn them on.")

    def _pi_button_status_callback(self, msg: String):
        """The Pi could not open some buttons (wiring, permissions): say which, instead of staying silent."""
        if msg.data.startswith("failed"):
            names = msg.data.split(":", 1)[1].split()
            what = "all the buttons" if len(names) >= 5 else "the " + " and ".join(names) + " button" + (
                "s" if len(names) > 1 else "")
            threading.Thread(target=self.speak, args=(f"The Pi cannot read {what}. Run the setup script on the Pi "
                                                      f"again, or check the wiring.",), daemon=True).start()

    def _pi_sensors_callback(self, msg: String):
        """SENSORS button on the Pi: say what the LiDAR and camera are doing."""
        state, first = msg.data.strip(), self._pi_sensors is None
        self._pi_sensors = state
        if first and state in ("off", "on"):
            return  # the latched state at start-up, not a change
        if state in ("stopping", "off"):
            self._off = False  # a fresh start: the next SENSORS press starts the mode as usual
        # Said once, when it is done ("starting" and "stopping" are not announced: the button clicked)
        said = {"on": "Camera and LiDAR on.", "off": "Camera and LiDAR off.",
                "failed": "The camera and LiDAR could not start. Check their cables."}.get(state)
        if state == "off":
            if self._turning_off:
                return  # a MODE hold: _all_off says it
            if AUTOSTART == "sensors":
                threading.Thread(target=self._pause, daemon=True).start()  # says it, with the mode
                return
        if state == "on":
            self._sensors_on_t = time.monotonic()  # their streams appear over the next seconds: not said again
        if said:
            threading.Thread(target=self.speak, args=(said,), daemon=True).start()

    def _pause(self):
        """Sensors switched off with the SENSORS button: say so, once, and stop the mode's programs (the session's
        map and objects are forgotten); they start again, with a new map, with the sensors."""
        if not self._active or self._switching:
            if not self._turning_off:
                self.speak("Camera and LiDAR off.")
            return
        self._switching = True
        try:
            for part in MODE_PARTS[self._mode]:
                self._sys.stop(part)
            self._active = False
            self._forget_session()
            self.speak(f"Camera and LiDAR off. {self._mode.capitalize()} mode paused. "
                       f"Press the sensor button to start again.")
        finally:
            self._switching = False

    def _system_watch(self):
        """The Pi connection and its sensors, checked every WATCH_S: greets the wearer, says when the Pi or a
        sensor stream is lost or back, and (AUTOSTART "sensors") starts the mode when the camera and LiDAR come on.
        No SSH or command on the laptop is involved: the Pi's programs appear on the ROS network by themselves."""
        time.sleep(3.0)  # ROS discovery
        pi_was, first = None, True
        while rclpy.ok():
            names = {n for n, _ in self.get_node_names_and_namespaces()}
            sensors = {name: self.count_publishers(topic) > 0 for topic, name in SENSORS.items()}
            all_on = all(sensors.values())
            pi = PI_NODE in names or any(sensors.values())
            if first:
                if all_on:
                    self.speak("VisionNav is on.")
                elif pi:
                    self.speak("VisionNav is on. Press the sensor button to start. Hold look and say help for help.")
                else:
                    self.speak("VisionNav is on. Waiting for the Pi. Check that it is switched on "
                               "and on the same Wi-Fi.")
            elif pi != pi_was:
                self.speak("Connected to the Pi." if pi else
                           "The Pi is not answering. Check that it is switched on and on the same Wi-Fi.")
            # Streams lost or back: not while the SENSORS button is switching them, nor while they appear after
            # "Camera and LiDAR on."
            settling = time.monotonic() - getattr(self, "_sensors_on_t", -math.inf) < SENSOR_SETTLE_S
            if self._pi_sensors in ("starting", "stopping", "off") or settling:
                self._sensor_ok = {}
            else:
                for name, ok in sensors.items():
                    was = self._sensor_ok.get(name)
                    if was is True and not ok and pi:
                        self.speak(f"The {name} signal is lost.")
                    elif was is False and ok and not first:
                        self.speak(f"The {name} is on.")
                    self._sensor_ok[name] = ok
            if AUTOSTART == "sensors" and all_on and not self._active and not self._switching and not self._off:
                threading.Thread(target=self._switch_mode, args=(self._mode, True), daemon=True).start()
                self._switching = True  # until the thread takes over
            pi_was, first = pi, False
            time.sleep(WATCH_S)

    def _ask_vision(self, question, announce=None):
        """Ask the camera (Qwen3-VL). Starts the vision AI first if it is off, and says so."""
        if self._sys.running("vision_ai"):
            if announce:
                self.stop_speech()
                self.speak(announce)
            self._send_vision(question)
            return
        self._vision_queue.append(question)
        if self._sys.starting("vision_ai") or len(self._vision_queue) > 1:
            self.speak("The vision AI is still starting. I will answer when it is ready.")
            return
        gen = self._vision_gen
        self.speak("Turning on the vision AI.")
        if gen != self._vision_gen:
            return  # "vision off" while this was being said: not started at all
        ok = self._sys.start("vision_ai")
        if gen != self._vision_gen:
            return  # turned off ("vision off") while it was starting: nothing to answer, nothing failed
        questions, self._vision_queue = self._vision_queue, []
        if not ok:
            self.speak("The vision AI could not start. Check that Ollama is running.")
            return
        time.sleep(1.5)  # its first camera frame (the answer itself says that it is on)
        # Only the last question: an earlier one (a LOOK tap's description) is no longer wanted
        if questions:  # none: a LOOK hold dropped the description and asked nothing for the camera
            self._send_vision(questions[-1])

    def _send_vision(self, question):
        now = time.monotonic()
        self._vision_asked = [a for a in self._vision_asked if now - a[0] < VISION_ANSWER_S] + [[now, True]]
        self._describe_cmd_pub.publish(String(data=question))

    def _drop_description(self):
        """LOOK held: the wearer is asking something, so a description not yet said (a tap just before the hold) is
        no longer wanted."""
        if self._look_timer is not None:
            self._look_timer.cancel()
            self._look_timer = None
        self._vision_queue = [q for q in self._vision_queue if q != DESCRIBE_PROMPT]
        for a in self._vision_asked:
            a[1] = False

    def _look_tap(self):
        """LOOK tap (not a double press): describe what is in front."""
        self._look_timer = None
        if time.monotonic() - self._look_last >= LOOK_REPEAT_S:
            self._look_last = time.monotonic()
            self._ask_vision(DESCRIBE_PROMPT, "Looking.")

    def _vision_off(self):
        """LOOK double press, or "vision off": stop the vision AI and free the GPU memory Qwen3-VL uses. Questions
        waiting for it to start are dropped."""
        self._vision_gen += 1
        pending, self._vision_queue = bool(self._vision_queue), []
        self._vision_asked = []
        if self._sys.stop("vision_ai") or pending:
            try:
                from visionnav.scene_describer import VLM_MODEL
                subprocess.run(["ollama", "stop", VLM_MODEL], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=10)
            except Exception:
                pass
            self.speak("Vision AI off.")
        elif self._sys.running("vision_ai"):
            self.speak("The vision AI was started in a terminal. Close it there.")
        else:
            self.speak("The vision AI is already off.")

    def _status(self):
        """ "status": where the system stands, then what is around."""
        parts = ["All modes are off." if self._off else f"{self._mode.capitalize()} mode."]
        parts.append("Vision AI on." if self._sys.running("vision_ai") else "Vision AI off.")
        missing = self._missing_sensors()
        if missing:
            parts.append(self._sensor_sentence(missing))
        if self._off:
            self.speak(" ".join(parts) + f" Press mode to start {self._mode} mode.")
            return
        if not self._sys.running("perception"):
            parts.append("The camera AI is not running.")
        if self._mode == "outdoor":
            sc = self._scene()
            if sc is not None:
                if not sc.get("lidar"):
                    parts.append("The LiDAR is not in use.")
                parts.append(sc["summary"])
            if time.monotonic() < self._alerts_muted_until:
                parts.append("Warnings are quiet.")
            self.speak(" ".join(parts))
            return
        if self._mode == "indoor" and self._map_info:
            parts.append("Mapping a new area.")
        n = len([o for o in self.objects if not o.get("dynamic")])
        parts.append(f"I know {n} object{'s' if n != 1 else ''}." if n else "I have not mapped any objects yet.")
        if self.navigating:
            parts.append("I am guiding you.")
        self.speak(" ".join(parts))
        if n:
            self._around()

    def _hand_off(self):
        """HAND hold: hand guidance off, and the camera AI with its camera window closed when no mode is using it
        (the HAND button opened it). In indoor or outdoor mode the camera stays: the mode needs it (MODE hold
        closes it)."""
        guiding = self.grasping or self.navigating
        if guiding:
            self._stop_all(say=None)
        camera = not self._active and not self._switching and self._sys.owned("perception")
        if camera:
            self._sys.stop("perception")
        if guiding or camera:
            self.speak("Hand guidance off." + (" Camera closed." if camera else ""))
        else:
            self.speak("Hand guidance is already off.")

    def _hand_button(self):
        """HAND tap: guide the hand to the object found last, or the nearest one ahead; walk there first if it
        is out of reach (arrival starts the hand guidance). Tap again to stop. Said once: what it will do."""
        if self._hand_busy:
            return  # a second tap while the camera AI is starting
        if self.grasping or self.navigating:
            self._stop_all("Hand guidance disabled.")
            return
        self._hand_busy = True
        try:
            if not self._sys.running("perception"):
                self.speak("Turning on the camera AI.")
                if not self._sys.start("perception"):
                    self.speak("The camera AI could not start.")
                    return
                time.sleep(2.0)  # first detections
            self._hand_target()
        finally:
            self._hand_busy = False

    def _hand_target(self):
        objects, pose = self._static_objects(), self.get_robot_pose()
        obj = next((o for o in objects if o["name"] == self.last_found_object and not o.get("dynamic")), None)
        if obj is None and pose is not None:
            ahead = [o for o in objects if not o.get("dynamic") and lang._bearing(o, pose)[0] <= HAND_SEARCH_RANGE
                     and abs(lang._bearing(o, pose)[1]) < math.radians(60)]
            small = [o for o in ahead if o["class"] not in lang.FURNITURE]  # a cup or a switch, not the table
            obj = min(small or ahead, key=lambda o: lang._bearing(o, pose)[0], default=None)
        if obj is None:
            self.speak("Nothing to reach for yet. Find an object first, then press the hand button.")
            return
        self.last_found_object = obj["name"]
        dist = lang._bearing(obj, pose)[0] if pose is not None else 0.0
        if dist <= HAND_REACH_M:
            self.start_grasp(obj["class"])  # speaks "Reach out your hand toward the cup."
        else:
            name = lang.spoken_name(obj, objects, **self._say_opts())
            self.speak(f"{name[0].upper()}{name[1:]} is {lang.where(obj, pose)}. "
                       f"Taking you there, then I will guide your hand.")
            self._start_nav(obj, objects, grasp=True)

    # ── SPOKEN OBJECT REQUESTS ──
    def _static_objects(self):
        return [o for o in self.objects if not o.get("dynamic")] + [o for o in self.objects if o.get("dynamic")]

    def _say_opts(self):
        """Keyword arguments describing objects the way this user can use them."""
        return {"places": self.named_places, "labels": self._object_names(), "colors": SPEAK_COLORS}

    # The user's own names
    def _active_map_callback(self, msg: String):
        try:
            info = json.loads(msg.data)
            self._map_info = info
            self._names = {}  # a new map: names given in another one do not apply
            self._names_file = os.path.join(info["dir"], f"{info['name']}_names.json")
            with open(self._names_file) as f:
                self._names = json.load(f)
        except (ValueError, KeyError, TypeError, OSError):
            pass

    def _save_names(self):
        if self._names_file:
            try:
                with open(self._names_file, "w") as f:
                    json.dump(self._names, f, indent=1)
            except OSError as e:
                print(f"Could not save names: {e}")

    def _object_names(self):
        """{object map name: user's name} — each name goes to the nearest object of its class near where
        it was given."""
        out, used = {}, set()
        for label, n in self._names.items():
            cands = [o for o in self.objects if o["class"] == n["class"] and o["name"] not in used
                     and math.hypot(o["x"] - n["x"], o["y"] - n["y"]) <= NAME_MATCH_RADIUS]
            if cands:
                o = min(cands, key=lambda o: math.hypot(o["x"] - n["x"], o["y"] - n["y"]))
                out[o["name"]] = label
                used.add(o["name"])
        return out

    def _name_this(self, label):
        """ "call this my chair": name the object just found or arrived at, else the nearest one ahead."""
        label = label[4:] if label.startswith("the ") else label
        label = label[3:] if label.startswith("as ") else label
        if not label:
            return
        pose = self.get_robot_pose()
        obj = next((o for o in self.objects if o["name"] == self.last_found_object), None)
        if obj is None and pose is not None:
            ahead = [o for o in self._static_objects() if not o.get("dynamic")
                     and lang._bearing(o, pose)[0] <= NAME_TARGET_RANGE and abs(lang._bearing(o, pose)[1]) < math.pi / 4]
            obj = min(ahead, key=lambda o: lang._bearing(o, pose)[0], default=None)
        if obj is None:
            self.speak("Which object? Find it first, then say call this, and the name.")
            return
        self._names[label] = {"class": obj["class"], "x": round(obj["x"], 3), "y": round(obj["y"], 3)}
        self._save_names()
        spoken = "your " + label[3:] if label.startswith("my ") else f"the {label}"
        self.speak(f"Okay. This {obj['class']} is now {spoken}.")

    def _named_request(self, verb, text):
        """ "go to my chair": an object the user named. False if the text is not one of their names."""
        key = text[4:] if text.startswith("the ") else text
        key = "my " + key[5:] if key.startswith("your ") else key
        n = self._names.get(key)
        if n is None:
            return False
        names = self._object_names()
        obj = next((o for o in self.objects if names.get(o["name"]) == key), None)
        spoken = "your " + key[3:] if key.startswith("my ") else f"the {key}"
        if obj is not None:
            self._pending = None
            self._found(verb, obj, self._static_objects(), self.get_robot_pose())
        elif verb == "go":
            self.speak(f"I can't see {spoken} right now. Taking you to where it was.")
            threading.Thread(target=self.navigate_to, args=(spoken, (n["x"], n["y"])), daemon=True).start()
        else:
            self.speak(f"I can't see {spoken} right now. Say go to {spoken} to walk to where it was.")
        return True

    # Choosing among several matches: nearest first, "another one" for the next
    def _offer_next(self, verb, cands, index, objects, pose):
        """Offer (find) or go to (go) the index-th nearest of several matches."""
        self._pending = [verb, cands, index]
        c = cands[index]
        cls = c["class"]
        plural = cls + ("es" if cls.endswith(("s", "sh", "ch", "x")) else "s")
        others = [x for x in cands if x is not c]
        where = lang.describe(c, objects, pose, others, **self._say_opts())
        which = "nearest" if index == 0 else ["", "second nearest", "third nearest"][index] if index < 3 \
            else f"number {index + 1}"
        head = f"There are {len(cands)} {plural}. " if index == 0 else ""
        if verb == "go":
            self.speak(f"{head}Taking you to the {which} one: {where}. Say another one for a different {cls}.")
            self._start_nav(c, objects)
        else:
            self.speak(f"{head}The {which} one is {where}. Say go there, or another one.")
        self.last_found_object = c["name"]

    def _next(self):
        verb, cands, index = self._pending
        index += 1
        if index >= len(cands):
            self.speak("That was the last one. Back to the nearest.")
            index = 0
        if verb == "go" and self.navigating:
            self._stop_nav()
        self._offer_next(verb, cands, index, self._static_objects(), self.get_robot_pose())

    def _list(self):
        verb, cands, _ = self._pending
        objects, pose = self._static_objects(), self.get_robot_pose()
        parts = []
        for k, c in enumerate(cands[:MAX_OFFERED]):
            others = [x for x in cands if x is not c]
            parts.append(f"{['First', 'Second', 'Third'][k]}: {lang.describe(c, objects, pose, others, **self._say_opts())}.")
        self.speak(" ".join(parts) + " Say go to the first one, the second one, or another one.")

    def _stop_nav(self):
        self.navigating = False
        if self._using_nav2:
            self._semantic_goal_pub.publish(String(data="stop"))
        time.sleep(0.3)

    def _start_nav(self, obj, objects, grasp=False):
        if self.navigating:
            self._stop_nav()  # a new destination replaces the old one
        self._grasp_on_arrival = grasp
        name = lang.spoken_name(obj, objects, **self._say_opts())
        spoken = name[4:] if name.startswith("the ") else name
        threading.Thread(target=self.navigate_to, args=(obj["name"],), kwargs={"spoken": spoken},
                         daemon=True).start()

    def _found(self, verb, obj, objects, pose):
        """One object matched: say where it is, or start guiding the user to it."""
        self._pending = None
        self.last_found_object = obj["name"]
        if verb == "go":
            name = lang.spoken_name(obj, objects, **self._say_opts())
            self.speak(f"Taking you to {name}. {lang.where(obj, pose).capitalize()}.")
            self._start_nav(obj, objects)
        else:
            others = [x for x in objects if x["class"] == obj["class"] and x is not obj]
            desc = lang.describe(obj, objects, pose, others, **self._say_opts())
            self.speak(f"{desc[0].upper()}{desc[1:]}. Say go there to be guided.")

    @staticmethod
    def _spoken_id(text):
        """ "chair one" / "the chair number 1" / "chair_1" -> "chair_1" (the IDs on the map and camera window)."""
        words = [w for w in text.strip().split() if w not in ("the", "a", "number", "no")]
        if len(words) >= 2 and (words[-1].isdigit() or words[-1] in NUMBER_WORDS):
            words[-1] = words[-1] if words[-1].isdigit() else str(NUMBER_WORDS[words[-1]])
        return "_".join(words)

    def _object_request(self, verb, text):
        """Handle "find ..." / "go to ..." for a spoken description. Returns False if nothing on the map
        is of the requested kind (the caller may then try a named place)."""
        if self._named_request(verb, text):
            return True
        objects = self._static_objects()
        pose = self.get_robot_pose()
        by_name = {o["name"]: o for o in objects}
        exact = self._spoken_id(text)
        if exact in by_name:  # an ID, typed ("chair_1") or said ("chair 1", "chair one", "the chair number one")
            self._found(verb, by_name[exact], objects, pose)
            return True
        query = lang.parse(text, {o["class"] for o in objects})
        if query.target is None:
            return False
        cands = lang.resolve(query, objects, pose)
        if len(cands) == 1:
            self._found(verb, cands[0], objects, pose)
        elif cands:
            if self.navigating:
                self._stop_nav()
            self._offer_next(verb, cands, 0, objects, pose)
        else:
            same_kind = [o for o in objects if o["class"] == query.target.cls]
            msg = f"I have not seen a {lang.query_text(query)} yet."
            if same_kind and (query.relations or query.target.color):
                n = len(same_kind)
                msg += f" I know {n} {query.target.cls}{'s' if n > 1 else ''}. Say find {query.target.cls} to hear them."
            self.speak(msg)
        return True

    def _choose(self, text):
        """Follow-up to several matches ("the second one", "the one next to the door", "the one in the kitchen")."""
        verb, offered, _ = self._pending
        objects = self._static_objects()
        pose = self.get_robot_pose()
        classes = {o["class"] for o in objects}
        plain = lang.parse(text, classes)
        lead = text.split()[:plain.target.start] if plain.target is not None else []
        answer = any(w in ("one", "that", "which", "ones") for w in lead)  # "the one next to the door"
        if plain.target is not None and plain.target.cls != offered[0]["class"] and not answer:
            # Not an answer but a new request ("go to the nearest door" while choosing a chair)
            self._pending = None
            go = text.startswith(GO_COMMANDS)
            prefix = next((p for p in GO_COMMANDS + FIND_COMMANDS if text.startswith(p)), "")
            self._object_request("go" if go else "find", text[len(prefix):])
            return
        words = text.split()
        if words and words[0] in ("go", "take"):  # "go to the first one": the choice becomes a navigation
            verb = "go"
        query = lang.parse(text, classes, default_class=offered[0]["class"])
        if isinstance(query.selector, int):
            pick = offered[query.selector] if -len(offered) <= query.selector < len(offered) else None
            picks = [pick] if pick else []
        else:
            names = {o["name"] for o in offered}
            picks = [o for o in lang.resolve(query, objects, pose) if o["name"] in names] if (
                query.relations or query.target.color or query.selector) else []
            # "the one in the kitchen": a saved room
            room = next((p for p in self.named_places if f" {p}" in f" {text}"), None)
            if room and not picks:
                picks = [o for o in offered if lang.place_phrase(o, {room: self.named_places[room]})]
        if len(picks) == 1:
            if verb == "go" and self.navigating:
                self._stop_nav()
            self._found(verb, picks[0], objects, pose)
        elif picks:
            self._offer_next(verb, picks, 0, objects, pose)
        else:
            self.speak("Sorry, which one? Say another one, list them, or describe where it is.")

    def _whats_on(self, text):
        """ "what is on the table": the things resting on it. False if there is no such object on the map."""
        objects = self._static_objects()
        pose = self.get_robot_pose()
        query = lang.parse(text, {o["class"] for o in objects})
        if query.target is None:
            return False
        cands = lang.resolve(query, objects, pose)
        if not cands:
            return False
        sentences = []
        for support in cands[:MAX_OFFERED]:
            items = [o for o in objects if lang.is_on(o, support, objects)]
            name = f"the {support['class']}" + (f", {lang.where(support, pose)}," if len(cands) > 1 else "")
            things = [f"a {o.get('color') + ' ' if SPEAK_COLORS and o.get('color') else ''}{o['class']}" for o in items]
            if not things:
                sentences.append(f"I have not seen anything on {name}.".replace(",.", "."))
            else:
                listed = things[0] if len(things) == 1 else ", ".join(things[:-1]) + " and " + things[-1]
                sentences.append(f"On {name} there is {listed}.")
        self.speak(" ".join(sentences))
        return True

    def _around(self):
        """The nearest objects with their directions ("what is around me"). Outdoors: what is ahead now."""
        if self._mode == "outdoor":
            sc = self._scene()
            self.speak(sc["summary"] if sc else "The camera AI is not seeing anything yet.")
            return
        objects = self._static_objects()
        pose = self.get_robot_pose()
        if pose is None or not objects:
            self.speak("I have not mapped anything around you yet.")
            return
        names = self._object_names()
        near = sorted(objects, key=lambda o: math.hypot(o["x"] - pose[0], o["y"] - pose[1]))[:AROUND_MAX]
        parts = []
        for o in near:
            own = lang.label_of(o, names)
            color = o.get("color") if SPEAK_COLORS and o["class"] not in lang.FURNITURE else None
            name = own or f"a {color + ' ' if color else ''}{o['class']}"
            parts.append(f"{name} {lang.where(o, pose)}")
        self.speak("Around you: " + "; ".join(parts) + ".")

    def _grasp_callback(self, msg: String):
        try:
            self._grasp_status = json.loads(msg.data)
        except ValueError:
            pass

    def start_grasp(self, obj_class):
        """Guide the hand to the object in a background thread (stops any grasp already running)."""
        self.grasping = False
        time.sleep(0.2)
        threading.Thread(target=self._grasp_loop, args=(obj_class,), daemon=True).start()

    @staticmethod
    def _grasp_cue(status, obj):
        """One short spoken cue: left/right first, then height, then forward."""
        state = status.get("state")
        if state == "no_target":
            return f"I can't see the {obj}. Step back a little, or turn toward it."
        if state == "no_hand":
            return f"Reach out your hand toward the {obj}."
        if state != "tracking":
            return None

        def inches(v):
            n = max(1, int(round(abs(v) * 39.37)))
            return f"{n} inch" if n == 1 else f"{n} inches"
        right, up, forward = status["right"], status["up"], status["forward"]
        if abs(right) > GRASP_TOL:
            return f"{'Right' if right > 0 else 'Left'} {inches(right)}."
        if abs(up) > GRASP_TOL:
            return f"{'Higher' if up > 0 else 'Lower'} {inches(up)}."
        if abs(forward) > GRASP_FORWARD_TOL:
            return f"{'Forward' if forward > 0 else 'Back'} {inches(forward)}."
        return None

    def _grasp_loop(self, obj):
        self.grasping, self._grasp_status = True, None
        self._grasp_cmd_pub.publish(String(data=f"start {obj}"))
        print(f"\n✋ GRASP MODE: {obj}   — type 'stop' to cancel")
        self.speak(f"Reach out your hand toward the {obj}.")
        start, last_cue, last_time = time.time(), None, 0.0
        while rclpy.ok() and self.grasping:
            status = self._grasp_status or {}
            state = status.get("state")
            if state == "unavailable":
                self.speak("Hand tracking is not available.")
                break
            if state == "reached":
                self.speak(f"Stop. The {obj} is at your hand.")
                break
            if time.time() - start > GRASP_TIMEOUT_S:
                self.speak("Grasp mode stopped.")
                break
            cue = self._grasp_cue(status, obj)
            now = time.time()
            if cue and now - last_time > GRASP_CUE_GAP_S and (cue != last_cue or now - last_time > 4.0):
                print(f"  ✋ {cue}")
                self.speak(cue)
                last_cue, last_time = cue, time.time()
            time.sleep(0.1)
        self._grasp_cmd_pub.publish(String(data="stop"))
        self.grasping = False

    def _places_callback(self, msg: String):
        try:
            self.named_places = json.loads(msg.data)
        except ValueError:
            pass

    def _where_am_i(self):
        pose = self.get_robot_pose()
        if pose is None or not self.named_places:
            self.speak("I don't know yet. Save places with: save this place as kitchen.")
            return
        name, p = min(self.named_places.items(), key=lambda kv: math.hypot(kv[1]['x'] - pose[0], kv[1]['y'] - pose[1]))
        dist = math.hypot(p['x'] - pose[0], p['y'] - pose[1])
        self.speak(f"You are at the {name}." if dist < 1.5 else f"You are {int(dist * 3.28084)} feet from the {name}.")

    def _nav2_status_callback(self, msg: String):
        try:
            self._nav2_status = json.loads(msg.data)
        except ValueError:
            pass

    def _nav2_path_callback(self, msg: Path):
        if self._using_nav2:
            self._nav2_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _nav2_available(self):
        """semantic_navigator.py is running (it subscribes to /semantic_goal)."""
        backend = os.environ.get("WEARABLE_NAV_BACKEND", "auto").lower()
        if backend == "astar":
            return False
        return self.count_subscribers('/semantic_goal') > 0

    def _map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def get_robot_pose(self):
        """Get current robot position and heading from TF."""
        try:
            transform = self._tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return rx, ry, yaw
        except Exception:
            return None

    def get_relative_direction(self, robot_yaw, target_x, target_y, robot_x, robot_y):
        """Calculate clock-face direction and turn instruction."""
        target_angle = math.atan2(target_y - robot_y, target_x - robot_x)
        rel_angle = target_angle - robot_yaw
        while rel_angle > math.pi: rel_angle -= 2 * math.pi
        while rel_angle < -math.pi: rel_angle += 2 * math.pi
        
        clock_hr = int(round(12 - (rel_angle * 6 / math.pi))) % 12
        if clock_hr == 0: clock_hr = 12
        
        return rel_angle, clock_hr

    def keyboard_listener_loop(self):
        import select
        import sys
        while rclpy.ok():
            # Wait up to 1 second for keyboard input without blocking permanently
            i, o, e = select.select([sys.stdin], [], [], 1.0)
            if i:
                cmd = sys.stdin.readline().strip().lower()
                if cmd:
                    self.command_queue.put(cmd)

    def main_logic_loop(self):
        time.sleep(2)
        # Spoken greeting: _system_watch (it knows whether the Pi and its sensors are there)
        print("\n" + "=" * 50)
        print("⌨️  READY FOR COMMANDS")
        print("  - Type commands here, or use the buttons and push-to-talk.")
        print("  - Commands: 'find [object]', 'go to [object or place]', 'describe...'")
        print("              e.g. 'find the table with the red cup', 'go to the chair next to the door',")
        print("              'what is on the table', 'what is around me', then 'go there' / 'another one'")
        print("              'call this my chair' (then 'go to my chair'), 'forget name my chair'")
        print("              'save this place as [name]', 'where am i', 'forget place [name]' (until MODE hold)")
        print("              'grasp [object]' (hand guidance; also starts on arrival at an object)")
        print("=" * 50 + "\n")
        
        # Typed commands (for testing at the laptop)
        threading.Thread(target=self.keyboard_listener_loop, daemon=True).start()
        
        while rclpy.ok():
            try:
                # A command: typed, or spoken with push-to-talk
                target = self.command_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            # "Where's the cup?" -> "where s the cup" (IDs such as chair_1 keep their underscore)
            target = " ".join(re.sub(r"[^a-z0-9_ ]", " ", target.lower()).split())
            
            if target in ('exit', 'shut down', 'shutdown'):
                self.speak("Shutting down.")
                rclpy.shutdown()
                break

            elif target in STOP_COMMANDS:
                # Spoken with push-to-talk as often as the STOP button is pressed: it never shuts the assistant down
                if self.grasping:
                    self.grasping = False
                    self.speak("Grasp mode off.")
                elif self.navigating:
                    self.navigating = False
                    if self._using_nav2:
                        self._semantic_goal_pub.publish(String(data="stop"))
                    self.speak("Navigation stopped.")
                    cancel_path = Path()
                    cancel_path.header.frame_id = "map"
                    cancel_path.header.stamp = self.get_clock().now().to_msg()
                    self._path_pub.publish(cancel_path)
                else:
                    self._stop_all()
                
            elif target in STATUS_COMMANDS:
                self._status()

            elif target in VISION_OFF_COMMANDS:
                threading.Thread(target=self._vision_off, daemon=True).start()

            elif self._mode == "outdoor" and self._outdoor_command(target):
                pass  # answered from what the camera sees now (outdoor_awareness.py)

            elif target in HELP_COMMANDS:
                self.speak(HELP_TEXT)

            elif target in AROUND_COMMANDS:
                self._around()

            elif target.startswith(("what is on ", "what s on ", "whats on ")) and self._whats_on(target):
                pass  # answered from the object map; otherwise the camera VLM below is asked

            elif target == "describe" or target.startswith(("describe ", "what ", "read ")):
                # In a thread: it starts the vision AI first when it is off (half a minute)
                question = "Describe what you see in this image in one sentence." if target == "describe" else target
                threading.Thread(target=self._ask_vision, args=(question,),
                                 daemon=True).start()

            elif target.startswith(GRASP_COMMANDS):
                prefix = next(p for p in GRASP_COMMANDS if target.startswith(p))
                obj = target[len(prefix):].strip()
                obj = obj[4:] if obj.startswith("the ") else obj
                if obj:
                    self.start_grasp(obj)

            elif target in ("save map", "save the map"):
                # Maps are not kept for another day (the wearer's choice): one lives as long as its session
                self.speak("The map is kept until you hold the mode button. It is not saved for another day.")

            elif target.startswith(PLACE_COMMANDS):
                prefix = next(p for p in PLACE_COMMANDS if target.startswith(p))
                name = target[len(prefix):].strip()
                if name:
                    self._map_cmd_pub.publish(String(data=f"place {name}"))

            elif target in WHO_COMMANDS:
                threading.Thread(target=self._recognize, daemon=True).start()

            elif target.startswith(("forget face ", "forget the face of ")):
                who = target.split("face ", 1)[1].removeprefix("of ").strip()
                if self._load_faces() is not None:
                    self.speak(f"Forgot {who.title()}." if self._faces.forget(who)
                               else f"I don't know anyone called {who.title()}.")

            elif target.startswith("forget place "):
                self._map_cmd_pub.publish(String(data=f"forget {target[len('forget place '):].strip()}"))

            elif target in ("where am i", "where am i?"):
                self._where_am_i()

            elif self._pending and lang.is_next(target):
                self._next()

            elif self._pending and target in LIST_WORDS:
                self._list()

            elif self._pending and lang.is_selection(target) and not target.startswith(FIND_COMMANDS):
                self._choose(target)

            elif target.startswith(NAME_COMMANDS):
                prefix = next(p for p in NAME_COMMANDS if target.startswith(p))
                self._name_this(target[len(prefix):].strip())

            elif target.startswith(("forget name ", "forget the name ")):
                label = target.split("name ", 1)[1].strip()
                if self._names.pop(label, None) is None:
                    self.speak(f"Nothing is called {label}.")
                else:
                    self._save_names()
                    self.speak(f"Forgot the name {label}.")

            elif target in GO_THERE:
                obj = next((o for o in self.objects if o["name"] == self.last_found_object), None)
                if obj is None:
                    self.speak("Where to? Say find, and describe what you are looking for.")
                else:
                    self._found("go", obj, self._static_objects(), self.get_robot_pose())

            elif target.startswith(FIND_COMMANDS):
                prefix = next(p for p in FIND_COMMANDS if target.startswith(p))
                if not self._object_request("find", target[len(prefix):]):
                    self.speak(f"I have not seen {target[len(prefix):]} yet. Keep walking.")

            elif target.startswith(GO_COMMANDS):
                prefix = next(p for p in GO_COMMANDS if target.startswith(p))
                dest_term = target[len(prefix):].strip()
                dest_term = dest_term[4:] if dest_term.startswith("the ") else dest_term
                place = self.named_places.get(dest_term)
                if place is not None:
                    self.speak(f"Starting navigation to the {dest_term}.")
                    threading.Thread(target=self.navigate_to, args=(dest_term, (place['x'], place['y'])),
                                     daemon=True).start()
                elif not self._object_request("go", dest_term):
                    self.speak(f"I don't know where {dest_term} is.")

            elif target.startswith(QUESTION_STARTS):
                # Any other question is for the camera ("how many people are here")
                threading.Thread(target=self._ask_vision, args=(target,),
                                 daemon=True).start()

            else:
                print("❓ Use: find <object>, go to <object or place>, what is around me, what is on the <object>, "
                      "save this place as <name>, where am i, status")

    # ── CONTINUOUS TURN-BY-TURN NAVIGATION ──
    def navigate_to(self, target_name, place=None, spoken=None):
        """Continuously guide the user to an object, or to a named place (place = (x, y)), by voice.
        `spoken` is how the object is named aloud ("table with the red cup on it"), never its ID."""
        if self._nav2_available():
            self._navigate_nav2(target_name, place, spoken)
            return
        self._nav_gen += 1
        gen = self._nav_gen
        self.navigating = True
        friendly_name = spoken or (target_name if place else
                                   ''.join(c for c in target_name if not c.isdigit()).replace("_", " ").strip())
        
        if self.map_data is None:
            self.speak("No map available yet.")
            self.navigating = False
            return
        
        if place:
            tx, ty = place
        else:
            target_pos = self.saved_objects.get(target_name)
            if not target_pos:
                self.speak(f"Lost track of {friendly_name}.")
                self.navigating = False
                return
            # Locked once: the route must not follow (or lose) the live detection near the object
            tx, ty = target_pos.x, target_pos.y

        last_instruction = ""
        last_speech_time = 0
        last_ft = None
        recalc_counter = 0
        current_grid_path = None
        
        print("\n" + "=" * 50)
        print(f"🧭 NAVIGATING TO: {friendly_name}")
        print("   Type 'stop' to cancel navigation")
        print("=" * 50)
        
        while rclpy.ok() and self.navigating and gen == self._nav_gen:
            pose = self.get_robot_pose()
            if pose is None:
                time.sleep(0.1)
                continue
                
            rx, ry, robot_yaw = pose
            dist_to_target = math.hypot(tx - rx, ty - ry)
            dist_ft = dist_to_target * 3.28084
            
            # ---- ARRIVAL CHECK (SLAM distance only: the object is in the blind spot by now) ----
            if dist_to_target < (ARRIVAL_RADIUS if place else OBJECT_ARRIVAL_RADIUS):
                self._arrive(friendly_name, place is not None, target_name.rsplit('_', 1)[0].replace('_', ' '),
                             target=(tx, ty))
                break
                
            # ---- RECALCULATE A* PATH every 1.0 second (10 cycles) ----
            if recalc_counter % 10 == 0:
                new_path = self._calculate_path(rx, ry, tx, ty)
                if new_path:
                    current_grid_path = new_path
            recalc_counter += 1
            
            # ---- DYNAMIC PATH PUBLISHING (10 Hz) ----
            # Strip waypoints that we have already passed
            if current_grid_path:
                # Find the closest waypoint to the robot
                min_idx = 0
                min_d = float('inf')
                for i, (gx, gy) in enumerate(current_grid_path):
                    wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
                    d = math.hypot(wx - rx, wy - ry)
                    if d < min_d:
                        min_d = d
                        min_idx = i
                # Keep only waypoints from the closest one onwards
                current_grid_path = current_grid_path[min_idx:]
                # Publish anchored to the wearer's real-time position
                self._publish_path(current_grid_path, rx, ry, tx, ty)
            
            # ---- PURE PURSUIT LOOKAHEAD STEERING ----
            lookahead_dist = max(1.5, min(3.0, dist_to_target * 0.3))
            waypoint_x, waypoint_y = tx, ty
            if current_grid_path and len(current_grid_path) > 1:
                accumulated_dist = 0.0
                prev_wx, prev_wy = self.grid_to_world(current_grid_path[0][0], current_grid_path[0][1], self.map_data.info)
                for i in range(1, len(current_grid_path)):
                    curr_wx, curr_wy = self.grid_to_world(current_grid_path[i][0], current_grid_path[i][1], self.map_data.info)
                    segment_dist = math.hypot(curr_wx - prev_wx, curr_wy - prev_wy)
                    
                    if accumulated_dist + segment_dist >= lookahead_dist:
                        ratio = (lookahead_dist - accumulated_dist) / segment_dist if segment_dist > 0 else 0
                        waypoint_x = prev_wx + ratio * (curr_wx - prev_wx)
                        waypoint_y = prev_wy + ratio * (curr_wy - prev_wy)
                        break
                        
                    accumulated_dist += segment_dist
                    prev_wx, prev_wy = curr_wx, curr_wy
            
            # ---- CALCULATE DIRECTION ----
            rel_angle, clock_hr = self.get_relative_direction(robot_yaw, waypoint_x, waypoint_y, rx, ry)
            
            # ---- GENERATE INSTRUCTION ----
            instruction = self._instruction(rel_angle, clock_hr, dist_ft)

            # ---- SPEAK INSTRUCTION ----
            instruction_type = instruction.split(".")[0]
            now = time.time()
            if self._say_again(instruction_type, dist_ft, last_instruction, last_ft, last_speech_time, now):
                self.speak(instruction)
                last_instruction, last_ft = instruction_type, dist_ft
                last_speech_time = now
                print(f"  📍 {instruction}")
            
            time.sleep(0.1)

        if gen == self._nav_gen:  # not replaced by a newer navigation
            self.navigating = False
        print("\n✅ Navigation ended.\n")

    def _arrive(self, friendly_name, is_place=False, obj_class=None, target=None):
        """Arrival: say where the object is from here and clear the drawn path; the navigation ends. Hand guidance
        follows only when the HAND button asked for it."""
        pose = self.get_robot_pose()
        if is_place or target is None or pose is None:
            self.speak(f"You have arrived at the {friendly_name}.")
        else:
            # Where it is from here, so the last step is the wearer's own and not a guess
            rel, clock_hr = self.get_relative_direction(pose[2], target[0], target[1], pose[0], pose[1])
            feet = max(1, int(round(math.hypot(target[0] - pose[0], target[1] - pose[1]) * 3.28084)))
            self.speak(f"You have arrived. The {friendly_name} is at {clock_hr} o'clock, about "
                       f"{feet} {'foot' if feet == 1 else 'feet'} away.")
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        self._path_pub.publish(empty_path)
        if not is_place and obj_class and self._grasp_on_arrival:
            self.start_grasp(obj_class)
        self._grasp_on_arrival = False

    @staticmethod
    def _instruction(rel_angle, clock_hr, dist_ft):
        """Spoken turn-by-turn instruction toward the lookahead point: always the clock direction to turn to and
        the feet left (close to the goal they were dropped: "Turn left now. Almost there." said neither)."""
        abs_angle_deg = abs(math.degrees(rel_angle))
        feet = max(1, int(round(dist_ft)))
        left = f"{feet} foot" if feet == 1 else f"{feet} feet"
        if abs_angle_deg > 45:
            return f"Turn {'left' if rel_angle > 0 else 'right'}, to {clock_hr} o'clock. {left}."
        if abs_angle_deg > 15:
            return f"Bear slightly {'left' if rel_angle > 0 else 'right'}, {clock_hr} o'clock. {left}."
        return f"Straight ahead. {left}."

    @staticmethod
    def _say_again(kind, dist_ft, last_kind, last_ft, last_t, now):
        """Say the next instruction when the direction changes, every 8 s, or (within 12 feet) every 2 feet."""
        return (kind != last_kind or now - last_t > 8.0
                or (dist_ft <= 12 and last_ft is not None and last_ft - dist_ft >= 2.0 and now - last_t > 1.5))

    def _navigate_nav2(self, target_name, place=None, spoken=None):
        """Guide the user along the smooth Nav2 path from semantic_navigator.py (re-planned every second)."""
        if place:
            friendly_name, (tx, ty) = target_name, place
        else:
            friendly_name = spoken or ''.join(c for c in target_name if not c.isdigit()).replace("_", " ").strip()
            target_pos = self.saved_objects.get(target_name)
            if not target_pos:
                self.speak(f"Lost track of {friendly_name}.")
                return
            # Lock the goal in the map frame: Nav2 routes to this fixed point, not the live detection
            tx, ty = target_pos.x, target_pos.y
        self._nav_gen += 1
        gen = self._nav_gen
        self.navigating, self._using_nav2 = True, True
        self._nav2_status, self._nav2_path = None, []
        goal = {"name": target_name.replace(" ", "_"), "x": tx, "y": ty}
        if place:
            goal["place"] = True  # walk to the point itself (no object to stop in front of)
        self._semantic_goal_pub.publish(String(data=json.dumps(goal)))
        print(f"\n🧭 NAVIGATING (Nav2) TO: {friendly_name}   — type 'stop' to cancel")
        last_instruction, last_speech_time, last_problem, last_ft = "", 0.0, None, None
        while rclpy.ok() and self.navigating and gen == self._nav_gen:
            status = self._nav2_status or {}
            state = status.get("state")
            if state == "arrived":
                self._arrive(friendly_name, place is not None, target_name.rsplit('_', 1)[0].replace('_', ' '),
                             target=(tx, ty))
                break
            if state in ("no_path", "blocked", "not_found", "no_planner") and state != last_problem:
                last_problem = state
                self.speak({"no_path": "The way is blocked right now. Please wait.",
                            "blocked": f"There is no free space next to the {friendly_name}.",
                            "not_found": f"I can't see the {friendly_name} on the map any more.",
                            "no_planner": "The path planner is not running."}[state])
            pose = self.get_robot_pose()
            if pose is not None and math.hypot(tx - pose[0], ty - pose[1]) < (ARRIVAL_RADIUS if place
                                                                                else OBJECT_ARRIVAL_RADIUS):
                self._arrive(friendly_name, place is not None, target_name.rsplit('_', 1)[0].replace('_', ' '),
                             target=(tx, ty))
                break
            path = self._nav2_path
            if pose is None or len(path) < 2:
                time.sleep(0.1)
                continue
            rx, ry, robot_yaw = pose
            # Drop the part of the path already walked, then look ahead along the rest
            nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - rx, path[i][1] - ry))
            ahead = path[nearest:]
            remaining = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(ahead, ahead[1:]))
            lookahead = max(1.0, min(2.5, remaining * 0.3))
            wx, wy = ahead[-1]
            travelled = 0.0
            for a, b in zip(ahead, ahead[1:]):
                seg = math.hypot(b[0] - a[0], b[1] - a[1])
                if travelled + seg >= lookahead:
                    k = (lookahead - travelled) / seg if seg > 0 else 0.0
                    wx, wy = a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1])
                    break
                travelled += seg
            rel_angle, clock_hr = self.get_relative_direction(robot_yaw, wx, wy, rx, ry)
            instruction = self._instruction(rel_angle, clock_hr, remaining * 3.28084)
            kind = instruction.split(".")[0]
            now = time.time()
            if self._say_again(kind, remaining * 3.28084, last_instruction, last_ft, last_speech_time, now):
                self.speak(instruction)
                print(f"  📍 {instruction}")
                last_instruction, last_speech_time, last_ft = kind, now, remaining * 3.28084
            time.sleep(0.1)
        if gen == self._nav_gen:  # not replaced by a newer navigation
            self._semantic_goal_pub.publish(String(data="stop"))
            self.navigating, self._using_nav2 = False, False
        print("\n✅ Navigation ended.\n")

    def smooth_path_chaikin(self, path, iterations=3):
        """Smooth a list of points with Chaikin's corner cutting."""
        if len(path) <= 2:
            return path
        for _ in range(iterations):
            new_path = [path[0]]
            for i in range(len(path) - 1):
                p0 = path[i]
                p1 = path[i+1]
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                new_path.extend([q, r])
            new_path.append(path[-1])
            path = new_path
        return path

    def _calculate_path(self, rx, ry, tx, ty):
        """Calculate A* path from robot to target."""
        if self.map_data is None:
            return None
        start_grid = self.world_to_grid(rx, ry, self.map_data.info)
        goal_grid = self.world_to_grid(tx, ty, self.map_data.info)
        raw_path = self.a_star(start_grid, goal_grid, self.map_data)
        if raw_path:
            return self.smooth_path_chaikin(raw_path, iterations=3)
        return [start_grid, goal_grid]  # Fallback to straight line if A* fails

    def _publish_path(self, grid_path, rx, ry, tx, ty):
        """Publish the path to RViz, anchored to the wearer and the target."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add exact robot position
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(rx)
        pose.pose.position.y = float(ry)
        path.poses.append(pose)
        
        # Add grid waypoints
        for (gx, gy) in grid_path:
            wx, wy = self.grid_to_world(gx, gy, self.map_data.info)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            path.poses.append(pose)
            
        # Add exact target position
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(tx)
        pose.pose.position.y = float(ty)
        path.poses.append(pose)
        
        self._path_pub.publish(path)

    def world_to_grid(self, x, y, map_info):
        gx = int((x - map_info.origin.position.x) / map_info.resolution)
        gy = int((y - map_info.origin.position.y) / map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, map_info):
        wx = (gx + 0.5) * map_info.resolution + map_info.origin.position.x
        wy = (gy + 0.5) * map_info.resolution + map_info.origin.position.y
        return wx, wy

    def a_star(self, start_idx, goal_idx, map_msg):
        w = map_msg.info.width
        h = map_msg.info.height
        data = map_msg.data
        
        def is_free(gx, gy, radius=4):
            if gx < radius or gx >= w - radius or gy < radius or gy >= h - radius: return False
            # Check bounding box (square is faster than circle in Python)
            for dy in range(-radius, radius + 1):
                row_idx = (gy + dy) * w
                for dx in range(-radius, radius + 1):
                    val = data[row_idx + gx + dx]
                    if val >= 50:  # Allow -1 (unknown space) to be traversed
                        return False
            return True
            
        def get_cost(gx, gy):
            if not is_free(gx, gy, radius=2):
                return float('inf')
            
            if not is_free(gx, gy, radius=3):
                base_cost = 5.0
            elif not is_free(gx, gy, radius=4):
                base_cost = 3.0
            elif not is_free(gx, gy, radius=5):
                base_cost = 2.0
            elif not is_free(gx, gy, radius=6):
                base_cost = 1.5
            else:
                base_cost = 1.0

            near_landmark = False
            for obj_name, pos in self.saved_objects.items():
                ox, oy = self.world_to_grid(pos.x, pos.y, map_msg.info)
                dist_cells = math.hypot(gx - ox, gy - oy)
                
                is_dynamic = any(k in obj_name for k in ["dynamic", "person", "human"])
                if is_dynamic:
                    if dist_cells < 40:
                        base_cost += 20.0 / (dist_cells + 1.0)
                else:
                    if dist_cells <= 30:
                        near_landmark = True
                        
            if near_landmark:
                base_cost *= 0.70
                
            return base_cost
            
        def line_of_sight(p1, p2):
            x0, y0 = p1
            x1, y1 = p2
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx = 1 if x0 < x1 else -1
            sy = 1 if y0 < y1 else -1
            err = dx - dy
            
            while True:
                if not is_free(x0, y0, radius=3): return False
                if x0 == x1 and y0 == y1: break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx
                if e2 < dx:
                    err += dx
                    y0 += sy
            return True

        sx, sy = start_idx
        gx, gy = goal_idx
        
        # Try to find a nearby free spot if start is stuck in a wall
        if not is_free(sx, sy, 3):
            found_free = False
            for r in range(1, 10):
                for dx in range(-r, r+1):
                    for dy in range(-r, r+1):
                        if is_free(sx+dx, sy+dy, 3):
                            sx += dx
                            sy += dy
                            found_free = True
                            break
                    if found_free: break
                if found_free: break
        
        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0}
        best_node = (sx, sy)
        min_dist = math.hypot(sx - gx, sy - gy)

        raw_path = []
        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            dist = math.hypot(cx - gx, cy - gy)
            
            if dist < min_dist:
                min_dist = dist
                best_node = (cx, cy)
                
            if dist <= 3:
                curr = (cx, cy)
                while curr in came_from:
                    raw_path.append(curr)
                    curr = came_from[curr]
                raw_path.reverse()
                break
                
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]:
                nx, ny = cx + dx, cy + dy
                if not is_free(nx, ny, radius=4):
                    continue
                cost = (1.414 if dx != 0 and dy != 0 else 1.0) * get_cost(nx, ny)
                if cost == float('inf'):
                    continue
                
                parent = came_from.get((cx, cy))
                if parent and line_of_sight(parent, (nx, ny)):
                    tentative_g = g_score[parent] + math.hypot(parent[0] - nx, parent[1] - ny)
                    came_from_node = parent
                else:
                    tentative_g = g_score[(cx, cy)] + cost
                    came_from_node = (cx, cy)
                    
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = came_from_node
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_set, (f_score, nx, ny))
                    
        if not raw_path:
            curr = best_node
            while curr in came_from:
                raw_path.append(curr)
                curr = came_from[curr]
            raw_path.reverse()
            
        if len(raw_path) <= 2:
            return raw_path
            
        # Path Smoothing (String Pulling)
        smoothed_path = [raw_path[0]]
        current = raw_path[0]
        for i in range(1, len(raw_path)):
            if not line_of_sight(current, raw_path[i]):
                smoothed_path.append(raw_path[i-1])
                current = raw_path[i-1]
        smoothed_path.append(raw_path[-1])
        
        return smoothed_path

def main(args=None):
    rclpy.init(args=args)
    node = FindObjectNode()
    # Closing the terminal (SIGHUP) or a kill (SIGTERM) also stops the programs it started, as Ctrl+C does
    import signal
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, lambda *_: rclpy.shutdown() if rclpy.ok() else None)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # A second Ctrl+C must not break off the clean-up (that left the parts running on their own)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print("Stopping the programs it started (a few seconds)...")
        node._sys.stop_all()  # the parts it started (brain, camera AI, vision AI, outdoor view)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
