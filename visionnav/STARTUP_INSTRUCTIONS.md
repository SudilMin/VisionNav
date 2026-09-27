# VisionNav Startup Instructions (Mega Upgrade Edition)

This file contains the complete, up-to-date sequence of commands needed to launch the entire VisionNav system across both the Raspberry Pi and the Laptop.

> **⚠️ IMPORTANT:** The Pi commands (Terminals 1-2) must be run on the **Raspberry Pi over SSH**.
> The Laptop commands (Terminals 3-6) must be run **locally on your laptop**.
> Do NOT run `pi_sensors.launch.py` on the laptop — there is no LiDAR connected to it!

---

## 🔨 0. One-Time Setup (build the workspace)
Run on **both** the laptop and the Pi after pulling changes. The ROS 2 package is `visionnav`
(`src/visionnav`); its nodes live in `src/visionnav/visionnav/` and model weights in `src/visionnav/models/`.

```bash
cd ~/wearable_ws
rm -rf build/wearable_sim install/wearable_sim   # only once, after the rename from wearable_sim
colcon build --symlink-install --packages-select visionnav
source install/setup.bash
```

**Node names** (renamed so each says what it does; old name → new name):
`vision_perception` → `object_perception`, `find_object` → `voice_navigation_assistant`,
`phone_camera` → `phone_camera_publisher`, `esp32_bridge` → `esp32_button_haptics_bridge`,
`scan_body_filter` → `lidar_body_filter`, `check_lidar_orientation` → `lidar_orientation_calibrator`,
`gps_nav` → `gps_voice_navigator`, `structure_mapper` → `wall_structure_mapper`.
Topics are unchanged.

**Hand tracking for grasp mode (laptop, one time).** Install MediaPipe *without* its dependencies: a normal
install pulls NumPy 2 and a second OpenCV, which breaks ROS (cv_bridge, matplotlib). Then fetch the hand model:
```bash
python3 -m pip install --user --break-system-packages --no-deps mediapipe==1.0.1 absl-py
curl -sSfL -o ~/wearable_ws/src/visionnav/models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

**Object detector (laptop):** the vision node uses **YOLOE-11s-seg** (open-vocabulary) with the indoor
vocabulary in `VOCABULARY` at the top of `visionnav/object_perception.py` (doors, light switches, wall sockets,
stairs, holes in the floor, furniture, …). On the first start, or after the vocabulary is edited, it compiles a
TensorRT FP16 engine for the RTX 2050 into `models/` automatically (≈2 min, one time) before the ROS loop starts.
To detect something new, add its name to `VOCABULARY`; the engine rebuilds itself on the next start.

---

## 📡 1. Raspberry Pi (Hardware Interface)
Run these on the **Raspberry Pi over SSH** — NOT on the laptop.

**Terminal 1 (Start the LiDAR):**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch visionnav pi_sensors.launch.py        # also starts the push buttons (see below); buttons:=false to skip
```

**Terminal 2 (Start the Camera):**
*(Pull the repo and build the workspace on the Pi first (step 0). It stays quiet while the camera is healthy
and warns only if the camera drops below 10 fps (`CAMERA_SLOW_FPS`); add `--ros-args --log-level debug` to
see the rate every 5 s. A low "camera" rate means the webcam is slow, often from auto-exposure in dim light.
If "published" is high but the laptop gets fewer frames, it's Wi-Fi loss; try `CAMERA_JPEG_QUALITY=70`.)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run visionnav phone_camera_publisher
```

---

## 💻 2. Laptop (AI & SLAM Brain)
Run these **locally on your laptop** (MSI Sword 15).

**Terminal 3 (Start Localization - Choose ONE):**
*(Note: `LIBGL_ALWAYS_SOFTWARE=1` is required to fix the RViz Map OpenGL bug)*

*Option A: Indoor SLAM (Cartographer — default)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export LIBGL_ALWAYS_SOFTWARE=1
cd ~/wearable_ws
source install/setup.bash
ros2 launch visionnav laptop_brain.launch.py
# Pass your measured rig geometry, e.g.:
# ros2 launch visionnav laptop_brain.launch.py camera_height:=1.32 camera_pitch_deg:=12 lidar_height:=1.18
```
This starts the sensor TFs (`sensor_tf.launch.py`), the body filter (`/scan` → `/scan_filtered`,
removes your own torso/arms from the LiDAR), Cartographer and RViz.
It also starts the `wall_structure_mapper` node, which turns the walls in the SLAM map into 3D blocks in RViz
(**Walls & Structure** display, `/structure_markers`): segments 0.6 m or longer become 2.4 m walls, and
shorter pieces become 1.3 m obstacle blocks. A detected **door** is cut out of the wall (and made passable
for Nav2), so routes can go through it; stairs and holes in the floor are kept out of routes with a wide margin. Walls grow longer as you walk around and the LiDAR sees more.

**Remembering the home (saved maps).** The first time, the brain *maps*: walk through every room and
finish somewhere you have already been, then say or type **`save map`** in the navigation assistant
(Terminal 5). That saves the map, the objects seen reliably, and your named places in `~/.visionnav/maps/`
(`home.pbstream`, `home_objects.json`, `home_places.json`). From then on the same command starts in
**localization** mode: Cartographer loads the saved map and finds you in it (walk a few metres after
starting), the remembered objects are on the map at once ("go to light switch" works before the camera has
seen it again), and the map no longer grows or drifts. Options: `map:=office` for another building,
`localize:=false` to map again from scratch. (The last minute of a mapping walk is not yet usable for
finding you, which is why the walk should end somewhere already covered.)

*Option B: SLAM Toolbox (legacy)* — `ros2 launch visionnav laptop_brain.launch.py slam:=slam_toolbox`.
SLAM Toolbox only adds a scan after wheel odometry reports motion; the wearable has none, so the map
freezes while you walk. Use Cartographer on the wearable.

*Option C: GPS + LiDAR Fusion (Outdoor Mode)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
ros2 launch visionnav gps_localization.launch.py
```

**Terminal 4 (Start the Vision AI):**
*(Note: `WEARABLE_CAMERA_MODE=ros` forces the AI to listen to the Pi's Wi-Fi camera stream.
The first start after the vocabulary changes builds the TensorRT engine (about 2 minutes): the camera window
opens only when it is done, so leave the terminal open. The log must say `YOLO device: CUDA fp16 (TensorRT)`;
`YOLO device: CPU` means the NVIDIA driver is not loaded, e.g. after a kernel update — install the matching
`linux-modules-nvidia-595-open-$(uname -r)` package and reboot.)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export WEARABLE_CAMERA_MODE=ros
cd ~/wearable_ws
source install/setup.bash
# For standard indoor mode:
ros2 run visionnav object_perception

# For outdoor mode (longer tracking timeouts):
WEARABLE_MODE=outdoor ros2 run visionnav object_perception
```

**Terminal 5 (Start the Navigation Assistant):**
*(Use this terminal to type commands like `find chair` and `go to chair_1`)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run visionnav voice_navigation_assistant
# Voice/keyboard assistant. "go to chair" is routed by Nav2 (Theta* + smoother, started by
# laptop_brain.launch.py) with spoken turn-by-turn guidance; falls back to the built-in A* if
# Nav2 is not running (force with WEARABLE_NAV_BACKEND=astar).
# Any class the vision node detects can be a goal, e.g. "go to light switch", "go to door".
# Describe objects the way you know them — IDs like table_2 are never needed or spoken, and neither are
# colours (all distances are in feet):
#   "find the table where the cup is"   "go to the chair next to the door"   "go to the chair in the kitchen"
#   "where is the cup on the table"   "go to the nearest chair"   "what is on the table"   "what is around me"
# Answers say what tells the object apart and where it is from you ("The table, with the cup on it,
# 7 feet away, at 1 o'clock"). Several matches: "go to the chair" goes to the nearest ("There are 3 chairs.
# Taking you to the nearest one ..."); say "another one" for the next, or "list them".
# Name things yourself once you are at them: "call this my chair" — then "go to my chair" works in every
# session (saved per map in ~/.visionnav/maps/<map>_names.json; "forget name my chair" to remove it).
# "in the kitchen" uses your saved places. A helper may still say a colour ("the red cup"); set
# WEARABLE_SPEAK_COLORS=1 for a partially sighted user to also hear colours.
# Saved maps and places: "save map", "save this place as kitchen" (or "mark kitchen"),
# "go to kitchen", "where am i", "forget place kitchen".
# Grasp mode: "grasp cup" (also "grab", "pick up", "reach for"), and automatically on arrival at an
# object: the camera locks onto the object, tracks your hand, and speaks "Right 4 inches",
# "Lower 2 inches", "Forward 6 inches" ... until "Stop. The cup is at your hand."


# (If Outdoors) Start the GPS Macro-Navigator in background
# ros2 run visionnav gps_voice_navigator &
```

**Terminal 6 (Start the Qwen3-VL Scene Describer):**
*(Type any question — "what colour is the door?", "is there a light switch?", "can I walk straight
ahead?" — or just press Enter for a description. Answers take ~0.3–2.5 s and are spoken aloud.)*
Qwen3-VL 2B instruct is the system's only VLM. One-time: `ollama pull qwen3-vl:2b-instruct`
(the plain `qwen3-vl:2b` tag is the *thinking* variant that gave empty answers; it and moondream were removed).
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run visionnav scene_describer
```

---

## 🔘 Push Buttons on the Raspberry Pi 5

Four buttons let the wearer use everything without a keyboard. `pi_button_panel` (on the Pi, started by
`pi_sensors.launch.py`) reads them and publishes `/button_event`; the navigation assistant (Terminal 5, laptop)
does the work and speaks the result, and the scene describer (Terminal 6) answers camera questions.

| Button | GPIO (BCM) | Header pin | Tap | Hold |
|--------|-----------|------------|-----|------|
| **LOOK** | GPIO17 | pin 11 | Describe what is in front of me (Qwen3-VL) | Ask the camera a question: beep, speak while holding, release |
| **MODE** | GPIO27 | pin 13 | Switch indoor ↔ outdoor (spoken) | Status: mode, map, how many objects, what is around |
| **HAND** | GPIO22 | pin 15 | Guide my hand to the object found last (or the nearest one ahead); walks there first if it is more than 1 m away. Tap again to stop | — |
| **TALK** | GPIO23 | pin 16 | **STOP** everything (speech, walking guidance, hand guidance) | Voice command: beep, speak while holding, release ("find the table with the cup", "go to my chair", "call this my chair") |
| GND (shared) | — | pin 14 (also 9, 20, 25) | | |

Double-tap **TALK**: "what is around me". A hold is 0.6 s; a double tap is two taps within 0.4 s.

**Parts:** 4 momentary, normally-open push buttons (12 mm tactile or 16-19 mm panel buttons; give each a
different shape or 1-4 raised dots so they can be told apart by touch), 5 female-to-female jumper wires (or
female Dupont wires soldered to the buttons), heat-shrink. No resistors: the Pi's internal pull-ups are used.

**Wiring** (Pi switched off). Every button has two sides: one goes to its GPIO pin, the other to GND.
```
 Pi 5 header (USB ports pointing down, pin 1 top-left)      Buttons
   pin 11  GPIO17 ─────────────────────────────── LOOK ──┐
   pin 13  GPIO27 ─────────────────────────────── MODE ──┤
   pin 15  GPIO22 ─────────────────────────────── HAND ──┤
   pin 16  GPIO23 ─────────────────────────────── TALK ──┤
   pin 14  GND    ───────────────────────────────────────┘ (one wire, daisy-chained to the 2nd leg of all four)
```
* 4-leg tactile buttons: the two legs on each **long** side are joined inside. Use two **diagonally opposite**
  legs — they are always on different sides of the switch.
* Never connect a button to 5 V (pins 2, 4) or 3.3 V (pins 1, 17): the GPIO pins take 3.3 V at most, and a
  button to a power pin would short it when pressed.
* The pins avoid I2C (GPIO2/3), UART (GPIO14/15) and SPI, so they stay free for other hardware.
* Other pins: `ros2 launch visionnav pi_sensors.launch.py` uses the defaults; to change them run the node alone,
  e.g. `ros2 run visionnav pi_button_panel --ros-args -p look_pin:=5 -p talk_pin:=6`.

**Software on the Pi** (once): gpiozero with the lgpio backend (`RPi.GPIO` does not work on the Pi 5).
```bash
# Raspberry Pi OS: already installed.  Ubuntu 24.04:
sudo apt install python3-gpiozero python3-lgpio
ls -l /dev/gpiochip*        # your user needs read/write access; if it is root-only:
sudo groupadd -f gpio && sudo usermod -aG gpio $USER
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules
sudo udevadm control --reload && sudo udevadm trigger     # then log out and back in
cd ~/wearable_ws && git pull && colcon build --symlink-install --packages-select visionnav
```

**Test** (Pi, Terminal 1 running): `ros2 topic echo /button_event` in another Pi terminal and press each
button — you should see `LOOK tap`, `TALK hold_start` / `hold_end`, etc. in the launch log and the echo. Without
the Pi, button events can be simulated from the laptop:
`ros2 topic pub --once /button_event std_msgs/msg/String "{data: '{\"button\": \"look\", \"event\": \"tap\"}'}"`.

**Voice (TALK / LOOK hold):** the laptop microphone records while the button is held, and Whisper (offline,
`tiny.en`, cached in `~/.cache/huggingface`) turns it into text in about 0.3-0.6 s. Wait for the beep, then
speak. Saying "stop" only stops; "exit" shuts the assistant down. For more accuracy in noise:
`WEARABLE_WHISPER_MODEL=base.en` (downloaded once, needs internet the first time).

---

## 📐 Calibrating the Chest Rig (do once, repeat if the mount changes)
All 3D accuracy depends on the sensor mount values in `sensor_tf.launch.py`. Both SLAM and the
vision node read them from TF, so there is one place to fix them.

0. **LiDAR direction (most important).** If the map moves the wrong way (walking backward shows as
   walking forward, or turning left shows as turning right), the LiDAR mounting is wrong. With the Pi
   LiDAR running, wear the rig and run `ros2 run visionnav lidar_orientation_calibrator`. Then follow
   the prompts: stand still, walk ~1 m forward, turn left ~90°. It prints the `lidar_yaw_deg` /
   `lidar_roll_deg` to use. The current default (188°, upright) was measured from camera depth plus
   your report that backward showed as forward. Confirm it with this walk. RViz's **Your Tracked Path**
   display shows where SLAM thinks you have been, so a wrong direction is easy to spot.
1. **Measure** the camera lens height, the LiDAR height and the camera's downward tilt, and pass
   them as launch arguments (see Option A). A camera tilted 10° that is configured as 0° puts a
   floor object 3 m away about 2.5 m too far.
2. **Check the LiDAR overlay:** with the vision node running, press **`l`** in the camera window.
   The dots are the LiDAR returns drawn where the TF says they are (red = near, blue = far).
   * The dots should sit on walls, door frames and people's torsos at about chest height.
   * Dots on the **wrong side** of the image (mirrored): use `lidar_roll_deg:=180` (LiDAR upside down).
   * The camera picture itself is mirrored: the Pi stream is flipped back by default in ROS mode;
     set `WEARABLE_CAMERA_FLIP=0` (or `=1` in direct USB mode) for a camera that is not mirrored.
   * Dots **rotated / shifted sideways**: re-run `lidar_orientation_calibrator` (step 0).
   * Dots consistently **too high or low**: fix `camera_pitch_deg` / heights.
3. Every label shows the distance from you, how it was measured (`LiDAR`, `depth` or `cam`), the
   object's real height, and the surface height for objects on a table. RViz labels show the same,
   with the distance updated live as you walk.
4. The indoor map is a **persistent global object map** for the whole session by default. An object
   seen reliably (15+ sightings over 1 s or more, detected in at least 60 % of the frames the camera was
   looking at it, with a mean score of 0.40 or more, measured within 6 m, and — while metric depth runs —
   ranged by LiDAR or depth at least 5 times) stays on the map for as long as the session runs,
   drawn translucent with `seen:Ns-ago` while out of view. When you look back at it, it is matched to
   the same object (same name and ID), not duplicated — walking backward or turning also works, as
   long as the LiDAR direction is calibrated (see **Calibrating the Chest Rig** above). Set
   `WEARABLE_MEMORY_S=8` (seconds) to go back to the old real-time-only behaviour instead.
   Walking around the room, a remembered object is **not** removed just because it is not detected from
   a new angle (the back of a chair). When it is **taken away**, it disappears from the map within about
   1-2 s once the camera looks at its spot from a direction it was seen from before, within 4 m and with
   nothing (and nobody) in front of it: after 0.7 s if the depth shows the background behind its spot,
   otherwise after 2 s of not being detected there (longer for objects whose detection normally flickers,
   such as an open doorway). Log: `Removed <object> from the map: ...`. If SLAM or the depth estimate shifts it (e.g. a loop closure),
   the object seen next to its old spot is merged into it and keeps its ID (log: `re-found … keeping its ID`).
   Objects are also tracked in the image: a detection whose box overlaps the box an object had a moment ago
   is that object even if its distance estimate jumped, so a noisy distance never creates a second copy.
   Doors, windows, switches and other things in a wall are drawn in RViz as thin panels along the wall
   (direction fitted to the LiDAR), not as blocks. The detector also has "negative prompts" (`NEGATIVE_PROMPTS`:
   floor, door threshold, …) that absorb look-alike false detections such as a step at a doorway threshold.
   Misdetections are filtered by their **detection rate**: an object must be detected in at least half
   the frames in which the camera looks at its spot before it is shown at all, so a flickering
   hallucination never appears in the camera view or on the map. The map holds at most 200 remembered
   objects; past that, the ones not seen for the longest are dropped first.
   Each object has its own colour, shared by its 3D shape, its label and the line joining them. People
   and other moving objects are shown only while detected. They are tracked frame to frame by their image box
   (two people side by side keep their own IDs), ranged by the LiDAR on their torso, and — when the LiDAR
   misses them — by metric depth corrected with that person's own LiDAR/depth ratio. Speed ("MOVING 0.8m/s")
   is reported after half a second of tracking. The camera window shows the frame the boxes were computed on,
   so boxes stay on moving people. The marker array is also published on
   `/vision_markers` (identical to `/semantic_markers`) for other tooling.

**Metric depth:** the vision node also runs Depth Anything V2 (indoor, weights in
`models/depth_anything_v2_metric_indoor_vits.pth`). It gives a real distance to every object, including
ones below the LiDAR plane (chairs, tables, desk items). Its scale is corrected against the LiDAR every
frame; the log prints `Depth calibrated by LiDAR: scale …` every 10 s. Disable with `WEARABLE_MONO_DEPTH=0`,
and trade accuracy for speed with `WEARABLE_DEPTH_SIZE` (default 392, must be a multiple of 14).

**Wrong object distance?** Start the vision node with `WEARABLE_LIDAR_SNAP_DEBUG=1`. Once a second per
object class it logs whether the LiDAR anchored the object (`reason: snap` / `row`) or why not (for
example `snap_too_few_pts`: no LiDAR return near the camera's estimate in that object's columns).

Optional: for a calibrated camera, set `WEARABLE_CAMERA_FX/FY/CX/CY` (pixels) instead of relying on
`WEARABLE_CAMERA_HFOV_DEG` (default 70°).

---

## 🔧 Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `cannot access '/dev/ttyUSB0'` | LiDAR not connected or wrong machine | Run `pi_sensors.launch.py` on the **Pi**, not the laptop |
| `error code: 80008004` | LiDAR serial port unavailable | Check USB cable and run `sudo chmod 666 /dev/ttyUSB0` on the Pi |
| `numpy.core.multiarray failed` | A pip install pulled NumPy 2 into `~/.local` (ROS Jazzy, cv_bridge and matplotlib need the system NumPy 1.26) | `python3 -m pip uninstall --break-system-packages numpy scipy` (removes only the `~/.local` copies) |
| `Package 'wearable_sim' not found` | Old package name | The package is now `visionnav`: rebuild as in step 0 and use `ros2 run visionnav <node>` (no `.py`) |
| Scene describer gives empty or cut-off answers | A *thinking* model (`qwen3-vl:2b`) spends its tokens on hidden reasoning | The describer uses only `qwen3-vl:2b-instruct` (`ollama pull qwen3-vl:2b-instruct`) |
| Bounding boxes too large on map | Depth estimation overshoot | Already fixed — per-category size clamping applied |
| Markers in the wrong place / on the wrong side | Sensor mount TF does not match the rig | Follow **Calibrating the Chest Rig** above (press `l` for the LiDAR overlay) |
| Map smears or stops updating while walking | SLAM Toolbox without odometry, or body hits in the scan | Use the default Cartographer backend (body filter is included) |
