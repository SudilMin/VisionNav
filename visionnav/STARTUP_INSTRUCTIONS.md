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
ros2 launch visionnav pi_sensors.launch.py
```

**Terminal 2 (Start the Camera):**
*(Pull the repo and build the workspace on the Pi first (step 0). It logs the real camera and publish rate
every 5 s. If "camera" is low (e.g. 8 fps), the webcam is slow, often from auto-exposure in dim light.
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
*(Note: `WEARABLE_CAMERA_MODE=ros` forces the AI to listen to the Pi's Wi-Fi camera stream)*
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


# (If Outdoors) Start the GPS Macro-Navigator in background
# ros2 run visionnav gps_voice_navigator &
```

**Terminal 6 (Start the Qwen3-VL Scene Describer):**
*(Type a question and press Enter to get an audio description of what the camera sees)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run visionnav scene_describer
```

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
   seen reliably (12+ sightings over 2 s or more) stays on the map for as long as the session runs,
   drawn translucent with `seen:Ns-ago` while out of view. When you look back at it, it is matched to
   the same object (same name and ID), not duplicated — walking backward or turning also works, as
   long as the LiDAR direction is calibrated (see **Calibrating the Chest Rig** above). Set
   `WEARABLE_MEMORY_S=8` (seconds) to go back to the old real-time-only behaviour instead.
   Misdetections never become reliable and disappear within about 1 s regardless. A reliable, remembered
   object is removed only when the camera looks straight at its spot — with nothing closer in the way —
   and doesn't see it there for 3 s; while something else is in front of it, it is left alone. The map
   holds at most 200 remembered objects; past that, the ones not seen for the longest are dropped first.
   Each object has its own colour, shared by its 3D shape, its label and the line joining them. People
   and other moving objects are shown only while detected. The marker array is also published on
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
| Qwen-VL returns empty answers | Thinking mode consuming all tokens | Already fixed — `think: False` is now set |
| Bounding boxes too large on map | Depth estimation overshoot | Already fixed — per-category size clamping applied |
| Markers in the wrong place / on the wrong side | Sensor mount TF does not match the rig | Follow **Calibrating the Chest Rig** above (press `l` for the LiDAR overlay) |
| Map smears or stops updating while walking | SLAM Toolbox without odometry, or body hits in the scan | Use the default Cartographer backend (body filter is included) |
