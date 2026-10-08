# VisionNav Architecture

VisionNav is a chest-worn navigation aid for blind users. It speaks: where objects are, how to walk to them, and
what is in the way. It never shows anything the user must see. Everything runs offline on two computers that
talk over Wi-Fi through ROS 2 (`ROS_DOMAIN_ID=42`). How to set it up and use it: [STARTUP_INSTRUCTIONS.md](STARTUP_INSTRUCTIONS.md).

## 1. Design goals

* **Remember, don't just react.** Indoors the camera's objects are placed on a LiDAR SLAM map, so the system still
  knows where the chair is after the wearer has turned away from it.
* **Say little.** One short sentence at a time, about the most urgent thing only; silence otherwise. Distances in
  feet, directions as clock positions ("Chair at 2 o'clock, 6 feet").
* **Hands and eyes free.** Five tactile push buttons and push-to-talk; no screen, no keyboard.
* **Per-session indoor maps.** Every indoor session maps the place afresh; the map, its objects, named places and
  object names are forgotten when the session ends (MODE hold). Only remembered faces persist.

## 2. Hardware

| Part | Where | Job |
|---|---|---|
| Raspberry Pi 5 | chest rig | Reads the 5 GPIO buttons; streams the sensors to the laptop |
| RPLiDAR C1 (`/scan`, 10 Hz) | chest, ~1.2 m | Walls and obstacles at chest height; SLAM; ranges for the camera's objects |
| USB camera (`/camera/image_raw/compressed`) | chest, ~1.3 m | Object detection, metric depth, faces, vision AI |
| MPU-6050 IMU (`/imu/data`, 100 Hz, optional) | chest plate | Turn rate and gravity for SLAM and outdoor odometry |
| Laptop (RTX 2050, 4 GB) | backpack | All AI, mapping, planning and speech |

## 3. Software on each computer

All nodes are in the `visionnav` package (`visionnav/visionnav/*.py`); `sllidar_ros2` is the LiDAR driver.

**Raspberry Pi**

| Node | Role |
|---|---|
| `pi_button_panel` | Boot service (`visionnav-buttons`). Reads the buttons, publishes `/button_event`; SENSORS starts/stops `pi_sensors.launch.py` |
| `phone_camera_publisher` | Camera → JPEG on `/camera/image_raw/compressed` |
| `mpu6050_imu` | IMU → `/imu/data` (`calibrate` measures its mount) |
| `sllidar_node` | LiDAR → `/scan` |

**Laptop**

| Node | Role |
|---|---|
| `voice_navigation_assistant` | The main program: button actions, push-to-talk (Whisper), speech (Piper), object requests (`object_language.py`), turn-by-turn guidance, face memory (`face_memory.py`), sound-alike repair (`speech_fix.py`). Starts and stops everything below through `system_manager.py` |
| `object_perception` | Camera AI: YOLOE open-vocabulary detection (TensorRT), Depth Anything V2 metric depth calibrated by the LiDAR, Kalman tracking, the session's object map (`/semantic_objects`, `/semantic_markers`), hand guidance (`grasp_tracker.py`); outdoor hazards via `outdoor_awareness.py` and `lidar_odometry.py` |
| `scene_describer` | Vision AI: Qwen3-VL 2B (Ollama) answers questions about the camera picture (`/describe_command` → `/scene_description`) |
| `lidar_body_filter` | Removes the wearer's body from `/scan` → `/scan_filtered` |
| Cartographer | SLAM on `/scan_filtered` (+ IMU): `/map` and the `map → odom` transform |
| `map_manager` | Named places of the session (`/map_command`, `/named_places`, `/active_map`) |
| `wall_structure_mapper` | 3-D walls for RViz from `/map` |
| `semantic_costmap` | `/map` + objects (+ people's predicted paths) → `/semantic_map` for Nav2 |
| Nav2 planner + smoother | Theta* path and smoothing (no controller: a person follows voice, not `cmd_vel`) |
| `semantic_navigator` | `/semantic_goal` ("chair") → an approach point in front of it → `/object_path` |
| `lidar_orientation_calibrator` | One-off tool: measures the LiDAR's mount on the rig |
| `web_dashboard` | Web page (http://localhost:8080): starts / stops the assistant (with no desktop windows: `WEARABLE_WINDOWS=0`), shows in 3D what RViz showed, from the same topics (`web/scene3d.js`, three.js served from the package), and the live LiDAR scan, the camera AI's picture (both only while the camera AI runs), what is said and heard, every press of the rig's buttons (`/button_event`), the laptop's free memory |

Sensor geometry (camera, LiDAR and IMU mounts) is defined once in `launch/sensor_tf.launch.py` and read from TF by
every node.

## 4. Modes

**Indoor** (`laptop_brain.launch.py` + map window, unless the web dashboard shows the map, + `object_perception`). Objects seen reliably are placed on the
SLAM map and kept for the session, even out of view; one taken away is removed once the camera looks at its
spot again. "Go to the chair next to the door": `object_language` resolves the description to one object, the
navigator plans a route, and the assistant speaks clock-direction guidance along it until arrival. In hand
mode (HAND tap), holding HAND and saying an object ("cup", "screwdriver") finds it in the camera picture and
guides the hand to it ("Left 2 inches… Forward 4 inches… Stop. The cup is at your hand.").

**Outdoor** (`outdoor_sensors.launch.py` + `object_perception` in outdoor mode). Nothing is mapped. The camera,
depth and the forward half of the LiDAR scan feed a live hazard model: walking corridor, holes and kerbs, head-height
branches, zebra crossings, light colours, and vehicles with their time to collision (LiDAR odometry removes the
wearer's own motion). `AlertPolicy` picks the single most urgent sentence (`/outdoor_alert`). Nothing behind the
wearer is tracked or said.

## 5. Buttons (Pi GPIO, wiring in STARTUP_INSTRUCTIONS.md section 3)

| Button | Tap | Hold | Double tap |
|---|---|---|---|
| SENSORS | camera, LiDAR, IMU on | off | — |
| LOOK | describe the scene (vision AI) | ask a question / say a command | vision AI off |
| MODE | indoor ↔ outdoor | close the map and camera, forget the session | — |
| HAND | hand mode on / off | say an object, the hand is guided to it | face mode on; in face mode: face and hand mode off |
| TALK | what is around me | say where to go (indoor) | stop the navigation |

## 6. Main topics

| Topic | From → to |
|---|---|
| `/button_event`, `/pi_sensors_state` | Pi button panel → assistant |
| `/scan`, `/scan_filtered`, `/imu/data`, `/camera/image_raw/compressed` | Pi sensors → laptop |
| `/semantic_objects` (JSON) | perception → costmap, navigator, wall mapper, assistant |
| `/semantic_goal`, `/object_path`, `/semantic_nav_status` | assistant ↔ navigator |
| `/perception_mode`, `/perception_mode_state` | assistant ↔ perception |
| `/outdoor_alert`, `/outdoor_scene` | perception → assistant |
| `/grasp_command`, `/grasp_offset`, `/perception_classes` | assistant ↔ perception (hand mode) |
| `/describe_command`, `/scene_description` | assistant ↔ vision AI |
| `/voice_log`, `/voice_command` | assistant → dashboard (what is said and heard, JSON) / dashboard → assistant (typed commands) |
| `/perception/view/compressed` | perception → dashboard (the camera window's picture, JPEG, only while subscribed) |
| `/map`, `/scan(_filtered)`, `/semantic_markers`, `/structure_markers`, `/trajectory_node_list`, `/outdoor_markers`, `/outdoor_occupancy`, `/object_path`, TF | → RViz or the dashboard's 3D view (subscribed only while a page shows it) |
