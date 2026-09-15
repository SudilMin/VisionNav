# VisionNav Startup Instructions

This file contains the complete, up-to-date sequence of commands needed to launch the entire VisionNav system across both the Raspberry Pi and the Laptop.

> **⚠️ IMPORTANT:** The Pi commands (Terminals 1-2) must be run on the **Raspberry Pi over SSH**.
> The Laptop commands (Terminals 3-6) must be run **locally on your laptop**.
> Do NOT run `pi_sensors.launch.py` on the laptop — there is no LiDAR connected to it!

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
ros2 launch wearable_sim pi_sensors.launch.py
```

**Terminal 2 (Start the Camera):**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run wearable_sim phone_camera.py
```

---

## 💻 2. Laptop (AI & SLAM Brain)
Run these **locally on your laptop** (MSI Sword 15).

**Terminal 3 (Start SLAM Mapping & RViz):**
*(Note: `LIBGL_ALWAYS_SOFTWARE=1` is required to fix the RViz Map OpenGL bug)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export LIBGL_ALWAYS_SOFTWARE=1
cd ~/wearable_ws
source install/setup.bash
ros2 launch wearable_sim laptop_brain.launch.py
```

**Terminal 4 (Start the Vision AI):**
*(Note: `WEARABLE_CAMERA_MODE=ros` forces the AI to listen to the Pi's Wi-Fi camera stream)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export WEARABLE_CAMERA_MODE=ros
cd ~/weale_ws
source install/setup.barabsh
ros2 run wearable_sim vision_perception.py
```

**Terminal 5 (Start the Voice Navigation Assistant):**
*(Use this terminal to type commands like `find chair` and `go to chair_1`)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run wearable_sim find_object.py
```

**Terminal 6 (Start the Qwen3-VL Scene Describer):**
*(Type a question and press Enter to get an audio description of what the camera sees)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run wearable_sim scene_describer.py
```

---

## 🔧 Troubleshooting

| Error | Cause | Fix |
|-------|-------|-----|
| `cannot access '/dev/ttyUSB0'` | LiDAR not connected or wrong machine | Run `pi_sensors.launch.py` on the **Pi**, not the laptop |
| `error code: 80008004` | LiDAR serial port unavailable | Check USB cable and run `sudo chmod 666 /dev/ttyUSB0` on the Pi |
| Qwen-VL returns empty answers | Thinking mode consuming all tokens | Already fixed — `think: False` is now set |
| Bounding boxes too large on map | Depth estimation overshoot | Already fixed — per-category size clamping applied |
| Markers move when tilting camera | Camera tilt changing depth estimate | Already fixed — position locking for established objects |

