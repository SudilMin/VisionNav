# VisionNav Startup Instructions

This file contains the complete, up-to-date sequence of commands needed to launch the entire VisionNav system across both the Raspberry Pi and the Laptop.

---

## 📡 1. Raspberry Pi (Hardware Interface)
Run these on the Raspberry Pi over SSH.

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
Run these locally on your laptop.

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
cd ~/wearable_ws
source install/setup.bash
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

**Terminal 6 (Start the Moondream VLM):**
*(Press Enter in this terminal at any time to get a detailed audio description of the scene)*
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
cd ~/wearable_ws
source install/setup.bash
ros2 run wearable_sim scene_describer.py
```
