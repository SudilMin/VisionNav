# VisionNav Installation Guide (fresh computer)

Everything to install and run on a **new laptop** and a **new Raspberry Pi 5** to get VisionNav working, from
nothing. When it is installed, see [visionnav/STARTUP_INSTRUCTIONS.md](visionnav/STARTUP_INSTRUCTIONS.md) for daily use.

**Before you start:** both computers need **Ubuntu 24.04** with **ROS 2 Jazzy** already installed (laptop:
`ros-jazzy-desktop`, Pi: `ros-jazzy-ros-base`). Installing ROS 2 itself is not covered here.

| | Laptop (the brain) | Raspberry Pi 5 (chest rig) |
|---|---|---|
| Runs | AI, map, navigation, voice, web dashboard | Buttons, LiDAR, camera, IMU |
| Needs | NVIDIA GPU (tested: RTX 2050, driver 595), ≥ 8 GB RAM, ~20 GB free disk, mic + speaker/earpiece | Ubuntu 24.04 for Pi, RPLiDAR C1, USB camera, MPU-6050, 5 buttons |
| Internet | Only during setup and the first start (downloads) | Only during setup |

> The workspace **must** be at `~/wearable_ws` (with the code in `~/wearable_ws/src`): the Pi script, the boot
> service and the autostart all use that path.

---

## Part A: Laptop

### A1. System packages

```bash
sudo apt update
sudo apt install -y git curl build-essential python3-pip python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-yaml python3-pyaudio portaudio19-dev alsa-utils \
    terminator earlyoom \
    ros-jazzy-cartographer-ros ros-jazzy-navigation2 ros-jazzy-rviz2 \
    ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-tf2-tools
```

* `alsa-utils` gives `aplay` (the voice, clicks and beeps). `python3-pyaudio` is the microphone.
* `earlyoom` closes programs before the laptop freezes when memory runs out. Turn it on:
  `sudo systemctl enable --now earlyoom`.

### A2. NVIDIA driver

The AI models need the GPU. PyTorch here is built for CUDA 13, which needs driver **580 or newer**.

```bash
sudo apt install -y nvidia-driver-595-open
sudo reboot
```
After the reboot, `nvidia-smi` must show your GPU. (If `nvidia-driver-595-open` is not offered, run
`ubuntu-drivers devices` and install the recommended `-open` driver, 580 or newer.)

### A3. Get the code

Clone it fresh (do not copy the old folder; its `build/`, `install/` and `log/` belong to the old computer).

```bash
mkdir -p ~/wearable_ws
git clone https://github.com/SudilMin/VisionNav.git ~/wearable_ws/src
```
If the repository is private, GitHub asks for your username and a **personal access token** as the password.

The LiDAR driver `sllidar_ros2` is **not** downloaded by that clone (the folder comes out empty). Get it
separately, at the same version the project uses:

```bash
cd ~/wearable_ws/src
rm -rf sllidar_ros2
git clone https://github.com/Slamtec/sllidar_ros2.git
git -C sllidar_ros2 checkout 34300099fadfc772965962dec837bf436706188f
```

### A4. Python packages

These go in your user folder (`~/.local`). **NumPy must stay at 1.26.4**: ROS 2 Jazzy breaks with NumPy 2
(`numpy.core.multiarray failed`). The constraint file below stops pip from upgrading it.

```bash
echo "numpy==1.26.4" > /tmp/visionnav-constraints.txt
PIP="python3 -m pip install --user --break-system-packages -c /tmp/visionnav-constraints.txt"

# 1. PyTorch for CUDA 13 (about 3 GB)
$PIP torch==2.14.0 torchvision==0.29.0 --index-url https://download.pytorch.org/whl/cu130

# 2. Object detector, speech, voice, vision-AI client, TensorRT
$PIP ultralytics==8.4.84 "git+https://github.com/ultralytics/CLIP.git" \
     faster-whisper==1.2.1 piper-tts==1.4.2 ollama==0.6.2 \
     onnx onnxslim onnxruntime-gpu==1.24.4 tensorrt-cu13==11.3.0.99

# 3. Hand tracking: MediaPipe WITHOUT its dependencies (they would pull NumPy 2)
python3 -m pip install --user --break-system-packages --no-deps mediapipe==1.0.1 absl-py
```

Check:
```bash
python3 -c "import numpy, torch; print('numpy', numpy.__version__, '| CUDA', torch.cuda.is_available())"
```
It must print `numpy 1.26.4 | CUDA True`. If NumPy shows 2.x:
`python3 -m pip uninstall --break-system-packages numpy` (removes only the `~/.local` copy) and check again.

### A5. Model files

Some models come with the code (the voice `en_US-lessac-medium.onnx` and the two face models). The big ones are
not in git and must be downloaded once into `~/wearable_ws/src/visionnav/models/`:

```bash
cd ~/wearable_ws/src/visionnav/models

# Object detector (YOLOE) and its text encoder
curl -L -o yoloe-11s-seg.pt     https://github.com/ultralytics/assets/releases/download/v8.3.0/yoloe-11s-seg.pt
curl -L -o mobileclip_blt.ts    https://github.com/ultralytics/assets/releases/download/v8.3.0/mobileclip_blt.ts

# Depth: indoor and outdoor
curl -L -o depth_anything_v2_metric_indoor_vits.pth \
  https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small/resolve/main/depth_anything_v2_metric_hypersim_vits.pth
curl -L -o depth_anything_v2_metric_outdoor_vits.pth \
  https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small/resolve/main/depth_anything_v2_metric_vkitti_vits.pth

# Hand tracking
curl -L -o hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task

ls -lh
```

You should now have: `yoloe-11s-seg.pt` (~28 MB), `mobileclip_blt.ts` (~600 MB), the two
`depth_anything_v2_metric_*_vits.pth` (~99 MB each), `hand_landmarker.task` (~8 MB), plus the files from git.

**Shortcut:** if the old laptop is still around, copy its whole models folder instead (USB stick, or over the
network): `rsync -av <old-user>@<old-laptop>:~/wearable_ws/src/visionnav/models/ ~/wearable_ws/src/visionnav/models/ --exclude '*.engine'`.
Do **not** copy the `*.engine` files: they are built for one GPU and are rebuilt by themselves (A8).

### A6. Ollama and the vision AI model

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen3-vl:2b-instruct      # ~1.9 GB
ollama list                           # must show qwen3-vl:2b-instruct
```
Use exactly `qwen3-vl:2b-instruct`: the plain `qwen3-vl:2b` gives empty answers.

### A7. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/wearable_ws
colcon build --symlink-install
```
It ends with `Summary: 2 packages finished` (`sllidar_ros2` and `visionnav`).

Make every new terminal ready by itself:
```bash
cat >> ~/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
source ~/wearable_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
EOF
source ~/.bashrc
```
`ROS_DOMAIN_ID=42` is how the laptop and the Pi find each other; both must use it.

### A8. First start (internet on)

```bash
ros2 run visionnav web_dashboard
```
Open **http://localhost:8080** and press **▶ START**. The first start does some one-time work:

* Downloads the speech model (Whisper `small.en`, ~0.5 GB) into `~/.cache/huggingface`.
* When indoor mode first starts, builds the detector's TensorRT engine (**~3–6 min**, the camera picture appears
  when done). Outdoor mode builds its own engine the first time it starts. Later starts take seconds.

Check that the GPU is used: `grep "YOLO device" ~/.visionnav/logs/perception.log` must say
`CUDA fp16 (TensorRT)`. `CPU` means the NVIDIA driver is not loaded (A2).

Check the sound: `speaker-test -t wav -c 2 -l 1` (speaker) and `arecord -d 3 /tmp/t.wav && aplay /tmp/t.wav`
(microphone). Pick the earpiece as the default input/output in Settings → Sound.

### A9. Optional laptop settings

* **Start the assistant at login**, **lid closed**, **never sleep**, **hotspot for the Pi**: see section 1 of
  [STARTUP_INSTRUCTIONS.md](visionnav/STARTUP_INSTRUCTIONS.md).
* **Firewall:** if `sudo ufw status` says `active`, the Pi cannot reach the laptop. Allow your Wi-Fi network,
  e.g. `sudo ufw allow from 192.168.0.0/16`, or `sudo ufw disable`.
* **Remembered faces:** they live in `~/.visionnav/faces/faces.json`. Copy that folder from the old laptop to keep
  them.

---

## Part B: Raspberry Pi 5

### B1. System packages

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-opencv python3-numpy ros-jazzy-cv-bridge
```
(The GPIO and I2C libraries are installed by the setup script in B3.)

### B2. Get the code and build it

The same as on the laptop:

```bash
mkdir -p ~/wearable_ws
git clone https://github.com/SudilMin/VisionNav.git ~/wearable_ws/src
cd ~/wearable_ws/src
rm -rf sllidar_ros2
git clone https://github.com/Slamtec/sllidar_ros2.git
git -C sllidar_ros2 checkout 34300099fadfc772965962dec837bf436706188f

source /opt/ros/jazzy/setup.bash
cd ~/wearable_ws
colcon build --symlink-install       # builds the LiDAR driver too (a few minutes on the Pi)
```

The Pi does not need the model files, PyTorch or Ollama.

### B3. Buttons, IMU, permissions and boot service

```bash
bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh
```
It installs the GPIO and I2C libraries, turns on I2C, gives your user access to the LiDAR port, GPIO and I2C,
and installs the `visionnav-buttons` service that starts at every boot. It ends with
`OK: the buttons are ready`. The first time it asks for a reboot:

```bash
sudo reboot
```

### B4. Wire and check the hardware

Wiring diagrams: sections 3 (buttons) and 3b (IMU) of
[STARTUP_INSTRUCTIONS.md](visionnav/STARTUP_INSTRUCTIONS.md). Then:

```bash
bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh test   # press each button, its name is printed
bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh imu    # IMU found (68) + its mount
ls -l /dev/ttyUSB0                                          # the LiDAR is plugged in
ls /dev/video*                                              # the camera is plugged in
```

### B5. Same Wi-Fi as the laptop

```bash
sudo nmcli dev wifi connect "<network name>" password "<password>"
```
Use the same network as the laptop (the laptop's or a phone's hotspot works everywhere). The boot service already
sets `ROS_DOMAIN_ID=42`.

---

## Part C: Check that everything works together

1. Switch on the Pi, wait ~30 s.
2. On the laptop: `ros2 topic echo /button_event`, press a button on the rig. A line like
   `{"button": "look", "event": "tap"}` appears → the two computers talk. Ctrl+C.
3. Start the assistant (`ros2 run visionnav voice_navigation_assistant`, or the web dashboard). It says
   "VisionNav is on. Press the sensor button to start."
4. Press **SENSORS** → "Sensors turned on." → "Indoor mode activated."; the map and camera picture appear.
5. Tap **LOOK** → the vision AI starts and describes the scene.

Next: calibrate the rig for the new mount (section 5 of
[STARTUP_INSTRUCTIONS.md](visionnav/STARTUP_INSTRUCTIONS.md)).

---

## Install problems

| Problem | Fix |
|---|---|
| `numpy.core.multiarray failed` / `_ARRAY_API not found` | A pip install pulled NumPy 2: `python3 -m pip uninstall --break-system-packages numpy` (A4) |
| `torch.cuda.is_available()` is `False` | No NVIDIA driver, or older than 580: A2, then reboot |
| `YOLO device: CPU` in `perception.log` | Same as above; after a kernel update: `sudo apt install linux-modules-nvidia-595-open-$(uname -r)` and reboot |
| The camera picture never appears the first time | The TensorRT engine is still being built (3–6 min); watch `~/.visionnav/logs/perception.log` |
| `FileNotFoundError` for a `.pth`, `.pt`, `.ts` or `.task` file | That model is missing from `visionnav/models/` (A5) |
| "The vision AI could not start" | `ollama list` must show `qwen3-vl:2b-instruct` (A6); `systemctl status ollama` |
| No speech is heard / no microphone | `aplay` missing (`sudo apt install alsa-utils`); pick the right device in Settings → Sound |
| Speech recognition fails at the first start | The Whisper model downloads at the first start: needs internet once (A8) |
| `colcon build` fails in `sllidar_ros2` with no `CMakeLists.txt` | The `sllidar_ros2` folder is empty: clone it (A3) |
| "Waiting for the Pi…" forever | Different Wi-Fi, different `ROS_DOMAIN_ID`, or the laptop's firewall (A9) |
| Buttons do nothing on the Pi | `systemctl status visionnav-buttons`, `journalctl -u visionnav-buttons -f`; run `setup_pi.sh` again |
