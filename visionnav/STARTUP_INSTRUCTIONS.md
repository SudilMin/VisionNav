# VisionNav Startup Guide

VisionNav runs on two computers:

* **Raspberry Pi 5** (on the chest rig): the five push buttons, the LiDAR and the camera. The button program
  starts by itself when the Pi boots; the **SENSORS** button turns the LiDAR and camera on.
* **Laptop** (MSI Sword 15, RTX 2050): the AI, the map and the voice. The navigation assistant starts at login
  (or with one command) and starts everything else — the map, the camera AI, the vision AI, GPS — as soon as the
  camera and LiDAR are on, and whenever the buttons ask for it.

**No SSH is needed to use it.** The Pi and the laptop find each other by themselves over Wi-Fi through ROS 2
(both use `ROS_DOMAIN_ID=42` on the same network): the laptop hears the Pi's buttons and sensors directly.
`ssh pi@raspberrypi.local` (or the Pi's user/IP) is only for the one-time setup and for updates (section 2).

---

## 🚀 Everyday Use

1. **Switch on the Pi and the laptop.** The Pi's buttons are ready about 30 s after it boots; the assistant
   opens when you log in to the laptop (one-time setup: sections 1 and 2) and says
   **"VisionNav is on. Press the sensor button to start."** — or "Waiting for the Pi…" until the Pi is on the
   network ("Connected to the Pi." when it is).
   Without the login start, run it yourself:
   ```bash
   export ROS_DOMAIN_ID=42
   export ROS_LOCALHOST_ONLY=0
   cd ~/wearable_ws
   source install/setup.bash
   ros2 run visionnav voice_navigation_assistant
   ```
2. **Press SENSORS.** "Turning on the camera and LiDAR." → "Camera and LiDAR on." → the laptop starts indoor
   mode by itself: "Starting indoor mode." → **"Indoor mode activated."**
3. Use the buttons (below), or type commands in the assistant's terminal. Every tap clicks, so you know it was
   heard. Hold TALK and say **"help"** to hear what the buttons do.
4. **Press SENSORS again** when you are done: "Camera and LiDAR off." → "Indoor mode paused." (the map is saved
   first while mapping). The next SENSORS press continues where you left off.

### What each button does, and what you hear

| Button | Press | What happens and what you hear |
|---|---|---|
| **SENSORS** | tap | Turns the LiDAR and camera on — "Turning on the camera and LiDAR." → "Camera and LiDAR on." → the current mode starts ("Indoor mode activated.") — or off: "Camera and LiDAR off." → "Indoor mode paused." If they fail: "The camera and LiDAR could not start. Check their cables." |
| | hold | Restarts them (for a camera that stopped sending) |
| **LOOK** | tap | If the vision AI is off: "Turning on the vision AI. This takes about half a minute." → starts Qwen3-VL → "Vision AI enabled." → describes the scene. If it is already on: "Looking." → the description |
| | hold | Beep → ask a question while holding ("what colour is the door?") → release → the camera answers |
| | double tap | Stops the vision AI and frees its ~2 GB of GPU memory: "Vision AI off." |
| **MODE** | tap | While paused (sensors off): "Outdoor mode selected. It starts when the camera and LiDAR are on." Otherwise: "Switching to outdoor mode." → stops the indoor map programs, starts the GPS ones → "Outdoor mode activated." (and back: "Indoor mode activated."). Without a GPS receiver: "…No GPS receiver is plugged in, so I will only warn you about obstacles." |
| | hold | Status: mode, vision AI on/off, missing sensors, map, how many objects, what is around you |
| **HAND** | tap | Starts the camera AI if it is off → "Hand guidance enabled." → guides your hand to the object found last (or the nearest one ahead), walking you there first if it is more than 1 m away. Tap again: "Hand guidance disabled." |
| **TALK** | tap | **STOP** everything (speech, walking guidance, hand guidance): "Stopped." |
| | hold | Beep → speak a command while holding (see the list below) → release |
| | double tap | What is around you |

A hold is 0.6 s; a double tap is two taps within 0.4 s. Every tap clicks. If the camera or LiDAR stream stops
you hear "The camera signal is lost." ("The camera is on." when it returns), and if the Pi drops off the
network, "The Pi is not answering. Check that it is switched on and on the same Wi-Fi."

### Which programs each mode runs

The assistant starts and stops these itself (`system_manager.py`); the Pi's button program starts the sensors.

| Mode / button | Programs |
|---|---|
| **SENSORS** (Pi) | `pi_sensors.launch.py`: RPLiDAR C1 (`sllidar_node`) + chest camera (`phone_camera_publisher`) |
| **Indoor** (default, laptop) | `laptop_brain.launch.py` (sensor TFs, Cartographer SLAM, Nav2, walls, RViz) + `object_perception` (camera AI, camera window) |
| **Outdoor** (laptop) | `nmea_navsat_driver` (only if a GPS receiver is on `/dev/ttyACM0`) + `gps_localization.launch.py` + `gps_voice_navigator` + `object_perception` (collision warnings) |
| **LOOK** (laptop) | `scene_describer` (Qwen3-VL), started on the first press, stopped by a double tap |

* A program already started by hand in a terminal is used as it is — never started twice, and never stopped by
  the buttons.
* Everything the assistant started stops when it exits (Ctrl+C, closing its terminal, or saying "exit").
* Each program's output: `~/.visionnav/logs/<part>.log` (`brain`, `perception`, `vision_ai`, `gps`,
  `pi_sensors` on the Pi).
* Settings (set before starting the assistant): `WEARABLE_AUTOSTART` — `sensors` (default: start the mode when
  the camera and LiDAR come on, pause it when they go off), `now` (start at once), `off` (never, section 6);
  `WEARABLE_MODE=outdoor` starts in outdoor mode;
  `WEARABLE_BRAIN_ARGS="camera_height:=1.32 camera_pitch_deg:=12 lidar_height:=1.18"` passes the rig's measured
  geometry to the map (section 5).
* Switching from indoor to outdoor while *mapping* saves the map first.

### Voice commands (hold TALK, or type them in the assistant's terminal)

Objects are described the way you know them: IDs such as `table_2` and colours are never needed or spoken, and
distances are in feet ("The table, with the cup on it, 7 feet away, at 1 o'clock").

| Say | What it does |
|---|---|
| "find the table where the cup is", "where is the cup on the table" | Says where it is |
| "go to the chair next to the door", "take me to the nearest chair", "go there" | Walking guidance (Nav2 route, turn-by-turn) |
| "another one", "list them", "the second one", "the one in the kitchen" | When several objects match: "go to the chair" goes to the nearest ("There are 3 chairs. Taking you to the nearest one…") |
| "what is on the table", "what is around me" | Answered from the map |
| "call this my chair" → later "go to my chair"; "forget name my chair" | Your own names for objects (saved per map) |
| "save this place as kitchen" (or "mark kitchen"), "go to kitchen", "where am i", "forget place kitchen" | Named places |
| "save map" | Saves the map, the objects and the places (section 4) |
| "grasp the cup" ("grab", "pick up", "reach for") | Hand guidance: "Right 4 inches", "Lower 2 inches", "Forward 6 inches"… "Stop. The cup is at your hand." (also starts on arrival at an object) |
| "what colour is the door?", "describe …", "read …" | Sent to the vision AI |
| "help" | What the buttons do and what you can say |
| "stop" / "exit" | Stop everything / shut the assistant down |

Speech is recognised offline (Whisper `tiny.en`, cached in `~/.cache/huggingface`) in about 0.3–0.6 s: wait for
the beep, then speak. For more accuracy in noise: `WEARABLE_WHISPER_MODEL=base.en` (downloaded once).
Colours are understood when someone says one ("the red cup") but only spoken with `WEARABLE_SPEAK_COLORS=1`
(for a partially sighted user).

---

## 🔨 1. One-Time Setup: Laptop

```bash
cd ~/wearable_ws/src && git pull
source /opt/ros/jazzy/setup.bash
cd ~/wearable_ws
colcon build --symlink-install --packages-select visionnav
```

* **GPU driver.** The perception log must say `YOLO device: CUDA fp16 (TensorRT)`. `CPU` means the NVIDIA driver
  is not loaded (often after a kernel update): `sudo apt install linux-modules-nvidia-595-open-$(uname -r)` and
  reboot.
* **Object detector.** YOLOE-11s-seg with the vocabulary in `VOCABULARY` (top of `visionnav/object_perception.py`).
  The first start after the vocabulary changes builds a TensorRT engine (~2 min, one time); the camera window
  opens when it is done. To detect something new, add its name to `VOCABULARY`.
* **Vision AI.** `ollama pull qwen3-vl:2b-instruct` (the plain `qwen3-vl:2b` tag is a *thinking* model that
  gives empty answers).
* **Hand tracking** (grasp mode). Install MediaPipe *without* its dependencies (a normal install pulls NumPy 2
  and breaks ROS), then fetch the hand model:
  ```bash
  python3 -m pip install --user --break-system-packages --no-deps mediapipe==1.0.1 absl-py
  curl -sSfL -o ~/wearable_ws/src/visionnav/models/hand_landmarker.task \
    https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
  ```

**Start the assistant at login** (optional, so no command is needed at all): it opens in its own terminal window
when you log in.
```bash
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/visionnav.desktop <<'EOF'
[Desktop Entry]
Type=Application
Name=VisionNav
Exec=terminator -e "bash -ic 'export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; source ~/wearable_ws/install/setup.bash; ros2 run visionnav voice_navigation_assistant'"
X-GNOME-Autostart-enabled=true
EOF
```
To turn it off again: `rm ~/.config/autostart/visionnav.desktop`.

**For a wearable laptop in a backpack** (recommended):
* **Log in automatically:** Settings → Users → *Automatic Login*, so the assistant starts when the laptop is
  switched on.
* **Keep running with the lid closed:** in `/etc/systemd/logind.conf` set `HandleLidSwitch=ignore` and
  `HandleLidSwitchExternalPower=ignore`, then `sudo systemctl restart systemd-logind`.
* **Never sleep:** Settings → Power → *Automatic Suspend* off, and *Screen Blank* never.
* **One network everywhere:** let the laptop share a hotspot (Settings → Wi-Fi → *Turn On Wi-Fi Hotspot*) or
  use a phone hotspot, and join the Pi to it once (`sudo nmcli dev wifi connect <name> password <password>`
  over SSH). Then the Pi and laptop find each other indoors and outdoors, with no home router needed.

---

## 🔧 2. One-Time Setup: Raspberry Pi 5

Over SSH on the Pi:

```bash
# Code
cd ~/wearable_ws/src && git pull
source /opt/ros/jazzy/setup.bash
cd ~/wearable_ws
colcon build --symlink-install --packages-select visionnav

# GPIO library for the buttons (Raspberry Pi OS: already installed; Ubuntu 24.04:)
sudo apt install -y python3-gpiozero python3-lgpio

# Permissions: LiDAR serial port (dialout) and GPIO pins
sudo usermod -aG dialout $USER
ls -l /dev/gpiochip*        # if these are root-only (crw------- root root):
sudo groupadd -f gpio && sudo usermod -aG gpio $USER
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules

# Start the button program at every boot
sudo tee /etc/systemd/system/visionnav-buttons.service > /dev/null <<EOF
[Unit]
Description=VisionNav push buttons (and the LiDAR and camera they switch on)
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
Environment=ROS_DOMAIN_ID=42
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source $HOME/wearable_ws/install/setup.bash && exec ros2 run visionnav pi_button_panel'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable visionnav-buttons
sudo reboot
```

After the reboot:

* `systemctl status visionnav-buttons` shows it running; `journalctl -u visionnav-buttons -f` shows its log
  (`Buttons ready: SENSORS=GPIO24, LOOK=GPIO17, …`, and every press).
* To have the LiDAR and camera on at boot without pressing SENSORS, change the `ExecStart` line to end in
  `ros2 run visionnav pi_button_panel --ros-args -p sensors_at_start:=true`, then
  `sudo systemctl daemon-reload && sudo systemctl restart visionnav-buttons`.
* After `git pull` + `colcon build` on the Pi: `sudo systemctl restart visionnav-buttons`.

---

## 🔘 3. Push Buttons: Wiring

| Button | GPIO (BCM) | Header pin | Main job |
|--------|-----------|------------|----------|
| **SENSORS** | GPIO24 | pin 18 | LiDAR + camera on / off / restart |
| **LOOK** | GPIO17 | pin 11 | Vision AI: describe / ask / off |
| **MODE** | GPIO27 | pin 13 | Indoor ↔ outdoor, status |
| **HAND** | GPIO22 | pin 15 | Hand guidance to an object |
| **TALK** | GPIO23 | pin 16 | STOP / voice command / what is around me |
| GND (shared) | — | pin 14 (also 9, 20, 25) | Second leg of every button |

**Parts:** 5 momentary, normally-open push buttons (12 mm tactile or 16–19 mm panel buttons; give each a
different shape or 1–5 raised dots so they can be told apart by touch), 6 female jumper wires (or Dupont wires
soldered to the buttons), heat-shrink. No resistors: the Pi's internal pull-ups are used.

**Wiring** (Pi switched off). Each button has two sides: one goes to its GPIO pin, the other to GND.
```
   pin 18  GPIO24 ─────────────── SENSORS ──┐
   pin 11  GPIO17 ─────────────── LOOK ─────┤
   pin 13  GPIO27 ─────────────── MODE ─────┤
   pin 15  GPIO22 ─────────────── HAND ─────┤
   pin 16  GPIO23 ─────────────── TALK ─────┤
   pin 14  GND    ──────────────────────────┘ (one wire, daisy-chained to the 2nd leg of every button)
```

**Finding the pins.** Pin 1 is the header pin nearest the corner **farthest from the USB/Ethernet ports**, on
the side **toward the middle of the board**. The header is 20 rows of 2 pins; row 1 is at that end. In each row
the pin toward the middle of the board is odd (1, 3, 5 …), the pin at the board edge is even (2, 4, 6 …).
With the USB ports pointing **up** (header on the left edge), count rows from the **bottom**:

```
          board edge (even)      middle of board (odd)
 row 10 → pin 20  (GND)          pin 19
 row 9  → pin 18  SENSORS        pin 17  3.3V ✗   ← do not confuse with pin 18
 row 8  → pin 16  TALK           pin 15  HAND
 row 7  → pin 14  GND            pin 13  MODE
 row 6  → pin 12  (unused)       pin 11  LOOK
 row 5  → pin 10                 pin 9   (GND)
 row 4  → pin 8                  pin 7
 row 3  → pin 6   (GND)          pin 5
 row 2  → pin 4   5V  ✗          pin 3
 row 1  → pin 2   5V  ✗          pin 1   3.3V ✗    ← next to the mounting hole
```

* 4-leg tactile buttons: the two legs on each **long** side are joined inside. Use two **diagonally opposite** legs.
* Never connect a button to 5 V (pins 2, 4) or 3.3 V (pins 1, 17): a button to a power pin shorts it when pressed.
* `pinout` on the Pi prints the header for your board.
* Other pins: add e.g. `-p look_pin:=5` to the service's `ros2 run` line.

**Test each button.** With the service stopped (`sudo systemctl stop visionnav-buttons`), this prints the name
of every button pressed (Ctrl+C to quit, then `sudo systemctl start visionnav-buttons`):
```bash
python3 -c "
from gpiozero import Button; from signal import pause
b = {n: Button(p) for n, p in dict(SENSORS=24, LOOK=17, MODE=27, HAND=22, TALK=23).items()}
for n, v in b.items(): v.when_pressed = lambda n=n: print(n, 'pressed')
print('Press the buttons (Ctrl+C to quit)'); pause()"
```
With the service running: `ros2 topic echo /button_event` (on either computer, `ROS_DOMAIN_ID=42`) shows every
press, e.g. `{"button": "look", "event": "tap"}`.

---

## 🗺️ 4. Saved Maps (remembering the home)

* **First time: mapping.** With no saved map, indoor mode *maps*. Walk through every room, finish somewhere you
  have already been, then say **"save map"**. That saves the map, the objects seen reliably, your places and
  your names in `~/.visionnav/maps/` (`home.pbstream`, `home_objects.json`, `home_places.json`,
  `home_names.json`). The last minute of a mapping walk is not yet usable for finding you, which is why the walk
  should end somewhere already covered.
* **From then on: localization.** Indoor mode loads the saved map and finds you in it (walk a few metres after
  starting). Remembered objects are on the map at once ("go to light switch" works before the camera has seen it
  again), and the map no longer grows or drifts.
* **Another building / map again:** `WEARABLE_BRAIN_ARGS="map:=office"`, or `"localize:=false"` to map from scratch.

**What the object map does:** an object seen reliably stays on the map, drawn translucent while out of view,
and keeps its name and ID when seen again from another side — it is not forgotten because the back of a chair
looks different. When an object is **taken away**, it disappears 1–2 s after the camera looks at its spot from a
direction it was seen from before (log: `Removed <object> from the map`). Flickering misdetections are never
shown; doors, windows and switches are drawn as thin panels along their wall; people are shown only while
detected, with their own IDs and speed.

---

## 📐 5. Calibrating the Chest Rig (once, and whenever the mount changes)

All 3D accuracy depends on where the camera and LiDAR sit on the rig. Both the map and the camera AI read it from
TF (`sensor_tf.launch.py`), so there is one place to fix it.

1. **LiDAR direction (most important).** If the map moves the wrong way (walking backward shows as walking
   forward, or turning left as turning right), the LiDAR mounting is wrong. With the sensors on, wear the rig and
   run `ros2 run visionnav lidar_orientation_calibrator` on the laptop; stand still, walk ~1 m forward, turn left
   ~90° as prompted. It prints the `lidar_yaw_deg` / `lidar_roll_deg` to use. RViz's **Your Tracked Path** shows
   where the map thinks you have been.
2. **Measure** the camera lens height, the LiDAR height and the camera's downward tilt, and set them before
   starting the assistant:
   `export WEARABLE_BRAIN_ARGS="camera_height:=1.32 camera_pitch_deg:=12 lidar_height:=1.18 lidar_yaw_deg:=188"`.
   A camera tilted 10° but configured as 0° puts a floor object 3 m away about 2.5 m too far.
3. **Check the LiDAR overlay:** press **`l`** in the camera window. The dots are the LiDAR returns drawn where TF
   says they are (red = near, blue = far). They should sit on walls, door frames and people's torsos at chest
   height. Mirrored: `lidar_roll_deg:=180`. Rotated or shifted sideways: repeat step 1. Too high or low: fix
   `camera_pitch_deg` / the heights. A mirrored camera picture: `WEARABLE_CAMERA_FLIP=0`/`1`.

Every label in the camera window shows the distance, how it was measured (`LiDAR`, `depth` or `cam`), the
object's real height and, for objects on a table, the surface height.

---

## 🛠️ 6. Manual Start (debugging)

To run parts yourself (for example to watch their output), start the assistant with `WEARABLE_AUTOSTART=0`, or
start a part in its own terminal before the assistant — the assistant then uses it instead of starting its own.
Every terminal needs `export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0` and `source ~/wearable_ws/install/setup.bash`.

```bash
# Pi — LiDAR + camera (the same as pressing SENSORS; the button service may keep running)
ros2 launch visionnav pi_sensors.launch.py                 # camera:=false for the LiDAR only
# Laptop — indoor map (TFs, Cartographer, Nav2, walls, RViz)
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch visionnav laptop_brain.launch.py
# Laptop — camera AI
WEARABLE_CAMERA_MODE=ros ros2 run visionnav object_perception
# Laptop — vision AI (type questions in its terminal)
ros2 run visionnav scene_describer
# Laptop — outdoor GPS
ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyACM0 -p baud:=9600
ros2 launch visionnav gps_localization.launch.py
ros2 run visionnav gps_voice_navigator
```

---

## ❓ Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| "Waiting for the Pi…" / "The Pi is not answering" | Pi off, not booted yet, on another Wi-Fi, or another `ROS_DOMAIN_ID` | Switch it on and wait ~30 s; put both on the same network (section 1, hotspot); the button service sets `ROS_DOMAIN_ID=42` |
| No button does anything | Button service not running, wrong pin, or no GPIO permission | Pi: `systemctl status visionnav-buttons`, `journalctl -u visionnav-buttons -f` (must say `Buttons ready`); test the wiring (section 3) |
| Presses show in `ros2 topic echo /button_event` but nothing is said | The assistant is not running, or in another `ROS_DOMAIN_ID` | Start the assistant with `ROS_DOMAIN_ID=42` |
| Camera window shows "Waiting for camera feed…" / "The camera is not running" | LiDAR and camera not switched on | Press **SENSORS**; if it says they could not start, check the USB cables and `~/.visionnav/logs/pi_sensors.log` on the Pi |
| "The camera and LiDAR could not start" | LiDAR not on `/dev/ttyUSB0`, no `dialout` permission, or camera unplugged | Check cables; `sudo usermod -aG dialout $USER` and reboot the Pi |
| `error code: 80008004` in `pi_sensors.log` | LiDAR serial port unavailable | Check the LiDAR USB cable; `ls -l /dev/ttyUSB0` |
| "The vision AI could not start" | Ollama not running or the model missing | `ollama list` must show `qwen3-vl:2b-instruct`; see `~/.visionnav/logs/vision_ai.log` |
| "… did not start" after a mode switch | That program failed | Read `~/.visionnav/logs/<part>.log` |
| `YOLO device: CPU` in `perception.log` | NVIDIA driver not loaded | `sudo apt install linux-modules-nvidia-595-open-$(uname -r)` and reboot |
| `numpy.core.multiarray failed` | A pip install pulled NumPy 2 into `~/.local` | `python3 -m pip uninstall --break-system-packages numpy scipy` (removes only the `~/.local` copies) |
| Markers in the wrong place / on the wrong side | Rig geometry in TF does not match the rig | Section 5 (press `l` for the LiDAR overlay) |
| Wrong distance to an object | LiDAR not anchoring it | Start the camera AI by hand with `WEARABLE_LIDAR_SNAP_DEBUG=1` (section 6): once a second per class it logs whether the LiDAR anchored the object and why not |
