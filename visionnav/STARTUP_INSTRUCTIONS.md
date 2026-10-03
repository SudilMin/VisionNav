# VisionNav Startup Guide

VisionNav runs on two computers:

* **Raspberry Pi 5** (on the chest rig): the five push buttons, the LiDAR, the camera and the IMU (MPU-6050). The button program
  starts by itself when the Pi boots; the **SENSORS** button turns the LiDAR and camera on and off, with or
  without the laptop.
* **Laptop** (MSI Sword 15, RTX 2050): the AI, the map and the voice. The navigation assistant starts at login
  (or with one command) and starts everything else — the map, the camera AI, the vision AI — as soon as the
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
2. **Press SENSORS.** Click → "Camera and LiDAR on." → the laptop starts indoor mode by itself (the map window
   and the camera window open) → **"Indoor mode activated."** Each thing is said once, when it is done.
3. Use the buttons (below), or type commands in the assistant's terminal. Every tap clicks, so you know it was
   heard. Hold LOOK and say **"help"** to hear what the buttons do.
4. **Hold MODE** when you are done with a place: "Map and camera closed." — the map and what it learned are
   forgotten; the camera and LiDAR stay on. **Hold SENSORS** to turn the camera and LiDAR off too: "Camera and
   LiDAR off. Indoor mode paused." The next start maps the place you are in afresh (section 4).

### What each button does, and what you hear

| Button | Press | What happens and what you hear |
|---|---|---|
| **SENSORS** | tap | Turns the LiDAR, camera and IMU on — click → "Camera and LiDAR on." → the current mode starts → "Indoor mode activated." Already on: "Camera and LiDAR on." again. One of them stopped (e.g. the camera was unplugged): all are started again. If they fail: "The camera and LiDAR could not start. Check their cables." A very short press counts. |
| | hold | Turns them off as soon as the button has been down 0.6 s, while it is still down (the LiDAR stops; no need to let go first), and the laptop closes the mode's programs: click → "Camera and LiDAR off. Indoor mode paused. Press the sensor button to continue." The Pi does this by itself: it works with the laptop off. Letting go afterwards does not turn them on again. |
| **LOOK** | tap | If the vision AI is off: "Turning on the vision AI." → starts Qwen3-VL (about half a minute) → describes the scene. If it is already on: "Looking." → the description |
| | hold | Beep → ask the camera a question while holding ("what colour is the door?") → let go → the answer (the vision AI starts first if it is off). Only the newest request is answered: a description still waiting or being made when you ask is dropped. A command said here is carried out instead ("help", "status", "vision off", "find the cup", "what is ahead", "can I cross"…). To turn the vision AI off and free its ~2 GB of GPU memory: hold LOOK, say **"vision off"** → "Vision AI off." |
| | double press | **Vision AI off** (frees its GPU memory) → "Vision AI off." (A single tap now describes after a short wait, 0.45 s, to tell it from a double press.) |
| **MODE** | tap | While paused (sensors off): "Outdoor mode selected. It starts when the camera and LiDAR are on." Otherwise: click → closes the indoor map (Cartographer, Nav2, its RViz), opens the outdoor view (its own RViz) → "Outdoor mode activated." Tap again: the outdoor view closes and the indoor map opens → "Indoor mode activated." The camera window stays open throughout. |
| | hold | **Close the map and the camera feed; the session ends.** Stops the map (Cartographer, Nav2, the map window), the camera AI with its camera window and the outdoor view, and any guidance → click → "Map and camera closed. Press mode to start again." The map, the objects, the places and the names of this session are forgotten (nothing is kept for another day). The camera and LiDAR stay on (SENSORS is their button); tap MODE to start again, with a new map. The vision AI stays on: say "vision off" to stop it. The status is a voice command: hold LOOK, say "status". |
| **HAND** | tap | (After 0.45 s, see double tap.) Starts the camera AI if it is off ("Turning on the camera AI.") → guides your hand to the object found last (or the nearest one ahead): "Reach out your hand toward the cup." — walking you there first if it is more than 1 m away. Nothing found: "Nothing to reach for yet…". Tap again: "Hand guidance disabled." |
| | hold | Click → stops hand guidance (and the walk to the object), and closes the camera AI and its camera window if the HAND button opened it (no mode running): "Hand guidance off. Camera closed." In indoor or outdoor mode the camera stays (the mode needs it; MODE hold closes it): "Hand guidance off." Nothing running: "Hand guidance is already off." |
| | double tap | **Face mode** on / off: "Face mode. Press to recognise a person. Hold and say a name to remember them." / "Face mode off." In face mode: **tap** → who is in front: "This is Kamal." / "I see 2 people: Kamal on your left, and someone I do not know on your right." / "I don't know this person…"; **hold** → beep → say the name ("Kamal", "his name is Kamal") → let go → "I will remember Kamal." (the largest face in view; do it again from another angle to recognise them better). Faces are kept for another day, on this laptop only (`~/.visionnav/faces/faces.json`). A HAND tap waits 0.45 s to see whether a second one follows. |
| **TALK** | tap | While navigating: **ends the navigation** → "Navigation stopped." Otherwise nothing. |
| | hold | **Navigation (indoor mode only).** Beep → say where to go while holding ("chair", "chair 1" — the ID shown on the map and camera window —, "the table with the cup", "my chair", "kitchen") → let go → "Taking you to the chair…" → turn-by-turn guidance, always with the clock direction and the feet left ("Turn left, to 10 o'clock. 12 feet.", "Bear slightly right, 1 o'clock. 6 feet.", "Straight ahead. 3 feet."; every 2 feet in the last 12 feet) → "You have arrived. The chair is at 12 o'clock, about 2 feet away." (where it is from where you stopped, for your last step) and the navigation ends (no hand guidance; the HAND button does that). A command also works here ("status", "save this place as kitchen", "call this my chair"). Not in indoor mode: "You can't use navigation now. It works only in indoor mode." |
| | double tap | What is around you |

A hold is 0.6 s and acts at that moment, with the button still down; a double tap is two taps within 0.4 s. The
Pi reads the buttons 200 times a second, so a very short press counts and a poor contact does not split one press
into several. Every tap clicks, and so does a SENSORS, LOOK or MODE
hold once it is long enough (let go). What a button did is said once, when it is done. If the camera or LiDAR stream stops
you hear "The camera signal is lost." ("The camera is on." when it returns), and if the Pi drops off the
network, "The Pi is not answering. Check that it is switched on and on the same Wi-Fi."

### Which programs each mode runs

The assistant starts and stops these itself (`system_manager.py`); the Pi's button program starts the sensors.

| Mode / button | Programs |
|---|---|
| **SENSORS** (Pi) | `pi_sensors.launch.py`: RPLiDAR C1 (`sllidar_node`) + chest camera (`phone_camera_publisher`) |
| **Indoor** (default, laptop) | `laptop_brain.launch.py use_rviz:=false` (sensor TFs, Cartographer SLAM, Nav2, walls) + the map window (RViz, `rviz/visionnav.rviz`; opened again at every mode start if it was closed) + `object_perception` (camera AI, camera window) |
| **Outdoor** (laptop) | `outdoor_sensors.launch.py` (camera and LiDAR mounts, live RViz view) + `object_perception` (hazard warnings, below) |
| **LOOK** (laptop) | `scene_describer` (Qwen3-VL), started on the first press, stopped by saying "vision off" |
| **MODE** hold (laptop) | Stops both modes' programs (everything above except the sensors and the vision AI) and forgets the session |

* A program already started by hand in a terminal is used as it is — never started twice, and never stopped by
  the buttons.
* Everything the assistant started stops when it exits (Ctrl+C, closing its terminal, or saying "exit").
* Each program's output: `~/.visionnav/logs/<part>.log` (`brain`, `map_view`, `perception`, `vision_ai`, `outdoor_tf`,
  `pi_sensors` on the Pi).
* Settings (set before starting the assistant): `WEARABLE_AUTOSTART` — `sensors` (default: start the mode when
  the camera and LiDAR come on, pause it when they go off), `now` (start at once), `off` (never, section 6);
  `WEARABLE_MODE=outdoor` starts in outdoor mode;
  `WEARABLE_BRAIN_ARGS="camera_height:=1.32 camera_pitch_deg:=12 lidar_height:=1.18"` passes the rig's measured
  geometry to the map (section 5).
* Switching from indoor to outdoor ends the indoor session: its map and objects are forgotten (section 4).

### Outdoor mode: hazard warnings

Outdoors nothing is mapped: the camera AI watches what is in front of you right now and says only what matters,
with the distance in feet and the direction — **one sentence at a time, about the most urgent thing only**:

* However many things are in your path, only the nearest / most urgent one is spoken about ("Chair ahead, 1 foot.
  Step right."); the next one is announced once that one is out of the way.
* It is said again only when it gets much closer, becomes a danger, or after a while (4 s for a danger, 8 s for a
  warning, twice as long each time while nothing changes — standing in front of a table is not repeated forever).
* A sentence is never cut off by another one of the same urgency. Only a greater danger cuts in: a danger over a
  warning or over anything else being said, and "Stop. Car coming…" over any other danger.
* A warning that could not be said within 2 s (the voice was busy) is dropped instead of being said late.
* What is shown and said follows the camera: an object it named disappears 1 s after the camera stops
  seeing it (4 s once it is beside you, out of the camera's view; the LiDAR still warns there, unnamed).
* Nothing behind you is tracked or said. The LiDAR spins all round, but its returns behind you (more than 100°
  from straight ahead: your own body) are dropped before outdoor mode uses the scan.
* Positions come from the LiDAR wherever it sees the object (anything reaching chest height); the camera
  names it. Things lower than the LiDAR (a chair, a kerb, a rock) are placed from the camera's depth, and
  never take the range of the wall behind them. That depth comes from whichever depth model agrees with
  the LiDAR (checked every second). In a room the indoor one won (the street one read everything 1-3 m
  away as about 5 m); on a street the street one is expected to. `~/.visionnav/logs/perception.log` shows
  `📏 Depth: the indoor model now …` when it changes.

| What | Example of what you hear | How it is found |
|---|---|---|
| Something in your path (pole, tree, wall, parked car, bin, person standing) | "Pole ahead, 6 feet. Step right." — "Obstacle ahead, 3 feet. Step left or right." — "Path clear." once it is behind you | LiDAR at chest height (anything, named after the camera's detection there) + ground analysis from depth for low things (rocks, bollards, cones) |
| Holes and drops (pothole, open drain, kerb, steps, stairs) | "Pothole ahead, 8 feet." — "Drop or hole ahead, 5 feet." | Detector + the ground plane fitted in the depth every frame |
| Head height (low branch, sign) | "Low branch at head height, 5 feet ahead. Duck." | Detector + depth: something at head height with free space below it |
| Vehicles | "Car approaching on your left, 40 feet." — "Stop. Three-wheeler coming ahead, 15 feet." | Tracked with their speed toward you: a warning under 6 s to reach you, "Stop" under 3 s |
| People and animals in your way | "Person coming toward you, 8 feet." — "Dog on your right, 6 feet." | Tracked like vehicles |
| Something coming from the side (outside the camera) | "Something coming on your left, 7 feet." | LiDAR, ahead and beside you, with your own motion subtracted |
| Zebra crossing | "Zebra crossing ahead, 12 feet." | The white stripe pattern on the ground (the detector alone rarely finds one) |
| Traffic and pedestrian lights | "Pedestrian signal is red. Wait." — "Pedestrian signal is green." (said when it changes) | The lit lamp's colour |

* **How it sees, like a self-driving car** (all live, nothing saved):
  * **Its own motion:** LiDAR odometry (scan matching against the last few metres of scans) knows how you walk
    and turn, so a parked car is still and a car's or cyclist's speed is its own, not relative to your walking.
    Also published on `/odom`. The log says `🧭 LiDAR odometry locked`; in a wide-open place with
    nothing within ~12 m it falls back to tracking relative to you.
  * **Objects ahead and beside you:** the LiDAR tracks them; the camera names them ("person") and the name stays
    while the LiDAR still sees it beside you. Something moving toward you from the side is said: "Something
    coming on your left, 7 feet." What is behind you (more than 100° round from straight ahead) is not tracked,
    drawn or said; the occupancy grid below still shows the whole scan.
  * **Occupancy:** a grid of the last ~2 seconds (fading) of what the LiDAR hits and what depth finds low or
    dropping away ahead — anything, whether or not it has a name.
* **RViz** (opens with outdoor mode, `rviz/visionnav_outdoor.rviz`): you (blue) at the centre facing up the
  screen, your walking path (blue, orange / red where it is blocked, with the distance), grey columns for
  occupied space and faint blue walkable ground, and a model for each object seen right now — cars (body and
  cabin), buses, bikes, people, animals, trees, poles, cones, lights showing their colour, crossings as white
  stripes, holes as magenta discs — with its distance, its own speed and its predicted path (3 s). Threats turn
  orange / red. An object leaves the view as soon as no sensor sees it. Topics: `/outdoor_markers`,
  `/outdoor_occupancy`. Without a screen: `use_rviz:=false` on `outdoor_sensors.launch.py`.
* **Say** (hold LOOK): "what is ahead" (also TALK double tap), "what colour is the light", "can I cross" (the
  crossing, its signal and any vehicle coming — it never says it is safe; listen for traffic), "quiet warnings"
  (two minutes; dangers are still said), "warnings on", "help".
* **Settings**: `WEARABLE_USER_HEIGHT=1.75` (m, for head-height warnings), `WEARABLE_UNITS=metric` (metres instead
  of feet), `WEARABLE_RECORD_DIR=~/walk1` (saves the camera view of every warning, and one frame every 2 s, with
  `alerts.jsonl` — to review a walk afterwards).
* The chest camera cannot see the ground closer than about 1.5–2.5 m (it depends on its tilt), so holes are
  warned about while they are still ahead. It does not replace the cane.
* The outdoor detector has its own engine (`yoloe-11s-seg-outdoor-*.engine`, built once on the first start, ~3–6
  min) and the outdoor depth model (`models/depth_anything_v2_metric_outdoor_vits.pth`, section 1).

### Voice commands (hold LOOK — or TALK in indoor mode — or type them in the assistant's terminal)

Objects are described the way you know them: IDs such as `table_2` and colours are never needed or spoken, and
distances are in feet ("The table, with the cup on it, 7 feet away, at 1 o'clock").

| Say | What it does |
|---|---|
| "find the table where the cup is", "where is the cup on the table" | Says where it is |
| "go to the chair next to the door", "take me to the nearest chair", "go there" | Walking guidance (Nav2 route, turn-by-turn) |
| "another one", "list them", "the second one", "the one in the kitchen" | When several objects match: "go to the chair" goes to the nearest ("There are 3 chairs. Taking you to the nearest one…") |
| "what is on the table", "what is around me" | Answered from the map |
| "call this my chair" → later "go to my chair"; "forget name my chair" | Your own names for objects (kept for this session) |
| "save this place as kitchen" (or "mark kitchen"), "go to kitchen", "where am i", "forget place kitchen" | Named places |
| "save map" | Nothing is saved for another day: "The map is kept until you hold the mode button. It is not saved for another day." |
| "grasp the cup" ("grab", "pick up", "reach for") | Hand guidance: "Right 4 inches", "Lower 2 inches", "Forward 6 inches"… "Stop. The cup is at your hand." (also starts on arrival at an object) |
| "what colour is the door?", "describe …", "read …", "how many people are here?" | Sent to the vision AI (it starts first if it is off) |
| "vision off" | Stops the vision AI and frees its GPU memory |
| "who is this", "who is here" | Who is in front of the camera (as a HAND tap in face mode) |
| "forget face Kamal" | Deletes that remembered face |
| "status" | Mode (or "All modes are off."), vision AI on/off, missing sensors, map, how many objects, what is around you |
| "help" | What the buttons do and what you can say |
| "stop" / "exit" | Stop everything / shut the assistant down |

Speech is recognised offline on the laptop's processor (Whisper `small.en`, ~0.5 GB, downloaded once into
`~/.cache/huggingface` at the first start) about 1.4 s after you let go: wait for the beep, then speak. Whisper is
given the system's own words and the names it knows (people, places, objects on the map) as a hint, and a word it
mishears as a sound-alike is put right ("share one" → "chair one", "vision of" → "vision off", "camel" → a
remembered "Kamal"; the terminal shows `🎤 Understood: …`). A new name being remembered is kept as heard.
Faster but a little less accurate: `WEARABLE_WHISPER_MODEL=base.en` (0.4 s). `small.en` understood 92.5 % of 120
recorded commands.
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
* **Object detector.** YOLOE-11s-seg with the vocabulary in `VOCABULARY` (top of `visionnav/object_perception.py`),
  and outdoors `OUTDOOR_VOCABULARY` (top of `visionnav/outdoor_awareness.py`). The first start after a vocabulary
  changes builds its TensorRT engine (~2–6 min, one time); the camera window opens when it is done. To detect
  something new, add its name to the vocabulary. The build needs the text encoder `models/mobileclip_blt.ts`
  (Ultralytics downloads it there if it is missing).
* **Outdoor depth model** (outdoor mode; without it outdoor mode uses the indoor one, which reads no farther than 20 m):
  ```bash
  curl -L -o ~/wearable_ws/src/visionnav/models/depth_anything_v2_metric_outdoor_vits.pth \
    https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-VKITTI-Small/resolve/main/depth_anything_v2_metric_vkitti_vits.pth
  ```
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

From the laptop, log in to the Pi, then run the setup script **in the Pi's terminal** (type the lines one at a
time; do not paste the `ssh` line again once you are logged in — that opens a second login and swallows
everything pasted after it):

```bash
ssh pi@raspberrypi.local                                   # on the laptop (your Pi's user / address)
cd ~/wearable_ws/src && git pull                           # on the Pi
bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh       # on the Pi
```

The script builds the package, installs the GPIO and I2C libraries, turns on the I2C bus for the IMU, gives your
user access to the LiDAR port, the GPIO pins and the I2C bus, installs the **visionnav-buttons** boot service and
starts it. It ends with `OK: the buttons are ready` (and, the first time, asks for one reboot to turn on I2C).
Run the same two Pi lines again after every update. Then `exit` — the Pi needs no terminal from now on.

* **Watch the buttons:** `journalctl -u visionnav-buttons -f` shows `Buttons ready: SENSORS=GPIO24, …` and every
  press (`SENSORS tap`, `sensors: starting`, `sensors: on`; `SENSORS hold_start`, `sensors: stopping`, `sensors: off`). `systemctl status visionnav-buttons` shows whether
  it runs.
* **Check the wiring:** `bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh test` prints the name of every
  button pressed (Ctrl+C to stop). `setup_pi.sh imu` checks the IMU (section 3b).
* **LiDAR and camera on at every boot** (without pressing SENSORS):
  `sudo sed -i 's/pi_button_panel$/pi_button_panel --ros-args -p sensors_at_start:=true/' /etc/systemd/system/visionnav-buttons.service`
  then `sudo systemctl daemon-reload && sudo systemctl restart visionnav-buttons`.

---

## 🔘 3. Push Buttons: Wiring

| Button | GPIO (BCM) | Header pin | Main job |
|--------|-----------|------------|----------|
| **SENSORS** | GPIO24 | pin 18 | LiDAR + camera + IMU: tap on, hold off |
| **LOOK** | GPIO17 | pin 11 | Vision AI: tap describe, hold ask a question, double press off |
| **MODE** | GPIO27 | pin 13 | Tap indoor ↔ outdoor, hold everything off |
| **HAND** | GPIO22 | pin 15 | Tap hand guidance, hold off; double tap face mode |
| **TALK** | GPIO23 | pin 16 | Indoor navigation: hold say where to go, press again to stop |
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
 row 5  → pin 10                 pin 9   IMU GND
 row 4  → pin 8                  pin 7
 row 3  → pin 6   (GND)          pin 5   IMU SCL
 row 2  → pin 4   5V  ✗          pin 3   IMU SDA
 row 1  → pin 2   5V  ✗          pin 1   IMU VCC (3.3V)  ← next to the mounting hole
```

* 4-leg tactile buttons: the two legs on each **long** side are joined inside. Use two **diagonally opposite** legs.
* Never connect a button to 5 V (pins 2, 4) or 3.3 V (pins 1, 17): a button to a power pin shorts it when pressed.
  Pins 1, 3, 5 and 9 belong to the IMU (section 3b).
* `pinout` on the Pi prints the header for your board.
* Other pins: add e.g. `--ros-args -p look_pin:=5` to the `ExecStart` line in
  `/etc/systemd/system/visionnav-buttons.service`, then `sudo systemctl daemon-reload && sudo systemctl restart visionnav-buttons`.

**Test each button:** `bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh test` on the Pi prints the name of
every button pressed and released (Ctrl+C to stop; the button service is paused meanwhile).
With the service running: `ros2 topic echo /button_event` (on either computer, `ROS_DOMAIN_ID=42`) shows every
press, e.g. `{"button": "look", "event": "tap"}`.

---

## 🧭 3b. IMU (MPU-6050): Wiring and Placement

The IMU tells the system how fast you turn (100 times a second; the LiDAR scans 10 times) and where down is.
Indoors the map uses it to follow fast turns and to level the scan when your chest leans; outdoors the motion
tracking uses it for every turn. Everything works without it; with it, fast turns no longer lose your position.

**Wiring** (Pi switched off; GY-521 board, 4 female–female jumper wires, as short as possible, max ~30 cm):

| GY-521 pin | Pi 5 header pin | What |
|------------|-----------------|------|
| **VCC** | pin 1 | 3.3 V (not 5 V: the board works at either, but 3.3 V keeps SDA/SCL at the Pi's safe level) |
| **GND** | pin 9 | Ground |
| **SDA** | pin 3 (GPIO2) | I2C data |
| **SCL** | pin 5 (GPIO3) | I2C clock |
| XDA, XCL, AD0, INT | — | Not connected (AD0 open = address 0x68) |

```
   GY-521              Pi 5 header (pin 1 end)
   VCC ──────────────── pin 1  3.3V
   SDA ──────────────── pin 3  GPIO2
   SCL ──────────────── pin 5  GPIO3
   GND ──────────────── pin 9  GND
```

**Where to put it: on the chest plate, not on top of the LiDAR and not on the side.** It must move exactly like
the LiDAR, so it goes on the same rigid part:

* **Centre of the chest plate** (on the breastbone), on the same hard plate as the LiDAR, a few cm from it,
  as close to it as fits (default height 1.15 m, just below a LiDAR at 1.2 m).
* **Stand the board upright** against the plate: **chip side facing forward** (away from your chest), the
  printed **Y arrow pointing up** (the X arrow then points to your left). This is the default mount; another
  way round works too once measured (below).
* **Fix it rigidly**: two M2.5/M3 screws through the board's holes with 3–5 mm spacers, or hard double-sided tape.
  Not foam, Velcro or a loose pocket: the board must not wobble or it measures its own wobble.
* **Not on top of the LiDAR**: its spinning motor shakes the gyro, and anything above it can block the 360° scan.
* **Not on the side pods, shoulder straps or backpack**: straps and pods bend and swing with your arms and
  breathing; the backpack moves differently from your chest.
* **Not on the Pi or next to its fan**: the fan shakes it, and the Pi's heat makes the gyro drift.

**Check it** (Pi, after `setup_pi.sh` and one reboot): `bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh imu`
shows `68` in the I2C table, the chip's readings, which arrow points up, then asks you to stand straight and to
lean forward, and prints the mount, e.g. `imu_roll_deg:=0 imu_pitch_deg:=1 imu_yaw_deg:=1` (the default: this rig's
board, flat, chip up, X arrow forward). If it prints other numbers, add them to `WEARABLE_BRAIN_ARGS` (section 5): with a wrong mount the map is levelled the
wrong way. Board flat on a shelf, chip up, X arrow forward: `imu_roll_deg:=0 imu_pitch_deg:=0 imu_yaw_deg:=0`.

**Using it** needs nothing more: SENSORS starts the IMU with the LiDAR and camera (`/imu/data`, 100 Hz), and the
assistant gives the map the IMU whenever the Pi publishes it (`~/.visionnav/logs/brain.log`: `Using the chest IMU`).
Keep still for a second after turning the sensors on (it measures the gyro's drift then, and again whenever you
stand still). `WEARABLE_IMU=0` on the laptop maps without it. If the IMU fails *while* mapping (a wire comes
loose), the map stops following you: hold SENSORS (off), then tap it (on) to restart without it, then fix the wire.

---

## 🗺️ 4. The Indoor Map: One Session at a Time

* **Every indoor session maps the place it is in, from scratch** — your home, a neighbour's house, an office.
  Walk around: the map (RViz) grows with what the LiDAR sees, the objects the camera finds are placed on it, and
  coming back to a place you have already walked through finds you on the same map again (loop closure), with the
  objects where they were.
* **The session ends** with a MODE hold (or when the camera and LiDAR are switched off, or when you switch to
  outdoor mode): the map, the objects, your named places ("save this place as kitchen") and your names for
  objects ("call this my chair") are all forgotten. Nothing is saved for another day and no old map is loaded:
  an old map put you in the wrong place anywhere else.
* Behind the scenes: this session's places and names live in a temporary folder (`/tmp/visionnav_session_*`)
  that is deleted when the map stops.

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
   The defaults are this rig's, measured on the rig: camera 52° wide (`WEARABLE_CAMERA_HFOV_DEG`, read by
   object_perception), tilted 10° down, turned 2° left. A wrong width spreads every object sideways.
   With the IMU, add its height and the mount `setup_pi.sh imu` printed (section 3b), e.g.
   `imu_height:=1.15 imu_roll_deg:=0 imu_pitch_deg:=1 imu_yaw_deg:=1`. If they do not put the IMU's gravity
   upward, the assistant starts the map without the IMU and says so.
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
# Laptop — outdoor camera AI (sensor mounts first; add the rig's geometry as for the brain)
ros2 launch visionnav outdoor_sensors.launch.py
WEARABLE_CAMERA_MODE=ros WEARABLE_MODE=outdoor ros2 run visionnav object_perception
ros2 topic echo /outdoor_alert            # what would be said, as it is decided
```

---

## ❓ Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| "Waiting for the Pi…" / "The Pi is not answering" | Pi off, not booted yet, on another Wi-Fi, or another `ROS_DOMAIN_ID` | Switch it on and wait ~30 s; put both on the same network (section 1, hotspot); the button service sets `ROS_DOMAIN_ID=42` |
| `setup_pi.sh imu`: no `68` in the table / `IMU not found` in the button log | I2C off (reboot after `setup_pi.sh`), SDA/SCL swapped, VCC not on pin 1, or a loose wire | Section 3b wiring; `ls /dev/i2c-1` must exist; `i2cdetect -y 1` |
| The map turns the wrong way or smears only with the IMU | IMU mount in TF does not match the board | `setup_pi.sh imu`, put the printed `imu_*_deg` in `WEARABLE_BRAIN_ARGS`; `WEARABLE_IMU=0` meanwhile |
| The LiDAR keeps spinning although the sensors are off, and SENSORS / MODE did not stop it | Its driver was ended without a clean exit (the button service restarted or stopped with the sensors on, a crash): only a clean exit stops the motor | The button program now stops the motor itself whenever it turns the sensors off and when it starts (`LiDAR motor stopped` in `journalctl -u visionnav-buttons`). If it still spins: tap SENSORS (on), wait 10 s, hold SENSORS (off) |
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
