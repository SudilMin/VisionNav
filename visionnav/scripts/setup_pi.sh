#!/bin/bash
# VisionNav — Raspberry Pi 5 setup. Run ON THE PI (after `git pull`), once and again after every update:
#
#   bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh          # build + GPIO + permissions + boot service
#   bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh test     # print every button press (wiring check)
#
# The boot service (visionnav-buttons) starts the button program at every boot; its SENSORS button starts
# the LiDAR and camera. Nothing else has to be typed on the Pi afterwards.
set -e
WS="$HOME/wearable_ws"

# Only on the Raspberry Pi: on the laptop it would install a second, button-less panel that confuses the real one
if ! grep -qi "raspberry pi" /proc/device-tree/model 2>/dev/null; then
    echo "This is not the Raspberry Pi ($(hostname)). Log in to the Pi first, then run this script there:"
    echo "    ssh pi@raspberrypi.local"
    exit 1
fi
PINS="SENSORS=24 LOOK=17 MODE=27 HAND=22 TALK=23"

if [ "$1" = "test" ]; then
    echo "Stopping the button service while testing (it is started again at the end)..."
    sudo systemctl stop visionnav-buttons 2>/dev/null || true
    trap 'echo; sudo systemctl start visionnav-buttons 2>/dev/null || true; echo "Button service started again."' EXIT
    python3 - $PINS <<'PY'
import sys
from gpiozero import Button
from signal import pause
buttons = {}
for arg in sys.argv[1:]:
    name, pin = arg.split("=")
    try:
        buttons[name] = Button(int(pin), pull_up=True, bounce_time=0.03)
        buttons[name].when_pressed = lambda n=name, p=pin: print(f"{n} pressed (GPIO{p})", flush=True)
        buttons[name].when_released = lambda n=name: print(f"{n} released", flush=True)
    except Exception as e:
        print(f"{name} on GPIO{pin}: cannot be used: {e}")
print("Press each button (Ctrl+C to stop). Nothing printed for a button = check its two wires.", flush=True)
pause()
PY
    exit 0
fi

echo "== 1/5  Build the visionnav package"
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install --packages-select visionnav

echo "== 2/5  GPIO library for the buttons"
if python3 -c "import gpiozero, lgpio" 2>/dev/null; then
    echo "gpiozero + lgpio already installed"
else
    sudo apt-get update
    sudo apt-get install -y python3-gpiozero python3-lgpio
fi

echo "== 3/5  Permissions: LiDAR serial port (dialout) and GPIO pins (gpio)"
sudo usermod -aG dialout "$USER"
sudo groupadd -f gpio
sudo usermod -aG gpio "$USER"
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== 4/5  Boot service visionnav-buttons"
sudo tee /etc/systemd/system/visionnav-buttons.service > /dev/null <<EOF
[Unit]
Description=VisionNav push buttons (and the LiDAR and camera they switch on)
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
# lgpio (the Pi 5 GPIO library) writes its notification files in the working directory: "/" is not writable,
# and the buttons then fail to open while the program keeps running
WorkingDirectory=$HOME
Environment=LG_WD=/tmp
Environment=ROS_DOMAIN_ID=42
Environment=ROS_LOCALHOST_ONLY=0
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && exec ros2 run visionnav pi_button_panel'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable visionnav-buttons
sudo systemctl restart visionnav-buttons

echo "== 5/5  Check"
sleep 8
systemctl --no-pager status visionnav-buttons | head -4
echo "--- last log lines:"
journalctl -u visionnav-buttons -n 12 --no-pager -o cat
echo
if journalctl -u visionnav-buttons -n 50 --no-pager -o cat | grep -q "unavailable"; then
    echo "PROBLEM: some buttons could not be opened — see the 'unavailable' lines above."
elif journalctl -u visionnav-buttons -n 50 --no-pager -o cat | grep -q "Buttons ready: SENSORS"; then
    echo "OK: the buttons are ready. Press SENSORS and watch:  journalctl -u visionnav-buttons -f"
else
    echo "The button program is not ready yet — see the log above, or run:  journalctl -u visionnav-buttons -f"
fi
