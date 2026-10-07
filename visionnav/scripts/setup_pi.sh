#!/bin/bash
# VisionNav — Raspberry Pi 5 setup. Run ON THE PI (after `git pull`), once and again after every update:
#
#   bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh          # build + GPIO + permissions + boot service
#   bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh test     # print every button press (wiring check)
#   bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh imu      # IMU wiring check + its mount on the rig
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

# I2C bus 1 (header pins 3 and 5) for the IMU: Python library, i2cdetect, the bus itself and its permission.
# Sets REBOOT_FOR_I2C=1 when the bus only appears after a reboot.
setup_i2c() {
    if ! { python3 -c "import smbus2" 2>/dev/null || python3 -c "import smbus" 2>/dev/null; } \
            || ! command -v i2cdetect > /dev/null; then
        sudo apt-get update
        sudo apt-get install -y python3-smbus i2c-tools
    else
        echo "smbus and i2c-tools already installed"
    fi
    # Ubuntu and Raspberry Pi OS: /boot/firmware/config.txt (older images: /boot/config.txt)
    local cfg=/boot/firmware/config.txt
    [ -f "$cfg" ] || cfg=/boot/config.txt
    REBOOT_FOR_I2C=0
    if [ -f "$cfg" ] && ! grep -qE "^dtparam=i2c_arm=on" "$cfg"; then
        # Under [all]: the file may end inside a model section ([cm4], [pi4]) that the Pi 5 skips
        printf '\n[all]\ndtparam=i2c_arm=on\n' | sudo tee -a "$cfg" > /dev/null
        echo "I2C turned on in $cfg"
        REBOOT_FOR_I2C=1
    fi
    echo i2c-dev | sudo tee /etc/modules-load.d/i2c-dev.conf > /dev/null
    sudo modprobe i2c-dev || true
    sudo groupadd -f i2c
    sudo usermod -aG i2c "$USER"
    echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' | sudo tee /etc/udev/rules.d/99-i2c.rules > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    [ -e /dev/i2c-1 ] || REBOOT_FOR_I2C=1
}

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

if [ "$1" = "imu" ]; then
    # Reads the chip next to a running IMU node without disturbing it (no reset)
    setup_i2c
    if [ "$REBOOT_FOR_I2C" = "1" ]; then
        echo
        echo "The I2C bus is not there yet: reboot the Pi (sudo reboot), then run this again:"
        echo "    bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh imu"
        exit 1
    fi
    if [ ! -r /dev/i2c-1 ] || [ ! -w /dev/i2c-1 ]; then
        echo
        echo "Your user was just given access to the I2C bus: reboot the Pi (sudo reboot), then run this again."
        exit 1
    fi
    # The IMU node (started with the sensors) would restart the chip under the check: pause the sensors
    echo "Pausing the button service and sensors during the check (started again at the end)..."
    sudo systemctl stop visionnav-buttons 2>/dev/null || true
    trap 'echo; sudo systemctl start visionnav-buttons 2>/dev/null || true; echo "Button service started again (press SENSORS to turn the sensors on)."' EXIT
    echo
    echo "I2C bus 1 (header pins 3 and 5) — the MPU-6050 shows as 68:"
    sudo i2cdetect -y 1
    source /opt/ros/jazzy/setup.bash
    source "$WS/install/setup.bash"
    ros2 run visionnav mpu6050_imu calibrate || true
    exit 0
fi

echo "== 1/6  Build the visionnav package"
source /opt/ros/jazzy/setup.bash
cd "$WS"
# Links to files since deleted from the repository (a removed launch file) make the build fail:
# "can't copy .../build/visionnav/launch/<name>.launch.py: doesn't exist or not a regular file"
find "$WS/build/visionnav" -xtype l -delete 2>/dev/null || true
colcon build --symlink-install --packages-select visionnav

echo "== 2/6  GPIO library for the buttons"
if python3 -c "import gpiozero, lgpio" 2>/dev/null; then
    echo "gpiozero + lgpio already installed"
else
    sudo apt-get update
    sudo apt-get install -y python3-gpiozero python3-lgpio
fi

echo "== 3/6  I2C bus for the IMU (MPU-6050 on header pins 3 and 5)"
setup_i2c

echo "== 4/6  Permissions: LiDAR serial port (dialout) and GPIO pins (gpio); I2C (i2c) in step 3"
sudo usermod -aG dialout "$USER"
sudo groupadd -f gpio
sudo usermod -aG gpio "$USER"
echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "== 5/6  Boot service visionnav-buttons"
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
START="$(date '+%Y-%m-%d %H:%M:%S')"
sudo systemctl restart visionnav-buttons

echo "== 6/6  Check"
sleep 8
systemctl --no-pager status visionnav-buttons | head -4
echo "--- log since this start:"
# Only this start: older runs' errors in the journal are not this run's problem
LOG="$(journalctl -u visionnav-buttons --since "$START" --no-pager -o cat)"
echo "$LOG" | tail -12
echo
if echo "$LOG" | grep -q "unavailable"; then
    echo "PROBLEM: some buttons could not be opened — see the 'unavailable' lines above."
elif echo "$LOG" | grep -q "Buttons ready: SENSORS"; then
    echo "OK: the buttons are ready. Press SENSORS and watch:  journalctl -u visionnav-buttons -f"
else
    echo "The button program is not ready yet — see the log above, or run:  journalctl -u visionnav-buttons -f"
fi
if [ "$REBOOT_FOR_I2C" = "1" ]; then
    echo
    echo "I2C for the IMU was just turned on: reboot the Pi once (sudo reboot), then check the IMU with:"
    echo "    bash ~/wearable_ws/src/visionnav/scripts/setup_pi.sh imu"
fi
