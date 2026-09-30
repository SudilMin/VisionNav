#!/bin/bash
# VisionNav — which button program is the Pi really running? Run FROM THE LAPTOP (it is sent to the Pi, nothing
# is changed there):
#
#   ssh pi@raspberrypi.local bash -s < ~/wearable_ws/src/visionnav/scripts/pi_check.sh
#
# Shows the service, the file Python loads for it and what that version does on a hold, the Pi's git state and
# the last button log. A Pi still on an older build keeps the old buttons whatever was copied into its src folder
# (before 2026-09-30 a SENSORS hold RESTARTED the sensors and a tap switched them on and off).
echo "== service"
systemctl show visionnav-buttons -p ActiveState -p ActiveEnterTimestamp -p ExecStart 2>&1 | cut -c1-260
echo "== the code it runs"
source /opt/ros/jazzy/setup.bash
source ~/wearable_ws/install/setup.bash
python3 - <<'EOF'
import os
import visionnav.pi_button_panel as m
c = m.ButtonPanel
print("file:                        ", os.path.realpath(m.__file__))
print("SENSORS hold turns them off: ", not hasattr(c, "_sensors_toggle"))
print("MODE hold turns them off:    ", hasattr(c, "_park_lidar"))
EOF
echo "== git (the Pi's copy of the repository)"
cd ~/wearable_ws/src && git log --oneline -1 && git status --short | head -10
echo "== running"
pgrep -a -f "pi_button_panel|pi_sensors.launch|sllidar_node" | cut -c1-160
echo "== last button log"
journalctl -u visionnav-buttons -n 40 --no-pager -o cat 2>&1 | cut -c1-200
