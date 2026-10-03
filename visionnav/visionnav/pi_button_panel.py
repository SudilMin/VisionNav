#!/usr/bin/env python3
"""
pi_button_panel.py
==================
Five push buttons on the Raspberry Pi 5 header, for a user who cannot see a screen. Runs on the Pi, started at
boot by the visionnav-buttons systemd service (see STARTUP_INSTRUCTIONS.md), so no Pi command is needed.

Each button is wired between a GPIO pin and GND (no resistors: the Pi's internal pull-ups are used, so the
pin reads 1 when released and 0 when pressed). Default pins (BCM numbering / physical header pin):

  SENSORS GPIO24 / pin 18  tap: turn the LiDAR, camera and IMU on (this node starts pi_sensors.launch.py);
                                already on: said again; one of them stopped (camera unplugged): all restarted
                           hold: turn them off, at once, while the button is still down (no laptop needed)
  LOOK    GPIO17 / pin 11  tap: describe what is in front (Qwen3-VL)   hold: ask the camera a question (speak)
                           double tap: the vision AI off (frees its GPU memory)
  MODE    GPIO27 / pin 13  tap: switch indoor <-> outdoor
                           hold: the laptop closes the map and the camera feed (the sensors stay on)
  HAND    GPIO22 / pin 15  tap: guide the hand to the object found last (walks there first if it is far);
                                tap again to stop        hold: hand guidance off
                           double tap: face mode (then tap: who is it; hold: say a name to remember the face)
  TALK    GPIO23 / pin 16  hold (indoor mode): say where to go, the laptop guides you there
                           tap: end the navigation (otherwise nothing)   double tap: what is around me
  GND     pin 14 (or 9, 20, 25) shared by all buttons

Publishes /button_event (std_msgs/String, JSON): {"button": "sensors"|"look"|"mode"|"hand"|"talk",
"event": "tap"|"double"|"hold_start"|"hold_end"}, and /pi_sensors_state (latched String: "off", "starting",
"on", "stopping", "failed"). voice_navigation_assistant (on the laptop) acts on the events and speaks the
results. A tap is published the moment the button is released (it must not wait for a possible second tap);
a "double" follows when a second tap comes within DOUBLE_TAP_S. A hold is acted on when it is recognised
(HOLD_S after the press), not at the release.

The pins are read by this node itself, 200 times a second (_watch_buttons): a press counts after two readings
with contact, a release only after RELEASE_S without contact. With the kernel's debounce filter (gpiozero's
bounce_time) a press had to stay perfectly steady for 30 ms before it was reported at all, and on the rig short
presses of a button with a poor contact were lost: the wearer had to keep it down to get anything.

Parameters: <button>_pin, hold_time, double_tap_time, sensors_at_start (turn the sensors on at boot).
Needs gpiozero with the lgpio backend (RPi.GPIO does not work on the Pi 5).
Test: ros2 topic echo /button_event
"""

import json
import os
import subprocess
import termios
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from visionnav.system_manager import SystemManager

BUTTONS = ("sensors", "look", "mode", "hand", "talk")
DEFAULT_PINS = {"sensors": 24, "look": 17, "mode": 27, "hand": 22, "talk": 23}
HOLD_S = 0.6          # pressed this long: a hold (push-to-talk, off) instead of a tap
DOUBLE_TAP_S = 0.4    # second tap within this: also a "double"
POLL_S = 0.005        # the pins are read this often
PRESS_S = 0.004       # contact this long (two readings in a row): pressed. Short, so that a quick press
                      # through a poor contact counts
RELEASE_S = 0.060     # no contact this long: released (breaks inside one press are not a release)
AFTER_HOLD_S = 1.0    # after a SENSORS hold ("off"), SENSORS taps are ignored this long: letting go of the button
                      # must not turn the sensors on again
# The LiDAR's motor, stopped without its driver (_park_lidar). Port and speed as in pi_sensors.launch.py (RPLiDAR
# C1); the bytes are what the driver itself sends on a clean exit (sllidar SDK: setMotorSpeed(0) = command A8 with
# a 2-byte speed and an XOR checksum, then stop() = command 25)
LIDAR_PORT, LIDAR_BAUD = "/dev/ttyUSB0", 460800
LIDAR_MOTOR_OFF, LIDAR_STOP = bytes([0xA5, 0xA8, 0x02, 0x00, 0x00, 0x0F]), bytes([0xA5, 0x25])


class ButtonPanel(Node):
    def __init__(self, pin_factory=None):
        super().__init__("pi_button_panel")
        self._pub = self.create_publisher(String, "/button_event", 10)
        self._state_pub = self.create_publisher(
            String, "/pi_sensors_state",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Whether the buttons could be opened (latched): the laptop says so aloud if they could not
        self._status_pub = self.create_publisher(
            String, "/pi_button_status",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._hold_s = float(self.declare_parameter("hold_time", HOLD_S).value)
        self._double_s = float(self.declare_parameter("double_tap_time", DOUBLE_TAP_S).value)
        sensors_at_start = bool(self.declare_parameter("sensors_at_start", False).value)
        pins = {b: int(self.declare_parameter(f"{b}_pin", DEFAULT_PINS[b]).value) for b in BUTTONS}
        self._sys = SystemManager(self, log=lambda m: self.get_logger().info(m))
        self._sensor_lock = threading.Lock()
        self._buttons = {}
        self._held = {b: False for b in BUTTONS}
        self._last_tap = {b: 0.0 for b in BUTTONS}
        self._off_hold_end = -1e9  # when the last SENSORS hold was let go (the hold is still on: inf)
        self._closing = False
        self._set_state("off")
        try:
            from gpiozero import InputDevice
        except ImportError:
            InputDevice = None
            self.get_logger().error("gpiozero is not installed. Raspberry Pi OS: it is preinstalled; "
                                    "Ubuntu: sudo apt install python3-gpiozero python3-lgpio")
        for name, pin in pins.items() if InputDevice else ():
            try:
                kw = {"pin_factory": pin_factory} if pin_factory is not None else {}
                dev = InputDevice(pin, pull_up=True, **kw)  # active = pressed (pin at GND)
                if dev.is_active:
                    self.get_logger().warn(f"{name.upper()} button on GPIO{pin} reads as pressed at start: "
                                           f"its wire may touch GND")
                self._buttons[name] = dev
            except Exception as e:  # wrong pin, no permission on /dev/gpiochip*, pin already in use
                self.get_logger().error(f"{name.upper()} button on GPIO{pin} unavailable: {e}")
        if self._buttons:
            threading.Thread(target=self._watch_buttons, daemon=True).start()
        missing = [n for n in BUTTONS if n not in self._buttons]
        if self._buttons:
            self.get_logger().info("Buttons ready: " + ", ".join(f"{n.upper()}=GPIO{pins[n]}" for n in self._buttons)
                                   + f" (hold {self._hold_s:.1f} s, double tap {self._double_s:.1f} s)")
        if missing:
            self.get_logger().error(f"Buttons NOT working: {', '.join(m.upper() for m in missing)} (see the errors above)")
        self._status_pub.publish(String(data="ok" if not missing else "failed: " + " ".join(missing)))
        if sensors_at_start:
            threading.Thread(target=self._sensors_on, daemon=True).start()
        else:
            threading.Thread(target=self._park_at_start, daemon=True).start()

    # ── events ──
    def _emit(self, button, event):
        self.get_logger().info(f"{button.upper()} {event}")
        self._pub.publish(String(data=json.dumps({"button": button, "event": event, "t": round(time.time(), 3)})))

    def _watch_buttons(self):
        """Read the pins and turn them into presses: a hold the moment the button has been down for hold_time
        (still down), a tap when it is let go before that."""
        down = {b: False for b in self._buttons}      # pressed, as decided here
        since = {b: None for b in self._buttons}      # when the pin last changed to the other level
        pressed_at = {b: 0.0 for b in self._buttons}
        raw, raw_logged = None, []  # diag: every change of the SENSORS pin, at most 10 lines a second
        while not self._closing:
            now = time.monotonic()
            for name, dev in self._buttons.items():
                try:
                    contact = dev.is_active
                except Exception:  # the device is being closed
                    continue
                if name == "sensors" and contact != raw:
                    raw = contact
                    raw_logged = [t for t in raw_logged if now - t < 1.0]
                    if len(raw_logged) < 10:
                        raw_logged.append(now)
                        self.get_logger().info(f"diag: SENSORS pin {'contact' if contact else 'open'} "
                                               f"(sensors {self._sensor_state})")
                if contact == down[name]:
                    since[name] = None
                elif since[name] is None:
                    since[name] = now
                elif now - since[name] >= (RELEASE_S if down[name] else PRESS_S):
                    down[name] = contact
                    if contact:
                        pressed_at[name] = since[name]
                    else:
                        self._fire(self._on_released, name)
                    since[name] = None
                # A hold: down for hold_time and in contact now (not let go a moment ago). Short breaks in the
                # contact do not restart the count: it is still the same press
                if down[name] and contact and not self._held[name] and now - pressed_at[name] >= self._hold_s:
                    self._fire(self._on_held, name)
            time.sleep(POLL_S)

    def _fire(self, handler, name):
        try:
            handler(name)
        except Exception as e:  # never stop reading the buttons
            self.get_logger().error(f"{name.upper()} button: {e}")

    def _on_held(self, name):
        self._held[name] = True
        self._emit(name, "hold_start")
        if name == "sensors":  # off, now, with the button still down
            self._off_hold_end = float("inf")
            threading.Thread(target=self._sensors_off, daemon=True).start()

    def _on_released(self, name):
        if self._held[name]:
            self._held[name] = False
            self._emit(name, "hold_end")
            if name == "sensors":
                self._off_hold_end = time.monotonic()
            return
        self._emit(name, "tap")
        if name == "sensors" and time.monotonic() - self._off_hold_end >= AFTER_HOLD_S:
            threading.Thread(target=self._sensors_on, daemon=True).start()
        now = time.monotonic()
        if now - self._last_tap[name] <= self._double_s:
            self._emit(name, "double")
            self._last_tap[name] = 0.0
        else:
            self._last_tap[name] = now

    # ── LiDAR + camera (SENSORS button) ──
    def _set_state(self, state):
        self._sensor_state = state
        self.get_logger().info(f"sensors: {state}")
        self._state_pub.publish(String(data=state))
        if state in ("on", "off") and "sensors" in getattr(self, "_buttons", {}):
            self.get_logger().info("diag: " + self._pin_report("sensors"))

    def _pin_report(self, name):
        """diag: how the pin reads and who holds it (on the rig SENSORS presses were not seen while the sensors
        were on, the other buttons were)."""
        dev = self._buttons[name]
        try:
            import lgpio
            ok, gpio, flags, _, user = lgpio.gpio_get_line_info(dev.pin.factory._handle, dev.pin._number)
            info = f"GPIO{gpio} flags 0x{flags:x} user '{user}'"
        except Exception as e:
            info = f"no line info ({e})"
        return f"{name.upper()} pin reads {'contact' if dev.is_active else 'open'}, {info}"

    def _sensors_on(self):
        """SENSORS tap (and sensors_at_start). Already on: the state is published again, so the laptop says so."""
        if self._sensor_state in ("starting", "stopping"):
            return  # busy; the laptop already said what is happening
        if self._sys.running("pi_sensors"):
            self._set_state("on")
            return
        with self._sensor_lock:
            self._set_state("starting")
            # Started earlier but one sensor has stopped (the camera exits when it finds no camera): start all again
            self._sys.stop("pi_sensors")
            self._set_state("on" if self._sys.start("pi_sensors") else "failed")

    def _sensors_off(self):
        """SENSORS hold. While they are starting, waits for that to finish, then turns them off."""
        with self._sensor_lock:
            self._set_state("stopping")
            if not self._sys.stop("pi_sensors") and self._sys.running("pi_sensors"):
                # Started in a terminal, not by this node: leave it, and say so
                self.get_logger().warn("The sensors were started in a terminal; stop them there.")
                self._set_state("on")
                return
            self._park_lidar()
            self._set_state("off")

    def _park_at_start(self):
        """At start-up the sensors are off: also the LiDAR's motor, left spinning when this service was restarted
        with the sensors on."""
        with self._sensor_lock:
            self._park_lidar()  # not while a driver runs (sensors left on by an earlier session, or a tap just now)

    def _park_lidar(self):
        """Stop the LiDAR's motor directly, when no driver is running. The driver stops the motor only on a clean
        exit (Ctrl+C); ended any other way — this service restarted or stopped (systemd's SIGTERM makes the driver
        abort), a crash, a kill — it leaves the LiDAR spinning, and this node, with no driver of its own to stop,
        had nothing to turn off: holding SENSORS or MODE did nothing. Harmless when the motor is already still."""
        if subprocess.run(["pgrep", "-x", "sllidar_node"], stdout=subprocess.DEVNULL).returncode == 0:
            return  # a driver has the port (started in a terminal): the motor is its own
        try:
            fd = os.open(LIDAR_PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            self.get_logger().info(f"LiDAR motor not stopped: {LIDAR_PORT} not opened ({e.strerror})")
            return
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0  # raw: the bytes go out as they are
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = getattr(termios, f"B{LIDAR_BAUD}")
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            os.write(fd, LIDAR_MOTOR_OFF)
            time.sleep(0.02)
            os.write(fd, LIDAR_STOP)
            termios.tcdrain(fd)
            self.get_logger().info("LiDAR motor stopped")
        except (OSError, termios.error) as e:
            self.get_logger().warn(f"LiDAR motor not stopped: {e}")
        finally:
            os.close(fd)

    def destroy_node(self):
        self._closing = True
        time.sleep(3 * POLL_S)
        for btn in self._buttons.values():
            btn.close()
        self._sys.stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonPanel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
