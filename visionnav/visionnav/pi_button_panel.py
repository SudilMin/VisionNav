#!/usr/bin/env python3
"""
pi_button_panel.py
==================
Five push buttons on the Raspberry Pi 5 header, for a user who cannot see a screen. Runs on the Pi, started at
boot by the visionnav-buttons systemd service (see STARTUP_INSTRUCTIONS.md), so no Pi command is needed.

Each button is wired between a GPIO pin and GND (no resistors: the Pi's internal pull-ups are used, so the
pin reads 1 when released and 0 when pressed). Default pins (BCM numbering / physical header pin):

  SENSORS GPIO24 / pin 18  tap: turn the LiDAR and camera on / off (this node starts pi_sensors.launch.py)
                           hold: restart them (a camera that stopped sending)
  LOOK    GPIO17 / pin 11  tap: describe what is in front (Qwen3-VL)   hold: ask the camera a question (speak)
  MODE    GPIO27 / pin 13  tap: switch indoor <-> outdoor              hold: status (mode, map, what is around)
  HAND    GPIO22 / pin 15  tap: guide the hand to the object found last (walks there first if it is far);
                                tap again to stop
  TALK    GPIO23 / pin 16  hold: speak a command ("find the table with the cup")   tap: STOP everything
                           double tap: what is around me
  GND     pin 14 (or 9, 20, 25) shared by all buttons

Publishes /button_event (std_msgs/String, JSON): {"button": "sensors"|"look"|"mode"|"hand"|"talk",
"event": "tap"|"double"|"hold_start"|"hold_end"}, and /pi_sensors_state (latched String: "off", "starting",
"on", "stopping", "failed"). voice_navigation_assistant (on the laptop) acts on the events and speaks the
results. A tap is published the moment the button is released (STOP must not wait for a possible second tap);
a "double" follows when a second tap comes within DOUBLE_TAP_S.

Parameters: <button>_pin, hold_time, double_tap_time, sensors_at_start (turn the sensors on at boot).
Needs gpiozero with the lgpio backend (RPi.GPIO does not work on the Pi 5).
Test: ros2 topic echo /button_event
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from visionnav.system_manager import SystemManager

BUTTONS = ("sensors", "look", "mode", "hand", "talk")
DEFAULT_PINS = {"sensors": 24, "look": 17, "mode": 27, "hand": 22, "talk": 23}
HOLD_S = 0.6          # pressed this long: a hold (push-to-talk, status) instead of a tap
DOUBLE_TAP_S = 0.4    # second tap within this: also a "double"
BOUNCE_S = 0.03       # contact bounce filter


class ButtonPanel(Node):
    def __init__(self, pin_factory=None):
        super().__init__("pi_button_panel")
        self._pub = self.create_publisher(String, "/button_event", 10)
        self._state_pub = self.create_publisher(
            String, "/pi_sensors_state",
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
        self._set_state("off")
        try:
            from gpiozero import Button
        except ImportError:
            Button = None
            self.get_logger().error("gpiozero is not installed. Raspberry Pi OS: it is preinstalled; "
                                    "Ubuntu: sudo apt install python3-gpiozero python3-lgpio")
        for name, pin in pins.items() if Button else ():
            try:
                kw = {"pin_factory": pin_factory} if pin_factory is not None else {}
                btn = Button(pin, pull_up=True, bounce_time=BOUNCE_S, hold_time=self._hold_s, **kw)
            except Exception as e:  # wrong pin, no permission on /dev/gpiochip*, pin already in use
                self.get_logger().error(f"{name.upper()} button on GPIO{pin} unavailable: {e}")
                continue
            btn.when_held = lambda n=name: self._on_held(n)
            btn.when_released = lambda n=name: self._on_released(n)
            self._buttons[name] = btn
        self.get_logger().info("Buttons ready: " + ", ".join(f"{n.upper()}=GPIO{pins[n]}" for n in self._buttons)
                               + f" (hold {self._hold_s:.1f} s, double tap {self._double_s:.1f} s)")
        if sensors_at_start:
            threading.Thread(target=self._sensors_on, daemon=True).start()

    # ── events ──
    def _emit(self, button, event):
        self.get_logger().info(f"{button.upper()} {event}")
        self._pub.publish(String(data=json.dumps({"button": button, "event": event, "t": round(time.time(), 3)})))

    def _on_held(self, name):
        self._held[name] = True
        self._emit(name, "hold_start")
        if name == "sensors":
            threading.Thread(target=self._sensors_restart, daemon=True).start()

    def _on_released(self, name):
        if self._held[name]:
            self._held[name] = False
            self._emit(name, "hold_end")
            return
        self._emit(name, "tap")
        if name == "sensors":
            threading.Thread(target=self._sensors_toggle, daemon=True).start()
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

    def _sensors_toggle(self):
        if self._sensor_state in ("starting", "stopping"):
            return  # busy; the laptop already said what is happening
        if self._sys.owned("pi_sensors") or self._sys.running("pi_sensors"):
            self._sensors_off()
        else:
            self._sensors_on()

    def _sensors_on(self):
        with self._sensor_lock:
            self._set_state("starting")
            self._set_state("on" if self._sys.start("pi_sensors") else "failed")

    def _sensors_off(self):
        with self._sensor_lock:
            self._set_state("stopping")
            if not self._sys.stop("pi_sensors") and self._sys.running("pi_sensors"):
                # Started in a terminal, not by this node: leave it, and say so
                self.get_logger().warn("The sensors were started in a terminal; stop them there.")
                self._set_state("on")
                return
            self._set_state("off")

    def _sensors_restart(self):
        if self._sys.owned("pi_sensors"):
            self._sensors_off()
        self._sensors_on()

    def destroy_node(self):
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
