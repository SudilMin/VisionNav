#!/usr/bin/env python3
"""
map_manager.py
==============
Named places of the current indoor session ("save this place as kitchen", "go to the kitchen").

The session folder comes from VISIONNAV_MAPS_DIR: system_manager.py creates an empty temporary folder for every
indoor session and deletes it when the map stops, so nothing is kept for another day.
  <name>_places.json    named places {"kitchen": {"x": .., "y": ..}}

Commands on /map_command (std_msgs/String):
  "place <name>"    remember the wearer's current position as <name>
  "forget <name>"   forget a named place
Results are published on /map_command_result (spoken by voice_navigation_assistant).

Publishes (latched): /active_map  JSON {"name", "dir"} (the assistant keeps the wearer's object names there)
                     /named_places JSON {"kitchen": {"x": .., "y": ..}, ...}
"""

import json
import os

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String


def maps_dir() -> str:
    return os.path.expanduser(os.environ.get("VISIONNAV_MAPS_DIR", "~/.visionnav/maps"))


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        self._name = self.declare_parameter('map_name', 'home').value
        self._dir = maps_dir()
        os.makedirs(self._dir, exist_ok=True)
        self._places_file = os.path.join(self._dir, f"{self._name}_places.json")
        self._places = self._load_places()

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._active_pub = self.create_publisher(String, '/active_map', latched)
        self._places_pub = self.create_publisher(String, '/named_places', latched)
        self._result_pub = self.create_publisher(String, '/map_command_result', 10)
        self.create_subscription(String, '/map_command', self._command_cb, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._active_pub.publish(String(data=json.dumps({'name': self._name, 'dir': self._dir})))
        self._publish_places()
        self.get_logger().info(f"Session '{self._name}' in {self._dir}; {len(self._places)} named places")

    # ── places ──
    def _load_places(self) -> dict:
        try:
            with open(self._places_file) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def _publish_places(self):
        self._places_pub.publish(String(data=json.dumps(self._places)))

    def _save_places(self):
        with open(self._places_file, 'w') as f:
            json.dump(self._places, f, indent=1)
        self._publish_places()

    def _pose(self):
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return None
        return round(tf.transform.translation.x, 3), round(tf.transform.translation.y, 3)

    # ── commands ──
    def _reply(self, text: str):
        self.get_logger().info(text)
        self._result_pub.publish(String(data=text))

    def _command_cb(self, msg: String):
        words = msg.data.strip().lower().split(maxsplit=1)
        if not words:
            return
        verb, arg = words[0], (words[1].strip() if len(words) > 1 else '')
        if verb == 'place' and arg:
            pose = self._pose()
            if pose is None:
                self._reply("I don't know where you are yet, so I can't save that place.")
                return
            self._places[arg] = {'x': pose[0], 'y': pose[1]}
            self._save_places()
            self._reply(f"Saved this place as {arg}.")
        elif verb == 'forget' and arg:
            if self._places.pop(arg, None) is None:
                self._reply(f"There is no place called {arg}.")
                return
            self._save_places()
            self._reply(f"Forgot {arg}.")


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
