#!/usr/bin/env python3
"""
map_manager.py
==============
Remembers the home between sessions: the SLAM map, the named places, and (via object_perception)
the mapped objects.

Files, per map name (default "home"), in ~/.visionnav/maps (override: VISIONNAV_MAPS_DIR):
  <name>.pbstream       Cartographer state; laptop_brain.launch.py loads it and localizes in it
  <name>_objects.json   reliable static objects (written/read by object_perception)
  <name>_places.json    named places {"kitchen": {"x": .., "y": ..}}

Commands on /map_command (std_msgs/String):
  "save"            save the map (mapping mode only) and ask object_perception to save its objects
  "place <name>"    remember the wearer's current position as <name>
  "forget <name>"   forget a named place
Results are published on /map_command_result (spoken by voice_navigation_assistant).

Publishes (latched): /active_map  JSON {"name", "mode": "mapping"|"localization", "dir"}
                     /named_places JSON {"kitchen": {"x": .., "y": ..}, ...}
"""

import json
import os

import rclpy
import tf2_ros
from cartographer_ros_msgs.srv import WriteState
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
        self._mode = self.declare_parameter('mode', 'mapping').value
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
        self._write_state = self.create_client(WriteState, '/write_state')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._active_pub.publish(String(data=json.dumps(
            {'name': self._name, 'mode': self._mode, 'dir': self._dir})))
        self._publish_places()
        self.get_logger().info(f"Map '{self._name}' ({self._mode}) in {self._dir}; "
                               f"{len(self._places)} named places")

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
        if verb == 'save':
            self._save_map()
        elif verb == 'place' and arg:
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

    def _save_map(self):
        # object_perception saves the objects on the same /map_command "save"
        if self._mode != 'mapping':
            self._reply("Objects and places saved. The map itself is kept as it is while localizing; "
                        "start with localize:=false to map again.")
            return
        if not self._write_state.service_is_ready():
            self._reply("The map cannot be saved: Cartographer is not running.")
            return
        req = WriteState.Request()
        req.filename = os.path.join(self._dir, f"{self._name}.pbstream")
        req.include_unfinished_submaps = True
        self._write_state.call_async(req).add_done_callback(self._saved)

    def _saved(self, future):
        try:
            status = future.result().status
        except Exception as e:
            self._reply(f"Saving the map failed: {e}")
            return
        if status.code == 0:
            self._reply(f"Map {self._name} saved. Next time I will find you in it.")
        else:
            self._reply(f"Saving the map failed: {status.message}")


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
