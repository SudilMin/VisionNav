#!/usr/bin/env python3
"""
scan_body_filter.py
===================
Removes the wearer's own body from the chest LiDAR scan before it reaches SLAM.

A 360° LiDAR on the chest sees the user's torso behind it and their arms at the sides.
Those returns move rigidly with the sensor, so the scan matcher treats them as a
perfectly static landmark and is pulled toward "no motion" — the main cause of the map
smearing and the pose lagging while walking.

This node drops (sets to NaN, i.e. "invalid" per REP-117):
  * returns closer than `min_range` (torso, swinging arms, straps)
  * returns outside the forward `keep_fov_deg` wedge measured in base_footprint,
    which is where the body blocks the view anyway

Subscribes: /scan            Publishes: /scan_filtered
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class ScanBodyFilter(Node):
    def __init__(self):
        super().__init__('scan_body_filter')
        self._min_range = self.declare_parameter('min_range', 0.45).value
        self._keep_fov = math.radians(self.declare_parameter('keep_fov_deg', 220.0).value)
        self._base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
        self._sub = self.create_subscription(LaserScan, '/scan', self._callback, qos_profile_sensor_data)

        self._mask_key = None
        self._keep_mask = None
        self._warned_tf = False
        self.get_logger().info(
            f'Body filter active: min_range={self._min_range:.2f} m, '
            f'forward FOV={math.degrees(self._keep_fov):.0f}°'
        )

    def _build_mask(self, msg: LaserScan):
        """Per-beam keep mask from each beam's bearing in base_footprint (handles any mount yaw/roll)."""
        tf = self._tf_buffer.lookup_transform(self._base_frame, msg.header.frame_id, Time())
        q = tf.transform.rotation
        # First two rows of the rotation matrix are enough to rotate in-plane beam directions.
        r00 = 1 - 2 * (q.y * q.y + q.z * q.z)
        r01 = 2 * (q.x * q.y - q.z * q.w)
        r10 = 2 * (q.x * q.y + q.z * q.w)
        r11 = 1 - 2 * (q.x * q.x + q.z * q.z)
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        c, s = np.cos(angles), np.sin(angles)
        bearing = np.arctan2(r10 * c + r11 * s, r00 * c + r01 * s)
        return np.abs(bearing) <= self._keep_fov / 2.0

    def _callback(self, msg: LaserScan):
        key = (msg.header.frame_id, len(msg.ranges), round(msg.angle_min, 5), round(msg.angle_increment, 7))
        if key != self._mask_key:
            try:
                self._keep_mask = self._build_mask(msg)
                self._mask_key = key
            except Exception as e:
                if not self._warned_tf:
                    self.get_logger().warn(f'Waiting for TF {self._base_frame} <- {msg.header.frame_id}: {e}')
                    self._warned_tf = True
                return

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        drop = ~self._keep_mask | (ranges < self._min_range)
        ranges[drop] = np.nan
        msg.ranges = ranges.tolist()
        if len(msg.intensities) == len(ranges):
            intensities = np.asarray(msg.intensities, dtype=np.float32)
            intensities[drop] = 0.0
            msg.intensities = intensities.tolist()
        msg.range_min = max(msg.range_min, float(self._min_range))
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBodyFilter()
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
