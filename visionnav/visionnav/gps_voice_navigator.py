#!/usr/bin/env python3
"""
gps_voice_navigator.py - Offline GPS Navigation for Outdoor Mode
Subscribes to /gps/fix (NavSatFix) and provides:
  1. Heading direction to a GPS waypoint
  2. Distance remaining
  3. Voice guidance: "Walk North-East for 150 meters"
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geopy.distance import geodesic
import math

class GPSNavNode(Node):
    def __init__(self):
        super().__init__('gps_voice_navigator')
        self._gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self._gps_callback, 10)
        self._cmd_sub = self.create_subscription(String, '/gps_destination', self._dest_callback, 10)
        self._guidance_pub = self.create_publisher(String, '/gps_guidance', 10)

        self._current_lat = None
        self._current_lon = None
        self._dest_lat = None
        self._dest_lon = None
        
        self.get_logger().info("GPS Navigation Node Started.")

    def _gps_callback(self, msg):
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude
        if self._dest_lat is not None:
            self._calculate_guidance()

    def _dest_callback(self, msg):
        # Format: "lat,lon" e.g. "6.7952,79.9009"
        try:
            parts = msg.data.split(',')
            self._dest_lat = float(parts[0])
            self._dest_lon = float(parts[1])
            self.get_logger().info(f"Received new destination: {self._dest_lat}, {self._dest_lon}")
        except Exception as e:
            self.get_logger().error(f"Invalid GPS destination format: {msg.data}")

    def _calculate_guidance(self):
        dist = geodesic(
            (self._current_lat, self._current_lon),
            (self._dest_lat, self._dest_lon)
        ).meters

        # Calculate bearing
        dlon = math.radians(self._dest_lon - self._current_lon)
        lat1 = math.radians(self._current_lat)
        lat2 = math.radians(self._dest_lat)
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(x, y))

        # Convert bearing to compass direction
        dirs = ['North', 'North-East', 'East', 'South-East',
                'South', 'South-West', 'West', 'North-West']
        idx = int(((bearing + 22.5) % 360) / 45)
        direction = dirs[idx]

        if dist < 5.0:
            guidance = "You have arrived at your destination."
            self._dest_lat = None # clear destination
        else:
            guidance = f"Walk {direction} for {int(dist)} meters."

        self._guidance_pub.publish(String(data=guidance))
        self.get_logger().info(f"Guidance: {guidance}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

