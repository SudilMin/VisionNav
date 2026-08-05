#!/usr/bin/env python3
"""
esp32_bridge.py
---------------
ROS 2 interface bridging physical ESP32 tactile buttons and vibration haptics to VisionNav.
Listens over USB Serial (/dev/ttyUSB1, /dev/ttyACM0, etc.) or Wi-Fi UDP Port 9090.
When the pushbutton on GPIO 15 is pressed, it publishes to /describe_command to instantly trigger Ollama Moondream VLM scene descriptions.
"""

import os
import sys
import time
import threading
import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__("esp32_bridge")
        self._pub_describe = self.create_publisher(String, "/describe_command", 10)
        
        self.get_logger().info("🔌 VisionNav ESP32 Tactical Bridge initializing...")
        
        self._stop_event = threading.Event()
        self._serial_port = None
        
        # Start Serial Listener Thread
        if HAS_SERIAL:
            self._serial_thread = threading.Thread(target=self._serial_listener_loop, daemon=True)
            self._serial_thread.start()
        else:
            self.get_logger().warn("pyserial module not installed. Run: pip3 install --break-system-packages pyserial. Only UDP Wi-Fi mode will be active.")
            
        # Start UDP Wi-Fi Listener Thread
        self._udp_thread = threading.Thread(target=self._udp_listener_loop, daemon=True)
        self._udp_thread.start()
        
        self.get_logger().info("✅ ESP32 Bridge Ready! Waiting for tactile pushbutton triggers from D12 (GPIO 12)...")

    def _trigger_scene_describer(self, source_name):
        self.get_logger().info(f"⚡ Tactical Pushbutton Pressed via {source_name}! Triggering Ollama Moondream VLM...")
        msg = String()
        msg.data = "Describe what you see in this image in one detailed, natural sentence."
        self._pub_describe.publish(msg)
        
    def _serial_listener_loop(self):
        while not self._stop_event.is_set():
            if self._serial_port is None or not self._serial_port.is_open:
                ports = serial.tools.list_ports.comports()
                target_port = None
                for p in ports:
                    # Avoid /dev/ttyUSB0 since it is typically dedicated to the Slamtec RPLiDAR C1
                    if ("USB" in p.device or "ACM" in p.device) and p.device != "/dev/ttyUSB0":
                        target_port = p.device
                        break
                
                # If target_port found, open at 115200 baud
                if target_port:
                    try:
                        self._serial_port = serial.Serial(target_port, 115200, timeout=1)
                        self.get_logger().info(f"Connected to ESP32 via USB Serial on {target_port}")
                    except Exception:
                        time.sleep(2)
                else:
                    time.sleep(3)
                    continue
                    
            try:
                line = self._serial_port.readline().decode('utf-8', errors='ignore').strip()
                if line == "TRIGGER_DESCRIBE":
                    self._trigger_scene_describer(f"USB Serial ({self._serial_port.port})")
            except Exception:
                if self._serial_port:
                    try:
                        self._serial_port.close()
                    except Exception:
                        pass
                self._serial_port = None
                time.sleep(2)
                
    def _udp_listener_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", 9090))
            self.get_logger().info("Listening for ESP32 Wi-Fi UDP triggers on Port 9090")
        except Exception as e:
            self.get_logger().error(f"Failed to bind UDP port 9090: {e}")
            return
            
        while not self._stop_event.is_set():
            try:
                data, addr = sock.recvfrom(1024)
                msg = data.decode('utf-8', errors='ignore').strip()
                if msg == "TRIGGER_DESCRIBE":
                    self._trigger_scene_describer(f"Wi-Fi UDP ({addr[0]})")
            except Exception:
                time.sleep(0.5)

    def destroy_node(self):
        self._stop_event.set()
        if self._serial_port and self._serial_port.is_open:
            self._serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
