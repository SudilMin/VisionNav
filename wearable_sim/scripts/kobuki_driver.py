#!/usr/bin/env python3
"""
kobuki_driver.py
================
ROS 2 node that bridges a physical Kobuki robot over its serial port.

Based on the kobuki-python library from https://github.com/SudilMin/kobuki-python
which provides the Kobuki serial packet structure. This node adds the missing
implementations: velocity command sending and encoder-based odometry.

Subscribes:
  /cmd_vel  (geometry_msgs/Twist) → converts to Kobuki BaseControl serial command

Publishes:
  /odom     (nav_msgs/Odometry)   → wheel encoder odometry
  TF        odom → base_footprint  → so Nav2 / semantic_navigator can localize

Usage:
  ros2 run diff_drive_robot kobuki_driver --ros-args -p serial_port:=/dev/ttyUSB0
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

try:
    import serial
except ImportError:
    raise ImportError("Run: pip3 install pyserial --break-system-packages")


# ── Kobuki Hardware Constants ──────────────────────────────────────────────────
WHEEL_DIAMETER  = 0.070          # 70 mm
WHEEL_SEP       = 0.230          # 230 mm between wheels  
TICKS_PER_REV   = 2578.33        # encoder ticks per wheel revolution
TICKS_PER_METER = TICKS_PER_REV / (math.pi * WHEEL_DIAMETER)   # ≈ 11724 ticks/m
MAX_SPEED_MM    = 500            # hardware max: 500 mm/s

# ── Kobuki Serial Protocol ─────────────────────────────────────────────────────
HEADER_0        = 0xAA
HEADER_1        = 0x55
CMD_BASE_CTRL   = 0x01           # BaseControl sub-payload ID
CMD_BASE_LEN    = 0x04           # 4 bytes: speed(int16) + radius(int16)
FEEDBACK_BASIC  = 0x01           # BasicSensorData sub-payload ID (feedback)

STRAIGHT_RADIUS = 0x7FFF         # special value = no curve, go straight


def _checksum(payload: bytes) -> int:
    cs = len(payload)
    for b in payload:
        cs ^= b
    return cs & 0xFF


def _build_velocity_packet(linear_x: float, angular_z: float) -> bytes:
    """
    Convert Twist to Kobuki BaseControl serial packet.
    Protocol from kobuki-python / kobuki.readthedocs.io
    """
    v = linear_x           # m/s
    w = angular_z          # rad/s

    if abs(v) < 0.001 and abs(w) < 0.001:
        speed_mm  = 0
        radius_mm = 0
    elif abs(v) < 0.001:          # pure rotation in place
        speed_mm  = int(abs(w) * (WHEEL_SEP / 2.0) * 1000.0)
        radius_mm = 1 if w > 0 else -1
    elif abs(w) < 0.001:          # straight line
        speed_mm  = int(v * 1000.0)
        radius_mm = STRAIGHT_RADIUS
    else:                          # curved motion
        radius_m  = v / w
        speed_mm  = int(v * 1000.0)
        radius_mm = int(radius_m * 1000.0)

    # Clamp speed to hardware limit
    speed_mm = max(-MAX_SPEED_MM, min(MAX_SPEED_MM, speed_mm))
    # Clamp radius to int16 range
    radius_mm = max(-32768, min(32767, radius_mm))

    sub_payload = struct.pack('<BBhh', CMD_BASE_CTRL, CMD_BASE_LEN,
                              speed_mm, radius_mm)
    cs = _checksum(sub_payload)
    packet = bytes([HEADER_0, HEADER_1, len(sub_payload)]) + sub_payload + bytes([cs])
    return packet


def _int16_from_bytes(lo: int, hi: int) -> int:
    val = (hi << 8) | lo
    if val >= 0x8000:
        val -= 0x10000
    return val


def _uint16_from_bytes(lo: int, hi: int) -> int:
    return (hi << 8) | lo


class KobukiDriver(Node):
    def __init__(self):
        super().__init__('kobuki_driver')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        port = self.get_parameter('serial_port').get_parameter_value().string_value

        # ── State ─────────────────────────────────────────────────────────────
        self.x    = 0.0
        self.y    = 0.0
        self.yaw  = 0.0
        self.prev_left  = None
        self.prev_right = None
        self._lock = threading.Lock()

        # ── Serial connection (kobuki-python style: 115200 baud) ──────────────
        try:
            self.ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
            self.get_logger().info(f'Kobuki connected on {port}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            self.get_logger().fatal(
                'Tip: check port with "ls /dev/ttyUSB* /dev/ttyACM*"')
            raise

        # ── ROS interfaces ────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_bcast = tf2_ros.TransformBroadcaster(self)

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, best_effort_qos)

        # ── Background thread: continuously reads sensor feedback ─────────────
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._read_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            'KobukiDriver ready.\n'
            '  Subscribing to /cmd_vel\n'
            '  Publishing  /odom + TF(odom→base_footprint)')

    # ── cmd_vel callback ───────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        packet = _build_velocity_packet(msg.linear.x, msg.angular.z)
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial read loop (runs in background thread) ───────────────────────────
    def _read_loop(self):
        """Reads Kobuki feedback packets and extracts encoder data."""
        buf = bytearray()
        last_data_time = time.time()
        
        while self._running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buf += data
                    last_data_time = time.time()
                elif time.time() - last_data_time > 2.0:
                    self.get_logger().warn("No data received from Kobuki! Is the power switch turned ON?", throttle_duration_sec=5.0)
            except Exception:
                time.sleep(0.01)
                continue

            # Scan for 0xAA 0x55 header (kobuki-python protocol)
            while len(buf) >= 4:
                if buf[0] != HEADER_0 or buf[1] != HEADER_1:
                    buf.pop(0)
                    continue

                payload_len = buf[2]
                total = 3 + payload_len + 1       # header(3) + payload + checksum
                if len(buf) < total:
                    break

                payload  = buf[3:3 + payload_len]
                checksum = buf[3 + payload_len]

                if _checksum(payload) != checksum:
                    buf.pop(0)
                    continue

                # Valid packet — parse sub-payloads
                self._parse_feedback(bytes(payload))
                buf = buf[total:]

    def _parse_feedback(self, payload: bytes):
        """
        Parse Kobuki feedback sub-payloads.
        SubPayloadSchemas from kobuki-python defines the structure.
        BasicSensorData (ID=0x01) contains LeftEncoder + RightEncoder.
        """
        i = 0
        while i + 1 < len(payload):
            sub_id  = payload[i]
            sub_len = payload[i + 1]
            data    = payload[i + 2: i + 2 + sub_len]
            i      += 2 + sub_len

            if sub_id == FEEDBACK_BASIC and len(data) >= 15:
                # BasicSensorData layout:
                # [0-1]  TimeStamp   UShort
                # [2]    Bumper      Flag1B
                # [3]    WheelDrop   Flag1B
                # [4]    Cliff       Flag1B
                # [5-6]  LeftEncoder UShort
                # [7-8]  RightEncoder UShort
                # ... rest (PWM, Button, Charger, Battery, OverCurrent)
                left_enc  = _uint16_from_bytes(data[5], data[6])
                right_enc = _uint16_from_bytes(data[7], data[8])

                bumper = data[2]
                if bumper:
                    # Stop on bumper hit for safety
                    self.ser.write(_build_velocity_packet(0.0, 0.0))

                self._update_odom(left_enc, right_enc)

    def _update_odom(self, left_enc: int, right_enc: int):
        now = self.get_clock().now()

        with self._lock:
            if self.prev_left is None:
                self.prev_left  = left_enc
                self.prev_right = right_enc
                return

            # Handle 16-bit rollover (kobuki encoder wraps at 65535)
            dl = (left_enc  - self.prev_left)  & 0xFFFF
            dr = (right_enc - self.prev_right) & 0xFFFF
            if dl > 32767: dl -= 65536
            if dr > 32767: dr -= 65536

            self.prev_left  = left_enc
            self.prev_right = right_enc

            dl_m = dl / TICKS_PER_METER      # metres
            dr_m = dr / TICKS_PER_METER

            dc     = (dl_m + dr_m) / 2.0    # centre displacement
            dtheta = (dr_m - dl_m) / WHEEL_SEP

            self.x   += dc * math.cos(self.yaw + dtheta / 2.0)
            self.y   += dc * math.sin(self.yaw + dtheta / 2.0)
            self.yaw += dtheta

            # Wrap yaw to [-π, π]
            while self.yaw >  math.pi: self.yaw -= 2 * math.pi
            while self.yaw < -math.pi: self.yaw += 2 * math.pi

        # ── Publish /odom ──────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.odom_pub.publish(odom)

        # ── Broadcast TF: odom → base_footprint ───────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_footprint'
        tf_msg.transform.translation.x  = self.x
        tf_msg.transform.translation.y  = self.y
        tf_msg.transform.rotation.z     = math.sin(self.yaw / 2.0)
        tf_msg.transform.rotation.w     = math.cos(self.yaw / 2.0)
        self.tf_bcast.sendTransform(tf_msg)

    def destroy_node(self):
        self._running = False
        # Send stop command before shutdown
        try:
            self.ser.write(_build_velocity_packet(0.0, 0.0))
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KobukiDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[kobuki_driver] Fatal: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
