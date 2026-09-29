#!/usr/bin/env python3
"""
mpu6050_imu.py
==============
The chest rig's MPU-6050 (GY-521 board) on the Raspberry Pi 5's I2C bus: publishes sensor_msgs/Imu on /imu/data
at 100 Hz, frame imu_link. Runs on the Pi, started with the LiDAR and camera by pi_sensors.launch.py.

What it is for: the gyroscope measures how fast the wearer turns, 100 times a second (the LiDAR scans only 10
times), and the accelerometer where down is. Indoors Cartographer uses both to predict each scan's rotation (fast
torso turns) and to level the scan when the chest leans; outdoors the LiDAR odometry (lidar_odometry.py) takes
the gyro's turn between two scans as its rotation guess.

Wiring (GY-521 -> Pi 5 header, Pi switched off; see STARTUP_INSTRUCTIONS.md section 3):
  VCC -> pin 1 (3.3 V)   GND -> pin 9   SDA -> pin 3 (GPIO2)   SCL -> pin 5 (GPIO3)
  XDA, XCL, AD0, INT not connected (AD0 open = address 0x68)

Data: angular velocity (rad/s) and linear acceleration (m/s^2, +9.8 upward at rest, REP-145) in the chip's own
axes (the X and Y arrows printed on the board, Z out of the chip side). How the board sits on the rig is the
base_footprint -> imu_link TF (sensor_tf.launch.py imu_* arguments; `calibrate` below measures them).
Orientation: roll and pitch from gravity (Mahony filter); yaw is only the integrated gyro (no magnetometer), so
its covariance is large.

The gyro's bias (a few deg/s on this chip, and it drifts with temperature) is measured whenever the rig is still
for a second, and saved (~/.visionnav/imu_bias.json) for a start while walking. /imu/data is only advertised
once the chip answers: the laptop turns Cartographer's IMU input on only when it sees this publisher, so an
unplugged IMU never leaves the map waiting for it. A lost connection (loose wire) is retried every second.

  ros2 run visionnav mpu6050_imu              the node (normally started by pi_sensors.launch.py)
  ros2 run visionnav mpu6050_imu calibrate    wiring check, then the imu_* mount arguments: stand straight,
                                              then lean forward (runs next to the node; needs no ROS)
"""

import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

# ── PARAMETERS ──
ADDRESS = 0x68            # AD0 open or to GND; 0x69 with AD0 to 3.3 V
BUS = 1                   # /dev/i2c-1 = header pins 3 (SDA) and 5 (SCL)
RATE_HZ = 100.0
GYRO_DPS = 500            # full scale: torso turns reach ~250 deg/s
ACCEL_G = 4               # full scale: heel strikes at the chest stay well below 4 g
DLPF_CFG = 3              # chip low-pass 44 Hz: below the 50 Hz Nyquist of 100 Hz reads
DLPF_DELAY_S = 0.005      # its delay, taken off the timestamps
SAMPLE_DIV = 4            # chip samples at 1 kHz / (1 + 4) = 200 Hz; each read gets the newest
G = 9.80665
STILL_GYRO = math.radians(0.6)   # rad/s std over a second: still (breathing moves a worn rig more)
STILL_ACC = 0.02 * G             # m/s^2 std of |a| over a second
BIAS_STEP = math.radians(2.0)    # a "still" second whose mean differs more from the bias is a slow turn
BIAS_ALPHA = 0.2
MAHONY_KP = 0.3           # low: walking accelerations must not tip the estimated vertical
BIAS_FILE = os.path.expanduser("~/.visionnav/imu_bias.json")

REG_SMPLRT_DIV, REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG = 0x19, 0x1A, 0x1B, 0x1C
REG_ACCEL_XOUT_H, REG_PWR_MGMT_1, REG_WHO_AM_I = 0x3B, 0x6B, 0x75
KNOWN_CHIPS = {0x68: "MPU-6050", 0x70: "MPU-6500", 0x71: "MPU-9250", 0x72: "MPU-6050 clone", 0x73: "MPU-9255",
               0x98: "MPU-6050 clone"}


class MPU6050:
    """Register access. read() -> (accel m/s^2, gyro rad/s, temperature C) in the chip's axes."""

    def __init__(self, bus=BUS, address=ADDRESS, gyro_dps=GYRO_DPS, accel_g=ACCEL_G):
        self.bus_no, self.address = bus, address
        self.gyro_scale = math.radians(gyro_dps) / 32768.0
        self.acc_scale = accel_g * G / 32768.0
        self._gyro_cfg = {250: 0, 500: 1, 1000: 2, 2000: 3}[gyro_dps] << 3
        self._acc_cfg = {2: 0, 4: 1, 8: 2, 16: 3}[accel_g] << 3
        self.bus = None
        self.who = None

    def open(self, reset=True):
        """Wake and configure the chip. reset=False when another process (the node) is already reading it."""
        if self.bus is None:
            try:
                from smbus2 import SMBus
            except ImportError:
                from smbus import SMBus
            self.bus = SMBus(self.bus_no)
        self.who = self.bus.read_byte_data(self.address, REG_WHO_AM_I)
        if reset:
            self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x80)
            time.sleep(0.1)
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x01)  # awake, clocked by the X gyro (steadier)
        time.sleep(0.05)
        self.bus.write_byte_data(self.address, REG_SMPLRT_DIV, SAMPLE_DIV)
        self.bus.write_byte_data(self.address, REG_CONFIG, DLPF_CFG)
        self.bus.write_byte_data(self.address, REG_GYRO_CONFIG, self._gyro_cfg)
        self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, self._acc_cfg)
        time.sleep(0.05)

    def close(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError:
                pass
        self.bus = None

    def read(self):
        raw = np.frombuffer(bytes(self.bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 14)), dtype=">i2")
        raw = raw.astype(np.float64)
        return raw[0:3] * self.acc_scale, raw[4:7] * self.gyro_scale, raw[3] / 340.0 + 36.53


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def up_in_sensor(q):
    """World up in the sensor's axes, from an orientation quaternion (w, x, y, z), sensor -> world."""
    w, x, y, z = q
    return np.array([2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z])


class Leveler:
    """Orientation from gyro and gravity (Mahony): roll and pitch are pulled toward gravity while the rig is not
    accelerating much; yaw only integrates the gyro."""

    def __init__(self):
        self.q = None  # (w, x, y, z)

    def update(self, gyro, acc, dt):
        n = float(np.linalg.norm(acc))
        if self.q is None:
            a = acc / n if n > 0 else np.array([0.0, 0.0, 1.0])
            roll, pitch = math.atan2(a[1], a[2]), math.atan2(-a[0], math.hypot(a[1], a[2]))
            cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
            self.q = np.array([cr * cp, sr * cp, cr * sp, -sr * sp])
            return self.q
        w = np.asarray(gyro, dtype=np.float64)
        if abs(n - G) < 0.15 * G:
            w = w + MAHONY_KP * np.cross(acc / n, up_in_sensor(self.q))
        self.q = self.q + 0.5 * _qmul(self.q, np.array([0.0, *w])) * dt
        self.q /= np.linalg.norm(self.q)
        return self.q


class GyroBias:
    """Gyro bias from the seconds the rig is still: the mean gyro of a still second is its bias."""

    def __init__(self, rate=RATE_HZ, bias=None):
        self.bias = None if bias is None else np.asarray(bias, dtype=np.float64)
        self.measured = False  # measured in this run (not only loaded from the file)
        self._win = deque(maxlen=max(10, int(rate)))

    def add(self, gyro_raw, acc) -> bool:
        """True when this sample completed a still second that updated the bias."""
        self._win.append(np.r_[gyro_raw, np.linalg.norm(acc)])
        if len(self._win) < self._win.maxlen:
            return False
        a = np.array(self._win)
        self._win.clear()
        if a[:, :3].std(axis=0).max() > STILL_GYRO or a[:, 3].std() > STILL_ACC or abs(a[:, 3].mean() - G) > 0.1 * G:
            return False
        m = a[:, :3].mean(axis=0)
        if not self.measured:
            self.bias, self.measured = m, True
        elif np.abs(m - self.bias).max() < BIAS_STEP:
            self.bias = (1 - BIAS_ALPHA) * self.bias + BIAS_ALPHA * m
        else:
            return False
        return True


def load_bias():
    try:
        with open(BIAS_FILE) as f:
            return np.array(json.load(f)["gyro_bias"], dtype=np.float64)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def save_bias(bias):
    try:
        os.makedirs(os.path.dirname(BIAS_FILE), exist_ok=True)
        with open(BIAS_FILE, "w") as f:
            json.dump({"gyro_bias": [float(b) for b in bias], "time": time.time()}, f)
    except OSError:
        pass


def describe_up(acc):
    """Which printed arrow points up, e.g. '+Y arrow up' or 'chip side up (+Z)'."""
    i = int(np.argmax(np.abs(acc)))
    sign = "+" if acc[i] > 0 else "-"
    if i == 2:
        return "chip side up (+Z)" if sign == "+" else "chip side down (-Z)"
    return f"{'XY'[i]} arrow pointing {'up' if sign == '+' else 'down'} ({sign}{'XY'[i]})"


def mount_rpy(up_straight, up_leaning):
    """imu_link mount on the rig (roll, pitch, yaw in rad, as sensor_tf.launch.py takes them) from gravity in the
    chip's axes standing straight and leaning forward. Leaning forward tips the body's forward axis down, so
    gravity's up direction moves toward the body's backward axis."""
    u = up_straight / np.linalg.norm(up_straight)
    d = u * float(u @ up_leaning) - up_leaning   # component of the change across u: points forward
    if np.linalg.norm(d) < 0.15 * np.linalg.norm(up_leaning):
        return None  # did not lean enough
    f = d / np.linalg.norm(d)
    R = np.vstack([f, np.cross(u, f), u])        # rows: body forward, left, up in the chip's axes
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def calibrate_mount():
    """Wiring check and mount measurement, in a terminal on the Pi."""
    imu = MPU6050()
    try:
        imu.open(reset=False)
    except (OSError, FileNotFoundError) as e:
        print(f"No IMU on /dev/i2c-{BUS} at 0x{ADDRESS:02x}: {e}\n"
              "Check: I2C enabled (setup_pi.sh), VCC to pin 1, GND to pin 9, SDA to pin 3, SCL to pin 5; "
              "`i2cdetect -y 1` must show 68.")
        return 1
    print(f"Found {KNOWN_CHIPS.get(imu.who, 'an unknown chip')} (WHO_AM_I 0x{imu.who:02x}) at 0x{ADDRESS:02x}.")

    def average(seconds, label):
        acc, gyro = [], []
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            a, g, temp = imu.read()
            acc.append(a)
            gyro.append(g)
            time.sleep(1.0 / RATE_HZ)
        acc, gyro = np.array(acc), np.array(gyro)
        a = acc.mean(axis=0)
        print(f"  {label}: accel {np.round(a, 2)} m/s^2 (|a| {np.linalg.norm(a):.2f}), "
              f"gyro {np.round(np.degrees(gyro.mean(axis=0)), 2)} deg/s, {temp:.0f} C — {describe_up(a)}")
        return a, np.degrees(gyro.std(axis=0)).max() < 1.5

    print("\nWear the rig. Stand up straight and still (3 s)...")
    time.sleep(1.0)
    up1, still = average(2.0, "straight")
    if abs(np.linalg.norm(up1) - G) > 0.1 * G:
        print("  |a| is not ~9.8 m/s^2: the chip is not reading correctly (wrong full-scale, or a bad clone).")
    if not still:
        print("  (you were moving: the result may be off by a few degrees)")
    print("\nNow lean your chest forward about 30 degrees, like a bow, and hold it (starts in 3 s)...")
    time.sleep(3.0)
    up2, _ = average(2.0, "leaning")
    rpy = mount_rpy(up1, up2)
    if rpy is None:
        print("\nThe lean was too small to find the forward direction. Run it again and lean further.")
        return 1
    roll, pitch, yaw = (round(math.degrees(v)) for v in rpy)
    print(f"\nMount: imu_roll_deg:={roll} imu_pitch_deg:={pitch} imu_yaw_deg:={yaw}")
    print("Add these to WEARABLE_BRAIN_ARGS on the laptop (next to the lidar_/camera_ values), e.g.\n"
          f'  export WEARABLE_BRAIN_ARGS="... imu_roll_deg:={roll} imu_pitch_deg:={pitch} imu_yaw_deg:={yaw}"')
    print("The recommended mount (upright on the chest plate, chip facing forward, Y arrow up) is "
          "imu_roll_deg:=90 imu_pitch_deg:=0 imu_yaw_deg:=90, the default.")
    imu.close()
    return 0


# ══════════════════════════════════════════════════════════════════════
# ── ROS NODE ──
# ══════════════════════════════════════════════════════════════════════
def main(args=None):
    if "calibrate" in sys.argv[1:]:
        sys.exit(calibrate_mount())

    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class ImuNode(Node):
        def __init__(self):
            super().__init__("mpu6050_imu")
            p = {k: self.declare_parameter(k, v).value for k, v in (
                ("bus", BUS), ("address", ADDRESS), ("rate", RATE_HZ), ("frame_id", "imu_link"),
                ("gyro_range_dps", GYRO_DPS), ("accel_range_g", ACCEL_G))}
            self._imu = MPU6050(int(p["bus"]), int(p["address"]), int(p["gyro_range_dps"]), int(p["accel_range_g"]))
            self._rate, self._frame = float(p["rate"]), str(p["frame_id"])
            self._delay = Duration(seconds=DLPF_DELAY_S)
            self._pub = None  # advertised once the chip answers
            self._bias = GyroBias(self._rate, load_bias())
            self._bias_saved_t = -1e9  # saved at the first still second, then at most once a minute
            self._level = Leveler()
            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

        def _connect(self):
            try:
                self._imu.close()
                self._imu.open()
            except (OSError, FileNotFoundError) as e:
                return str(e)
            acc = np.mean([self._imu.read()[0] for _ in range(20)], axis=0)
            self.get_logger().info(
                f"{KNOWN_CHIPS.get(self._imu.who, 'Unknown chip')} (WHO_AM_I 0x{self._imu.who:02x}) on "
                f"/dev/i2c-{self._imu.bus_no} at 0x{self._imu.address:02x}, {self._rate:.0f} Hz; at rest "
                f"{describe_up(acc)}. "
                + ("Gyro bias from the last run until the rig is still for 1 s." if self._bias.bias is not None
                   else "Keep the rig still for 1 s to measure the gyro bias."))
            if self._pub is None:
                self._pub = self.create_publisher(Imu, "/imu/data", 50)
            return None

        def _loop(self):
            period = 1.0 / self._rate
            connected, errors, last_err, last_warn = False, 0, None, 0.0
            last_t, last_log, n = None, time.monotonic(), 0
            next_t = time.monotonic()
            while not self._stop.is_set() and rclpy.ok():
                if not connected:
                    err = self._connect()
                    if err is None:
                        connected, errors, last_t = True, 0, None
                    else:
                        if err != last_err or time.monotonic() - last_warn > 30.0:
                            self.get_logger().error(
                                f"IMU not found on /dev/i2c-{self._imu.bus_no} at 0x{self._imu.address:02x}: {err}. "
                                "Check the wiring (pins 1, 3, 5, 9) and that I2C is on (setup_pi.sh). Retrying.")
                            last_err, last_warn = err, time.monotonic()
                        self._stop.wait(1.0)
                        continue
                    next_t = time.monotonic()
                try:
                    acc, gyro_raw, temp = self._imu.read()
                    errors = 0
                except OSError as e:
                    errors += 1
                    if errors >= 10:
                        self.get_logger().warn(f"IMU stopped answering ({e}): reconnecting")
                        connected = False
                    self._stop.wait(period)
                    continue
                now = time.monotonic()
                stamp = self.get_clock().now() - self._delay
                if self._bias.add(gyro_raw, acc) and now - self._bias_saved_t > 60.0:
                    if self._bias_saved_t < 0:
                        self.get_logger().info(
                            f"Gyro bias measured: {np.round(np.degrees(self._bias.bias), 2)} deg/s")
                    save_bias(self._bias.bias)
                    self._bias_saved_t = now
                gyro = gyro_raw - (self._bias.bias if self._bias.bias is not None else 0.0)
                q = self._level.update(gyro, acc, (now - last_t) if last_t is not None else period)
                last_t = now
                self._publish(stamp, q, gyro, acc)
                n += 1
                if now - last_log > 60.0:
                    self.get_logger().info(f"IMU: {n / (now - last_log):.0f} Hz, {temp:.0f} C, gyro bias "
                                           f"{np.round(np.degrees(self._bias.bias), 2) if self._bias.bias is not None else 'not measured yet'} deg/s")
                    last_log, n = now, 0
                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_t = time.monotonic()  # fell behind: do not burst to catch up

        def _publish(self, stamp, q, gyro, acc):
            m = Imu()
            m.header.stamp = stamp.to_msg()
            m.header.frame_id = self._frame
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (float(v) for v in q)
            m.orientation_covariance = [0.0012, 0.0, 0.0, 0.0, 0.0012, 0.0, 0.0, 0.0, 1000.0]  # 2 deg; yaw unknown
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(v) for v in gyro)
            m.angular_velocity_covariance = [2.5e-5, 0.0, 0.0, 0.0, 2.5e-5, 0.0, 0.0, 0.0, 2.5e-5]
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(v) for v in acc)
            m.linear_acceleration_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            self._pub.publish(m)

        def destroy_node(self):
            self._stop.set()
            self._thread.join(timeout=1.0)
            self._imu.close()
            super().destroy_node()

    rclpy.init(args=args)
    node = ImuNode()
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
