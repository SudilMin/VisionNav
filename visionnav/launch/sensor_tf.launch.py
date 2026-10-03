"""
sensor_tf.launch.py
===================
Single source of truth for the chest-rig sensor extrinsics.

Both the SLAM backend and object_perception.py read these transforms, so the LiDAR map
and the camera-projected objects always agree. Measure your rig once and pass the values
as launch arguments instead of editing numbers in several files.

Frames:
  odom -> base_footprint        identity (no wheel odometry on a wearable; SLAM moves map->odom)
  base_footprint -> laser       LiDAR mount
  base_footprint -> camera_mount -> camera_link (optical: z forward, x right, y down)
  base_footprint -> imu_link    MPU-6050 mount (the chip's printed axes); Cartographer tracks this frame
                                when the IMU is used (laptop_brain.launch.py imu:=true)
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    # LiDAR yaw MEASURED on the rig with visionnav/lidar_orientation_calibrator.py (camera depth vs LiDAR
    # ranges): its forward axis points backward, ~8 deg off. With 0 here, walking backward showed as
    # walking forward in the map. Re-run the tool whenever the LiDAR is re-mounted.
    # roll 180 = LiDAR mounted upside down (the tool reports this as a "mirrored" scan).
    'lidar_height': ('1.2', 'LiDAR scan plane height above the floor (m)'),
    'lidar_yaw_deg': ('188.0', 'LiDAR yaw on the rig (deg), measured by lidar_orientation_calibrator.py'),
    'lidar_roll_deg': ('0.0', 'LiDAR roll on the rig, 180 = upside down (deg)'),
    # Camera tilt and turn MEASURED on the recording ~/visionnav_chair (2026-10-01; with object_perception's 52 deg
    # field of view): the LiDAR ranges drawn into the depth picture agree best turned 2 deg left (the turn is
    # sharp: 3 deg off loses a tenth of the matches); the tilt only loosely (0 deg 43%, 10 deg 49%, 20 deg 51%),
    # so it is set by the picture: a chair 2.3 m ahead (its size; the wall behind it 2.8 m by LiDAR) has its seat
    # and top rail where a 10 deg downward tilt puts them, and the door tops stay in view as they do.
    'camera_height': ('1.3', 'Camera lens height above the floor (m)'),
    'camera_pitch_deg': ('10.0', 'Camera downward tilt, positive = looking down (deg)'),
    'camera_yaw_deg': ('2.0', 'Camera yaw relative to the body, positive = left (deg)'),
    # MPU-6050 mount MEASURED on the rig: board flat, chip up, X arrow forward. From the gravity it read while the
    # wearer stood still at the start of a recorded walk (2026-10-01, accel -0.13 -0.03 9.06 m/s^2): roll -0.2 deg,
    # pitch 0.8 deg. The 6 deg pitch measured earlier (with a lean) tilted every scan 6 deg in the map. Re-run
    # `setup_pi.sh imu` whenever the board is re-mounted (system_manager refuses the IMU when these do not put its
    # gravity upward).
    'imu_height': ('1.15', 'IMU height above the floor (m)'),
    'imu_x': ('0.0', 'IMU forward of the LiDAR axis (m)'),
    'imu_roll_deg': ('0.0', 'IMU mount roll (deg), from mpu6050_imu calibrate'),
    'imu_pitch_deg': ('1.0', 'IMU mount pitch (deg), from mpu6050_imu calibrate'),
    'imu_yaw_deg': ('1.0', 'IMU mount yaw (deg), from mpu6050_imu calibrate'),
}
# Not numbers: outdoor mode (outdoor_sensors.launch.py) runs these TFs without the SLAM brain, under its own
# node names (so the brain's nodes lingering in the network's node list after a mode switch are not mistaken
# for them), and without odom -> base_footprint (outdoors everything is drawn around the wearer, base_footprint).
FLAGS = {
    'name_prefix': ('', 'Prefix of the static TF publisher node names'),
    'odom_tf': ('true', "Publish the identity odom -> base_footprint ('false' when an EKF publishes it)"),
}


def _static_tf(name, parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', parent, '--child-frame-id', child],
    )


def _launch_setup(context):
    val = {k: float(LaunchConfiguration(k).perform(context)) for k in ARGS}
    pre = LaunchConfiguration('name_prefix').perform(context)
    odom = LaunchConfiguration('odom_tf').perform(context).lower() == 'true'
    return ([_static_tf(pre + 'odom_to_base', 'odom', 'base_footprint')] if odom else []) + [
        _static_tf(pre + 'lidar_mount', 'base_footprint', 'laser',
                   z=val['lidar_height'],
                   roll=math.radians(val['lidar_roll_deg']),
                   yaw=math.radians(val['lidar_yaw_deg'])),
        _static_tf(pre + 'camera_mount', 'base_footprint', 'camera_mount',
                   z=val['camera_height'],
                   pitch=math.radians(val['camera_pitch_deg']),
                   yaw=math.radians(val['camera_yaw_deg'])),
        # Body frame (x forward) -> optical frame (z forward, x right, y down)
        _static_tf(pre + 'camera_optical', 'camera_mount', 'camera_link',
                   roll=-math.pi / 2.0, yaw=-math.pi / 2.0),
        _static_tf(pre + 'imu_mount', 'base_footprint', 'imu_link',
                   x=val['imu_x'], z=val['imu_height'],
                   roll=math.radians(val['imu_roll_deg']),
                   pitch=math.radians(val['imu_pitch_deg']),
                   yaw=math.radians(val['imu_yaw_deg'])),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=desc) for k, (d, desc) in {**ARGS, **FLAGS}.items()]
        + [OpaqueFunction(function=_launch_setup)]
    )
