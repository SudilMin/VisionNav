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
    'camera_height': ('1.3', 'Camera lens height above the floor (m)'),
    'camera_pitch_deg': ('0.0', 'Camera downward tilt, positive = looking down (deg)'),
    'camera_yaw_deg': ('0.0', 'Camera yaw relative to the body, positive = left (deg)'),
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
    return [
        _static_tf('odom_to_base', 'odom', 'base_footprint'),
        _static_tf('lidar_mount', 'base_footprint', 'laser',
                   z=val['lidar_height'],
                   roll=math.radians(val['lidar_roll_deg']),
                   yaw=math.radians(val['lidar_yaw_deg'])),
        _static_tf('camera_mount', 'base_footprint', 'camera_mount',
                   z=val['camera_height'],
                   pitch=math.radians(val['camera_pitch_deg']),
                   yaw=math.radians(val['camera_yaw_deg'])),
        # Body frame (x forward) -> optical frame (z forward, x right, y down)
        _static_tf('camera_optical', 'camera_mount', 'camera_link',
                   roll=-math.pi / 2.0, yaw=-math.pi / 2.0),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=desc) for k, (d, desc) in ARGS.items()]
        + [OpaqueFunction(function=_launch_setup)]
    )
