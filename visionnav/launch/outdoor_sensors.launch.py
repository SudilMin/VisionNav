"""
outdoor_sensors.launch.py
=========================
Outdoor mode's sensor geometry: the chest rig's camera and LiDAR mounts (sensor_tf.launch.py), without the
indoor SLAM brain. object_perception needs them for the LiDAR walking corridor and for placing what the camera
sees. Also RViz with the live outdoor view (rviz/visionnav_outdoor.rviz): the wearer at the centre, the LiDAR
scan and only what is detected right now (/outdoor_markers) — no map is kept. use_rviz:=false leaves it out.
The rig's measured geometry passes straight through, as for the brain:
  ros2 launch visionnav outdoor_sensors.launch.py camera_height:=1.32 camera_pitch_deg:=12 lidar_yaw_deg:=188
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('visionnav')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='RViz with the live outdoor view'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sensor_tf.launch.py')),
            launch_arguments={'name_prefix': 'outdoor_', 'odom_tf': 'false'}.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='outdoor_rviz', output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'visionnav_outdoor.rviz')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
