"""Outdoor GPS + LiDAR fusion: robot_localization EKF and navsat_transform with config/robot_localization_gps.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('visionnav'), 'config', 'robot_localization_gps.yaml')
    return LaunchDescription([
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[params], output='screen'),
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform_node',
             parameters=[params], output='screen'),
    ])
