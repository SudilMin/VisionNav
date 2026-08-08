import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Slamtec RPLiDAR C1 via sllidar_ros2
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[
                {'channel_type': 'serial'},
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 460800},
                {'frame_id': 'laser'},
                {'angle_compensate': True},
                {'scan_mode': 'Standard'},
            ]
        ),

        # 2. Camera Node (Publishes to /camera/image_raw)
        # Note: requires running `sudo apt install ros-jazzy-v4l2-camera` on the Pi 5
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[
                {'image_size': [640, 480]},
                {'camera_frame_id': 'camera_link'}
            ]
        )
    ])
