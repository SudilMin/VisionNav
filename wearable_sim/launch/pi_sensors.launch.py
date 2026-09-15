import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Slamtec RPLiDAR C1 via sllidar_ros2
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
        )
    ])
