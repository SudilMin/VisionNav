import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('buttons', default_value='true',
                              description='Start the GPIO push-button panel (pi_button_panel)'),
        # Four push buttons on the GPIO header: LOOK, MODE, HAND, TALK (see pi_button_panel.py for wiring)
        Node(
            package='visionnav',
            executable='pi_button_panel',
            name='pi_button_panel',
            output='screen',
            condition=IfCondition(LaunchConfiguration('buttons')),
        ),
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
