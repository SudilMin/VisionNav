import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('wearable_sim')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'wearable.rviz')

    return LaunchDescription([
        # Fake Odometry / SLAM (Locks the user to the center of the map)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_odom',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'base_footprint']
        ),
        
        # Fake Chest Camera Mount (Places the camera at 1.3 meters high on the user)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_camera_mount',
            arguments=['--x', '0', '--y', '0', '--z', '1.3', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link']
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
