import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_dir = get_package_share_directory('wearable_sim')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'wearable.rviz')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params_real.yaml')

    return LaunchDescription([
        # 1. Slamtec RPLiDAR C1 via sllidar_ros2 (works with C1 firmware)
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

        # 2. odom → base_footprint (identity transform)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                        '--frame-id', 'odom', '--child-frame-id', 'base_footprint']
        ),

        # 3. LiDAR Mount (1.2m high on chest)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_lidar_mount',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '1.2',
                        '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'laser']
        ),

        # 4. Camera Mount (1.3m high, rotated to match camera optical convention)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_camera_mount',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '1.3',
                        '--roll', '-1.57079632679', '--pitch', '0.0', '--yaw', '-1.57079632679',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link']
        ),

        # 5. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'false'
            }.items()
        ),

        # 6. RViz for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
