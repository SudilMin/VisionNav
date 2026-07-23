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
        # 1. LDROBOT STL-19P Driver (Baudrate 230400)
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': 'scan'},
                {'frame_id': 'laser'},
                {'port_name': '/dev/ttyUSB0'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False}
            ]
        ),

        # 2. Fake Odometry (Locks odom to base_footprint)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_odom',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'odom', '--child-frame-id', 'base_footprint']
        ),
        
        # 3. Chest Camera Mount (Places the camera 1.3m high)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_camera_mount',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '1.3', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'camera_link']
        ),

        # 4. Chest LiDAR Mount (Places the LiDAR 1.2m high, flat on the chest)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fake_lidar_mount',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '1.2', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'laser']
        ),

        # 5. USB Camera Publisher (Commented out to run separately)
        # Node(
        #     package='wearable_sim',
        #     executable='phone_camera.py',
        #     name='phone_camera',
        #     output='screen'
        # ),
        # 
        # # 6. Vision Perception and Semantic Marker Publisher (Commented out to run separately)
        # Node(
        #     package='wearable_sim',
        #     executable='vision_perception.py',
        #     name='vision_perception',
        #     output='screen'
        # ),

        # 7. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'false'
            }.items()
        ),

        # 8. RViz for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
