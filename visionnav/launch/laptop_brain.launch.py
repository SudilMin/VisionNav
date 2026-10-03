"""
laptop_brain.launch.py
======================
Indoor mode's map on the laptop: sensor TFs, the body filter, Cartographer SLAM (a fresh map every session),
the map manager (named places), 3-D walls, semantic navigation and, optionally, RViz.
The assistant starts it (system_manager.py) with use_rviz:=false: the map window is a part of its own.

  imu:=true      use the chest IMU (/imu/data) in Cartographer (cartographer_imu.lua)
Rig geometry arguments (camera_pitch_deg:=12, lidar_yaw_deg:=188, ...) pass through to sensor_tf.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cartographer(context, config_dir):
    """Cartographer in mapping mode, with the chest IMU when imu:=true."""
    imu = LaunchConfiguration('imu').perform(context).lower() == 'true'
    actions = [LogInfo(msg='Using the chest IMU (/imu/data)')] if imu else []
    actions.append(Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', config_dir,
                   '-configuration_basename', 'cartographer_imu.lua' if imu else 'cartographer_config.lua'],
        remappings=[('scan', 'scan_filtered'), ('imu', '/imu/data')],
    ))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('visionnav')
    config_dir = os.path.join(pkg_dir, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz with the map'),
        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='Start semantic navigation (Nav2 planner/smoother + semantic costmap + navigator)'),
        # Cartographer waits for every sensor it is configured with: only true while the Pi publishes /imu/data
        # (the assistant checks that when it starts the brain; WEARABLE_IMU=0 turns it off)
        DeclareLaunchArgument(
            'imu', default_value='false',
            description='Use the chest IMU (/imu/data, mpu6050_imu on the Pi) in Cartographer'),

        # 1. Sensor extrinsics (odom->base_footprint, base_footprint->laser/camera_link/imu_link)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sensor_tf.launch.py')),
        ),

        # 2. Remove the wearer's own body from the scan before SLAM
        Node(package='visionnav', executable='lidar_body_filter', name='lidar_body_filter', output='screen'),

        # 3. Cartographer (scan-matching SLAM, needs no wheel odometry) and its occupancy grid on /map
        OpaqueFunction(function=_cartographer, args=[config_dir]),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}, {'resolution': 0.05}],
        ),

        # 4. Named places of this session
        Node(package='visionnav', executable='map_manager', name='map_manager', output='screen'),

        # 5. 3-D walls and fixed structure from the SLAM map (YOLO cannot see walls)
        Node(package='visionnav', executable='wall_structure_mapper', name='wall_structure_mapper', output='screen'),

        # 6. Semantic navigation: "go to chair" -> smooth walkable path (navigation.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'navigation.launch.py')),
            condition=IfCondition(LaunchConfiguration('navigation')),
        ),

        # 7. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'visionnav.rviz')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
