#!/usr/bin/env python3
"""
sim_launch.py
=============
ROS 2 Jazzy + Gazebo Harmonic launch file for the wearable blind-assist chest rig.

Features:
- Launches Gazebo Harmonic with an indoor Depot world (via Fuel)
- Spawns the humanoid robot with chest_rig.urdf
- Bridges sensors, odometry, cmd_vel, and TF
- Runs pointcloud_to_laserscan to flatten 3D LiDAR into 2D /scan
- Launches SLAM Toolbox for 2D mapping
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = "wearable_sim"
    pkg_share = get_package_share_directory(pkg_name)

    # 1. URDF Path
    urdf_file = os.path.join(pkg_share, 'urdf', 'chest_rig.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. SLAM params path
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # Environment variables for hybrid graphics (NVIDIA Optimus)
    env_vars = [
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),
    ]

    # 3. Gazebo Harmonic with our custom local world
    # This avoids network issues (404) and missing default files
    world_file = os.path.join(pkg_share, 'worlds', 'wearable_world.sdf')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 5. Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'blind_assist_wearable',
            '-topic', 'robot_description',
            '-z', '0.1'  # Slightly above ground to prevent sinking
        ],
        output='screen'
    )

    # 6. Bridge Gazebo topics to ROS 2
    # Direction notation:
    #   [  = Gazebo → ROS 2   (sensor / odom data flowing out)
    #   ]  = ROS 2 → Gazebo   (commands flowing in)
    #   @  = bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Sensors: Gazebo → ROS 2
            '/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # Drive command: ROS 2 → Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Odometry: Gazebo → ROS 2
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # TF from DiffDrive: Gazebo → ROS 2
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/lidar/points/points', '/lidar/points')
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 7. PointCloud to LaserScan
    # We use base_footprint as the target_frame so the 3D pointcloud is transformed
    # into a level plane relative to the ground. This prevents the tilted LiDAR from
    # seeing the floor as a solid wall.
    pc_to_ls = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'min_height': 0.05,  # Lowered to 0.05m to detect small objects like the Coke Can
            'max_height': 2.0,   # Up to 2m high
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087, # ~0.5 degrees
            'scan_time': 0.1,    # 10Hz
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': True
        }],
        remappings=[
            ('cloud_in', '/lidar/points'),
            ('scan', '/scan')
        ]
    )

    # 8. SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    # 9. Vision AI Node (Auto-synced with Sim Time)
    vision_node = Node(
        package='wearable_sim',
        executable='vision_perception.py',
        name='vision_perception',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. RViz (Auto-synced with Sim Time)
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'wearable.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(env_vars + [
        gz_sim,
        rsp,
        spawn_entity,
        bridge,
        pc_to_ls,
        slam_toolbox,
        vision_node,
        rviz
    ])
