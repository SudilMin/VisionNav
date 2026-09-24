import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _maps_dir():
    return os.path.expanduser(os.environ.get('VISIONNAV_MAPS_DIR', '~/.visionnav/maps'))


def _slam_and_map_manager(context, config_dir):
    """Cartographer in mapping mode, or localizing in a saved map, plus the map manager.

    localize:=auto (default) localizes when ~/.visionnav/maps/<map>.pbstream exists, else maps.
    """
    name = LaunchConfiguration('map').perform(context)
    localize = LaunchConfiguration('localize').perform(context).lower()
    use_cartographer = LaunchConfiguration('slam').perform(context) == 'cartographer'
    state_file = os.path.join(_maps_dir(), f'{name}.pbstream')
    localizing = use_cartographer and localize != 'false' and os.path.isfile(state_file)
    actions = []
    if localize == 'true' and not localizing:
        actions.append(LogInfo(msg=f'WARNING: no saved map {state_file}; mapping instead.'))
    if use_cartographer:
        args = ['-configuration_directory', config_dir]
        if localizing:
            args += ['-configuration_basename', 'cartographer_localization.lua',
                     '-load_state_filename', state_file]
            actions.append(LogInfo(msg=f'Localizing in saved map {state_file}'))
        else:
            args += ['-configuration_basename', 'cartographer_config.lua']
            actions.append(LogInfo(msg=f"Mapping a new area (say 'save map' to keep it as '{name}')"))
        actions.append(Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=args,
            remappings=[('scan', 'scan_filtered')],
        ))
    actions.append(Node(
        package='visionnav',
        executable='map_manager',
        name='map_manager',
        output='screen',
        parameters=[{'map_name': name, 'mode': 'localization' if localizing else 'mapping'}],
    ))
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('visionnav')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'visionnav.rviz')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params_real.yaml')
    config_dir = os.path.join(pkg_dir, 'config')

    use_rviz = LaunchConfiguration('use_rviz')
    slam = LaunchConfiguration('slam')
    is_cartographer = PythonExpression(["'", slam, "' == 'cartographer'"])
    is_slam_toolbox = PythonExpression(["'", slam, "' == 'slam_toolbox'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz for visualization'),
        DeclareLaunchArgument(
            'navigation', default_value='true',
            description='Start human-walkable semantic navigation (Nav2 planner/smoother + semantic costmap)'),
        DeclareLaunchArgument(
            'slam', default_value='cartographer',
            description="SLAM backend: 'cartographer' (recommended, needs no odometry) or 'slam_toolbox'"),
        DeclareLaunchArgument(
            'map', default_value='home',
            description='Name of the saved map (~/.visionnav/maps/<map>.pbstream, objects and places)'),
        DeclareLaunchArgument(
            'localize', default_value='auto',
            description="'auto': localize in the saved map if it exists, else map; 'false': map anew"),
        # 1. Sensor extrinsics (odom->base_footprint, base_footprint->laser, base_footprint->camera_link).
        #    Mount arguments (camera_pitch_deg:=12, lidar_yaw_deg:=180, ...) pass straight through.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'sensor_tf.launch.py')),
        ),

        # 2. Remove the wearer's own body from the scan before SLAM
        Node(
            package='visionnav',
            executable='lidar_body_filter',
            name='lidar_body_filter',
            output='screen',
        ),

        # 3a. Cartographer (scan-matching SLAM — works without wheel odometry): maps a new area, or
        #     localizes in the saved map of it; plus the map manager (save map, named places)
        OpaqueFunction(function=_slam_and_map_manager, args=[config_dir]),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}, {'resolution': 0.05}],
            condition=IfCondition(is_cartographer),
        ),

        # 3b. SLAM Toolbox — only processes a scan after odometry reports motion, so with the
        #     static odom->base_footprint of a wearable it will not track you while walking.
        LogInfo(
            msg='WARNING: slam_toolbox needs real odometry; on the wearable use slam:=cartographer.',
            condition=IfCondition(is_slam_toolbox),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'false',
            }.items(),
            condition=IfCondition(is_slam_toolbox),
        ),

        # 3c. 3D walls and fixed structure from the SLAM map (YOLO cannot see walls)
        Node(
            package='visionnav',
            executable='wall_structure_mapper',
            name='wall_structure_mapper',
            output='screen',
        ),

        # 4. Semantic navigation: "go to chair" -> smooth walkable path (navigation.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'navigation.launch.py')),
            condition=IfCondition(LaunchConfiguration('navigation')),
        ),

        # 5. RViz for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
