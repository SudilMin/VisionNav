import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_dir = get_package_share_directory('wearable_sim')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'wearable.rviz')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params_real.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz for visualization'
    )

    return LaunchDescription([
        declare_use_rviz,

        # 1. SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
            ),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': 'false'
            }.items()
        ),

        # 2. RViz for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz)
        )
    ])
