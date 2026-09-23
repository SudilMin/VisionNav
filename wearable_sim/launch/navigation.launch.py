"""
navigation.launch.py
====================
Human-walkable semantic navigation: Nav2 planner + smoother servers, the semantic costmap layer
and the semantic navigator. Included by laptop_brain.launch.py (navigation:=true).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('wearable_sim'), 'config', 'nav2_params.yaml')
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen',
             parameters=[{'autostart': True, 'node_names': ['planner_server', 'smoother_server'],
                          'bond_timeout': 0.0}]),
        Node(package='wearable_sim', executable='semantic_costmap.py', name='semantic_costmap', output='screen'),
        Node(package='wearable_sim', executable='semantic_navigator.py', name='semantic_navigator', output='screen'),
    ])
