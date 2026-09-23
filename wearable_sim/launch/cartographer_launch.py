import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Kept for backwards compatibility: identical to `laptop_brain.launch.py slam:=cartographer`,
    # so both entry points share the same sensor TFs and body filter.
    pkg_share = get_package_share_directory('wearable_sim')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'laptop_brain.launch.py')),
            launch_arguments={'slam': 'cartographer'}.items(),
        ),
    ])
