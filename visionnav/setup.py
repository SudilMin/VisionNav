import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'visionnav'

NODES = [
    'object_perception', 'voice_navigation_assistant', 'scene_describer', 'phone_camera_publisher', 'esp32_button_haptics_bridge',
    'lidar_body_filter', 'semantic_costmap', 'semantic_navigator', 'wall_structure_mapper',
    'lidar_orientation_calibrator', 'gps_voice_navigator',
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # TensorRT engines are generated at run time next to the weights, so they are not installed.
        (os.path.join('share', package_name, 'models'),
         [f for f in glob('models/*') if not f.endswith('.engine')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudil-minthaka',
    maintainer_email='sudilminthaka8797@gmail.com',
    description='VisionNav: chest-worn assistive navigation for blind users.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [f'{n} = {package_name}.{n}:main' for n in NODES],
    },
)
