import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'visionnav'

NODES = [
    'vision_perception', 'find_object', 'scene_describer', 'phone_camera', 'esp32_bridge',
    'scan_body_filter', 'semantic_costmap', 'semantic_navigator', 'structure_mapper',
    'check_lidar_orientation', 'gps_nav', 'sensor_sender', 'sensor_receiver', 'download_model',
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
        (os.path.join('share', package_name, 'models'), glob('models/*')),
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
