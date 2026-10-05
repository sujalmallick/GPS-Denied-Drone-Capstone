from setuptools import setup
import os
from glob import glob

package_name = 'coordination'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={'coordination': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Capstone Team',
    maintainer_email='capstone@university.edu',
    description='Multi-Drone Swarm Coordination Layer for GPS-Denied Indoor Exploration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_merger = coordination.map_merger:main',
            'task_allocator = coordination.task_allocator:main',
            'swarm_sim_bridge = coordination.swarm_sim_bridge:main',
            'depth_mapper = coordination.depth_mapper:main',
            'flight_controller = coordination.flight_controller:main',
            'gz_overlay = coordination.gz_overlay:main',
            'clock_relay = coordination.clock_relay:main',
        ],
    },
)
