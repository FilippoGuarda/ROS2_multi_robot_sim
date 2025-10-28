from setuptools import setup
import os
from glob import glob

package_name = 'smrta_wrapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Filippo Guarda',
    maintainer_email='filippo.guarda96@gmail.com',
    description='SMrTA wrapper for ROS2 multi-robot task allocation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_pos_aggregator = smrta_wrapper.fleet_pos_aggregator:main',
            'task_assignment_node = smrta_wrapper.task_assignment_node:main',
            'robot_controller_node = smrta_wrapper.robot_controller_node:main',
            'task_publisher_cli = smrta_wrapper.task_publisher_cli:main',
        ],
    },
)