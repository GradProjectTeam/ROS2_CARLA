from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_fusion_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mostafa',
    maintainer_email='mostafahendy@std.mans.edu.eg',
    description='ROS2 package for sensor fusion with TCP interfaces for IMU, radar, and lidar',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_tcp_node = sensor_fusion_2.imu_tcp_node:main',
            'radar_tcp_node = sensor_fusion_2.radar_tcp_node:main',
            'lidar_tcp_node = sensor_fusion_2.lidar_tcp_node:main',
            'lidar_listener_clusters_2 = sensor_fusion_2.lidar_listener_clusters_2:main',
        ],
    },
)
