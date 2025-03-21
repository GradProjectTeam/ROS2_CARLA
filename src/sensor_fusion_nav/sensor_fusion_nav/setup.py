from setuptools import setup
import os
from glob import glob

package_name = 'sensor_fusion_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mostafa',
    maintainer_email='your.email@example.com',
    description='Sensor fusion and navigation package for LiDAR, Radar, and IMU',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processing = sensor_fusion_nav.lidar_processing:main',
            'radar_processing = sensor_fusion_nav.radar_processing:main',
            'imu_processing = sensor_fusion_nav.imu_processing:main',
            'sensor_fusion = sensor_fusion_nav.sensor_fusion:main',
            'costmap_creator = sensor_fusion_nav.costmap_creator:main',
            'navigation = sensor_fusion_nav.navigation:main',
        ],
    },
)
