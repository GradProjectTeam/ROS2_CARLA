from setuptools import setup
import os
from glob import glob

package_name = 'sensor_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mostafa',
    maintainer_email='user@example.com',
    description='Sensor fusion package for LiDAR, radar, and IMU integration with trajectory planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_costmap_creator = sensor_fusion.fusion_costmap_creator:main',
            'trajectory_planner = sensor_fusion.trajectory_planner:main',
            'imu_processor = sensor_fusion.imu_processor:main',
            'lidar_costmap_creator = sensor_fusion.lidar_costmap_creator:main',
            'lidar_listener_clusters_2 = sensor_fusion.lidar_listener_clusters_2:main',
            'lidar_permanent_mapper = sensor_fusion.lidar_permanent_mapper:main',
            'radar_costmap_creator = sensor_fusion.radar_costmap_creator:main',
            'radar_listener_clusters = sensor_fusion.radar_listener_clusters:main',
        ],
    },
)
