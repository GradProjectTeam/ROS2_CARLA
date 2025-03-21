from setuptools import setup
import os
from glob import glob

package_name = 'carla_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='CARLA sensor integration for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_listener = carla_sensors.radar_listener:main',
            'imu_listener = carla_sensors.imu_listener:main',
            'lidar_listener = carla_sensors.lidar_listener:main',
            'lidar_map_creator = carla_sensors.lidar_map_creator:main',
            'radar_map_creator = carla_sensors.radar_map_creator:main',
            'lidar_slam_node = carla_sensors.lidar_slam_node:main',
            'lidar_listener_raw = carla_sensors.lidar_listener_raw:main',
            'lidar_listener_clusters = carla_sensors.lidar_listener_clusters:main',
            'lidar_listener_raw_clusters = carla_sensors.lidar_listener_raw_clusters:main',
            'lidar_param_tuner = carla_sensors.lidar_param_tuner:main',
            'radar_listener_raw = carla_sensors.radar_listener_raw:main',
            'radar_listener_clusters = carla_sensors.radar_listener_clusters:main',
            'radar_listener_clusters_colored = carla_sensors.radar_listener_clusters_colored:main',
            'enhanced_radar_visualizer = carla_sensors.enhanced_radar_visualizer:main',
            'radar_costmap_creator = carla_sensors.radar_costmap_creator:main',
            'lidar_costmap_creator = carla_sensors.lidar_costmap_creator:main',
            'sensor_fusion_costmap = carla_sensors.sensor_fusion_costmap:main',
            'imu_rviz_visualizer = carla_sensors.imu_rviz_visualizer:main',
            
        
        ],

    },
    # Add this section to include message generation
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Include all launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Include RViz configuration files
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # Include README files
        ('share/' + package_name, glob('*.md')),
    ],
    # Add this section to include additional packages
    package_data={
        package_name: ['../opt/ros/rolling/include/*'],
    },
)
