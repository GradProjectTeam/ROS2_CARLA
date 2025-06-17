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
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
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
            'imu_tcp_node = ' + package_name + '.imu_tcp_node:main',
            'radar_tcp_node = ' + package_name + '.radar_tcp_node:main',
            'lidar_tcp_node = ' + package_name + '.lidar_tcp_node:main',
            'lidar_listener_clusters_2 = ' + package_name + '.lidar_listener_clusters_2:main',
            'imu_euler_visualizer = ' + package_name + '.imu_euler_visualizer:main',
            'lidar_listener_clusters_3 = ' + package_name + '.lidar_listener_clusters_3:main',
            'lidar_realtime_mapper = ' + package_name + '.lidar_realtime_mapper:main',
            'radar_listener_clusters = ' + package_name + '.radar_listener_clusters:main',
            'radar_map_generator = ' + package_name + '.radar_map_generator:main',
            'radar_costmap_creator = ' + package_name + '.radar_costmap_creator:main',
            'fusion_costmap_creator = ' + package_name + '.fusion_costmap_creator:main',
            'three_sensor_fusion = ' + package_name + '.three_sensor_fusion:main',
            'radar_object_detector = ' + package_name + '.radar_object_detector:main',
            'imu_euler_visualizer_simple = ' + package_name + '.imu_euler_visualizer_simple:main',
            'tf_alignment_check = ' + package_name + '.tf_alignment_check:main',
            'semantic_costmap_visualizer = ' + package_name + '.semantic_costmap_visualizer:main',
            'tune_semantic_costmap = ' + package_name + '.tune_semantic_costmap:main',
        ],
    },
)
