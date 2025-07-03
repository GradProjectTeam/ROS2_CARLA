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
            'lidar_listener_clusters_3 = ' + package_name + '.lidar_listener_clusters_3:main',
            'lidar_realtime_mapper = ' + package_name + '.lidar_realtime_mapper:main',
            'radar_listener_clusters = ' + package_name + '.radar_listener_clusters:main',
            'imu_euler_visualizer_simple = ' + package_name + '.imu_euler_visualizer_simple:main',
            'semantic_costmap_visualizer = ' + package_name + '.semantic_costmap_visualizer:main',
            'waypoint_listener = ' + package_name + '.waypoint_listener:main',
            'waypoint_map_generator = ' + package_name + '.waypoint_map_generator:main',
            'binary_map_combiner = ' + package_name + '.binary_map_combiner:main',
            'trajectory_receiver = ' + package_name + '.trajectory_receiver_entry_point:main',
            'mpc_controller = ' + package_name + '.mpc_controller_entry_point:main',
            'trajectory_mpc = ' + package_name + '.trajectory_mpc_entry_point:main',
            'enhanced_dwa_planner_node = ' + package_name + '.enhanced_dwa_planner_node:main',
            'dwa_mpc_bridge = ' + package_name + '.dwa_mpc_bridge:main',
        ],
    },
)
