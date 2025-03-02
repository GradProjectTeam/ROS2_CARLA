from setuptools import setup

package_name = 'carla_sensors'

setup(
    name=package_name,
    version='0.0.1',
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
        ],
    },
    # Add this section to include message generation
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
    ],
)
