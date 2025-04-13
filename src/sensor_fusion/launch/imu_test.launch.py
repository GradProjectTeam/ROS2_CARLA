from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for TCP connections
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12352',
            description='Port number of the IMU TCP server'
        ),
        
        # IMU processing parameters
        DeclareLaunchArgument(
            'use_madgwick_filter',
            default_value='true',
            description='Use Madgwick filter for orientation estimation'
        ),
        DeclareLaunchArgument(
            'madgwick_beta',
            default_value='0.1',
            description='Beta parameter for Madgwick filter'
        ),
        DeclareLaunchArgument(
            'gravity_compensation',
            default_value='true',
            description='Apply gravity compensation to accelerometer readings'
        ),
        DeclareLaunchArgument(
            'gyro_bias_correction',
            default_value='true',
            description='Apply bias correction to gyro readings'
        ),
        DeclareLaunchArgument(
            'accel_lpf_cutoff',
            default_value='5.0',
            description='Low-pass filter cutoff frequency for acceleration data (Hz)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Rate to publish processed IMU data (Hz)'
        ),
        
        # IMU Processor Node
        Node(
            package='sensor_fusion',
            executable='imu_processor',
            name='imu_processor',
            parameters=[{
                'imu_topic': '/imu/data_raw',
                'use_madgwick_filter': LaunchConfiguration('use_madgwick_filter'),
                'madgwick_beta': LaunchConfiguration('madgwick_beta'),
                'gravity_compensation': LaunchConfiguration('gravity_compensation'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'gyro_bias_correction': LaunchConfiguration('gyro_bias_correction'),
                'accel_lpf_cutoff': LaunchConfiguration('accel_lpf_cutoff'),
                'velocity_reset_threshold': 0.1,
                'velocity_decay_factor': 0.99,
            }],
            output='screen'
        ),
        
        # Visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/sensor_fusion.rviz']],
            output='screen'
        ),
        
        # TF static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
        
        # Optional: Add a node to publish test vehicle movement (to visualize IMU effects)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
    ]) 