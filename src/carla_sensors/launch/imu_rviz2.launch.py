#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('carla_sensors')
    
    # Set the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'imu_config.rviz')
    
    # Declare launch arguments
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='IP address of the IMU TCP server'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='12345',
        description='Port number of the IMU TCP server'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for the IMU'
    )
    
    extended_format_arg = DeclareLaunchArgument(
        'extended_format',
        default_value='true',
        description='Use extended format with 10 values (true) or basic format with 7 values (false)'
    )
    
    # Node for IMU data processing and visualization
    imu_node = Node(
        package='carla_sensors',
        executable='imu_listener',
        name='imu_data_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('tcp_ip'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'extended_format': LaunchConfiguration('extended_format')
        }],
        output='screen'
    )
    
    # Static TF publisher for IMU link
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link']
    )
    
    # Launch RViz2 with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([
        tcp_ip_arg,
        tcp_port_arg,
        frame_id_arg,
        extended_format_arg,
        tf_node,
        imu_node,
        rviz_node
    ]) 