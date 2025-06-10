#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Path to the RViz configuration file
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'radar_polar_visualization.rviz')
    
    
    # Add TCP connection parameters as launch arguments
    tcp_ip = LaunchConfiguration('tcp_ip')
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='IP address of the CARLA radar TCP server'
    )
    
    tcp_port = LaunchConfiguration('tcp_port')
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='12347',
        description='Port number of the CARLA radar TCP server'
    )
    
    connection_retry_interval = LaunchConfiguration('connection_retry_interval')
    connection_retry_interval_arg = DeclareLaunchArgument(
        'connection_retry_interval',
        default_value='5.0',
        description='Interval in seconds between connection retry attempts'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(tcp_ip_arg)
    ld.add_action(tcp_port_arg)
    ld.add_action(connection_retry_interval_arg)
    
    # Add the CARLA radar direct visualization node
    radar_carla_direct_node = Node(
        package='sensor_fusion',
        executable='radar_carla_direct',
        name='radar_carla_direct',
        output='screen',
        parameters=[
            {
                'jitter_scale': jitter_scale,
                'jitter_scaling_with_distance': True,
                'use_advanced_coloring': True,
                'show_velocity_vectors': True,
                'tcp_ip': tcp_ip,
                'tcp_port': tcp_port,
                'connection_retry_interval': connection_retry_interval
            }
        ]
    )
    ld.add_action(radar_carla_direct_node)
    
    # Add static transform publisher for radar frame
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'radar_link']
    )
    ld.add_action(static_transform_node)
    
    # Add RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    ld.add_action(rviz_node)
    
    return ld 