#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Define paths
    rviz_config_path = os.path.join(pkg_dir, 'config', 'imu_euler_visualization.rviz')
    
    # Launch arguments
    ip_address = LaunchConfiguration('ip_address')
    port = LaunchConfiguration('port')
    
    # Declare launch arguments
    declare_ip_address = DeclareLaunchArgument(
        'ip_address',
        default_value='127.0.0.1',
        description='IP address for TCP connection to IMU server'
    )
    
    declare_port = DeclareLaunchArgument(
        'port',
        default_value='12345',
        description='Port for TCP connection to IMU server'
    )
    
    # Launch the IMU Euler visualization node
    imu_viz_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        output='screen',
        parameters=[
            {'tcp_ip': ip_address},
            {'tcp_port': port},
            {'frame_id': 'imu_link'},
            {'world_frame_id': 'world'},
            {'filter_window_size': 5}
        ]
    )
    
    # Launch RViz2 with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([
        declare_ip_address,
        declare_port,
        imu_viz_node,
        rviz_node
    ]) 