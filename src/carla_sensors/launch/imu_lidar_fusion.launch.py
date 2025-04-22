#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('carla_sensors')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'imu_lidar_fusion.rviz')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # IMU-LiDAR fusion node
    imu_lidar_fusion_node = Node(
        package='carla_sensors',
        executable='imu_listener',
        name='imu_lidar_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12345,
            'frame_id': 'imu_link',
            'lidar_frame_id': 'lidar_link',
            'world_frame_id': 'world',
            'filter_window_size': 5,
            'use_first_7_only': True
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Static transform publisher for any fixed transforms if needed
    # This might be useful if you have fixed LiDAR transforms that don't change with IMU
    # (This is just an example, our IMU node is already publishing these transforms)
    # static_transform_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        imu_lidar_fusion_node,
        rviz_node
    ]) 