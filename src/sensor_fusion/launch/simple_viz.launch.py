#!/usr/bin/env python3
# Simple visualization launch file - just TF and RViz, no complex nodes

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # =============== Launch Arguments ===============
    
    # Simulation parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # Launch arguments
    package_dir = get_package_share_directory('sensor_fusion')
    rviz_config = os.path.join(package_dir, 'rviz', 'simple.rviz')
    
    # Declare all launch arguments
    launch_args = [
        # Simulation parameters
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
    ]
    
    # =============== TF Tree Setup ===============
    
    # Start with world frame as root
    tf_static_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Map to odom transform
    tf_static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Odom to base_link transform
    tf_static_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_baselink',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # Base_link to lidar transform
    tf_static_baselink_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_baselink_lidar',
        arguments=['0', '0', '1.8', '0', '0', '0', 'base_link', 'lidar_link']
    )
    
    # =============== Visualization ===============
    
    # RViz node with configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # TF tree setup - launch all transforms first
            tf_static_world_map,
            tf_static_map_odom,
            tf_static_odom_baselink,
            tf_static_baselink_lidar,
            # Start RViz
            rviz_node,
            # Log info
            LogInfo(msg="Simple visualization launched successfully")
        ]
    ) 