#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    map_frame_id = LaunchConfiguration('map_frame_id')
    
    # RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'lidar_filter_view.rviz')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    declare_lidar_frame_id = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='lidar',
        description='Frame ID for the LIDAR sensors (default for Carla)'
    )
    
    declare_map_frame_id = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Frame ID for the map coordinate system'
    )
    
    # Include LIDAR vehicle filter launch file with parameters
    lidar_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'lidar_vehicle_filter.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'lidar_frame_id': lidar_frame_id,
            'map_frame_id': map_frame_id
        }.items()
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Log info message
    log_info = LogInfo(
        msg=['Starting LIDAR visualization for Carla with frame_id: ', lidar_frame_id]
    )
    
    # Return the launch description
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_lidar_frame_id,
        declare_map_frame_id,
        
        # Log info
        log_info,
        
        # Include LIDAR filter launch file
        lidar_filter_launch,
        
        # RViz2 node
        rviz_node,
    ]) 