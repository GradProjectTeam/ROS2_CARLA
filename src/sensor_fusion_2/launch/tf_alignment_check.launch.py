#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'three_sensors_fusion.rviz')
    
    # ==================== DECLARE LAUNCH ARGUMENTS ====================
    # Common arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_lidar_frame = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_link',
        description='LiDAR frame ID'
    )
    
    declare_radar_frame = DeclareLaunchArgument(
        'radar_frame',
        default_value='radar_link',
        description='Radar frame ID'
    )
    
    declare_map_frame = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )
    
    declare_arrow_length = DeclareLaunchArgument(
        'arrow_length',
        default_value='5.0',
        description='Length of direction arrows in meters'
    )
    
    declare_reference_distance = DeclareLaunchArgument(
        'reference_distance',
        default_value='10.0',
        description='Distance for reference points in meters'
    )
    
    # ==================== TF ALIGNMENT CHECKER NODE ====================
    tf_alignment_check_node = Node(
        package='sensor_fusion_2',
        executable='tf_alignment_check',
        name='tf_alignment_checker',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'radar_frame': LaunchConfiguration('radar_frame'),
            'map_frame': LaunchConfiguration('map_frame'),
            'arrow_length': LaunchConfiguration('arrow_length'),
            'reference_distance': LaunchConfiguration('reference_distance'),
        }],
        output='screen'
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim_time,
        declare_lidar_frame,
        declare_radar_frame,
        declare_map_frame,
        declare_arrow_length,
        declare_reference_distance,
        
        # Nodes
        tf_alignment_check_node,
        rviz_node,
    ]) 