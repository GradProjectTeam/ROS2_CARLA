#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    sensor_fusion_pkg = get_package_share_directory('sensor_fusion')
    
    # Parameters
    map_resolution = LaunchConfiguration('map_resolution')
    map_width = LaunchConfiguration('map_width')
    map_height = LaunchConfiguration('map_height')
    map_origin_x = LaunchConfiguration('map_origin_x')
    map_origin_y = LaunchConfiguration('map_origin_y')
    
    # Costmap color values
    free_space_value = LaunchConfiguration('free_space_value')
    obstacle_value = LaunchConfiguration('obstacle_value')
    unknown_value = LaunchConfiguration('unknown_value')
    detection_radius = LaunchConfiguration('detection_radius')
    
    # Parameter tuning config
    config_file = LaunchConfiguration('config_file')
    tuning_method = LaunchConfiguration('tuning_method')
    max_samples = LaunchConfiguration('max_samples')
    
    # Launch Arguments
    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.1',
        description='Resolution of the costmap in meters per cell'
    )
    
    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='1000',
        description='Width of the costmap in cells'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='1000',
        description='Height of the costmap in cells'
    )
    
    map_origin_x_arg = DeclareLaunchArgument(
        'map_origin_x',
        default_value='-50.0',
        description='X coordinate of the map origin'
    )
    
    map_origin_y_arg = DeclareLaunchArgument(
        'map_origin_y',
        default_value='-50.0',
        description='Y coordinate of the map origin'
    )
    
    # Costmap color value arguments
    free_space_value_arg = DeclareLaunchArgument(
        'free_space_value',
        default_value='0',
        description='Value for free space in costmap (white)'
    )
    
    obstacle_value_arg = DeclareLaunchArgument(
        'obstacle_value',
        default_value='100',
        description='Value for obstacles in costmap (black)'
    )
    
    unknown_value_arg = DeclareLaunchArgument(
        'unknown_value',
        default_value='50',
        description='Value for unknown areas in costmap (gray)'
    )
    
    detection_radius_arg = DeclareLaunchArgument(
        'detection_radius',
        default_value='30.0',
        description='Radius within which areas are classified as free or obstacle'
    )
    
    # Parameter tuning arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/home/mostafa/parameter_tuning/sample_config.yaml',
        description='Path to the parameter tuning configuration file'
    )
    
    tuning_method_arg = DeclareLaunchArgument(
        'tuning_method',
        default_value='random',
        description='Tuning method: "exhaustive" or "random"'
    )
    
    max_samples_arg = DeclareLaunchArgument(
        'max_samples',
        default_value='10',
        description='Maximum number of parameter combinations to test'
    )
    
    # LiDAR Cluster Listener - Receives LiDAR data via TCP
    lidar_listener_clusters_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_cube_visualizer',
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12350,  # Using LiDAR port from the CARLA script
            'point_size': 1.5,
            'center_size': 3.0,
            'use_convex_hull': True,
            'use_point_markers': False,
            'use_cluster_stats': True,
            'verbose_logging': False,
            'cube_alpha': 0.3,
        }],
        output='screen'
    )
    
    # LiDAR Costmap Creator Node
    lidar_costmap_creator_node = Node(
        package='sensor_fusion',
        executable='lidar_costmap_creator',
        name='lidar_costmap_creator',
        parameters=[{
            # Map resolution and size parameters
            'map_resolution': map_resolution,
            'map_width': map_width,
            'map_height': map_height,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'publish_rate': 5.0,
            'ground_threshold': 0.1,
            'max_points_per_cell': 5,
            'min_height': -5.0,
            'max_height': 5.0,
            'obstacle_inflation': 0.5,
            'max_data_age': 60.0,
            
            # Three-color scheme parameters
            'free_space_value': free_space_value,
            'obstacle_value': obstacle_value,
            'unknown_value': unknown_value,
            'detection_radius': detection_radius
        }],
        output='screen'
    )
    
    # RViz for visualization
    rviz_config_file = os.path.join(sensor_fusion_pkg, 'rviz', 'lidar_clusters.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Parameter Tuner Process - using the wrapper script
    parameter_tuner = ExecuteProcess(
        cmd=[
            'python3', '/home/mostafa/parameter_tuning/wrapper.py',
            '--config', config_file,
            '--method', tuning_method,
            '--samples', max_samples,
            '--node-name', '/lidar_costmap_creator',
            '--verbose'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        map_resolution_arg,
        map_width_arg,
        map_height_arg,
        map_origin_x_arg,
        map_origin_y_arg,
        
        # Costmap color value arguments
        free_space_value_arg,
        obstacle_value_arg,
        unknown_value_arg,
        detection_radius_arg,
        
        # Parameter tuning arguments
        config_file_arg,
        tuning_method_arg,
        max_samples_arg,
        
        # Data source node
        lidar_listener_clusters_node,
        
        # Processing node
        lidar_costmap_creator_node,
        
        # Visualization
        rviz_node,
        
        # Parameter tuner
        parameter_tuner
    ]) 