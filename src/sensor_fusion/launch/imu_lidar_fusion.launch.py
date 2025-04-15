#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    return LaunchDescription([
        # TCP connection parameters for IMU and LiDAR
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        
        # IMU and Map Parameters
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='Topic for IMU data'
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/permanent_map',
            description='Topic for permanent map'
        ),
        
        # Permanent map parameters (same as in permanent_map.launch.py)
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.2',
            description='Resolution of the permanent map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='100.0',
            description='Width of the permanent map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='100.0',
            description='Height of the permanent map in meters'
        ),
        DeclareLaunchArgument(
            'detection_radius',
            default_value='50.0',
            description='Detection radius for the permanent map (meters)'
        ),
        
        # Performance parameters
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='2.0',
            description='Rate to process map data (Hz)'
        ),
        
        # Bayesian update parameters for lidar mapper
        DeclareLaunchArgument(
            'hit_weight',
            default_value='0.99',
            description='Weight for obstacle hits in Bayesian update'
        ),
        DeclareLaunchArgument(
            'miss_weight',
            default_value='0.05',
            description='Weight for misses (free space) in Bayesian update'
        ),
        
        # Map saving parameters
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/mostafa/GP/ROS2/maps',
            description='Directory to save permanent maps'
        ),
        DeclareLaunchArgument(
            'enable_auto_save',
            default_value='true',
            description='Enable automatic saving of the map periodically'
        ),
        DeclareLaunchArgument(
            'auto_save_interval',
            default_value='60.0',
            description='Interval for auto-saving the map (seconds)'
        ),
        
        # IMU-specific parameters
        DeclareLaunchArgument(
            'initial_yaw_offset',
            default_value='0.0',
            description='Initial yaw offset in radians'
        ),
        DeclareLaunchArgument(
            'use_filtered_yaw',
            default_value='true',
            description='Use filtered yaw for stability'
        ),
        DeclareLaunchArgument(
            'yaw_filter_size',
            default_value='10',
            description='Size of filter buffer for yaw smoothing'
        ),
        DeclareLaunchArgument(
            'yaw_weight',
            default_value='0.7',
            description='Weight of yaw in the fusion (0-1)'
        ),
        
        # TF static transform publishers - complete transform tree
        # 1. World to Map - top level transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        ),
        
        # 2. Map to Base Link - vehicle position in the map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),
        
        # 3. Base Link to IMU Link - position of IMU on the vehicle
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        
        # 4. IMU Link to LiDAR Link - position of LiDAR relative to IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_imu_to_lidar',
            arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        ),
        
        # Set up the IMU Euler Visualizer node (to display IMU data)
        Node(
            package='sensor_fusion',
            executable='imu_euler_visualizer',
            name='imu_euler_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
                'tcp_port': LaunchConfiguration('imu_tcp_port'),
                'reconnect_interval': 5.0,
                'frame_id': 'imu_link',
                'world_frame_id': 'world',
                'filter_window_size': 5,
            }],
            output='screen'
        ),
        
        # Set up the LiDAR listener node
        Node(
            package='sensor_fusion',
            executable='lidar_listener_clusters_2',
            name='lidar_cube_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
                'tcp_port': LaunchConfiguration('lidar_tcp_port'),
                'point_size': 1.5,
                'center_size': 3.0,
                'use_convex_hull': True,
                'use_point_markers': False,
                'use_cluster_stats': True,
                'verbose_logging': False,
                'cube_alpha': 0.3,
                'filter_vehicle_points': True,
                'frame_id': 'lidar_link',  # Explicitly set the frame ID
                'use_tf_transform': True,   # Enable transform usage
            }],
            output='screen'
        ),
        
        # Set up the lidar permanent mapper node
        Node(
            package='sensor_fusion',
            executable='lidar_permanent_mapper',
            name='lidar_permanent_mapper',
            parameters=[{
                # Map resolution and size parameters
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_width_meters': LaunchConfiguration('map_width_meters'),
                'map_height_meters': LaunchConfiguration('map_height_meters'),
                
                # Processing parameters
                'publish_rate': LaunchConfiguration('publish_rate'),
                'process_rate': LaunchConfiguration('process_rate'),
                
                # Bayesian update parameters
                'hit_weight': LaunchConfiguration('hit_weight'),
                'miss_weight': LaunchConfiguration('miss_weight'),
                'update_threshold': 0.1,
                'prior_weight': 0.5,
                'count_threshold': 2.0,
                
                # Map saving parameters
                'map_save_dir': LaunchConfiguration('map_save_dir'),
                'enable_auto_save': LaunchConfiguration('enable_auto_save'),
                'auto_save_interval': LaunchConfiguration('auto_save_interval'),
                
                # Other parameters
                'ground_threshold': 0.4,
                'min_height': -0.5,
                'max_height': 4.0,
                'detection_radius': LaunchConfiguration('detection_radius'),
                'use_cluster_data': True,
                'frame_id': 'map',          # Set the map frame
                'vehicle_frame_id': 'base_link',  # Set the vehicle frame to match tf
            }],
            output='screen'
        ),
        
        # Set up the IMU-Lidar yaw fusion node
        Node(
            package='sensor_fusion',
            executable='imu_lidar_yaw_fusion',
            name='imu_lidar_yaw_fusion',
            parameters=[{
                # Connection parameters
                'imu_topic': LaunchConfiguration('imu_topic'),
                'map_topic': LaunchConfiguration('map_topic'),
                
                # Processing parameters
                'publish_rate': LaunchConfiguration('publish_rate'),
                
                # IMU-specific parameters
                'initial_yaw_offset': LaunchConfiguration('initial_yaw_offset'),
                'use_filtered_yaw': LaunchConfiguration('use_filtered_yaw'),
                'yaw_filter_size': LaunchConfiguration('yaw_filter_size'),
                'yaw_weight': LaunchConfiguration('yaw_weight'),
                
                # Map saving parameters
                'map_save_dir': LaunchConfiguration('map_save_dir'),
                'enable_auto_save': LaunchConfiguration('enable_auto_save'),
                'auto_save_interval': LaunchConfiguration('auto_save_interval'),
            }],
            output='screen'
        ),
        
        # RViz for visualization - Use the same configuration as permanent map
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/permanent_map.rviz']],
            output='screen'
        ),
    ]) 