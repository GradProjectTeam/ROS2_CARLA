#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Create transform publisher nodes
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False}],
    )
    
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': False}],
    )
    
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False}],
    )
    
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': False}],
    )
    
    # Create the realtime mapper node with a slight delay
    realtime_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_realtime_mapper',
        parameters=[{
            # Map resolution and size parameters
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'center_on_vehicle': LaunchConfiguration('center_on_vehicle'),
            
            # Processing parameters - set to consistent 10Hz for all operations
            'publish_rate': 10.0,          # Fixed 10 FPS publish rate
            'process_rate': 10.0,          # Fixed 10 FPS process rate 
            
            # Bayesian update weights - optimized for black/white contrast
            'hit_weight': 1.0,             # Maximum confidence in hits for black obstacles
            'miss_weight': 0.3,            # Strong clearing for white free space
            'prior_weight': 0.5,           # Start with unknown/gray
            'count_threshold': 0.2,        # Very low threshold for immediate binary mapping
            
            # Real-time specific parameters for aggressive old data removal
            'decay_rate': 0.9,             # Very aggressive decay (increased from 0.8)
            'update_threshold': 0.001,     # Very low threshold for immediate updates
            'temporal_memory': 0.5,        # Only remember points for half a second
            'enable_map_reset': True,      # Enable periodic map reset
            'map_reset_interval': 20.0,    # Reset the entire map every 20 seconds (reduced from 30)
            'use_binary_map': True,        # Force binary (black/white) map output
            'obstacle_threshold': 0.75,    # Higher threshold for clearer black obstacles
            
            # Other parameters
            'ground_threshold': 0.2,
            'min_height': -0.3,
            'max_height': 2.0,
            'raycast_skip': 1,             # Process every point for more detail at 10Hz
            'max_points_to_process': 8000, # Process more points for a complete picture
            'use_cluster_data': True,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # TF parameters to make transform lookups more robust
            'tf_buffer_duration': 10.0,    # Longer buffer for transforms (seconds)
            'tf_timeout': 1.0,             # Longer timeout for lookups (seconds) 
            'use_sim_time': False,         # Ensure real clock time is used
            'wait_for_transform': True,    # Wait for transform to be available
            'transform_tolerance': 0.5,    # Tolerance for transform lookups (seconds)
        }],
        output='screen'
    )
    
    # Delay the start of the realtime mapper to ensure transforms are available
    delayed_mapper = TimerAction(
        period=2.0,  # 2 second delay
        actions=[realtime_mapper_node]
    )
    
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
        
        # Map Parameters - real-time specific parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.1',
            description='Resolution of the real-time map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='40.0',
            description='Width of the real-time map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='40.0',
            description='Height of the real-time map in meters'
        ),
        DeclareLaunchArgument(
            'center_on_vehicle',
            default_value='true',
            description='Keep map centered on vehicle position'
        ),
        
        # Performance parameters - higher rates for real-time
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',           # Changed to 10.0 from 15.0
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='10.0',           # Changed to 10.0 from 30.0
            description='Rate to process map data (Hz)'
        ),
        
        # Visualization parameters
        DeclareLaunchArgument(
            'point_size',
            default_value='1.5',
            description='Size of individual point markers'
        ),
        DeclareLaunchArgument(
            'center_size',
            default_value='3.0',
            description='Size of cluster center markers'
        ),
        DeclareLaunchArgument(
            'use_convex_hull',
            default_value='true',
            description='Whether to display 2D convex hull around clusters'
        ),
        
        # TF static transform publishers - complete transform tree
        # 1. World to Map - top level transform
        world_to_map_node,
        
        # 2. Map to Base Link - vehicle position in the map
        map_to_base_link_node,
        
        # 3. Base Link to IMU Link - position of IMU on the vehicle
        base_to_imu_node,
        
        # 4. IMU Link to LiDAR Link - position of LiDAR relative to IMU
        imu_to_lidar_node,
        
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
                'point_size': LaunchConfiguration('point_size'),
                'center_size': LaunchConfiguration('center_size'),
                'use_convex_hull': LaunchConfiguration('use_convex_hull'),
                'use_point_markers': False,
                'use_cluster_stats': True,
                'verbose_logging': False,
                'cube_alpha': 0.3,
                'filter_vehicle_points': True,
                'frame_id': 'lidar_link',
                'use_tf_transform': True,
            }],
            output='screen'
        ),
        
        # Set up the lidar real-time mapper node with a delay to ensure transforms are available
        delayed_mapper,
        
        # Set up the IMU-Lidar yaw fusion node
        Node(
            package='sensor_fusion',
            executable='imu_lidar_yaw_fusion',
            name='imu_lidar_yaw_fusion',
            parameters=[{
                # Connection parameters
                'imu_topic': '/imu/data',
                'map_topic': '/realtime_map',  # Use realtime map instead of permanent map
                
                # Processing parameters
                'publish_rate': LaunchConfiguration('publish_rate'),
                
                # IMU-specific parameters
                'initial_yaw_offset': 0.0,
                'use_filtered_yaw': True,
                'yaw_filter_size': 10,
                'yaw_weight': 0.7,
            }],
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/realtime_map.rviz']],
            output='screen'
        ),
    ]) 