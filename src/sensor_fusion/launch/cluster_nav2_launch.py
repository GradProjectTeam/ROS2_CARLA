#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Create transform publisher nodes with higher publish frequency for faster updates
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}],
    )
    
    # Add back the map_to_odom static transform for Nav2
    # Nav2 expects map -> odom -> base_link
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}],
    )
    
    # Static transform from odom to base_link
    odom_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}],
    )
    
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}],
    )
    
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}],
    )
    
    # Create the cluster to map converter node
    cluster_to_map_node = Node(
        package='sensor_fusion',
        executable='cluster_to_nav2_map_converter',
        name='cluster_to_nav2_map_converter',
        parameters=[{
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width': LaunchConfiguration('map_width_meters'),
            'map_height': LaunchConfiguration('map_height_meters'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'cluster_topic': '/lidar/cubes',
            'point_cloud_topic': '/lidar/points',
            'map_topic': '/map',  # Standard topic that Nav2 expects
            'publish_rate': 5.0,   # Standard map publishing rate for Nav2
            'obstacle_inflation': LaunchConfiguration('obstacle_inflation'),
            'free_space_range': LaunchConfiguration('free_space_range'),
            'map_persistence': LaunchConfiguration('map_persistence'),
            'use_distance_filter': LaunchConfiguration('use_distance_filter'),
            'max_obstacle_distance': LaunchConfiguration('max_obstacle_distance'),
            'update_free_space_only': LaunchConfiguration('update_free_space_only'),
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
            'tf_timeout': LaunchConfiguration('tf_timeout'),
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    
    # Set up the IMU Euler Visualizer node
    imu_visualizer_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'reconnect_interval': 2.0,
            'frame_id': 'imu_link',
            'world_frame_id': 'world',
            'filter_window_size': 2,
            'queue_size': 10,
            'publish_rate': 100.0,
        }],
        output='screen'
    )
    
    # Set up the LiDAR listener node
    lidar_clusters_node = Node(
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
            'vehicle_filter_radius': 2.0,
            'vehicle_filter_height_min': -1.0,
            'vehicle_filter_height_max': 1.5,
            'min_range_filter': 1.0,
            'vehicle_filter_x_offset': -1.5,
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'queue_size': 10,
            'publish_rate': 40.0,
            'min_points_per_cluster': 7,
            'max_cluster_distance': 0.35,
        }],
        output='screen'
    )
    
    # Set up the IMU-Lidar yaw fusion node with explicit TF publishing
    imu_lidar_fusion_node = Node(
        package='sensor_fusion',
        executable='imu_lidar_yaw_fusion',
        name='imu_lidar_yaw_fusion',
        parameters=[{
            # Connection parameters
            'imu_topic': '/imu/data',
            'map_topic': '/map',  # Updated to use the map topic
            
            # Processing parameters - higher rate for faster updates
            'publish_rate': 50.0,
            'publish_tf': True,
            'yaw_filter_size': 3,
            'yaw_weight': 0.95,
            'adaptive_fusion': True,
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'override_static_tf': True,
            
            # TF parameters
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
            'tf_timeout': LaunchConfiguration('tf_timeout'),
            'wait_for_transform': True,
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    
    # Set up RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'path_planning.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        # TCP connection parameters for IMU and LiDAR
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='0.0.0.0',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='0.0.0.0',
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        
        # Map Parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.05',  # Higher resolution for better visualization
            description='Resolution of the map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='60.0',
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='60.0',
            description='Height of the map in meters'
        ),
        
        # Navigation Parameters
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='70',  # Higher threshold for clearer obstacles
            description='Threshold for considering a cell as an obstacle (0-100)'
        ),
        DeclareLaunchArgument(
            'obstacle_inflation',
            default_value='0.3',
            description='Amount to inflate obstacles for safer path planning (meters)'
        ),
        DeclareLaunchArgument(
            'free_space_range',
            default_value='2.0',
            description='Range of free space to mark around obstacles (meters)'
        ),
        DeclareLaunchArgument(
            'map_persistence',
            default_value='5.0',
            description='How long obstacles remain on map without updates (seconds)'
        ),
        
        DeclareLaunchArgument(
            'use_distance_filter',
            default_value='true',
            description='Whether to filter out obstacles beyond a certain distance'
        ),
        
        DeclareLaunchArgument(
            'max_obstacle_distance',
            default_value='25.0',
            description='Maximum distance to include obstacles (meters)'
        ),
        
        DeclareLaunchArgument(
            'update_free_space_only',
            default_value='false',
            description='Whether to only update free space between robot and obstacles'
        ),
        
        # TF parameters
        DeclareLaunchArgument(
            'tf_buffer_duration',
            default_value='4.0',
            description='Duration of TF buffer (seconds)'
        ),
        DeclareLaunchArgument(
            'tf_timeout',
            default_value='0.2',
            description='Timeout for TF lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'transform_tolerance',
            default_value='0.3',
            description='Tolerance for transform lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time'
        ),

        # Visualization parameters
        DeclareLaunchArgument(
            'point_size',
            default_value='1.0',
            description='Size of individual point markers'
        ),
        DeclareLaunchArgument(
            'center_size',
            default_value='2.0',
            description='Size of cluster center markers'
        ),
        DeclareLaunchArgument(
            'use_convex_hull',
            default_value='false',
            description='Whether to display 2D convex hull around clusters'
        ),
        
        # TF static transform publishers
        world_to_map_node,
        map_to_odom_node,
        odom_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Sensor nodes
        imu_visualizer_node,
        lidar_clusters_node,
        
        # Conversion node
        cluster_to_map_node,
        
        # Fusion nodes
        imu_lidar_fusion_node,
        
        # Visualization
        rviz_node,
    ])