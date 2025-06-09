#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Include the base sensor fusion launch file
    # This provides all the sensor fusion capabilities
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'fast_imu_lidar_fusion_lidar_only.launch.py')]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'lidar_tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'imu_tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'imu_tcp_port': LaunchConfiguration('imu_tcp_port'),
            'map_resolution': LaunchConfiguration('map_resolution'),  # Pass map resolution
            'vehicle_length': LaunchConfiguration('vehicle_length'),  # Pass vehicle dimensions
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'vehicle_height': LaunchConfiguration('vehicle_height'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),  # Pass obstacle threshold
        }.items()
    )
    
    # =========== HIGHWAY NAVIGATION CONFIGURATION ===========
    # Highway Follower Node - Autonomous highway navigation without goals
    highway_follower_node = Node(
        package='sensor_fusion',
        executable='highway_follower_node',
        name='highway_follower',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'local_path_topic': '/highway_path',
            'cmd_vel_topic': '/cmd_vel',
            'map_topic': '/realtime_map',
            'visualization_topic': '/highway_visualization',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'planning_frequency': 10.0,
            'visualization_frequency': 2.0,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'safe_distance': 5.0,
            'lane_width': 3.5,  # Standard highway lane width
            'lane_detection_range': 30.0,  # How far to look for lane detection
            
            # Highway following parameters
            'forward_distance': 30.0,  # Distance to project forward
            'lateral_weight': 1.5,  # Weight for lateral positioning
            'forward_weight': 1.0,  # Weight for forward movement
            'obstacle_weight': 2.0,  # Weight for obstacle avoidance
            
            # DWA parameters
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': 5.0,   # Minimum speed
            'max_yaw_rate': 0.8,  # Reduced for highway
            'max_accel': 2.0,
            'max_delta_yaw_rate': 0.5,  # Reduced for highway
            'dt': 0.1,
            'predict_time': 5.0,  # Increased for highway
            'speed_cost_gain': 0.1,  # Prefer higher speeds
            'obstacle_cost_gain': 2.0,  # Higher obstacle avoidance
            'lane_following_gain': 1.5,  # Lane following importance
            'lookahead_distance': 20.0,  # Look far ahead
        }],
        output='screen'
    )
    
    # Path Visualization Node - Visualizes the highway path
    path_visualization_node = Node(
        package='sensor_fusion',
        executable='path_visualization_node',
        name='path_visualization',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_path_topic': '/highway_path',  # Use highway path as global path
            'local_path_topic': '/highway_path',   # Same as global for visualization
            'map_frame_id': 'map',
            'global_path_color_r': 0.0,
            'global_path_color_g': 0.0,
            'global_path_color_b': 1.0,
            'global_path_color_a': 0.8,
            'local_path_color_r': 0.0,
            'local_path_color_g': 1.0,
            'local_path_color_b': 0.0,
            'local_path_color_a': 1.0,
            'path_line_width': 0.1,
            'path_lifetime': 0.0,  # 0 means persistent
        }],
        output='screen'
    )
    
    # Create the launch description with all nodes and argument declarations
    return LaunchDescription([
        # ================= LAUNCH ARGUMENTS =================
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
        
        # Motion planning parameters
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time'
        ),
        
        # Map parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.8',
            description='Resolution of the map in meters per cell'
        ),
        
        # Obstacle threshold parameter
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='70',
            description='Threshold for marking cells as obstacles (0-100)'
        ),
        
        # Vehicle dimensions
        DeclareLaunchArgument(
            'vehicle_length',
            default_value='5.0',
            description='Length of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_width',
            default_value='2.5',
            description='Width of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_height',
            default_value='2.2',
            description='Height of vehicle (meters)'
        ),
        
        # Highway specific parameters
        DeclareLaunchArgument(
            'max_speed',
            default_value='20.0',
            description='Maximum speed for highway driving (m/s)'
        ),
        
        # =============== NODE INSTANTIATION ===============
        # Include the base sensor fusion launch
        base_launch,
        
        # Launch highway navigation nodes
        highway_follower_node,
        path_visualization_node,
        
        # Launch RViz for visualization with highway navigation config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/highway_navigation.rviz']],
            output='screen'
        ),
    ]) 