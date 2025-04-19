#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Create transform publisher nodes for TF tree
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
    
    # Create the test_navigation node with IMU integration
    test_navigation_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='test_navigation',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'use_filtered_yaw': LaunchConfiguration('use_filtered_yaw'),
            'yaw_filter_size': LaunchConfiguration('yaw_filter_size'),
            'yaw_weight': LaunchConfiguration('yaw_weight'),
            'goal_distance': LaunchConfiguration('goal_distance'),
            'yaw_offset': LaunchConfiguration('yaw_offset'),
        }],
        output='screen'
    )
    
    # Create the IMU euler visualization node
    imu_euler_visualizer_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'reconnect_interval': 3.0,
            'frame_id': 'imu_link',
            'world_frame_id': 'world',
            'filter_window_size': 3,
        }],
        output='screen'
    )
    
    # Create the real-time mapper node with a short delay
    lidar_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_realtime_mapper',
        parameters=[{
            # Map parameters
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'center_on_vehicle': True,
            
            # Processing parameters
            'publish_rate': 10.0,
            'process_rate': 20.0,
            
            # Bayesian update weights
            'hit_weight': 0.8,
            'miss_weight': 0.4,
            'prior_weight': 0.5,
            
            # Other parameters
            'decay_rate': 0.95,
            'temporal_memory': 0.5,
            'enable_map_reset': True,
            'map_reset_interval': 20.0,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
        }],
        output='screen'
    )
    
    # Delay the start of the mapper (to ensure transforms are available)
    delayed_mapper = TimerAction(
        period=2.0,
        actions=[lidar_mapper_node]
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'realtime_map.rviz')]],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='Topic for IMU data'
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/realtime_map',
            description='Topic for map data'
        ),
        DeclareLaunchArgument(
            'use_filtered_yaw',
            default_value='true',
            description='Whether to use filtered IMU yaw (smoothed)'
        ),
        DeclareLaunchArgument(
            'yaw_filter_size',
            default_value='5',
            description='Size of IMU yaw filter buffer'
        ),
        DeclareLaunchArgument(
            'yaw_weight',
            default_value='0.8',
            description='Weight of IMU yaw data in fusion'
        ),
        DeclareLaunchArgument(
            'goal_distance',
            default_value='5.0',
            description='Distance to goal pose in meters'
        ),
        DeclareLaunchArgument(
            'yaw_offset',
            default_value='3.14159',  # Pi (180 degrees)
            description='Offset to correct IMU yaw orientation in radians'
        ),
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
            'map_resolution',
            default_value='0.1',
            description='Resolution of the map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='50.0',
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='50.0',
            description='Height of the map in meters'
        ),
        
        # Nodes
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_euler_visualizer_node,
        delayed_mapper,
        test_navigation_node,
        rviz_node,
    ]) 