#!/usr/bin/env python3
# Layer 2 Launch File: Path Planning

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # =============== Launch Arguments ===============
    
    # Simulation parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # Map parameters  
    map_topic = LaunchConfiguration('map_topic', default='/realtime_map')
    grid_size = LaunchConfiguration('grid_size', default='0.2')  # meters per grid cell
    
    # Planning parameters
    planner_publish_rate = LaunchConfiguration('planner_publish_rate', default='10.0')  # Hz
    
    # Launch arguments
    package_dir = get_package_share_directory('sensor_fusion')
    
    # Declare launch arguments
    launch_args = [
        # Simulation parameters
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        
        # Map parameters
        DeclareLaunchArgument('map_topic', default_value='/realtime_map', description='Map topic name'),
        DeclareLaunchArgument('grid_size', default_value='0.2', description='Grid size in meters'),
        
        # Planner parameters
        DeclareLaunchArgument('planner_publish_rate', default_value='10.0', description='Planner publish rate in Hz'),
    ]
    
    # =============== Include Layer 1 Launch File ===============
    
    layer1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'layered_01_tf_map.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size
        }.items()
    )
    
    # =============== Path Publishers ===============
    
    # Simple path publishers (one for each planner type)
    hybrid_astar_path_pub = Node(
        package='sensor_fusion',
        executable='test_path_publisher',
        name='hybrid_astar_path_pub',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': planner_publish_rate,
            'path_topic': '/planner/hybrid_astar/path',
            'path_frame_id': 'map',
            'path_color': [0.1, 1.0, 0.1, 1.0]  # Green
        }]
    )
    
    astar_path_pub = Node(
        package='sensor_fusion',
        executable='test_path_publisher',
        name='astar_path_pub',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': planner_publish_rate,
            'path_topic': '/planner/astar/path',
            'path_frame_id': 'map',
            'path_color': [0.0, 0.0, 1.0, 1.0]  # Blue
        }]
    )
    
    dwa_path_pub = Node(
        package='sensor_fusion',
        executable='test_path_publisher',
        name='dwa_path_pub',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': planner_publish_rate,
            'path_topic': '/planner/dwa/path',
            'path_frame_id': 'map',
            'path_color': [1.0, 0.0, 0.0, 1.0]  # Red
        }]
    )
    
    mpc_path_pub = Node(
        package='sensor_fusion',
        executable='test_path_publisher',
        name='mpc_path_pub',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': planner_publish_rate,
            'path_topic': '/planner/mpc/path',
            'path_frame_id': 'map',
            'path_color': [1.0, 0.0, 1.0, 1.0]  # Purple
        }]
    )
    
    # Goal publisher node
    goal_publisher_node = Node(
        package='sensor_fusion',
        executable='test_goal_pose_publisher',
        name='test_goal_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame': 'map'
        }]
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # Include Layer 1
            layer1_launch,
            
            # Start path publishers
            hybrid_astar_path_pub,
            astar_path_pub,
            dwa_path_pub,
            mpc_path_pub,
            
            # Start goal publisher
            goal_publisher_node,
            
            # Log info
            LogInfo(msg="Layer 2 (Path Planning) launched successfully")
        ]
    ) 