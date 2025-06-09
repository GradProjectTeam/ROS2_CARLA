#!/usr/bin/env python3
# Modified multi-planner comparison launch file with fixes for visualization

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, TimerAction
)
from launch.conditions import IfCondition
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
    rviz_config = os.path.join(package_dir, 'rviz', 'multi_planner_fixed.rviz')
    
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
    
    # =============== Basic Map Node ===============
    
    # Simple map publisher node
    map_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='realtime_mapper',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 5.0,
            'map_size_x': 100.0,
            'map_size_y': 100.0,
            'map_resolution': grid_size,
            'vehicle_length': 4.0,
            'vehicle_width': 2.0,
            'wait_for_transform': False,
            'publish_static_map': True,
            'disable_tf_lookup': True,
            'fixed_frame': 'map'
        }]
    )
    
    # =============== Simple Path Publishers ===============
    
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
    
    # =============== Visualization ===============
    
    # RViz node with configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # =============== Sequence ===============
    
    # Start path publishers after the map
    delayed_path_publishers = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting path publishers..."),
            hybrid_astar_path_pub,
            astar_path_pub,
            dwa_path_pub,
            mpc_path_pub
        ]
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # TF tree setup - launch all transforms first
            tf_static_world_map,
            tf_static_map_odom,
            tf_static_odom_baselink,
            tf_static_baselink_lidar,
            
            # Launch map node
            map_node,
            
            # Start RViz
            rviz_node,
            
            # Start path publishers
            delayed_path_publishers,
            
            # Start goal publisher
            goal_publisher_node,
            
            # Log info
            LogInfo(msg="Multi-planner visualization launched successfully")
        ]
    ) 