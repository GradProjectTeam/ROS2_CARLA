#!/usr/bin/env python3
# Test visualization launch file for multi-planner visualization

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('sensor_fusion')
    
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Configure RViz
    rviz_config = os.path.join(package_dir, 'rviz', 'multi_planner_fixed.rviz')
    
    # Static TF publishers
    tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
    )
    
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    
    tf_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_baselink',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
    )
    
    tf_baselink_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_baselink_lidar',
        arguments=['0', '0', '1.0', '0', '0', '0', 'base_link', 'lidar'],
    )
    
    # Test path publishers for different planners
    hybrid_astar_path_publisher = Node(
        package='sensor_fusion',
        executable='test_path_publisher.py',
        name='hybrid_astar_path_publisher',
        parameters=[{
            'path_topic': '/planning/hybrid_astar/path',
            'path_frame_id': 'map',
            'publish_rate': 2.0,
        }],
    )
    
    astar_path_publisher = Node(
        package='sensor_fusion',
        executable='test_path_publisher.py',
        name='astar_path_publisher',
        parameters=[{
            'path_topic': '/planning/astar/path',
            'path_frame_id': 'map',
            'publish_rate': 2.0,
        }],
    )
    
    dwa_path_publisher = Node(
        package='sensor_fusion',
        executable='test_path_publisher.py',
        name='dwa_path_publisher',
        parameters=[{
            'path_topic': '/planning/dwa/path',
            'path_frame_id': 'map',
            'publish_rate': 2.0,
        }],
    )
    
    mpc_path_publisher = Node(
        package='sensor_fusion',
        executable='test_path_publisher.py',
        name='mpc_path_publisher',
        parameters=[{
            'path_topic': '/planning/mpc/path',
            'path_frame_id': 'map',
            'publish_rate': 2.0,
        }],
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time,
        tf_world_map,
        tf_map_odom,
        tf_odom_baselink,
        tf_baselink_lidar,
        hybrid_astar_path_publisher,
        # Add a small delay to stagger the path publishers for visual effect
        TimerAction(
            period=1.0,
            actions=[astar_path_publisher]
        ),
        TimerAction(
            period=2.0,
            actions=[dwa_path_publisher]
        ),
        TimerAction(
            period=3.0,
            actions=[mpc_path_publisher]
        ),
        rviz_node,
    ]) 