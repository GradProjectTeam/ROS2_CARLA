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
    
    # Create the launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time'
        ),
        
        # Launch global planner node
        Node(
            package='sensor_fusion',
            executable='global_planner_node',
            name='global_planner',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'planner_type': 'astar'},
                {'planning_frequency': 1.0}
            ],
            output='screen'
        ),
        
        # Launch local planner node
        Node(
            package='sensor_fusion',
            executable='local_planner_node',
            name='local_planner',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'planning_frequency': 5.0}
            ],
            output='screen'
        ),
        
        # Launch path visualization node
        Node(
            package='sensor_fusion',
            executable='path_visualization_node',
            name='path_visualization',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),
        
        # Launch goal publisher node
        Node(
            package='sensor_fusion',
            executable='goal_publisher_node',
            name='goal_publisher',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen'
        ),
        
        # Launch robot controller node
        Node(
            package='sensor_fusion',
            executable='robot_controller_node',
            name='robot_controller',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'control_frequency': 10.0}
            ],
            output='screen'
        )
    ]) 