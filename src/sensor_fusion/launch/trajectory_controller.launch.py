#!/usr/bin/env python3
# Trajectory Controller Launch File

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # =============== Launch Arguments ===============
    
    # Simulation parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # Controller parameters
    trajectory_topic = LaunchConfiguration('trajectory_topic', default='/planner/hybrid_astar/trajectory')
    control_topic = LaunchConfiguration('control_topic', default='/cmd_vel')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    control_rate = LaunchConfiguration('control_rate', default='20.0')
    
    # Launch arguments
    package_dir = get_package_share_directory('sensor_fusion')
    
    # Declare launch arguments
    launch_args = [
        # Simulation parameters
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        
        # Controller parameters
        DeclareLaunchArgument('trajectory_topic', default_value='/planner/hybrid_astar/trajectory', 
                             description='Topic from which to get trajectory data'),
        DeclareLaunchArgument('control_topic', default_value='/cmd_vel', 
                             description='Topic to publish control commands'),
        DeclareLaunchArgument('odom_topic', default_value='/odom', 
                             description='Topic for odometry data'),
        DeclareLaunchArgument('control_rate', default_value='20.0', 
                             description='Control loop frequency in Hz'),
    ]
    
    # =============== Trajectory Controller Node ===============
    
    trajectory_controller_node = Node(
        package='sensor_fusion',
        executable='trajectory_controller',
        name='trajectory_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'trajectory_topic': trajectory_topic,
            'control_topic': control_topic,
            'odom_topic': odom_topic,
            'control_rate': control_rate,
            'position_tolerance': 0.2,
            'heading_tolerance': 0.1,
            'lookahead_time': 0.5,
            # PID controller gains
            'k_p_linear': 1.0,
            'k_i_linear': 0.1,
            'k_d_linear': 0.05,
            'k_p_angular': 1.0,
            'k_i_angular': 0.1,
            'k_d_angular': 0.05
        }],
        output='screen'
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # Start the trajectory controller node
            trajectory_controller_node,
            
            # Log info
            LogInfo(msg="Trajectory Controller launched successfully")
        ]
    ) 