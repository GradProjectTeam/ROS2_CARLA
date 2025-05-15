#!/usr/bin/env python3
# Combined launch file for multi-planner comparison with trajectory controller

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, TimerAction, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        
        # Map parameters
        DeclareLaunchArgument('map_topic', default_value='/realtime_map', description='Map topic name'),
        DeclareLaunchArgument('grid_size', default_value='0.2', description='Grid size in meters'),
        
        # Planner parameters
        DeclareLaunchArgument('planner_publish_rate', default_value='10.0', description='Planner publish rate in Hz'),
        
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
    
    # =============== Include Multi-Planner Launch File ===============
    
    multi_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'multi_planner_comparison_fixed.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'planner_publish_rate': planner_publish_rate
        }.items()
    )
    
    # =============== Path to Trajectory Converter Node ===============
    
    path_to_trajectory_converter = Node(
        package='sensor_fusion',
        executable='path_to_trajectory_converter',
        name='path_to_trajectory_converter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_velocity': 2.0,  # m/s
            'max_acceleration': 1.0,  # m/s^2
            'target_dt': 0.1,  # seconds between trajectory points
            'wheelbase': 2.7,  # meters
            'vehicle_width': 2.0,  # meters
            'vehicle_length': 4.0,  # meters
            'min_radius': 3.0  # minimum turning radius in meters
        }]
    )
    
    # =============== Trajectory Visualizer Node ===============
    
    trajectory_visualizer = Node(
        package='sensor_fusion',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_width': 2.0,  # meters
            'vehicle_length': 4.0,  # meters
            'wheelbase': 2.7,  # meters
            'arrow_scale': 0.5  # size of velocity/steering arrows
        }]
    )
    
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
    
    # =============== Odometry Publisher for Testing ===============
    
    # This node publishes odometry data for testing when real odometry isn't available
    odom_publisher = Node(
        package='sensor_fusion',
        executable='current_pose_publisher',
        name='odom_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 30.0,  # Hz
            'topic': odom_topic,
            'frame_id': 'map',
            'child_frame_id': 'base_link'
        }]
    )
    
    # =============== Sequence ===============
    
    # Start trajectory-related nodes after the planners
    delayed_trajectory_nodes = TimerAction(
        period=7.0,  # Wait 7 seconds for the planners to start
        actions=[
            LogInfo(msg="Starting trajectory processing nodes..."),
            path_to_trajectory_converter,
            trajectory_visualizer,
            odom_publisher,
            trajectory_controller_node
        ]
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # Launch multi-planner setup
            multi_planner_launch,
            
            # Start trajectory processing nodes after a delay
            delayed_trajectory_nodes,
            
            # Log info
            LogInfo(msg="Multi-planner with trajectory control launched successfully")
        ]
    ) 