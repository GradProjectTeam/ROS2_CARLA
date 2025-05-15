#!/usr/bin/env python3
# Layer 6 Launch File: MPC Control

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
    
    # Vehicle parameters
    wheelbase = LaunchConfiguration('wheelbase', default='2.7')
    vehicle_width = LaunchConfiguration('vehicle_width', default='2.0')
    vehicle_length = LaunchConfiguration('vehicle_length', default='4.0')
    
    # Visualization parameters
    arrow_scale = LaunchConfiguration('arrow_scale', default='0.5')
    
    # Odometry parameters
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    odom_publish_rate = LaunchConfiguration('odom_publish_rate', default='30.0')
    
    # Controller parameters
    trajectory_topic = LaunchConfiguration('trajectory_topic', default='/planner/hybrid_astar/trajectory')
    control_topic = LaunchConfiguration('control_topic', default='/cmd_vel')
    control_rate = LaunchConfiguration('control_rate', default='20.0')
    
    # MPC parameters
    prediction_horizon = LaunchConfiguration('prediction_horizon', default='10')
    control_horizon = LaunchConfiguration('control_horizon', default='5')
    dt = LaunchConfiguration('dt', default='0.1')
    max_linear_velocity = LaunchConfiguration('max_linear_velocity', default='2.0')
    min_linear_velocity = LaunchConfiguration('min_linear_velocity', default='0.0')
    max_angular_velocity = LaunchConfiguration('max_angular_velocity', default='1.0')
    min_angular_velocity = LaunchConfiguration('min_angular_velocity', default='-1.0')
    max_linear_acceleration = LaunchConfiguration('max_linear_acceleration', default='1.0')
    max_angular_acceleration = LaunchConfiguration('max_angular_acceleration', default='0.5')
    
    # MPC weights
    w_x = LaunchConfiguration('w_x', default='1.0')
    w_y = LaunchConfiguration('w_y', default='1.0')
    w_theta = LaunchConfiguration('w_theta', default='0.5')
    w_v = LaunchConfiguration('w_v', default='0.1')
    w_linear_rate = LaunchConfiguration('w_linear_rate', default='0.1')
    w_angular_rate = LaunchConfiguration('w_angular_rate', default='0.1')
    
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
        DeclareLaunchArgument('planner_publish_rate', default_value='10.0', 
                             description='Planner publish rate in Hz'),
        
        # Vehicle parameters
        DeclareLaunchArgument('wheelbase', default_value='2.7', 
                             description='Vehicle wheelbase in meters'),
        DeclareLaunchArgument('vehicle_width', default_value='2.0', 
                             description='Vehicle width in meters'),
        DeclareLaunchArgument('vehicle_length', default_value='4.0', 
                             description='Vehicle length in meters'),
        
        # Visualization parameters
        DeclareLaunchArgument('arrow_scale', default_value='0.5', 
                             description='Scale factor for visualization arrows'),
        
        # Odometry parameters
        DeclareLaunchArgument('odom_topic', default_value='/odom', 
                             description='Odometry topic name'),
        DeclareLaunchArgument('odom_publish_rate', default_value='30.0', 
                             description='Odometry publish rate in Hz'),
        
        # Controller parameters
        DeclareLaunchArgument('trajectory_topic', default_value='/planner/hybrid_astar/trajectory', 
                             description='Topic from which to get trajectory data'),
        DeclareLaunchArgument('control_topic', default_value='/cmd_vel', 
                             description='Topic to publish control commands'),
        DeclareLaunchArgument('control_rate', default_value='20.0', 
                             description='Control loop frequency in Hz'),
        
        # MPC parameters
        DeclareLaunchArgument('prediction_horizon', default_value='10',
                             description='MPC prediction horizon length'),
        DeclareLaunchArgument('control_horizon', default_value='5',
                             description='MPC control horizon length'),
        DeclareLaunchArgument('dt', default_value='0.1',
                             description='Time step for MPC discretization'),
        DeclareLaunchArgument('max_linear_velocity', default_value='2.0',
                             description='Maximum linear velocity (m/s)'),
        DeclareLaunchArgument('min_linear_velocity', default_value='0.0',
                             description='Minimum linear velocity (m/s)'),
        DeclareLaunchArgument('max_angular_velocity', default_value='1.0',
                             description='Maximum angular velocity (rad/s)'),
        DeclareLaunchArgument('min_angular_velocity', default_value='-1.0',
                             description='Minimum angular velocity (rad/s)'),
        DeclareLaunchArgument('max_linear_acceleration', default_value='1.0',
                             description='Maximum linear acceleration (m/s^2)'),
        DeclareLaunchArgument('max_angular_acceleration', default_value='0.5',
                             description='Maximum angular acceleration (rad/s^2)'),
        
        # MPC weights
        DeclareLaunchArgument('w_x', default_value='1.0',
                             description='Weight for x position tracking'),
        DeclareLaunchArgument('w_y', default_value='1.0',
                             description='Weight for y position tracking'),
        DeclareLaunchArgument('w_theta', default_value='0.5',
                             description='Weight for heading tracking'),
        DeclareLaunchArgument('w_v', default_value='0.1',
                             description='Weight for velocity tracking'),
        DeclareLaunchArgument('w_linear_rate', default_value='0.1',
                             description='Weight for minimizing linear acceleration'),
        DeclareLaunchArgument('w_angular_rate', default_value='0.1',
                             description='Weight for minimizing angular acceleration'),
    ]
    
    # =============== Include Layer 5 Launch File ===============
    
    layer5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'layered_05_odometry.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'planner_publish_rate': planner_publish_rate,
            'wheelbase': wheelbase,
            'vehicle_width': vehicle_width,
            'vehicle_length': vehicle_length,
            'arrow_scale': arrow_scale,
            'odom_topic': odom_topic,
            'odom_publish_rate': odom_publish_rate
        }.items()
    )
    
    # =============== MPC Trajectory Controller Node ===============
    
    mpc_controller_node = Node(
        package='sensor_fusion',
        executable='mpc_trajectory_controller',
        name='mpc_trajectory_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'trajectory_topic': trajectory_topic,
            'control_topic': control_topic,
            'odom_topic': odom_topic,
            'control_rate': control_rate,
            
            # MPC parameters
            'prediction_horizon': prediction_horizon,
            'control_horizon': control_horizon,
            'dt': dt,
            'max_linear_velocity': max_linear_velocity,
            'min_linear_velocity': min_linear_velocity,
            'max_angular_velocity': max_angular_velocity,
            'min_angular_velocity': min_angular_velocity,
            'max_linear_acceleration': max_linear_acceleration,
            'max_angular_acceleration': max_angular_acceleration,
            'wheelbase': wheelbase,
            
            # MPC weights
            'w_x': w_x,
            'w_y': w_y,
            'w_theta': w_theta,
            'w_v': w_v,
            'w_linear_rate': w_linear_rate,
            'w_angular_rate': w_angular_rate
        }],
        output='screen'
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # Include Layer 5 (which includes Layers 1, 2, 3, and 4)
            layer5_launch,
            
            # Start MPC controller
            mpc_controller_node,
            
            # Log info
            LogInfo(msg="Layer 6 (MPC Control) launched successfully - Complete stack is running!")
        ]
    ) 