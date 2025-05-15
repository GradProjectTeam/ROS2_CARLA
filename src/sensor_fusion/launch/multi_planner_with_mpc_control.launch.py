#!/usr/bin/env python3
# Combined launch file for multi-planner comparison with MPC trajectory controller

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
    wheelbase = LaunchConfiguration('wheelbase', default='2.7')
    
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
        DeclareLaunchArgument('wheelbase', default_value='2.7',
                             description='Vehicle wheelbase in meters'),
        
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
            mpc_controller_node
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
            LogInfo(msg="Multi-planner with MPC trajectory control launched successfully")
        ]
    ) 