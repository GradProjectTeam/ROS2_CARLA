#!/usr/bin/env python3
"""
Launch file for integrated DWA-MPC direct pipeline.
This launch file configures the system to use a direct pipeline from DWA to MPC to TCP,
eliminating the DWA-MPC bridge component. It includes enhanced synchronization between
DWA and MPC components for optimal trajectory handling.

Author: Claude
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Include the costmap and waypoints launch file
    costmap_waypoints_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'integrated_costmap_waypoints.launch.py')
        ])
    )
    
    # Declare launch arguments
    map_frame = LaunchConfiguration('map_frame', default='local_map_link')
    vehicle_frame = LaunchConfiguration('vehicle_frame', default='base_link')
    tcp_port = LaunchConfiguration('tcp_port', default='12344')
    tcp_host = LaunchConfiguration('tcp_host', default='127.0.0.1')
    debug_level = LaunchConfiguration('debug_level', default='info')
    
    # Use a unified update rate for better synchronization
    unified_rate = LaunchConfiguration('unified_rate', default='15.0')
    
    # Declare LaunchArguments
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='local_map_link',
        description='Name of the map frame'
    )
    
    vehicle_frame_arg = DeclareLaunchArgument(
        'vehicle_frame',
        default_value='base_link',
        description='Name of the vehicle frame'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='12344',
        description='TCP port for command sending'
    )
    
    tcp_host_arg = DeclareLaunchArgument(
        'tcp_host',
        default_value='127.0.0.1',
        description='TCP host for command sending'
    )
    
    debug_level_arg = DeclareLaunchArgument(
        'debug_level',
        default_value='info',
        description='Debug level (debug, info, warn, error)'
    )
    
    unified_rate_arg = DeclareLaunchArgument(
        'unified_rate',
        default_value='15.0',
        description='Unified update rate for DWA and MPC components'
    )
    
    # Define the Enhanced DWA Planner node
    dwa_node = Node(
        package='sensor_fusion_2',
        executable='enhanced_dwa_planner_node',
        name='enhanced_dwa_planner',
        parameters=[{
            'map_frame_id': map_frame,
            'vehicle_frame_id': vehicle_frame,
            'waypoints_topic': '/carla/waypoints',
            'waypoint_markers_topic': '/carla/waypoint_markers',
            'binary_map_topic': '/combined_binary_map',
            'path_topic': '/dwa/path',  # Path for MPC to follow
            'cmd_vel_topic': '/dwa/cmd_vel',  # Direct velocity commands
            'parking_spots_topic': '/dwa/parking_spots',
            'publish_rate': unified_rate,  # Use unified rate for synchronization
            'max_speed': 25.0,
            'min_speed': 0.0,  # Prevent negative speed for better motion
            'max_yaw_rate': 0.6,  # Reduced from 0.8 to limit steering angle even more
            'max_accel': 5.0,  # Reduced from 50.0 to allow smoother acceleration
            'max_delta_yaw_rate': 0.3,  # Reduced from 0.4 for even smoother steering changes
            'dt': 0.1,  # Keep shorter timestep for more precise paths
            'predict_time': 3.0,  # Slightly reduced from 4.0 for better local planning
            'to_goal_cost_gain': 1.0,  # Slightly reduced from 1.2 to balance with other costs
            'speed_cost_gain': 0.8,  # Increased from 0.5 to prioritize maintaining speed
            'obstacle_cost_gain': 8.0,  # Reduced from 10.0 to be less sensitive to distant obstacles
            'path_following_gain': 0.8,  # Significantly increased to prioritize waypoint following
            'lookahead_distance': 8.0,  # Reduced from 10.0 for more responsive control
            'obstacle_threshold': 150,
            'safe_distance': 3.0,  # Reduced from 5.0 to be less conservative
            'default_lane_width': 3.2,
            'lane_width_factor': 0.6,
            'start_point_offset': 0.8,
            'min_obstacle_distance': 3.0,  # Reduced from 5.0 to be less conservative
            'path_smoothing_factor': 0.8,  # Higher smoothing for better MPC following
            'lateral_safety_margin': 1.0,
            'use_lane_based_obstacles': True,
            'ignore_obstacles_outside_lane': False,  # Detect all obstacles
            'strict_lane_obstacle_detection': True,
            'obstacle_influence_radius': 2.5,  # Reduced from 3.0 for more precise obstacle detection
            'obstacle_detection_range': 25.0,  # Reduced from 30.0 to focus on closer obstacles
            'obstacle_weight_decay': 0.7,
            'dynamic_obstacle_prediction': True,
            'adaptive_lookahead': True,
            'emergency_stop_enabled': True,
            'emergency_brake_distance': 5.0,  # Reduced from 8.0 to only stop for closer obstacles
            'obstacle_path_pruning': True,
            'obstacle_velocity_factor': 1.5,
            'min_obstacle_count': 2,  # Reduced from 3 to be more sensitive to real obstacles
            'debug_level': debug_level,
        }],
        output='screen'
    )
    
    # Define the MPC Controller node with improved synchronization with DWA
    mpc_node = Node(
        package='sensor_fusion_2',
        executable='mpc_controller',
        name='mpc_controller',
        parameters=[{
            'vehicle_frame_id': vehicle_frame,
            'map_frame_id': map_frame,
            'cmd_vel_topic': '/mpc/cmd_vel',
            'dwa_cmd_vel_topic': '/dwa/cmd_vel',  
            'path_topic': '/dwa/path',  # Match with DWA planner
            'obstacle_detected_topic': '/dwa/obstacle_detected',
            'direct_dwa_connection': True,  # Enable direct connection to DWA
            'bypass_trajectory_receiver': True,  # Skip trajectory_receiver
            'tcp_port': tcp_port,
            'tcp_host': tcp_host,
            'update_rate': unified_rate,  # Use unified rate for synchronization
            'prediction_horizon': 20,
            'control_horizon': 8,
            'dt': 0.1,  # Match with DWA dt for better sync
            'max_speed': 25.0,
            'max_acceleration': 2.0,  # Increased from 1.0 for more responsive acceleration
            'max_deceleration': 2.0,  # Increased from 1.0 for stronger braking when needed
            'min_deceleration': 0.05,
            'max_steering_angle': 0.6,  # Match with DWA max_yaw_rate
            'max_steering_rate': 0.3,  # Reduced from 0.4 to match DWA's max_delta_yaw_rate
            'wheelbase': 2.8,
            'position_weight': 1.0,
            'heading_weight': 0.8,
            'velocity_weight': 0.8,  # Slightly reduced from 1.0 to allow for obstacle response
            'steering_weight': 0.25,
            'acceleration_weight': 0.3,
            'deceleration_weight': 0.3,  # Increased from 0.2 to encourage smoother braking
            'jerk_weight': 0.08,
            'steering_rate_weight': 0.3,
            'brake_smoothness_weight': 0.5,
            'min_velocity': 0.0,
            'stop_threshold': 0.1,
            'deceleration_profile': 'smooth',
            'brake_transition_threshold': 0.2,
            'reconnect_interval': 1.0,
            'max_reconnect_attempts': 10,
            'use_waypoints': True,  # We're using DWA path instead
            'test_mode': False,
            'debug_level': debug_level,
            'progressive_braking': True,
            'prevent_unnecessary_stops': True,  # Enable to maintain forward motion when possible
            'maintain_continuous_motion': True,  # Enable to ensure vehicle keeps moving
            'respect_obstacle_detection': True,  # Enable to properly respond to obstacle detection
            'emergency_stop_distance': 2.5,  # Reduced from 3.0 to only stop for very close obstacles
            'obstacle_stop_threshold': 0.1,  # Balanced value for obstacle detection
            'strict_path_following': True,  # Enable strict path following for better waypoint tracking
            'hard_brake_on_obstacle': True,  # Enable hard braking for obstacles
            'obstacle_reaction_factor': 1.5,  # Reduced from 2.0 for more balanced reactions
            'dwa_path_priority': True,  # Always prioritize DWA paths
            'min_path_points': 3,  # Minimum points for a valid path
            'path_update_timeout': 0.5,  # Reduced for fresher paths
            'default_speed': 5.0,  # Increased default speed for when no valid trajectory is available
        }],
        output='screen'
    )
    
    # Create a launch description and populate
    ld = LaunchDescription()
    
    # Add the arguments
    ld.add_action(map_frame_arg)
    ld.add_action(vehicle_frame_arg)
    ld.add_action(tcp_port_arg)
    ld.add_action(tcp_host_arg)
    ld.add_action(debug_level_arg)
    ld.add_action(unified_rate_arg)
    
    # Add the included launch file
    ld.add_action(costmap_waypoints_launch)
    
    # Add the nodes
    ld.add_action(dwa_node)
    ld.add_action(mpc_node)
    
    return ld 