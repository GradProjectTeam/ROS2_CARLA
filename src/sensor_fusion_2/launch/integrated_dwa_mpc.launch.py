#!/usr/bin/env python3
"""
Integrated Enhanced DWA Planner with MPC System Launch File

This launch file combines the Enhanced DWA planner with the MPC system.
The DWA planner generates paths and the MPC system receives these paths
and sends control commands to the vehicle.

Author: Mostafa
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for integrated DWA-MPC system."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Include the integrated_costmap_waypoints.launch.py
    integrated_costmap_waypoints_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'integrated_costmap_waypoints.launch.py')
        ])
    )
    
    # Launch arguments
    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='local_map_link',
        description='Frame ID for the map'
    )
    
    vehicle_frame_arg = DeclareLaunchArgument(
        'vehicle_frame', default_value='base_link',
        description='Frame ID for the vehicle'
    )
    
    # TCP connection parameters
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port', default_value='12344',
        description='TCP port for MPC controller'
    )
    
    tcp_host_arg = DeclareLaunchArgument(
        'tcp_host', default_value='127.0.0.1',
        description='TCP host for MPC controller'
    )
    
    # Enhanced DWA Planner Node with revised parameters
    enhanced_dwa_node = Node(
        package='sensor_fusion_2',
        executable='enhanced_dwa_planner_node',
        name='enhanced_dwa_planner',
        parameters=[{
            'map_frame_id': LaunchConfiguration('map_frame'),
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame'),
            'waypoints_topic': '/carla/waypoints',
            'waypoint_markers_topic': '/carla/waypoint_markers',
            'binary_map_topic': '/combined_binary_map',
            'path_topic': '/dwa/planned_path',
            'cmd_vel_topic': '/dwa/cmd_vel',
            'parking_spots_topic': '/dwa/parking_spots',
            'publish_rate': 15.0,  # Increased from 10.0 for more responsive planning
            'max_speed': 25.0,  # Reduced from 30.0 for safer operation
            'min_speed': -2.5,  # Reduced from -4.0 for more controlled reversing
            'max_yaw_rate': 1.2,  # Reduced from 1.5 for smoother turning
            'max_accel': 5.0,  # Reduced from 8.0 for smoother acceleration
            'max_delta_yaw_rate': 0.6,  # Reduced from 0.8 for smoother steering changes
            'dt': 0.15,  # Reduced from 0.2 for finer trajectory prediction
            'predict_time': 4.0,  # Increased from 3.0 for longer-term planning
            'to_goal_cost_gain': 1.2,  # Reduced from 1.5 to balance with obstacle avoidance
            'speed_cost_gain': 0.5,  # Reduced from 0.7 to prioritize safety over speed
            'obstacle_cost_gain': 2.5,  # Increased from 1.8 for stronger obstacle avoidance
            'path_following_gain': 0.015,  # Increased from 0.01 for better path adherence
            'lookahead_distance': 10.0,  # Increased from 8.0 for earlier path planning
            'obstacle_threshold': 180,  # Reduced from 220 to detect obstacles earlier
            'safe_distance': 0.1,  # Fixed from 0.2 to 1.2 for larger safety margin
            'default_lane_width': 3.2,  # Slightly increased from 3.0 for more space
            'lane_width_factor': 0.65,  # Reduced from 0.7 to stay more centered in lane
            'start_point_offset': 0.8,  # Reduced from 1.0 to start planning closer to vehicle
            'min_obstacle_distance': 0.8,  # Increased from 0.5 for better obstacle clearance
            'path_smoothing_factor': 0.8,  # Increased from 0.7 for smoother paths
            'lateral_safety_margin': 0.6,  # Increased from 0.4 for wider obstacle clearance
            'use_lane_based_obstacles': True,
            'ignore_obstacles_outside_lane': True,  # Keep ignoring obstacles outside lane
            'obstacle_influence_radius': 2.0,  # Increased from 1.5 for wider obstacle influence
            'obstacle_detection_range': 15.0,  # Added parameter for longer obstacle detection
            'obstacle_weight_decay': 0.85,  # Added parameter for distance-based obstacle weight decay
            'dynamic_obstacle_prediction': True,  # Added parameter to enable prediction of moving obstacles
            'adaptive_lookahead': True,  # Added parameter to adjust lookahead based on speed
        }],
        output='screen'
    )
    
    # Trajectory Receiver Node (uses waypoints and DWA path)
    trajectory_receiver_node = Node(
        package='sensor_fusion_2',
        executable='trajectory_receiver',
        name='trajectory_receiver',
        parameters=[{
            'update_rate': 15.0,  # Increased from 10.0 to match DWA planner
            'path_topic': '/dwa/planned_path',
            'waypoints_topic': '/carla/waypoints',
            'use_waypoints': True,
            'start_point_offset': 1.0,  # Reduced from 1.2 for better alignment with DWA
            'min_path_points': 6,  # Increased from 5 for more complete paths
            'filter_close_points': True,
            'lane_width': 3.2,  # Matched with DWA parameter
            'smoothing_factor': 0.75,  # Increased from 0.6 for smoother trajectories
            'min_point_spacing': 0.4,  # Reduced from 0.5 for finer path resolution
            'max_point_spacing': 2.0,  # Reduced from 2.5 for more consistent spacing
            'path_interpolation': True,  # Added parameter to enable path interpolation
        }],
        output='screen'
    )
    
    # MPC Controller Node with modified parameters to prevent binary braking
    mpc_controller_node = Node(
        package='sensor_fusion_2',
        executable='mpc_controller',
        name='mpc_controller',
        parameters=[{
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame'),
            'map_frame_id': LaunchConfiguration('map_frame'),
            'cmd_vel_topic': '/mpc/cmd_vel',
            'trajectory_topic': 'reference_trajectory',
            'update_rate': 25.0,  # Increased from 20.0 for more responsive control
            'prediction_horizon': 20,
            'control_horizon': 8,
            'dt': 0.1,
            'max_speed': 25.0,
            'max_acceleration': 1.0,  # Further reduced from 1.2 for smoother acceleration
            'max_deceleration': 0.5,  # Reduced from 0.6 for smoother braking
            'min_deceleration': 0.05,  # Reduced from 0.1 for finer deceleration control
            'max_steering_angle': 0.8,  # Reduced from 1.0 for more conservative steering
            'max_steering_rate': 0.4,  # Reduced from 0.5 for smoother steering changes
            'wheelbase': 2.8,
            'position_weight': 1.0,  # Reduced from 1.2 to be less strict on position
            'heading_weight': 0.8,  # Reduced from 1.0 to be less strict on heading
            'velocity_weight': 0.5,  # Increased from 0.4 for better velocity tracking
            'steering_weight': 0.25,  # Increased from 0.2 for smoother steering
            'acceleration_weight': 0.3,  # Increased from 0.2 for smoother acceleration
            'deceleration_weight': 0.4,  # Increased from 0.3 for smoother deceleration
            'jerk_weight': 0.08,  # Increased from 0.05 to further penalize rapid changes
            'steering_rate_weight': 0.3,  # Increased from 0.25 for smoother steering transitions
            'brake_smoothness_weight': 0.5,  # Increased from 0.4 for smoother braking
            'min_velocity': 1.5,  # Reduced from 2.0 for finer low-speed control
            'stop_threshold': 0.3,  # Reduced from 0.5 for more precise stopping
            'deceleration_profile': 'smooth',
            'brake_transition_threshold': 0.2,  # Reduced from 0.3 for earlier brake transitions
            # TCP connection parameters
            'tcp_port': LaunchConfiguration('tcp_port'),
            'tcp_host': LaunchConfiguration('tcp_host'),
            'reconnect_interval': 2.0,
            'max_reconnect_attempts': 5,
            'use_waypoints': True,
            # Test mode parameters
            'test_mode': False,
            'test_throttle': 0.3,
            'test_duration': 5.0,
            'debug_level': 'info',
            'progressive_braking': True,  # Added parameter for gradual braking application
        }],
        output='screen'
    )
    
    # DWA-MPC Bridge Node
    dwa_mpc_bridge_node = Node(
        package='sensor_fusion_2',
        executable='dwa_mpc_bridge',
        name='dwa_mpc_bridge',
        parameters=[{
            'update_rate': 25.0,  # Increased from 20.0 to match MPC controller
            'dwa_cmd_topic': '/dwa/cmd_vel',
            'mpc_cmd_topic': '/mpc/cmd_vel',
            'output_cmd_topic': '/cmd_vel',
            'dwa_path_topic': '/dwa/planned_path',
            'min_path_length': 8,  # Increased from 6 for more complete paths
            'min_path_distance': 15.0,  # Increased from 12.0 for longer planning
            'max_path_angle': 0.4,  # Reduced from 0.5 for smoother path transitions
            'hysteresis_threshold': 0.2,  # Reduced from 0.25 for faster switching when needed
            'blend_time': 1.0,  # Reduced from 1.2 for faster transitions
            'lane_width': 3.2,  # Matched with DWA parameter
            'prefer_mpc': True,
            'min_speed': 1.5,  # Reduced from 2.0 to match MPC controller
            'obstacle_influence_radius': 2.0,  # Increased from 1.5 to match DWA
            'smooth_braking': True,
            'brake_smoothing_factor': 0.8,  # Increased from 0.7 for smoother braking
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        # Include the integrated costmap waypoints launch file first
        integrated_costmap_waypoints_launch,
        
        # Then add the rest of the nodes
        map_frame_arg,
        vehicle_frame_arg,
        tcp_port_arg,
        tcp_host_arg,
        enhanced_dwa_node,
        trajectory_receiver_node,
        mpc_controller_node,
        dwa_mpc_bridge_node,
    ]) 