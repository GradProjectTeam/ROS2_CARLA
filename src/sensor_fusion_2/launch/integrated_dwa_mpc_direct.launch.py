#!/usr/bin/env python3
"""
Launch file for MPC waypoint following.
This launch file configures the system to use MPC to follow waypoints directly,
with DWA planner disabled. This allows testing of the MPC controller's ability
to follow waypoints without DWA's interference.

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
    debug_level = LaunchConfiguration('debug_level', default='debug')
    
    # Use a unified update rate for better synchronization
    unified_rate = LaunchConfiguration('unified_rate', default='20.0')
    
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
        default_value='debug',
        description='Debug level (debug, info, warn, error)'
    )
    
    unified_rate_arg = DeclareLaunchArgument(
        'unified_rate',
        default_value='20.0',
        description='Unified update rate for MPC component'
    )
    
    velocity_listener_node = Node(
        package='sensor_fusion_2',
        executable='velocity_listener',
        name='velocity_listener',
        parameters=[{
            'tcp_ip': '0.0.0.0',
            'tcp_port': 12346,
            'velocity_topic': '/carla/ego_vehicle/velocity',
            'velocity_vector_topic': '/carla/ego_vehicle/velocity_vector',
            'publish_rate': 30.0,
            'verbose_logging': False,
        }],
        output='screen'
    )
    
    # Add velocity visualizer node
    velocity_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='velocity_visualizer',
        name='velocity_visualizer',
        parameters=[{
            'input_topic': 'vehicle_velocity',
            'output_topic': 'velocity_visualization',
            'frame_id': vehicle_frame,
            'scale_factor': 1.0,
            'arrow_shaft_diameter': 0.05,
            'arrow_head_diameter': 0.1,
            'max_velocity': 10.0
        }],
        output='screen'
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
            'binary_map_topic': '/combined_binary_map',  # Combined map with obstacles (black) and lanes (grey)
            'path_topic': '/dwa/path',
            'cmd_vel_topic': '/dwa/cmd_vel',
            'publish_rate': unified_rate,
            # Debug and logging settings
            'debug_level': 'debug',  # Force debug level for this node
            'verbose_logging': True,  # Enable verbose logging
            'log_map_processing': True,  # Log details about map processing
            'log_obstacle_detection': True,  # Log details about obstacle detection
            'log_path_planning': True,  # Log details about path planning
            'publish_debug_images': True,  # Publish debug images
            'publish_debug_markers': True,  # Publish debug markers
            'diagnostic_level': 2,  # Maximum diagnostic level
            # Enhanced parameters for better integration with MPC
            'max_speed': 30.0,  # Reduced from 50.0 to 30.0 to better match MPC's max speed
            'min_speed': 0.0,
            'max_yaw_rate': 0.5,  # Increased for better heading control
            'max_accel': 8.0,  # Increased for faster acceleration
            'max_delta_yaw_rate': 0.3,  # Increased for more responsive heading changes
            'dt': 0.1,
            'predict_time': 2.5,  # Reduced for more frequent heading updates
            'to_goal_cost_gain': 1.5,  # Adjusted for better balance
            'speed_cost_gain': 0.2,  # Reduced to prioritize obstacle avoidance
            'obstacle_cost_gain': 25.0,  # Further increased for stronger obstacle avoidance
            'path_following_gain': 15.0,  # Adjusted for better path following
            'lookahead_distance': 10.0,  # Reduced for tighter heading control
            'obstacle_threshold': 30,  # Significantly lowered threshold for black obstacle detection (black is close to 0)
            'safe_distance': 6.0,  # Increased safety margin
            'default_lane_width': 3.2,
            'lane_width_factor': 0.9,  # Increased for wider lane consideration
            'start_point_offset': 0.3,  # Reduced to start turns earlier
            'min_obstacle_distance': 2.0,  # Increased minimum distance to obstacles
            'path_smoothing_factor': 0.5,  # Reduced for more precise path following
            'lateral_safety_margin': 1.8,  # Increased lateral safety margin
            'obstacle_weight_decay': 0.1,  # Adjusted decay for stronger close-range avoidance
            'adaptive_lookahead': True,
            'emergency_stop_enabled': True,
            'emergency_brake_distance': 8.0,  # Increased emergency brake distance
            'obstacle_path_pruning': True,
            # Enhanced parameters for black obstacles on gray lanes
            'lane_obstacle_detection_enabled': True,  # Enable specific detection of obstacles crossing lanes
            'lane_obstacle_threshold': 0,  # Lower threshold for detecting black obstacles (black is close to 0)
            'lane_gray_min_threshold': 50,  # Minimum gray value to be considered a lane
            'lane_gray_max_threshold': 200,  # Maximum gray value to be considered a lane
            'lane_width_for_obstacles': 4.0,  # Width of lane to check for obstacles (meters)
            'lane_obstacle_min_area': 30,  # Minimum area of obstacle pixels to consider
            'lane_obstacle_stop_distance': 20.0,  # Distance to start stopping when lane obstacle detected
            'lane_obstacle_slow_distance': 25.0,  # Distance to start slowing down when lane obstacle detected
            # Black lane detection parameters (for different purpose than black obstacles)
            'black_lane_detection_enabled': True,  # Enable black lane detection
            'black_lane_threshold': 0,  # Lower threshold for detecting black lanes (0-255)
            'black_lane_min_area': 10,  # Minimum area of black pixels to consider as a lane
            'black_lane_stop_distance': 20.0,  # Distance to start stopping when black lane detected
            'black_lane_slow_distance': 25.0,  # Distance to start slowing down when black lane detected
            'min_cmd_vel_for_stop': 0.3,  # Lower velocity to send when stopping (for more immediate stops)
            # Waypoint crossing obstacle parameters
            'waypoint_obstacle_check_enabled': True,  # Enable checking if obstacles cross waypoints
            'waypoint_corridor_width': 3.5,  # Width of corridor around waypoints to check for obstacles
            'waypoint_obstacle_lookahead': 20.0,  # How far ahead to check waypoints for obstacles
            'waypoint_obstacle_threshold': 30,  # Lower threshold for detecting black obstacles on waypoint path
            'waypoint_obstacle_stop_command': True,  # Send stop command when obstacle crosses waypoints
            # Color-specific detection
            'color_detection_enabled': True,  # Enable color-specific detection
            'black_obstacle_max_value': 50,  # Maximum pixel value to be considered black (obstacles)
            'gray_lane_min_value': 100,  # Minimum pixel value to be considered gray (lanes)
            'gray_lane_max_value': 200,  # Maximum pixel value to be considered gray (lanes)
            'contrast_enhancement_enabled': True,  # Enable contrast enhancement for better detection
            # Map subscription parameters
            'map_subscription_qos': 'reliable_transient',  # Use reliable and transient local QoS for map subscription
            'map_subscription_timeout': 60.0,  # Wait up to 60 seconds for map data
            'map_subscription_retry_interval': 2.0,  # Retry every 2 seconds
        }],
        output='screen'
    )
    
    # Define the MPC Controller node configured for direct waypoint following
    mpc_node = Node(
        package='sensor_fusion_2',
        executable='mpc_controller',
        name='mpc_controller',
        parameters=[{
            'vehicle_frame_id': vehicle_frame,
            'map_frame_id': map_frame,
            'cmd_vel_topic': '/mpc/cmd_vel',
            # Enable DWA connections for obstacle detection
            'direct_dwa_connection': True,
            'dwa_cmd_vel_topic': '/dwa/cmd_vel',
            'bypass_trajectory_receiver': True,
            # Configure for direct waypoint following
            'use_waypoints': True,
            'waypoints_topic': '/carla/waypoints',
            'path_topic': '/dwa/path',
            'tcp_port': tcp_port,
            'tcp_host': tcp_host,
            'update_rate': 60.0,  # Increased from 50.0 to 60.0 for even more responsive control
            'prediction_horizon': 8,  # Maintained at 8 which is a good balance
            'control_horizon': 2,   # Maintained at 2 for immediate control
            'dt': 0.1,
            'max_speed': 20.0,  # Maximum speed in m/s
            'min_speed': 0.0,
            'max_yaw_rate': 4.0,  # Increased from 3.0 to 4.0 for even more responsive turns
            'max_accel': 10.0,  # Increased from 10.0 to 20.0 to match controller settings
            'max_deceleration': 5.0,
            'min_deceleration': 0.05,
            'max_steering_angle': 0.9,  # Increased from 0.8 to 0.9 for even tighter turns
            'max_steering_rate': 0.6,  # Increased from 0.4 to 0.6 for much faster steering response
            'wheelbase': 2.8,
            'position_weight': 5.0,  # Maintained at 5.0
            'heading_weight': 300.0,  # Dramatically increased from 20.0 to 300.0 to match controller settings
            'velocity_weight': 0.05,  # Maintained at 0.05
            'steering_weight': 0.2,  # Reduced from 0.5 to 0.2 to match controller settings
            'acceleration_weight': 0.05,
            'deceleration_weight': 0.05,
            'jerk_weight': 0.05,  # Maintained at 0.05
            'steering_rate_weight': 0.05,  # Reduced from 0.1 to 0.05 for even more aggressive steering changes
            'brake_smoothness_weight': 0.2,  # Reduced from 0.3 to 0.2
            'min_velocity': 0.0,
            'stop_threshold': 0.1,
            'deceleration_profile': 'smooth',
            'brake_transition_threshold': 0.2,
            'reconnect_interval': 1.0,
            'max_reconnect_attempts': 10,
            'test_mode': False,
            'debug_level': debug_level,
            'min_path_points': 2,
            'path_update_timeout': 2.0,
            'velocity_topic': '/carla/ego_vehicle/velocity',
            'fallback_throttle': 0.9,  # Increased from 0.8 to 0.9 for higher default speed
            # IMU integration parameters
            'use_imu': True,  # Enable IMU integration
            'imu_topic': '/carla/ego_vehicle/imu',  # Updated to use CARLA's IMU topic
            'imu_angular_velocity_weight': 2.0,  # Weight for IMU angular velocity
            'imu_linear_acceleration_weight': 1.0,  # Weight for IMU linear acceleration
            # Parameters for obstacle detection via DWA
            'curve_detection_threshold': 3.0,  # Threshold in degrees for curve detection
            'min_throttle': 0.1,  # Minimum throttle during gradual stop
            'max_throttle': 0.8,  # Maximum throttle on straight segments
            'max_steering': 0.8,  # Maximum steering value
            'dwa_stop_threshold': 0.3,  # Lowered from 0.5 to 0.3 to match DWA's min_cmd_vel_for_stop
            # Enhanced parameters for responding to DWA obstacle detection
            'emergency_brake_force': 0.9,  # Strong brake force when obstacle detected
            'obstacle_stop_timeout': 3.0,  # How long to remain stopped after obstacle detection (seconds)
            'obstacle_resume_threshold': 0.5,  # Speed to resume after obstacle is cleared
            'prioritize_dwa_commands': True,  # Always prioritize DWA commands for safety
            'waypoint_obstacle_response': True,  # Enable specific response to obstacles on waypoint lane
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
    
    # Add velocity listener node
    ld.add_action(velocity_listener_node)
    
    # Add velocity visualizer node
    ld.add_action(velocity_visualizer_node)
    
    # Add DWA node first to ensure it starts before MPC
    ld.add_action(dwa_node)
    
    # Add the MPC node after DWA
    ld.add_action(mpc_node)
    
    return ld 