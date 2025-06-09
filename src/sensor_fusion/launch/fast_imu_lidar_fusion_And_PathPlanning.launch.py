#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # =========== TRANSFORM CONFIGURATION ===========
    # These transforms establish the complete TF tree for the robot
    # world → map → base_link → imu_link → lidar_link
    
    # Root transform: world to map
    # This provides a fixed reference frame for global positioning
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0  # Increased frequency for transform stability with C++ data
        }],
    )
    
    # Map to base_link transform
    # This is a fallback static transform until the dynamic one from imu_lidar_yaw_fusion takes over
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0  # Match other transform frequencies
        }],
    )
    
    # Base to IMU transform
    # This represents the physical mounting position of the IMU on the robot
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '0',    # X offset (forward/back)
            '0',    # Y offset (left/right)
            '0.1',  # Z offset (up/down) - IMU is 10cm above the base
            '0',    # Roll (rotation around X)
            '0',    # Pitch (rotation around Y)
            '0',    # Yaw (rotation around Z)
            'base_link', 
            'imu_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # IMU to LiDAR transform
    # This represents the physical mounting position of the LiDAR relative to the IMU
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=[
            LaunchConfiguration('lidar_x_offset'),    # X offset (forward/back)
            LaunchConfiguration('lidar_y_offset'),    # Y offset (left/right)
            LaunchConfiguration('lidar_z_offset'),    # Z offset (up/down)
            LaunchConfiguration('lidar_roll'),        # Roll (rotation around X)
            LaunchConfiguration('lidar_pitch'),       # Pitch (rotation around Y)
            LaunchConfiguration('lidar_yaw'),         # Yaw (rotation around Z)
            'imu_link', 
            'lidar_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # IMU Visualization Node - optimized for C++ data integration
    imu_euler_visualizer_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'reconnect_interval': 1.0,                  # Faster reconnects for C++ stream
            'frame_id': 'imu_link',
            'world_frame_id': 'world',
            'filter_window_size': 2,                    # Small filter for faster response
            'queue_size': 20,                           # Larger queue for C++ data bursts
            'publish_rate': 200.0,                      # Higher rate for C++ data
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'socket_buffer_size': 65536,                # Larger socket buffer for C++ data
            'enable_bias_correction': True,             # Enable IMU bias correction for more accurate data
            'enable_complementary_filter': True,        # Use complementary filter for more stable orientation
            'zero_velocity_threshold': 0.02,            # Add zero velocity detection for better stationary stability
        }],
        output='screen'
    )
    
    # LiDAR Point Cloud Visualization Node - adjusted for updated LiDAR configuration
    lidar_listener_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_cube_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'point_size': LaunchConfiguration('point_size'),
            'center_size': LaunchConfiguration('center_size'),
            'use_convex_hull': True,
            'use_point_markers': False,  
            'use_cluster_stats': True,   
            'verbose_logging': False,    
            'cube_alpha': 0.6,                         # Increased for better visibility in large map
            
            # C++ Integration parameters - optimized for large map processing
            'socket_buffer_size': 524288,              # DOUBLED buffer size for larger data streams (512KB)
            'enable_data_validation': True,
            'reconnect_on_error': True,
            'reconnect_cooldown': 0.5,
            'packet_timeout_ms': 120,                  # Increased timeout for larger data packets
            'enable_async_reception': True,
            'max_packet_size': 131072,                 # DOUBLED for larger data chunks (128KB)
            'enable_zero_copy_reception': True,
            'apply_cpp_timestamp_correction': True,
            
            # UPDATED C++ Statistical Outlier Removal parameters - MATCHED WITH UPDATED C++ CODE
            'sor_std_dev_multiplier': 4.2,             # Match updated C++ SOR parameters
            'sor_neighbor_radius': 0.7,                # Match updated C++ neighbor radius
            'sor_min_neighbors': 12,                   # Match updated C++ min neighbors
            'voxel_leaf_size': 0.15,                   # Match updated C++ voxel grid size
            
            # UPDATED C++ RANSAC Ground Plane parameters - MATCHED WITH UPDATED C++ CODE
            'ransac_iterations': 200,                  # Match updated C++ RANSAC iterations
            'ransac_distance_threshold': 0.22,         # Match updated C++ RANSAC distance threshold
            
            # UPDATED C++ Euclidean Clustering parameters - MATCHED WITH UPDATED C++ CODE
            'max_cluster_distance': 0.4,               # Match updated C++ cluster tolerance
            'min_points_per_cluster': 8,               # Match updated C++ minimum points
            
            # Vehicle filtering parameters - optimized for updated LiDAR position
            'filter_vehicle_points': True,
            'vehicle_length': 5.2,          
            'vehicle_width': 2.7,           
            'vehicle_height': 2.3,          
            'vehicle_x_offset': 0.0,
            'vehicle_y_offset': 0.0,
            'vehicle_z_offset': -1.0,
            'vehicle_safety_margin': 0.9,              # Increased for better vehicle filtering
            'vehicle_visualization': True,
            
            # Motion compensation - UPDATED for narrow FOV
            'enable_motion_compensation': True,
            'use_imu_data': True,
            'point_time_compensation': True,
            'motion_prediction_time': 0.18,            # Increased for smoother transitions with narrow FOV
            
            # LiDAR specific parameters - EXACTLY matching CARLA configuration
            'lidar_upper_fov': 0.0,                    # EXACT match with CARLA config
            'lidar_lower_fov': -15.0,                  # EXACT match with CARLA config
            'lidar_pitch_angle': 5.0,                  # EXACT match with CARLA config 
            'min_point_distance': 2.2,                 # Adjusted for cleaner data matching CARLA settings
            'max_negative_z': -0.7,                    # Adjusted for CARLA height configuration
            
            # Clustering parameters - optimized for narrow FOV in large area
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'queue_size': 15,                          # Increased for smoother visualization in large map
            'publish_rate': 50.0,                      # Adjusted for balanced visualization
            
            # Object boxing parameters - adjusted for larger map area
            'enable_object_boxes': True,
            'box_padding': 0.2,                        # Match updated settings
            'min_box_points': 8,                       # Match updated C++ min cluster points
            'use_oriented_boxes': True,
            'publish_box_markers': True,
            'box_lifetime': 0.3,                       # Increased for better visibility in large map
            'box_color_r': 0.0,
            'box_color_g': 1.0,
            'box_color_b': 0.0,
            'box_color_a': 0.8,                        # Increased for better visibility in large map
            
            # Additional C++ data specific parameters - optimized for large map
            'cpp_data_format': 'packed_float',
            'cpp_data_endianness': 'little',
            'cpp_format_version': 2,
            'cpp_source_path': '/home/mostafa/ROS2andCarla/CPP/Lidar_Processing',
            'enable_cpp_data_logging': False,
            'log_dropped_packets': True,
            'enable_data_rate_limiting': False,
            
            # Movement filtering - adjusted for narrow FOV in large area
            'movement_detection': True,
            'movement_speed_threshold': 0.12,          # Adjusted for stability with narrow FOV
            'stationary_cluster_threshold': 8,         # Match updated C++ min cluster points
            'moving_cluster_threshold': 8,             # Match updated C++ min cluster points
            'adaptive_clustering': True,
        }],
        output='screen'
    )
    
    # IMU-LiDAR Fusion Node - optimized for C++ data streams
    imu_lidar_fusion_node = Node(
        package='sensor_fusion',
        executable='imu_lidar_yaw_fusion',
        name='imu_lidar_yaw_fusion',
        parameters=[{
            # Connection parameters
            'imu_topic': '/imu/data',
            'map_topic': '/realtime_map',
            
            # Processing parameters - optimized for C++ data
            'publish_rate': 200.0,
            
            # IMU-specific parameters - optimized for C++ data
            'initial_yaw_offset': LaunchConfiguration('imu_yaw_offset'),
            'use_filtered_yaw': True,
            'yaw_filter_size': 2,
            'yaw_weight': 0.97,
            'fusion_mode': 'weighted_avg',
            'adaptive_fusion': True,
            'stationary_fusion_weight': 0.9,           # Weight when stationary
            'moving_fusion_weight': 0.98,              # Weight when moving
            
            # Motion detection
            'motion_detection_enabled': True,
            'motion_threshold': 0.05,                  # Lower threshold for better motion detection
            'yaw_rate_threshold': 0.03,                # Lower threshold for better rotation detection
            
            # Transform parameters
            'publish_tf': True,
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'override_static_tf': True,
            'publish_tf_rate': 200.0,
            
            # Map configuration
            'all_white_map': True,
            'invert_map_colors': False,
            
            # TF parameters
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
            'tf_timeout': LaunchConfiguration('tf_timeout'),
            'wait_for_transform': True,
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # Map saving parameters
            'enable_auto_save': LaunchConfiguration('enable_auto_save'),
            'auto_save_interval': LaunchConfiguration('auto_save_interval'),
            'map_save_dir': LaunchConfiguration('map_save_dir'),
        }],
        output='screen'
    )
    
    # Real-time mapper node - optimized for grid-based visualization with larger map
    fast_imu_lidar_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_fast_fusion_mapper',
        parameters=[{
            # Map resolution and size parameters - ENLARGED for bigger visualization area
            'map_resolution': 0.4,          # Increased for larger area coverage with reasonable detail
            'map_width_meters': 100.0,      # DOUBLED map width for much larger coverage area
            'map_height_meters': 100.0,     # DOUBLED map height for much larger coverage area
            'center_on_vehicle': True,
            
            # Processing rates - optimized for larger map
            'publish_rate': 30.0,           # Reduced for better performance with larger map
            'process_rate': 80.0,           # Adjusted for efficient processing of larger area
            
            # Bayesian update weights - optimized for larger grid space
            'hit_weight': 0.82,             # Slightly reduced for less aggressive occupation in large map
            'miss_weight': 0.90,            # Adjusted for smoother clearing in large spaces
            'prior_weight': 0.0,
            'count_threshold': 0.12,        # Increased for more definitive grid cells in large map
            
            # Fast mapping parameters - optimized for larger grid space
            'decay_rate': 0.75,             # Reduced for longer persistence in large grid
            'update_threshold': 0.00015,    # Adjusted for stable grid updates in large area
            'temporal_memory': 0.08,        # Increased for better object persistence in large map
            'enable_map_reset': True,
            'map_reset_interval': 5.0,      # Increased for larger map area
            'obstacle_threshold': 0.48,     # Increased for clearer obstacle definition in large map
            'initialize_as_free': True,
            'unknown_to_free': True,
            
            # C++ Integration optimizations - with updated parameters to match C++ code
            'enable_multi_threading': True,
            'thread_count': 6,              # Increased for better parallel processing of larger map
            'use_lockless_updates': True,
            'enable_simd_optimization': True,
            'enable_point_batching': True,
            'batch_size': 1500,             # Increased for more efficient batch processing
            'enable_adaptive_processing': True,
            'cpp_data_preprocessing': True,
            'cpp_data_source_path': '/home/mostafa/ROS2andCarla/CPP/Lidar_Processing',
            
            # Point filtering parameters - MATCHED WITH UPDATED C++ CODE
            'outlier_std_dev_multiplier': 4.2,         # Match updated C++ SOR parameter
            'outlier_neighbor_radius': 0.7,            # Match updated C++ neighbor radius
            'outlier_min_neighbors': 12,               # Match updated C++ min neighbors
            'voxel_filter_size': 0.15,                 # Match updated C++ voxel grid parameter
            
            # Ground plane parameters - MATCHED WITH UPDATED C++ CODE
            'ransac_iterations': 200,                  # Match updated C++ RANSAC iterations
            'ransac_distance_threshold': 0.22,         # Match updated C++ RANSAC distance threshold
            
            # Motion compensation parameters - adjusted for larger grid
            'enable_motion_compensation': True,
            'motion_compensation_max_speed': 8.0,      # Increased for faster vehicle movement in large map
            'motion_compensation_max_yaw_rate': 1.2,   # Increased for more responsive turning compensation
            'use_velocity_history': True,
            
            # Map saving parameters
            'enable_map_save': LaunchConfiguration('enable_map_save'),
            'map_save_dir': LaunchConfiguration('map_save_dir'),
            'enable_auto_save': LaunchConfiguration('enable_auto_save'),
            'auto_save_interval': LaunchConfiguration('auto_save_interval'),
            'map_base_filename': LaunchConfiguration('map_base_filename'),
            'save_format': LaunchConfiguration('save_format'),
            
            # Point processing parameters - adjusted for updated FOV
            'ground_threshold': 0.22,       # Match updated C++ RANSAC distance threshold
            'min_height': -0.7,             # Adjusted for updated LiDAR configuration
            'max_height': 3.0,              # Increased for taller objects in larger map
            'raycast_skip': 4,              # Increased for faster processing of larger map
            'max_points_to_process': 7000,  # Adjusted for larger area coverage
            'use_cluster_data': True,
            
            # Vehicle motion filtering - adjusted for larger grid
            'motion_filter_enabled': True,
            'motion_detection_threshold': 0.15,        # Increased for less sensitivity in large map
            'motion_clearing_factor': 1.6,             # Adjusted for better clearing behavior
            'stationary_decay_factor': 0.35,           # Adjusted for stable grid
            'clear_obstacles_behind': True,
            'behind_vehicle_distance': 4.0,            # Increased for larger area
            'use_velocity_direction': True,
            
            # Frame IDs
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # TF parameters
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
            'tf_timeout': LaunchConfiguration('tf_timeout'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wait_for_transform': True,
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            
            # Vehicle point filtering parameters - adjusted for updated LiDAR position
            'filter_vehicle_points': True,
            'vehicle_filter_radius': 3.5,              # Increased for better vehicle filtering
            'vehicle_filter_height_min': -1.3,         # Adjusted for updated LiDAR height
            'vehicle_filter_height_max': 3.2,          # Increased for taller vehicle models
            'min_range_filter': 2.2,                   # Increased for cleaner near-vehicle data
            'vehicle_filter_x_offset': 0.0,
            'dynamic_filter_expansion': 0.6,           # Increased for better dynamic filtering
            
            # Binary map configuration - essential for grid visualization
            'use_binary_map': True,
            
            # A* specific parameters - optimized for larger grid-based planning
            'obstacle_inflation': 0.7,                 # Increased for safer path planning in large map
            
            # Grid visualization parameters - ENHANCED for larger map
            'enable_grid_lines': True,                 # Enabled for better large map visualization
            'grid_cell_size': 5,                       # Increased for larger map
            'grid_line_thickness': 1,
            'grid_line_color': LaunchConfiguration('grid_line_color'),
            
            # Object boxing parameters - adjusted for larger map
            'enable_object_boxing': True,      
            'min_box_size': 0.5,                       # Match updated C++ settings
            'max_box_size': 20.0,                      # Increased for larger objects in big map
            'box_padding': 0.2,                        # Match updated C++ settings
            'min_points_for_box': 8,                   # Match updated C++ min cluster points
            'use_oriented_boxes': True,        
            'box_retention_time': 0.3,                 # Increased for better visibility in large map
            'publish_box_markers': True,       
            'object_tracking': True,           
            'object_velocity_estimation': True,
            'minimum_track_confidence': 0.6,
            
            # Clustering parameters - MATCHED WITH UPDATED C++ CODE
            'cluster_tolerance': 0.4,                  # Match updated C++ cluster tolerance
            'min_cluster_size': 8,                     # Match updated C++ minimum points
            'max_cluster_size': 8000,                  # Increased for larger objects in big map
            
            # ROS 2 DDS tuning for grid visualization
            'use_transient_local_durability': True,
            'use_reliable_reliability': True,
            'history_depth': 5,
            'use_best_effort_subscription': False,
        }],
        output='screen'
    )
    
    # Delay the start of the mapper for transform tree to be established
    delayed_mapper = TimerAction(
        period=LaunchConfiguration('mapper_delay'),
        actions=[fast_imu_lidar_mapper_node]
    )
    
    # ======== NEW A* PATH PLANNING NODES ========
    # Robot pose publisher node - converts TF to PoseStamped for A* planner
    robot_pose_publisher_node = Node(
        package='sensor_fusion',
        executable='test_goal_pose_publisher',
        name='robot_pose_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'auto_publish': True,
            'publish_rate': 30.0,                   # Higher frequency for robot pose updates
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'goal_x': 0.0,
            'goal_y': 0.0,
            'goal_yaw': 0.0,
            'goal_tolerance': 0.1,
            'spawn_ui_controls': False,
            'use_current_pose': True,               # Important: Make it publish the current robot pose
            'publish_topic': '/robot_pose',         # The topic that A* planner expects
            'skip_occupied_goals': False,           # Not relevant for current pose
            'obstacle_detection_radius': 0.5        # Small radius just for informational purposes
        }],
        output='screen'
    )
    
    # Goal pose publisher for testing path planning
    goal_pose_publisher_node = Node(
        package='sensor_fusion',
        executable='test_goal_pose_publisher',
        name='goal_pose_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'auto_publish': True,
            'publish_rate': 0.2,                    # Much slower publish rate (was 1.0)
            'map_frame_id': 'map',
            'goal_x': 10.0,                         # Set this to your desired goal position
            'goal_y': 10.0,                         # Set this to your desired goal position
            'goal_yaw': 0.0,
            'goal_tolerance': 0.5,
            'spawn_ui_controls': True,
            'use_current_pose': False,              # Make it publish goal poses
            'publish_topic': '/goal_pose',          # Topic for goal poses
            'random_goals': False,                  # Disable random goals
            'skip_occupied_goals': True,            # Avoid placing goals in occupied areas
            'obstacle_detection_radius': 3.0,       # Check 3 meters around goal for obstacles
            'retry_goal_seconds': 5.0,              # Retry blocked goals after 5 seconds
            'max_goal_retries': 3                   # Max attempts to find unoccupied goal
        }],
        output='screen'
    )
    
    # A* path planner node
    a_star_planner_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='a_star_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'grid_size': 0.4,                     # Keep resolution high enough for detailed planning
            'obstacle_threshold': 50,             # Lower threshold makes more cells count as obstacles (was 65)
            'map_topic': '/realtime_map',
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'publish_rate': 5.0,
            'max_iterations': 15000,              # Increased to allow more search iterations (was 10000)
            'motion_resolution': 5,               # More motion steps for finer paths (was 10)
            'angle_resolution': 72,               # More angle divisions for smoother paths (was 36)
            'heuristic_weight': 1.2,              # Reduced weight to favor safer paths (was 1.5)
            'replan_on_move': True,
            'position_change_threshold': 0.25,    # Replan more frequently (was 0.5)
            'orientation_change_threshold': 0.1,  # Reduced to replan when turning less (was 0.15)
            'treat_unknown_as_obstacle': True,
            'unknown_cost_multiplier': 8.0,       # Higher cost for unknown areas (was 5.0)
            'publish_debug_viz': True,
            'min_explored_percentage': 95.0,      # Higher threshold for path through known areas (was 80.0)
            'obstacle_inflation': 1.2,            # Significantly increased obstacle inflation (was 0.7)
            'collision_check_resolution': 0.1,    # Fine-grained collision checking
            'obstacle_avoidance_weight': 2.0,     # Higher weight for obstacle avoidance
            'prefer_straight_paths': True,        # Prefer straight paths when possible
            'safety_margin': 0.8                  # Extra safety margin around the vehicle
        }],
        remappings=[
            ('/current_pose', '/robot_pose')
        ],
        output='screen'
    )
    
    # Path visualizer node
    path_visualizer_node = Node(
        package='sensor_fusion',
        executable='trajectory_visualizer',
        name='path_visualizer',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'marker_lifetime': 0.5,
            'path_width': 0.1,
            'path_color_r': 0.0,
            'path_color_g': 0.8,
            'path_color_b': 0.2,
            'path_color_a': 1.0,
            'publish_rate': 10.0,
            'marker_scale': 0.3,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map'
        }],
        remappings=[
            ('/path', '/hybrid_astar_path'),
            ('/visualization_markers', '/path_visualization_markers')
        ],
        output='screen'
    )
    
    # Path to trajectory converter for smoother path
    path_to_trajectory_node = Node(
        package='sensor_fusion',
        executable='path_to_trajectory_converter',
        name='path_to_trajectory_converter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'smoothing_strength': 0.6,            # Less smoothing to better preserve obstacle avoidance paths (was 0.75)
            'max_speed': 1.5,                     # Lower top speed for safety (was 2.0)
            'min_speed': 0.3,                     # Lower min speed for very tight turns (was 0.5)
            'wheelbase': 2.5,
            'look_ahead_distance': 3.0,           # Increased to look further ahead (was 2.0)
            'trajectory_resolution': 0.15,        # Higher resolution for more precise trajectory (was 0.2)
            'enable_speed_profile': True,
            'corner_slowdown_factor': 0.7,        # Stronger slowdown in corners (was 0.5)
            'publish_rate': 5.0,
            'enable_acceleration_limits': True,
            'max_acceleration': 0.5,              # Reduced acceleration for smoother ride (was 1.0)
            'max_deceleration': 1.5,              # Reduced deceleration for comfort (was 2.0)
            'proximity_slowdown_enabled': True,   # New parameter to slow down near obstacles
            'proximity_min_distance': 2.0,        # Slow down when obstacles are within this distance
            'proximity_slowdown_factor': 0.4      # How much to slow down near obstacles
        }],
        remappings=[
            ('/path', '/hybrid_astar_path'),
            ('/trajectory', '/trajectory')
        ],
        output='screen'
    )
    
    # Delayed start of path planning nodes
    delayed_path_planning = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[robot_pose_publisher_node, a_star_planner_node, path_visualizer_node, path_to_trajectory_node]
    )
    
    # Create the launch description with all nodes and argument declarations
    return LaunchDescription([
        # ================= LAUNCH ARGUMENTS =================
        # TCP connection parameters for IMU and LiDAR
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        
        # IMU parameters
        DeclareLaunchArgument(
            'imu_yaw_offset',
            default_value='0.0',
            description='Initial yaw offset for IMU (radians)'
        ),

        # LiDAR mounting parameters
        DeclareLaunchArgument(
            'lidar_x_offset',
            default_value='0.0',
            description='X offset of LiDAR from IMU (meters)'
        ),
        DeclareLaunchArgument(
            'lidar_y_offset',
            default_value='0.0',
            description='Y offset of LiDAR from IMU (meters)'
        ),
        DeclareLaunchArgument(
            'lidar_z_offset',
            default_value='0.2',
            description='Z offset of LiDAR from IMU (meters)'
        ),
        DeclareLaunchArgument(
            'lidar_roll',
            default_value='0.0',
            description='Roll angle of LiDAR mounting (radians)'
        ),
        DeclareLaunchArgument(
            'lidar_pitch',
            default_value='0.0',
            description='Pitch angle of LiDAR mounting (radians)'
        ),
        DeclareLaunchArgument(
            'lidar_yaw',
            default_value='0.0',
            description='Yaw angle of LiDAR mounting (radians)'
        ),
        
        # LiDAR field of view parameters - updated to match CARLA config
        DeclareLaunchArgument(
            'lidar_upper_fov',
            default_value='0.0',           # Updated to match CARLA config
            description='Upper vertical field of view of LiDAR (degrees)'
        ),
        DeclareLaunchArgument(
            'lidar_lower_fov',
            default_value='-15.0',         # Updated to match CARLA config
            description='Lower vertical field of view of LiDAR (degrees)'
        ),
        
        # Vehicle dimensions
        DeclareLaunchArgument(
            'vehicle_length',
            default_value='5.0',
            description='Length of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_width',
            default_value='2.5',
            description='Width of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_height',
            default_value='2.2',
            description='Height of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_x_offset',
            default_value='0.0',
            description='X offset of vehicle center from base_link (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_y_offset',
            default_value='0.0',
            description='Y offset of vehicle center from base_link (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_z_offset',
            default_value='-1.0',
            description='Z offset of vehicle center from base_link (meters)'
        ),
        
        # Map Parameters - updated for larger visualization
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.4',            # Updated for larger map
            description='Resolution of the fast map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='100.0',          # DOUBLED for much larger map
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='100.0',          # DOUBLED for much larger map
            description='Height of the map in meters'
        ),
        DeclareLaunchArgument(
            'center_on_vehicle',
            default_value='true',
            description='Keep map centered on vehicle position'
        ),
        
        # Performance parameters
        DeclareLaunchArgument(
            'publish_rate',
            default_value='60.0',
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='150.0',
            description='Rate to process map data (Hz)'
        ),
        DeclareLaunchArgument(
            'mapper_delay',
            default_value='0.3',
            description='Delay before starting mapper (seconds)'
        ),
        
        # Map saving parameters
        DeclareLaunchArgument(
            'enable_map_save',
            default_value='true',
            description='Enable saving map to disk'
        ),
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/mostafa/Robot_local/maps',
            description='Directory to save maps'
        ),
        DeclareLaunchArgument(
            'enable_auto_save',
            default_value='true',
            description='Enable automatic saving of map at intervals'
        ),
        DeclareLaunchArgument(
            'auto_save_interval',
            default_value='10.0',
            description='Interval for auto-saving the map (seconds)'
        ),
        DeclareLaunchArgument(
            'map_base_filename',
            default_value='fast_fusion_map',
            description='Base filename for saved maps'
        ),
        DeclareLaunchArgument(
            'save_format',
            default_value='png',
            description='Format to save maps in (png, pgm, or both)'
        ),
        
        # Grid visualization parameters
        DeclareLaunchArgument(
            'enable_grid_lines',
            default_value='false',
            description='Whether to display grid lines on the map'
        ),
        DeclareLaunchArgument(
            'grid_cell_size',
            default_value='4',
            description='Number of cells between grid lines'
        ),
        DeclareLaunchArgument(
            'grid_line_thickness',
            default_value='1',
            description='Thickness of grid lines in pixels'
        ),
        DeclareLaunchArgument(
            'grid_line_color',
            default_value='128 128 128 128',
            description='Color of grid lines in RGBA format (0-255 each)'
        ),
        
        # TF parameters
        DeclareLaunchArgument(
            'tf_buffer_duration',
            default_value='3.0',
            description='Duration of TF buffer (seconds)'
        ),
        DeclareLaunchArgument(
            'tf_timeout',
            default_value='0.3',
            description='Timeout for TF lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'transform_tolerance',
            default_value='0.2',
            description='Tolerance for transform lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time'
        ),

        # Visualization parameters
        DeclareLaunchArgument(
            'point_size',
            default_value='1.0',
            description='Size of individual point markers'
        ),
        DeclareLaunchArgument(
            'center_size',
            default_value='2.0',
            description='Size of cluster center markers'
        ),
        DeclareLaunchArgument(
            'use_convex_hull',
            default_value='true',
            description='Whether to display 2D convex hull around clusters'
        ),
        
        # =============== NODE INSTANTIATION ===============
        # Launch TF tree first to ensure transform availability
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/realtime_map.rviz']],
            output='screen'
        ),
        
        # Launch sensor data providers
        imu_euler_visualizer_node,
        lidar_listener_node,
        
        # Launch the fusion node
        imu_lidar_fusion_node,
        
        # Launch the mapper with a delay
        delayed_mapper,
        
        # Path planning nodes with delayed start
        delayed_path_planning,
        goal_pose_publisher_node,
    ]) 