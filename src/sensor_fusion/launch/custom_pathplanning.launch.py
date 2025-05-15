#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # First define ALL launch arguments to avoid "does not exist" errors
    # Path planning specific parameters that must be defined before nodes
    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='0.6',  # Slightly larger grid cells for faster planning
        description='Grid size for A* planning (meters)'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='2.5',  # Vehicle wheelbase for motion model
        description='Vehicle wheelbase for motion model (meters)'
    )
    
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='55',  # Threshold for considering a cell as an obstacle
        description='Threshold for considering a cell as an obstacle (0-100)'
    )
    
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='10000',  # Change to 10000 for more thorough search
        description='Maximum iterations for the A* algorithm'
    )
    
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/realtime_map',
        description='Topic for map data'
    )
    
    # Now create the nodes that use these parameters
    # Create transform publisher nodes with higher publish frequency for faster updates
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}], # Increased frequency for transforms
    )
    
    # Add back the map_to_base_link static transform, but with a conditional option
    # This provides initial connectivity until the dynamic transform from imu_lidar_yaw_fusion takes over
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}], # Increased frequency for transforms
    )
    
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}], # Increased frequency for transforms
    )
    
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 150.0}], # Increased frequency for transforms
    )
    
    # Create the fused mapper node - with shorter delay for TF to be established
    fast_imu_lidar_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_fast_fusion_mapper',
        parameters=[{
            # Map resolution and size parameters - optimized for performance/accuracy balance
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'center_on_vehicle': LaunchConfiguration('center_on_vehicle'),
            
            # Processing parameters - higher rates for real-time response
            'publish_rate': LaunchConfiguration('publish_rate'),
            'process_rate': LaunchConfiguration('process_rate'),
            
            # Bayesian update weights - enhanced for better obstacle detection
            'hit_weight': LaunchConfiguration('hit_weight'),
            'miss_weight': LaunchConfiguration('miss_weight'),
            'prior_weight': LaunchConfiguration('prior_weight'),
            'count_threshold': LaunchConfiguration('count_threshold'),
            
            # Fast mapping parameters for optimized data processing
            'decay_rate': LaunchConfiguration('decay_rate'),
            'update_threshold': LaunchConfiguration('update_threshold'),
            'temporal_memory': LaunchConfiguration('temporal_memory'),
            'enable_map_reset': LaunchConfiguration('enable_map_reset'),
            'map_reset_interval': LaunchConfiguration('map_reset_interval'),
            'use_binary_map': LaunchConfiguration('use_binary_map'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'obstacle_inflation': LaunchConfiguration('obstacle_inflation'),
            
            # Map saving parameters - configurable options
            'enable_map_save': LaunchConfiguration('enable_map_save'),
            'map_save_dir': LaunchConfiguration('map_save_dir'),
            'enable_auto_save': LaunchConfiguration('enable_auto_save'),
            'auto_save_interval': LaunchConfiguration('auto_save_interval'),
            'map_base_filename': LaunchConfiguration('map_base_filename'),
            'save_format': LaunchConfiguration('save_format'),
            
            # Other parameters - now more configurable
            'ground_threshold': LaunchConfiguration('ground_threshold'),
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'raycast_skip': LaunchConfiguration('raycast_skip'),
            'max_points_to_process': LaunchConfiguration('max_points_to_process'),
            'use_cluster_data': LaunchConfiguration('use_cluster_data'),
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # TF parameters optimized for faster lookups
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
            'tf_timeout': LaunchConfiguration('tf_timeout'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'wait_for_transform': True,
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            
            # Vehicle point filtering parameters - tailored for CARLA LiDAR mounted at x=1.5, z=2.0
            'filter_vehicle_points': True,
            'vehicle_filter_radius': 2.0,
            'vehicle_filter_height_min': -1.0,
            'vehicle_filter_height_max': 1.5,
            'min_range_filter': 1.0,
            'vehicle_filter_x_offset': -1.5,
            
            # Grid visualization
            'enable_grid_lines': True,
            'grid_cell_size': 4,
            'grid_line_thickness': 1,
            'grid_line_color': [50, 50, 50, 128],
        }],
        output='screen'
    )
    
    # Delay the start of the mapper - further reduced delay for faster startup
    delayed_mapper = TimerAction(
        period=LaunchConfiguration('mapper_delay'),
        actions=[fast_imu_lidar_mapper_node]
    )
    
    # Set up the Current Pose Publisher node
    current_pose_publisher_node = Node(
        package='sensor_fusion',
        executable='current_pose_publisher',
        name='current_pose_publisher',
        parameters=[{
            'publish_rate': 20.0,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map'
        }],
        output='screen'
    )
    
    # Set up the Hybrid A* planner node
    hybrid_astar_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[{
            # Grid and vehicle parameters
            'grid_size': LaunchConfiguration('grid_size'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            
            # Planning parameters 
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'publish_rate': 20.0,
            
            # Advanced A* parameters
            'max_iterations': LaunchConfiguration('max_iterations'),
            'motion_resolution': LaunchConfiguration('motion_resolution'),
            'angle_resolution': LaunchConfiguration('angle_resolution'),
            'heuristic_weight': LaunchConfiguration('heuristic_weight'),
            
            # Make MORE sensitive to movement changes
            'replan_on_move': True,
            'position_change_threshold': 0.05,
            'orientation_change_threshold': 0.02,
            
            # Force path replanning when obstacles are detected
            'detect_map_changes': True,
            'force_replan_interval': 0.5,
            'avoid_obstacles_weight': 10.0,
            
            # Add the new parameters for handling unexplored areas
            'min_explored_percentage': LaunchConfiguration('min_explored_percentage'),
            'treat_unknown_as_obstacle': LaunchConfiguration('treat_unknown_as_obstacle'),
            'unknown_cost_multiplier': LaunchConfiguration('unknown_cost_multiplier'),
            
            # Topics and frames
            'map_topic': '/realtime_map',
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
        }],
        output='screen'
    )
    
    # Create a vehicle marker publisher for visualization
    vehicle_marker_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='vehicle_visualizer',
        parameters=[{
            'visualize_only': True,
            'publish_rate': 10.0,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map'
        }],
        output='screen'
    )
    
    # IMU-LiDAR yaw fusion node with explicit TF publishing and higher rate
    imu_lidar_fusion_node = Node(
        package='sensor_fusion',
        executable='imu_lidar_yaw_fusion',
        name='imu_lidar_yaw_fusion',
        parameters=[{
            # Connection parameters
            'imu_topic': '/imu/data',
            'map_topic': '/realtime_map',
            
            # Processing parameters
            'publish_rate': 100.0,
            'publish_tf': True,
            'publish_tf_rate': 100.0,
            
            # IMU parameters
            'initial_yaw_offset': 0.0,
            'use_filtered_yaw': True,
            'yaw_filter_size': 3,
            'yaw_weight': 0.9,
            'override_static_tf': True,
            
            # Only include parameters that are actually implemented in your node
            # Removing complementary filter parameters that may not exist
            
            # Adaptive fusion if implemented
            'adaptive_fusion': True,
            
            # Frame IDs
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            
            # TF parameters
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
            'tf_timeout': LaunchConfiguration('tf_timeout'),
            'wait_for_transform': True,
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Add all pre-declared arguments first
        grid_size_arg,
        wheelbase_arg,
        obstacle_threshold_arg,
        max_iterations_arg,
        map_topic_arg,
        
        # Connection parameters for IMU and LiDAR
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='0.0.0.0',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='0.0.0.0',
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        
        # Map Parameters - optimized for A* path planning
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.15',  # Higher resolution for better mapping
            description='Resolution of the fast map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='60.0',  # Wider map for longer path planning
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='60.0',  # Taller map for longer path planning
            description='Height of the map in meters'
        ),
        DeclareLaunchArgument(
            'center_on_vehicle',
            default_value='true',
            description='Keep map centered on vehicle position'
        ),
        
        # Performance parameters - optimized for real-time response
        DeclareLaunchArgument(
            'publish_rate',
            default_value='40.0',  # Increased publish rate for smoother visualization
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='80.0',  # Increased process rate for faster updates
            description='Rate to process map data (Hz)'
        ),
        DeclareLaunchArgument(
            'mapper_delay',
            default_value='0.5',  # Reduced delay for faster startup
            description='Delay before starting mapper (seconds)'
        ),
        
        # Bayesian update parameters - optimized for better obstacle detection
        DeclareLaunchArgument(
            'hit_weight',
            default_value='0.7',  # Reduced for less aggressive obstacle marking
            description='Weight for obstacle hits in Bayesian update'
        ),
        DeclareLaunchArgument(
            'miss_weight',
            default_value='0.8',  # Increased for faster free space clearing
            description='Weight for misses (free space) in Bayesian update'
        ),
        DeclareLaunchArgument(
            'prior_weight',
            default_value='0.4',  # Slightly favor free space in priors
            description='Prior weight for initial map state'
        ),
        DeclareLaunchArgument(
            'count_threshold',
            default_value='0.06',  # Lower threshold for faster binary mapping
            description='Count threshold for binary mapping'
        ),
        
        # Fast mapping specific parameters - optimized for A* path planning
        DeclareLaunchArgument(
            'decay_rate',
            default_value='0.85',  # Faster decay for more dynamic environments
            description='Rate at which old map data decays (0-1, lower = faster decay)'
        ),
        DeclareLaunchArgument(
            'update_threshold',
            default_value='0.0003',  # Lower threshold for more responsive updates
            description='Threshold for map updates'
        ),
        DeclareLaunchArgument(
            'temporal_memory',
            default_value='0.05',  # Shorter memory for more dynamic updates
            description='Duration to remember points (seconds)'
        ),
        DeclareLaunchArgument(
            'enable_map_reset',
            default_value='true',  # Keep map resets enabled
            description='Enable periodic map reset to clear old data'
        ),
        DeclareLaunchArgument(
            'map_reset_interval',
            default_value='5.0',  # Reset more frequently
            description='Interval for map reset (seconds)'
        ),
        DeclareLaunchArgument(
            'use_binary_map',
            default_value='true',  # Keep binary mapping for A*
            description='Use binary (black/white) map output for clear path planning'
        ),
        DeclareLaunchArgument(
            'obstacle_inflation',
            default_value='0.2',  # Reduced inflation for tighter paths
            description='Amount to inflate obstacles for safer path planning (meters)'
        ),
        DeclareLaunchArgument(
            'max_points_to_process',
            default_value='10000',  # Process more points for better detail
            description='Maximum number of points to process per update'
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
            default_value='5.0',
            description='Interval for auto-saving the map (seconds)'
        ),
        DeclareLaunchArgument(
            'map_base_filename',
            default_value='path_planning_map',
            description='Base filename for saved maps'
        ),
        DeclareLaunchArgument(
            'save_format',
            default_value='png',
            description='Format to save maps in (png, pgm, or both)'
        ),
        
        # Additional configurable parameters
        DeclareLaunchArgument(
            'ground_threshold',
            default_value='0.10',  # Slightly lower for better ground detection
            description='Threshold for ground detection'
        ),
        DeclareLaunchArgument(
            'min_height',
            default_value='-0.5',
            description='Minimum height for point cloud processing'
        ),
        DeclareLaunchArgument(
            'max_height',
            default_value='2.5',
            description='Maximum height for point cloud processing'
        ),
        DeclareLaunchArgument(
            'raycast_skip',
            default_value='1',
            description='Number of points to skip during raycasting'
        ),
        DeclareLaunchArgument(
            'use_cluster_data',
            default_value='true',
            description='Whether to use cluster data for mapping'
        ),
        
        # TF parameters
        DeclareLaunchArgument(
            'tf_buffer_duration',
            default_value='4.0',  # Increased buffer for better stability
            description='Duration of TF buffer (seconds)'
        ),
        DeclareLaunchArgument(
            'tf_timeout',
            default_value='0.2',  # Shorter timeout for faster recovery
            description='Timeout for TF lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'transform_tolerance',
            default_value='0.3',  # Increased tolerance for better stability
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
            default_value='false',
            description='Whether to display 2D convex hull around clusters'
        ),
        
        # Path planning specific parameters
        DeclareLaunchArgument(
            'motion_resolution',
            default_value='5',
            description='Number of steering angles to consider in motion model'
        ),
        DeclareLaunchArgument(
            'angle_resolution',
            default_value='18',
            description='Angle discretization (18 = 20 degree resolution)'
        ),
        DeclareLaunchArgument(
            'heuristic_weight',
            default_value='2.0',
            description='Weight for A* heuristic (higher = faster but less optimal)'
        ),
        
        # Additional parameters for hybrid A* planner
        DeclareLaunchArgument(
            'min_explored_percentage',
            default_value='10.0',  # Reduce from current value to 10.0 to allow paths through less explored areas
            description='Minimum percentage of environment to be explored before planning'
        ),
        DeclareLaunchArgument(
            'treat_unknown_as_obstacle',
            default_value='false',  # Change to false (was likely true)
            description='Whether to treat unknown areas as obstacles'
        ),
        DeclareLaunchArgument(
            'unknown_cost_multiplier',
            default_value='1.5',  # Reduce cost multiplier (was likely higher)
            description='Cost multiplier for unknown cells'
        ),
        
        # TF static transform publishers
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Set up the IMU Euler Visualizer node
        Node(
            package='sensor_fusion',
            executable='imu_euler_visualizer',
            name='imu_euler_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
                'tcp_port': LaunchConfiguration('imu_tcp_port'),
                'reconnect_interval': 2.0,
                'frame_id': 'imu_link',
                'world_frame_id': 'world',
                'filter_window_size': 2,
                'queue_size': 10,
                'publish_rate': 100.0,
            }],
            output='screen'
        ),
        
        # Set up the LiDAR listener node
        Node(
            package='sensor_fusion',
            executable='lidar_listener_clusters_2',
            name='lidar_cube_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
                'tcp_port': LaunchConfiguration('lidar_tcp_port'),
                'point_size': LaunchConfiguration('point_size'),
                'center_size': LaunchConfiguration('center_size'),
                'use_convex_hull': LaunchConfiguration('use_convex_hull'),
                'use_point_markers': False,
                'use_cluster_stats': True,
                'verbose_logging': False,
                'cube_alpha': 0.3,
                'filter_vehicle_points': True,
                'vehicle_filter_radius': 2.0,
                'vehicle_filter_height_min': -1.0,
                'vehicle_filter_height_max': 1.5,
                'min_range_filter': 1.0,
                'vehicle_filter_x_offset': -1.5,
                'frame_id': 'lidar_link',
                'use_tf_transform': True,
                'queue_size': 10,
                'publish_rate': 40.0,
                'min_points_per_cluster': 7,
                'max_cluster_distance': 0.35,
            }],
            output='screen'
        ),
        
        # Set up the fast imu-lidar mapper node with a delay
        delayed_mapper,
        
        # IMU-Lidar yaw fusion node
        imu_lidar_fusion_node,
        
        # Current pose publisher node
        current_pose_publisher_node,
        
        # Path planning nodes
        hybrid_astar_node,
        vehicle_marker_node,
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/path_planning.rviz']],
            output='screen'
        ),
    ]) 