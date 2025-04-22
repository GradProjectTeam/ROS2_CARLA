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
            'publish_rate': LaunchConfiguration('publish_rate'),  # Configurable publish rate
            'process_rate': LaunchConfiguration('process_rate'),  # Configurable process rate 
            
            # Bayesian update weights - enhanced for better obstacle detection
            'hit_weight': LaunchConfiguration('hit_weight'),      # Configurable hit weight
            'miss_weight': LaunchConfiguration('miss_weight'),    # Configurable miss weight
            'prior_weight': LaunchConfiguration('prior_weight'),  # Now configurable
            'count_threshold': LaunchConfiguration('count_threshold'), # Now configurable
            
            # Fast mapping parameters for optimized data processing
            'decay_rate': LaunchConfiguration('decay_rate'),      # Configurable decay rate
            'update_threshold': LaunchConfiguration('update_threshold'), # Now configurable
            'temporal_memory': LaunchConfiguration('temporal_memory'),  # Configurable temporal memory
            'enable_map_reset': LaunchConfiguration('enable_map_reset'),  # Enable periodic map reset
            'map_reset_interval': LaunchConfiguration('map_reset_interval'),  # Reset interval
            'use_binary_map': LaunchConfiguration('use_binary_map'),  # Force binary map output
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),  # Threshold for obstacles
            
            # Map saving parameters - configurable options
            'enable_map_save': LaunchConfiguration('enable_map_save'),  # Enable saving map 
            'map_save_dir': LaunchConfiguration('map_save_dir'),  # Directory to save maps
            'enable_auto_save': LaunchConfiguration('enable_auto_save'),  # Enable auto save
            'auto_save_interval': LaunchConfiguration('auto_save_interval'),  # Auto save interval
            'map_base_filename': LaunchConfiguration('map_base_filename'),  # Base filename for saved maps
            'save_format': LaunchConfiguration('save_format'),  # Format to save maps in
            
            # Other parameters - now more configurable
            'ground_threshold': LaunchConfiguration('ground_threshold'),  # Now configurable
            'min_height': LaunchConfiguration('min_height'),  # Now configurable
            'max_height': LaunchConfiguration('max_height'),  # Now configurable
            'raycast_skip': LaunchConfiguration('raycast_skip'),  # Now configurable
            'max_points_to_process': LaunchConfiguration('max_points_to_process'),  # Configurable max points
            'use_cluster_data': LaunchConfiguration('use_cluster_data'),  # Now configurable
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # TF parameters optimized for faster lookups
            'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),  # Now configurable
            'tf_timeout': LaunchConfiguration('tf_timeout'),  # Now configurable
            'use_sim_time': LaunchConfiguration('use_sim_time'),  # Now configurable for testing
            'wait_for_transform': True,    # Wait for transform to be available
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),  # Now configurable
            
            # Vehicle point filtering parameters - tailored for CARLA LiDAR mounted at x=1.5, z=2.0
            'filter_vehicle_points': True,           # Enable filtering of vehicle points
            'vehicle_filter_radius': 2.0,            # Larger radius for better vehicle filtering
            'vehicle_filter_height_min': -1.0,       # Lower minimum to catch entire car
            'vehicle_filter_height_max': 1.5,        # Higher maximum for high-mounted LiDAR 
            'min_range_filter': 1.0,                 # Larger minimum range
            'vehicle_filter_x_offset': -1.5,         # Account for forward position of LiDAR
            
            # A* specific parameters
            'use_binary_map': True,                 # Force binary map for clear A* implementation
            'obstacle_inflation': LaunchConfiguration('obstacle_inflation'),  # Inflate obstacles for safer path planning
            'enable_grid_lines': True,              # Optional: Draw grid lines for easier debugging
            'grid_cell_size': 4,                    # Draw grid every 4 cells
            'grid_line_thickness': 1,               # Thin grid lines
            'grid_line_color': [50, 50, 50, 128],   # Semi-transparent dark gray
        }],
        output='screen'
    )
    
    # Delay the start of the mapper - further reduced delay for faster startup
    delayed_mapper = TimerAction(
        period=LaunchConfiguration('mapper_delay'),  # Now configurable
        actions=[fast_imu_lidar_mapper_node]
    )
    
    # Set up the Current Pose Publisher node
    current_pose_publisher_node = Node(
        package='sensor_fusion',
        executable='current_pose_publisher',
        name='current_pose_publisher',
        parameters=[{
            'publish_rate': 20.0,  # Higher frequency for current pose
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
            'publish_rate': 10.0,  # Higher rate for more responsive planning
            
            # Advanced A* parameters
            'max_iterations': LaunchConfiguration('max_iterations'),
            'motion_resolution': 5,  # Number of steering angles to sample
            'angle_resolution': 18,  # Number of angles to discretize 360 degrees
            'heuristic_weight': 1.5,  # Higher values bias toward goal
            
            # NEW: Movement-based replanning parameters
            'replan_on_move': True,  # Enable replanning based on vehicle movement
            'position_change_threshold': 0.1,  # More sensitive - replan if moved more than 0.1m
            'orientation_change_threshold': 0.05,  # More sensitive - replan if rotated more than ~3°
            
            # Topics and frames
            'map_topic': LaunchConfiguration('map_topic'),
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
            'visualize_only': True,  # Only publish visualization, don't do navigation
            'publish_rate': 10.0,    # Publish vehicle markers at 10Hz
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # TCP connection parameters for IMU and LiDAR
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
            default_value='0.20',  # Less detailed but more efficient for path planning
            description='Resolution of the fast map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='50.0',  # Wider map for longer path planning
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='50.0',  # Taller map for longer path planning
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
            default_value='30.0',  # Increased publish rate for smoother visualization
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='60.0',  # Increased process rate for faster updates
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
            default_value='1.0',  # Maximum confidence in hits
            description='Weight for obstacle hits in Bayesian update'
        ),
        DeclareLaunchArgument(
            'miss_weight',
            default_value='0.5',  # Stronger clearing for faster free space updates
            description='Weight for misses (free space) in Bayesian update'
        ),
        DeclareLaunchArgument(
            'prior_weight',
            default_value='0.5',  # Start with unknown/gray
            description='Prior weight for initial map state'
        ),
        DeclareLaunchArgument(
            'count_threshold',
            default_value='0.12',  # Lower threshold for faster binary mapping
            description='Count threshold for binary mapping'
        ),
        
        # Fast mapping specific parameters - optimized for A* path planning
        DeclareLaunchArgument(
            'decay_rate',
            default_value='0.95',  # Slightly slower decay for more persistent obstacles
            description='Rate at which old map data decays (0-1, higher = faster decay)'
        ),
        DeclareLaunchArgument(
            'update_threshold',
            default_value='0.0003',  # Lower threshold for faster updates
            description='Threshold for map updates'
        ),
        DeclareLaunchArgument(
            'temporal_memory',
            default_value='0.15',  # Shorter memory for faster updates
            description='Duration to remember points (seconds)'
        ),
        DeclareLaunchArgument(
            'enable_map_reset',
            default_value='true',  # Enable map reset
            description='Enable periodic map reset to clear old data'
        ),
        DeclareLaunchArgument(
            'map_reset_interval',
            default_value='8.0',  # Reset map more frequently
            description='Interval for map reset (seconds)'
        ),
        DeclareLaunchArgument(
            'use_binary_map',
            default_value='true',  # Binary is essential for A* - clear obstacle/free distinction
            description='Use binary (black/white) map output for clear path planning'
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='55',  # Lower threshold to be more cautious with obstacles
            description='Threshold for marking cells as obstacles (0-100)'
        ),
        DeclareLaunchArgument(
            'obstacle_inflation',
            default_value='0.4',  # New parameter: inflate obstacles for safer paths
            description='Amount to inflate obstacles for safer path planning (meters)'
        ),
        DeclareLaunchArgument(
            'max_points_to_process',
            default_value='8000',  # Process more points for better detail
            description='Maximum number of points to process per update'
        ),
        
        # Map saving parameters
        DeclareLaunchArgument(
            'enable_map_save',
            default_value='true',  # Enable map saving
            description='Enable saving map to disk'
        ),
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/mostafa/Robot_local/maps',  # Updated path to match workspace
            description='Directory to save maps'
        ),
        DeclareLaunchArgument(
            'enable_auto_save',
            default_value='true',  # Enable auto save
            description='Enable automatic saving of map at intervals'
        ),
        DeclareLaunchArgument(
            'auto_save_interval',
            default_value='5.0',  # Save interval
            description='Interval for auto-saving the map (seconds)'
        ),
        DeclareLaunchArgument(
            'map_base_filename',
            default_value='path_planning_map',  # Base filename for saved maps
            description='Base filename for saved maps'
        ),
        DeclareLaunchArgument(
            'save_format',
            default_value='png',  # Format to save maps in
            description='Format to save maps in (png, pgm, or both)'
        ),
        
        # Additional configurable parameters
        DeclareLaunchArgument(
            'ground_threshold',
            default_value='0.12',  # Lower threshold for better ground detection
            description='Threshold for ground detection'
        ),
        DeclareLaunchArgument(
            'min_height',
            default_value='-0.5',  # Lower minimum height for better ground mapping
            description='Minimum height for point cloud processing'
        ),
        DeclareLaunchArgument(
            'max_height',
            default_value='2.5',  # Higher maximum height for taller obstacles
            description='Maximum height for point cloud processing'
        ),
        DeclareLaunchArgument(
            'raycast_skip',
            default_value='1',  # Process more points for better detail
            description='Number of points to skip during raycasting'
        ),
        DeclareLaunchArgument(
            'use_cluster_data',
            default_value='true',  # Use cluster data
            description='Whether to use cluster data for mapping'
        ),
        
        # TF parameters
        DeclareLaunchArgument(
            'tf_buffer_duration',
            default_value='3.0',  # Shorter buffer for less memory usage
            description='Duration of TF buffer (seconds)'
        ),
        DeclareLaunchArgument(
            'tf_timeout',
            default_value='0.3',  # Shorter timeout for faster failures and retries
            description='Timeout for TF lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'transform_tolerance',
            default_value='0.2',  # Lower tolerance for faster lookups
            description='Tolerance for transform lookups (seconds)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',  # Use real time by default
            description='Whether to use simulation time'
        ),

        # Visualization parameters
        DeclareLaunchArgument(
            'point_size',
            default_value='1.0',  # Point size for visualization
            description='Size of individual point markers'
        ),
        DeclareLaunchArgument(
            'center_size',
            default_value='2.0',  # Center size for visualization
            description='Size of cluster center markers'
        ),
        DeclareLaunchArgument(
            'use_convex_hull',
            default_value='false',  # Disable convex hull for better performance
            description='Whether to display 2D convex hull around clusters'
        ),
        
        # Add vehicle filtering parameters with updated values - tailored for CARLA setup
        DeclareLaunchArgument(
            'filter_vehicle_points',
            default_value='true',  # Enable vehicle point filtering
            description='Whether to filter out points belonging to the vehicle itself'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_radius',
            default_value='2.0',  # Larger radius to cover entire car from forward-mounted LiDAR
            description='Radius around vehicle center to filter out points'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_height_min',
            default_value='-1.0',  # Lower to ensure catching the entire car height from top-mounted LiDAR
            description='Minimum height for vehicle point filtering'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_height_max',
            default_value='1.5',  # Higher to account for high-mounted LiDAR (2.0m)
            description='Maximum height for vehicle point filtering'
        ),
        DeclareLaunchArgument(
            'min_range_filter',
            default_value='1.0',  # Increased minimum range to account for forward mounting
            description='Minimum distance to keep points (creates blind spot)'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_x_offset',
            default_value='-1.5',  # Negative of the x-position to center filter on vehicle not LiDAR
            description='X offset for vehicle filtering (negative of LiDAR x position)'
        ),
        
        # Path planning specific parameters
        DeclareLaunchArgument(
            'planner_publish_rate',
            default_value='10.0',  # Frequency to run the path planning algorithm
            description='Rate to publish planned paths (Hz)'
        ),
        DeclareLaunchArgument(
            'grid_size',
            default_value='0.5',  # Grid size for A* planning
            description='Grid size for A* planning (meters)'
        ),
        DeclareLaunchArgument(
            'planning_obstacle_threshold',
            default_value='55',  # Threshold for considering a cell as an obstacle
            description='Threshold for considering a cell as an obstacle (0-100)'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='2.5',  # Vehicle wheelbase for motion model
            description='Vehicle wheelbase for motion model (meters)'
        ),
        DeclareLaunchArgument(
            'max_iterations',
            default_value='10000',  # Maximum search iterations 
            description='Maximum iterations for A* search'
        ),
        DeclareLaunchArgument(
            'motion_resolution',
            default_value='10',  # Number of steering angles to consider
            description='Number of steering angles to consider in motion model'
        ),
        DeclareLaunchArgument(
            'angle_resolution',
            default_value='36',  # Angle discretization (36 = 10 degree resolution)
            description='Angle discretization for A* (36 = 10 degree resolution)'
        ),
        DeclareLaunchArgument(
            'heuristic_weight',
            default_value='1.5',  # Weight for A* heuristic
            description='Weight for A* heuristic (higher = faster but less optimal)'
        ),
        
        # TF static transform publishers - restored map_to_base_link for initial connectivity
        world_to_map_node,
        map_to_base_link_node,  # Add this back to ensure connectivity
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Set up the IMU Euler Visualizer node (to display IMU data)
        Node(
            package='sensor_fusion',
            executable='imu_euler_visualizer',
            name='imu_euler_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
                'tcp_port': LaunchConfiguration('imu_tcp_port'),
                'reconnect_interval': 2.0,  # Faster reconnects
                'frame_id': 'imu_link',
                'world_frame_id': 'world',
                'filter_window_size': 2,    # Smaller filter window for faster response
                'queue_size': 10,           # New parameter for queue size
                'publish_rate': 100.0,      # New parameter for publish rate
            }],
            output='screen'
        ),
        
        # Set up the LiDAR listener node with optimized parameters
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
                'filter_vehicle_points': True,          # Keep this enabled
                'vehicle_filter_radius': 2.0,           # Increased for forward-mounted LiDAR
                'vehicle_filter_height_min': -1.0,      # Lower to catch entire vehicle from high mounting
                'vehicle_filter_height_max': 1.5,       # Higher for high-mounted LiDAR
                'min_range_filter': 1.0,                # Slightly larger blind spot for forward mounting
                'vehicle_filter_x_offset': -1.5,        # Account for forward mounting position
                'frame_id': 'lidar_link',
                'use_tf_transform': True,
                'queue_size': 10,
                'publish_rate': 40.0,
                'min_points_per_cluster': 7,
                'max_cluster_distance': 0.35,
            }],
            output='screen'
        ),
        
        # Set up the fast imu-lidar mapper node with a delay to ensure transforms are available
        delayed_mapper,
        
        # Set up the IMU-Lidar yaw fusion node with explicit TF publishing and higher rate
        Node(
            package='sensor_fusion',
            executable='imu_lidar_yaw_fusion',
            name='imu_lidar_yaw_fusion',
            parameters=[{
                # Connection parameters
                'imu_topic': '/imu/data',
                'map_topic': '/realtime_map',  # Use realtime map for fusion
                
                # Processing parameters - higher rate for faster updates
                'publish_rate': 50.0,  # Higher rate specifically for map publishing
                
                # TF publishing - MORE AGGRESSIVE
                'publish_tf': True,  # Explicitly enable TF publishing
                'publish_tf_rate': 100.0,  # VERY high rate (100Hz) for responsive transform updates
                
                # IMU-specific parameters - optimized for map rotation
                'initial_yaw_offset': 0.0,
                'use_filtered_yaw': True,
                'yaw_filter_size': 3,          # Smaller filter for faster response
                'yaw_weight': 0.95,            # High IMU weight to ensure map rotation
                'override_static_tf': True,    # Override the static transform publisher
                
                # Frame IDs
                'map_frame_id': 'map',         # Map frame ID
                'base_frame_id': 'base_link',  # Base frame ID
                
                # TF parameters - optimized for faster lookups
                'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
                'tf_timeout': LaunchConfiguration('tf_timeout'),
                'wait_for_transform': True,
                'transform_tolerance': LaunchConfiguration('transform_tolerance'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen'
        ),
        
        # Current pose publisher node - needed for path planning
        current_pose_publisher_node,
        
        # Path planning nodes
        hybrid_astar_node,
        vehicle_marker_node,  # Vehicle marker visualization
        
        # RViz for visualization - use the path planning specific config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/path_planning.rviz']],
            output='screen'
        ),

        # Add the missing map_topic parameter in the LaunchDescription
        DeclareLaunchArgument(
            'map_topic',
            default_value='/realtime_map',
            description='Topic for map data'
        ),
    ]) 