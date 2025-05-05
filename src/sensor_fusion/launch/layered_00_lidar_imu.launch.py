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
            'publish_frequency': 100.0  # Reduced frequency to save resources but still maintain smoothness
        }],
    )
    
    # Map to base_link transform
    # NOTE: This is a fallback static transform that is used until the dynamic one 
    # from imu_lidar_yaw_fusion node takes over. Having this ensures the TF tree
    # is connected from startup.
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',  # Renamed for clarity
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 100.0  # Reduced but still high enough for smooth transitions
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
            'publish_frequency': 100.0  # Consistent with other transform frequencies
        }],
    )
    
    # IMU to LiDAR transform
    # This represents the physical mounting position of the LiDAR relative to the IMU
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=[
            '0',    # X offset (forward/back)
            '0',    # Y offset (left/right)
            '0.2',  # Z offset (up/down) - LiDAR is 20cm above the IMU
            '0',    # Roll (rotation around X)
            '0',    # Pitch (rotation around Y)
            '0',    # Yaw (rotation around Z)
            'imu_link', 
            'lidar_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 100.0  # Consistent with other transform frequencies
        }],
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
            'prior_weight': 0.0,  # Fixed: changed from string '0.0' to float 0.0
            'count_threshold': LaunchConfiguration('count_threshold'),
            
            # Fast mapping parameters for optimized data processing
            'decay_rate': .5,  # Fixed: changed from string '1.5' to float 1.5
            'update_threshold': 0.0,  # Fixed: changed from string '0.0' to float 0.0
            'temporal_memory': 0.1,  # Fixed: changed from string '0.1' to float 0.1
            'enable_map_reset': True,  # Fixed: changed from string 'true' to boolean True
            'map_reset_interval': 15.0,  # Fixed: changed from string '15.0' to float 15.0
            'obstacle_threshold': 0.1,  # Fixed: changed from string '0.1' to float 0.1
            'initialize_as_free': LaunchConfiguration('initialize_as_free'),
            'unknown_to_free': LaunchConfiguration('unknown_to_free'),
            
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
            'wait_for_transform': True,  # Fixed: changed from string 'True' to boolean True
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            
            # Vehicle point filtering parameters
            'filter_vehicle_points': LaunchConfiguration('filter_vehicle_points'),
            'vehicle_filter_radius': LaunchConfiguration('vehicle_filter_radius'),
            'vehicle_filter_height_min': LaunchConfiguration('vehicle_filter_height_min'),
            'vehicle_filter_height_max': LaunchConfiguration('vehicle_filter_height_max'),
            'min_range_filter': LaunchConfiguration('min_range_filter'),
            'vehicle_filter_x_offset': LaunchConfiguration('vehicle_filter_x_offset'),
            
            # Binary map configuration (explicitly set once to avoid duplicates)
            'use_binary_map': LaunchConfiguration('use_binary_map'),
            
            # A* specific parameters
            'obstacle_inflation': LaunchConfiguration('obstacle_inflation'),
            
            # Grid visualization parameters
            'enable_grid_lines': LaunchConfiguration('enable_grid_lines'),
            'grid_cell_size': LaunchConfiguration('grid_cell_size'),
            'grid_line_thickness': LaunchConfiguration('grid_line_thickness'),
            'grid_line_color': LaunchConfiguration('grid_line_color'),
        }],
        output='screen'
    )
    
    # Delay the start of the mapper - further reduced delay for faster startup
    delayed_mapper = TimerAction(
        period=LaunchConfiguration('mapper_delay'),
        actions=[fast_imu_lidar_mapper_node]
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
        
        # Map Parameters - optimized for A* path planning
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.5',  # Less detailed but more efficient for path planning
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
            default_value='0.55',  # Lower threshold to be more cautious with obstacles
            description='Threshold for marking cells as obstacles (0-1)'
        ),
        DeclareLaunchArgument(
            'initialize_as_free',
            default_value='true',  # Initialize all cells as free space (white)
            description='Initialize all cells as free space (white) rather than unknown (gray)'
        ),
        DeclareLaunchArgument(
            'unknown_to_free',
            default_value='true',  # Treat unknown cells as free space
            description='Treat all unknown cells as free space for visualization'
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
            default_value='fast_fusion_map',  # Base filename for saved maps
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
        
        # Grid visualization parameters - added to fix missing parameters error
        DeclareLaunchArgument(
            'enable_grid_lines',
            default_value='false',  # Disable grid lines by default
            description='Whether to display grid lines on the map'
        ),
        DeclareLaunchArgument(
            'grid_cell_size',
            default_value='4',  # Draw grid every 4 cells
            description='Number of cells between grid lines'
        ),
        DeclareLaunchArgument(
            'grid_line_thickness',
            default_value='1',  # 1-pixel line thickness
            description='Thickness of grid lines in pixels'
        ),
        DeclareLaunchArgument(
            'grid_line_color',
            default_value='128 128 128 128',  # Semi-transparent gray (RGBA)
            description='Color of grid lines in RGBA format (0-255 each)'
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
            default_value='3.0',  # Increased to match our new min_point_distance value
            description='Radius around vehicle center to filter out points'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_height_min',
            default_value='-0.8',  # Adjusted to align with max_negative_z
            description='Minimum height for vehicle point filtering'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_height_max',
            default_value='2.5',  # Higher to account for the increased LiDAR height (1.0m above vehicle)
            description='Maximum height for vehicle point filtering'
        ),
        DeclareLaunchArgument(
            'min_range_filter',
            default_value='3.0',  # Match our LIDAR_MIN_DISTANCE value
            description='Minimum distance to keep points (creates blind spot)'
        ),
        DeclareLaunchArgument(
            'vehicle_filter_x_offset',
            default_value='0.0',  # Updated to 0.0 since our LiDAR is now centered
            description='X offset for vehicle filtering (matches LiDAR position)'
        ),
        
        # =============== NODE INSTANTIATION ===============
        # Order matters for proper startup sequence
        
        # Step 1: Launch TF tree first to ensure transform availability
        world_to_map_node,         # Root transform: world → map
        map_to_base_link_node,     # Initial localization: map → base_link
        base_to_imu_node,          # Sensor mount position: base_link → imu_link
        imu_to_lidar_node,         # Sensor mount position: imu_link → lidar_link
        
        # Step 2: Launch RViz (moved up to avoid dependency issues)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/realtime_map.rviz']],
            output='screen'
        ),
        
        # Step 3: Launch sensor data providers
        Node(
            package='sensor_fusion',
            executable='imu_euler_visualizer',
            name='imu_euler_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
                'tcp_port': LaunchConfiguration('imu_tcp_port'),
                'reconnect_interval': 2.0,  # Fixed: explicit float
                'frame_id': 'imu_link',
                'world_frame_id': 'world',
                'filter_window_size': 2,  # Fixed: integer value
                'queue_size': 10,         # Fixed: integer value
                'publish_rate': 100.0,    # Fixed: float value
            }],
            output='screen'
        ),
        
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
                'use_point_markers': False,  # Fixed: boolean value
                'use_cluster_stats': True,   # Fixed: boolean value
                'verbose_logging': False,    # Fixed: boolean value
                'cube_alpha': 0.3,           # Fixed: float value
                'filter_vehicle_points': True,  # Fixed: boolean value
                'vehicle_length': 5.0,          # Fixed: float value
                'vehicle_width': 2.5,           # Fixed: float value
                'vehicle_height': 2.2,          # Fixed: float value
                'vehicle_x_offset': 0.0,        # Fixed: float value
                'vehicle_y_offset': 0.0,        # Fixed: float value
                'vehicle_z_offset': -1.0,       # Fixed: float value
                'vehicle_safety_margin': 0.5,   # Fixed: float value
                'vehicle_visualization': True,  # Fixed: boolean value
                
                # LiDAR specific parameters 
                'lidar_upper_fov': 15.0,      # Fixed: float value
                'lidar_lower_fov': -25.0,     # Fixed: float value
                'lidar_pitch_angle': 5.0,     # Fixed: float value
                'min_point_distance': 3.0,    # Fixed: float value
                'max_negative_z': -0.5,       # Fixed: float value
                
                'frame_id': 'lidar_link',
                'use_tf_transform': True,     # Fixed: boolean value
                'queue_size': 10,             # Fixed: integer value
                'publish_rate': 40.0,         # Fixed: float value
                'min_points_per_cluster': 5,  # Fixed: integer value
                'max_cluster_distance': 0.45, # Fixed: float value
            }],
            output='screen'
        ),
        
        # Step 4: Launch the fusion node which will override the static transform
        Node(
            package='sensor_fusion',
            executable='imu_lidar_yaw_fusion',
            name='imu_lidar_yaw_fusion',
            parameters=[{
                # Connection parameters
                'imu_topic': '/imu/data',
                'map_topic': '/realtime_map',  # Use realtime map for fusion
                
                # Processing parameters - optimized for TF performance
                'publish_rate': 100.0,  # Fixed: explicit float instead of string
                
                # IMU-specific parameters - optimized for map rotation
                'initial_yaw_offset': 0.0,  # Fixed: explicit float
                'use_filtered_yaw': True,  # Fixed: boolean instead of string
                'yaw_filter_size': 3,       # Fixed: integer instead of string
                'yaw_weight': 0.95,         # Fixed: float instead of string
                'fusion_mode': 'weighted_avg',  # String is appropriate here
                'adaptive_fusion': True,    # Fixed: boolean instead of string
                
                # Transform parameters - explicit settings to ensure proper TF tree
                'publish_tf': True,         # Fixed: boolean instead of string
                'map_frame_id': 'map',      # String is appropriate here
                'base_frame_id': 'base_link',  # String is appropriate here
                'override_static_tf': True,  # Fixed: boolean instead of string
                'publish_tf_rate': 100.0,   # Fixed: float instead of string
                
                # All-white map configuration
                'all_white_map': True,      # Fixed: boolean instead of string
                'invert_map_colors': False, # Fixed: boolean instead of string
                
                # TF parameters - optimized for faster lookups
                'tf_buffer_duration': LaunchConfiguration('tf_buffer_duration'),
                'tf_timeout': LaunchConfiguration('tf_timeout'),
                'wait_for_transform': True,  # Fixed: boolean instead of string
                'transform_tolerance': LaunchConfiguration('transform_tolerance'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                
                # Map saving parameters
                'enable_auto_save': True,   # Fixed: boolean instead of string
                'auto_save_interval': 30.0, # Fixed: float instead of string
                'map_save_dir': LaunchConfiguration('map_save_dir'),
            }],
            output='screen'
        ),
        
        # Step 5: Launch the mapper last with a delay to ensure all transforms and data streams are ready
        delayed_mapper,
    ]) 