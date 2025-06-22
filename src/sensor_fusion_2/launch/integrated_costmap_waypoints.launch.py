#!/usr/bin/env python3
"""
Semantic Costmap Launch File - Balanced for Responsive Highway Navigation and Trajectory Planning

This launch file has been tuned for balanced responsiveness with the following settings:
- Moderate decay_time of 0.1s for responsive map updates without excessive flickering
- Dynamic object decay time of 0.05s for timely fading of moving objects
- Cell_memory set to 0.1s for responsive radar map updates
- Max_tracking_age set to 0.2s for reliable object tracking
- Marker_lifetime set to 0.1s for smooth visualization

Enhanced Vegetation and Object Detection:
- All detection weights maximized to 10.0 (maximum allowed)
- Reduced vegetation_height_ratio to 2.0 for more sensitive vegetation detection
- Binary_threshold set to 0.05 for reliable classification of obstacles
- Added special conversion flags to ensure all vegetation and detections appear as black

Binary Map Output for Navigation:
- All detected objects (including vegetation) are converted to black for navigation
- The binary map is published on the /semantic_costmap/binary topic
- Uses standard values: 100 for occupied cells (black), 0 for free space
- All layer weights maximized within allowed ranges (0.0-10.0)

These balanced decay times make the system responsive while avoiding excessive
flicker or instability. This provides a good balance between responsiveness and
stability for highway navigation.

Visualization notes:
- Red cells represent dynamic objects (high cost areas)
- Blue cells represent static obstacles (medium cost areas)
- Green cells represent vegetation (also converted to black for navigation)
- All non-ground detections are converted to black (occupied = 100) in the binary map
- Cells fade to transparent/empty (value = 0) based on the decay times

Added Waypoint Integration:
- Waypoints from CARLA are received via TCP connection
- Waypoints are visualized and added to the binary map
- The combined binary map includes both semantic data and waypoints
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'semantic_waypoints.rviz')
    
    # ==================== DECLARE LAUNCH ARGUMENTS ====================
    # Common arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_show_rviz = DeclareLaunchArgument(
        'show_rviz',
        default_value='true',
        description='Show RViz visualization'
    )
    
    # TCP connection parameters
    declare_lidar_tcp_ip = DeclareLaunchArgument(
        'lidar_tcp_ip',
        default_value='127.0.0.1',
        description='IP address for LiDAR TCP connection'
    )
    
    declare_lidar_tcp_port = DeclareLaunchArgument(
        'lidar_tcp_port',
        default_value='12350',
        description='Port for LiDAR TCP connection'
    )
    
    declare_radar_tcp_ip = DeclareLaunchArgument(
        'radar_tcp_ip',
        default_value='127.0.0.1',  # Use localhost as a safe default
        description='IP address for radar TCP connection'
    )
    
    declare_radar_tcp_port = DeclareLaunchArgument(
        'radar_tcp_port',
        default_value='12348',
        description='Port for radar TCP connection'
    )
    
    # Add additional radar connection parameters
    declare_radar_reconnect_interval = DeclareLaunchArgument(
        'radar_reconnect_interval',
        default_value='2.0',  # Increase from 1.0 to 2.0 for more time between retries
        description='Seconds between radar reconnection attempts'
    )

    declare_radar_connection_timeout = DeclareLaunchArgument(
        'radar_connection_timeout',
        default_value='10.0',  # Increase from 5.0 to 10.0 for longer timeout
        description='Timeout in seconds for radar connection attempts'
    )
    
    # Add waypoint TCP connection parameters
    declare_waypoint_tcp_ip = DeclareLaunchArgument(
        'waypoint_tcp_ip',
        default_value='127.0.0.1',
        description='IP address for Waypoint TCP connection'
    )
    
    declare_waypoint_tcp_port = DeclareLaunchArgument(
        'waypoint_tcp_port',
        default_value='12343',
        description='Port for Waypoint TCP connection'
    )
    
    declare_waypoint_reconnect_interval = DeclareLaunchArgument(
        'waypoint_reconnect_interval',
        default_value='2.0',
        description='Seconds between waypoint reconnection attempts'
    )
    
    declare_waypoint_connection_timeout = DeclareLaunchArgument(
        'waypoint_connection_timeout',
        default_value='10.0',
        description='Timeout in seconds for waypoint connection attempts'
    )
    
    # TF Tree parameters
    declare_vehicle_frame_id = DeclareLaunchArgument(
        'vehicle_frame_id',
        default_value='base_link',
        description='Frame ID for the vehicle'
    )
    
    declare_map_frame_id = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Frame ID for the map'
    )
    
    # Semantic Costmap parameters - Updated from test1.yaml
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.5',  # Updated from 0.2 to 0.5
        description='Resolution of the costmap in meters per cell'
    )
    
    declare_map_width = DeclareLaunchArgument(
        'map_width_meters',
        default_value='120.0',  # Unchanged
        description='Width of the costmap in meters'
    )
    
    declare_map_height = DeclareLaunchArgument(
        'map_height_meters',
        default_value='120.0',  # Unchanged
        description='Height of the costmap in meters'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',  # Reduced from 60.0 to 30.0 for more stable visualization
        description='Rate at which to publish costmap layers (Hz)'
    )
    
    declare_temporal_filtering = DeclareLaunchArgument(
        'temporal_filtering',
        default_value='true',
        description='Enable temporal filtering of costmap layers'
    )
    
    declare_motion_prediction = DeclareLaunchArgument(
        'motion_prediction',
        default_value='false',
        description='Enable motion prediction for dynamic objects'
    )
    
    # Classification parameters
    declare_ground_height_threshold = DeclareLaunchArgument(
        'ground_height_threshold',
        default_value='0.05',
        description='Maximum height for ground classification (meters)'
    )
    
    declare_vegetation_height_ratio = DeclareLaunchArgument(
        'vegetation_height_ratio',
        default_value='2.0',
        description='Height to width ratio for vegetation classification'
    )
    
    declare_building_width_threshold = DeclareLaunchArgument(
        'building_width_threshold',
        default_value='5.0',
        description='Minimum width for building classification (meters)'
    )
    
    declare_dynamic_velocity_threshold = DeclareLaunchArgument(
        'dynamic_velocity_threshold',
        default_value='0.1',
        description='Minimum velocity for dynamic classification (m/s)'
    )
    
    # Layer weight parameters - Updated based on test2.yaml
    declare_ground_weight = DeclareLaunchArgument(
        'ground_weight',
        default_value='5.0',  # Changed from 0.0 to 5.0 to include ground in the combined map
        description='Weight of ground layer in combined map'
    )
    
    declare_obstacle_weight = DeclareLaunchArgument(
        'obstacle_weight',
        default_value='10.0',  # Increased to maximum allowed value (10.0) for clear visualization
        description='Weight of obstacle layer in combined map'
    )
    
    declare_vegetation_weight = DeclareLaunchArgument(
        'vegetation_weight',
        default_value='10.0',  # Increased to maximum allowed value (10.0) to ensure vegetation is properly detected
        description='Weight of vegetation layer in combined map'
    )
    
    declare_building_weight = DeclareLaunchArgument(
        'building_weight',
        default_value='10.0',  # Increased to maximum allowed value (10.0) for clear visualization
        description='Weight of building layer in combined map'
    )
    
    declare_dynamic_weight = DeclareLaunchArgument(
        'dynamic_weight',
        default_value='10.0',  # Maximum allowed value
        description='Weight of dynamic layer in combined map'
    )
    
    declare_enable_3d_visualization = DeclareLaunchArgument(
        'enable_3d_visualization',
        default_value='true',
        description='Enable 3D visualization of the costmap'
    )
    
    declare_enable_text_labels = DeclareLaunchArgument(
        'enable_text_labels',
        default_value='true',
        description='Enable text labels for semantic categories'
    )
    
    # Vehicle parameters for filtering
    declare_vehicle_length = DeclareLaunchArgument(
        'vehicle_length',
        default_value='5.0',
        description='Length of the vehicle for point filtering'
    )
    
    declare_vehicle_width = DeclareLaunchArgument(
        'vehicle_width',
        default_value='2.4',
        description='Width of the vehicle for point filtering'
    )
    
    declare_vehicle_height = DeclareLaunchArgument(
        'vehicle_height',
        default_value='2.0',
        description='Height of the vehicle for point filtering'
    )
    
    # Add rqt_reconfigure node for parameter tuning - enabled by default
    declare_enable_tuning = DeclareLaunchArgument(
        'enable_tuning',
        default_value='true',  # Enable tuning by default
        description='Enable rqt_reconfigure for parameter tuning'
    )
    
    # Add decay time parameter for better highway navigation
    declare_decay_time = DeclareLaunchArgument(
        'decay_time',
        default_value='0.1',  # Increased from 0.01s to 0.1s for more stable visualization
        description='Decay time for costmap cells in seconds'
    )
    
    # Add dynamic object specific decay time
    declare_dynamic_decay_time = DeclareLaunchArgument(
        'dynamic_decay_time',
        default_value='0.05',  # Increased from 0.005s to 0.05s for more stable tracking
        description='Decay time specifically for dynamic objects in seconds'
    )
    
    # Add cell memory parameter for radar map
    declare_cell_memory = DeclareLaunchArgument(
        'cell_memory',
        default_value='0.1',  # Increased from 0.01s to 0.1s for more stable radar map
        description='Memory time for radar map cells in seconds'
    )
    
    # Add tracking age parameter for highway speeds
    declare_max_tracking_age = DeclareLaunchArgument(
        'max_tracking_age',
        default_value='0.2',  # Increased from 0.05s to 0.2s for more stable tracking
        description='Maximum age for tracked objects in seconds'
    )
    
    # Add marker lifetime parameter
    declare_marker_lifetime = DeclareLaunchArgument(
        'marker_lifetime',
        default_value='0.1',  # Increased from 0.01s to 0.1s for more stable visualization
        description='Lifetime of visualization markers in seconds'
    )
    
    # Add parameters for binary output map
    declare_binary_threshold = DeclareLaunchArgument(
        'binary_threshold',
        default_value='0.3',  # Increased from 0.05 to 0.3 for more precise obstacle classification
        description='Threshold value for binary obstacle classification'
    )
    
    declare_enable_binary_output = DeclareLaunchArgument(
        'enable_binary_output',
        default_value='true',  # Enable binary output by default
        description='Enable binary (black/white) output map for navigation'
    )
    
    declare_binary_topic = DeclareLaunchArgument(
        'binary_topic',
        default_value='/semantic_costmap/binary',
        description='Topic name for publishing binary obstacle map'
    )
    
    # Add parameters for map saving
    declare_enable_map_saving = DeclareLaunchArgument(
        'enable_map_saving',
        default_value='true',  # Disabled by default
        description='Enable saving maps to local files'
    )
    
    declare_save_directory = DeclareLaunchArgument(
        'save_directory',
        # default_value='/home/shishtawy/Documents/ROS2/maps/NewMaps',
        default_value='/home/mostafa/GP/ROS2/maps/NewMaps',
        description='Directory to save map files'
    )
    
    declare_save_interval = DeclareLaunchArgument(
        'save_interval',
        default_value='5.0',  # Updated from 3.0 to 5.0 seconds
        description='Interval in seconds between map saves'
    )
    
    declare_save_binary_map = DeclareLaunchArgument(
        'save_binary_map',
        default_value='true',  # Save binary map by default
        description='Save the binary map'
    )
    
    declare_save_combined_map = DeclareLaunchArgument(
        'save_combined_map',
        default_value='true',  # Save combined map by default
        description='Save the combined map'
    )
    
    declare_save_layer_maps = DeclareLaunchArgument(
        'save_layer_maps',
        default_value='false',  # Don't save layer maps by default
        description='Save individual layer maps'
    )
    
    declare_occupied_value = DeclareLaunchArgument(
        'occupied_value',
        default_value='100',
        description='Value for occupied cells in the binary map (black)'
    )
    
    declare_free_value = DeclareLaunchArgument(
        'free_value',
        default_value='0',
        description='Value for free cells in the binary map (white/transparent)'
    )
    
    declare_map_format = DeclareLaunchArgument(
        'map_format',
        default_value='nav_msgs/OccupancyGrid',
        description='Format of the output map message'
    )
    
    declare_publish_binary_map = DeclareLaunchArgument(
        'publish_binary_map',
        default_value='true',
        description='Whether to publish the binary map'
    )
    
    # New parameter for comprehensive object inclusion
    declare_include_all_objects = DeclareLaunchArgument(
        'include_all_objects',
        default_value='true',
        description='Include all detected objects in the binary map, regardless of classification'
    )

    # New parameter for dedicated binary map
    declare_enhanced_binary_map = DeclareLaunchArgument(
        'enhanced_binary_map',
        default_value='true',
        description='Generate an enhanced binary map with all detected objects'
    )
    
    # New parameter for binary map topic
    declare_all_objects_binary_topic = DeclareLaunchArgument(
        'all_objects_binary_topic',
        default_value='/semantic_costmap/all_objects_binary',
        description='Topic name for publishing comprehensive binary map with all objects'
    )
    
    # Add a new parameter for low car detection
    declare_low_car_height_threshold = DeclareLaunchArgument(
        'low_car_height_threshold',
        default_value='0.03',  # Threshold for detecting low-profile cars (3cm)
        description='Minimum height for low-profile car detection (meters)'
    )
    
    declare_car_detection_width = DeclareLaunchArgument(
        'car_detection_width',
        default_value='0.5',  # Minimum width for car detection (50cm)
        description='Minimum width for car detection (meters)'
    )

    declare_enhance_low_cars = DeclareLaunchArgument(
        'enhance_low_cars',
        default_value='true',
        description='Enhance detection of low-profile cars'
    )
    
    # Add new parameters for object expansion control
    declare_car_expansion_radius = DeclareLaunchArgument(
        'car_expansion_radius',
        default_value='2.0',  # Reduced from 3.0 to 2.0 for even smaller car expansion
        description='Expansion radius for cars in cells'
    )

    declare_obstacle_expansion_radius = DeclareLaunchArgument(
        'obstacle_expansion_radius',
        default_value='0.5',  # Reduced from 1.0 to 0.5 for minimal obstacle expansion
        description='Expansion radius for obstacles in cells'
    )

    declare_dynamic_expansion_radius = DeclareLaunchArgument(
        'dynamic_expansion_radius',
        default_value='1.0',  # Reduced from 2.0 to 1.0 for smaller dynamic object expansion
        description='Expansion radius for dynamic objects in cells'
    )
    
    # Add waypoint visualization parameters
    declare_waypoint_marker_size = DeclareLaunchArgument(
        'waypoint_marker_size',
        default_value='0.5',
        description='Size of waypoint markers in meters'
    )
    
    declare_waypoint_line_width = DeclareLaunchArgument(
        'waypoint_line_width',
        default_value='0.2',
        description='Width of lines connecting waypoints in meters'
    )
    
    declare_waypoint_lifetime = DeclareLaunchArgument(
        'waypoint_lifetime',
        default_value='0.0',  # 0.0 means markers never expire
        description='Lifetime of waypoint visualization markers in seconds (0.0 = never expire)'
    )
    
    declare_waypoint_width = DeclareLaunchArgument(
        'waypoint_width',
        default_value='1.0',
        description='Width of waypoints in cells on the binary map'
    )
    
    # Add parameter for waypoint map topic
    declare_waypoint_binary_topic = DeclareLaunchArgument(
        'waypoint_binary_topic',
        default_value='/waypoint_map/binary',
        description='Topic name for publishing binary waypoint map'
    )
    
    # Add parameter for combined binary map topic
    declare_combined_binary_topic = DeclareLaunchArgument(
        'combined_binary_topic',
        default_value='/combined_binary_map',
        description='Topic name for publishing combined binary map with waypoints'
    )
    
    # Add parameters for local coordinate transformation
    declare_use_local_coordinates = DeclareLaunchArgument(
        'use_local_coordinates',
        default_value='true',
        description='Transform waypoints to local coordinates relative to map origin'
    )
    
    declare_fixed_origin = DeclareLaunchArgument(
        'fixed_origin',
        default_value='true',
        description='Use fixed origin for waypoints to prevent them from moving away'
    )
    
    declare_persistent_markers = DeclareLaunchArgument(
        'persistent_markers',
        default_value='true',
        description='Keep waypoint markers persistent on the map'
    )
    
    # Add map origin parameters to ensure waypoints are local to the origin
    declare_map_origin_x = DeclareLaunchArgument(
        'map_origin_x',
        default_value='-60.0',
        description='X-coordinate of map origin in meters'
    )
    
    declare_map_origin_y = DeclareLaunchArgument(
        'map_origin_y',
        default_value='-60.0',
        description='Y-coordinate of map origin in meters'
    )
    
    # Add IMU TCP connection parameters
    declare_imu_tcp_ip = DeclareLaunchArgument(
        'imu_tcp_ip',
        default_value='127.0.0.1',
        description='IP address for IMU TCP connection'
    )
    
    declare_imu_tcp_port = DeclareLaunchArgument(
        'imu_tcp_port',
        default_value='12345',  # Use an appropriate port for IMU
        description='Port for IMU TCP connection'
    )
    
    declare_imu_reconnect_interval = DeclareLaunchArgument(
        'imu_reconnect_interval',
        default_value='2.0',
        description='Seconds between IMU reconnection attempts'
    )

    declare_imu_connection_timeout = DeclareLaunchArgument(
        'imu_connection_timeout',
        default_value='10.0',
        description='Timeout in seconds for IMU connection attempts'
    )
    
    # Add IMU frame parameters
    declare_imu_frame_id = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='Frame ID for the IMU'
    )
    
    # ==================== TF TREE CONFIGURATION ====================
    # Root transform: world to map
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Map to base_link transform
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Map to waypoint_frame transform (fixed in map frame)
    map_to_waypoint_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_waypoint',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'waypoint_frame'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Base to LiDAR transform with rotation
    base_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=[
            '1.5',  # X offset - LiDAR is 1.5m forward from base
            '0.0',  # Y offset - centered on vehicle
            '1.9',  # Z offset - LiDAR is 1.9m above the base
            '90',    # Roll - no roll (0 degrees)
            '0',    # Pitch - no pitch (0 degrees)
            '90',   # Yaw - 90 degree rotation (looking to the side)
            'base_link', 
            'lidar_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Base to Radar transform with rotation
    base_to_radar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_radar',
        arguments=[
            '1.5',  # X offset - Radar is 1.5m forward from base
            '0.5',  # Y offset - Radar is 0.5m to the right
            '1.9',  # Z offset - Radar is 1.9m above the base
            '0',    # Roll - no roll (0 degrees)
            '0',    # Pitch - no pitch (0 degrees)
            '0',    # Yaw - 0 degree rotation (looking forward, perpendicular to LiDAR)
            'base_link', 
            'radar_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Base to IMU transform
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '0.0',  # X offset - IMU is centered on the vehicle
            '0.0',  # Y offset - centered on vehicle
            '1.5',  # Z offset - IMU is 1.5m above the base
            '0',    # Roll - no roll (0 degrees)
            '0',    # Pitch - no pitch (0 degrees)
            '0',    # Yaw - no yaw (0 degrees)
            'base_link', 
            'imu_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ==================== SENSOR NODES ====================
    # LiDAR Listener Node - Updated from test1_lidar.yaml
    lidar_listener_node = Node(
        package='sensor_fusion_2',
        executable='lidar_listener_clusters_3',
        name='lidar_cube_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'filter_vehicle_points': True,
            'vehicle_length': LaunchConfiguration('vehicle_length'),          
            'vehicle_width': LaunchConfiguration('vehicle_width'),           
            'vehicle_height': LaunchConfiguration('vehicle_height'),
            'enable_motion_compensation': True,
            'use_imu_data': True,  # Use IMU data for motion compensation
            'frame_id': 'lidar_link',
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),  # Added IMU frame ID
            'imu_topic': '/imu/data',  # Topic published by imu_euler_visualizer
            'use_tf_transform': True,  # Make sure TF transform is enabled
            'publish_grid_map': True,
            'map_topic': '/lidar/map',
            'map_frame_id': 'map',  # Ensure map frame is set correctly
            'point_size': 2.0,
            'cube_alpha': 0.8,
            'use_cluster_stats': True,
            'use_convex_hull': True,
            'use_point_markers': True,
            'vehicle_safety_margin': 0.5,
            'vehicle_visualization': True,
            'vehicle_x_offset': 0.0,
            'vehicle_y_offset': 0.0,
            'vehicle_z_offset': -1.0,
            'center_size': 3.0,
            'lidar_lower_fov': -5.0,
            'lidar_upper_fov': 10.0,
            'lidar_pitch_angle': 0.0,
            'min_point_distance': 0.0,
            'max_negative_z': -100.0,
            'verbose_logging': True,  # Enable verbose logging for debugging
        }],
        output='screen'
    )
    
    # Radar Listener Node - Updated from test1_radar.yaml
    radar_listener_node = Node(
        package='sensor_fusion_2',
        executable='radar_listener_clusters',
        name='radar_listener_clusters',
        parameters=[{
            'tcp_ip': LaunchConfiguration('radar_tcp_ip'),
            'tcp_port': LaunchConfiguration('radar_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'grid_resolution': 0.2,
            'grid_width': LaunchConfiguration('map_width_meters'),
            'grid_height': LaunchConfiguration('map_height_meters'),
            'show_velocity_vectors': True,
            'radar_to_map_fusion': True,
            'marker_lifetime': LaunchConfiguration('marker_lifetime'),
            'point_size': 0.2,
            'use_advanced_coloring': True,
            'velocity_arrow_scale': 1.0,
            'min_velocity_for_display': 0.0,  # Set to 0.0 to display all objects regardless of velocity
            'max_velocity_for_display': 30.0,
            'frame_id': 'radar_link',
            'map_frame_id': 'map',
            'points_topic': '/radar/points',
            'clusters_topic': '/radar/clusters',
            'velocity_vectors_topic': '/radar/velocity_vectors',
            'moving_objects_topic': '/radar/moving_objects',
            'static_objects_topic': '/radar/static_objects',
            'object_tracking_topic': '/radar/object_tracking',
            'min_points_per_cluster': 1,  # Keep at 1 to detect sparse clusters
            'cluster_distance_threshold': 0.4,  # Keep at 0.4 for precise clustering
            'static_velocity_threshold': 0.0,  # Set to 0.0 to detect all objects regardless of velocity
            'moving_velocity_threshold': 0.0,  # Set to 0.0 to consider any movement as a moving object
            'use_dbscan_clustering': True,
            'dbscan_epsilon': 0.7,  # Further reduced from 0.8 to 0.7 for tighter clustering
            'dbscan_min_samples': 1,  # Keep at 1 to detect even single-point cars
            'track_objects': True,
            'max_tracking_age': LaunchConfiguration('max_tracking_age'),
            'min_track_confidence': 0.6,
            'verbose_logging': True,
            'publish_rate': 60.0,  # Reduced from 100.0 to 60.0 Hz for more stable processing
            'moving_object_color_r': 1.0,
            'moving_object_color_g': 0.0,
            'moving_object_color_b': 0.0,
            'moving_object_color_a': 0.8,
            'static_object_color_r': 0.0,
            'static_object_color_g': 0.0,
            'static_object_color_b': 1.0,
            'static_object_color_a': 0.8,
            # Add additional connection parameters
            'radar_host': LaunchConfiguration('radar_tcp_ip'),  # Use same IP as tcp_ip
            'radar_port': LaunchConfiguration('radar_tcp_port'),  # Use same port as tcp_port
            'reconnect_interval': LaunchConfiguration('radar_reconnect_interval'),
            'connection_timeout': LaunchConfiguration('radar_connection_timeout'),
            'auto_reconnect': True,
            'socket_buffer_size': 262144,  # Increased buffer size
            'socket_timeout': 0.5,  # Increased timeout
            'enable_tcp_nodelay': True,
            'enable_socket_keepalive': True,
        }],
        output='screen'
    )
    
    # Radar Object Detector Node - Updated from test1_radar.yaml
    radar_object_detector_node = Node(
        package='sensor_fusion_2',
        executable='radar_object_detector',
        name='radar_object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'radar_link',  # Unchanged
            'map_frame_id': 'map',  # Unchanged
            'points_topic': '/radar/points',  # Unchanged
            'moving_objects_topic': '/radar/moving_objects',  # Unchanged
            'static_objects_topic': '/radar/static_objects',  # Unchanged
            'object_tracking_topic': '/radar/object_tracking',  # Unchanged
            'min_points_per_cluster': 1,  # Keep at 1 to detect sparse clusters from low-profile cars
            'cluster_distance_threshold': 0.8,  # Increased from 0.7 to 0.8 for better clustering of small objects
            'static_velocity_threshold': 0.0,  # Set to 0.0 to detect all objects regardless of velocity
            'moving_velocity_threshold': 0.0,  # Set to 0.0 to consider any movement as a moving object
            'use_dbscan_clustering': True,  # Unchanged
            'dbscan_epsilon': 0.8,  # Increased from 0.7 to 0.8 for better clustering of low-profile cars
            'dbscan_min_samples': 1,  # Keep at 1 to detect even single-point low-profile cars
            'track_objects': True,  # Unchanged
            'max_tracking_age': 0.3,  # Increased from 0.2 to 0.3 for better tracking of intermittent detections
            'min_track_confidence': 0.5,  # Reduced from 0.6 to 0.5 to maintain tracking of low-confidence detections
            'verbose_logging': True,  # Keep enabled for debugging
            'publish_rate': 60.0,  # Keep at 60.0 Hz for responsive updates
            'moving_object_color_r': 1.0,  # Unchanged
            'moving_object_color_g': 0.0,  # Unchanged
            'moving_object_color_b': 0.0,  # Unchanged
            'moving_object_color_a': 0.8,  # Unchanged
            'static_object_color_r': 0.0,  # Unchanged
            'static_object_color_g': 0.0,  # Unchanged
            'static_object_color_b': 1.0,  # Unchanged
            'static_object_color_a': 0.8,  # Unchanged
            'marker_lifetime': 0.2,  # Increased from 0.1 to 0.2 for better visualization
        }]
    )
    
    # LiDAR Realtime Mapper Node
    lidar_realtime_mapper_node = Node(
        package='sensor_fusion_2',
        executable='lidar_realtime_mapper',
        name='lidar_realtime_mapper',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            # Calculate origin dynamically - using float values instead of strings
            'map_origin_x': -60.0,  # Half of highway width (120.0/2)
            'map_origin_y': -60.0,  # Half of highway height (120.0/2)
            'publish_rate': 5.0,
            'process_rate': 10.0,
            'map_topic': '/lidar/map',
            'map_frame': LaunchConfiguration('map_frame_id'),
        }],
        output='screen'
    )
    
    # Radar Map Generator Node - Updated from test2_radar.yaml
    radar_map_generator_node = Node(
        package='sensor_fusion_2',
        executable='radar_map_generator',
        name='radar_map_generator',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width': LaunchConfiguration('map_width_meters'),
            'map_height': LaunchConfiguration('map_height_meters'),
            'map_frame': LaunchConfiguration('map_frame_id'),
            'radar_topic': '/radar/points',
            'map_topic': '/radar/map',
            'publish_rate': 30.0,  # Keep at 30.0 for stable map updates
            'use_velocity_filter': False,  # Keep disabled to include all points regardless of velocity
            'use_temporal_filtering': True,
            'min_velocity': 0.0,  # Keep at 0.0 to detect even stationary objects
            'cell_memory': 0.3,  # Increased from current value to 0.3 for better persistence of detected objects
            'obstacle_threshold': 0.0,  # Keep at minimum to detect all objects
            'free_threshold': -0.3,  # Reduced from -0.2 to -0.3 to be more conservative about marking cells as free
            'start_type_description_service': True,
            'enable_fusion_layer': True,  # Keep enabled
            'obstacle_value': 100,  # Maximum obstacle value
            'publish_to_realtime_map': True,  # Added from test2_radar.yaml
            'realtime_map_topic': '/realtime_map',  # Added from test2_radar.yaml
            'use_reliable_qos': True,  # Added from test2_radar.yaml
            'use_transient_local_durability': True,  # Added from test2_radar.yaml
        }],
        output='screen'
    )
    
    # Semantic Costmap Visualizer Node
    semantic_costmap_node = Node(
        package='sensor_fusion_2',
        executable='semantic_costmap_visualizer',
        name='semantic_costmap_visualizer',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'map_frame': LaunchConfiguration('map_frame_id'),
            'lidar_points_topic': '/lidar/points',
            'lidar_clusters_topic': '/lidar/cubes',
            'radar_points_topic': '/radar/points',
            'radar_clusters_topic': '/radar/clusters',
            'publish_rate': LaunchConfiguration('publish_rate'),
            'temporal_filtering': LaunchConfiguration('temporal_filtering'),
            'motion_prediction': LaunchConfiguration('motion_prediction'),
            'min_confidence': 0.5,
            'decay_time': LaunchConfiguration('decay_time'),
            'dynamic_decay_time': LaunchConfiguration('dynamic_decay_time'),
            'ground_height_threshold': LaunchConfiguration('ground_height_threshold'),
            'vegetation_height_ratio': LaunchConfiguration('vegetation_height_ratio'),
            'building_width_threshold': LaunchConfiguration('building_width_threshold'),
            'dynamic_velocity_threshold': 0.1,  # Set to minimum allowed value (0.1) to detect objects with minimal velocity
            'ground_weight': LaunchConfiguration('ground_weight'),
            'obstacle_weight': LaunchConfiguration('obstacle_weight'),
            'vegetation_weight': LaunchConfiguration('vegetation_weight'),
            'building_weight': LaunchConfiguration('building_weight'),
            'dynamic_weight': LaunchConfiguration('dynamic_weight'),
            'enable_3d_visualization': LaunchConfiguration('enable_3d_visualization'),
            'enable_text_labels': LaunchConfiguration('enable_text_labels'),
            'start_type_description_service': True,
            
            # Binary output for navigation - simplified parameters
            'enable_binary_output': True,
            'binary_topic': LaunchConfiguration('binary_topic'),
            'occupied_value': 100,
            'free_value': 0,
            'binary_threshold': LaunchConfiguration('binary_threshold'),
            'convert_vegetation_to_occupied': True,  # Explicitly convert vegetation to occupied
            'convert_all_non_ground_to_occupied': True,  # Changed from False to True - mark all non-ground as occupied
            'include_ground_in_binary': True,  # New parameter to explicitly include ground in binary map
            'ground_as_occupied': False,  # Changed from True to False - don't mark ground as occupied
            
            # Enhanced binary map with all objects
            'enable_enhanced_binary_map': LaunchConfiguration('enhanced_binary_map'),
            'all_objects_binary_topic': LaunchConfiguration('all_objects_binary_topic'),
            'include_all_objects': LaunchConfiguration('include_all_objects'),
            'include_radar_objects': True,
            'include_lidar_objects': True,
            'include_dynamic_objects': True,
            'include_static_objects': True,
            'use_lower_threshold_for_all_objects': True,
            'all_objects_threshold': 0.01,  # Very low threshold to include even faint detections
            
            # Map saving parameters
            'enable_map_saving': LaunchConfiguration('enable_map_saving'),
            'save_directory': LaunchConfiguration('save_directory'),
            'save_interval': LaunchConfiguration('save_interval'),
            'save_binary_map': LaunchConfiguration('save_binary_map'),
            'save_combined_map': LaunchConfiguration('save_combined_map'),
            'save_layer_maps': LaunchConfiguration('save_layer_maps'),
            
            # Add new parameters for low car detection
            'enhance_low_cars': LaunchConfiguration('enhance_low_cars'),
            'low_car_height_threshold': LaunchConfiguration('low_car_height_threshold'),
            'car_detection_width': LaunchConfiguration('car_detection_width'),
            
            # Add new parameters for object expansion control
            'car_expansion_radius': LaunchConfiguration('car_expansion_radius'),
            'obstacle_expansion_radius': LaunchConfiguration('obstacle_expansion_radius'),
            'dynamic_expansion_radius': LaunchConfiguration('dynamic_expansion_radius'),
        }],
        output='screen'
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('show_rviz')),
        output='screen'
    )
    
    # Add rqt_reconfigure node for parameter tuning
    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        condition=IfCondition(LaunchConfiguration('enable_tuning')),
        output='screen'
    )
    
    # ==================== WAYPOINT LISTENER NODE ====================
    waypoint_listener_node = Node(
        package='sensor_fusion_2',
        executable='waypoint_listener',
        name='waypoint_listener',
        parameters=[{
            'tcp_ip': LaunchConfiguration('waypoint_tcp_ip'),
            'tcp_port': LaunchConfiguration('waypoint_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'waypoint_frame',  # Use waypoint_frame to keep waypoints fixed
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'waypoints_topic': '/carla/waypoints',
            'marker_topic': '/carla/waypoint_markers',
            'waypoint_marker_size': LaunchConfiguration('waypoint_marker_size'),
            'waypoint_line_width': LaunchConfiguration('waypoint_line_width'),
            'waypoint_lifetime': LaunchConfiguration('waypoint_lifetime'),
            'reconnect_interval': LaunchConfiguration('waypoint_reconnect_interval'),
            'connection_timeout': LaunchConfiguration('waypoint_connection_timeout'),
            'auto_reconnect': True,
            'socket_buffer_size': 262144,
            'socket_timeout': 0.5,
            'enable_tcp_nodelay': True,
            'enable_socket_keepalive': True,
            'verbose_logging': True,
            'use_local_coordinates': LaunchConfiguration('use_local_coordinates'),
            'fixed_origin': LaunchConfiguration('fixed_origin'),
            'persistent_markers': LaunchConfiguration('persistent_markers'),
            'clear_markers_on_update': False,  # Don't clear previous markers when new ones arrive
            'use_persistent_durability': True,  # Use persistent durability for markers
        }],
        output='screen'
    )
    
    # ==================== WAYPOINT MAP GENERATOR NODE ====================
    waypoint_map_generator_node = Node(
        package='sensor_fusion_2',
        executable='waypoint_map_generator',
        name='waypoint_map_generator',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'map_frame_id': 'waypoint_frame',  # Use waypoint_frame for consistency
            'publish_rate': LaunchConfiguration('publish_rate'),
            'waypoint_marker_topic': '/carla/waypoint_markers',
            'binary_topic': LaunchConfiguration('waypoint_binary_topic'),
            'occupied_value': LaunchConfiguration('occupied_value'),
            'free_value': LaunchConfiguration('free_value'),
            'waypoint_width': LaunchConfiguration('waypoint_width'),
            'use_vehicle_frame': False,  # Don't center on vehicle
            'map_origin_x': LaunchConfiguration('map_origin_x'),
            'map_origin_y': LaunchConfiguration('map_origin_y'),
        }],
        output='screen'
    )
    
    # ==================== BINARY MAP COMBINER NODE ====================
    binary_map_combiner_node = Node(
        package='sensor_fusion_2',
        executable='binary_map_combiner',
        name='binary_map_combiner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'semantic_binary_topic': LaunchConfiguration('binary_topic'),
            'waypoint_binary_topic': LaunchConfiguration('waypoint_binary_topic'),
            'combined_binary_topic': LaunchConfiguration('combined_binary_topic'),
            'occupied_value': LaunchConfiguration('occupied_value'),
            'free_value': LaunchConfiguration('free_value'),
            'prioritize_waypoints': True,  # Waypoints take priority over semantic data
            'use_transient_local_durability': True,
            'verbose_logging': True,  # Add verbose logging for debugging
        }],
        output='screen'
    )
    
    # IMU Listener Node
    imu_listener_node = Node(
        package='sensor_fusion_2',
        executable='imu_euler_visualizer_simple',  # Use the full version of the IMU visualizer
        name='imu_euler_visualizer_simple',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'frame_id': LaunchConfiguration('imu_frame_id'),
            'world_frame_id': 'map',  # Use map as the world frame
            'reconnect_interval': LaunchConfiguration('imu_reconnect_interval'),
            'filter_window_size': 5,  # Use a moderate filter window size for smooth data
            'queue_size': 20,  # Buffer size for IMU data
            'publish_rate': 100.0,  # High rate for accurate motion compensation
            'socket_buffer_size': 262144,
            'enable_bias_correction': True,  # Enable gyro bias correction
            'enable_complementary_filter': True,  # Use complementary filter for sensor fusion
            'zero_velocity_threshold': 0.02,  # m/s^2
            'yaw_offset': 0.0,  # No yaw offset by default
            'road_plane_correction': True,  # Correct for road plane
            'gravity_aligned': True,  # Align with gravity
        }],
        output='screen'
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim_time,
        declare_show_rviz,
        declare_lidar_tcp_ip,
        declare_lidar_tcp_port,
        declare_radar_tcp_ip,
        declare_radar_tcp_port,
        # Add new radar connection arguments
        declare_radar_reconnect_interval,
        declare_radar_connection_timeout,
        # Add waypoint connection arguments
        declare_waypoint_tcp_ip,
        declare_waypoint_tcp_port,
        declare_waypoint_reconnect_interval,
        declare_waypoint_connection_timeout,
        declare_vehicle_frame_id,
        declare_map_frame_id,
        declare_map_resolution,
        declare_map_width,
        declare_map_height,
        declare_map_origin_x,
        declare_map_origin_y,
        declare_publish_rate,
        # Add waypoint visualization parameters
        declare_waypoint_marker_size,
        declare_waypoint_line_width,
        declare_waypoint_lifetime,
        declare_waypoint_width,
        declare_waypoint_binary_topic,
        declare_combined_binary_topic,
        declare_use_local_coordinates,
        declare_fixed_origin,
        declare_persistent_markers,
        
        # Add the rest of the parameters that were missing
        declare_temporal_filtering,
        declare_motion_prediction,
        declare_ground_height_threshold,
        declare_vegetation_height_ratio,
        declare_building_width_threshold,
        declare_dynamic_velocity_threshold,
        declare_ground_weight,
        declare_obstacle_weight,
        declare_vegetation_weight,
        declare_building_weight,
        declare_dynamic_weight,
        declare_enable_3d_visualization,
        declare_enable_text_labels,
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        declare_enable_tuning,
        declare_decay_time,
        declare_dynamic_decay_time,
        declare_cell_memory,
        declare_max_tracking_age,
        declare_marker_lifetime,
        declare_binary_threshold,
        declare_enable_binary_output,
        declare_binary_topic,
        declare_enable_map_saving,
        declare_save_directory,
        declare_save_interval,
        declare_save_binary_map,
        declare_save_combined_map,
        declare_save_layer_maps,
        declare_occupied_value,
        declare_free_value,
        declare_map_format,
        declare_publish_binary_map,
        declare_include_all_objects,
        declare_enhanced_binary_map,
        declare_all_objects_binary_topic,
        declare_low_car_height_threshold,
        declare_car_detection_width,
        declare_enhance_low_cars,
        declare_car_expansion_radius,
        declare_obstacle_expansion_radius,
        declare_dynamic_expansion_radius,
        
        # Add IMU connection parameters
        declare_imu_tcp_ip,
        declare_imu_tcp_port,
        declare_imu_reconnect_interval,
        declare_imu_connection_timeout,
        
        # Add IMU frame parameters
        declare_imu_frame_id,
        
        # TF Tree Nodes - Launch these first to establish the TF tree
        world_to_map_node,
        map_to_base_link_node,
        map_to_waypoint_node,
        base_to_lidar_node,
        base_to_radar_node,
        base_to_imu_node,
        
        # Add a timing delay to ensure TF tree is established before other nodes start
        TimerAction(
            period=2.0,  # Wait 2 seconds for TF tree to be established
            actions=[
                # Sensor Nodes
                lidar_listener_node,
                radar_listener_node,
                radar_object_detector_node,
                lidar_realtime_mapper_node,
                radar_map_generator_node,
                
                # Waypoint Nodes
                waypoint_listener_node,
                waypoint_map_generator_node,
                
                # Visualization and Integration Nodes
                semantic_costmap_node,
                binary_map_combiner_node,
                rviz_node,
                rqt_reconfigure_node,
                imu_listener_node
            ]
        )
    ]) 