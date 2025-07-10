#!/usr/bin/env python3
"""
Integrated Costmap and Waypoints Launch File
===========================================

Authors: Shishtawy & Hendy
Project: TechZ Autonomous Driving System

OVERVIEW:
This launch file orchestrates a comprehensive sensor fusion and mapping system for autonomous vehicles.
It integrates multiple sensors (LiDAR, Radar, IMU) with waypoint navigation to create a unified
perception and planning system for autonomous driving applications.

KEY FEATURES:
- Multi-sensor fusion (LiDAR, Radar, IMU)
- Real-time semantic costmap generation
- Dynamic obstacle detection and tracking
- Waypoint-based navigation integration
- Fixed map with vehicle-oriented sensors
- Enhanced binary map output for navigation
- Unified map combining all sensor data and waypoints

SYSTEM ARCHITECTURE:
1. Sensor Layer: LiDAR, Radar, IMU data collection
2. Processing Layer: Semantic classification, obstacle detection
3. Integration Layer: Sensor fusion, map generation
4. Navigation Layer: Waypoint processing and visualization
5. Output Layer: Binary and unified map generation

FRAME STRUCTURE:
- odom: Root frame (fixed)
  ├── base_link: Vehicle center
  │   └── imu_link: IMU sensor
  │       └── lidar_link: LiDAR sensor
  ├── map: Global reference frame
  │   └── radar_link: Radar sensor (connected directly to map)
  └── local_map_link: Fixed perception map
      └── waypoint_frame: Navigation waypoints

LAUNCH COMPONENTS:
- TF Tree Nodes: Establish coordinate frame relationships
- Sensor Nodes: Process LiDAR, radar, and IMU data
- Costmap Nodes: Generate semantic and binary maps
- Waypoint Nodes: Process and visualize navigation waypoints
- Integration Nodes: Combine sensor data and waypoints
- Visualization: RViz configuration for system monitoring

PARAMETERS:
- Network: TCP connections for sensor data streams
- Frames: TF tree configuration
- Map: Resolution, dimensions, and update rates
- Sensor: Detection thresholds and filtering options
- Processing: Temporal filtering and motion prediction
- Visualization: 3D visualization and text label options
- Output: Binary map thresholds and save options

MAP CONFIGURATION:
- Fixed map with vehicle-oriented sensors
- Map origin remains stationary while sensors rotate with vehicle
- Sensor data transformed into fixed map frame
- Combined binary map includes semantic data and waypoints
- Maps saved to: /home/shishtawy/Documents/ROS2/maps/NewMaps

USAGE:
- Launch with default parameters: ros2 launch sensor_fusion_2 integrated_costmap_waypoints.launch.py
- Enable/disable RViz: show_rviz:=true|false
- Control map saving: enable_map_saving:=true|false
- Adjust map quality: unified_map_quality:=low|medium|high

This system provides a robust foundation for autonomous navigation by combining
multiple sensor inputs into a unified perception and planning framework.
"""

# Standard library imports
import os                                                                 # File system operations and path manipulation
import time                                                               # Time utilities for timestamps and delays

# ROS2 package utilities
from ament_index_python.packages import get_package_share_directory       # Resolves package paths in the ROS2 workspace

# ROS2 launch framework imports
from launch import LaunchDescription                                      # Main container for launch configurations
from launch.actions import DeclareLaunchArgument                          # Defines configurable launch parameters
from launch.actions import ExecuteProcess                                 # Executes system processes
from launch.actions import TimerAction                                    # Schedules actions with delays
from launch.substitutions import LaunchConfiguration                      # Resolves parameter values at runtime
from launch.substitutions import TextSubstitution                         # Text replacement in launch configurations
from launch_ros.actions import Node                                       # Creates and configures ROS2 nodes
from launch.conditions import IfCondition                                 # Conditional execution of launch elements

def generate_launch_description():
    """
    Main function that generates the launch description for the integrated costmap and waypoints system.
    
    This function:
    1. Sets up file paths and directories
    2. Declares all configurable launch parameters
    3. Configures the TF tree nodes for coordinate transformations
    4. Sets up sensor processing nodes (LiDAR, Radar, IMU)
    5. Configures mapping and visualization nodes
    6. Organizes all components into a complete launch sequence
    
    Returns:
        LaunchDescription: Complete launch configuration for the system
    """
    # Get the package share directory for accessing configuration files
    pkg_share = get_package_share_directory('sensor_fusion_2')            # Locate the sensor_fusion_2 package directory
    
    # Define RViz config file path for visualization setup
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'semantic_waypoints.rviz')  # Path to RViz configuration
    
    # Set default map save directory with automatic creation
    # default_save_dir = '/home/shishtawy/Documents/ROS2/maps/NewMaps'  
    default_save_dir = '/home/mostafa/GP/ROS2/maps/NewMaps'    # Directory for saving generated maps
    os.makedirs(default_save_dir, exist_ok=True)                          # Create directory if it doesn't exist
    
    # ==================== DECLARE LAUNCH ARGUMENTS ====================
    # Common arguments for simulation and visualization
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Default to real-time operation
        description='Use simulation time if true'
    )
    
    declare_show_rviz = DeclareLaunchArgument(
        'show_rviz',
        default_value='true',  # Enable visualization by default
        description='Show RViz visualization'
    )
    
    # TCP connection parameters for sensor data streams
    declare_lidar_tcp_ip = DeclareLaunchArgument(
        'lidar_tcp_ip',
        default_value='127.0.0.1',  # Default to localhost for local testing
        description='IP address for LiDAR TCP connection'
    )
    
    declare_lidar_tcp_port = DeclareLaunchArgument(
        'lidar_tcp_port',
        default_value='12350',  # Dedicated port for LiDAR data
        description='Port for LiDAR TCP connection'
    )
    
    declare_radar_tcp_ip = DeclareLaunchArgument(
        'radar_tcp_ip',
        default_value='127.0.0.1',  # Use localhost for radar connection
        description='IP address for radar TCP connection'
    )
    
    declare_radar_tcp_port = DeclareLaunchArgument(
        'radar_tcp_port',
        default_value='12348',  # Dedicated port for radar data
        description='Port for radar TCP connection'
    )
    
    # Enhanced radar connection parameters for reliability
    declare_radar_reconnect_interval = DeclareLaunchArgument(
        'radar_reconnect_interval',
        default_value='2.0',  # 2-second retry interval for stability
        description='Seconds between radar reconnection attempts'
    )

    declare_radar_connection_timeout = DeclareLaunchArgument(
        'radar_connection_timeout',
        default_value='10.0',  # 10-second timeout for robust connection handling
        description='Timeout in seconds for radar connection attempts'
    )
    
    # Waypoint communication setup
    declare_waypoint_tcp_ip = DeclareLaunchArgument(
        'waypoint_tcp_ip',
        default_value='127.0.0.1',  # Local connection for waypoints
        description='IP address for Waypoint TCP connection'
    )
    
    declare_waypoint_tcp_port = DeclareLaunchArgument(
        'waypoint_tcp_port',
        default_value='12343',  # Dedicated port for waypoint data
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
        default_value='base_link',  # Standard ROS frame for vehicle base
        description='Frame ID for the vehicle'
    )
    
    declare_map_frame_id = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',  # Global reference frame
        description='Frame ID for the map'
    )
    
    declare_odom_frame_id = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',  # Odometry reference frame
        description='Frame ID for the odometry frame'
    )
    
    declare_local_map_frame_id = DeclareLaunchArgument(
        'local_map_frame_id',
        default_value='local_map_link',  # Local perception map frame
        description='Frame ID for the local map'
    )
    
    # Semantic Costmap core parameters for map generation
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.2',  # 20cm resolution for detailed obstacle representation
        description='Resolution of the costmap in meters per cell'
    )
    
    declare_map_width = DeclareLaunchArgument(
        'map_width_meters',
        default_value='120.0',  # 120-meter width for wide area coverage
        description='Width of the costmap in meters'
    )
    
    declare_map_height = DeclareLaunchArgument(
        'map_height_meters',
        default_value='120.0',  # 120-meter height for extended perception
        description='Height of the costmap in meters'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',  # 30Hz update rate for smooth visualization
        description='Rate at which to publish costmap layers (Hz)'
    )
    
    # Advanced costmap processing features
    declare_temporal_filtering = DeclareLaunchArgument(
        'temporal_filtering',
        default_value='true',  # Enable temporal smoothing of map updates
        description='Enable temporal filtering of costmap layers'
    )
    
    declare_motion_prediction = DeclareLaunchArgument(
        'motion_prediction',
        default_value='false',  # Disable motion prediction for static mapping
        description='Enable motion prediction for dynamic objects'
    )
    
    # Object classification parameters
    declare_ground_height_threshold = DeclareLaunchArgument(
        'ground_height_threshold',
        default_value='0.05',  # 5cm threshold for ground point classification
        description='Maximum height for ground classification (meters)'
    )
    
    declare_vegetation_height_ratio = DeclareLaunchArgument(
        'vegetation_height_ratio',
        default_value='2.0',  # Height/width ratio for vegetation detection
        description='Height to width ratio for vegetation classification'
    )
    
    declare_building_width_threshold = DeclareLaunchArgument(
        'building_width_threshold',
        default_value='5.0',  # 5-meter minimum width for building detection
        description='Minimum width for building classification (meters)'
    )
    
    declare_dynamic_velocity_threshold = DeclareLaunchArgument(
        'dynamic_velocity_threshold',
        default_value='0.1',  # 0.1 m/s threshold for dynamic object detection
        description='Minimum velocity for dynamic classification (m/s)'
    )
    
    # Layer weight configuration for map integration
    declare_ground_weight = DeclareLaunchArgument(
        'ground_weight',
        default_value='5.0',  # Moderate weight for ground layer influence
        description='Weight of ground layer in combined map'
    )
    
    declare_obstacle_weight = DeclareLaunchArgument(
        'obstacle_weight',
        default_value='10.0',  # Maximum weight for critical obstacle detection
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
        default_value='true',  # Enable by default
        description='Enable saving unified map to local file'  # Updated description
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
    
    # Add new launch arguments for unified map configuration
    declare_unified_map_topic = DeclareLaunchArgument(
        'unified_map_topic',
        default_value='/unified_map',
        description='Topic name for publishing the unified map'
    )
    
    declare_unified_map_quality = DeclareLaunchArgument(
        'unified_map_quality',
        default_value='high',
        description='Quality of the unified map (options: low, medium, high)'
    )
    
    declare_unified_map_format = DeclareLaunchArgument(
        'unified_map_format',
        default_value='pgm',
        description='Format for saving the unified map'
    )
    
    declare_enhanced_radar_integration = DeclareLaunchArgument(
        'enhanced_radar_integration',
        default_value='true',
        description='Enable enhanced integration of radar data'
    )
    
    declare_enhanced_lidar_integration = DeclareLaunchArgument(
        'enhanced_lidar_integration',
        default_value='true',
        description='Enable enhanced integration of LiDAR data'
    )
    
    declare_add_timestamp_to_filename = DeclareLaunchArgument(
        'add_timestamp_to_filename',
        default_value='true',
        description='Add timestamp to map filenames for uniqueness'
    )
    
    declare_force_save_on_shutdown = DeclareLaunchArgument(
        'force_save_on_shutdown',
        default_value='true',
        description='Save map on shutdown'
    )
    
    # ==================== TF TREE CONFIGURATION - UPDATED ====================
    # Root transform: Establishes the relationship between map and odometry
    map_to_odom_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Publishes fixed transforms
        name='tf_map_to_odom',                                           # Node name for identification
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom'],  # Identity transform at origin (x,y,z,roll,pitch,yaw,parent,child)
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # Base odometry transform - Updated dynamically by IMU-based odometry
    odom_to_base_link_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Initial static transform, later updated by EKF
        name='tf_odom_to_base_link',                                     # Node name for identification
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],  # Initial identity transform (x,y,z,roll,pitch,yaw,parent,child)
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # Base to IMU transform - Defines IMU mounting position on vehicle
    base_to_imu_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Publishes fixed transforms
        name='tf_base_to_imu',                                           # Node name for identification
        arguments=[
            '0.0',  # X: Centered on vehicle longitudinally
            '0.0',  # Y: Centered on vehicle laterally
            '1.5',  # Z: IMU mounted 1.5m above base (roof height)
            '0',    # Roll: No rotation around X axis
            '0',    # Pitch: No rotation around Y axis
            '0',    # Yaw: Forward-facing (no rotation around Z axis)
            'base_link',  # Parent frame - vehicle reference
            'imu_link'    # Child frame - IMU sensor location
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # IMU to LiDAR transform - Defines LiDAR mounting position relative to IMU
    imu_to_lidar_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Publishes fixed transforms
        name='tf_imu_to_lidar',                                          # Node name for identification
        arguments=[
            '1.5',      # X: LiDAR 1.5m forward of IMU (front of vehicle)
            '0.0',      # Y: Centered on vehicle laterally
            '0.4',      # Z: LiDAR 0.4m above IMU (higher on roof)
            '0',        # Roll: No rotation around X axis
            '3.14159',  # Pitch: 180 degrees (π radians) for upright mounting
            '0',        # Yaw: Forward-facing (no rotation around Z axis)
            'imu_link',    # Parent frame - IMU reference
            'lidar_link'   # Child frame - LiDAR sensor location
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # Map to Radar transform - Fixed radar position in the map frame
    map_to_radar_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Publishes fixed transforms
        name='tf_map_to_radar',                                          # Node name for identification
        arguments=[
            '0.0',      # X offset - Radar is fixed at the map origin
            '0.0',      # Y offset - centered at map origin
            '1.9',      # Z offset - Radar is 1.9m above the ground
            '3.14159',  # Roll - 180 degrees (π radians) around X axis
            '0',        # Pitch - no rotation around Y axis
            '0',        # Yaw - no rotation around Z axis
            'map',         # Parent frame - global map reference
            'radar_link'   # Child frame - radar sensor location
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # Map to local_map_link transform - This connects the map frame to our local map frame
    map_to_local_map_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Publishes fixed transforms
        name='tf_map_to_local_map',                                      # Node name for identification
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'local_map_link'],  # Identity transform (x,y,z,roll,pitch,yaw,parent,child)
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # Local map to waypoint frame transform
    local_map_to_waypoint_node = Node(
        package='tf2_ros',                                               # TF2 package for coordinate transforms
        executable='static_transform_publisher',                         # Publishes fixed transforms
        name='tf_local_map_to_waypoint',                                 # Node name for identification
        arguments=[
            '0.0',  # X offset - No horizontal offset from local map
            '0.0',  # Y offset - No lateral offset from local map
            '0.0',  # Z offset - Same height as local map
            '0',    # Roll - no rotation around X axis
            '0',    # Pitch - no rotation around Y axis
            '0',    # Yaw - no rotation around Z axis
            'local_map_link',  # Parent frame - local map reference
            'waypoint_frame'   # Child frame - waypoint visualization frame
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]  # Time source configuration
    )
    
    # ==================== IMU LISTENER NODE ====================
    imu_listener_node = Node(
        package='sensor_fusion_2',
        executable='imu_euler_visualizer_simple',  # Use the full version of the IMU visualizer
        name='imu_euler_visualizer_simple',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'frame_id': 'imu_link',  # FIXED: Explicitly set frame_id
            'world_frame_id': 'odom',  # FIXED: Explicitly set world_frame_id
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
            'publish_tf': True,  # Ensure TF is published for IMU orientation
            'broadcast_transform': True,  # Broadcast the IMU transform
            'update_rate': 100.0,  # High update rate for smooth orientation tracking
            'use_imu_orientation': True,  # Use IMU orientation data
            'use_filtered_orientation': True,  # Use filtered orientation for smoother results
            'use_local_frame': True,  # Explicitly use local frame
            'local_frame_id': 'local_map_link',  # FIXED: Explicitly set local_frame_id
            'tf_buffer_duration': 10.0,  # Increase TF buffer duration to handle timing issues
            'tf_timeout': 1.0,  # Increase TF lookup timeout
            'allow_time_offset': 0.5,  # Allow small time offsets in TF lookups
            'publish_static_transforms': True,  # Publish static transforms
            'transform_tolerance': 0.5,  # Set transform tolerance to handle timing issues
        }],
        output='screen'
    )
    
    # ==================== IMU-BASED ODOMETRY NODE ====================
    imu_odometry_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,  # Set to True for planar motion (highway scenario)
            'publish_tf': True,
            'odom_frame': LaunchConfiguration('odom_frame_id'),
            'base_link_frame': LaunchConfiguration('vehicle_frame_id'),
            'world_frame': 'odom',  # Use odom as the world frame for local mapping
            'imu0': '/imu/data',  # Topic published by imu_euler_visualizer_simple
            'imu0_config': [False, False, False,  # x, y, z position
                            False, False, True,     # roll, pitch, yaw
                            False, False, False,  # x, y, z velocity
                            False, False, True,     # roll, pitch, yaw rates
                            False, False, True],    # x, y, z accelerations
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_queue_size': 10,
            'imu0_remove_gravitational_acceleration': True,
            # Additional parameters for local mapping
            'print_diagnostics': True,
            'publish_acceleration': True,
            'reset_on_time_jump': True,
            'smooth_lagged_data': True,
            'transform_time_offset': 0.1,  # ADDED: Add time offset for transforms
            'transform_timeout': 0.5,  # ADDED: Increase transform timeout
            'transform_tolerance': 1.0,  # ADDED: Increase transform tolerance
            'reset_on_time_jump': False,  # ADDED: Don't reset on time jumps
            'predict_to_current_time': True,  # ADDED: Predict to current time
            'history_length': 10.0,  # ADDED: Increase history length
        }]
    )
    
    # ==================== SENSOR NODES ====================
    # LiDAR processing node - Handles point cloud processing and obstacle detection
    lidar_listener_node = Node(
        package='sensor_fusion_2',
        executable='lidar_listener_clusters_3',
        name='lidar_cube_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'filter_vehicle_points': True,  # Remove ego-vehicle points from scan
            'vehicle_length': LaunchConfiguration('vehicle_length'),          
            'vehicle_width': LaunchConfiguration('vehicle_width'),           
            'vehicle_height': LaunchConfiguration('vehicle_height'),
            'enable_motion_compensation': True,
            'use_imu_data': True,  # Use IMU data for motion compensation
            'frame_id': 'local_map_link',  # FIXED: Explicitly set frame_id to local_map_link
            'parent_frame_id': 'imu_link',  # Parent frame is now IMU link
            'imu_frame_id': 'imu_link',  # FIXED: Explicitly set IMU frame ID
            'imu_topic': '/imu/data',  # Topic published by imu_euler_visualizer
            'use_tf_transform': False,  # Don't rely on TF transforms for visualization
            'publish_grid_map': True,
            'map_topic': '/lidar/map',
            'map_frame_id': 'local_map_link',  # FIXED: Explicitly set map_frame_id to local_map_link
            'output_frame': 'local_map_link',  # FIXED: Explicitly set output_frame to local_map_link
            'point_size': 0.2,  # Updated to match radar point size
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
            'use_imu_orientation': True,  # Explicitly use IMU orientation data
            'apply_imu_transform': True,  # Apply IMU transform to LiDAR data
            'follow_imu_rotation': True,  # Ensure LiDAR follows IMU rotation
            'sync_with_imu': True,  # Synchronize with IMU data
            'use_local_coordinates': True,  # Ensure using local coordinates
            'reference_frame': 'local_map_link',  # FIXED: Explicitly set reference_frame to local_map_link
            'center_on_vehicle': False,  # Don't keep map centered on vehicle
            'track_vehicle_position': False,  # Don't track vehicle position for a fixed map
            'update_map_position': False,  # Don't update map position as vehicle moves
            'vehicle_frame_id': 'base_link',  # FIXED: Explicitly set vehicle frame ID
            'transform_points_to_map': True,  # Transform points from vehicle frame to fixed map frame
            'use_fixed_map': True,  # Use fixed map configuration
            'publish_in_map_frame': True,  # Publish points in map frame
            'points_frame_id': 'local_map_link',  # FIXED: Explicitly set points_frame_id to local_map_link
            'disable_tf_lookup': False,  # Enable TF lookup to ensure proper rotation
            'radar_points_topic': '/radar/points',  # Add reference to radar points topic for sensor fusion
            'enable_sensor_fusion': True,  # Enable fusion with radar data
            'tf_buffer_duration': 10.0,  # ADDED: Increase TF buffer duration to handle timing issues
            'tf_timeout': 1.0,  # ADDED: Increase TF lookup timeout
            'allow_time_offset': 0.5,  # ADDED: Allow small time offsets in TF lookups
            'transform_tolerance': 1.0,  # ADDED: Increase transform tolerance
        }],
        output='screen'
    )
    
    # Radar processing node - Handles velocity detection and dynamic object tracking
    radar_listener_node = Node(
        package='sensor_fusion_2',
        executable='radar_listener_clusters',
        name='radar_listener_clusters',
        parameters=[{
            'tcp_ip': LaunchConfiguration('radar_tcp_ip'),
            'tcp_port': LaunchConfiguration('radar_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'grid_resolution': 0.2,  # Match LiDAR resolution
            'grid_width': LaunchConfiguration('map_width_meters'),
            'grid_height': LaunchConfiguration('map_height_meters'),
            'show_velocity_vectors': True,  # Visualize object motion
            'radar_to_map_fusion': True,  # Enable radar-map fusion
            'marker_lifetime': LaunchConfiguration('marker_lifetime'),
            'point_size': 0.2,
            'use_advanced_coloring': True,
            'velocity_arrow_scale': 1.0,
            'min_velocity_for_display': 0.0,  # Set to 0.0 to display all objects regardless of velocity
            'max_velocity_for_display': 30.0,
            'frame_id': 'local_map_link',  # FIXED: Explicitly set frame_id to local_map_link
            'parent_frame_id': 'imu_link',  # Parent frame is now IMU link
            'map_frame_id': 'local_map_link',  # FIXED: Explicitly set map_frame_id to local_map_link
            'output_frame': 'local_map_link',  # FIXED: Explicitly set output_frame to local_map_link
            'use_tf_transform': False,  # Don't rely on TF transforms for visualization
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
            'use_imu_orientation': True,  # Explicitly use IMU orientation data
            'apply_imu_transform': True,  # Apply IMU transform to radar data
            'follow_imu_rotation': True,  # Ensure radar follows IMU rotation
            'sync_with_imu': True,  # Synchronize with IMU data
            'imu_topic': '/imu/data',  # Topic published by imu_euler_visualizer
            'imu_frame_id': 'imu_link',  # ADDED: Explicitly set IMU frame ID
            'use_local_coordinates': True,  # Ensure using local coordinates
            'reference_frame': 'local_map_link',  # FIXED: Explicitly set reference_frame to local_map_link
            'center_on_vehicle': False,  # Don't keep map centered on vehicle
            'track_vehicle_position': False,  # Don't track vehicle position for a fixed map
            'update_map_position': False,  # Don't update map position as vehicle moves
            'vehicle_frame_id': 'base_link',  # FIXED: Explicitly set vehicle frame ID
            'transform_points_to_map': True,  # Transform points from vehicle frame to fixed map frame
            'use_fixed_map': True,  # Use fixed map configuration
            'publish_in_map_frame': True,  # Publish points in map frame
            'points_frame_id': 'local_map_link',  # FIXED: Enable TF lookup to ensure proper rotation
            'disable_tf_lookup': False,  # FIXED: Enable TF lookup to ensure proper rotation
            'target_frame': 'lidar_link',  # Add reference to lidar frame for sensor fusion
            'enable_sensor_fusion': True,  # Enable fusion with lidar data
            'tf_buffer_duration': 10.0,  # ADDED: Increase TF buffer duration to handle timing issues
            'tf_timeout': 1.0,  # ADDED: Increase TF lookup timeout
            'allow_time_offset': 0.5,  # ADDED: Allow small time offsets in TF lookups
            'transform_tolerance': 1.0,  # ADDED: Increase transform tolerance
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
            'map_frame': LaunchConfiguration('local_map_frame_id'),  # Use fixed local map frame
            'vehicle_frame': LaunchConfiguration('vehicle_frame_id'),  # Reference vehicle frame
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
            'occupied_value': 100,  # Force black for occupied cells
            'free_value': 0,        # Force white for free cells
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
            'enable_map_saving': False,  # Changed from LaunchConfiguration('enable_map_saving') to False
            'save_directory': LaunchConfiguration('save_directory'),
            'save_interval': LaunchConfiguration('save_interval'),
            'save_binary_map': False,  # Changed from LaunchConfiguration('save_binary_map') to False
            'save_combined_map': False,  # Changed from LaunchConfiguration('save_combined_map') to False
            'save_layer_maps': False,  # Already False
            'save_as_black_white': True,  # Save as black and white
            'save_as_pgm': True,  # Save as PGM format
            'save_as_yaml': True,  # Save YAML metadata
            'enforce_binary_values': True,  # Ensure only binary values (0 or 100) are used
            'binary_map_name_prefix': 'semantic_binary_bw_',  # Distinctive name prefix with black/white indicator
            'combined_map_name_prefix': 'semantic_combined_bw_',  # Distinctive name prefix with black/white indicator
            
            # Add new parameters for low car detection
            'enhance_low_cars': LaunchConfiguration('enhance_low_cars'),
            'low_car_height_threshold': LaunchConfiguration('low_car_height_threshold'),
            'car_detection_width': LaunchConfiguration('car_detection_width'),
            
            # Add new parameters for object expansion control
            'car_expansion_radius': LaunchConfiguration('car_expansion_radius'),
            'obstacle_expansion_radius': LaunchConfiguration('obstacle_expansion_radius'),
            'dynamic_expansion_radius': LaunchConfiguration('dynamic_expansion_radius'),
            
            # Additional parameters for binary output
            'map_color_scheme': 'binary',  # Use binary color scheme (white=free, black=occupied)
            'convert_to_black_white': True,  # Convert map to black and white
            
            # Fixed-map configuration parameters
            'use_fixed_map': True,  # Use fixed map instead of vehicle-centered map
            'use_vehicle_frame': False,  # Don't use vehicle frame as reference for the map
            'track_vehicle_position': False,  # Don't track vehicle position to maintain map position
            'update_map_position': False,  # Don't update map position as vehicle moves
            'reference_frame': LaunchConfiguration('local_map_frame_id'),  # Reference fixed map frame
            'transform_sensor_data': True,  # Transform sensor data from vehicle frame to fixed map frame
        }],
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
            'map_frame_id': LaunchConfiguration('local_map_frame_id'),  # Explicitly use local map frame
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame_id'),  # Reference vehicle frame
            'waypoints_topic': '/carla/waypoints',
            'marker_topic': '/carla/waypoint_markers',
            'waypoint_marker_size': LaunchConfiguration('waypoint_marker_size'),
            'waypoint_line_width': LaunchConfiguration('waypoint_line_width'),
            'waypoint_lifetime': LaunchConfiguration('waypoint_lifetime'),
            'reconnect_interval': LaunchConfiguration('waypoint_reconnect_interval'),
            'connection_timeout': LaunchConfiguration('waypoint_connection_timeout'),
            'auto_reconnect': True,
            'socket_buffer_size': 262144,  # Increased buffer size for better data handling
            'socket_timeout': 1.0,  # Increased timeout for more reliable connections
            'enable_tcp_nodelay': True,
            'enable_socket_keepalive': True,
            'verbose_logging': True,
            'use_local_coordinates': True,  # Explicitly set to use local coordinates
            'fixed_origin': True,  # Keep origin fixed relative to vehicle
            'persistent_markers': LaunchConfiguration('persistent_markers'),
            'clear_markers_on_update': False,  # Don't clear previous markers when new ones arrive
            'use_persistent_durability': True,  # Use persistent durability for markers
            'debug_mode': True,  # Enable debug mode for more detailed logging
            'publish_even_without_data': True,  # Publish empty maps even if no waypoint data is received
            'center_on_vehicle': False,  # Don't keep waypoints centered on vehicle
            'track_vehicle_position': False,  # Don't track vehicle position for a fixed map
            'reference_frame': LaunchConfiguration('local_map_frame_id'),  # Reference fixed map frame
            'transform_waypoints_to_map_frame': True,  # Transform waypoints to fixed map frame
            'maintain_waypoint_history': True,  # Keep history of waypoints
            'use_fixed_map': True,  # Use fixed map configuration
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
            'map_frame_id': LaunchConfiguration('local_map_frame_id'),  # Explicitly use local map frame
            'publish_rate': 20.0,  # Increased publish rate for more frequent updates
            'waypoint_marker_topic': '/carla/waypoint_markers',
            'binary_topic': LaunchConfiguration('waypoint_binary_topic'),
            'binary_updates_topic': '/waypoint_map/binary_updates',  # Use explicit string instead of concatenation
            'occupied_value': 100,  # Force black for occupied cells
            'free_value': 0,        # Force white for free cells
            'waypoint_width': 3.0,  # Significantly increased width for better visibility
            'use_vehicle_frame': False,  # Don't center on vehicle since we use a fixed map
            'map_origin_x': LaunchConfiguration('map_origin_x'),
            'map_origin_y': LaunchConfiguration('map_origin_y'),
            'verbose_logging': True,  # Enable verbose logging
            'publish_empty_map': True,  # Publish an empty map even if no waypoints are available
            'use_transient_local_durability': True,  # Use transient local durability for better reliability
            'always_republish': True,  # Always republish the map even if there are no changes
            'save_waypoint_map': False,  # Changed from True to False - disable individual waypoint map saving
            'save_as_black_white': True,  # Save as black and white
            'use_persistent_topics': True,  # Use persistent topics for better reliability
            'draw_thick_lines': True,  # Draw thicker lines for better visibility
            'connect_waypoints': True,  # Connect waypoints with lines
            'fill_enclosed_areas': False,  # Don't fill enclosed areas
            'clear_between_updates': False,  # Don't clear between updates to maintain waypoints
            'save_directory': LaunchConfiguration('save_directory'),  # Use launch argument for save directory
            'save_interval': LaunchConfiguration('save_interval'),  # Use launch argument for save interval
            'map_name_prefix': 'waypoints_only_bw_',  # Distinctive name prefix with black/white indicator
            'line_thickness': 3,  # Thicker lines for better visibility
            'waypoint_persistence': 10.0,  # Keep waypoints visible longer
            'force_publish': True,  # Force publish even if no waypoints are available
            'debug_mode': True,  # Enable debug mode for more detailed logging
            'initial_map_publish': True,  # Publish an initial empty map on startup
            'publish_frequency': 1.0,  # Publish at least once per second
            'map_color_scheme': 'binary',  # Use binary color scheme (white=free, black=occupied)
            'save_as_pgm': True,  # Save as PGM format
            'save_as_yaml': True,  # Save YAML metadata
            'enforce_binary_values': True,  # Ensure only binary values (0 or 100) are used
            'binary_threshold': 50,  # Threshold for binary conversion
            'use_local_coordinates': True,  # Explicitly use local coordinates
            'reference_frame': LaunchConfiguration('local_map_frame_id'),  # Reference fixed map frame
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame_id'),  # Reference vehicle frame
            'center_on_vehicle': False,  # Don't keep map centered on vehicle
            'track_vehicle_position': False,  # Don't track vehicle position for a fixed map
            'update_map_position': False,  # Don't update map position as vehicle moves
            'transform_waypoints': True,  # Transform waypoints from vehicle frame to fixed map frame
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
            'map_frame_id': LaunchConfiguration('local_map_frame_id'),  # Explicitly use local map frame
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame_id'),  # Reference vehicle frame
            'publish_rate': 20.0,  # Increased from default for more frequent updates
            'semantic_binary_topic': LaunchConfiguration('binary_topic'),  # Explicitly set topic name
            'waypoint_binary_topic': LaunchConfiguration('waypoint_binary_topic'),  # Explicitly set topic name
            'combined_binary_topic': LaunchConfiguration('combined_binary_topic'),  # Explicitly set topic name
            'combined_binary_updates_topic': '/combined_binary_map_updates',  # Use explicit string instead of concatenation
            'occupied_value': 100,  # Force black for occupied cells
            'free_value': 0,        # Force white for free cells
            'prioritize_waypoints': True,  # Waypoints take priority over semantic data
            'use_transient_local_durability': True,  # Ensure consistent QoS settings
            'verbose_logging': True,  # Add verbose logging for debugging
            'save_combined_map': LaunchConfiguration('enable_map_saving'),  # Use the global enable_map_saving parameter
            'save_as_black_white': True,  # Save as black and white
            'ensure_waypoints_visible': True,  # Make sure waypoints are visible on the map
            'waypoint_overlay_value': 100,  # Use maximum value (black) for waypoints to ensure visibility
            'republish_on_waypoint_update': True,  # Republish when waypoints are updated
            'waypoint_thickness': 5,  # Increase waypoint thickness for better visibility
            'force_waypoint_overlay': True,  # Force waypoints to be overlaid on the map
            'invert_map_colors': False,  # Don't invert colors (white=free, black=occupied)
            'save_directory': LaunchConfiguration('save_directory'),  # Use launch argument for save directory
            'save_interval': LaunchConfiguration('save_interval'),  # Use launch argument for save interval
            'map_name_prefix': 'unified_map_',  # Changed from 'combined_with_waypoints_bw_' to 'unified_map_'
            'waypoint_persistence': 10.0,  # Keep waypoints visible longer
            'always_include_waypoints': True,  # Always include waypoints in the combined map
            'clear_between_updates': False,  # Don't clear between updates to maintain waypoints
            'force_publish': True,  # Force publish even if no input maps are available
            'debug_mode': True,  # Enable debug mode for more detailed logging
            'publish_empty_map': True,  # Publish an empty map if no input maps are available
            'initial_map_publish': True,  # Publish an initial empty map on startup
            'publish_frequency': 1.0,  # Publish at least once per second
            'map_color_scheme': 'binary',  # Use binary color scheme (white=free, black=occupied)
            'monitor_topics': True,  # Monitor input topics for changes
            'overlay_mode': 'waypoints_on_top',  # Always put waypoints on top
            'enhance_waypoints': True,  # Make waypoints more visible
            'waypoint_dilation': 2,  # Dilate waypoints for better visibility
            'publish_to_semantic_combined': True,  # Also publish to semantic combined topic
            'semantic_combined_topic': '/semantic_costmap/combined',  # Topic to publish combined data to
            'save_as_pgm': True,  # Save as PGM format
            'save_as_yaml': True,  # Save YAML metadata
            'binary_threshold': 50,  # Threshold for binary conversion
            'enforce_binary_values': True,  # Ensure only binary values (0 or 100) are used
            'convert_to_black_white': True,  # Convert map to black and white
            
            # Enhanced unified map parameters
            'include_all_data_sources': True,  # Include all available data sources 
            'include_lidar_data': True,       # Explicitly include LiDAR data
            'include_radar_data': True,       # Explicitly include radar data
            'include_waypoints': True,        # Explicitly include waypoints
            'unified_map_topic': LaunchConfiguration('unified_map_topic'),
            'publish_unified_map': True,      # Publish the unified map
            'include_all_objects': True,      # Include all detected objects
            'enhance_map_with_all_sensors': True, # Use data from all sensors
            'use_high_quality_mode': True,    # Use high quality mode for better map
            'add_timestamp_to_filename': LaunchConfiguration('add_timestamp_to_filename'),
            
            # Additional unified map enhancement parameters
            'unified_map_resolution': LaunchConfiguration('map_resolution'),
            'unified_map_quality': LaunchConfiguration('unified_map_quality'),
            'unified_map_format': LaunchConfiguration('unified_map_format'),
            'enhanced_radar_integration': LaunchConfiguration('enhanced_radar_integration'),
            'enhanced_lidar_integration': LaunchConfiguration('enhanced_lidar_integration'),
            'radar_confidence_threshold': 0.1, # Lower threshold to include more radar detections
            'lidar_confidence_threshold': 0.1, # Lower threshold to include more LiDAR detections
            'combine_close_detections': True,  # Combine nearby detections for cleaner map
            'filter_noise': True,              # Filter noise for cleaner map
            'preserve_thin_obstacles': True,   # Preserve thin obstacles that might be filtered otherwise
            'preserve_distant_obstacles': True, # Make sure distant obstacles are included
            'force_save_on_shutdown': LaunchConfiguration('force_save_on_shutdown'),
            'max_map_age_before_save': 60.0,   # Save map at least every 60 seconds
            'extra_safety_expansion': 1.0,     # Expand obstacles slightly for safety
            'add_map_metadata': True,          # Include metadata in saved map
            'metadata_format': 'yaml',         # Format for metadata
            'use_local_coordinates': True,     # Explicitly use local coordinates
            'reference_frame': LaunchConfiguration('vehicle_frame_id'),  # Reference vehicle frame
            
            # Fixed-map configuration parameters
            'center_on_vehicle': False,  # Don't keep map centered on vehicle
            'use_vehicle_frame': False,  # Don't use vehicle frame as reference
            'track_vehicle_position': False,  # Don't track vehicle position for a fixed map
            'update_map_position': False,  # Don't update map position as vehicle moves
            'use_fixed_map': True,  # Use fixed map configuration
            'transform_data': True,  # Transform data to fixed map frame
        }],
        output='screen'
    )
    
    # ==================== PATH PLANNING NODES ====================
    # Hybrid A* Planner - Advanced path planning with non-holonomic constraints
    hybrid_astar_planner_node = Node(
        package='sensor_fusion_2',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'grid_size': 0.5,
            'wheelbase': 2.5,
            'obstacle_threshold': 65,  # Increased from 60 to 65 to be less sensitive to obstacles
            'publish_rate': 5.0,
            'map_topic': '/unified_map',  # Using unified map for better integration
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame_id'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'max_iterations': 500,
            'motion_resolution': 30,
            'angle_resolution': 36,
            'heuristic_weight': 1.0,
            'replan_on_move': True,
            'position_change_threshold': 0.25,
            'orientation_change_threshold': 0.15,
            'replan_on_map_change': True,
            'treat_unknown_as_obstacle': False,  # Don't treat unknown areas as obstacles
            'unknown_cost_multiplier': 1.2,  # Reduced from 1.5 to 1.2 for less penalty
            'publish_debug_viz': True,
            'min_explored_percentage': 40.0,  # Reduced from 50.0 to 40.0 to allow more paths
            
            # Enhanced vehicle parameters
            'vehicle_length': LaunchConfiguration('vehicle_length'),
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'safety_margin': 0.6,  # Reduced from 0.8 to 0.6 for less conservative collision checking
            
            # Enhanced obstacle handling
            'obstacle_inflation_radius': 1.2,  # Reduced from 1.5 to 1.2 for less inflation
            'use_distance_transform': True,
            'collision_check_resolution': 6,  # Reduced from 8 to 6 for faster collision checking
            
            # Cost gradient parameters for smoother paths
            'distance_cost_weight': 60.0,  # Reduced from 75.0 to 60.0 for less obstacle avoidance cost
            'max_distance_cost': 100.0,  # Reduced from 1500.0 to 1200.0
            'min_safe_distance': 2.0,  # Reduced from 2.5 to 2.0 for less conservative planning
            
            # Fixed start/goal parameters
            'use_fixed_start': True,
            'start_x': 0.0,
            'start_y': 0.0,
            'start_yaw': 0.0,
            'goal_x': 30.0,
            'goal_y': 30.0,
            'goal_yaw': 0.0,
            'use_rviz_goal': True,
            'goal_topic': '/goal_pose',
            
            # Waypoint handling parameters
            'waypoint_topic': '/carla/waypoints',
            'respect_waypoints': True,
            'waypoint_safety_distance': 1.0,  # Reduced from 1.5 to 1.0 for less conservative waypoint avoidance
            'waypoint_cost_multiplier': 1.8,  # Reduced from 2.5 to 1.8 for less penalty
            'follow_waypoints': False  # Don't try to follow waypoints (just avoid them)
        }],
        output='screen'
    )
    
    # Frenet Path Smoother - Smooths the planned path for better vehicle control
    frenet_path_smoother_node = Node(
        package='sensor_fusion_2',
        executable='frenet_path_smoother',
        name='frenet_path_smoother',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'num_points': 200,  # Increased for smoother path
            'vehicle_frame_id': LaunchConfiguration('vehicle_frame_id'),
            'map_frame_id': LaunchConfiguration('map_frame_id'),
            'input_path_topic': '/hybrid_astar_path',
            'output_path_topic': '/smooth_path'
        }],
        output='screen'
    )
    
    # RViz Node - Configured for local mapping visualization
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
    
    # ==================== LAUNCH DESCRIPTION ====================
    """
    Return the complete launch description containing all nodes, parameters, and configurations.
    
    The launch sequence is organized in logical groups:
    1. Core system parameters (time source, visualization)
    2. Network configuration (TCP connections for sensors)
    3. Frame structure (TF tree configuration)
    4. Map parameters (resolution, dimensions, processing)
    5. TF tree nodes (coordinate frame relationships)
    6. Sensor processing nodes (LiDAR, radar, IMU)
    7. Mapping and visualization nodes
    
    A 2-second delay is added before starting the nodes to ensure
    the TF tree is properly established first.
    """
    return LaunchDescription([
        # ===== CORE SYSTEM PARAMETERS =====
        # Launch Arguments - Core system configuration
        declare_use_sim_time,                                             # Controls time source (sim vs. real)
        declare_show_rviz,                                                # Toggles visualization
        
        # ===== NETWORK CONFIGURATION =====
        # Network Configuration - Sensor data streams
        declare_lidar_tcp_ip,                                             # LiDAR server connection parameters
        declare_lidar_tcp_port,
        declare_radar_tcp_ip,                                             # Radar server connection parameters
        declare_radar_tcp_port,
        declare_radar_reconnect_interval,                                 # Connection recovery parameters
        declare_radar_connection_timeout,
        
        # ===== WAYPOINT SYSTEM =====
        # Waypoint System Configuration
        declare_waypoint_tcp_ip,                                          # Waypoint server connection parameters
        declare_waypoint_tcp_port,
        declare_waypoint_reconnect_interval,
        declare_waypoint_connection_timeout,
        
        # ===== FRAME CONFIGURATION =====
        # Frame Configuration - TF tree structure
        declare_vehicle_frame_id,                                         # Vehicle reference frame
        declare_map_frame_id,                                             # Global map reference frame
        declare_odom_frame_id,                                            # Odometry reference frame
        declare_local_map_frame_id,                                       # Local perception map frame
        
        # ===== MAP PARAMETERS =====
        # Map Configuration - Spatial parameters
        declare_map_resolution,                                           # Grid cell size (meters/cell)
        declare_map_width,                                                # Map width (meters)
        declare_map_height,                                               # Map height (meters)
        declare_map_origin_x,                                             # Map origin position
        declare_map_origin_y,
        declare_publish_rate,                                             # Update frequency (Hz)
        
        # ===== WAYPOINT VISUALIZATION =====
        # Waypoint Visualization
        declare_waypoint_marker_size,                                     # Marker appearance parameters
        declare_waypoint_line_width,
        declare_waypoint_lifetime,
        declare_waypoint_width,
        declare_waypoint_binary_topic,                                    # Communication topics
        declare_combined_binary_topic,
        
        # ===== COORDINATE SYSTEM =====
        # Coordinate System Configuration
        declare_use_local_coordinates,                                    # Local vs. global coordinates
        declare_fixed_origin,                                             # Fixed reference point
        declare_persistent_markers,                                       # Visualization persistence
        
        # ===== PROCESSING PARAMETERS =====
        # Costmap Processing Parameters
        declare_temporal_filtering,                                       # Temporal smoothing
        declare_motion_prediction,                                        # Motion prediction for dynamics
        declare_ground_height_threshold,                                  # Ground classification threshold
        declare_vegetation_height_ratio,                                  # Vegetation detection parameter
        declare_building_width_threshold,                                 # Building classification threshold
        declare_dynamic_velocity_threshold,                               # Dynamic object detection threshold
        
        # ===== LAYER WEIGHTS =====
        # Layer Weights and Visualization
        declare_ground_weight,                                            # Ground layer influence
        declare_obstacle_weight,                                          # Obstacle importance
        declare_vegetation_weight,                                        # Vegetation detection weight
        declare_building_weight,                                          # Building detection weight
        declare_dynamic_weight,                                           # Dynamic object weight
        declare_enable_3d_visualization,                                  # 3D visualization toggle
        declare_enable_text_labels,                                       # Label visibility toggle
        
        # ===== VEHICLE PARAMETERS =====
        # Vehicle Parameters
        declare_vehicle_length,                                           # Vehicle dimensions (meters)
        declare_vehicle_width,
        declare_vehicle_height,
        declare_enable_tuning,                                            # Parameter tuning interface toggle
        
        # ===== TIMING PARAMETERS =====
        # Time-based Parameters
        declare_decay_time,                                               # Cell decay time (seconds)
        declare_dynamic_decay_time,                                       # Dynamic object decay time
        declare_cell_memory,                                              # Cell persistence time
        declare_max_tracking_age,                                         # Object tracking time
        declare_marker_lifetime,                                          # Visualization marker lifetime
        
        # ===== BINARY MAP CONFIGURATION =====
        # Binary Map Configuration
        declare_binary_threshold,                                         # Classification threshold
        declare_enable_binary_output,                                     # Binary output toggle
        declare_binary_topic,                                             # Output topic name
        
        # ===== MAP SAVING CONFIGURATION =====
        # Map Saving Configuration
        declare_enable_map_saving,                                        # Map saving toggle
        declare_save_directory,                                           # Save directory path
        declare_save_interval,                                            # Save frequency (seconds)
        declare_save_binary_map,                                          # Binary map saving toggle
        declare_save_combined_map,                                        # Combined map saving toggle
        declare_save_layer_maps,                                          # Layer map saving toggle
        declare_occupied_value,                                           # Occupied cell value
        declare_free_value,                                               # Free cell value
        declare_map_format,                                               # Output format specification
        declare_publish_binary_map,                                       # Binary map publishing toggle
        
        # ===== ENHANCED DETECTION =====
        # Enhanced Detection Parameters
        declare_include_all_objects,                                      # All objects inclusion toggle
        declare_enhanced_binary_map,                                      # Enhanced map toggle
        declare_all_objects_binary_topic,                                 # All objects topic name
        declare_low_car_height_threshold,                                 # Low vehicle detection threshold
        declare_car_detection_width,                                      # Car width threshold
        declare_enhance_low_cars,                                         # Low car enhancement toggle
        declare_car_expansion_radius,                                     # Car expansion radius (cells)
        declare_obstacle_expansion_radius,                                # Obstacle expansion radius (cells)
        declare_dynamic_expansion_radius,                                 # Dynamic object expansion radius (cells)
        
        # ===== IMU CONFIGURATION =====
        # IMU Configuration
        declare_imu_tcp_ip,                                               # IMU connection parameters
        declare_imu_tcp_port,
        declare_imu_reconnect_interval,                                   # Connection recovery parameters
        declare_imu_connection_timeout,
        declare_imu_frame_id,                                             # IMU frame identifier
        
        # ===== UNIFIED MAP CONFIGURATION =====
        # Unified Map Configuration
        declare_unified_map_topic,                                        # Unified map topic name
        declare_unified_map_quality,                                      # Map quality setting (low/medium/high)
        declare_unified_map_format,                                       # Map file format
        declare_enhanced_radar_integration,                               # Radar integration toggle
        declare_enhanced_lidar_integration,                               # LiDAR integration toggle
        declare_add_timestamp_to_filename,                                # Filename timestamp toggle
        declare_force_save_on_shutdown,                                   # Shutdown save toggle
        
        # ===== TF TREE NODES =====
        # TF Tree Nodes - Transform hierarchy
        map_to_odom_node,                                                 # Root transform (map→odom)
        odom_to_base_link_node,                                           # Vehicle odometry (odom→base_link)
        base_to_imu_node,                                                 # IMU mounting (base_link→imu_link)
        imu_to_lidar_node,                                                # LiDAR mounting (imu_link→lidar_link)
        map_to_radar_node,                                                # Radar position (map→radar_link)
        map_to_local_map_node,                                            # Local map connection (map→local_map_link)
        local_map_to_waypoint_node,                                       # Waypoint frame (local_map_link→waypoint_frame)
        
        # ===== NODE INITIALIZATION WITH DELAY =====
        # Timing Control - Ensure proper initialization
        TimerAction(
            period=1.0,                                                   # 2-second initialization delay
            actions=[
                # Core System Components
                imu_odometry_node,                                        # Vehicle motion tracking (EKF)
                
                # Sensor Processing Pipeline
                lidar_listener_node,                                      # LiDAR point cloud processing
                radar_listener_node,                                      # Radar velocity detection
                semantic_costmap_node,                                    # Environment semantic mapping
                
                # Navigation and Visualization
                waypoint_listener_node,                                   # Path planning data receiver
                waypoint_map_generator_node,                              # Route visualization generator
                binary_map_combiner_node,                                 # Unified map generation
                hybrid_astar_planner_node,                                # Hybrid A* path planning
                frenet_path_smoother_node,                                # Path smoothing for vehicle control
                
                # User Interface
                rviz_node,                                                # 3D visualization interface
                rqt_reconfigure_node,                                     # Dynamic parameter tuning
                imu_listener_node                                         # IMU data processing and monitoring
            ]
        )
    ]) 