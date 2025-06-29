#!/usr/bin/env python3
"""
Semantic Costmap Launch File - Fixed Map with Vehicle-Oriented Sensors

This launch file has been configured to create a fixed map while having sensors that rotate with the vehicle:
- The map stays fixed at a single position (fixed to the odom frame)
- LiDAR and radar sensors rotate with the vehicle as it turns
- Sensor data is transformed into the fixed map frame
- This provides a consistent global representation while maintaining accurate sensor orientation

FIXED MAP WITH ROTATING SENSORS BENEFITS:
- Map maintains a consistent position in space, not moving with the vehicle
- Obstacles stay in the same location on the map as they're detected
- Sensors maintain proper orientation relative to the vehicle
- Allows for building a persistent map while accurately capturing vehicle heading
- Ideal for navigation in environments where position consistency matters

FRAME STRUCTURE:
- odom: Root frame providing odometry reference
  ├── base_link: Vehicle center frame (rotates as vehicle turns)
  │   └── imu_link: Inertial measurement unit frame
  │       ├── lidar_link: LiDAR sensor frame
  │       └── radar_link: Radar sensor frame
  └── local_map_link: Fixed local perception map frame (attached to odom)
      └── waypoint_frame: Frame for waypoint visualization

MAP CONFIGURATION:
- The map is generated in the local_map_link frame which is fixed to the odom frame
- Map origin remains stationary regardless of vehicle movement
- Sensors rotate with the vehicle but data is transformed into the fixed map frame
- This provides the best of both worlds: fixed map and properly oriented sensors

SENSOR INTEGRATION:
- All sensors (LiDAR, radar, IMU) rotate with the vehicle
- Sensor data is properly transformed into the fixed map frame
- Vehicle orientation is captured accurately for perception

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

UNIFIED MAP CONFIGURATION:
- This launch file has been modified to save only ONE combined map instead of multiple separate maps
- The binary_map_combiner node is now the only node saving maps to disk
- The unified map combines LiDAR data, radar data, and waypoints into a single comprehensive map
- Individual map saving is disabled in the semantic_costmap_visualizer and waypoint_map_generator nodes

UNIFIED MAP FEATURES:
- High-quality integration of data from all available sensors (LiDAR, radar)
- Complete inclusion of all detected objects, regardless of classification
- Seamless integration of waypoints for path planning and navigation
- Preservation of thin obstacles that might be filtered otherwise
- Expanded obstacles for safe navigation
- Map saves automatically at intervals specified by save_interval (default: 5.0 seconds)
- Map also saves on system shutdown to prevent data loss
- PGM format (image) with YAML metadata file for navigation stack compatibility
- Unified map available on ROS topic: /unified_map
- Enhanced parameters for fine-tuning the unified map's appearance and behavior

USING THE UNIFIED MAP:
- The unified map is the only map saved to disk, in the directory: /home/mostafa/GP/ROS2/maps/NewMaps
- Each map is saved with a timestamp for uniqueness: unified_map_YYYYMMDD_HHMMSS.pgm
- Maps can be loaded directly into ROS2 navigation2 stack using map_server
- Control map saving with the enable_map_saving parameter (default: true)
- Control map quality with unified_map_quality parameter (options: low, medium, high)
- The directory is automatically created if it doesn't exist

UPDATED TF TREE:
- Simplified TF tree structure with odom as the root frame
- IMU-based odometry for accurate vehicle positioning
- Three main sensors: LiDAR, radar, and IMU
- Local map frame for perception and planning
- Waypoint frame for path visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
import time  # Import time module for timestamp generation

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'semantic_waypoints.rviz')
    
    # Define default save directory and ensure it exists
    default_save_dir = '/home/mostafa/GP/ROS2/maps/NewMaps'
    os.makedirs(default_save_dir, exist_ok=True)
    
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
    
    # Add odom frame parameter
    declare_odom_frame_id = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Frame ID for the odometry frame'
    )
    
    # Add local map frame parameter
    declare_local_map_frame_id = DeclareLaunchArgument(
        'local_map_frame_id',
        default_value='local_map_link',
        description='Frame ID for the local map'
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
    # Root transform: map to odom
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_odom',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Base odometry transform (will be updated dynamically by odometry)
    odom_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
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
    
    # IMU to LiDAR transform
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=[
            '1.5',  # X offset - LiDAR is 1.5m forward from IMU
            '0.0',  # Y offset - centered on vehicle
            '0.4',  # Z offset - LiDAR is 0.4m above the IMU (1.9m - 1.5m)
            '0',    # Roll - no roll (0 degrees)
            '3.14159',    # Pitch - no pitch (0 degrees)
            '0',    # Yaw - 0 degrees (LiDAR looking forward, same as vehicle)
            'imu_link', 
            'lidar_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Map to Radar transform - Fixed radar position in the map frame
    map_to_radar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_radar',
        arguments=[
            '0.0',  # X offset - Radar is fixed at the map origin
            '0.0',  # Y offset - centered at map origin
            '1.9',  # Z offset - Radar is 1.9m above the ground
            '3.14159',    # Roll - no roll (0 degrees)
            '0',    # Pitch - no pitch (0 degrees)
            '0',    # Yaw - 180 degrees (π radians) - Radar rotated 180 degrees around Z-axis
            'map', 
            'radar_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Map to local_map_link transform - This connects the map frame to our local map frame
    map_to_local_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_local_map',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'local_map_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Local map to waypoint frame transform
    local_map_to_waypoint_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_local_map_to_waypoint',
        arguments=[
            '0.0',  # X offset
            '0.0',  # Y offset
            '0.0',  # Z offset
            '0',    # Roll - no roll (0 degrees)
            '0',    # Pitch - no pitch (0 degrees)
            '0',    # Yaw - no yaw (0 degrees)
            'local_map_link', 
            'waypoint_frame'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
            'points_frame_id': 'local_map_link',  # FIXED: Explicitly set points_frame_id to local_map_link
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
            'waypoint_width': 7.0,  # Significantly increased width for better visibility
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
        # Add new frame IDs
        declare_odom_frame_id,
        declare_local_map_frame_id,
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
        
        # Add unified map parameters
        declare_unified_map_topic,
        declare_unified_map_quality,
        declare_unified_map_format,
        declare_enhanced_radar_integration,
        declare_enhanced_lidar_integration,
        declare_add_timestamp_to_filename,
        declare_force_save_on_shutdown,
        
        # TF Tree Nodes - Launch these first to establish the TF tree
        map_to_odom_node,
        odom_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        map_to_radar_node,
        map_to_local_map_node,
        local_map_to_waypoint_node,
        
        # Add a timing delay to ensure TF tree is established before other nodes start
        TimerAction(
            period=2.0,  # Wait 2 seconds for TF tree to be established
            actions=[
                # IMU-based odometry node
                imu_odometry_node,
                
                # Sensor Nodes
                lidar_listener_node,
                radar_listener_node,
                semantic_costmap_node,
                
                # Visualization and Integration Nodes
                waypoint_listener_node,
                waypoint_map_generator_node,
                binary_map_combiner_node,
                
                rviz_node,
                rqt_reconfigure_node,
                imu_listener_node
            ]
        )
    ]) 