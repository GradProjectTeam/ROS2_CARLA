#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'three_sensors_fusion.rviz')
    
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
    
    # Lidar-specific arguments
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
    
    # Radar-specific arguments
    declare_radar_tcp_ip = DeclareLaunchArgument(
        'radar_tcp_ip',
        default_value='127.0.0.1',
        description='IP address for radar TCP connection'
    )
    
    declare_radar_tcp_port = DeclareLaunchArgument(
        'radar_tcp_port',
        default_value='12348',
        description='Port for radar TCP connection'
    )
    
    # IMU-specific arguments
    declare_imu_tcp_ip = DeclareLaunchArgument(
        'imu_tcp_ip',
        default_value='127.0.0.1',
        description='IP address for IMU TCP connection'
    )
    
    declare_imu_tcp_port = DeclareLaunchArgument(
        'imu_tcp_port',
        default_value='12345',
        description='Port for IMU TCP connection'
    )
    
    # Vehicle parameters
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
    
    declare_vehicle_x_offset = DeclareLaunchArgument(
        'vehicle_x_offset',
        default_value='0.0',
        description='X offset of the vehicle center relative to the lidar'
    )
    
    declare_vehicle_y_offset = DeclareLaunchArgument(
        'vehicle_y_offset',
        default_value='0.0',
        description='Y offset of the vehicle center relative to the lidar'
    )
    
    declare_vehicle_z_offset = DeclareLaunchArgument(
        'vehicle_z_offset',
        default_value='-1.0',
        description='Z offset of the vehicle center relative to the lidar'
    )
    
    # Sensor positioning parameters
    declare_lidar_x_offset = DeclareLaunchArgument(
        'lidar_x_offset',
        default_value='0.0',
        description='X offset of the lidar relative to the IMU'
    )
    
    declare_lidar_y_offset = DeclareLaunchArgument(
        'lidar_y_offset',
        default_value='0.0',
        description='Y offset of the lidar relative to the IMU'
    )
    
    declare_lidar_z_offset = DeclareLaunchArgument(
        'lidar_z_offset',
        default_value='1.9',
        description='Z offset of the lidar relative to the IMU'
    )
    
    declare_lidar_roll = DeclareLaunchArgument(
        'lidar_roll',
        default_value='0.0',
        description='Roll angle of the lidar relative to the IMU'
    )
    
    declare_lidar_pitch = DeclareLaunchArgument(
        'lidar_pitch',
        default_value='0.0',
        description='Pitch angle of the lidar relative to the IMU'
    )
    
    declare_lidar_yaw = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='Yaw angle of the lidar relative to the IMU'
    )
    
    # Visualization parameters
    declare_point_size = DeclareLaunchArgument(
        'point_size',
        default_value='0.05',
        description='Size of point cloud points in visualization'
    )
    
    declare_center_size = DeclareLaunchArgument(
        'center_size',
        default_value='0.2',
        description='Size of cluster center points in visualization'
    )
    
    # ==================== TF TREE CONFIGURATION ====================
    # Root transform: world to map
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # Map to base_link transform
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # Base to IMU transform
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '0',    # X offset
            '0',    # Y offset
            '0.1',  # Z offset - IMU is 10cm above the base
            '0',    # Roll
            '0',    # Pitch
            '0',    # Yaw
            'base_link', 
            'imu_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # IMU to LiDAR transform
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=[
            LaunchConfiguration('lidar_x_offset'),
            LaunchConfiguration('lidar_y_offset'),
            LaunchConfiguration('lidar_z_offset'),
            LaunchConfiguration('lidar_roll'),
            LaunchConfiguration('lidar_pitch'),
            LaunchConfiguration('lidar_yaw'),
            'imu_link', 
            'lidar_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # Base to Radar transform
    base_to_radar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_radar',
        arguments=[
            '1.5',  # X offset (forward from base)
            '0.0',  # Y offset (left/right)
            '1.0',  # Z offset (up from base) - radar is 1m above the base
            '0',    # Roll
            '0',    # Pitch
            '0',    # Yaw
            'base_link', 
            'radar_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # ==================== SENSOR NODES ====================
    # IMU Node
    imu_euler_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'reconnect_interval': 1.0,
            'frame_id': 'imu_link',
            'world_frame_id': 'world',
            'filter_window_size': 2,
            'queue_size': 20,
            'publish_rate': 200.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'socket_buffer_size': 65536,
            'enable_bias_correction': True,
            'enable_complementary_filter': True,
            'zero_velocity_threshold': 0.02,
        }],
        output='screen'
    )
    
    # LiDAR Node
    lidar_listener_node = Node(
        package='sensor_fusion_2',
        executable='lidar_listener_clusters_3',
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
            'cube_alpha': 0.6,
            
            # C++ Integration parameters
            'socket_buffer_size': 524288,
            'enable_data_validation': True,
            'reconnect_on_error': True,
            'reconnect_cooldown': 0.5,
            'packet_timeout_ms': 120,
            'enable_async_reception': True,
            'max_packet_size': 131072,
            'enable_zero_copy_reception': True,
            'apply_cpp_timestamp_correction': True,
            
            # Point cloud processing parameters
            'sor_std_dev_multiplier': 4.2,
            'sor_neighbor_radius': 0.7,
            'sor_min_neighbors': 12,
            'voxel_leaf_size': 0.15,
            'ransac_iterations': 200,
            'ransac_distance_threshold': 0.22,
            'max_cluster_distance': 0.4,
            'min_points_per_cluster': 8,
            
            # Vehicle filtering parameters
            'filter_vehicle_points': True,
            'vehicle_length': LaunchConfiguration('vehicle_length'),          
            'vehicle_width': LaunchConfiguration('vehicle_width'),           
            'vehicle_height': LaunchConfiguration('vehicle_height'),          
            'vehicle_x_offset': LaunchConfiguration('vehicle_x_offset'),
            'vehicle_y_offset': LaunchConfiguration('vehicle_y_offset'),
            'vehicle_z_offset': LaunchConfiguration('vehicle_z_offset'),
            'vehicle_safety_margin': 0.9,
            'vehicle_visualization': True,
            
            # Motion compensation
            'enable_motion_compensation': True,
            'use_imu_data': True,
            'point_time_compensation': True,
            'motion_prediction_time': 0.18,
            
            # LiDAR specific parameters
            'lidar_upper_fov': 0.0,
            'lidar_lower_fov': -15.0,
            'lidar_pitch_angle': 5.0,
            'min_point_distance': 2.2,
            'max_negative_z': -0.7,
            
            # Output configuration
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'queue_size': 15,
            'publish_rate': 50.0,
            
            # Object boxing parameters
            'enable_object_boxes': True,
            'box_padding': 0.2,
            'min_box_points': 8,
            'use_oriented_boxes': True,
            'publish_box_markers': True,
            'box_lifetime': 0.3,
            'box_color_r': 0.0,
            'box_color_g': 1.0,
            'box_color_b': 0.0,
            'box_color_a': 0.8,
            
            # Output map configuration
            'publish_grid_map': True,
            'map_topic': '/lidar/map',
            'map_resolution': 0.2,
            'map_width': 60.0,
            'map_height': 60.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_frame': 'map',
            'map_qos_reliable': True,
            'map_qos_transient_local': True,
        }],
        output='screen'
    )
    
    # LiDAR Costmap Creator Node (using lidar_realtime_mapper)
    lidar_costmap_node = Node(
        package='sensor_fusion_2',
        executable='lidar_realtime_mapper',
        name='lidar_realtime_mapper',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': 0.2,
            'map_width_meters': 60.0,
            'map_height_meters': 60.0,
            'map_origin_x': -30.0,
            'map_origin_y': -30.0,
            'publish_rate': 5.0,
            'process_rate': 10.0,
            'vehicle_x_offset': LaunchConfiguration('vehicle_x_offset'),
            'vehicle_y_offset': LaunchConfiguration('vehicle_y_offset'),
            'free_space_value': 0,
            'obstacle_value': 100,
            'unknown_value': 50,
            # Ensure it publishes to the correct topic for fusion
            'map_topic': '/realtime_map',
            # Radar integration parameters
            'integrate_radar_data': True,
            'radar_map_topic': '/radar/map',
            'radar_weight': 0.5,  # Equal weight to radar data
            # Use reliable QoS settings for better map delivery
            'use_reliable_qos': True,
            'use_transient_local_durability': True,
            # Frame consistency
            'map_frame': 'map',
            # Improve detection parameters
            'hit_weight': 0.95,
            'count_threshold': 0.8,  # Lower threshold to detect obstacles faster
            'decay_rate': 0.1,  # Slower decay for more stable detection
        }],
        output='screen'
    )
    
    # Radar Listener Node
    radar_listener_node = Node(
        package='sensor_fusion_2',
        executable='radar_listener_clusters',
        name='radar_listener_clusters',
        parameters=[{
            'tcp_ip': LaunchConfiguration('radar_tcp_ip'),
            'tcp_port': LaunchConfiguration('radar_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # Enhanced settings for C++ integration
            'socket_buffer_size': 262144,          # 256KB buffer for C++ data
            'marker_lifetime': 0.3,                # Longer lifetime for better visualization
            'grid_resolution': 0.2,                # Match lidar grid resolution
            'grid_width': 60.0,                    # Wider grid for better fusion
            'grid_height': 60.0,                   # Taller grid for better fusion
            'velocity_threshold': 0.5,             # Minimum velocity to consider moving
            'use_advanced_coloring': True,         # Use velocity-based coloring
            'show_velocity_vectors': True,         # Show velocity vectors
            'max_recent_measurements': 30,         # Track more measurements
            'radar_to_map_fusion': True,           # Enable fusion with map
            
            # C++ data processing settings
            'max_buffer_size': 2097152,            # 2MB buffer size limit
            'enable_data_validation': True,
            'reconnect_on_error': True,
            'reconnect_cooldown': 0.5,
            'packet_timeout_ms': 120,
            'enable_async_reception': True,
            'max_tcp_points': 2000,                # Store more points from C++ processor
            
            # Radar specific parameters matching C++ code
            'radar_horizontal_fov': 60.0,          # degrees
            'radar_vertical_fov': 20.0,            # degrees
            'radar_range': 100.0,                  # meters
        }],
        output='screen'
    )
    
    # Radar Map Generator Node
    radar_map_node = Node(
        package='sensor_fusion_2',
        executable='radar_map_generator',
        name='radar_map_generator',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': 0.2,                 # Match lidar resolution
            'map_width': 60.0,                     # Match radar grid width
            'map_height': 60.0,                    # Match radar grid height
            'map_frame': 'map',                    # Use the map frame for fusion
            'radar_topic': '/radar/points',        # Input radar points topic
            'map_topic': '/radar/map',             # Output map topic
            'realtime_map_topic': '/realtime_map', # Also publish to realtime map
            'publish_to_realtime_map': False,      # Disable direct publishing to realtime map (use integration instead)
            'publish_rate': 10.0,                  # Increased publish rate (Hz)
            'obstacle_threshold': 0.2,             # Lower threshold to detect more obstacles
            'free_threshold': -0.2,                # Threshold for considering a cell free
            'use_velocity_filter': True,           # Filter points by velocity
            'min_velocity': 0.3,                   # Lower minimum velocity to include more objects
            'use_temporal_filtering': True,        # Use temporal filtering
            'cell_memory': 2.0,                    # Increased cell memory in seconds
            'enable_fusion_layer': True,           # Enable radar layer for fusion
            'obstacle_value': 100,                 # Value for obstacles in the map
            # QoS settings for consistent communication
            'use_reliable_qos': True,              # Use reliable QoS
            'use_transient_local_durability': True, # Use transient local durability
        }],
        output='screen'
    )
    
    # Enhanced Radar Costmap Creator Node
    radar_costmap_node = Node(
        package='sensor_fusion_2',
        executable='radar_costmap_creator',
        name='radar_costmap_creator',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': 0.2,                 # Match other maps resolution
            'map_width': 300,                      # 60m with 0.2m resolution
            'map_height': 300,                     # 60m with 0.2m resolution
            'map_origin_x': -30.0,                 # Center map
            'map_origin_y': -30.0,                 # Center map
            'map_frame': 'map',                    # Use the map frame
            'publish_rate': 10.0,                  # Hz
            'velocity_impact_factor': 30.0,        # Higher cost for faster objects
            'max_object_age': 2.0,                 # Keep tracking objects for 2 seconds
            'radar_weight': 0.7,                   # Weight for radar data
            'prediction_time': 1.5,                # Predict 1.5 seconds into future
            'min_velocity_threshold': 0.3,         # Min velocity to consider moving (m/s)
            'publish_to_realtime_map': True,       # Publish to realtime map
            'realtime_map_topic': '/realtime_map', # Realtime map topic
            'use_transient_local_durability': True, # Use transient local durability
            'object_tracking_distance': 1.0,       # Max distance for object association
            'enable_visualization': True,          # Enable visualization markers
        }],
        output='screen'
    )
    
    # Map Fusion Node
    map_fusion_node = Node(
        package='sensor_fusion_2',
        executable='fusion_costmap_creator',
        name='map_fusion_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': 0.2,
            'map_width': 60.0,
            'map_height': 60.0,
            'map_frame': 'map',
            'lidar_map_topic': '/lidar/map',
            'radar_map_topic': '/radar/map',
            'fused_map_topic': '/fused_map',
            'publish_rate': 2.0,
            'lidar_weight': 0.7,
            'radar_weight': 0.3,
            'obstacle_threshold': 50,
            'max_timestamp_diff': 0.5,
            'use_adaptive_weighting': True,
            'enable_debug_output': True,
            'unknown_cell_value': 50,
            'min_confidence_threshold': 0.3,
            # Add detailed logging to help diagnose issues
            'verbose_logging': True,
            'log_topic_info': True,
        }],
        output='screen'
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
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
        declare_imu_tcp_ip,
        declare_imu_tcp_port,
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        declare_vehicle_x_offset,
        declare_vehicle_y_offset,
        declare_vehicle_z_offset,
        declare_lidar_x_offset,
        declare_lidar_y_offset,
        declare_lidar_z_offset,
        declare_lidar_roll,
        declare_lidar_pitch,
        declare_lidar_yaw,
        declare_point_size,
        declare_center_size,
        
        # TF Tree Nodes
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        base_to_radar_node,
        
        # Sensor Nodes
        imu_euler_visualizer_node,
        lidar_listener_node,
        lidar_costmap_node,
        radar_listener_node,
        radar_map_node,
        radar_costmap_node,
        map_fusion_node,
        
        # Visualization
        rviz_node
    ]) 