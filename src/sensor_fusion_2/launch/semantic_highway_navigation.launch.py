#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package share directories
    sensor_fusion_2_dir = get_package_share_directory('sensor_fusion_2')
    sensor_fusion_dir = get_package_share_directory('sensor_fusion')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(sensor_fusion_2_dir, 'rviz', 'semantic_costmap.rviz')
    
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
        default_value='127.0.0.1',
        description='IP address for radar TCP connection'
    )
    
    declare_radar_tcp_port = DeclareLaunchArgument(
        'radar_tcp_port',
        default_value='12348',
        description='Port for radar TCP connection'
    )
    
    declare_imu_tcp_ip = DeclareLaunchArgument(
        'imu_tcp_ip',
        default_value='127.0.0.1',
        description='IP address of the IMU TCP server'
    )
    
    declare_imu_tcp_port = DeclareLaunchArgument(
        'imu_tcp_port',
        default_value='12345',
        description='Port number of the IMU TCP server'
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
    
    # Semantic Costmap parameters
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.5',
        description='Resolution of the costmap in meters per cell'
    )
    
    declare_map_width = DeclareLaunchArgument(
        'map_width_meters',
        default_value='120.0',
        description='Width of the costmap in meters'
    )
    
    declare_map_height = DeclareLaunchArgument(
        'map_height_meters',
        default_value='120.0',
        description='Height of the costmap in meters'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Rate at which to publish costmap layers (Hz)'
    )
    
    declare_temporal_filtering = DeclareLaunchArgument(
        'temporal_filtering',
        default_value='true',
        description='Enable temporal filtering of costmap layers'
    )
    
    declare_motion_prediction = DeclareLaunchArgument(
        'motion_prediction',
        default_value='true',
        description='Enable motion prediction for dynamic objects'
    )
    
    # Classification parameters
    declare_ground_height_threshold = DeclareLaunchArgument(
        'ground_height_threshold',
        default_value='0.3',
        description='Maximum height for ground classification (meters)'
    )
    
    declare_vegetation_height_ratio = DeclareLaunchArgument(
        'vegetation_height_ratio',
        default_value='3.0',
        description='Height to width ratio for vegetation classification'
    )
    
    declare_building_width_threshold = DeclareLaunchArgument(
        'building_width_threshold',
        default_value='5.0',
        description='Minimum width for building classification (meters)'
    )
    
    declare_dynamic_velocity_threshold = DeclareLaunchArgument(
        'dynamic_velocity_threshold',
        default_value='1.5',
        description='Minimum velocity for dynamic classification (m/s)'
    )
    
    # Layer weight parameters
    declare_ground_weight = DeclareLaunchArgument(
        'ground_weight',
        default_value='0.0',
        description='Weight of ground layer in combined map'
    )
    
    declare_obstacle_weight = DeclareLaunchArgument(
        'obstacle_weight',
        default_value='8.0',
        description='Weight of obstacle layer in combined map'
    )
    
    declare_vegetation_weight = DeclareLaunchArgument(
        'vegetation_weight',
        default_value='8.0',
        description='Weight of vegetation layer in combined map'
    )
    
    declare_building_weight = DeclareLaunchArgument(
        'building_weight',
        default_value='8.0',
        description='Weight of building layer in combined map'
    )
    
    declare_dynamic_weight = DeclareLaunchArgument(
        'dynamic_weight',
        default_value='10.0',
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
    
    # Highway specific parameters
    declare_max_speed = DeclareLaunchArgument(
        'max_speed',
        default_value='20.0',
        description='Maximum speed for highway driving (m/s)'
    )
    
    # Obstacle threshold parameter
    declare_obstacle_threshold = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='70',
        description='Threshold for marking cells as obstacles (0-100)'
    )
    
    # Add rqt_reconfigure node for parameter tuning - enabled by default
    declare_enable_tuning = DeclareLaunchArgument(
        'enable_tuning',
        default_value='true',
        description='Enable rqt_reconfigure for parameter tuning'
    )
    
    # ==================== DWA PLANNER PARAMETERS ====================
    # DWA parameters from autonomous_navigation.launch.py
    declare_robot_radius = DeclareLaunchArgument(
        'robot_radius',
        default_value='1.0',
        description='Robot radius for collision detection'
    )
    
    declare_max_linear_velocity = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='5.0',
        description='Maximum linear velocity'
    )
    
    declare_min_linear_velocity = DeclareLaunchArgument(
        'min_linear_velocity',
        default_value='0.0',
        description='Minimum linear velocity'
    )
    
    declare_max_angular_velocity = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='0.5',
        description='Maximum angular velocity'
    )
    
    declare_min_angular_velocity = DeclareLaunchArgument(
        'min_angular_velocity',
        default_value='-0.5',
        description='Minimum angular velocity'
    )
    
    declare_max_linear_accel = DeclareLaunchArgument(
        'max_linear_accel',
        default_value='1.0',
        description='Maximum linear acceleration'
    )
    
    declare_max_angular_accel = DeclareLaunchArgument(
        'max_angular_accel',
        default_value='0.5',
        description='Maximum angular acceleration'
    )
    
    # Cost function weights for DWA
    declare_dwa_obstacle_weight = DeclareLaunchArgument(
        'dwa_obstacle_weight',
        default_value='2.0',
        description='Weight for obstacle avoidance in DWA'
    )
    
    declare_forward_weight = DeclareLaunchArgument(
        'forward_weight',
        default_value='1.0',
        description='Weight for forward motion in DWA'
    )
    
    declare_heading_weight = DeclareLaunchArgument(
        'heading_weight',
        default_value='1.0',
        description='Weight for heading alignment in DWA'
    )
    
    declare_velocity_weight = DeclareLaunchArgument(
        'velocity_weight',
        default_value='0.5',
        description='Weight for velocity in DWA'
    )
    
    # Navigation parameters
    declare_preferred_direction = DeclareLaunchArgument(
        'preferred_direction',
        default_value='0.0',
        description='Preferred direction for navigation in radians'
    )
    
    declare_clearance_threshold = DeclareLaunchArgument(
        'clearance_threshold',
        default_value='2.0',
        description='Minimum clearance from obstacles'
    )
    
    declare_stop_threshold = DeclareLaunchArgument(
        'stop_threshold',
        default_value='0.5',
        description='Distance threshold to stop'
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
    
    # Base to LiDAR transform
    base_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=[
            '0.0',  # X offset
            '0.0',  # Y offset
            '1.9',  # Z offset - LiDAR is 1.9m above the base
            '0',    # Roll - aligned with vehicle
            '0',    # Pitch - aligned with vehicle
            '0',    # Yaw - aligned with vehicle
            'base_link', 
            'lidar_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
            '0',    # Roll - aligned with vehicle
            '0',    # Pitch - aligned with vehicle
            '0',    # Yaw - adjusted to align with LiDAR view
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
            '0',    # X offset (forward/back)
            '0',    # Y offset (left/right)
            '0.1',  # Z offset (up/down) - IMU is 10cm above the base
            '0',    # Roll (rotation around X)
            '0',    # Pitch (rotation around Y)
            '0',    # Yaw (rotation around Z)
            'base_link', 
            'imu_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ==================== SENSOR NODES ====================
    # LiDAR Listener Node
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
            'use_imu_data': False,  # We're not using IMU data in this setup
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'publish_grid_map': True,
            'map_topic': '/lidar/map',
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
            'verbose_logging': False,
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
            'grid_resolution': 0.2,
            'grid_width': LaunchConfiguration('map_width_meters'),
            'grid_height': LaunchConfiguration('map_height_meters'),
            'show_velocity_vectors': True,
            'radar_to_map_fusion': True,
            'marker_lifetime': 0.5,
            'point_size': 0.2,
            'use_advanced_coloring': True,
            'velocity_arrow_scale': 1.0,
            'min_velocity_for_display': 0.1,
            'max_velocity_for_display': 30.0,
            'frame_id': 'radar_link',
            'map_frame_id': 'map',
            'points_topic': '/radar/points',
            'clusters_topic': '/radar/clusters',
            'velocity_vectors_topic': '/radar/velocity_vectors',
            'moving_objects_topic': '/radar/moving_objects',
            'static_objects_topic': '/radar/static_objects',
            'object_tracking_topic': '/radar/object_tracking',
            'min_points_per_cluster': 1,
            'cluster_distance_threshold': 1.0,
            'static_velocity_threshold': 0.2,
            'moving_velocity_threshold': 0.5,
            'use_dbscan_clustering': True,
            'dbscan_epsilon': 1.0,
            'dbscan_min_samples': 2,
            'track_objects': True,
            'max_tracking_age': 2.0,
            'min_track_confidence': 0.6,
            'verbose_logging': True,
            'publish_rate': 60.0,
            'moving_object_color_r': 1.0,
            'moving_object_color_g': 0.0,
            'moving_object_color_b': 0.0,
            'moving_object_color_a': 0.8,
            'static_object_color_r': 0.0,
            'static_object_color_g': 0.0,
            'static_object_color_b': 1.0,
            'static_object_color_a': 0.8,
        }],
        output='screen'
    )
    
    # Radar Object Detector Node
    radar_object_detector_node = Node(
        package='sensor_fusion_2',
        executable='radar_object_detector',
        name='radar_object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'radar_link',
            'map_frame_id': 'map',
            'points_topic': '/radar/points',
            'moving_objects_topic': '/radar/moving_objects',
            'static_objects_topic': '/radar/static_objects',
            'object_tracking_topic': '/radar/object_tracking',
            'min_points_per_cluster': 1,
            'cluster_distance_threshold': 1.0,
            'static_velocity_threshold': 0.2,
            'moving_velocity_threshold': 0.5,
            'use_dbscan_clustering': True,
            'dbscan_epsilon': 1.0,
            'dbscan_min_samples': 2,
            'track_objects': True,
            'max_tracking_age': 2.0,
            'min_track_confidence': 0.6,
            'verbose_logging': True,
            'publish_rate': 60.0,
            'moving_object_color_r': 1.0,
            'moving_object_color_g': 0.0,
            'moving_object_color_b': 0.0,
            'moving_object_color_a': 0.8,
            'static_object_color_r': 0.0,
            'static_object_color_g': 0.0,
            'static_object_color_b': 1.0,
            'static_object_color_a': 0.8,
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
            'map_origin_x': -60.0,  # Half of highway width (120.0/2)
            'map_origin_y': -60.0,  # Half of highway height (120.0/2)
            'publish_rate': 5.0,
            'process_rate': 10.0,
            'map_topic': '/lidar/map',
            'map_frame': LaunchConfiguration('map_frame_id'),
        }],
        output='screen'
    )
    
    # Radar Map Generator Node
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
            'publish_rate': 20.0,
            'use_velocity_filter': True,
            'use_temporal_filtering': True,
            'min_velocity': 0.0,
            'cell_memory': 1.0,
            'obstacle_threshold': 0.0,
            'free_threshold': -0.2,
            'start_type_description_service': True,
            'enable_fusion_layer': True,
            'obstacle_value': 100,
            'publish_to_realtime_map': True,
            'realtime_map_topic': '/realtime_map',
            'use_reliable_qos': True,
            'use_transient_local_durability': True,
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
            'decay_time': 0.5,
            'ground_height_threshold': LaunchConfiguration('ground_height_threshold'),
            'vegetation_height_ratio': LaunchConfiguration('vegetation_height_ratio'),
            'building_width_threshold': LaunchConfiguration('building_width_threshold'),
            'dynamic_velocity_threshold': LaunchConfiguration('dynamic_velocity_threshold'),
            'ground_weight': LaunchConfiguration('ground_weight'),
            'obstacle_weight': LaunchConfiguration('obstacle_weight'),
            'vegetation_weight': LaunchConfiguration('vegetation_weight'),
            'building_weight': LaunchConfiguration('building_weight'),
            'dynamic_weight': LaunchConfiguration('dynamic_weight'),
            'enable_3d_visualization': LaunchConfiguration('enable_3d_visualization'),
            'enable_text_labels': LaunchConfiguration('enable_text_labels'),
            'start_type_description_service': True,
        }],
        output='screen'
    )
    
    # ==================== AUTONOMOUS DWA PLANNER CONFIGURATION ====================
    # Autonomous DWA Planner Node from autonomous_navigation.launch.py
    autonomous_dwa_node = Node(
        package='sensor_fusion',
        executable='autonomous_dwa_node',
        name='autonomous_dwa_planner',
        output='screen',
        parameters=[{
            # Robot parameters
            'robot_radius': LaunchConfiguration('robot_radius'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'min_linear_velocity': LaunchConfiguration('min_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'min_angular_velocity': LaunchConfiguration('min_angular_velocity'),
            'max_linear_accel': LaunchConfiguration('max_linear_accel'),
            'max_angular_accel': LaunchConfiguration('max_angular_accel'),
            'velocity_resolution': 0.1,
            'angular_velocity_resolution': 0.1,
            'prediction_time': 3.0,
            'prediction_steps': 60,
            
            # Cost function weights
            'obstacle_weight': LaunchConfiguration('dwa_obstacle_weight'),
            'forward_weight': LaunchConfiguration('forward_weight'),
            'heading_weight': LaunchConfiguration('heading_weight'),
            'velocity_weight': LaunchConfiguration('velocity_weight'),
            'semantic_weight': 1.5,  # Weight for semantic map influence
            
            # Navigation parameters
            'preferred_direction': LaunchConfiguration('preferred_direction'),
            'clearance_threshold': LaunchConfiguration('clearance_threshold'),
            'stop_threshold': LaunchConfiguration('stop_threshold'),
            
            # Update frequencies
            'control_frequency': 10.0,
            'visualization_frequency': 5.0,
            
            # Frame IDs
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            
            # Map topic - use semantic costmap
            'map_topic': '/semantic_costmap/combined',
            
            # Visualization parameters
            'show_trajectories': True,
            'max_trajectories_shown': 20,  # Increased from 10 to show more trajectories
            
            # IMU integration parameters
            'use_imu_orientation': True,
            'imu_topic': '/imu/data',
            'use_filtered_yaw': True,
            'yaw_filter_size': 5,
            
            # Use simulation time
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Path to Trajectory Converter from autonomous_navigation.launch.py
    path_to_trajectory_converter = Node(
        package='sensor_fusion',
        executable='path_to_trajectory_converter',
        name='path_to_trajectory_converter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_path_topic': '/dwa/best_trajectory',
            'output_trajectory_topic': '/trajectory',
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'vehicle_length': LaunchConfiguration('vehicle_length'),
            'max_speed': LaunchConfiguration('max_linear_velocity'),
            'min_speed': LaunchConfiguration('min_linear_velocity'),
            'publish_rate': 10.0
        }],
    )
    
    # IMU-LiDAR fusion node with enhanced parameters
    imu_lidar_fusion_node = Node(
        package='sensor_fusion',
        executable='imu_lidar_yaw_fusion',
        name='imu_lidar_fusion',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic': '/imu/data',
            'map_topic': '/semantic_costmap/combined',  # Use semantic costmap
            'publish_rate': 30.0,
            'use_filtered_yaw': True,
            'yaw_filter_size': 5,
            'publish_tf': True,
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'gravity_aligned': True,
            'vehicle_forward_axis': 'x'
        }],
    )
    
    # ==================== HIGHWAY NAVIGATION NODES ====================
    # Highway Follower Node - Autonomous highway navigation without goals
    highway_follower_node = Node(
        package='sensor_fusion',
        executable='highway_follower_node',
        name='highway_follower',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'local_path_topic': '/highway_path',
            'cmd_vel_topic': '/cmd_vel',
            'map_topic': '/semantic_costmap/combined',  # Use the semantic costmap instead of realtime_map
            'visualization_topic': '/highway_visualization',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'planning_frequency': 10.0,
            'visualization_frequency': 2.0,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'safe_distance': 5.0,
            'lane_width': 3.5,  # Standard highway lane width
            'lane_detection_range': 30.0,  # How far to look for lane detection
            
            # Highway following parameters
            'forward_distance': 30.0,  # Distance to project forward
            'lateral_weight': 1.5,  # Weight for lateral positioning
            'forward_weight': 1.0,  # Weight for forward movement
            'obstacle_weight': 2.0,  # Weight for obstacle avoidance
            
            # DWA parameters
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': 5.0,   # Minimum speed
            'max_yaw_rate': 0.8,  # Reduced for highway
            'max_accel': 2.0,
            'max_delta_yaw_rate': 0.5,  # Reduced for highway
            'dt': 0.1,
            'predict_time': 5.0,  # Increased for highway
            'speed_cost_gain': 0.1,  # Prefer higher speeds
            'obstacle_cost_gain': 2.0,  # Higher obstacle avoidance
            'lane_following_gain': 1.5,  # Lane following importance
            'lookahead_distance': 20.0,  # Look far ahead
            
            # Explicit heading alignment parameters
            'use_vehicle_heading': True,  # Use the vehicle's heading for planning
            'heading_frame': 'base_link',  # Use car's frame for heading reference
            'align_trajectory_with_heading': True,  # Align trajectories with vehicle heading
            'heading_weight': 2.0,  # Weight for heading alignment in trajectory planning
        }],
        output='screen'
    )
    
    # Path Visualization Node - Visualizes the highway path
    path_visualization_node = Node(
        package='sensor_fusion',
        executable='path_visualization_node',
        name='path_visualization',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_path_topic': '/highway_path',  # Use highway path as global path
            'local_path_topic': '/highway_path',   # Same as global for visualization
            'map_frame_id': 'map',
            'global_path_color_r': 0.0,
            'global_path_color_g': 0.0,
            'global_path_color_b': 1.0,
            'global_path_color_a': 0.8,
            'local_path_color_r': 0.0,
            'local_path_color_g': 1.0,
            'local_path_color_b': 0.0,
            'local_path_color_a': 1.0,
            'path_line_width': 0.1,
            'path_lifetime': 0.0,  # 0 means persistent
        }],
        output='screen'
    )
    
    # Heading Visualization Node - Visualizes the vehicle heading
    heading_visualization_node = Node(
        package='sensor_fusion',
        executable='heading_visualization_node',
        name='heading_visualization',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'heading_topic': '/vehicle/heading',
            'imu_topic': '/imu/data',  # Use IMU data for heading source
            'publish_rate': 10.0,
            'arrow_length': 3.0,  # Length of the visualization arrow
            'arrow_width': 0.5,   # Width of the visualization arrow
            'arrow_color_r': 1.0,
            'arrow_color_g': 0.0,
            'arrow_color_b': 0.0,
            'arrow_color_a': 1.0,
            'visualize_imu_frame': True,  # Also visualize IMU frame
            'visualize_base_frame': True,  # Also visualize base_link frame
            'apply_180_degree_flip': True,  # Flip IMU heading by 180 degrees to match car direction
            'use_first_reading_as_reference': True,  # Use first reading as reference for heading
        }],
        output='screen'
    )
    
    # IMU Euler Visualizer Node - Process IMU data
    imu_euler_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )
    
    # RViz Node with the new configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(sensor_fusion_2_dir, 'rviz', 'semantic_dwa_planning.rviz')],
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
        declare_imu_tcp_ip,
        declare_imu_tcp_port,
        declare_vehicle_frame_id,
        declare_map_frame_id,
        declare_map_resolution,
        declare_map_width,
        declare_map_height,
        declare_publish_rate,
        declare_temporal_filtering,
        declare_motion_prediction,
        # Classification parameters
        declare_ground_height_threshold,
        declare_vegetation_height_ratio,
        declare_building_width_threshold,
        declare_dynamic_velocity_threshold,
        # Layer weights
        declare_ground_weight,
        declare_obstacle_weight,
        declare_vegetation_weight,
        declare_building_weight,
        declare_dynamic_weight,
        declare_enable_3d_visualization,
        declare_enable_text_labels,
        # Vehicle parameters
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        # Highway specific parameters
        declare_max_speed,
        declare_obstacle_threshold,
        declare_enable_tuning,
        
        # DWA Planner parameters
        declare_robot_radius,
        declare_max_linear_velocity,
        declare_min_linear_velocity,
        declare_max_angular_velocity,
        declare_min_angular_velocity,
        declare_max_linear_accel,
        declare_max_angular_accel,
        declare_dwa_obstacle_weight,
        declare_forward_weight,
        declare_heading_weight,
        declare_velocity_weight,
        declare_preferred_direction,
        declare_clearance_threshold,
        declare_stop_threshold,
        
        # TF Tree Nodes
        world_to_map_node,
        map_to_base_link_node,
        base_to_lidar_node,
        base_to_radar_node,
        base_to_imu_node,
        
        # Sensor Nodes
        lidar_listener_node,
        radar_listener_node,
        radar_object_detector_node,
        lidar_realtime_mapper_node,
        radar_map_generator_node,
        semantic_costmap_node,
        
        # IMU Processing
        imu_euler_visualizer_node,
        
        # Autonomous Navigation Nodes
        autonomous_dwa_node,
        path_to_trajectory_converter,
        imu_lidar_fusion_node,
        
        # Highway Navigation Nodes
        highway_follower_node,
        path_visualization_node,
        heading_visualization_node,
        
        # Visualization Nodes
        rviz_node,
        rqt_reconfigure_node
    ]) 