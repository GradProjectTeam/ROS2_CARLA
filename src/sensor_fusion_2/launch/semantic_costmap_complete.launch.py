#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'semantic_costmap.rviz')
    
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
        default_value='30.0',  # Updated from 20.0 to 30.0
        description='Rate at which to publish costmap layers (Hz)'
    )
    
    declare_temporal_filtering = DeclareLaunchArgument(
        'temporal_filtering',
        default_value='true',  # Unchanged
        description='Enable temporal filtering of costmap layers'
    )
    
    declare_motion_prediction = DeclareLaunchArgument(
        'motion_prediction',
        default_value='true',  # Unchanged
        description='Enable motion prediction for dynamic objects'
    )
    
    # Classification parameters
    declare_ground_height_threshold = DeclareLaunchArgument(
        'ground_height_threshold',
        default_value='0.3',  # Unchanged
        description='Maximum height for ground classification (meters)'
    )
    
    declare_vegetation_height_ratio = DeclareLaunchArgument(
        'vegetation_height_ratio',
        default_value='3.0',  # Unchanged
        description='Height to width ratio for vegetation classification'
    )
    
    declare_building_width_threshold = DeclareLaunchArgument(
        'building_width_threshold',
        default_value='5.0',  # Unchanged
        description='Minimum width for building classification (meters)'
    )
    
    declare_dynamic_velocity_threshold = DeclareLaunchArgument(
        'dynamic_velocity_threshold',
        default_value='1.5',  # Unchanged
        description='Minimum velocity for dynamic classification (m/s)'
    )
    
    # Layer weight parameters - Updated based on test2.yaml
    declare_ground_weight = DeclareLaunchArgument(
        'ground_weight',
        default_value='0.0',  # Keep at 0.0
        description='Weight of ground layer in combined map'
    )
    
    declare_obstacle_weight = DeclareLaunchArgument(
        'obstacle_weight',
        default_value='8.0',  # Adjusted from 10.0 to 8.0
        description='Weight of obstacle layer in combined map'
    )
    
    declare_vegetation_weight = DeclareLaunchArgument(
        'vegetation_weight',
        default_value='8.0',  # Adjusted from 10.0 to 8.0
        description='Weight of vegetation layer in combined map'
    )
    
    declare_building_weight = DeclareLaunchArgument(
        'building_weight',
        default_value='8.0',  # Adjusted from 10.0 to 8.0
        description='Weight of building layer in combined map'
    )
    
    declare_dynamic_weight = DeclareLaunchArgument(
        'dynamic_weight',
        default_value='10.0',  # Keep at 10.0 to emphasize dynamic objects
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
            'use_imu_data': False,  # We're not using IMU data in this setup
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'publish_grid_map': True,
            'map_topic': '/lidar/map',
            'point_size': 2.0,  # Added from test1_lidar.yaml
            'cube_alpha': 0.8,  # Added from test1_lidar.yaml
            'use_cluster_stats': True,  # Added from test1_lidar.yaml
            'use_convex_hull': True,  # Added from test1_lidar.yaml
            'use_point_markers': True,  # Added from test1_lidar.yaml
            'vehicle_safety_margin': 0.5,  # Added from test1_lidar.yaml
            'vehicle_visualization': True,  # Added from test1_lidar.yaml
            'vehicle_x_offset': 0.0,  # Added from test1_lidar.yaml
            'vehicle_y_offset': 0.0,  # Added from test1_lidar.yaml
            'vehicle_z_offset': -1.0,  # Added from test1_lidar.yaml
            'center_size': 3.0,  # Added from test1_lidar.yaml
            'lidar_lower_fov': -5.0,  # Added from test1_lidar.yaml
            'lidar_upper_fov': 10.0,  # Added from test1_lidar.yaml
            'lidar_pitch_angle': 0.0,  # Added from test1_lidar.yaml
            'min_point_distance': 0.0,  # Added from test1_lidar.yaml
            'max_negative_z': -100.0,  # Added from test1_lidar.yaml
            'verbose_logging': False,  # Added from test1_lidar.yaml
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
            'marker_lifetime': 0.5,  # Unchanged
            'point_size': 0.2,
            'use_advanced_coloring': True,
            'velocity_arrow_scale': 1.0,
            'min_velocity_for_display': 0.1,
            'max_velocity_for_display': 30.0,
            'frame_id': 'radar_link',  # Unchanged
            'map_frame_id': 'map',  # Unchanged
            'points_topic': '/radar/points',  # Unchanged
            'clusters_topic': '/radar/clusters',
            'velocity_vectors_topic': '/radar/velocity_vectors',
            'moving_objects_topic': '/radar/moving_objects',  # Unchanged
            'static_objects_topic': '/radar/static_objects',  # Unchanged
            'object_tracking_topic': '/radar/object_tracking',  # Unchanged
            'min_points_per_cluster': 1,  # Keep at 1 to detect sparse clusters
            'cluster_distance_threshold': 1.0,  # Increased from 0.0 to 1.0 to better group points
            'static_velocity_threshold': 0.2,  # Increased from 0.0 to 0.2 to better identify stationary objects
            'moving_velocity_threshold': 0.5,  # Increased from 0.0 to 0.5 to better identify moving objects
            'use_dbscan_clustering': True,  # Unchanged
            'dbscan_epsilon': 1.0,  # Increased from 0.7 to 1.0 to create larger clusters
            'dbscan_min_samples': 2,  # Reduced from 3 to 2 to detect smaller clusters
            'track_objects': True,  # Unchanged
            'max_tracking_age': 2.0,  # Unchanged
            'min_track_confidence': 0.6,  # Unchanged
            'verbose_logging': True,  # Changed from False to True for better debugging
            'publish_rate': 60.0,  # Added from test1_radar.yaml
            'moving_object_color_r': 1.0,  # Added from test1_radar.yaml
            'moving_object_color_g': 0.0,  # Added from test1_radar.yaml
            'moving_object_color_b': 0.0,  # Added from test1_radar.yaml
            'moving_object_color_a': 0.8,  # Added from test1_radar.yaml
            'static_object_color_r': 0.0,  # Added from test1_radar.yaml
            'static_object_color_g': 0.0,  # Added from test1_radar.yaml
            'static_object_color_b': 1.0,  # Added from test1_radar.yaml
            'static_object_color_a': 0.8,  # Added from test1_radar.yaml
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
            'min_points_per_cluster': 1,  # Keep at 1 to detect sparse clusters
            'cluster_distance_threshold': 1.0,  # Increased from 0.0 to 1.0 to better group points
            'static_velocity_threshold': 0.2,  # Increased from 0.0 to 0.2
            'moving_velocity_threshold': 0.5,  # Increased from 0.0 to 0.5
            'use_dbscan_clustering': True,  # Unchanged
            'dbscan_epsilon': 1.0,  # Increased from 0.7 to 1.0
            'dbscan_min_samples': 2,  # Reduced from 3 to 2
            'track_objects': True,  # Unchanged
            'max_tracking_age': 2.0,  # Unchanged
            'min_track_confidence': 0.6,  # Unchanged
            'verbose_logging': True,  # Changed from False to True
            'publish_rate': 60.0,  # Added from test1_radar.yaml
            'moving_object_color_r': 1.0,  # Added from test1_radar.yaml
            'moving_object_color_g': 0.0,  # Added from test1_radar.yaml
            'moving_object_color_b': 0.0,  # Added from test1_radar.yaml
            'moving_object_color_a': 0.8,  # Added from test1_radar.yaml
            'static_object_color_r': 0.0,  # Added from test1_radar.yaml
            'static_object_color_g': 0.0,  # Added from test1_radar.yaml
            'static_object_color_b': 1.0,  # Added from test1_radar.yaml
            'static_object_color_a': 0.8,  # Added from test1_radar.yaml
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
            'publish_rate': 20.0,  # From test2_radar.yaml
            'use_velocity_filter': True,
            'use_temporal_filtering': True,
            'min_velocity': 0.0,  # From test2_radar.yaml
            'cell_memory': 1.0,
            'obstacle_threshold': 0.0,  # From test2_radar.yaml
            'free_threshold': -0.2,
            'start_type_description_service': True,
            'enable_fusion_layer': True,  # Added from test2_radar.yaml
            'obstacle_value': 100,  # Added from test2_radar.yaml
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
            'decay_time': 0.5,  # From test1.yaml
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
            'start_type_description_service': True,  # Added from test1.yaml
        }],
        output='screen'
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
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
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        declare_enable_tuning,
        
        # TF Tree Nodes
        world_to_map_node,
        map_to_base_link_node,
        base_to_lidar_node,
        base_to_radar_node,
        
        # Sensor Nodes
        lidar_listener_node,
        radar_listener_node,
        radar_object_detector_node,
        lidar_realtime_mapper_node,
        radar_map_generator_node,
        
        # Visualization Nodes
        semantic_costmap_node,
        rviz_node,
        rqt_reconfigure_node
    ]) 