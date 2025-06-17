#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    # Semantic Costmap parameters
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.2',
        description='Resolution of the costmap in meters per cell'
    )
    
    declare_map_width = DeclareLaunchArgument(
        'map_width_meters',
        default_value='60.0',
        description='Width of the costmap in meters'
    )
    
    declare_map_height = DeclareLaunchArgument(
        'map_height_meters',
        default_value='60.0',
        description='Height of the costmap in meters'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
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
            'grid_width': 60.0,
            'grid_height': 60.0,
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
            'min_points_per_cluster': 3,
            'cluster_distance_threshold': 0.8,
            'static_velocity_threshold': 0.5,
            'moving_velocity_threshold': 1.0,
            'use_dbscan_clustering': True,
            'dbscan_epsilon': 0.7,
            'dbscan_min_samples': 3,
            'track_objects': True,
            'max_tracking_age': 2.0,
            'min_track_confidence': 0.6,
            'verbose_logging': False,
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
            'min_points_per_cluster': 3,
            'cluster_distance_threshold': 0.8,
            'static_velocity_threshold': 0.5,
            'moving_velocity_threshold': 1.0,
            'use_dbscan_clustering': True,
            'dbscan_epsilon': 0.7,
            'dbscan_min_samples': 3,
            'track_objects': True,
            'max_tracking_age': 2.0,
            'min_track_confidence': 0.6,
            'verbose_logging': False,
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
            'map_origin_x': '-30.0',  # Half of default map width (60.0/2)
            'map_origin_y': '-30.0',  # Half of default map height (60.0/2)
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
            'publish_rate': 10.0,
            'use_velocity_filter': True,
            'use_temporal_filtering': True,
            'min_velocity': 0.2,
            'max_velocity': 40.0,
            'cell_memory': 2.0,
            'obstacle_threshold': 0.2,
            'free_threshold': -0.2,
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
            'enable_3d_visualization': LaunchConfiguration('enable_3d_visualization'),
            'enable_text_labels': LaunchConfiguration('enable_text_labels'),
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
        declare_enable_3d_visualization,
        declare_enable_text_labels,
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        
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
        rviz_node
    ]) 