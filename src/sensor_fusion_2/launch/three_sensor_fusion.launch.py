#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

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
    
    # IMU processing parameters
    declare_imu_filter_window = DeclareLaunchArgument(
        'imu_filter_window',
        default_value='5',
        description='Window size for IMU moving average filter'
    )
    
    declare_imu_enable_bias_correction = DeclareLaunchArgument(
        'imu_enable_bias_correction',
        default_value='true',
        description='Enable IMU gyroscope bias correction'
    )
    
    declare_imu_enable_complementary_filter = DeclareLaunchArgument(
        'imu_enable_complementary_filter',
        default_value='true',
        description='Enable IMU complementary filter for sensor fusion'
    )
    
    declare_imu_publish_rate = DeclareLaunchArgument(
        'imu_publish_rate',
        default_value='100.0',
        description='Rate at which to publish IMU data (Hz)'
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
    
    declare_lidar_yaw_offset = DeclareLaunchArgument(
        'lidar_yaw_offset',
        default_value='0.0',  # Default to forward direction
        description='Yaw rotation of LiDAR in radians'
    )
    
    # Radar positioning parameters
    declare_radar_yaw_offset = DeclareLaunchArgument(
        'radar_yaw_offset',
        default_value='0.0',  # ~96 degrees in radians - to align with observed LiDAR direction
        description='Yaw rotation of radar in radians (to align with LiDAR)'
    )
    
    # Fusion parameters
    declare_lidar_weight = DeclareLaunchArgument(
        'lidar_weight',
        default_value='0.6',
        description='Weight for LiDAR data in fusion'
    )
    
    declare_radar_weight = DeclareLaunchArgument(
        'radar_weight',
        default_value='0.3',
        description='Weight for radar data in fusion'
    )
    
    declare_imu_weight = DeclareLaunchArgument(
        'imu_weight',
        default_value='0.1',
        description='Weight for IMU data in fusion'
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Base to LiDAR transform (directly connected to base_link)
    base_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_lidar',
        arguments=[
            LaunchConfiguration('lidar_x_offset'),
            LaunchConfiguration('lidar_y_offset'),
            LaunchConfiguration('lidar_z_offset'),
            '0',    # Roll - aligned with vehicle
            '0',    # Pitch - aligned with vehicle
            LaunchConfiguration('lidar_yaw_offset'),    # Yaw - adjustable to set LiDAR orientation
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
            LaunchConfiguration('radar_yaw_offset'),    # Yaw - adjusted to align with LiDAR view
            'base_link', 
            'radar_link'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Add comment about transform chain
    # Note: The transform chain is:
    # world -> map -> base_link -> [imu_link, lidar_link, radar_link]
    # All transforms are static relative to their parent frames
    
    # ==================== SENSOR NODES ====================
    # IMU Node
    imu_euler_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='imu_euler_visualizer_simple',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'reconnect_interval': 1.0,
            'frame_id': 'imu_link',
            'world_frame_id': 'world',
            'publish_rate': LaunchConfiguration('imu_publish_rate'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'socket_buffer_size': 65536,
            'yaw_offset': 0.0,  
            'vehicle_forward_axis': 'x',  # Vehicle forward axis
            # Note: This node now expects compass values in degrees from TCP connection
            # and handles the conversion to radians internally for ROS messages
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'filter_vehicle_points': True,
            'vehicle_length': LaunchConfiguration('vehicle_length'),          
            'vehicle_width': LaunchConfiguration('vehicle_width'),           
            'vehicle_height': LaunchConfiguration('vehicle_height'),
            'enable_motion_compensation': True,
            'use_imu_data': True,  # Uses IMU data where compass is in degrees (via TF transforms)
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'publish_grid_map': True,
            'map_topic': '/lidar/map',
        }],
        output='screen'
    )
    
    # LiDAR Costmap Creator Node
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
            'map_topic': '/lidar/map',
            'map_frame': 'map',
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
            'moving_object_color_r': 1.0,
            'moving_object_color_g': 0.0,
            'moving_object_color_b': 0.0,
            'moving_object_color_a': 0.8,
            'static_object_color_r': 0.0,
            'static_object_color_g': 0.0,
            'static_object_color_b': 1.0,
            'static_object_color_a': 0.8,
            'verbose_logging': True,
            'socket_buffer_size': 262144,
            'max_recent_measurements': 30,
            'enable_async_reception': True,
            'reconnect_on_error': True,
            'max_tcp_points': 2000,
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
            'map_resolution': 0.2,
            'map_width': 60.0,
            'map_height': 60.0,
            'map_frame': 'map',
            'radar_topic': '/radar/points',
            'map_topic': '/radar/map',
            'costmap_topic': '/radar/costmap',
            'publish_rate': 10.0,
            'use_velocity_filter': True,
            'use_temporal_filtering': True,
            'min_velocity': 0.2,
            'max_velocity': 40.0,
            'cell_memory': 2.0,
            'obstacle_threshold': 0.2,
            'free_threshold': -0.2,
            'use_reliable_qos': True,
            'use_transient_local_durability': True,
            'verbose_logging': True,
        }],
        output='screen'
    )
    
    # Three Sensor Fusion Node
    three_sensor_fusion_node = Node(
        package='sensor_fusion_2',
        executable='three_sensor_fusion',
        name='three_sensor_fusion',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_resolution': 0.2,
            'map_width': 60.0,
            'map_height': 60.0,
            'map_frame': 'map',
            'lidar_map_topic': '/lidar/map',
            'radar_map_topic': '/radar/map',
            'radar_points_topic': '/radar/points',
            'imu_topic': '/imu/data',  # This topic now receives IMU data with compass in degrees (converted to rad)
            'fused_map_topic': '/three_sensor_fused_map',
            'publish_rate': 5.0,
            'lidar_weight': LaunchConfiguration('lidar_weight'),
            'radar_weight': LaunchConfiguration('radar_weight'),
            'imu_weight': LaunchConfiguration('imu_weight'),
            'obstacle_threshold': 50,
            'max_timestamp_diff': 0.5,
            'use_adaptive_weighting': True,
            'enable_debug_output': True,
            'unknown_cell_value': 50,
            'min_confidence_threshold': 0.3,
            'verbose_logging': True,
            'log_topic_info': True,
            'use_imu_for_orientation': True,  # Uses quaternion created from compass in degrees
            'orientation_correction': True,   # Orientation correction now handles compass in degrees
            'dynamic_obstacle_tracking': True,
            'radar_data_timeout': 2.0,
            'check_radar_points': True,
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
            'moving_object_color_r': 1.0,
            'moving_object_color_g': 0.0,
            'moving_object_color_b': 0.0,
            'moving_object_color_a': 0.8,
            'static_object_color_r': 0.0,
            'static_object_color_g': 0.0,
            'static_object_color_b': 1.0,
            'static_object_color_a': 0.8,
            'verbose_logging': True,
            'marker_lifetime': 0.5,
            'publish_rate': 10.0
        }]
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
        declare_imu_tcp_ip,
        declare_imu_tcp_port,
        declare_imu_filter_window,
        declare_imu_enable_bias_correction,
        declare_imu_enable_complementary_filter,
        declare_imu_publish_rate,
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        declare_lidar_x_offset,
        declare_lidar_y_offset,
        declare_lidar_z_offset,
        declare_lidar_yaw_offset,
        declare_radar_yaw_offset,
        declare_lidar_weight,
        declare_radar_weight,
        declare_imu_weight,
        
        # TF Tree Nodes
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        base_to_lidar_node,
        base_to_radar_node,
        
        # Sensor Nodes
        imu_euler_visualizer_node,
        lidar_listener_node,
        lidar_costmap_node,
        radar_listener_node,
        radar_map_node,
        radar_object_detector_node,
        
        # Fusion Node
        three_sensor_fusion_node,
        
        # Visualization
        rviz_node
    ])