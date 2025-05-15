#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Parameters for launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # TCP parameters
    tcp_ip = LaunchConfiguration('tcp_ip', default='127.0.0.1')
    tcp_port = LaunchConfiguration('tcp_port', default='12350')
    point_size = LaunchConfiguration('point_size', default='2.0')
    verbose_logging = LaunchConfiguration('verbose_logging', default='false')
    
    # Filter parameters
    vehicle_length = LaunchConfiguration('vehicle_length', default='4.5')
    vehicle_width = LaunchConfiguration('vehicle_width', default='2.0')
    vehicle_height = LaunchConfiguration('vehicle_height', default='1.8')
    vehicle_x_offset = LaunchConfiguration('vehicle_x_offset', default='-0.75')
    vehicle_y_offset = LaunchConfiguration('vehicle_y_offset', default='0.0')
    vehicle_safety_margin = LaunchConfiguration('vehicle_safety_margin', default='0.2')
    visualize_vehicle = LaunchConfiguration('visualize_vehicle', default='true')
    output_topic = LaunchConfiguration('output_topic', default='/lidar/filtered_points')
    
    # Comparison parameters
    visualize_removed_points = LaunchConfiguration('visualize_removed_points', default='true')
    show_comparison_stats = LaunchConfiguration('show_comparison_stats', default='true')
    use_box_mode = LaunchConfiguration('use_box_mode', default='true')
    filter_height_min = LaunchConfiguration('filter_height_min', default='-1.5')
    filter_height_max = LaunchConfiguration('filter_height_max', default='2.0')
    debug_mode = LaunchConfiguration('debug_mode', default='true')
    comparison_update_rate = LaunchConfiguration('comparison_update_rate', default='0.5')
    
    # Frame parameters for Carla
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='lidar')
    map_frame_id = LaunchConfiguration('map_frame_id', default='map')
    
    # RViz parameters
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'lidar_filter_comparison.rviz')
    
    # Launch arguments - Simulation
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Launch arguments - TCP
    declare_tcp_ip = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='IP address of the LIDAR TCP server'
    )
    
    declare_tcp_port = DeclareLaunchArgument(
        'tcp_port',
        default_value='12350',
        description='Port number of the LIDAR TCP server'
    )
    
    declare_point_size = DeclareLaunchArgument(
        'point_size',
        default_value='2.0',
        description='Size of the point markers for visualization'
    )
    
    declare_verbose_logging = DeclareLaunchArgument(
        'verbose_logging',
        default_value='false',
        description='Enable verbose logging'
    )
    
    # Launch arguments - Vehicle
    declare_vehicle_length = DeclareLaunchArgument(
        'vehicle_length',
        default_value='4.5',
        description='Length of vehicle in meters (x direction)'
    )
    
    declare_vehicle_width = DeclareLaunchArgument(
        'vehicle_width',
        default_value='2.0',
        description='Width of vehicle in meters (y direction)'
    )
    
    declare_vehicle_height = DeclareLaunchArgument(
        'vehicle_height',
        default_value='1.8',
        description='Height of vehicle in meters (z direction)'
    )
    
    declare_vehicle_x_offset = DeclareLaunchArgument(
        'vehicle_x_offset',
        default_value='-0.75',
        description='Offset of vehicle center in x direction (negative for Carla where LIDAR is 1.5m forward)'
    )
    
    declare_vehicle_y_offset = DeclareLaunchArgument(
        'vehicle_y_offset',
        default_value='0.0',
        description='Offset of vehicle center in y direction'
    )
    
    declare_vehicle_safety_margin = DeclareLaunchArgument(
        'vehicle_safety_margin',
        default_value='0.2',
        description='Extra margin around vehicle to filter'
    )
    
    declare_visualize_vehicle = DeclareLaunchArgument(
        'visualize_vehicle',
        default_value='true',
        description='Whether to visualize the vehicle filter zone'
    )
    
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/lidar/filtered_points',
        description='Output topic for filtered point cloud'
    )
    
    # New comparison arguments
    declare_visualize_removed_points = DeclareLaunchArgument(
        'visualize_removed_points',
        default_value='true',
        description='Show points that would be filtered out in red'
    )
    
    declare_show_comparison_stats = DeclareLaunchArgument(
        'show_comparison_stats',
        default_value='true',
        description='Print detailed comparison statistics'
    )
    
    declare_use_box_mode = DeclareLaunchArgument(
        'use_box_mode',
        default_value='true',
        description='Use box-shaped filter instead of cylinder'
    )
    
    declare_filter_height_min = DeclareLaunchArgument(
        'filter_height_min',
        default_value='-1.5',
        description='Minimum height for filtering in box mode'
    )
    
    declare_filter_height_max = DeclareLaunchArgument(
        'filter_height_max',
        default_value='2.0',
        description='Maximum height for filtering in box mode'
    )
    
    declare_debug_mode = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debugging visualizations'
    )
    
    declare_comparison_update_rate = DeclareLaunchArgument(
        'comparison_update_rate',
        default_value='0.5',
        description='Update rate for comparison visualization in seconds'
    )
    
    # Launch arguments - Frame IDs for Carla
    declare_lidar_frame_id = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='lidar',
        description='Frame ID for LIDAR point cloud messages'
    )
    
    declare_map_frame_id = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map', 
        description='Frame ID for visualization in the map frame'
    )
    
    # Define the TF static transform publisher for basic frame setup
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_lidar',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Define the LIDAR filter node
    lidar_vehicle_filter_node = Node(
        package='sensor_fusion',
        executable='lidar_vehicle_filter_node',
        name='lidar_vehicle_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # TCP Parameters
            'tcp_ip': tcp_ip,
            'tcp_port': tcp_port,
            'point_size': point_size,
            'verbose_logging': verbose_logging,
            # Vehicle Parameters
            'vehicle_length': vehicle_length,
            'vehicle_width': vehicle_width,
            'vehicle_height': vehicle_height,
            'vehicle_x_offset': vehicle_x_offset,
            'vehicle_y_offset': vehicle_y_offset,
            'vehicle_safety_margin': vehicle_safety_margin,
            'visualize_vehicle': visualize_vehicle,
            'output_topic': output_topic,
            # Comparison Parameters
            'visualize_removed_points': visualize_removed_points,
            'show_comparison_stats': show_comparison_stats,
            'use_box_mode': use_box_mode,
            'filter_height_min': filter_height_min,
            'filter_height_max': filter_height_max,
            'debug_mode': debug_mode,
            'comparison_update_rate': comparison_update_rate,
            # Frame Parameters for Carla
            'lidar_frame_id': lidar_frame_id,
            'map_frame_id': map_frame_id,
        }]
    )
    
    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        # Launch arguments - Simulation
        declare_use_sim_time,
        # Launch arguments - TCP
        declare_tcp_ip,
        declare_tcp_port,
        declare_point_size,
        declare_verbose_logging,
        # Launch arguments - Vehicle
        declare_vehicle_length,
        declare_vehicle_width,
        declare_vehicle_height,
        declare_vehicle_x_offset,
        declare_vehicle_y_offset,
        declare_vehicle_safety_margin,
        declare_visualize_vehicle,
        declare_output_topic,
        # New comparison arguments
        declare_visualize_removed_points,
        declare_show_comparison_stats,
        declare_use_box_mode,
        declare_filter_height_min,
        declare_filter_height_max,
        declare_debug_mode,
        declare_comparison_update_rate,
        # Launch arguments - Frame IDs for Carla
        declare_lidar_frame_id,
        declare_map_frame_id,
        
        # Nodes
        static_tf_node,
        lidar_vehicle_filter_node,
        rviz_node,
    ]) 