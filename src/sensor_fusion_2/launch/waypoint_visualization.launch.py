#!/usr/bin/env python3
"""
Waypoint Visualization Launch File - For CARLA Waypoint Visualization

This launch file configures the system to:
1. Receive waypoints from CARLA simulation via TCP
2. Process and visualize these waypoints on a binary map
3. Publish the waypoint map for navigation planning

Features:
- TCP connection to CARLA for waypoint data reception
- Binary map visualization of waypoints
- Integration with existing semantic costmap for navigation
- RViz visualization of waypoints and map
- Fixed map origin to keep waypoints local to the origin
- Persistent waypoint visualization that stays visible on the map
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
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'waypoint_visualization.rviz')
    
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
    
    # TCP connection parameters for waypoints
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
    
    # Add reconnection parameters
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
    
    # Map parameters
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.5',
        description='Resolution of the map in meters per cell'
    )
    
    declare_map_width = DeclareLaunchArgument(
        'map_width_meters',
        default_value='120.0',
        description='Width of the map in meters'
    )
    
    declare_map_height = DeclareLaunchArgument(
        'map_height_meters',
        default_value='120.0',
        description='Height of the map in meters'
    )
    
    # Add map origin parameters to ensure waypoints are local to the origin
    declare_map_origin_x = DeclareLaunchArgument(
        'map_origin_x',
        default_value='-60.0',  # Center the map (half of width)
        description='X origin of the map in meters'
    )
    
    declare_map_origin_y = DeclareLaunchArgument(
        'map_origin_y',
        default_value='-60.0',  # Center the map (half of height)
        description='Y origin of the map in meters'
    )
    
    declare_publish_rate = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Rate at which to publish map layers (Hz)'
    )
    
    # Waypoint visualization parameters
    declare_waypoint_marker_size = DeclareLaunchArgument(
        'waypoint_marker_size',
        default_value='0.3',  # Increased from 0.2 for better visibility
        description='Size of waypoint markers in meters'
    )
    
    declare_waypoint_line_width = DeclareLaunchArgument(
        'waypoint_line_width',
        default_value='0.2',  # Increased from 0.1 for better visibility
        description='Width of lines connecting waypoints in meters'
    )
    
    declare_waypoint_lifetime = DeclareLaunchArgument(
        'waypoint_lifetime',
        default_value='0.0',  # Changed to 0.0 for permanent visibility (never expires)
        description='Lifetime of waypoint visualization markers in seconds (0.0 = never expire)'
    )
    
    # Binary map parameters
    declare_binary_topic = DeclareLaunchArgument(
        'binary_topic',
        default_value='/waypoint_map/binary',
        description='Topic name for publishing binary waypoint map'
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
    
    declare_waypoint_width = DeclareLaunchArgument(
        'waypoint_width',
        default_value='1.5',  # Increased from 1.0 for better visibility
        description='Width of waypoints in cells on the binary map'
    )
    
    # Add parameter for local waypoint transformation
    declare_use_local_coordinates = DeclareLaunchArgument(
        'use_local_coordinates',
        default_value='true',
        description='Transform waypoints to local coordinates relative to map origin'
    )
    
    # Add parameter for fixed origin mode
    declare_fixed_origin = DeclareLaunchArgument(
        'fixed_origin',
        default_value='true',
        description='Use fixed origin for waypoints to prevent them from moving away'
    )
    
    # Add parameter for persistent visualization
    declare_persistent_markers = DeclareLaunchArgument(
        'persistent_markers',
        default_value='true',
        description='Keep markers visible even when moving away'
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
    
    # Map to base_link transform - fixed at origin to ensure waypoints stay local
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
            'frame_id': LaunchConfiguration('map_frame_id'),  # Changed from vehicle_frame_id to map_frame_id to keep waypoints fixed to map
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
            'map_width': LaunchConfiguration('map_width_meters'),
            'map_height': LaunchConfiguration('map_height_meters'),
            'map_frame': LaunchConfiguration('map_frame_id'),
            # Explicitly set map origin to keep waypoints local
            'map_origin_x': LaunchConfiguration('map_origin_x'),
            'map_origin_y': LaunchConfiguration('map_origin_y'),
            'waypoints_topic': '/carla/waypoints',
            'map_topic': '/waypoint_map',
            'binary_topic': LaunchConfiguration('binary_topic'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'occupied_value': LaunchConfiguration('occupied_value'),
            'free_value': LaunchConfiguration('free_value'),
            'waypoint_width': LaunchConfiguration('waypoint_width'),
            'use_reliable_qos': True,
            'use_transient_local_durability': True,
            'use_local_coordinates': LaunchConfiguration('use_local_coordinates'),
            'fixed_origin': LaunchConfiguration('fixed_origin'),
            'persistent_markers': LaunchConfiguration('persistent_markers'),
            # Add parameters to ensure map is properly generated
            'publish_map_metadata': True,
            'always_publish': True,
            'initialize_empty_map': True,  # Initialize an empty map at startup
            'keep_waypoints_on_map': True,  # Keep waypoints on map even when moving away
            'accumulate_waypoints': True,  # Accumulate waypoints instead of replacing them
        }],
        output='screen'
    )
    
    # ==================== WAYPOINT VISUALIZER NODE ====================
    waypoint_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='waypoint_visualizer',
        name='waypoint_visualizer',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': LaunchConfiguration('map_frame_id'),
            'waypoints_topic': '/carla/waypoints',
            'visualization_topic': '/visualization/waypoints',
            'marker_size': LaunchConfiguration('waypoint_marker_size'),
            'line_width': LaunchConfiguration('waypoint_line_width'),
            'marker_lifetime': LaunchConfiguration('waypoint_lifetime'),
            'show_lane_info': True,
            'show_road_info': True,
            'color_by_lane_type': True,
            'use_local_coordinates': LaunchConfiguration('use_local_coordinates'),
            'fixed_origin': LaunchConfiguration('fixed_origin'),
            'persistent_markers': LaunchConfiguration('persistent_markers'),
            'clear_markers_on_update': False,  # Don't clear previous markers when new ones arrive
            'use_persistent_durability': True,  # Use persistent durability for markers
        }],
        output='screen'
    )
    
    # ==================== RVIZ NODE ====================
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
        declare_waypoint_marker_size,
        declare_waypoint_line_width,
        declare_waypoint_lifetime,
        declare_binary_topic,
        declare_occupied_value,
        declare_free_value,
        declare_waypoint_width,
        declare_use_local_coordinates,
        declare_fixed_origin,
        declare_persistent_markers,
        
        # TF Tree Nodes
        world_to_map_node,
        map_to_base_link_node,
        
        # Add a timing delay to ensure TF tree is established before other nodes start
        TimerAction(
            period=2.0,  # Wait 2 seconds for TF tree to be established
            actions=[
                # Waypoint Nodes
                waypoint_listener_node,
                waypoint_map_generator_node,
                waypoint_visualizer_node,
                
                # Visualization
                rviz_node,
            ]
        )
    ]) 