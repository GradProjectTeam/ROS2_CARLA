#!/usr/bin/env python3
"""
Waypoint Path Planning Launch File

This launch file builds on the waypoint visualization system and adds path planning capabilities.
It configures the system to:
1. Receive waypoints from CARLA simulation via TCP
2. Process and visualize these waypoints on a binary map
3. Generate a path plan from the waypoints
4. Publish the path plan for navigation

Features:
- TCP connection to CARLA for waypoint data reception
- Binary map visualization of waypoints
- Path planning based on waypoints
- Path visualization in RViz
- Fixed map origin to keep waypoints and path local to the origin
- Persistent visualization that stays visible on the map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'waypoint_path_planning.rviz')
    
    # Define the path to the waypoint visualization launch file
    waypoint_viz_launch_file = os.path.join(pkg_share, 'launch', 'waypoint_visualization.launch.py')
    
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
    
    # Path planning parameters
    declare_path_topic = DeclareLaunchArgument(
        'path_topic',
        default_value='/waypoint_path',
        description='Topic name for publishing path plan'
    )
    
    declare_path_visualization_topic = DeclareLaunchArgument(
        'path_visualization_topic',
        default_value='/visualization/waypoint_path',
        description='Topic name for publishing path visualization'
    )
    
    declare_path_update_rate = DeclareLaunchArgument(
        'path_update_rate',
        default_value='10.0',
        description='Rate at which to update path plan (Hz)'
    )
    
    declare_path_lookahead = DeclareLaunchArgument(
        'path_lookahead',
        default_value='50.0',
        description='Lookahead distance for path planning (meters)'
    )
    
    declare_path_density = DeclareLaunchArgument(
        'path_density',
        default_value='1.0',
        description='Density of points in path (points per meter)'
    )
    
    declare_path_smoothing = DeclareLaunchArgument(
        'path_smoothing',
        default_value='true',
        description='Apply smoothing to path'
    )
    
    declare_path_width = DeclareLaunchArgument(
        'path_width',
        default_value='0.2',
        description='Width of path visualization (meters)'
    )
    
    declare_path_lifetime = DeclareLaunchArgument(
        'path_lifetime',
        default_value='0.0',
        description='Lifetime of path visualization (seconds, 0.0 = never expire)'
    )
    
    # ==================== PATH PLANNER NODE ====================
    waypoint_path_planner_node = Node(
        package='sensor_fusion_2',
        executable='waypoint_path_planner',
        name='waypoint_path_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frame_id': 'map',
            'vehicle_frame_id': 'base_link',
            'waypoints_topic': '/carla/waypoints',
            'path_topic': LaunchConfiguration('path_topic'),
            'path_visualization_topic': LaunchConfiguration('path_visualization_topic'),
            'path_update_rate': LaunchConfiguration('path_update_rate'),
            'path_lookahead': LaunchConfiguration('path_lookahead'),
            'path_density': LaunchConfiguration('path_density'),
            'path_smoothing': LaunchConfiguration('path_smoothing'),
            'path_width': LaunchConfiguration('path_width'),
            'path_lifetime': LaunchConfiguration('path_lifetime'),
            'use_binary_map': True,
            'binary_map_topic': '/waypoint_map/binary',
            'fixed_origin': True,
            'use_local_coordinates': True,
        }],
        output='screen'
    )
    
    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription([
        # Launch Arguments
        declare_use_sim_time,
        declare_show_rviz,
        declare_path_topic,
        declare_path_visualization_topic,
        declare_path_update_rate,
        declare_path_lookahead,
        declare_path_density,
        declare_path_smoothing,
        declare_path_width,
        declare_path_lifetime,
        
        # Include waypoint visualization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(waypoint_viz_launch_file),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'show_rviz': 'false',  # Don't show RViz from included launch file
            }.items()
        ),
        
        # Add a timing delay to ensure waypoint visualization is established
        TimerAction(
            period=3.0,  # Wait 3 seconds for waypoint visualization to be established
            actions=[
                # Path planning node
                waypoint_path_planner_node,
                
                # RViz with path planning configuration
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    condition=IfCondition(LaunchConfiguration('show_rviz')),
                    output='screen'
                ),
            ]
        )
    ]) 