#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion_2')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_resolution = LaunchConfiguration('map_resolution')
    map_width_meters = LaunchConfiguration('map_width_meters')
    map_height_meters = LaunchConfiguration('map_height_meters')
    
    # RViz configuration
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'semantic_costmap.rviz')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_resolution = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.2',
        description='Resolution of the costmap in meters per cell'
    )
    
    declare_map_width_meters = DeclareLaunchArgument(
        'map_width_meters',
        default_value='60.0',
        description='Width of the costmap in meters'
    )
    
    declare_map_height_meters = DeclareLaunchArgument(
        'map_height_meters',
        default_value='60.0',
        description='Height of the costmap in meters'
    )
    
    # Semantic costmap visualizer node
    semantic_costmap_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='semantic_costmap_visualizer',
        name='semantic_costmap_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_resolution': map_resolution,
            'map_width_meters': map_width_meters,
            'map_height_meters': map_height_meters,
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # rqt_reconfigure node for parameter tuning
    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure',
        output='screen'
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_resolution)
    ld.add_action(declare_map_width_meters)
    ld.add_action(declare_map_height_meters)
    
    # Add nodes to launch description
    ld.add_action(semantic_costmap_visualizer_node)
    ld.add_action(rviz_node)
    ld.add_action(rqt_reconfigure_node)
    
    return ld 