#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    publish_rate = LaunchConfiguration('publish_rate')
    temporal_filtering = LaunchConfiguration('temporal_filtering')
    motion_prediction = LaunchConfiguration('motion_prediction')
    enable_3d_visualization = LaunchConfiguration('enable_3d_visualization')
    enable_text_labels = LaunchConfiguration('enable_text_labels')
    
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
    
    # Semantic costmap visualizer node
    semantic_costmap_visualizer_node = Node(
        package='sensor_fusion_2',
        executable='semantic_costmap_visualizer.py',
        name='semantic_costmap_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_resolution': map_resolution,
            'map_width_meters': map_width_meters,
            'map_height_meters': map_height_meters,
            'publish_rate': publish_rate,
            'temporal_filtering': temporal_filtering,
            'motion_prediction': motion_prediction,
            'enable_3d_visualization': enable_3d_visualization,
            'enable_text_labels': enable_text_labels,
        }],
        remappings=[
            ('/lidar/points', '/lidar/points'),
            ('/lidar/cubes', '/lidar/cubes'),
            ('/radar/points', '/radar/points'),
            ('/radar/clusters', '/radar/clusters'),
        ]
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
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_resolution)
    ld.add_action(declare_map_width_meters)
    ld.add_action(declare_map_height_meters)
    ld.add_action(declare_publish_rate)
    ld.add_action(declare_temporal_filtering)
    ld.add_action(declare_motion_prediction)
    ld.add_action(declare_enable_3d_visualization)
    ld.add_action(declare_enable_text_labels)
    
    # Add nodes to launch description
    ld.add_action(semantic_costmap_visualizer_node)
    ld.add_action(rviz_node)
    
    return ld 