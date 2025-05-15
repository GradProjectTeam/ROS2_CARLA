#!/usr/bin/env python3
# Layer 4 Launch File: Trajectory Visualization

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # =============== Launch Arguments ===============
    
    # Simulation parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # Map parameters  
    map_topic = LaunchConfiguration('map_topic', default='/realtime_map')
    grid_size = LaunchConfiguration('grid_size', default='0.2')  # meters per grid cell
    
    # Planning parameters
    planner_publish_rate = LaunchConfiguration('planner_publish_rate', default='10.0')  # Hz
    
    # Vehicle parameters
    wheelbase = LaunchConfiguration('wheelbase', default='2.7')
    vehicle_width = LaunchConfiguration('vehicle_width', default='2.0')
    vehicle_length = LaunchConfiguration('vehicle_length', default='4.0')
    
    # Visualization parameters
    arrow_scale = LaunchConfiguration('arrow_scale', default='0.5')
    
    # Launch arguments
    package_dir = get_package_share_directory('sensor_fusion')
    
    # Declare launch arguments
    launch_args = [
        # Simulation parameters
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        
        # Map parameters
        DeclareLaunchArgument('map_topic', default_value='/realtime_map', description='Map topic name'),
        DeclareLaunchArgument('grid_size', default_value='0.2', description='Grid size in meters'),
        
        # Planner parameters
        DeclareLaunchArgument('planner_publish_rate', default_value='10.0', 
                             description='Planner publish rate in Hz'),
        
        # Vehicle parameters
        DeclareLaunchArgument('wheelbase', default_value='2.7', 
                             description='Vehicle wheelbase in meters'),
        DeclareLaunchArgument('vehicle_width', default_value='2.0', 
                             description='Vehicle width in meters'),
        DeclareLaunchArgument('vehicle_length', default_value='4.0', 
                             description='Vehicle length in meters'),
        
        # Visualization parameters
        DeclareLaunchArgument('arrow_scale', default_value='0.5', 
                             description='Scale factor for visualization arrows'),
    ]
    
    # =============== Include Layer 3 Launch File ===============
    
    layer3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'layered_03_path_to_trajectory.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'planner_publish_rate': planner_publish_rate,
            'wheelbase': wheelbase,
            'vehicle_width': vehicle_width,
            'vehicle_length': vehicle_length
        }.items()
    )
    
    # =============== Trajectory Visualizer Node ===============
    
    trajectory_visualizer = Node(
        package='sensor_fusion',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_width': vehicle_width,
            'vehicle_length': vehicle_length,
            'wheelbase': wheelbase,
            'arrow_scale': arrow_scale
        }]
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # Include Layer 3 (which includes Layers 1 and 2)
            layer3_launch,
            
            # Start trajectory visualizer
            trajectory_visualizer,
            
            # Log info
            LogInfo(msg="Layer 4 (Trajectory Visualization) launched successfully")
        ]
    ) 