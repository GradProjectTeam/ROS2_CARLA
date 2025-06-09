#!/usr/bin/env python3
# Layer 1 Launch File: TF Tree and Map Generation

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
    
    # TCP connection parameters for IMU and LiDAR
    imu_tcp_ip = LaunchConfiguration('imu_tcp_ip', default='127.0.0.1')
    imu_tcp_port = LaunchConfiguration('imu_tcp_port', default='12345')
    lidar_tcp_ip = LaunchConfiguration('lidar_tcp_ip', default='127.0.0.1')
    lidar_tcp_port = LaunchConfiguration('lidar_tcp_port', default='12350')
    
    # Launch arguments
    package_dir = get_package_share_directory('sensor_fusion')
    
    # Declare launch arguments
    launch_args = [
        # Simulation parameters
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        
        # Map parameters
        DeclareLaunchArgument('map_topic', default_value='/realtime_map', description='Map topic name'),
        DeclareLaunchArgument('grid_size', default_value='0.2', description='Grid size in meters'),
        
        # TCP connection parameters
        DeclareLaunchArgument('imu_tcp_ip', default_value='127.0.0.1', 
                             description='IP address of the IMU TCP server'),
        DeclareLaunchArgument('imu_tcp_port', default_value='12345', 
                             description='Port number of the IMU TCP server'),
        DeclareLaunchArgument('lidar_tcp_ip', default_value='127.0.0.1', 
                             description='IP address of the LiDAR TCP server'),
        DeclareLaunchArgument('lidar_tcp_port', default_value='12350', 
                             description='Port number of the LiDAR TCP server'),
    ]
    
    # =============== Include Layer 0 Launch File ===============
    
    layer0_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(package_dir, 'launch', 'layered_00_lidar_imu.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'map_resolution': grid_size,
            'imu_tcp_ip': imu_tcp_ip,
            'imu_tcp_port': imu_tcp_port,
            'lidar_tcp_ip': lidar_tcp_ip,
            'lidar_tcp_port': lidar_tcp_port
        }.items()
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            # Include Layer 0
            layer0_launch,
            
            # Log info
            LogInfo(msg="Layer 1 (TF Tree and Map Generation) launched successfully")
        ]
    ) 