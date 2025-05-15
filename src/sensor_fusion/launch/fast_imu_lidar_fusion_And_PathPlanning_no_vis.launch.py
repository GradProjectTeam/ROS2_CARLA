#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    imu_tcp_ip_arg = DeclareLaunchArgument('imu_tcp_ip', default_value='127.0.0.1')
    imu_tcp_port_arg = DeclareLaunchArgument('imu_tcp_port', default_value='8080')
    lidar_x_offset_arg = DeclareLaunchArgument('lidar_x_offset', default_value='0.0')
    lidar_y_offset_arg = DeclareLaunchArgument('lidar_y_offset', default_value='0.0')
    lidar_z_offset_arg = DeclareLaunchArgument('lidar_z_offset', default_value='0.0')
    lidar_roll_arg = DeclareLaunchArgument('lidar_roll', default_value='0.0')
    lidar_pitch_arg = DeclareLaunchArgument('lidar_pitch', default_value='0.0')
    lidar_yaw_arg = DeclareLaunchArgument('lidar_yaw', default_value='0.0')

    # =========== TRANSFORM CONFIGURATION ===========
    # These transforms establish the complete TF tree for the robot
    # world → map → base_link → imu_link → lidar_link
    
    # Root transform: world to map
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # Map to base_link transform
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # Base to IMU transform
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=[
            '0',    # X offset (forward/back)
            '0',    # Y offset (left/right)
            '0.1',  # Z offset (up/down) - IMU is 10cm above the base
            '0',    # Roll (rotation around X)
            '0',    # Pitch (rotation around Y)
            '0',    # Yaw (rotation around Z)
            'base_link', 
            'imu_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )
    
    # IMU to LiDAR transform
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=[
            LaunchConfiguration('lidar_x_offset'),    # X offset (forward/back)
            LaunchConfiguration('lidar_y_offset'),    # Y offset (left/right)
            LaunchConfiguration('lidar_z_offset'),    # Z offset (up/down)
            LaunchConfiguration('lidar_roll'),        # Roll (rotation around X)
            LaunchConfiguration('lidar_pitch'),       # Pitch (rotation around Y)
            LaunchConfiguration('lidar_yaw'),         # Yaw (rotation around Z)
            'imu_link', 
            'lidar_link'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 200.0
        }],
    )

    # IMU data provider (without visualization)
    lidar_listener_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_listener',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # IMU-LiDAR fusion node
    imu_lidar_fusion_node = Node(
        package='sensor_fusion',
        executable='imu_lidar_yaw_fusion',
        name='imu_lidar_fusion',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # Mapper node
    mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_mapper',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # Path planning node
    path_planning_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='path_planning',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # Visualization nodes are commented out
    # imu_euler_visualizer_node = Node(
    #     package='sensor_fusion',
    #     executable='imu_euler_visualizer',
    #     name='imu_euler_visualizer',
    #     parameters=[{
    #         'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
    #         'tcp_port': LaunchConfiguration('imu_tcp_port'),
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #     }],
    # )
    
    # trajectory_visualizer_node = Node(
    #     package='sensor_fusion',
    #     executable='trajectory_visualizer',
    #     name='trajectory_visualizer',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #     }],
    # )

    return LaunchDescription([
        use_sim_time_arg,
        imu_tcp_ip_arg,
        imu_tcp_port_arg,
        lidar_x_offset_arg,
        lidar_y_offset_arg,
        lidar_z_offset_arg,
        lidar_roll_arg,
        lidar_pitch_arg,
        lidar_yaw_arg,
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        lidar_listener_node,
        imu_lidar_fusion_node,
        mapper_node,
        path_planning_node,
        # imu_euler_visualizer_node,  # Commented out
        # trajectory_visualizer_node   # Commented out
    ])
