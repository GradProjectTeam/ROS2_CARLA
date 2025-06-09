#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for testing DWA local planner."""
    
    # Launch arguments
    imu_tcp_ip_arg = DeclareLaunchArgument(
        'imu_tcp_ip', default_value='0.0.0.0',
        description='IP address for IMU TCP connection'
    )
    
    imu_tcp_port_arg = DeclareLaunchArgument(
        'imu_tcp_port', default_value='8888',
        description='Port for IMU TCP connection'
    )
    
    lidar_tcp_ip_arg = DeclareLaunchArgument(
        'lidar_tcp_ip', default_value='0.0.0.0',
        description='IP address for LiDAR TCP connection'
    )
    
    lidar_tcp_port_arg = DeclareLaunchArgument(
        'lidar_tcp_port', default_value='2368',
        description='Port for LiDAR TCP connection'
    )
    
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode', default_value='false',
        description='Whether to run in simulation mode'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value='/imu/data',
        description='Topic for IMU data'
    )
    
    map_topic_arg = DeclareLaunchArgument(
        'map_topic', default_value='/map',
        description='Topic for map data'
    )
    
    # Get launch configurations
    imu_tcp_ip = LaunchConfiguration('imu_tcp_ip')
    imu_tcp_port = LaunchConfiguration('imu_tcp_port')
    lidar_tcp_ip = LaunchConfiguration('lidar_tcp_ip')
    lidar_tcp_port = LaunchConfiguration('lidar_tcp_port')
    simulation_mode = LaunchConfiguration('simulation_mode')
    imu_topic = LaunchConfiguration('imu_topic')
    map_topic = LaunchConfiguration('map_topic')
    
    # Command to check and install libtiff6 dependency
    install_dependencies_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'dpkg -l | grep -q libtiff6 || sudo apt-get update && sudo apt-get install -y libtiff6'],
        name='install_dependencies',
        shell=True
    )
    
    # TF Tree Setup
    # Static transforms for coordinate frames
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': simulation_mode}]
    )
    
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': simulation_mode}]
    )
    
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': simulation_mode}]
    )
    
    # IMU Visualization Node
    imu_euler_visualizer_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'use_sim_time': simulation_mode,
            'tcp_ip': imu_tcp_ip,
            'tcp_port': imu_tcp_port,
            'simulation_mode': simulation_mode,
            'publish_tf': True,
            'imu_topic': imu_topic
        }],
        output='screen'
    )
    
    # LiDAR Listener Node - Changed to use available executable
    lidar_listener_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_listener',
        parameters=[{
            'use_sim_time': simulation_mode,
            'tcp_ip': lidar_tcp_ip,
            'tcp_port': lidar_tcp_port,
            'simulation_mode': simulation_mode,
            'point_size': 0.05,
            'rate': 10,
            'log_data': False
        }],
        output='screen'
    )
    
    # Test Navigation Node
    test_navigation_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='navigation_tester',
        parameters=[{
            'use_sim_time': simulation_mode,
            'imu_topic': imu_topic,
            'map_topic': map_topic,
            'goal_distance': 10.0,
            'update_rate': 5.0,
            'use_current_orientation': True
        }],
        output='screen'
    )
    
    # Orientation TF Handler Node - Using test_navigation with specific parameters
    orientation_tf_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='tf_orientation_handler',
        parameters=[{
            'use_sim_time': simulation_mode,
            'imu_topic': imu_topic,
            'publish_tf': True,
            'publish_goal': False,
            'parent_frame': 'map',
            'child_frame': 'base_link',
            'update_rate': 50.0
        }],
        output='screen'
    )
    
    # Mapper Node - Started with a delay to ensure the TF tree is set up
    lidar_mapper_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='sensor_fusion',
                executable='lidar_realtime_mapper',
                name='lidar_realtime_mapper',
                parameters=[{
                    'use_sim_time': simulation_mode,
                    'map_resolution': 0.1,  # meters per cell
                    'map_width': 1000,      # cells
                    'map_height': 1000,     # cells
                    'map_origin_x': -50.0,  # meters
                    'map_origin_y': -50.0,  # meters
                    'map_topic': map_topic,
                    'obstacle_threshold': 70,
                    'update_rate': 10.0,
                    'decay_rate': 0.01
                }],
                output='screen'
            )
        ]
    )
    
    # Reference Path Generator - Create a simple command to publish a path
    reference_path_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5 && ros2 topic pub --once /reference_path nav_msgs/msg/Path "{"header: {frame_id: \'map\'}, poses: [{pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}, {pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}}, {pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}]}"'],
        name='reference_path_publisher',
        output='screen'
    )
    
    # DWA Local Planner Node - Started with a delay for map initialization
    dwa_planner_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='sensor_fusion',
                executable='dwa_local_planner',
                name='dwa_local_planner',
                parameters=[{
                    'use_sim_time': simulation_mode,
                    'map_topic': map_topic,
                    'max_linear_velocity': 1.0,      # m/s
                    'max_angular_velocity': 1.0,     # rad/s
                    'linear_velocity_resolution': 0.1,
                    'angular_velocity_resolution': 0.1,
                    'prediction_time': 3.0,          # seconds to simulate forward
                    'simulation_time_step': 0.1,     # time step for simulation
                    'goal_weight': 1.0,
                    'obstacle_weight': 1.0,
                    'velocity_weight': 0.5,
                    'wheelbase': 0.5,               # vehicle wheelbase in meters
                    'vehicle_width': 0.4,           # vehicle width in meters
                    'vehicle_length': 0.6,          # vehicle length in meters
                    'update_rate': 5.0,             # Hz
                    'use_current_orientation': True,
                    'obstacle_inflation': 0.3,      # meters
                    'path_topic': '/reference_path'  # Use the manually published path
                }],
                output='screen'
            )
        ]
    )
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(os.getcwd(), 'src', 'sensor_fusion', 'config', 'navigation.rviz')],
        parameters=[{'use_sim_time': simulation_mode}]
    )
    
    # Return the launch description
    return LaunchDescription([
        # Arguments
        imu_tcp_ip_arg,
        imu_tcp_port_arg,
        lidar_tcp_ip_arg,
        lidar_tcp_port_arg,
        simulation_mode_arg,
        imu_topic_arg,
        map_topic_arg,
        
        # Install dependencies
        install_dependencies_cmd,
        
        # TF Tree Setup
        world_to_map_node,
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Sensor Nodes
        imu_euler_visualizer_node,
        lidar_listener_node,
        
        # Navigation Nodes
        test_navigation_node,
        orientation_tf_node,
        
        # Mapper Node
        lidar_mapper_node,
        
        # Path Generation and Planning
        reference_path_cmd,
        dwa_planner_node,
        
        # Visualization
        rviz_node
    ])