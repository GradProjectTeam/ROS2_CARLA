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
    
    # =========== LAUNCH ARGUMENTS ===========
    # Basic parameters
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    
    # Network parameters for sensors
    lidar_tcp_ip_arg = DeclareLaunchArgument('lidar_tcp_ip', default_value='127.0.0.1')
    lidar_tcp_port_arg = DeclareLaunchArgument('lidar_tcp_port', default_value='8080')
    imu_tcp_ip_arg = DeclareLaunchArgument('imu_tcp_ip', default_value='127.0.0.1')
    imu_tcp_port_arg = DeclareLaunchArgument('imu_tcp_port', default_value='8081')
    
    # LiDAR mounting position parameters
    lidar_x_offset_arg = DeclareLaunchArgument('lidar_x_offset', default_value='0.0')
    lidar_y_offset_arg = DeclareLaunchArgument('lidar_y_offset', default_value='0.0')
    lidar_z_offset_arg = DeclareLaunchArgument('lidar_z_offset', default_value='0.0')
    lidar_roll_arg = DeclareLaunchArgument('lidar_roll', default_value='0.0')
    lidar_pitch_arg = DeclareLaunchArgument('lidar_pitch', default_value='0.0')
    lidar_yaw_arg = DeclareLaunchArgument('lidar_yaw', default_value='0.0')
    
    # DWA parameters
    robot_radius_arg = DeclareLaunchArgument('robot_radius', default_value='1.0')
    max_linear_velocity_arg = DeclareLaunchArgument('max_linear_velocity', default_value='5.0')
    min_linear_velocity_arg = DeclareLaunchArgument('min_linear_velocity', default_value='0.0')
    max_angular_velocity_arg = DeclareLaunchArgument('max_angular_velocity', default_value='0.5')
    min_angular_velocity_arg = DeclareLaunchArgument('min_angular_velocity', default_value='-0.5')
    max_linear_accel_arg = DeclareLaunchArgument('max_linear_accel', default_value='1.0')
    max_angular_accel_arg = DeclareLaunchArgument('max_angular_accel', default_value='0.5')
    
    # Cost function weights
    obstacle_weight_arg = DeclareLaunchArgument('obstacle_weight', default_value='2.0')
    goal_weight_arg = DeclareLaunchArgument('goal_weight', default_value='1.0')
    heading_weight_arg = DeclareLaunchArgument('heading_weight', default_value='1.0')
    velocity_weight_arg = DeclareLaunchArgument('velocity_weight', default_value='0.5')
    
    # Goal parameters
    goal_tolerance_arg = DeclareLaunchArgument('goal_tolerance', default_value='2.0')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='50.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='50.0')
    
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
    # This is a fallback static transform until the dynamic one from imu_lidar_yaw_fusion takes over
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
    
    # =========== SENSOR DATA PROCESSING ===========
    # LiDAR data processing node
    lidar_listener_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_cube_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )
    
    # IMU data processing node
    imu_euler_visualizer_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
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
    
    # =========== DWA PLANNER CONFIGURATION ===========
    # DWA Planner Node
    dwa_planner_node = Node(
        package='sensor_fusion',
        executable='dwa_planner_node',
        name='dwa_planner',
        output='screen',
        parameters=[{
            # Robot parameters
            'robot_radius': LaunchConfiguration('robot_radius'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'min_linear_velocity': LaunchConfiguration('min_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'min_angular_velocity': LaunchConfiguration('min_angular_velocity'),
            'max_linear_accel': LaunchConfiguration('max_linear_accel'),
            'max_angular_accel': LaunchConfiguration('max_angular_accel'),
            'velocity_resolution': 0.1,
            'angular_velocity_resolution': 0.1,
            'prediction_time': 3.0,
            'prediction_steps': 60,
            
            # Cost function weights
            'obstacle_weight': LaunchConfiguration('obstacle_weight'),
            'goal_weight': LaunchConfiguration('goal_weight'),
            'heading_weight': LaunchConfiguration('heading_weight'),
            'velocity_weight': LaunchConfiguration('velocity_weight'),
            
            # Goal parameters
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            
            # Update frequencies
            'control_frequency': 10.0,
            'visualization_frequency': 5.0,
            
            # Frame IDs
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            
            # Visualization parameters
            'show_trajectories': True,
            'max_trajectories_shown': 10,
            
            # Use simulation time
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # =========== PATH TO TRAJECTORY CONVERTER ===========
    # This node converts the DWA path to a trajectory format compatible with Carla
    path_to_trajectory_converter = Node(
        package='sensor_fusion',
        executable='path_to_trajectory_converter',
        name='path_to_trajectory_converter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_path_topic': '/dwa/best_trajectory',
            'output_trajectory_topic': '/trajectory',
            'vehicle_width': 2.0,
            'vehicle_length': 4.5,
            'max_speed': LaunchConfiguration('max_linear_velocity'),
            'min_speed': LaunchConfiguration('min_linear_velocity'),
            'publish_rate': 10.0
        }],
    )
    
    # =========== VISUALIZATION ===========
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'dwa_planner.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        # Basic parameters
        use_sim_time_arg,
        
        # Network parameters
        lidar_tcp_ip_arg,
        lidar_tcp_port_arg,
        imu_tcp_ip_arg,
        imu_tcp_port_arg,
        
        # LiDAR mounting parameters
        lidar_x_offset_arg,
        lidar_y_offset_arg,
        lidar_z_offset_arg,
        lidar_roll_arg,
        lidar_pitch_arg,
        lidar_yaw_arg,
        
        # DWA parameters
        robot_radius_arg,
        max_linear_velocity_arg,
        min_linear_velocity_arg,
        max_angular_velocity_arg,
        min_angular_velocity_arg,
        max_linear_accel_arg,
        max_angular_accel_arg,
        
        # Cost function weights
        obstacle_weight_arg,
        goal_weight_arg,
        heading_weight_arg,
        velocity_weight_arg,
        
        # Goal parameters
        goal_tolerance_arg,
        goal_x_arg,
        goal_y_arg,
        
        # TF tree nodes
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Sensor data processing nodes
        lidar_listener_node,
        imu_euler_visualizer_node,
        imu_lidar_fusion_node,
        
        # Planning and trajectory nodes
        dwa_planner_node,
        path_to_trajectory_converter,
        
        # Visualization
        rviz_node,
    ])
