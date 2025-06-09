#!/usr/bin/env python3
# Multi-planner comparison launch file - launches all four path planning algorithms with proper configuration

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction,
    IncludeLaunchDescription, LogInfo, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # =============== Launch Arguments ===============
    
    # Connection parameters
    imu_tcp_ip = LaunchConfiguration('imu_tcp_ip', default='0.0.0.0')
    imu_tcp_port = LaunchConfiguration('imu_tcp_port', default='12345')
    lidar_tcp_ip = LaunchConfiguration('lidar_tcp_ip', default='0.0.0.0')  
    lidar_tcp_port = LaunchConfiguration('lidar_tcp_port', default='12350')
    
    # Simulation parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # Map parameters  
    map_topic = LaunchConfiguration('map_topic', default='/realtime_map')
    grid_size = LaunchConfiguration('grid_size', default='0.2')  # meters per grid cell
    obstacle_threshold = LaunchConfiguration('obstacle_threshold', default='50')  # 0-100 occupancy threshold
    
    # IMU parameters
    use_imu = LaunchConfiguration('use_imu', default='True')
    imu_orientation_correction = LaunchConfiguration('imu_orientation_correction', default='0.0')  # radians
    
    # Planning parameters
    max_iterations = LaunchConfiguration('max_iterations', default='3000')  # planning iterations
    planner_publish_rate = LaunchConfiguration('planner_publish_rate', default='10.0')  # Hz
    planner_timeout = LaunchConfiguration('planner_timeout', default='0.5')  # seconds
    
    # MPC parameters
    prediction_horizon = LaunchConfiguration('prediction_horizon', default='20')
    control_horizon = LaunchConfiguration('control_horizon', default='10')
    
    # Trajectory parameters
    max_velocity = LaunchConfiguration('max_velocity', default='2.0')  # m/s
    max_acceleration = LaunchConfiguration('max_acceleration', default='1.0')  # m/s^2
    min_turning_radius = LaunchConfiguration('min_turning_radius', default='3.0')  # meters
    
    # Launch arguments
    package_dir = get_package_share_directory('sensor_fusion')
    rviz_config = os.path.join(package_dir, 'rviz', 'multi_planner.rviz')
    
    # Declare all launch arguments
    launch_args = [
        # Connection parameters
        DeclareLaunchArgument('imu_tcp_ip', default_value='0.0.0.0', description='IMU TCP IP address'),
        DeclareLaunchArgument('imu_tcp_port', default_value='12345', description='IMU TCP port'),
        DeclareLaunchArgument('lidar_tcp_ip', default_value='0.0.0.0', description='LiDAR TCP IP address'),
        DeclareLaunchArgument('lidar_tcp_port', default_value='12350', description='LiDAR TCP port'),
        # Simulation parameters
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        # Map parameters
        DeclareLaunchArgument('map_topic', default_value='/realtime_map', description='Map topic name'),
        DeclareLaunchArgument('grid_size', default_value='0.2', description='Grid size in meters'),
        DeclareLaunchArgument('obstacle_threshold', default_value='50', description='Obstacle threshold 0-100'),
        # IMU parameters
        DeclareLaunchArgument('use_imu', default_value='True', description='Use IMU for orientation'),
        DeclareLaunchArgument('imu_orientation_correction', default_value='0.0', description='IMU orientation correction in radians'),
        # Planning parameters
        DeclareLaunchArgument('max_iterations', default_value='3000', description='Maximum planning iterations'),
        DeclareLaunchArgument('planner_publish_rate', default_value='10.0', description='Planner publish rate in Hz'),
        DeclareLaunchArgument('planner_timeout', default_value='0.5', description='Planner timeout in seconds'),
        # MPC parameters
        DeclareLaunchArgument('prediction_horizon', default_value='20', description='MPC prediction horizon'),
        DeclareLaunchArgument('control_horizon', default_value='10', description='MPC control horizon'),
        # Trajectory parameters
        DeclareLaunchArgument('max_velocity', default_value='2.0', description='Maximum velocity in m/s'),
        DeclareLaunchArgument('max_acceleration', default_value='1.0', description='Maximum acceleration in m/s^2'),
        DeclareLaunchArgument('min_turning_radius', default_value='3.0', description='Minimum turning radius in meters'),
    ]
    
    # =============== Core Infrastructure ===============
    
    # Install dependencies
    install_dep_cmd = ExecuteProcess(
        cmd=['sudo apt-get update && sudo apt-get install -y python3-pip && pip3 install numpy==1.24.3 scipy matplotlib'],
        shell=True
    )
    
    # TF tree setup
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # =============== Sensor Nodes ===============
    
    # IMU visualization node
    imu_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tcp_ip': imu_tcp_ip,
            'tcp_port': imu_tcp_port,
            'orientation_correction': imu_orientation_correction
        }]
    )
    
    # LiDAR Processing Node
    lidar_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_listener',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tcp_ip': lidar_tcp_ip,
            'tcp_port': lidar_tcp_port,
            'publish_rate': 10.0,
            'vehicle_length': 4.0,
            'vehicle_width': 2.0
        }]
    )
    
    # Real-time Map Node
    map_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='realtime_mapper',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 5.0,
            'map_size_x': 100.0,
            'map_size_y': 100.0,
            'map_resolution': grid_size,
            'vehicle_length': 4.0,
            'vehicle_width': 2.0
        }]
    )
    
    # =============== Path Planners ===============
    
    # Hybrid A* Planner
    hybrid_astar_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'publish_rate': planner_publish_rate,
            'wheelbase': 2.7,
            'steering_angle_max': 0.5,  # ~28 degrees
            'obstacle_threshold': obstacle_threshold,
            'max_iterations': max_iterations,
            'timeout': planner_timeout,
            'topic_namespace': 'planner/hybrid_astar'
        }],
        remappings=[
            ('/path', '/planner/hybrid_astar/path'),
            ('/debug', '/planner/hybrid_astar/debug')
        ]
    )
    
    # A* Planner
    astar_node = Node(
        package='sensor_fusion',
        executable='test_realtime_astar',
        name='astar_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'publish_rate': planner_publish_rate,
            'obstacle_threshold': obstacle_threshold,
            'max_iterations': max_iterations,
            'timeout': planner_timeout,
            'topic_namespace': 'planner/astar'
        }],
        remappings=[
            ('/path', '/planner/astar/path'),
            ('/debug', '/planner/astar/debug')
        ]
    )
    
    # DWA Local Planner
    dwa_node = Node(
        package='sensor_fusion',
        executable='dwa_local_planner',
        name='dwa_local_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'publish_rate': planner_publish_rate,
            'obstacle_threshold': obstacle_threshold,
            'max_linear_vel': 3.0,  # m/s
            'max_angular_vel': 0.8,  # rad/s
            'linear_vel_resolution': 0.2,  # m/s
            'angular_vel_resolution': 0.2,  # rad/s
            'sim_time': 3.0,  # seconds to simulate
            'topic_namespace': 'planner/dwa'
        }],
        remappings=[
            ('/path', '/planner/dwa/path'),
            ('/trajectories', '/planner/dwa/trajectories')
        ]
    )
    
    # MPC Planner
    mpc_node = Node(
        package='sensor_fusion',
        executable='mpc_planner',
        name='mpc_planner',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': map_topic,
            'grid_size': grid_size,
            'publish_rate': planner_publish_rate,
            'obstacle_threshold': obstacle_threshold,
            'prediction_horizon': prediction_horizon,
            'control_horizon': control_horizon,
            'max_linear_vel': 3.0,  # m/s
            'max_steering_angle': 0.6,  # rad
            'topic_namespace': 'planner/mpc'
        }],
        remappings=[
            ('/path', '/planner/mpc/path'),
            ('/controls', '/planner/mpc/controls')
        ]
    )
    
    # =============== Path to Trajectory Converter ===============
    
    # Converter Node that transforms paths to trajectories with velocity and time information
    trajectory_converter_node = Node(
        package='sensor_fusion',
        executable='path_to_trajectory_converter',
        name='path_to_trajectory_converter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_velocity': max_velocity,
            'max_acceleration': max_acceleration,
            'target_dt': 0.1,  # Time step between trajectory points
            'wheelbase': 2.7,
            'vehicle_width': 2.0,
            'vehicle_length': 4.0,
            'min_radius': min_turning_radius
        }]
    )
    
    # Trajectory visualizer node to display trajectories in RViz
    trajectory_visualizer_node = Node(
        package='sensor_fusion',
        executable='trajectory_visualizer',
        name='trajectory_visualizer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_width': 2.0,
            'vehicle_length': 4.0,
            'wheelbase': 2.7,
            'arrow_scale': 0.5
        }]
    )
    
    # =============== Visualization ===============
    
    # RViz node with configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # =============== Test Node ===============
    
    # Goal publisher node
    goal_publisher_node = Node(
        package='sensor_fusion',
        executable='test_goal_pose_publisher',
        name='test_goal_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # =============== Delayed Starts ===============
    
    # Start planner nodes after a delay to let sensors and map initialize
    delayed_planners = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="Starting planner nodes..."),
            hybrid_astar_node,
            astar_node,
            dwa_node,
            mpc_node,
        ]
    )
    
    # Start trajectory converter after planners
    delayed_converter = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="Starting trajectory converter..."),
            trajectory_converter_node,
            trajectory_visualizer_node,
        ]
    )
    
    # Start goal publisher after delay to let planners initialize
    delayed_goal_publisher = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="Starting goal publisher..."),
            goal_publisher_node,
        ]
    )
    
    # Launch description
    return LaunchDescription(
        launch_args + [
            install_dep_cmd,
            tf_static_node,
            imu_node,
            lidar_node,
            map_node,
            rviz_node,
            delayed_planners,
            delayed_converter,
            delayed_goal_publisher,
        ]
    ) 