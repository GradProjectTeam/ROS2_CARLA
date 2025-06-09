#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Include the base sensor fusion launch file
    # This provides all the sensor fusion capabilities
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'fast_imu_lidar_fusion.launch.py')]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'lidar_tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'imu_tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'imu_tcp_port': LaunchConfiguration('imu_tcp_port'),
        }.items()
    )
    
    # =========== MOTION PLANNING CONFIGURATION ===========
    # Global Planner Node - Implements A* and Dijkstra algorithms
    global_planner_node = Node(
        package='sensor_fusion',
        executable='global_planner_node',
        name='global_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'planner_type': LaunchConfiguration('planner_type'),  # 'astar' or 'dijkstra'
            'map_topic': '/realtime_map',  # Use the map from sensor fusion
            'goal_topic': '/goal_pose',
            'path_topic': '/global_path',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'planning_frequency': 1.0,  # Plan once per second
            'obstacle_inflation_radius': 0.5,  # Inflate obstacles by this radius
            'path_resolution': 0.1,  # Resolution of the path in meters
            'heuristic_weight': 1.0,  # Weight for A* heuristic
            'enable_visualization': True,
            'visualize_search': True,  # Visualize the search process
            'visualize_heatmap': True,  # Visualize cost heatmap
        }],
        output='screen'
    )
    
    # Local Planner Node - Implements path following with obstacle avoidance
    local_planner_node = Node(
        package='sensor_fusion',
        executable='local_planner_node',
        name='local_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_path_topic': '/global_path',
            'local_path_topic': '/local_path',
            'cmd_vel_topic': '/cmd_vel',
            'map_topic': '/realtime_map',
            'obstacles_topic': '/obstacles',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'planning_frequency': 5.0,  # Plan 5 times per second
            'max_linear_velocity': 0.5,  # Maximum linear velocity in m/s
            'max_angular_velocity': 0.5,  # Maximum angular velocity in rad/s
            'goal_tolerance': 0.2,  # Goal tolerance in meters
            'obstacle_avoidance_weight': 0.8,  # Weight for obstacle avoidance
            'path_following_weight': 0.6,  # Weight for path following
            'goal_attraction_weight': 0.4,  # Weight for goal attraction
            'lookahead_distance': 1.0,  # Distance to look ahead on the path
            'enable_visualization': True,
        }],
        output='screen'
    )
    
    # Path Visualization Node - Visualizes the global and local paths
    path_visualization_node = Node(
        package='sensor_fusion',
        executable='path_visualization_node',
        name='path_visualization',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_path_topic': '/global_path',
            'local_path_topic': '/local_path',
            'map_frame_id': 'map',
            'global_path_color_r': 0.0,
            'global_path_color_g': 0.0,
            'global_path_color_b': 1.0,
            'global_path_color_a': 0.8,
            'local_path_color_r': 0.0,
            'local_path_color_g': 1.0,
            'local_path_color_b': 0.0,
            'local_path_color_a': 1.0,
            'path_line_width': 0.1,
            'path_lifetime': 0.0,  # 0 means persistent
        }],
        output='screen'
    )
    
    # Goal Publisher Node - Allows setting goals via RViz
    goal_publisher_node = Node(
        package='sensor_fusion',
        executable='goal_publisher_node',
        name='goal_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_frame_id': 'map',
            'goal_topic': '/goal_pose',
            'enable_visualization': True,
            'goal_marker_color_r': 1.0,
            'goal_marker_color_g': 0.0,
            'goal_marker_color_b': 0.0,
            'goal_marker_color_a': 1.0,
            'goal_marker_scale': 0.5,
        }],
        output='screen'
    )
    
    # Robot Controller Node - Controls the robot based on the local path
    robot_controller_node = Node(
        package='sensor_fusion',
        executable='robot_controller_node',
        name='robot_controller',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_topic': '/cmd_vel',
            'local_path_topic': '/local_path',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'control_frequency': 10.0,  # Control at 10Hz
            'max_linear_velocity': 0.5,  # Maximum linear velocity in m/s
            'max_angular_velocity': 0.5,  # Maximum angular velocity in rad/s
            'linear_velocity_gain': 1.0,  # Gain for linear velocity control
            'angular_velocity_gain': 1.0,  # Gain for angular velocity control
            'goal_tolerance': 0.2,  # Goal tolerance in meters
            'lookahead_distance': 0.5,  # Distance to look ahead on the path
            'enable_pure_pursuit': True,  # Use pure pursuit algorithm
            'enable_pid_control': True,  # Use PID control
            'pid_p_gain': 1.0,  # P gain for PID controller
            'pid_i_gain': 0.1,  # I gain for PID controller
            'pid_d_gain': 0.1,  # D gain for PID controller
        }],
        output='screen'
    )
    
    # Create the launch description with all nodes and argument declarations
    return LaunchDescription([
        # ================= LAUNCH ARGUMENTS =================
        # TCP connection parameters for IMU and LiDAR
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        
        # Motion planning parameters
        DeclareLaunchArgument(
            'planner_type',
            default_value='astar',
            description='Type of global planner to use (astar or dijkstra)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Whether to use simulation time'
        ),
        
        # =============== NODE INSTANTIATION ===============
        # Include the base sensor fusion launch
        base_launch,
        
        # Launch motion planning nodes
        global_planner_node,
        local_planner_node,
        path_visualization_node,
        goal_publisher_node,
        robot_controller_node,
        
        # Launch RViz for visualization with motion planning config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/motion_planning.rviz']],
            output='screen'
        ),
    ]) 