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

        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'fast_imu_lidar_fusion_lidar_only.launch.py')]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lidar_tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'lidar_tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'imu_tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'imu_tcp_port': LaunchConfiguration('imu_tcp_port'),

            'map_resolution': LaunchConfiguration('map_resolution'),  # Pass map resolution
            'vehicle_length': LaunchConfiguration('vehicle_length'),  # Pass vehicle dimensions
            'vehicle_width': LaunchConfiguration('vehicle_width'),
            'vehicle_height': LaunchConfiguration('vehicle_height'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),  # Pass obstacle threshold
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

            'obstacle_inflation_radius': 1.5,  # Increased from 0.5 to 1.5 for larger cells
            'path_resolution': 0.1,  # Resolution of the path in meters
            'heuristic_weight': 1.0,  # Weight for A* heuristic
            'enable_visualization': True,
            'visualize_search': True,  # Visualize the search process
            'visualize_heatmap': True,  # Visualize cost heatmap

            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),  # Use the same threshold as the mapper
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

            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),  # Use the same threshold as the mapper
        }],
        output='screen'
    )
    
    # DWA Planner Node - Dynamic Window Approach for local planning and control
    dwa_planner_node = Node(
        package='sensor_fusion',
        executable='dwa_planner_node',
        name='dwa_planner',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_path_topic': '/global_path',
            'local_path_topic': '/dwa_local_path',  # Different topic to avoid conflict
            'cmd_vel_topic': '/cmd_vel',
            'map_topic': '/realtime_map',
            'visualization_topic': '/dwa_visualization',
            'map_frame_id': 'map',
            'base_frame_id': 'base_link',
            'planning_frequency': 10.0,  # Higher frequency for DWA
            
            # Obstacle and parking parameters
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'safe_distance': 2.0,
            'min_parking_width': 2.5,
            'min_parking_length': 5.0,
            
            # DWA parameters - معلمات متوازنة للحركة
            'max_speed': 8.0,  # سرعة معقولة
            'min_speed': -2.0,  # سرعة عكسية معقولة
            'max_yaw_rate': 1.5,  # معدل دوران معقول
            'max_accel': 2.0,  # تسارع معقول
            'max_delta_yaw_rate': 0.8,  # تغيير زاوية دوران معقول
            'dt': 0.1,  # فاصل زمني مناسب
            'predict_time': 2.0,  # وقت تنبؤ مناسب
            'to_goal_cost_gain': 1.0,
            'speed_cost_gain': 0.3,  # وزن معقول لتكلفة السرعة
            'obstacle_cost_gain': 1.0,
            'path_following_gain': 1.0,
            'lookahead_distance': 3.0,

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

            'local_path_topic': '/dwa_local_path',  # Use DWA's local path
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
    

    # robot_controller_node = Node(
    #     package='sensor_fusion',
    #     executable='robot_controller_node',
    #     name='robot_controller',
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'cmd_vel_topic': '/cmd_vel',
    #         'carla_control_topic': '/carla/ego_vehicle/vehicle_control_cmd',  # CARLA control topic
    #         'local_path_topic': '/local_path',
    #         'map_frame_id': 'map',
    #         'base_frame_id': 'base_link',
    #         'control_frequency': 10.0,  # Control at 10Hz
    #         'max_linear_velocity': 0.5,  # Maximum linear velocity in m/s
    #         'max_angular_velocity': 0.5,  # Maximum angular velocity in rad/s
    #         'linear_velocity_gain': 1.0,  # Gain for linear velocity control
    #         'angular_velocity_gain': 1.0,  # Gain for angular velocity control
    #         'goal_tolerance': 0.2,  # Goal tolerance in meters
    #         'lookahead_distance': 0.5,  # Distance to look ahead on the path
    #         'enable_pure_pursuit': True,  # Use pure pursuit algorithm
    #         'enable_pid_control': True,  # Use PID control
    #         'pid_p_gain': 1.0,  # P gain for PID controller
    #         'pid_i_gain': 0.1,  # I gain for PID controller
    #         'pid_d_gain': 0.1,  # D gain for PID controller
            
    #         # CARLA specific parameters
    #         'enable_carla_control': LaunchConfiguration('enable_carla_control'),
    #         'max_throttle': 1.0,  # Maximum throttle value (0-1)
    #         'max_brake': 1.0,  # Maximum brake value (0-1)
    #         'max_steer': 1.0,  # Maximum steering value (-1 to 1)
    #         'throttle_gain': 1.0,  # Gain for throttle conversion
    #         'brake_gain': 1.0,  # Gain for brake conversion
    #         'steer_gain': 1.0,  # Gain for steering conversion
    #         'steer_ratio': 1.0,  # Steering ratio for converting angular velocity to steering angle
    #     }],
    #     output='screen'
    # )
    
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

        # Map parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.8',  # Larger cell size (was 0.4)
            description='Resolution of the map in meters per cell'
        ),
        
        # Obstacle threshold parameter
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='70',
            description='Threshold for marking cells as obstacles (0-100)'
        ),
        
        # Vehicle dimensions
        DeclareLaunchArgument(
            'vehicle_length',
            default_value='5.0',
            description='Length of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_width',
            default_value='2.5',
            description='Width of vehicle (meters)'
        ),
        DeclareLaunchArgument(
            'vehicle_height',
            default_value='2.2',
            description='Height of vehicle (meters)'
        ),
        
        # CARLA control parameters
        DeclareLaunchArgument(
            'enable_carla_control',
            default_value='true',
            description='Whether to enable CARLA control conversion'
        ),
        
        # =============== NODE INSTANTIATION ===============
        # Include the base sensor fusion launch
        base_launch,
        
        # Launch motion planning nodes
        global_planner_node,
        local_planner_node,

        dwa_planner_node,  # Add DWA planner node
        path_visualization_node,
        goal_publisher_node,
        # robot_controller_node,
        
        # Launch RViz for visualization with motion planning config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/motion_planning.rviz']],
            output='screen'
        ),
    ]) 