#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Install required dependencies if not already installed
    install_deps_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'if ! dpkg -s libtiff6 &> /dev/null; then sudo apt-get update && sudo apt-get install -y libtiff6; fi'],
        name='install_dependencies',
        output='screen'
    )
    
    # Create transform publisher nodes for TF tree with higher frequency
    # The world_to_map transform is essential for the TF tree
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    # Note: The map_to_base_link transform is now handled by the dynamic_tf_publisher
    # to account for the vehicle's rotation based on IMU data
    
    # Base to IMU transform
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    # IMU to LiDAR transform
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    # Create the IMU euler visualization node with simulation support
    imu_euler_visualizer_node = Node(
        package='sensor_fusion',
        executable='imu_euler_visualizer',
        name='imu_euler_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('imu_tcp_ip'),
            'tcp_port': LaunchConfiguration('imu_tcp_port'),
            'reconnect_interval': 2.0,
            'frame_id': 'imu_link',
            'world_frame_id': 'world',
            'filter_window_size': 3,
            'verbose_logging': True,
            'retry_count': 10,
            'simulation_mode': LaunchConfiguration('simulation_mode'),  # Enable simulation
            'sim_publish_rate': 50.0,  # Hz
        }],
        output='screen'
    )
    
    # Add the LiDAR listener with simulation support
    lidar_listener_node = Node(
        package='sensor_fusion',
        executable='lidar_listener_clusters_2',
        name='lidar_listener',
        parameters=[{
            'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
            'tcp_port': LaunchConfiguration('lidar_tcp_port'),
            'reconnect_interval': 2.0,
            'point_size': 1.5,
            'center_size': 3.0,
            'use_convex_hull': True,
            'use_point_markers': False,
            'use_cluster_stats': True,
            'verbose_logging': True,
            'retry_count': 10,
            'cube_alpha': 0.3,
            'filter_vehicle_points': True,
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
            'simulation_mode': LaunchConfiguration('simulation_mode'),  # Enable simulation
            'sim_publish_rate': 20.0,  # Hz
        }],
        output='screen'
    )
    
    # Create the test_navigation node with IMU integration - focus on core parameters that actually exist
    test_navigation_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='navigation_tester',
        parameters=[{
            # Basic parameters (these are the ones that definitely exist)
            'imu_topic': LaunchConfiguration('imu_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'use_filtered_yaw': True,
            'yaw_filter_size': 3,  # Smaller value for more responsive updates
            'yaw_weight': 1.0,  # Maximum weight to IMU data
            'goal_distance': LaunchConfiguration('goal_distance'),
            'yaw_offset': LaunchConfiguration('yaw_offset'),
            'orientation_debug': True,
            'path_verification': True,
            # Critical - tell the test_navigation node to publish TF with the correct orientation
            'publish_tf': True,
        }],
        output='screen'
    )
    
    # Add a static map_to_base_link transform as fallback - USING IDENTITY ROTATION
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 10.0}],
    )
    
    # Instead of a bash command, create a Python node specifically for the navigation demo
    # that will publish the TF transform based on IMU data
    orientation_tf_node = Node(
        package='sensor_fusion',
        executable='test_navigation',  # Reuse the test_navigation node 
        name='tf_orientation_handler',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_filtered_yaw': True,
            'yaw_filter_size': 3,
            'yaw_weight': 1.0,
            'yaw_offset': LaunchConfiguration('yaw_offset'),
            'publish_tf': True,  # Tell it to publish TF
            'publish_goal': False,  # Don't publish goals, just handle TF
            'publish_tf_rate': 50.0,  # High rate for responsive transforms
        }],
        remappings=[
            # Rename topics to avoid conflicts with main test_navigation node
            ('/cmd_vel', '/tf_handler/cmd_vel'),
            ('/goal_pose', '/tf_handler/goal_pose'),
        ],
        output='screen'
    )
    
    # Create the real-time mapper node with improved reliability
    lidar_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_realtime_mapper',
        parameters=[{
            # Map parameters
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'center_on_vehicle': True,
            
            # Processing parameters - higher rates for faster updates
            'publish_rate': 15.0,
            'process_rate': 30.0,
            
            # Bayesian update weights - tuned for better map quality
            'hit_weight': 0.9,
            'miss_weight': 0.4,
            'prior_weight': 0.5,
            
            # Other parameters - optimized for performance
            'decay_rate': 0.98,
            'temporal_memory': 0.3,
            'enable_map_reset': True,
            'map_reset_interval': 15.0,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # Added parameters for better map quality
            'use_binary_map': True,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'max_points_to_process': 7000,
            
            # TF parameters optimized for faster lookups
            'tf_buffer_duration': 5.0,
            'tf_timeout': 0.5,
            'wait_for_transform': True,
            'transform_tolerance': 0.25,
            
            # Connection robustness
            'retry_count': 5,
            'verbose_logging': True,
            
            # Dynamic orientation parameters
            'use_imu_orientation': True,  # Use IMU for orientation
            'update_on_rotation': True,  # Update the map when the vehicle rotates
        }],
        output='screen'
    )
    
    # Hybrid A* path planner - focus on core parameters
    hybrid_astar_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[{
            # Grid and vehicle parameters
            'grid_size': LaunchConfiguration('grid_size'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            
            # Planning parameters 
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'publish_rate': 10.0,  # Higher rate for more responsive planning
            
            # Advanced A* parameters
            'max_iterations': LaunchConfiguration('max_iterations'),
            'motion_resolution': 5,  # FIXED: Integer value
            'angle_resolution': 18,  # FIXED: Integer value
            'heuristic_weight': 1.0,  # Lower value for more optimal paths
            
            # Topics and frames
            'map_topic': LaunchConfiguration('map_topic'),
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # IMU topic to get rotation updates
            'imu_topic': LaunchConfiguration('imu_topic'),
            
            # Critical orientation parameters - try every possible name they might have
            'use_current_orientation': True,
            'use_vehicle_orientation': True,
            'use_imu_orientation': True,
            'use_tf_orientation': True,
            'track_orientation': True,
            'adjust_for_orientation': True,
            'orientation_from_imu': True,
            'orientation_from_tf': True,
        }],
        output='screen'
    )
    
    # Path smoother for A* output - enhanced for smoother paths
    path_smoother_node = Node(
        package='sensor_fusion',
        executable='frenet_path_smoother',
        name='frenet_path_smoother',
        parameters=[{
            'num_points': 150,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'input_path_topic': '/hybrid_astar_path',
            'output_path_topic': '/smooth_path',
        }],
        output='screen'
    )
    
    # Add the MPC Planner with rotation awareness
    mpc_planner_node = Node(
        package='sensor_fusion',
        executable='mpc_planner',
        name='mpc_planner',
        parameters=[{
            'horizon': 10,
            'dt': 0.1,
            'wheelbase': 2.5,
            'publish_rate': 10.0,
            'map_topic': '/realtime_map',
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'use_imu_heading': True,  # Use IMU for vehicle heading
            'imu_topic': LaunchConfiguration('imu_topic'),
            'replan_on_rotation': True,  # Replan when vehicle rotates
            'rotation_threshold': 0.1,  # Radians, threshold to detect significant rotation
        }],
        output='screen'
    )
    
    # Add the DWA Local Planner with rotation awareness
    dwa_planner_node = Node(
        package='sensor_fusion',
        executable='dwa_local_planner',
        name='dwa_local_planner',
        parameters=[{
            'min_linear_velocity': 0.0,
            'max_linear_velocity': 1.0,
            'min_angular_velocity': -0.8,
            'max_angular_velocity': 0.8,
            'linear_acceleration': 0.5,
            'angular_acceleration': 1.0,
            'dt': 0.1,
            'predict_horizon': 1.0,
            'obstacle_weight': 1.0,
            'goal_weight': 1.0,
            'heading_weight': 0.8,
            'velocity_weight': 0.2,
            'publish_rate': 10.0,
            'map_topic': '/realtime_map',
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'path_topic': '/smooth_path',
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'safety_radius': 0.3,
            'use_current_heading': True,  # Use current vehicle heading for planning
            'imu_topic': LaunchConfiguration('imu_topic'),
            'heading_update_rate': 20.0,  # Hz, rate to update vehicle heading
        }],
        output='screen'
    )
    
    # Verify TF tree connections
    verify_tf_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'ros2 run tf2_tools view_frames || echo "TF tools not available to check frame connections"'],
        name='verify_tf_tree',
        output='screen'
    )
    
    # Create a test connection node to check if sensors are available
    test_connection_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'echo "Testing TCP connections..."; \
             nc -zv -w 1 $(ros2 param get /lidar_listener tcp_ip) $(ros2 param get /lidar_listener tcp_port) || echo "LiDAR connection failed"; \
             nc -zv -w 1 $(ros2 param get /imu_euler_visualizer tcp_ip) $(ros2 param get /imu_euler_visualizer tcp_port) || echo "IMU connection failed"'],
        name='test_connections',
        output='screen'
    )
    
    # Delay the start of the mapper (to ensure transforms and sensors are available)
    delayed_mapper = TimerAction(
        period=3.0,  # Give more time for sensors to connect
        actions=[lidar_mapper_node]
    )
    
    # Delay the start of the planning nodes to ensure mapping is established
    delayed_planning = TimerAction(
        period=6.0,  # Give more time for the map to be created
        actions=[hybrid_astar_node, path_smoother_node, mpc_planner_node, dwa_planner_node]
    )
    
    # Delay the verification to ensure TF tree is established
    delayed_verify_tf = TimerAction(
        period=8.0,  # Run after TF tree should be established
        actions=[verify_tf_cmd]
    )
    
    # Delay the connection test to ensure parameters are loaded
    delayed_connection_test = TimerAction(
        period=10.0,  # Run after all nodes are started
        actions=[test_connection_cmd]
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'navigation.rviz')]],
        output='screen'
    )
    
    return LaunchDescription([
        # Install dependencies first
        install_deps_cmd,
        
        # Connection parameters for IMU and LiDAR
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='0.0.0.0',  # Listen on all interfaces
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='0.0.0.0',  # Listen on all interfaces 
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        
        # Enable simulation mode by default to work without real sensors
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Enable simulation mode to run without real sensors'
        ),
        
        # IMU and map topics
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data',
            description='Topic for IMU data'
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/realtime_map',
            description='Topic for map data'
        ),
        
        # IMU processing parameters
        DeclareLaunchArgument(
            'use_filtered_yaw',
            default_value='true',
            description='Whether to use filtered IMU yaw (smoothed)'
        ),
        DeclareLaunchArgument(
            'yaw_filter_size',
            default_value='5',
            description='Size of IMU yaw filter buffer'
        ),
        DeclareLaunchArgument(
            'yaw_weight',
            default_value='0.8',
            description='Weight of IMU yaw data in fusion'
        ),
        DeclareLaunchArgument(
            'yaw_offset',
            default_value='1.5',  # Approximately 85 degrees
            description='Offset to correct IMU yaw orientation in radians'
        ),
        
        # Navigation parameters
        DeclareLaunchArgument(
            'goal_distance',
            default_value='5.0',
            description='Distance to goal pose in meters'
        ),
        
        # Map parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.1',
            description='Resolution of the map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='50.0',
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='50.0',
            description='Height of the map in meters'
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='65',
            description='Threshold for marking cells as obstacles (0-100)'
        ),
        
        # A* parameters
        DeclareLaunchArgument(
            'grid_size',
            default_value='0.3',
            description='Grid size for A* planning (meters)'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='2.5',
            description='Vehicle wheelbase (meters)'
        ),
        DeclareLaunchArgument(
            'max_iterations',
            default_value='5000',
            description='Maximum A* iterations before giving up'
        ),
        
        # Nodes and actions - TF tree must start first for proper connections
        world_to_map_node,
        base_to_imu_node,
        imu_to_lidar_node,
        imu_euler_visualizer_node,
        lidar_listener_node,
        test_navigation_node,
        map_to_base_link_node,
        orientation_tf_node,
        delayed_mapper,
        delayed_planning,
        delayed_verify_tf,
        rviz_node,
        delayed_connection_test,
    ]) 