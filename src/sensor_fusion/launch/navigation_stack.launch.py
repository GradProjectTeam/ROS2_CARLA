#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Create transform publisher nodes with higher publish frequency for faster updates
    # These are critical for correct orientation, so ensure they have correct values
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}], # Higher frequency for transforms
    )
    
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}], # Higher frequency for transforms
    )
    
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}], # Higher frequency for transforms
    )
    
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}], # Higher frequency for transforms
    )
    
    # Create the fused mapper node - with shorter delay for TF to be established
    fast_imu_lidar_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_fast_fusion_mapper',
        parameters=[{
            # Map resolution and size parameters - smaller map for faster processing
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'center_on_vehicle': LaunchConfiguration('center_on_vehicle'),
            
            # Processing parameters - higher rates for faster updates
            'publish_rate': LaunchConfiguration('publish_rate'),  # Configurable publish rate
            'process_rate': LaunchConfiguration('process_rate'),  # Configurable process rate 
            
            # Bayesian update weights - optimized for faster updates
            'hit_weight': LaunchConfiguration('hit_weight'),      # Configurable hit weight
            'miss_weight': LaunchConfiguration('miss_weight'),    # Configurable miss weight
            'prior_weight': 0.5,                                 # Start with unknown/gray
            'count_threshold': 0.15,                             # Lower threshold for faster binary mapping
            
            # Fast mapping parameters for more aggressive data removal
            'decay_rate': LaunchConfiguration('decay_rate'),      # Configurable decay rate
            'update_threshold': 0.0005,                          # Lower threshold for faster updates
            'temporal_memory': LaunchConfiguration('temporal_memory'),  # Configurable temporal memory
            'enable_map_reset': LaunchConfiguration('enable_map_reset'),  # Enable periodic map reset
            'map_reset_interval': LaunchConfiguration('map_reset_interval'),  # Reset interval
            'use_binary_map': LaunchConfiguration('use_binary_map'),  # Force binary map output
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),  # Threshold for obstacles
            
            # Map saving parameters - faster save intervals
            'enable_map_save': LaunchConfiguration('enable_map_save'),  # Enable saving map 
            'map_save_dir': LaunchConfiguration('map_save_dir'),  # Directory to save maps
            'enable_auto_save': LaunchConfiguration('enable_auto_save'),  # Enable auto save
            'auto_save_interval': LaunchConfiguration('auto_save_interval'),  # Auto save interval
            'map_base_filename': LaunchConfiguration('map_base_filename'),  # Base filename for saved maps
            'save_format': LaunchConfiguration('save_format'),  # Format to save maps in
            
            # Other parameters - tuned for speed
            'ground_threshold': 0.15,                            # Lower threshold for faster ground detection
            'min_height': -0.3,
            'max_height': 2.0,
            'raycast_skip': 2,                                   # Skip more points for faster processing
            'max_points_to_process': LaunchConfiguration('max_points_to_process'),  # Configurable max points
            'use_cluster_data': True,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            
            # TF parameters optimized for faster lookups
            'tf_buffer_duration': 5.0,     # Shorter buffer for less memory usage
            'tf_timeout': 0.5,             # Shorter timeout for faster failures and retries
            'use_sim_time': False,         # Ensure real clock time is used
            'wait_for_transform': True,    # Wait for transform to be available
            'transform_tolerance': 0.25,   # Lower tolerance for faster lookups
        }],
        output='screen'
    )
    
    # Delay the start of the mapper - reduced delay for faster startup
    delayed_mapper = TimerAction(
        period=1.0,  # 1 second delay (reduced from 2 seconds)
        actions=[fast_imu_lidar_mapper_node]
    )
    
    # NEW PLANNING NODES
    
    # Hybrid A* path planner
    hybrid_astar_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[{
            'grid_size': 0.5,
            'wheelbase': 2.5,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'publish_rate': 5.0,  # Lower rate for planning (computationally intensive)
            'map_topic': '/realtime_map',
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
        }],
        output='screen'
    )
    
    # Path smoother using Frenet coordinates
    path_smoother_node = Node(
        package='sensor_fusion',
        executable='frenet_path_smoother',
        name='frenet_path_smoother',
        parameters=[{
            'num_points': 100,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'input_path_topic': '/hybrid_astar_path',
            'output_path_topic': '/smooth_path',
        }],
        output='screen'
    )
    
    # MPC planner for trajectory generation
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
        }],
        output='screen'
    )
    
    # DWA local planner for obstacle avoidance
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
        }],
        output='screen'
    )
    
    # NEW: Add test navigation node for verifying orientation and path
    test_navigation_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='navigation_tester',
        parameters=[{
            'imu_topic': '/imu/data',
            'map_topic': '/realtime_map',
            'use_filtered_yaw': True,
            'yaw_filter_size': 5,
            'yaw_weight': 0.8,
            'goal_distance': 5.0,
            # Set the yaw offset in radians (approx 85.9 degrees)
            'yaw_offset': LaunchConfiguration('yaw_offset'),  # Make yaw offset configurable
            'orientation_debug': True,  # Enable orientation debugging
            'path_verification': True,  # Enable path verification
        }],
        output='screen'
    )
    
    # Install required dependencies if not already installed
    install_deps_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'if ! dpkg -s libtiff6 &> /dev/null; then sudo apt-get update && sudo apt-get install -y libtiff6; fi'],
        name='install_dependencies',
        output='screen'
    )
    
    # Delay the start of the planning nodes to ensure mapping is established first
    delayed_planning = TimerAction(
        period=3.0,  # 3 second delay to allow mapping to stabilize
        actions=[hybrid_astar_node, path_smoother_node, mpc_planner_node, dwa_planner_node, 
                test_navigation_node]
    )
    
    # Set up visualizer for RVIZ
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
        
        # Map Parameters - more optimized for speed
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.15',  # Slightly lower resolution for faster processing
            description='Resolution of the fast map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='30.0',  # Smaller map area for faster processing
            description='Width of the map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='30.0',  # Smaller map area for faster processing
            description='Height of the map in meters'
        ),
        DeclareLaunchArgument(
            'center_on_vehicle',
            default_value='true',
            description='Keep map centered on vehicle position'
        ),
        
        # Performance parameters - higher rates for best performance
        DeclareLaunchArgument(
            'publish_rate',
            default_value='20.0',  # Higher publish rate for smoother visualization
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='40.0',  # Higher process rate for faster updates
            description='Rate to process map data (Hz)'
        ),
        
        # Bayesian update parameters - tuned for faster updates
        DeclareLaunchArgument(
            'hit_weight',
            default_value='1.0',  # Maximum confidence in hits
            description='Weight for obstacle hits in Bayesian update'
        ),
        DeclareLaunchArgument(
            'miss_weight',
            default_value='0.4',  # Stronger clearing for faster free space updates
            description='Weight for misses (free space) in Bayesian update'
        ),
        
        # Fast mapping specific parameters - more aggressive for speed
        DeclareLaunchArgument(
            'decay_rate',
            default_value='0.98',  # More aggressive decay
            description='Rate at which old map data decays (0-1, higher = faster decay)'
        ),
        DeclareLaunchArgument(
            'temporal_memory',
            default_value='0.2',  # Even shorter memory of points
            description='Duration to remember points (seconds)'
        ),
        DeclareLaunchArgument(
            'enable_map_reset',
            default_value='true',  # Enable map reset
            description='Enable periodic map reset to clear old data'
        ),
        DeclareLaunchArgument(
            'map_reset_interval',
            default_value='10.0',  # Reset map more frequently (was 15s)
            description='Interval for map reset (seconds)'
        ),
        DeclareLaunchArgument(
            'use_binary_map',
            default_value='true',  # Use binary map for clearer visualization
            description='Use binary (black/white) map output'
        ),
        DeclareLaunchArgument(
            'obstacle_threshold',
            default_value='0.65',  # Slightly lower threshold for faster obstacle marking
            description='Threshold for marking cells as obstacles (0-1)'
        ),
        DeclareLaunchArgument(
            'max_points_to_process',
            default_value='6000',  # Process fewer points for faster updates
            description='Maximum number of points to process per update'
        ),
        
        # Map saving parameters - faster save intervals
        DeclareLaunchArgument(
            'enable_map_save',
            default_value='true',  # Enable map saving
            description='Enable saving map to disk'
        ),
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/mostafa/GP/ROS2/maps',  # Directory to save maps
            description='Directory to save maps'
        ),
        DeclareLaunchArgument(
            'enable_auto_save',
            default_value='true',  # Enable auto save
            description='Enable automatic saving of map at intervals'
        ),
        DeclareLaunchArgument(
            'auto_save_interval',
            default_value='5.0',  # Save more frequently (was 10s)
            description='Interval for auto-saving the map (seconds)'
        ),
        DeclareLaunchArgument(
            'map_base_filename',
            default_value='navigation_map',  # Base filename for saved maps
            description='Base filename for saved maps'
        ),
        DeclareLaunchArgument(
            'save_format',
            default_value='png',  # Format to save maps in
            description='Format to save maps in (png, pgm, or both)'
        ),
        
        # IMU orientation parameter
        DeclareLaunchArgument(
            'yaw_offset',
            default_value='1.5',  # Default to ~85 degrees (can be adjusted)
            description='Offset to correct IMU yaw orientation in radians'
        ),
        
        # Nodes and actions
        install_deps_cmd,
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        delayed_mapper,
        delayed_planning,
        rviz_node,
    ]) 