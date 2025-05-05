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
    
    # TF Tree Setup (essential for path planning)
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    # Add a direct map to base_link transform to fix the TF tree error
    map_to_base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    base_to_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_imu',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    imu_to_lidar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_lidar',
        arguments=['0', '0', '0.2', '0', '0', '0', 'imu_link', 'lidar_link'],
        parameters=[{'use_sim_time': False, 'publish_frequency': 100.0}],
    )
    
    # IMU Visualization with simulation mode
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
            'simulation_mode': LaunchConfiguration('simulation_mode'),
            'sim_publish_rate': 50.0,  # Hz
        }],
        output='screen'
    )
    
    # LiDAR processing
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
            'simulation_mode': LaunchConfiguration('simulation_mode'),
            'sim_publish_rate': 20.0,
            'frame_id': 'lidar_link',
            'use_tf_transform': True,
        }],
        output='screen'
    )
    
    # Test navigation node for orientation handling
    test_navigation_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='navigation_tester',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'use_filtered_yaw': True,
            'yaw_filter_size': 3,
            'yaw_weight': 1.0,
            'goal_distance': LaunchConfiguration('goal_distance'),
            'yaw_offset': LaunchConfiguration('yaw_offset'),
            'orientation_debug': True,
            'path_verification': True,
            'publish_tf': True,
            'publish_goal_with_orientation': True,  # Publish goal with current orientation
            'update_goal_on_rotation': True,        # Update goal when vehicle rotates
            'broadcast_orientation_tf': True,       # Explicitly broadcast orientation TF
            'rotation_update_threshold': 0.05,      # More sensitive to rotations
            'tf_publish_rate': 100.0,               # Higher rate for TF publishing
        }],
        output='screen'
    )
    
    # Dynamic TF based on IMU orientation
    orientation_tf_node = Node(
        package='sensor_fusion',
        executable='test_navigation',
        name='tf_orientation_handler',
        parameters=[{
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_filtered_yaw': True,
            'yaw_filter_size': 5,  # Increased for smoother filtering
            'yaw_weight': 1.0,
            'yaw_offset': LaunchConfiguration('yaw_offset'),
            'publish_tf': True,
            'publish_goal': False,
            'publish_tf_rate': 100.0,  # Higher rate for more frequent updates
            
            # Orientation handling parameters
            'override_static_transform': True,
            'priority_level': 100,
            'smooth_transitions': True,
            'interpolate_transforms': True,
            'orientation_only': True,  # CRITICAL: Only update orientation, not position
            'force_tf_update': True,   # Force updates to the TF tree
            'clear_tf_cache': True,    # Prevent caching of transforms
        }],
        remappings=[
            ('/cmd_vel', '/tf_handler/cmd_vel'),
            ('/goal_pose', '/tf_handler/goal_pose'),
        ],
        output='screen'
    )
    
    # Mapper node
    lidar_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='lidar_realtime_mapper',
        parameters=[{
            'map_resolution': LaunchConfiguration('map_resolution'),
            'map_width_meters': LaunchConfiguration('map_width_meters'),
            'map_height_meters': LaunchConfiguration('map_height_meters'),
            'center_on_vehicle': True,
            'publish_rate': 15.0,
            'process_rate': 30.0,
            'hit_weight': 0.9,
            'miss_weight': 0.4,
            'prior_weight': 0.5,
            'decay_rate': 0.98,
            'temporal_memory': 0.3,
            'enable_map_reset': True,
            'map_reset_interval': 15.0,
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'use_binary_map': True,
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'max_points_to_process': 7000,
            'use_imu_orientation': True,
            
            # Critical orientation handling
            'rotate_map_with_vehicle': True,    # CRITICAL: Rotate map with vehicle orientation
            'update_map_on_rotation': True,     # Update map when rotation occurs
            'publish_rotated_map': True,        # Publish map in rotated frame
        }],
        output='screen'
    )
    
    # Simplified path generator for testing the smoother - improved for orientation
    path_generator_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='simple_path_generator',
        parameters=[{
            'grid_size': 0.5,
            'wheelbase': LaunchConfiguration('wheelbase'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'publish_rate': 5.0,
            'max_iterations': 2000,
            'motion_resolution': 5,
            'angle_resolution': 36,  # Increased from 18 to 36
            'heuristic_weight': 1.5,
            'map_topic': LaunchConfiguration('map_topic'),
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'imu_topic': LaunchConfiguration('imu_topic'),
            
            # Critical orientation parameters
            'use_current_orientation': True,  # Use current vehicle orientation
            'rotation_weight': 0.8,  # Weight for cost of changing orientation
            'replan_on_rotation': True, # Replan when vehicle rotates
            'rotation_threshold': 0.05, # Lower threshold for more sensitive replanning
            'transform_path_to_vehicle_frame': True, # Transform path to vehicle frame
        }],
        output='screen'
    )
    
    # FRENET PATH SMOOTHER with improved orientation handling
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
            'smoothing_factor': 0.85,
            'use_orientation': True,
            'path_visualization': True,
            'imu_topic': LaunchConfiguration('imu_topic'),
            
            # Critical orientation parameters
            'update_path_on_rotation': True,  # CRITICAL: Update path when vehicle rotates
            'rotation_threshold': 0.05,       # Lower threshold for more sensitivity
            'transform_path_to_vehicle_frame': True, # Transform path to vehicle frame
            'heading_bias': 0.7,             # Bias smoothing towards current heading
            'output_in_vehicle_frame': True, # Output path in vehicle frame
            'continuous_path_update': True,  # Continuously update path
            'use_tf_for_orientation': True,  # Use TF for orientation data
            'path_orientation_weight': 2.0,  # Higher weight for orientation in smoothing
        }],
        output='screen'
    )
    
    # Delayed starts to ensure proper initialization
    delayed_mapper = TimerAction(
        period=5.0,
        actions=[lidar_mapper_node]
    )
    
    delayed_path_generator = TimerAction(
        period=8.0,
        actions=[path_generator_node]
    )
    
    delayed_smoother = TimerAction(
        period=10.0,  # Start smoother after path generator
        actions=[path_smoother_node]
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'frenet_smoother_test.rviz')]],
        output='screen'
    )
    
    # Add diagnostic tools to verify transforms are working
    verify_tf_cmd = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 15 && ros2 run tf2_ros tf2_echo map base_link'],
        name='verify_tf_tree',
        output='screen'
    )
    
    return LaunchDescription([
        # Install dependencies
        install_deps_cmd,
        
        # Connection parameters
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='0.0.0.0',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12345',
            description='Port number of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_ip',
            default_value='0.0.0.0',
            description='IP address of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'lidar_tcp_port',
            default_value='12350',
            description='Port number of the LiDAR TCP server'
        ),
        DeclareLaunchArgument(
            'simulation_mode',
            default_value='true',
            description='Enable simulation mode to run without real sensors'
        ),
        
        # Topics
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
        
        # IMU parameters
        DeclareLaunchArgument(
            'yaw_offset',
            default_value='1.5',
            description='Offset to correct IMU yaw orientation in radians'
        ),
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
        
        # Path parameters
        DeclareLaunchArgument(
            'wheelbase',
            default_value='2.5',
            description='Vehicle wheelbase (meters)'
        ),
        
        # Nodes
        world_to_map_node,
        map_to_base_link_node,  # Add the static transform from map to base_link
        base_to_imu_node,
        imu_to_lidar_node,
        imu_euler_visualizer_node,
        lidar_listener_node,
        test_navigation_node,
        orientation_tf_node,
        delayed_mapper,
        delayed_path_generator,
        delayed_smoother,
        rviz_node,
        verify_tf_cmd,  # Add a verification command to check if transforms are working
    ])