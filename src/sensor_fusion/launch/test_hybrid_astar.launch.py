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
    
    # Run verification script
    verify_tf_script = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 3 && cd /home/mostafa/GP/ROS2 && source install/setup.bash && ros2 run tf2_tools view_frames'],
        name='verify_tf_script',
        output='screen'
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
            'publish_goal_with_orientation': True,
            'update_goal_on_rotation': True,
            'broadcast_orientation_tf': True,
            'rotation_update_threshold': 0.05,
            'tf_publish_rate': 100.0,
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
            'yaw_filter_size': 5,
            'yaw_weight': 1.0,
            'yaw_offset': LaunchConfiguration('yaw_offset'),
            'publish_tf': True,
            'publish_goal': False,
            'publish_tf_rate': 100.0,
            'override_static_transform': True,
            'orientation_only': True,
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
            'rotate_map_with_vehicle': True,
            'update_map_on_rotation': True,
            'publish_rotated_map': True,
        }],
        output='screen'
    )
    
    # HYBRID A* PLANNER
    hybrid_astar_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[{
            'grid_size': LaunchConfiguration('grid_size'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
            'publish_rate': 20.0,
            'max_iterations': LaunchConfiguration('max_iterations'),
            'motion_resolution': 10,
            'angle_resolution': 36,
            'heuristic_weight': 1.5,
            'map_topic': LaunchConfiguration('map_topic'),
            'vehicle_frame_id': 'base_link',
            'map_frame_id': 'map',
            'imu_topic': LaunchConfiguration('imu_topic'),
            'use_current_orientation': True,
            'rotation_weight': 0.8,
            'path_smoothing': True,
            'heading_bias': 0.7,
            'replan_on_rotation': True,
            'rotation_threshold': 0.15,
            'obstacle_inflation': 0.4,
            'consider_turning_radius': True,
            'lookahead_distance': 2.0,
            'always_recalculate_path': True,
            'transform_path_to_vehicle_frame': True,
            'use_global_path_in_local_frame': True,
            'use_tf_for_goal_orientation': True,
            'rotation_update_threshold': 0.05,
            'tf_lookup_timeout': 0.5,
        }],
        output='screen'
    )
    
    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'hybrid_astar_test.rviz')]],
        output='screen'
    )
    
    # Delayed starts to ensure proper initialization
    delayed_mapper = TimerAction(
        period=5.0,
        actions=[lidar_mapper_node]
    )
    
    delayed_planner = TimerAction(
        period=8.0,
        actions=[hybrid_astar_node]
    )
    
    # Sequential launch description
    return LaunchDescription([
        # Install dependencies
        install_deps_cmd,
        
        # Launch arguments
        DeclareLaunchArgument('imu_tcp_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('imu_tcp_port', default_value='12345'),
        DeclareLaunchArgument('lidar_tcp_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('lidar_tcp_port', default_value='12350'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('map_topic', default_value='/realtime_map'),
        DeclareLaunchArgument('yaw_offset', default_value='1.5'),
        DeclareLaunchArgument('goal_distance', default_value='5.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('map_width_meters', default_value='60.0'),
        DeclareLaunchArgument('map_height_meters', default_value='60.0'),
        DeclareLaunchArgument('obstacle_threshold', default_value='60'),
        DeclareLaunchArgument('grid_size', default_value='0.2'),
        DeclareLaunchArgument('wheelbase', default_value='2.5'),
        DeclareLaunchArgument('max_iterations', default_value='8000'),
        
        # Essential TF transforms first
        world_to_map_node,
        map_to_base_link_node,
        base_to_imu_node,
        imu_to_lidar_node,
        
        # Verify TF setup
        verify_tf_script,
        
        # Start nodes with a delay
        TimerAction(
            period=2.0,
            actions=[
                imu_euler_visualizer_node,
                orientation_tf_node,
                lidar_listener_node,
                test_navigation_node,
            ]
        ),
        
        # Mapper and planner
        delayed_mapper,
        delayed_planner,
        
        # Start RViz last
        TimerAction(
            period=10.0,
            actions=[rviz_node]
        ),
    ]) 