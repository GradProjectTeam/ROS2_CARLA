from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for TCP connections
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
        DeclareLaunchArgument(
            'radar_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the Radar TCP server'
        ),
        DeclareLaunchArgument(
            'radar_tcp_port',
            default_value='12351',
            description='Port number of the Radar TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the IMU TCP server'
        ),
        DeclareLaunchArgument(
            'imu_tcp_port',
            default_value='12352',
            description='Port number of the IMU TCP server'
        ),
        
        # Launch arguments for fusion and planning
        DeclareLaunchArgument(
            'lidar_weight',
            default_value='0.7',
            description='Weight for LiDAR data in fusion (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'radar_weight',
            default_value='0.3',
            description='Weight for Radar data in fusion (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'use_dynamic_weighting',
            default_value='true',
            description='Use dynamic confidence-based weighting'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='5.0',
            description='Maximum vehicle speed in m/s'
        ),
        DeclareLaunchArgument(
            'planning_frequency',
            default_value='10.0',
            description='Trajectory planning frequency in Hz'
        ),
        
        # Launch arguments for RViz
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='sensor_fusion.rviz',
            description='RViz configuration file'
        ),
        
        # Costmap parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.1',
            description='Resolution of the costmap (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='100.0',
            description='Width of the costmap in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='100.0',
            description='Height of the costmap in meters'
        ),
        DeclareLaunchArgument(
            'detection_radius',
            default_value='35.0',
            description='Detection radius for the costmap (meters)'
        ),
        DeclareLaunchArgument(
            'obstacle_inflation',
            default_value='0.3',
            description='Inflation radius for obstacles in the costmap (meters)'
        ),
        
        # Performance parameters
        DeclareLaunchArgument(
            'publish_rate',
            default_value='5.0',
            description='Rate to publish costmap (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='10.0',
            description='Rate to process costmap data (Hz)'
        ),
        DeclareLaunchArgument(
            'raycast_skip',
            default_value='3',
            description='Only raytrace every Nth point for better performance'
        ),
        DeclareLaunchArgument(
            'max_points_to_process',
            default_value='5000',
            description='Maximum number of points to process per update'
        ),
        
        # LiDAR Listener Node
        Node(
            package='sensor_fusion',
            executable='lidar_listener_clusters_2',
            name='lidar_cube_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
                'tcp_port': LaunchConfiguration('lidar_tcp_port'),
                'point_size': 1.5,
                'center_size': 3.0,
                'use_convex_hull': True,
                'use_point_markers': False,
                'use_cluster_stats': True,
                'verbose_logging': False,
                'cube_alpha': 0.3,
            }],
            output='screen'
        ),
        
        # LiDAR Costmap Creator Node
        Node(
            package='sensor_fusion',
            executable='lidar_costmap_creator',
            name='lidar_costmap_creator',
            parameters=[{
                # Map resolution and size parameters - optimized for better performance
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_width': 1000, # Reduced from 2000 for performance
                'map_height': 1000, # Reduced from 2000 for performance
                'map_width_meters': LaunchConfiguration('map_width_meters'),
                'map_height_meters': LaunchConfiguration('map_height_meters'),
                
                # Set map origin to center the vehicle
                # With 1000 cells at 0.1m resolution = 100m total width/height
                # So origin should be at -50,-50 to center the map on (0,0)
                'map_origin_x': -50.0,
                'map_origin_y': -50.0,
                
                # Vehicle offset parameters - adjust to center the car in the map
                'vehicle_x_offset': 0.0,  # Forward/backward adjustment if needed
                'vehicle_y_offset': 0.0,  # Left/right adjustment if needed
                
                # Processing parameters - optimized for better performance
                'publish_rate': LaunchConfiguration('publish_rate'),
                'process_rate': LaunchConfiguration('process_rate'),
                'raycast_skip': LaunchConfiguration('raycast_skip'),
                'max_points_to_process': LaunchConfiguration('max_points_to_process'),
                
                # Other costmap parameters
                'detection_radius': LaunchConfiguration('detection_radius'),
                'obstacle_inflation': LaunchConfiguration('obstacle_inflation'),
                'max_data_age': 2.0,
                'min_height': -3.0,
                'max_height': 4.0,
                'ground_threshold': 0.15,
                
                # Extra parameters to prioritize center area
                'center_priority_radius': 10.0,  # Ensure center area is well-represented
                'vehicle_center_weight': 1.5,    # Give more weight to central readings
            }],
            output='screen'
        ),
        
        # Radar Listener Node
        Node(
            package='sensor_fusion',
            executable='radar_listener_clusters',
            name='radar_listener',
            parameters=[{
                'tcp_ip': LaunchConfiguration('radar_tcp_ip'),
                'tcp_port': LaunchConfiguration('radar_tcp_port'),
                'point_size': 2.0,
                'verbose_logging': False,
            }],
            output='screen'
        ),
        
        # Radar Costmap Creator Node
        Node(
            package='sensor_fusion',
            executable='radar_costmap_creator',
            name='radar_costmap_creator',
            parameters=[{
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_width': 1000,
                'map_height': 1000,
                'map_width_meters': LaunchConfiguration('map_width_meters'),
                'map_height_meters': LaunchConfiguration('map_height_meters'),
                'map_origin_x': -50.0,
                'map_origin_y': -50.0,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'process_rate': LaunchConfiguration('process_rate'),
                'detection_radius': LaunchConfiguration('detection_radius'),
                'obstacle_inflation': LaunchConfiguration('obstacle_inflation'),
                'max_data_age': 2.0,
                'velocity_threshold': 0.5,  # Threshold for considering an object as moving (m/s)
                'moving_object_weight': 2.0,  # Higher weight for moving objects
            }],
            output='screen'
        ),
        
        # IMU Processor Node
        Node(
            package='sensor_fusion',
            executable='imu_processor',
            name='imu_processor',
            parameters=[{
                'imu_topic': '/imu/data_raw',
                'use_madgwick_filter': True,
                'madgwick_beta': 0.1,
                'gravity_compensation': True,
                'publish_rate': 50.0,
                'gyro_bias_correction': True,
                'accel_lpf_cutoff': 5.0,
                'velocity_reset_threshold': 0.1,
                'velocity_decay_factor': 0.99,
            }],
            output='screen'
        ),
        
        # Fusion Costmap Creator Node
        Node(
            package='sensor_fusion',
            executable='fusion_costmap_creator',
            name='fusion_costmap_creator',
            parameters=[{
                'lidar_weight': LaunchConfiguration('lidar_weight'),
                'radar_weight': LaunchConfiguration('radar_weight'),
                'dynamic_weighting': LaunchConfiguration('use_dynamic_weighting'),
                'use_imu_correction': True,
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_debug': True,
            }],
            output='screen'
        ),
        
        # Trajectory Planner Node
        Node(
            package='sensor_fusion',
            executable='trajectory_planner',
            name='trajectory_planner',
            parameters=[{
                'planning_frequency': LaunchConfiguration('planning_frequency'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_acceleration': 2.0,
                'max_deceleration': 3.0,
                'wheel_base': 2.7,
                'max_steering_angle': 0.5,
                'goal_tolerance': 1.0,
                'prediction_horizon': 3.0,
                'safety_margin': 0.5,
                'use_dynamic_window': True,
                'debug_visualization': True,
                'path_optimization': True,
            }],
            output='screen'
        ),
        
        # RViz for visualization (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/', LaunchConfiguration('rviz_config')]],
            condition=LaunchConfiguration('use_rviz'),
            output='screen'
        ),
        
        # TF static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            arguments=['0', '0', '2.0', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_radar',
            arguments=['2.5', '0', '0.5', '0', '0', '0', 'base_link', 'radar_link'],
            output='screen'
        ),
    ]) 