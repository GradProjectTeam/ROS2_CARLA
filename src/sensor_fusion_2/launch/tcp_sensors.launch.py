from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='IP address for TCP connections'
    )
    
    # Enable/disable flags for each sensor
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='false',
        description='Enable IMU sensor'
    )
    
    enable_radar_arg = DeclareLaunchArgument(
        'enable_radar',
        default_value='false',
        description='Enable radar sensor'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable LiDAR sensor'
    )
    
    # Get the launch configurations
    tcp_ip = LaunchConfiguration('tcp_ip')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_radar = LaunchConfiguration('enable_radar')
    enable_lidar = LaunchConfiguration('enable_lidar')
    
    # Create IMU TCP node
    imu_node = Node(
        package='sensor_fusion_2',
        executable='imu_tcp_node',
        name='imu_tcp_node',
        output='screen',
        parameters=[{
            'tcp_ip': tcp_ip,
            'tcp_port': 12345,
            'frame_id': 'imu_link',
            'filter_window_size': 5
        }],
        condition=IfCondition(enable_imu)
    )
    
    # Create radar TCP node
    radar_node = Node(
        package='sensor_fusion_2',
        executable='radar_tcp_node',
        name='radar_tcp_node',
        output='screen',
        parameters=[{
            'tcp_ip': tcp_ip,
            'tcp_port': 12348,
            'frame_id': 'radar_link',
        }],
        condition=IfCondition(enable_radar)
    )
    
    # Create LiDAR cluster listener node for pre-processed clusters
    lidar_node = Node(
        package='sensor_fusion_2',
        executable='lidar_listener_clusters_2',
        name='lidar_cluster_listener',
        output='screen',
        parameters=[{
            'tcp_ip': tcp_ip,
            'tcp_port': 12350,
            'frame_id': 'lidar_link',
            'point_size': 2.0,
            'center_size': 3.0,
            'use_convex_hull': True,
            'use_point_markers': True,
            'use_cluster_stats': True,
            'verbose_logging': False,
            'cube_alpha': 0.3,
            
            # CARLA LiDAR Configuration - MOUNTED AT x=1.5, z=2.0
            'filter_vehicle_points': True,             # Enable filtering
            'vehicle_length': 4.8,                     # Slightly larger than actual vehicle
            'vehicle_width': 2.2,                      # Slightly larger than actual vehicle
            'vehicle_height': 2.0,                     # Match actual vehicle height
            'vehicle_x_offset': -1.5,                  # Based on LIDAR position (sensor is 1.5m forward of car center)
            'vehicle_y_offset': 0.0,                   # Centered on y-axis
            'vehicle_z_offset': -2.0,                  # Based on LIDAR position (sensor is 2.0m above car)
            'vehicle_safety_margin': 0.3,              # Safety margin around vehicle
            'vehicle_visualization': True,             # Enable vehicle boundary box visualization
            
            # LiDAR specific parameters from CARLA config
            'lidar_channels': 32,                      # CARLA config: 32 channels
            'lidar_points_per_second': 100000,         # CARLA config: 100K points/sec
            'lidar_rotation_frequency': 30,            # CARLA config: 30 Hz
            'lidar_range': 100.0,                      # CARLA config: 100m range
            'lidar_upper_fov': 0.0,                    # CARLA config: 0 degrees upper FOV
            'lidar_lower_fov': -25.0,                  # CARLA config: -25 degrees lower FOV
            
            # Sun and atmospheric filtering - OPTIMIZED FOR CARLA XYZ LIDAR DATA
            'filter_sun_effects': True,                # Enable sun effect filtering
            'sky_removal_angle': 2.5,                  # Remove points with elevation > 2.5° (more precise for CARLA)
            'max_distance_filter': 85.0,               # Slightly increased from 80m to match CARLA's 100m range
            'isolated_point_removal': True,            # Remove isolated points (sun effects often create isolated returns)
            'isolated_point_radius': 0.6,              # Reduced from 0.8m for more precise filtering
            'isolated_point_min_neighbors': 3,         # Increased from 2 to 3 for better filtering
            'geometric_filter_enabled': True,          # Enable geometric-based filtering (for XYZ-only data)
            'atmospheric_filter_enabled': True,        # Enable atmospheric noise filtering
            'height_filter_max': 3.5,                  # Reduced from 5.0m to 3.5m (more appropriate for urban settings)
            'height_filter_min': -1.0,                 # Increased from -2.0m to -1.0m (less aggressive below ground)
            'density_based_filtering': True,           # Enable density-based filtering (sun effects create sparse areas)
            'density_radius': 1.0,                     # Radius to check point density
            'min_density_threshold': 4,                # Minimum number of points required in density radius
            'sun_pattern_recognition': True,           # Enable pattern recognition for sun effects
            'temporal_filter_enabled': True,           # Enable filtering across multiple frames
            'temporal_consistency_frames': 2,          # Number of frames to check for consistency
            
            # Note: Filtering parameters are now handled by the C++ pipeline:
            # - Statistical Outlier Removal (4.5, 0.8m, 8, 0.12m)
            # - RANSAC Ground Plane (250 iterations, 0.20m threshold)
            # - Euclidean Clustering (0.35m tolerance, 7 min points)
        }],
        condition=IfCondition(enable_lidar)
    )
    
    # Return the launch description
    return LaunchDescription([
        tcp_ip_arg,
        enable_imu_arg,
        enable_radar_arg,
        enable_lidar_arg,
        imu_node,
        radar_node,
        lidar_node
    ]) 