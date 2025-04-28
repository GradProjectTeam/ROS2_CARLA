from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_dir = get_package_share_directory('sensor_fusion_2')
    
    # RViz configuration file
    rviz_config = os.path.join(package_dir, 'rviz2', 'sensor_fusion.rviz')
    
    # Create LaunchDescription
    ld = LaunchDescription()
    
    # Add static tf broadcaster for sensor frames
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'lidar_link'],
        name='lidar_frame_publisher'
    )
    
    # Commenting out IMU tf node for now
    # imu_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0', '0.1', '0', '0', '0', 'map', 'imu_link'],
    #     name='imu_frame_publisher'
    # )
    
    # Commenting out Radar tf node for now
    # radar_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0', '0.1', '0', '0', '0', '0', 'map', 'radar_link'],
    #     name='radar_frame_publisher'
    # )
    
    # Commenting out IMU TCP Node for now
    # imu_node = Node(
    #     package='sensor_fusion_2',
    #     executable='imu_tcp_node',
    #     name='imu_tcp_node',
    #     parameters=[{
    #         'tcp_ip': '127.0.0.1',
    #         'tcp_port': 12345,
    #         'frame_id': 'imu_link'
    #     }],
    #     output='screen'
    # )
    
    # Commenting out Radar TCP Node for now
    # radar_node = Node(
    #     package='sensor_fusion_2',
    #     executable='radar_tcp_node',
    #     name='radar_tcp_node',
    #     parameters=[{
    #         'tcp_ip': '127.0.0.1',
    #         'tcp_port': 12346,
    #         'frame_id': 'radar_link'
    #     }],
    #     output='screen'
    # )
    
    # LiDAR Listener for processed clusters from C++ pipeline
    lidar_cluster_node = Node(
        package='sensor_fusion_2',
        executable='lidar_listener_clusters_2',
        name='lidar_cluster_listener',
        parameters=[{
            # Connection parameters
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12350,  # Must match LIDAR_SEND_PORT in C++ code
            'frame_id': 'lidar_link',
            
            # Debug parameters
            'verbose_logging': True,
            
            # Visualization parameters
            'point_size': 2.0,
            'center_size': 3.5,  # Increased for better visibility of cluster centers
            'use_convex_hull': True,
            'use_point_markers': True,
            'use_cluster_stats': True,
            'cube_alpha': 0.3,
            
            # CARLA LiDAR Configuration - UPDATED BASED ON NEW SETTINGS
            'filter_vehicle_points': True,             # Enable filtering
            'vehicle_length': 5.0,                     # Increased to ensure all roof points are filtered
            'vehicle_width': 2.5,                      # Increased to ensure all roof points are filtered
            'vehicle_height': 2.2,                     # Slightly increased from original
            'vehicle_x_offset': 0.0,                   # Centered (LiDAR is now centered on vehicle x=0.0)
            'vehicle_y_offset': 0.0,                   # Centered on y-axis
            'vehicle_z_offset': -1.0,                  # Updated based on new LiDAR height (1.0m above car)
            'vehicle_safety_margin': 0.5,              # Increased safety margin for better filtering
            'vehicle_visualization': True,             # Enable vehicle boundary box visualization
            
            # LiDAR specific parameters from updated CARLA config
            'lidar_channels': 32,                      # CARLA config: 32 channels
            'lidar_points_per_second': 100000,         # CARLA config: 100K points/sec
            'lidar_rotation_frequency': 30,            # CARLA config: 30 Hz
            'lidar_range': 100.0,                      # CARLA config: 100m range
            'lidar_upper_fov': 15.0,                   # UPDATED: 15 degrees upper FOV
            'lidar_lower_fov': -25.0,                  # UPDATED: -25 degrees lower FOV
            'lidar_pitch_angle': 5.0,                  # NEW: 5 degrees upward pitch
            'min_point_distance': 3.0,                 # NEW: Minimum distance from sensor to keep points
            'max_negative_z': -0.5                     # NEW: Filter out points below this Z value
            
            # Filtering parameters are commented out since they're handled by the C++ pipeline
            # These parameters are provided for reference only:
            # 
            # Statistical Outlier Removal:
            # - std dev multiplier: 4.5
            # - neighbor radius: 0.8m
            # - neighbor count: 8
            # - voxel size: 0.12m
            #
            # RANSAC Ground Plane:
            # - iterations: 250
            # - distance threshold: 0.20m
            #
            # Euclidean Clustering:
            # - cluster tolerance: 0.35m
            # - minimum points: 7
        }],
        output='screen'
    )
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Add all nodes to the launch description
    ld.add_action(tf_static_node)
    # Commenting out IMU and radar nodes to focus on LiDAR only
    # ld.add_action(imu_tf_node)
    # ld.add_action(radar_tf_node)
    # ld.add_action(imu_node)
    # ld.add_action(radar_node)
    ld.add_action(lidar_cluster_node)
    ld.add_action(rviz_node)
    
    return ld 