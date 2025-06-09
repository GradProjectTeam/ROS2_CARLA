from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    carla_sensors_pkg = get_package_share_directory('carla_sensors')
    
    # Parameters
    map_resolution = LaunchConfiguration('map_resolution')
    map_width = LaunchConfiguration('map_width')
    map_height = LaunchConfiguration('map_height')
    map_origin_x = LaunchConfiguration('map_origin_x')
    map_origin_y = LaunchConfiguration('map_origin_y')
    
    # Launch Arguments
    map_resolution_arg = DeclareLaunchArgument(
        'map_resolution',
        default_value='0.1',
        description='Resolution of the costmap in meters per cell'
    )
    
    map_width_arg = DeclareLaunchArgument(
        'map_width',
        default_value='1000',
        description='Width of the costmap in cells'
    )
    
    map_height_arg = DeclareLaunchArgument(
        'map_height',
        default_value='1000',
        description='Height of the costmap in cells'
    )
    
    map_origin_x_arg = DeclareLaunchArgument(
        'map_origin_x',
        default_value='-50.0',
        description='X coordinate of the map origin'
    )
    
    map_origin_y_arg = DeclareLaunchArgument(
        'map_origin_y',
        default_value='-50.0',
        description='Y coordinate of the map origin'
    )
    
    # Define the data source nodes
    # 1. Enhanced Radar Visualizer - Receives radar data via TCP
    enhanced_radar_visualizer_node = Node(
        package='carla_sensors',
        executable='enhanced_radar_visualizer',
        name='enhanced_radar_visualizer',
        output='screen',
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12348,
            'reconnect_interval': 5.0,
            'marker_lifetime': 0.5,
            'max_buffer_size': 1048576
        }]
    )
    
    # 2. LiDAR Cluster Listener - Receives LiDAR data via TCP
    lidar_listener_clusters_node = Node(
        package='carla_sensors',
        executable='lidar_listener_clusters',
        name='lidar_listener_clusters',
        output='screen',
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12350,
            'point_size': 2.0,
            'center_size': 3.0,
            'use_convex_hull': True,
            'use_point_markers': True,
            'use_cluster_stats': True,
            'verbose_logging': False
        }]
    )
    
    # 3. IMU Listener - Receives IMU data via TCP
    imu_listener_node = Node(
        package='carla_sensors',
        executable='imu_listener',
        name='imu_listener',
        output='screen',
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12350,
            'reconnect_interval': 5.0,
            'frame_id': 'imu_link'
        }]
    )
    
    # Define the costmap creation nodes
    # 4. Radar Costmap Creator - Converts radar markers to costmap
    radar_costmap_creator_node = Node(
        package='carla_sensors',
        executable='radar_costmap_creator',
        name='radar_costmap_creator',
        output='screen',
        parameters=[{
            'map_resolution': map_resolution,
            'map_width': map_width,
            'map_height': map_height,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'publish_rate': 5.0,
            'velocity_impact_factor': 20.0,
            'max_marker_age': 1.0,
            'radar_weight': 0.7
        }]
    )
    
    # 5. LiDAR Costmap Creator - Converts LiDAR points to costmap
    lidar_costmap_creator_node = Node(
        package='carla_sensors',
        executable='lidar_costmap_creator',
        name='lidar_costmap_creator',
        output='screen',
        parameters=[{
            'map_resolution': map_resolution,
            'map_width': map_width,
            'map_height': map_height,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'publish_rate': 5.0,
            'ground_threshold': 0.3,
            'max_points_per_cell': 5,
            'min_height': -2.0,
            'max_height': 3.0,
            'obstacle_inflation': 0.4,
            'max_data_age': 1.0
        }]
    )
    
    # 6. Sensor Fusion Costmap - Combines radar and lidar costmaps
    sensor_fusion_costmap_node = Node(
        package='carla_sensors',
        executable='sensor_fusion_costmap',
        name='sensor_fusion_costmap',
        output='screen',
        parameters=[{
            'map_resolution': map_resolution,
            'map_width': map_width,
            'map_height': map_height,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'publish_rate': 5.0,
            'lidar_weight_static': 0.8,
            'radar_weight_static': 0.2,
            'lidar_weight_dynamic': 0.3,
            'radar_weight_dynamic': 0.7,
            'dynamic_threshold': 60,
            'max_data_age': 1.0
        }]
    )
    
    # Optional: RViz for visualization
    rviz_config_file = os.path.join(carla_sensors_pkg, 'rviz', 'sensor_fusion.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        map_resolution_arg,
        map_width_arg,
        map_height_arg,
        map_origin_x_arg,
        map_origin_y_arg,
        
        # Data source nodes
        enhanced_radar_visualizer_node,
        lidar_listener_clusters_node,
        imu_listener_node,
        
        # Processing nodes
        radar_costmap_creator_node,
        lidar_costmap_creator_node,
        sensor_fusion_costmap_node,
        
        # Visualization
        rviz_node
    ]) 