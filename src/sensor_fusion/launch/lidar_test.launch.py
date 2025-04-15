from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        
        # Visualization parameters
        DeclareLaunchArgument(
            'point_size',
            default_value='1.5',
            description='Size of individual point markers'
        ),
        DeclareLaunchArgument(
            'center_size',
            default_value='3.0',
            description='Size of cluster center markers'
        ),
        DeclareLaunchArgument(
            'use_convex_hull',
            default_value='true',
            description='Whether to display 2D convex hull around clusters'
        ),
        DeclareLaunchArgument(
            'use_point_markers',
            default_value='false',
            description='Whether to display individual point markers (impacts performance)'
        ),
        DeclareLaunchArgument(
            'use_cluster_stats',
            default_value='true',
            description='Whether to display cluster statistics'
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
        
        # Vehicle filter parameters
        DeclareLaunchArgument(
            'filter_vehicle_points',
            default_value='true',
            description='Whether to filter out points that hit the car itself'
        ),
        DeclareLaunchArgument(
            'vehicle_length',
            default_value='4.5',
            description='Length of vehicle in meters (x direction)'
        ),
        DeclareLaunchArgument(
            'vehicle_width',
            default_value='2.0',
            description='Width of vehicle in meters (y direction)'
        ),
        DeclareLaunchArgument(
            'vehicle_height',
            default_value='1.8',
            description='Height of vehicle in meters (z direction)'
        ),
        DeclareLaunchArgument(
            'vehicle_x_offset',
            default_value='0.0',
            description='Offset of vehicle center in x direction'
        ),
        DeclareLaunchArgument(
            'vehicle_y_offset',
            default_value='0.0',
            description='Offset of vehicle center in y direction'
        ),
        DeclareLaunchArgument(
            'vehicle_safety_margin',
            default_value='0.2',
            description='Extra margin around vehicle to filter'
        ),
        
        # LiDAR Listener Node
        Node(
            package='sensor_fusion',
            executable='lidar_listener_clusters_2',
            name='lidar_cube_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('lidar_tcp_ip'),
                'tcp_port': LaunchConfiguration('lidar_tcp_port'),
                'point_size': LaunchConfiguration('point_size'),
                'center_size': LaunchConfiguration('center_size'),
                'use_convex_hull': LaunchConfiguration('use_convex_hull'),
                'use_point_markers': LaunchConfiguration('use_point_markers'),
                'use_cluster_stats': LaunchConfiguration('use_cluster_stats'),
                'verbose_logging': False,
                'cube_alpha': 0.3,
                
                # Vehicle filter parameters
                'filter_vehicle_points': LaunchConfiguration('filter_vehicle_points'),
                'vehicle_length': LaunchConfiguration('vehicle_length'),
                'vehicle_width': LaunchConfiguration('vehicle_width'),
                'vehicle_height': LaunchConfiguration('vehicle_height'),
                'vehicle_x_offset': LaunchConfiguration('vehicle_x_offset'),
                'vehicle_y_offset': LaunchConfiguration('vehicle_y_offset'),
                'vehicle_safety_margin': LaunchConfiguration('vehicle_safety_margin'),
            }],
            output='screen'
        ),
        
        # LiDAR Costmap Creator Node
        Node(
            package='sensor_fusion',
            executable='lidar_costmap_creator',
            name='lidar_costmap_creator',
            parameters=[{
                # Map resolution and size parameters 
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_width': 1000, 
                'map_height': 1000, 
                'map_width_meters': LaunchConfiguration('map_width_meters'),
                'map_height_meters': LaunchConfiguration('map_height_meters'),
                
                # Set map origin to center the vehicle
                'map_origin_x': -50.0,
                'map_origin_y': -50.0,
                
                # Vehicle offset parameters
                'vehicle_x_offset': 0.0,
                'vehicle_y_offset': 0.0,
                
                # Processing parameters
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
                'center_priority_radius': 10.0,
                'vehicle_center_weight': 1.5,
            }],
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/sensor_fusion.rviz']],
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
            name='tf_base_to_lidar',
            arguments=['0', '0', '2.0', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),
    ]) 