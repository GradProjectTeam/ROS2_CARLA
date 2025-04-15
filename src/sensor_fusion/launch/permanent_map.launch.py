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
        DeclareLaunchArgument(
            'cube_alpha',
            default_value='0.3',
            description='Transparency for cube visualization (0.0-1.0)'
        ),
        
        # Permanent map parameters
        DeclareLaunchArgument(
            'map_resolution',
            default_value='0.2',
            description='Resolution of the permanent map (meters per cell)'
        ),
        DeclareLaunchArgument(
            'map_width_meters',
            default_value='100.0',
            description='Width of the permanent map in meters'
        ),
        DeclareLaunchArgument(
            'map_height_meters',
            default_value='100.0',
            description='Height of the permanent map in meters'
        ),
        DeclareLaunchArgument(
            'detection_radius',
            default_value='50.0',
            description='Detection radius for the permanent map (meters)'
        ),
        
        # Performance parameters
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate to publish map (Hz)'
        ),
        DeclareLaunchArgument(
            'process_rate',
            default_value='2.0',
            description='Rate to process map data (Hz)'
        ),
        DeclareLaunchArgument(
            'raycast_skip',
            default_value='5',
            description='Only raytrace every Nth point for better performance'
        ),
        DeclareLaunchArgument(
            'max_points_to_process',
            default_value='3000',
            description='Maximum number of points to process per update'
        ),
        
        # Bayesian update parameters - Extreme binary mapping
        DeclareLaunchArgument(
            'hit_weight',
            default_value='0.99',  # Increased to almost certain for hits
            description='Weight for obstacle hits in Bayesian update'
        ),
        DeclareLaunchArgument(
            'miss_weight',
            default_value='0.05',  # Decreased for minimal free space marking
            description='Weight for misses (free space) in Bayesian update'
        ),
        
        # Map saving parameters
        DeclareLaunchArgument(
            'map_save_dir',
            default_value='/home/mostafa/GP/ROS2/maps',
            description='Directory to save permanent maps'
        ),
        DeclareLaunchArgument(
            'enable_auto_save',
            default_value='true',
            description='Enable automatic saving of the map periodically'
        ),
        DeclareLaunchArgument(
            'auto_save_interval',
            default_value='60.0',
            description='Interval for auto-saving the map (seconds)'
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
        
        # LiDAR Permanent Mapper Node
        Node(
            package='sensor_fusion',
            executable='lidar_permanent_mapper',
            name='lidar_permanent_mapper',
            parameters=[{
                # Map resolution and size parameters
                'map_resolution': LaunchConfiguration('map_resolution'),
                'map_width_meters': LaunchConfiguration('map_width_meters'),
                'map_height_meters': LaunchConfiguration('map_height_meters'),
                
                # We can't calculate these directly in the launch file
                # The node will calculate them internally based on the above parameters
                
                # Processing parameters
                'publish_rate': LaunchConfiguration('publish_rate'),
                'process_rate': LaunchConfiguration('process_rate'),
                'raycast_skip': LaunchConfiguration('raycast_skip'),
                'max_points_to_process': LaunchConfiguration('max_points_to_process'),
                
                # Bayesian update parameters - Extreme binary mapping
                'hit_weight': LaunchConfiguration('hit_weight'),
                'miss_weight': LaunchConfiguration('miss_weight'),
                'update_threshold': 0.1,     # Very low threshold for quick updates
                'prior_weight': 0.5,
                'count_threshold': 2.0,      # Very low threshold to mark cells as occupied quickly
                
                # Map saving parameters
                'map_save_dir': LaunchConfiguration('map_save_dir'),
                'enable_auto_save': LaunchConfiguration('enable_auto_save'),
                'auto_save_interval': LaunchConfiguration('auto_save_interval'),
                
                # Other parameters
                'ground_threshold': 0.4,     # Increased further to more aggressively filter ground
                'min_height': -0.5,          # Raised further to ignore lower points
                'max_height': 4.0,
                'detection_radius': LaunchConfiguration('detection_radius'),
                'use_cluster_data': True,
            }],
            output='screen'
        ),
        
        # RViz for visualization - Make sure to use a custom Black & White map color scheme
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/permanent_map.rviz']],
            output='screen'
        ),
        
        # TF static transform publishers - complete tree
        # 1. World to Map - top level transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),
        
        # 2. Map to Odom - necessary intermediate transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),
        
        # 3. Odom to Base Link - connects vehicle base to the map
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),
        
        # 4. Base Link to LiDAR Link - LiDAR sensor position
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            arguments=['1.5', '0', '2.0', '0', '0', '0', 'base_link', 'lidar_link'],
            output='screen'
        ),
    ]) 