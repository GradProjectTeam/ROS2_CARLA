from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for easy customization
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the TCP server'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='12350',
            description='Port number of the TCP server'
        ),
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
            'use_point_markers',
            default_value='false',
            description='Whether to display individual point markers (impacts performance)'
        ),
        DeclareLaunchArgument(
            'use_convex_hull',
            default_value='true',
            description='Whether to display 2D convex hull around clusters'
        ),
        DeclareLaunchArgument(
            'use_cluster_stats',
            default_value='true',
            description='Whether to display cluster statistics'
        ),
        DeclareLaunchArgument(
            'verbose_logging',
            default_value='false',
            description='Enable verbose logging'
        ),
        DeclareLaunchArgument(
            'cube_alpha',
            default_value='0.3',
            description='Transparency for cube visualization (0.0-1.0)'
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
        
        # The main visualization node with enhanced 3D cube visualization
        Node(
            package='carla_sensors',
            executable='lidar_listener_clusters_2',
            name='lidar_cube_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('tcp_ip'),
                'tcp_port': LaunchConfiguration('tcp_port'),
                'point_size': LaunchConfiguration('point_size'),
                'center_size': LaunchConfiguration('center_size'),
                'use_convex_hull': LaunchConfiguration('use_convex_hull'),
                'use_point_markers': LaunchConfiguration('use_point_markers'),
                'use_cluster_stats': LaunchConfiguration('use_cluster_stats'),
                'verbose_logging': LaunchConfiguration('verbose_logging'),
                'cube_alpha': LaunchConfiguration('cube_alpha'),
            }],
            output='screen'
        ),
        
        # Costmap node to create 2D occupancy grid from LIDAR data
        Node(
            package='carla_sensors',
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
        
        # Auto-launch RViz with a prepared configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/mostafa/GP/ROS2/src/carla_sensors/rviz/lidar_clusters.rviz'],
            output='screen'
        ),
        
        # Add TF2 static transform publisher for the map frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        )
    ]) 