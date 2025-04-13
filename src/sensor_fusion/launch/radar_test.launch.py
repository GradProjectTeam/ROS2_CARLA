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
            'radar_tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the Radar TCP server'
        ),
        DeclareLaunchArgument(
            'radar_tcp_port',
            default_value='12351',
            description='Port number of the Radar TCP server'
        ),
        
        # Visualization parameters
        DeclareLaunchArgument(
            'point_size',
            default_value='2.0',
            description='Size of radar point markers'
        ),
        DeclareLaunchArgument(
            'verbose_logging',
            default_value='false',
            description='Enable verbose logging'
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
            default_value='50.0',
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
        
        # Radar Listener Node
        Node(
            package='sensor_fusion',
            executable='radar_listener_clusters',
            name='radar_listener',
            parameters=[{
                'tcp_ip': LaunchConfiguration('radar_tcp_ip'),
                'tcp_port': LaunchConfiguration('radar_tcp_port'),
                'point_size': LaunchConfiguration('point_size'),
                'verbose_logging': LaunchConfiguration('verbose_logging'),
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
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [get_package_share_directory('sensor_fusion'), '/rviz/radar_visualization.rviz']],
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
            name='tf_base_to_radar',
            arguments=['2.5', '0', '0.5', '0', '0', '0', 'base_link', 'radar_link'],
            output='screen'
        ),
    ]) 