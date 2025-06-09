from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

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
    
    # Enhanced Radar Visualizer - Receives radar data via TCP
    enhanced_radar_visualizer_node = Node(
        package='carla_sensors',
        executable='enhanced_radar_visualizer',
        name='enhanced_radar_visualizer',
        output='screen',
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12347,  # Using Radar port from the CARLA script
            'reconnect_interval': 5.0,
            'marker_lifetime': 0.5,
            'max_buffer_size': 1048576
        }]
    )
    
    # Radar Costmap Creator - Converts radar markers to costmap
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
    
    # Use the radar-specific RViz configuration
    rviz_config_file = os.path.join(carla_sensors_pkg, 'rviz', 'radar_test.rviz')
    
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
        
        # Data source node
        enhanced_radar_visualizer_node,
        
        # Processing node
        radar_costmap_creator_node,
        
        # Visualization
        rviz_node
    ]) 