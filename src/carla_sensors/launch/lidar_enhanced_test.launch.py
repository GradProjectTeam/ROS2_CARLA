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
    sensor_x = LaunchConfiguration('sensor_x')
    sensor_y = LaunchConfiguration('sensor_y')
    sensor_height = LaunchConfiguration('sensor_height')
    
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
    
    sensor_x_arg = DeclareLaunchArgument(
        'sensor_x',
        default_value='0.0',
        description='X position of the LiDAR sensor in map frame'
    )
    
    sensor_y_arg = DeclareLaunchArgument(
        'sensor_y',
        default_value='0.0',
        description='Y position of the LiDAR sensor in map frame'
    )
    
    sensor_height_arg = DeclareLaunchArgument(
        'sensor_height',
        default_value='1.0',
        description='Height of the LiDAR sensor above ground'
    )
    
    # LiDAR Cluster Listener - Receives LiDAR data via TCP
    lidar_listener_clusters_node = Node(
        package='carla_sensors',
        executable='lidar_listener_clusters',
        name='lidar_listener_clusters',
        output='screen',
        respawn=True,  # Automatically respawn if the node crashes
        respawn_delay=2.0,  # Wait 2 seconds before respawning
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12350,  # Using LiDAR port from the CARLA script
            'point_size': 2.0,
            'center_size': 3.0,
            'use_convex_hull': True,
            'use_point_markers': True,
            'use_cluster_stats': True,
            'verbose_logging': False
        }]
    )
    
    # Enhanced LiDAR Costmap Creator with Free Space Detection
    lidar_costmap_enhanced_node = Node(
        package='carla_sensors',
        executable='lidar_costmap_enhanced',
        name='lidar_costmap_enhanced',
        output='screen',
        parameters=[{
            'map_resolution': map_resolution,
            'map_width': map_width,
            'map_height': map_height,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'publish_rate': 5.0,
            'ground_threshold': 0.1,
            'max_points_per_cell': 5,
            'min_height': -5.0,
            'max_height': 5.0,
            'obstacle_inflation': 0.4,
            'max_data_age': 60.0,
            'sensor_x': sensor_x,
            'sensor_y': sensor_y,
            'sensor_height': sensor_height
        }]
    )
    
    # RViz for visualization
    rviz_config_file = os.path.join(carla_sensors_pkg, 'rviz', 'lidar_clusters.rviz')
    
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
        sensor_x_arg,
        sensor_y_arg,
        sensor_height_arg,
        
        # Data source node
        lidar_listener_clusters_node,
        
        # Enhanced processing node
        lidar_costmap_enhanced_node,
        
        # Visualization
        rviz_node
    ]) 