from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'radar_visualization.rviz')
    
    # Launch the static TF publisher nodes
    tf_world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    )
    
    tf_map_to_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='screen'
    )
    
    tf_base_to_radar_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_radar',
        arguments=['1.5', '0', '2.0', '0', '0', '0', 'base_link', 'radar_link'],
        output='screen'
    )
    
    # Launch the radar listener node
    radar_listener_node = Node(
        package='sensor_fusion',
        executable='radar_listener_clusters',
        name='radar_listener_clusters',
        output='screen'
    )
    
    # Launch RViz with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        tf_world_to_map_node,
        tf_map_to_base_node,
        tf_base_to_radar_node,
        radar_listener_node,
        rviz_node
    ]) 