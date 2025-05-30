from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('sensor_fusion')
    
    # Define the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'radar_visualization.rviz')
    
    # Launch the TF publisher node
    tf_publisher_node = Node(
        package='sensor_fusion',
        executable='radar_tf_publisher',
        name='radar_tf_publisher',
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
        tf_publisher_node,
        radar_listener_node,
        rviz_node
    ]) 