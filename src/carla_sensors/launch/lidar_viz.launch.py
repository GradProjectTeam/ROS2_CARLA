from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('carla_sensors')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'lidar_config.rviz')

    return LaunchDescription([
        Node(
            package='carla_sensors',
            executable='lidar_listener',
            name='lidar_listener',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ]) 