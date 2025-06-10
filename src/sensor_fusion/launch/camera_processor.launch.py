#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('sensor_fusion')
    
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'camera_view.rviz')
    
    # Launch argument for RViz config
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')
        
    # Create camera processor node
    camera_processor_node = Node(
        package='sensor_fusion',
        executable='camera_processor_node',
        name='camera_processor',
        output='screen',
        emulate_tty=True
    )
    
    # Create RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Create and return launch description
    return LaunchDescription([
        use_rviz_arg,
        camera_processor_node,
        rviz_node
    ]) 