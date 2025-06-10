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
    
    # Camera TCP connection parameters
    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='localhost',
        description='Camera TCP server hostname or IP address')
        
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='12342',
        description='Camera TCP server port')
    
    # Launch arguments for lane detection parameters
    canny_low_threshold_arg = DeclareLaunchArgument(
        'canny_low_threshold',
        default_value='50',
        description='Low threshold for Canny edge detector')
        
    canny_high_threshold_arg = DeclareLaunchArgument(
        'canny_high_threshold',
        default_value='100',
        description='High threshold for Canny edge detector')
        
    hough_threshold_arg = DeclareLaunchArgument(
        'hough_threshold',
        default_value='25',
        description='Threshold for Hough line detection')
        
    min_line_length_arg = DeclareLaunchArgument(
        'min_line_length',
        default_value='10',
        description='Minimum line length for Hough detection')
        
    max_line_gap_arg = DeclareLaunchArgument(
        'max_line_gap',
        default_value='20',
        description='Maximum gap between line segments for Hough detection')
        
    lane_fill_alpha_arg = DeclareLaunchArgument(
        'lane_fill_alpha',
        default_value='0.9',
        description='Transparency of lane fill (0.0-1.0)')
    
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
        emulate_tty=True,
        parameters=[
            {'server_host': LaunchConfiguration('server_host')},
            {'server_port': LaunchConfiguration('server_port')},
            {'canny_low_threshold': LaunchConfiguration('canny_low_threshold')},
            {'canny_high_threshold': LaunchConfiguration('canny_high_threshold')},
            {'hough_threshold': LaunchConfiguration('hough_threshold')},
            {'min_line_length': LaunchConfiguration('min_line_length')},
            {'max_line_gap': LaunchConfiguration('max_line_gap')},
            {'lane_fill_alpha': LaunchConfiguration('lane_fill_alpha')}
        ]
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
        server_host_arg,
        server_port_arg,
        canny_low_threshold_arg,
        canny_high_threshold_arg,
        hough_threshold_arg,
        min_line_length_arg,
        max_line_gap_arg,
        lane_fill_alpha_arg,
        use_rviz_arg,
        camera_processor_node,
        rviz_node
    ]) 