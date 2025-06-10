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
    rviz_config_file = os.path.join(pkg_dir, 'config', 'lane_visualization.rviz')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')
        
    use_camera_processor_arg = DeclareLaunchArgument(
        'use_camera_processor',
        default_value='true',
        description='Whether to start the camera processor node')
    
    # IMU TCP parameters
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='IP address of the IMU TCP server')
        
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='12345',
        description='Port of the IMU TCP server')
    
    # Camera TCP parameters
    camera_tcp_ip_arg = DeclareLaunchArgument(
        'camera_tcp_ip',
        default_value='localhost',
        description='IP address of the camera TCP server')
        
    camera_tcp_port_arg = DeclareLaunchArgument(
        'camera_tcp_port',
        default_value='12342',
        description='Port of the camera TCP server')
    
    compass_offset_arg = DeclareLaunchArgument(
        'compass_offset',
        default_value='0.0',
        description='Offset to apply to compass heading in radians')
        
    heading_smoothing_factor_arg = DeclareLaunchArgument(
        'heading_smoothing_factor',
        default_value='0.5',
        description='Smoothing factor for heading changes (0.0-1.0, higher = more responsive)')
        
    heading_rate_limit_arg = DeclareLaunchArgument(
        'heading_rate_limit',
        default_value='60.0',
        description='Maximum heading change rate in degrees per second (0.0 = no limit)')
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='Rate to publish visualization data in Hz')
    
    # Lane detection parameters
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
    
    # Create camera processor node with improved lane detection
    camera_processor_node = Node(
        package='sensor_fusion',
        executable='camera_processor_node',
        name='camera_processor',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'server_host': LaunchConfiguration('camera_tcp_ip')},
            {'server_port': LaunchConfiguration('camera_tcp_port')},
            {'canny_low_threshold': LaunchConfiguration('canny_low_threshold')},
            {'canny_high_threshold': LaunchConfiguration('canny_high_threshold')},
            {'hough_threshold': LaunchConfiguration('hough_threshold')},
            {'min_line_length': LaunchConfiguration('min_line_length')},
            {'max_line_gap': LaunchConfiguration('max_line_gap')},
            {'lane_fill_alpha': LaunchConfiguration('lane_fill_alpha')}
        ],
        condition=IfCondition(LaunchConfiguration('use_camera_processor'))
    )
    
    # Create IMU-Camera Lane Fusion node
    imu_camera_lane_fusion_node = Node(
        package='sensor_fusion',
        executable='imu_camera_lane_fusion',
        name='imu_camera_lane_fusion',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'camera_topic': '/carla/camera/rgb/image_raw'},
            {'publish_rate': LaunchConfiguration('publish_rate')},
            {'map_frame_id': 'map'},
            {'base_frame_id': 'base_link'},
            {'camera_frame_id': 'camera_link'},
            {'lane_projection_distance': 20.0},
            {'lane_width': 3.5},
            {'use_filtered_heading': True},
            {'heading_filter_size': 10},
            {'heading_smoothing_factor': LaunchConfiguration('heading_smoothing_factor')},
            {'heading_rate_limit': LaunchConfiguration('heading_rate_limit')},
            {'tcp_ip': LaunchConfiguration('tcp_ip')},
            {'tcp_port': LaunchConfiguration('tcp_port')},
            {'reconnect_interval': 5.0},
            {'compass_offset': LaunchConfiguration('compass_offset')}
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
        use_rviz_arg,
        use_camera_processor_arg,
        tcp_ip_arg,
        tcp_port_arg,
        camera_tcp_ip_arg,
        camera_tcp_port_arg,
        compass_offset_arg,
        heading_smoothing_factor_arg,
        heading_rate_limit_arg,
        publish_rate_arg,
        canny_low_threshold_arg,
        canny_high_threshold_arg,
        hough_threshold_arg,
        min_line_length_arg,
        max_line_gap_arg,
        lane_fill_alpha_arg,
        camera_processor_node,
        imu_camera_lane_fusion_node,
        rviz_node
    ]) 