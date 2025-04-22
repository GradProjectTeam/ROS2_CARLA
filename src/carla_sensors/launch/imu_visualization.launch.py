from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('carla_sensors')
    
    # Set the path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'imu_visualization.rviz')
    
    # Declare launch arguments
    tcp_ip_arg = DeclareLaunchArgument(
        'tcp_ip',
        default_value='127.0.0.1',
        description='IP address of the IMU TCP server'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='12345',
        description='Port number of the IMU TCP server'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for the IMU'
    )
    
    arrow_scale_arg = DeclareLaunchArgument(
        'arrow_scale',
        default_value='1.0',
        description='Scale factor for visualization arrows (larger = bigger arrows)'
    )
    
    update_frequency_arg = DeclareLaunchArgument(
        'update_frequency',
        default_value='100.0',
        description='Frequency to process IMU data (Hz)'
    )
    
    # Node for IMU visualization
    imu_rviz_node = Node(
        package='carla_sensors',
        executable='imu_rviz_visualizer.py',
        name='imu_rviz_visualizer',
        parameters=[{
            'tcp_ip': LaunchConfiguration('tcp_ip'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'arrow_scale': LaunchConfiguration('arrow_scale'),
            'update_frequency': LaunchConfiguration('update_frequency')
        }],
        output='screen'
    )
    
    # Launch RViz2 with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([
        tcp_ip_arg,
        tcp_port_arg,
        frame_id_arg,
        arrow_scale_arg,
        update_frequency_arg,
        imu_rviz_node,
        rviz_node
    ]) 