from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    carla_sensors_pkg = get_package_share_directory('carla_sensors')
    
    # IMU Listener - Receives IMU data via TCP
    imu_listener_node = Node(
        package='carla_sensors',
        executable='imu_listener',
        name='imu_listener',
        output='screen',
        parameters=[{
            'tcp_ip': '127.0.0.1',
            'tcp_port': 12350,  # Using IMU port from the CARLA script
            'reconnect_interval': 5.0,
            'frame_id': 'imu_link'
        }]
    )
    
    # Use the IMU-specific RViz configuration
    rviz_config_file = os.path.join(carla_sensors_pkg, 'rviz', 'imu_test.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Optional: Add a node to visualize the IMU as arrows
    imu_visualizer_node = Node(
        package='rviz_imu_plugin',
        executable='rviz_imu_plugin_node',
        name='imu_visualizer',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'topic': '/imu/data'
        }],
        condition=LaunchConfiguration('use_rviz_imu_plugin', default='false')
    )
    
    use_rviz_imu_plugin_arg = DeclareLaunchArgument(
        'use_rviz_imu_plugin',
        default_value='false',
        description='Use the RViz IMU plugin for visualization (requires rviz_imu_plugin package)'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_imu_plugin_arg,
        
        # IMU data source node
        imu_listener_node,
        
        # Visualization
        rviz_node,
        imu_visualizer_node
    ]) 