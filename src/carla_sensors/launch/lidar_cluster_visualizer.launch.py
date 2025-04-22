from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for easy customization
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='127.0.0.1',
            description='IP address of the TCP server'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='12350',
            description='Port number of the TCP server'
        ),
        DeclareLaunchArgument(
            'point_size',
            default_value='2.0',
            description='Size of individual point markers'
        ),
        DeclareLaunchArgument(
            'update_frequency',
            default_value='30.0',
            description='Visualization update frequency in Hz'
        ),
        DeclareLaunchArgument(
            'use_point_markers',
            default_value='false',
            description='Whether to display individual point markers (impacts performance)'
        ),
        DeclareLaunchArgument(
            'fade_out_time',
            default_value='0.5',
            description='Time in seconds for markers to fade out when no longer updated'
        ),
        DeclareLaunchArgument(
            'fade_out_clusters',
            default_value='true',
            description='Whether to enable fading out of clusters'
        ),
        DeclareLaunchArgument(
            'live_animation',
            default_value='true',
            description='Whether to enable smooth animation effects'
        ),
        
        # The main visualization node
        Node(
            package='carla_sensors',
            executable='lidar_listener_clusters.py',
            name='lidar_cluster_visualizer',
            parameters=[{
                'tcp_ip': LaunchConfiguration('tcp_ip'),
                'tcp_port': int(LaunchConfiguration('tcp_port')),
                'point_size': float(LaunchConfiguration('point_size')),
                'center_size': 3.0,
                'use_convex_hull': True,
                'use_point_markers': LaunchConfiguration('use_point_markers'),
                'use_cluster_stats': True,
                'verbose_logging': False,
                'update_frequency': float(LaunchConfiguration('update_frequency')),
                'fade_out_time': float(LaunchConfiguration('fade_out_time')),
                'fade_out_clusters': LaunchConfiguration('fade_out_clusters'),
                'live_animation': LaunchConfiguration('live_animation'),
                'socket_timeout': 0.1,
            }],
            output='screen'
        ),
        
        # Optional: Auto-launch RViz with a prepared configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/mostafa/GP/ROS2/src/carla_sensors/rviz/lidar_clusters.rviz'],
            output='screen'
        )
    ]) 