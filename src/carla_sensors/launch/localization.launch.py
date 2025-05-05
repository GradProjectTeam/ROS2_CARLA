from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Construct the path to the ekf.yaml configuration file
    config_file = PathJoinSubstitution([
        FindPackageShare('carla_sensors'),
        'config',
        'ekf.yaml'
    ])
    
    return LaunchDescription([
        # Node(
        #     package='carla_sensors',
        #     executable='radar_listener',
        #     name='radar_listener',
        #     output='screen'
        # ),
        Node(
            package='carla_sensors',
            executable='imu_listener',
            name='imu_listener',
            output='screen'
        ),
        # Node(
        #     package='carla_sensors',
        #     executable='lidar_listener',
        #     name='lidar_listener',
        #     output='screen'
        # ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
                ('imu/data', 'imu/data_raw')
            ]
        ),
        Node(
            package='carla_sensors',
            executable='pcd_publisher',
            name='pcd_publisher',
            output='screen'
        ),
        # Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='radar_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'radar_link']
        ),

        
    ])