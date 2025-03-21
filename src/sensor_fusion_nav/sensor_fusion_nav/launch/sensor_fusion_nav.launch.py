import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('sensor_fusion_nav')
    
    # Configuration files
    ekf_config_file = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    costmap_params_file = os.path.join(pkg_dir, 'config', 'costmap_params.yaml')
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo/CARLA) clock if true')
    
    # Define the sensor processing nodes
    lidar_processor_node = Node(
        package='sensor_fusion_nav',
        executable='lidar_processing',
        name='lidar_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    radar_processor_node = Node(
        package='sensor_fusion_nav',
        executable='radar_processing',
        name='radar_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    imu_processor_node = Node(
        package='sensor_fusion_nav',
        executable='imu_processing',
        name='imu_processor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Define the sensor fusion node
    sensor_fusion_node = Node(
        package='sensor_fusion_nav',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Define the costmap creator node
    costmap_creator_node = Node(
        package='sensor_fusion_nav',
        executable='costmap_creator',
        name='costmap_creator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            costmap_params_file
        ]
    )
    
    # Define the navigation node
    navigation_node = Node(
        package='sensor_fusion_nav',
        executable='navigation',
        name='navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Define the Extended Kalman Filter node for robot localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add the nodes to the launch description
    ld.add_action(lidar_processor_node)
    ld.add_action(radar_processor_node)
    ld.add_action(imu_processor_node)
    ld.add_action(ekf_node)
    ld.add_action(sensor_fusion_node)
    ld.add_action(costmap_creator_node)
    ld.add_action(navigation_node)
    
    return ld