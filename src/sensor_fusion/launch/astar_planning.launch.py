#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """Launch file to run the realtime mapper and Hybrid A* planner together"""
    
    # Declare launch arguments
    map_topic = LaunchConfiguration('map_topic')
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/realtime_map',
        description='Topic for the realtime map'
    )
    
    grid_size = LaunchConfiguration('grid_size')
    grid_size_arg = DeclareLaunchArgument(
        'grid_size',
        default_value='0.5',
        description='Grid size for A* planning (meters)'
    )
    
    obstacle_threshold = LaunchConfiguration('obstacle_threshold')
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='50',
        description='Threshold for considering a cell as an obstacle (0-100)'
    )
    
    # Start the realtime mapper node
    # Note: Adjust parameters as needed for your environment
    realtime_mapper_node = Node(
        package='sensor_fusion',
        executable='lidar_realtime_mapper',
        name='realtime_mapper',
        parameters=[
            {'publish_rate': 5.0},
            {'map_resolution': 0.5},
            {'map_width': 500},
            {'map_height': 500}
        ],
        output='screen'
    )
    
    # Start the Hybrid A* planner node
    hybrid_astar_node = Node(
        package='sensor_fusion',
        executable='hybrid_astar_planner',
        name='hybrid_astar_planner',
        parameters=[
            {'grid_size': grid_size},
            {'obstacle_threshold': obstacle_threshold},
            {'map_topic': map_topic},
            {'publish_rate': 5.0},
            {'max_iterations': 10000},
            {'motion_resolution': 10},
            {'angle_resolution': 36},
            {'heuristic_weight': 1.5}
        ],
        output='screen'
    )
    
    # Start the test node that integrates the realtime map with A* planner
    test_node = Node(
        package='sensor_fusion',
        executable='test_realtime_astar',
        name='realtime_astar_tester',
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        map_topic_arg,
        grid_size_arg,
        obstacle_threshold_arg,
        
        # Nodes
        realtime_mapper_node,
        hybrid_astar_node,
        test_node
    ]) 