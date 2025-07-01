from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    mpc_port_arg = DeclareLaunchArgument(
        'mpc_port',
        default_value='12344',
        description='TCP port for sending MPC control commands'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Update rate in Hz for MPC control loop'
    )
    
    waypoint_port_arg = DeclareLaunchArgument(
        'waypoint_port',
        default_value='12343',
        description='TCP port for receiving waypoint data'
    )
    
    # Create waypoint listener node
    waypoint_listener_node = Node(
        package='sensor_fusion_2',
        executable='waypoint_listener',
        name='waypoint_listener_node',
        output='screen',
        parameters=[{
            'tcp_port': LaunchConfiguration('waypoint_port'),
            'update_rate': LaunchConfiguration('update_rate')
        }]
    )
    
    # Create trajectory receiver node
    trajectory_receiver_node = Node(
        package='sensor_fusion_2',
        executable='trajectory_receiver',
        name='trajectory_receiver_node',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate')
        }]
    )
    
    # Create MPC controller node
    mpc_controller_node = Node(
        package='sensor_fusion_2',
        executable='mpc_controller',
        name='mpc_controller_node',
        output='screen',
        parameters=[{
            'tcp_port': LaunchConfiguration('mpc_port'),
            'update_rate': LaunchConfiguration('update_rate')
        }]
    )
    
    return LaunchDescription([
        mpc_port_arg,
        update_rate_arg,
        waypoint_port_arg,
        waypoint_listener_node,
        trajectory_receiver_node,
        mpc_controller_node
    ]) 