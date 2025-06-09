#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')
        
        # Declare parameters - check if use_sim_time already exists
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
            
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('goal_marker_color_r', 1.0)
        self.declare_parameter('goal_marker_color_g', 0.0)
        self.declare_parameter('goal_marker_color_b', 0.0)
        self.declare_parameter('goal_marker_color_a', 1.0)
        self.declare_parameter('goal_marker_scale', 0.5)
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        
        self.goal_marker_color = ColorRGBA()
        self.goal_marker_color.r = self.get_parameter('goal_marker_color_r').get_parameter_value().double_value
        self.goal_marker_color.g = self.get_parameter('goal_marker_color_g').get_parameter_value().double_value
        self.goal_marker_color.b = self.get_parameter('goal_marker_color_b').get_parameter_value().double_value
        self.goal_marker_color.a = self.get_parameter('goal_marker_color_a').get_parameter_value().double_value
        
        self.goal_marker_scale = self.get_parameter('goal_marker_scale').get_parameter_value().double_value
        
        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscribers for goal from RViz
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.rviz_goal_callback,
            qos_profile
        )
        
        # Create publishers
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.goal_topic,
            qos_profile
        )
        
        # Visualization publishers
        if self.enable_visualization:
            self.goal_marker_pub = self.create_publisher(
                Marker,
                '/goal_publisher/goal_marker',
                qos_profile
            )
        
        self.current_goal = None
        
        self.get_logger().info('Goal publisher node initialized')
    
    def rviz_goal_callback(self, msg):
        """Callback for goal from RViz"""
        self.get_logger().info(f'Received goal from RViz: ({msg.pose.position.x}, {msg.pose.position.y})')
        
        # Update current goal
        self.current_goal = msg
        
        # Republish goal
        self.goal_pub.publish(msg)
        
        # Visualize goal if enabled
        if self.enable_visualization:
            self.visualize_goal(msg)
    
    def visualize_goal(self, goal_msg):
        """Visualize the goal as a marker"""
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose = goal_msg.pose
        
        marker.scale.x = self.goal_marker_scale
        marker.scale.y = self.goal_marker_scale
        marker.scale.z = self.goal_marker_scale
        
        marker.color = self.goal_marker_color
        
        # Add a flag/arrow to the goal
        flag_marker = Marker()
        flag_marker.header.frame_id = self.map_frame_id
        flag_marker.header.stamp = self.get_clock().now().to_msg()
        flag_marker.ns = "goal_flag"
        flag_marker.id = 1
        flag_marker.type = Marker.ARROW
        flag_marker.action = Marker.ADD
        
        flag_marker.pose = goal_msg.pose
        flag_marker.pose.position.z += self.goal_marker_scale / 2.0
        
        flag_marker.scale.x = self.goal_marker_scale * 1.5
        flag_marker.scale.y = self.goal_marker_scale * 0.2
        flag_marker.scale.z = self.goal_marker_scale * 0.2
        
        flag_marker.color = self.goal_marker_color
        
        # Publish markers
        self.goal_marker_pub.publish(marker)
        self.goal_marker_pub.publish(flag_marker)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 