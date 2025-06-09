#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import math

class PathVisualizationNode(Node):
    def __init__(self):
        super().__init__('path_visualization_node')
        
        # Declare parameters - check if use_sim_time already exists
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
            
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('global_path_color_r', 0.0)
        self.declare_parameter('global_path_color_g', 0.0)
        self.declare_parameter('global_path_color_b', 1.0)
        self.declare_parameter('global_path_color_a', 0.8)
        self.declare_parameter('local_path_color_r', 0.0)
        self.declare_parameter('local_path_color_g', 1.0)
        self.declare_parameter('local_path_color_b', 0.0)
        self.declare_parameter('local_path_color_a', 1.0)
        self.declare_parameter('path_line_width', 0.1)
        self.declare_parameter('path_lifetime', 0.0)  # 0 means persistent
        
        # Get parameters
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.local_path_topic = self.get_parameter('local_path_topic').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        
        self.global_path_color = ColorRGBA()
        self.global_path_color.r = self.get_parameter('global_path_color_r').get_parameter_value().double_value
        self.global_path_color.g = self.get_parameter('global_path_color_g').get_parameter_value().double_value
        self.global_path_color.b = self.get_parameter('global_path_color_b').get_parameter_value().double_value
        self.global_path_color.a = self.get_parameter('global_path_color_a').get_parameter_value().double_value
        
        self.local_path_color = ColorRGBA()
        self.local_path_color.r = self.get_parameter('local_path_color_r').get_parameter_value().double_value
        self.local_path_color.g = self.get_parameter('local_path_color_g').get_parameter_value().double_value
        self.local_path_color.b = self.get_parameter('local_path_color_b').get_parameter_value().double_value
        self.local_path_color.a = self.get_parameter('local_path_color_a').get_parameter_value().double_value
        
        self.path_line_width = self.get_parameter('path_line_width').get_parameter_value().double_value
        self.path_lifetime = self.get_parameter('path_lifetime').get_parameter_value().double_value
        
        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscribers
        self.global_path_sub = self.create_subscription(
            Path,
            self.global_path_topic,
            self.global_path_callback,
            qos_profile
        )
        
        self.local_path_sub = self.create_subscription(
            Path,
            self.local_path_topic,
            self.local_path_callback,
            qos_profile
        )
        
        # Create publishers
        self.path_vis_pub = self.create_publisher(
            MarkerArray,
            '/path_visualization/paths',
            qos_profile
        )
        
        self.get_logger().info('Path visualization node initialized')
    
    def global_path_callback(self, msg):
        """Callback for global path updates"""
        self.get_logger().debug(f'Received global path with {len(msg.poses)} points')
        self.visualize_path(msg, 'global_path', self.global_path_color)
    
    def local_path_callback(self, msg):
        """Callback for local path updates"""
        self.get_logger().debug(f'Received local path with {len(msg.poses)} points')
        self.visualize_path(msg, 'local_path', self.local_path_color)
    
    def visualize_path(self, path_msg, namespace, color):
        """Visualize a path as a line strip marker"""
        if not path_msg.poses:
            return
        
        marker_array = MarkerArray()
        
        # Create line strip marker for path
        line_marker = Marker()
        line_marker.header.frame_id = self.map_frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = namespace
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # Set line properties
        line_marker.scale.x = self.path_line_width
        line_marker.color = color
        
        # Set lifetime (0 means persistent)
        if self.path_lifetime > 0.0:
            line_marker.lifetime.sec = int(self.path_lifetime)
            line_marker.lifetime.nanosec = int((self.path_lifetime - int(self.path_lifetime)) * 1e9)
        
        # Add points to line strip
        for pose in path_msg.poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z + 0.05  # Slightly above ground for visibility
            line_marker.points.append(point)
        
        marker_array.markers.append(line_marker)
        
        # Create sphere markers for waypoints
        for i, pose in enumerate(path_msg.poses):
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.map_frame_id
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = f"{namespace}_waypoints"
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            sphere_marker.pose.position.x = pose.pose.position.x
            sphere_marker.pose.position.y = pose.pose.position.y
            sphere_marker.pose.position.z = pose.pose.position.z + 0.05  # Slightly above ground
            
            sphere_marker.scale.x = self.path_line_width * 2.0
            sphere_marker.scale.y = self.path_line_width * 2.0
            sphere_marker.scale.z = self.path_line_width * 2.0
            
            sphere_marker.color = color
            
            # Set lifetime
            if self.path_lifetime > 0.0:
                sphere_marker.lifetime.sec = int(self.path_lifetime)
                sphere_marker.lifetime.nanosec = int((self.path_lifetime - int(self.path_lifetime)) * 1e9)
            
            marker_array.markers.append(sphere_marker)
        
        # Publish markers
        self.path_vis_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 