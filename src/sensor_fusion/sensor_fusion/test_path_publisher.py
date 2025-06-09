#!/usr/bin/env python3
# Simple Path Publisher for Visualization

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Time

class TestPathPublisher(Node):
    def __init__(self):
        super().__init__('test_path_publisher')
        
        # Declare parameters
        self.declare_parameter('path_topic', '/test/path')
        self.declare_parameter('path_frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('path_color', [1.0, 1.0, 1.0, 1.0])  # RGBA
        
        # Get parameters
        self.path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('path_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Set up publisher
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        
        # Set up timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_path)
        
        # Path generation parameters
        self.path_radius = 10.0
        self.path_points = 100
        self.center_x = 0.0
        self.center_y = 0.0
        self.current_angle = 0.0
        self.angle_step = 0.01
        
        # Log initialization
        self.get_logger().info(f'Path publisher initialized on topic: {self.path_topic}')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz with frame_id: {self.frame_id}')
    
    def publish_path(self):
        """Generate and publish a path message"""
        # Create a new Path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        
        # Add poses to the path (create a simple spiral)
        for i in range(self.path_points):
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = self.frame_id
            
            # Calculate position along a spiral
            angle = i * 0.1 + self.current_angle
            radius = self.path_radius * (1.0 - i / self.path_points)
            
            pose.pose.position.x = self.center_x + radius * math.cos(angle)
            pose.pose.position.y = self.center_y + radius * math.sin(angle)
            pose.pose.position.z = 0.0
            
            # Simple orientation (facing direction of travel)
            pose.pose.orientation.w = 1.0
            
            # Add this pose to the path
            path_msg.poses.append(pose)
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        
        # Update the angle for the next iteration to create movement
        self.current_angle += self.angle_step
        
        # Occasionally log that we're publishing
        if int(self.get_clock().now().nanoseconds / 1e9) % 10 == 0:
            self.get_logger().debug(f'Published path with {len(path_msg.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    node = TestPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 