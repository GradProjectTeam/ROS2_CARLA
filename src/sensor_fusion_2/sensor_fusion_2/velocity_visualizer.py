#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
import math

class VelocityVisualizer(Node):
    """
    A ROS2 node that visualizes velocity data from TwistStamped messages as arrows in RViz2.
    This provides a workaround for the missing Twist visualization plugin in RViz2.
    """
    
    def __init__(self):
        super().__init__('velocity_visualizer')
        
        # Declare parameters
        self.declare_parameter('input_topic', 'vehicle_velocity')
        self.declare_parameter('output_topic', 'velocity_visualization')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('scale_factor', 1.0)
        self.declare_parameter('arrow_shaft_diameter', 0.05)
        self.declare_parameter('arrow_head_diameter', 0.1)
        self.declare_parameter('max_velocity', 10.0)  # m/s, for color scaling
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.arrow_shaft_diameter = self.get_parameter('arrow_shaft_diameter').value
        self.arrow_head_diameter = self.get_parameter('arrow_head_diameter').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Create subscription to TwistStamped messages
        self.subscription = self.create_subscription(
            TwistStamped,
            self.input_topic,
            self.twist_callback,
            10)
        
        # Create publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.output_topic,
            10)
        
        # Log initialization
        self.get_logger().info(f"Velocity visualizer started. Subscribing to {self.input_topic}, publishing to {self.output_topic}")
    
    def twist_callback(self, msg):
        """
        Process incoming TwistStamped messages and publish visualization markers
        
        Args:
            msg: TwistStamped message
        """
        try:
            # Extract linear and angular velocity components
            linear = msg.twist.linear
            angular = msg.twist.angular
            
            # Create marker array
            marker_array = MarkerArray()
            
            # Create linear velocity arrow marker
            linear_marker = self.create_velocity_arrow(
                msg.header.stamp,
                msg.header.frame_id or self.frame_id,
                linear.x, linear.y, linear.z,
                "linear_velocity",
                0)
            marker_array.markers.append(linear_marker)
            
            # Create angular velocity arrow marker (if significant)
            angular_magnitude = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
            if angular_magnitude > 0.01:  # Only show if there's meaningful angular velocity
                angular_marker = self.create_angular_marker(
                    msg.header.stamp,
                    msg.header.frame_id or self.frame_id,
                    angular.x, angular.y, angular.z,
                    "angular_velocity",
                    1)
                marker_array.markers.append(angular_marker)
            
            # Create text marker showing speed
            speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
            text_marker = self.create_text_marker(
                msg.header.stamp,
                msg.header.frame_id or self.frame_id,
                speed,
                "velocity_text",
                2)
            marker_array.markers.append(text_marker)
            
            # Publish marker array
            self.marker_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f"Error in twist_callback: {str(e)}")
    
    def create_velocity_arrow(self, timestamp, frame_id, x, y, z, ns, id):
        """
        Create an arrow marker for linear velocity
        
        Args:
            timestamp: ROS timestamp
            frame_id: TF frame ID
            x, y, z: Velocity components
            ns: Namespace for the marker
            id: Marker ID
            
        Returns:
            Marker: Arrow marker
        """
        # Create arrow marker
        marker = Marker()
        marker.header.stamp = timestamp
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Start point at origin
        start_point = Point(x=0.0, y=0.0, z=0.0)
        marker.points.append(start_point)
        
        # End point based on velocity scaled by factor
        magnitude = math.sqrt(x**2 + y**2 + z**2)
        if magnitude < 0.001:  # Avoid division by zero
            end_point = Point(x=0.001, y=0.0, z=0.0)  # Minimal arrow
        else:
            end_point = Point(
                x=x * self.scale_factor,
                y=y * self.scale_factor,
                z=z * self.scale_factor
            )
        marker.points.append(end_point)
        
        # Set marker properties
        marker.scale.x = self.arrow_shaft_diameter  # Shaft diameter
        marker.scale.y = self.arrow_head_diameter   # Head diameter
        marker.scale.z = 0.0  # Not used for ARROW type
        
        # Set color based on speed (green to red)
        speed = math.sqrt(x**2 + y**2 + z**2)
        speed_ratio = min(1.0, speed / self.max_velocity)
        
        marker.color = ColorRGBA()
        marker.color.r = speed_ratio
        marker.color.g = 1.0 - speed_ratio
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Set lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 200ms
        
        return marker
    
    def create_angular_marker(self, timestamp, frame_id, x, y, z, ns, id):
        """
        Create a marker for angular velocity (circular arrow)
        
        Args:
            timestamp: ROS timestamp
            frame_id: TF frame ID
            x, y, z: Angular velocity components
            ns: Namespace for the marker
            id: Marker ID
            
        Returns:
            Marker: Angular velocity marker
        """
        # Create marker
        marker = Marker()
        marker.header.stamp = timestamp
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = id
        marker.type = Marker.CYLINDER  # Use cylinder as a simple representation
        marker.action = Marker.ADD
        
        # Set position at origin
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Set orientation (identity quaternion)
        marker.pose.orientation.w = 1.0
        
        # Calculate magnitude
        magnitude = math.sqrt(x**2 + y**2 + z**2)
        
        # Set size proportional to angular velocity
        radius = 0.2  # Base radius
        marker.scale.x = radius * 2  # Diameter
        marker.scale.y = radius * 2  # Diameter
        marker.scale.z = 0.02  # Height
        
        # Set color based on angular velocity magnitude
        angular_ratio = min(1.0, magnitude / 3.0)  # 3 rad/s is considered high
        
        marker.color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # Blue for angular velocity
        marker.color.a = 0.3 + 0.7 * angular_ratio  # More opaque for higher values
        
        # Set lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 200ms
        
        return marker
    
    def create_text_marker(self, timestamp, frame_id, speed, ns, id):
        """
        Create a text marker showing the current speed
        
        Args:
            timestamp: ROS timestamp
            frame_id: TF frame ID
            speed: Current speed in m/s
            ns: Namespace for the marker
            id: Marker ID
            
        Returns:
            Marker: Text marker
        """
        # Create text marker
        marker = Marker()
        marker.header.stamp = timestamp
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position text above the vehicle
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.3
        
        # Set orientation (identity quaternion)
        marker.pose.orientation.w = 1.0
        
        # Set text content
        marker.text = f"Speed: {speed:.2f} m/s"
        
        # Set text size
        marker.scale.z = 0.1  # Text height
        
        # Set color (white)
        marker.color = ColorRGBA()
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Set lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 200ms
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = VelocityVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 