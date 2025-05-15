#!/usr/bin/env python3
# Trajectory Visualizer Node
# Converts JointTrajectory messages to MarkerArray for RViz visualization

import rclpy
from rclpy.node import Node
import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Twist
from std_msgs.msg import ColorRGBA, Header
from builtin_interfaces.msg import Duration

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Declare parameters
        self.declare_parameter('vehicle_width', 2.0)  # meters
        self.declare_parameter('vehicle_length', 4.0)  # meters
        self.declare_parameter('wheelbase', 2.7)  # meters
        self.declare_parameter('arrow_scale', 0.5)  # size of velocity/steering arrows
        
        # Get parameters
        self.vehicle_width = self.get_parameter('vehicle_width').value
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.arrow_scale = self.get_parameter('arrow_scale').value
        
        # Set up subscribers and publishers for each planner
        planners = ['hybrid_astar', 'astar', 'dwa', 'mpc']
        planner_colors = {
            'hybrid_astar': [0.1, 1.0, 0.1],  # Green
            'astar': [0.1, 0.1, 1.0],         # Blue
            'dwa': [1.0, 0.1, 0.1],           # Red
            'mpc': [1.0, 0.1, 1.0]            # Purple
        }
        
        self.trajectory_subscribers = {}
        self.marker_publishers = {}
        
        for planner in planners:
            self.trajectory_subscribers[planner] = self.create_subscription(
                JointTrajectory,
                f'/planner/{planner}/trajectory',
                lambda msg, p=planner: self.trajectory_callback(msg, p, planner_colors[p]),
                10
            )
            
            self.marker_publishers[planner] = self.create_publisher(
                MarkerArray,
                f'/planner/{planner}/trajectory_viz',
                10
            )
        
        # Subscribe to control commands for visualization
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for control command visualization
        self.control_viz_pub = self.create_publisher(
            Marker,
            '/control_viz',
            10
        )
        
        self.get_logger().info('Trajectory Visualizer initialized')
    
    def trajectory_callback(self, msg, planner_name, color):
        """
        Convert a JointTrajectory message to a MarkerArray visualization
        
        Args:
            msg (JointTrajectory): The input trajectory message
            planner_name (str): The name of the planner that produced the trajectory
            color (list): RGB color values for this planner's visualization
        """
        if len(msg.points) == 0:
            self.get_logger().debug(f'Received empty trajectory from {planner_name}')
            return
        
        self.get_logger().debug(f'Visualizing trajectory from {planner_name} with {len(msg.points)} points')
        
        marker_array = MarkerArray()
        
        # 1. Create path line marker
        line_marker = Marker()
        line_marker.header = msg.header
        line_marker.ns = f"{planner_name}_trajectory_path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.1  # Line width
        
        # Set color with 80% opacity
        line_marker.color.r = color[0]
        line_marker.color.g = color[1]
        line_marker.color.b = color[2]
        line_marker.color.a = 0.8
        
        # Add all points to the line
        for point in msg.points:
            p = Point()
            p.x = point.positions[0]  # x
            p.y = point.positions[1]  # y
            p.z = 0.1  # Slightly above ground
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # 2. Create vehicle pose markers at each point
        for i, point in enumerate(msg.points):
            # Skip some points for clarity if there are many
            if len(msg.points) > 20 and i % 2 != 0 and i != len(msg.points) - 1:
                continue
            
            # Get position and orientation
            x = point.positions[0]
            y = point.positions[1]
            heading = point.positions[2]
            
            # Create vehicle outline
            vehicle_marker = Marker()
            vehicle_marker.header = msg.header
            vehicle_marker.ns = f"{planner_name}_vehicle"
            vehicle_marker.id = i
            vehicle_marker.type = Marker.LINE_LIST
            vehicle_marker.action = Marker.ADD
            
            # Set pose - centered at position with proper orientation
            vehicle_marker.pose.position.x = x
            vehicle_marker.pose.position.y = y
            vehicle_marker.pose.position.z = 0.1
            
            # Convert heading to quaternion
            vehicle_marker.pose.orientation.x = 0.0
            vehicle_marker.pose.orientation.y = 0.0
            vehicle_marker.pose.orientation.z = math.sin(heading / 2.0)
            vehicle_marker.pose.orientation.w = math.cos(heading / 2.0)
            
            # Set size
            vehicle_marker.scale.x = 0.05  # Line width
            
            # Set color
            opacity = 0.5 if i < len(msg.points) - 1 else 0.9  # More opacity for last point
            vehicle_marker.color.r = color[0]
            vehicle_marker.color.g = color[1]
            vehicle_marker.color.b = color[2]
            vehicle_marker.color.a = opacity
            
            # Create vehicle outline points
            half_length = self.vehicle_length / 2.0
            half_width = self.vehicle_width / 2.0
            
            # Define the corner points of the vehicle rectangle in the vehicle frame
            fl = Point(x=half_length, y=half_width, z=0)
            fr = Point(x=half_length, y=-half_width, z=0)
            bl = Point(x=-half_length, y=half_width, z=0)
            br = Point(x=-half_length, y=-half_width, z=0)
            
            # Connect the corners to form the vehicle outline
            vehicle_marker.points.extend([fl, fr, fr, br, br, bl, bl, fl])
            
            # Add a heading indicator line
            heading_marker = Marker()
            heading_marker.header = msg.header
            heading_marker.ns = f"{planner_name}_heading"
            heading_marker.id = i
            heading_marker.type = Marker.ARROW
            heading_marker.action = Marker.ADD
            
            # Set pose - at vehicle center
            heading_marker.pose.position.x = x
            heading_marker.pose.position.y = y
            heading_marker.pose.position.z = 0.15
            
            # Set orientation same as vehicle
            heading_marker.pose.orientation = vehicle_marker.pose.orientation
            
            # Arrow size - forward pointing
            heading_marker.scale.x = self.vehicle_length * 0.8
            heading_marker.scale.y = 0.2
            heading_marker.scale.z = 0.1
            
            # Slightly darker color
            heading_marker.color.r = color[0] * 0.8
            heading_marker.color.g = color[1] * 0.8
            heading_marker.color.b = color[2] * 0.8
            heading_marker.color.a = opacity + 0.1
            
            # Add the vehicle and heading markers
            marker_array.markers.append(vehicle_marker)
            marker_array.markers.append(heading_marker)
            
            # If the point has velocity/steering data, add those indicators
            if len(point.velocities) >= 5:
                velocity = point.velocities[3]
                steering_angle = point.velocities[4]
                
                if abs(velocity) > 0.1:
                    # Velocity indicator (forward/backward arrow)
                    vel_marker = Marker()
                    vel_marker.header = msg.header
                    vel_marker.ns = f"{planner_name}_velocity"
                    vel_marker.id = i
                    vel_marker.type = Marker.ARROW
                    vel_marker.action = Marker.ADD
                    
                    # Position at vehicle center
                    vel_marker.pose.position.x = x
                    vel_marker.pose.position.y = y
                    vel_marker.pose.position.z = 0.3
                    
                    # Same orientation as vehicle
                    vel_marker.pose.orientation = vehicle_marker.pose.orientation
                    
                    # Size proportional to velocity
                    vel_scale = min(1.0, abs(velocity) / 3.0) * self.arrow_scale
                    vel_marker.scale.x = self.vehicle_length * vel_scale
                    vel_marker.scale.y = 0.3
                    vel_marker.scale.z = 0.1
                    
                    # Color - using velocity-based hue (green to red)
                    vel_color = self.velocity_to_color(velocity)
                    vel_marker.color.r = vel_color[0]
                    vel_marker.color.g = vel_color[1]
                    vel_marker.color.b = vel_color[2]
                    vel_marker.color.a = 0.8
                    
                    marker_array.markers.append(vel_marker)
                
                if abs(steering_angle) > 0.01:
                    # Steering indicator (small side arrow or wheel visualization)
                    steer_marker = Marker()
                    steer_marker.header = msg.header
                    steer_marker.ns = f"{planner_name}_steering"
                    steer_marker.id = i
                    steer_marker.type = Marker.ARROW
                    steer_marker.action = Marker.ADD
                    
                    # Position at front of vehicle
                    steer_marker.pose.position.x = x + (half_length * 0.8) * math.cos(heading)
                    steer_marker.pose.position.y = y + (half_length * 0.8) * math.sin(heading)
                    steer_marker.pose.position.z = 0.2
                    
                    # Orientation: vehicle heading + steering angle
                    steer_angle = heading + steering_angle
                    steer_marker.pose.orientation.x = 0.0
                    steer_marker.pose.orientation.y = 0.0
                    steer_marker.pose.orientation.z = math.sin(steer_angle / 2.0)
                    steer_marker.pose.orientation.w = math.cos(steer_angle / 2.0)
                    
                    # Size
                    steer_marker.scale.x = self.vehicle_width * 0.6
                    steer_marker.scale.y = 0.1
                    steer_marker.scale.z = 0.1
                    
                    # Color - blue for left, yellow for right
                    if steering_angle > 0:
                        steer_marker.color.r = 0.1
                        steer_marker.color.g = 0.3
                        steer_marker.color.b = 0.9
                    else:
                        steer_marker.color.r = 0.9
                        steer_marker.color.g = 0.9
                        steer_marker.color.b = 0.1
                    steer_marker.color.a = 0.7
                    
                    marker_array.markers.append(steer_marker)
        
        # 3. Create time indicator markers
        for i, point in enumerate(msg.points):
            if i % 5 != 0 and i != len(msg.points) - 1:  # Show every fifth point and the last one
                continue
                
            time_marker = Marker()
            time_marker.header = msg.header
            time_marker.ns = f"{planner_name}_time"
            time_marker.id = i
            time_marker.type = Marker.TEXT_VIEW_FACING
            time_marker.action = Marker.ADD
            
            # Position slightly above the path
            time_marker.pose.position.x = point.positions[0]
            time_marker.pose.position.y = point.positions[1]
            time_marker.pose.position.z = 0.5
            
            # Set size
            time_marker.scale.z = 0.3  # Text height
            
            # Set color
            time_marker.color.r = 1.0
            time_marker.color.g = 1.0
            time_marker.color.b = 1.0
            time_marker.color.a = 0.8
            
            # Format time as seconds
            time_sec = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            time_marker.text = f"{time_sec:.1f}s"
            
            marker_array.markers.append(time_marker)
        
        # Publish all markers
        self.marker_publishers[planner_name].publish(marker_array)
    
    def cmd_vel_callback(self, msg):
        """
        Visualize the current control command
        
        Args:
            msg (Twist): The control command message
        """
        # Create a marker to show the current control command
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "control_command"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at current vehicle position (use TF in a real system)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        
        # Set orientation based on angular velocity
        if abs(msg.angular.z) > 0.01:
            # Calculate the angle of rotation based on the angular velocity
            angle = math.atan2(msg.angular.z, msg.linear.x)
            marker.pose.orientation.z = math.sin(angle / 2.0)
            marker.pose.orientation.w = math.cos(angle / 2.0)
        else:
            # Just point forward
            marker.pose.orientation.w = 1.0
        
        # Size based on velocity magnitude
        vel_magnitude = math.sqrt(msg.linear.x**2 + msg.linear.y**2)
        marker.scale.x = vel_magnitude * 2.0  # Length
        marker.scale.y = 0.3  # Width
        marker.scale.z = 0.1  # Height
        
        # Color based on velocity - green for forward, red for reverse
        if msg.linear.x >= 0:
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2
        else:
            marker.color.r = 0.8
            marker.color.g = 0.2
            marker.color.b = 0.2
        marker.color.a = 1.0
        
        # Publish the marker
        self.control_viz_pub.publish(marker)
    
    def velocity_to_color(self, velocity):
        """
        Convert velocity to a color gradient
        
        Args:
            velocity (float): Velocity value
            
        Returns:
            list: RGB color values
        """
        # Normalize velocity to range [0, 1]
        max_vel = 3.0  # m/s, adjust based on your vehicle
        norm_vel = min(1.0, abs(velocity) / max_vel)
        
        # Green (low velocity) to yellow to red (high velocity) gradient
        if norm_vel < 0.5:
            # Green to yellow
            return [2.0 * norm_vel, 1.0, 0.0]
        else:
            # Yellow to red
            return [1.0, 2.0 * (1.0 - norm_vel), 0.0]

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 