#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
import time
import math
from scipy.interpolate import CubicSpline
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class FrenetPathSmoother(Node):
    def __init__(self):
        super().__init__('frenet_path_smoother')
        
        # Declare parameters
        self.declare_parameter('num_points', 100)
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('input_path_topic', '/hybrid_astar_path')
        self.declare_parameter('output_path_topic', '/smooth_path')
        
        # Get parameters
        self.num_points = self.get_parameter('num_points').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.input_path_topic = self.get_parameter('input_path_topic').value
        self.output_path_topic = self.get_parameter('output_path_topic').value
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.smooth_path_pub = self.create_publisher(
            Path, 
            self.output_path_topic, 
            qos_profile
        )
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            self.input_path_topic,
            self.path_callback,
            qos_profile
        )
        
        self.get_logger().info('Frenet Path Smoother node initialized')
    
    def path_callback(self, msg):
        """Process incoming path and publish smoothed version"""
        if not msg.poses or len(msg.poses) < 4:
            self.get_logger().warning('Received path has too few points for smoothing')
            return
            
        # Extract path points
        path_points = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            # Extract yaw from quaternion
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            
            # Convert quaternion to Euler angles
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            path_points.append((x, y, yaw))
        
        # Smooth the path
        start_time = time.time()
        smooth_path = self.smooth(path_points)
        smoothing_time = time.time() - start_time
        
        if not smooth_path:
            self.get_logger().warning('Failed to smooth path')
            return
            
        # Convert to ROS message and publish
        path_msg = self.smooth_path_to_msg(smooth_path)
        path_msg.header = msg.header  # Preserve original header
        self.smooth_path_pub.publish(path_msg)
        
        self.get_logger().info(f'Path smoothed in {smoothing_time:.3f} seconds')
    
    def smooth(self, path_points):
        """Smooth a path using Frenet approach with cubic splines"""
        if len(path_points) < 4:
            self.get_logger().warning("Not enough points to smooth")
            return path_points  # Return original path
        
        # Extract x, y coordinates
        x = [p[0] for p in path_points]
        y = [p[1] for p in path_points]
        
        try:
            # Calculate cumulative arc length (s coordinate in Frenet frame)
            s = [0]
            for i in range(1, len(x)):
                dx = x[i] - x[i-1]
                dy = y[i] - y[i-1]
                s.append(s[-1] + math.hypot(dx, dy))
            
            # Fit cubic splines
            sx = CubicSpline(s, x)
            sy = CubicSpline(s, y)
            
            # Sample the path smoothly
            s_new = np.linspace(0, s[-1], num=self.num_points)
            smooth_path = []
            
            # Create smooth path with position and calculated heading
            for i, si in enumerate(s_new):
                xi = float(sx(si))
                yi = float(sy(si))
                
                # Calculate heading (tangent to the path)
                if i < len(s_new) - 1:
                    next_si = s_new[i+1]
                    next_xi = float(sx(next_si))
                    next_yi = float(sy(next_si))
                    yaw = math.atan2(next_yi - yi, next_xi - xi)
                else:
                    # For the last point, use the same heading as the previous one
                    yaw = smooth_path[-1][2] if smooth_path else path_points[-1][2]
                
                smooth_path.append((xi, yi, yaw))
            
            return smooth_path
            
        except Exception as e:
            self.get_logger().error(f'Error during path smoothing: {e}')
            return None
    
    def smooth_path_to_msg(self, path):
        """Convert smoothed path to ROS Path message"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id
        
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            
            # Convert yaw to quaternion
            yaw = point[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
            
        return path_msg

def main(args=None):
    rclpy.init(args=args)
    node = FrenetPathSmoother()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 