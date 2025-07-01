#!/usr/bin/env python3
"""
DWA-MPC Bridge Node

This node acts as a bridge between the Enhanced DWA Planner and MPC Controller.
It subscribes to velocity commands from both controllers and decides which one
to forward to the vehicle based on certain conditions.

Author: Mostafa
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Path
import math
import numpy as np
from std_msgs.msg import Bool, Float32


class DWAMPCBridge(Node):
    def __init__(self):
        super().__init__('dwa_mpc_bridge')
        
        # Declare parameters
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('dwa_cmd_topic', '/dwa/cmd_vel')
        self.declare_parameter('mpc_cmd_topic', '/mpc/cmd_vel')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('dwa_path_topic', '/dwa/planned_path')
        self.declare_parameter('min_path_length', 5)  # Minimum number of points in path
        self.declare_parameter('min_path_distance', 10.0)  # Minimum distance covered by path
        self.declare_parameter('max_path_angle', 0.5)  # Maximum angle change in path (radians)
        self.declare_parameter('hysteresis_threshold', 0.2)  # Hysteresis threshold
        self.declare_parameter('blend_time', 1.0)  # Time to blend between controllers
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.dwa_cmd_topic = self.get_parameter('dwa_cmd_topic').value
        self.mpc_cmd_topic = self.get_parameter('mpc_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self.dwa_path_topic = self.get_parameter('dwa_path_topic').value
        self.min_path_length = self.get_parameter('min_path_length').value
        self.min_path_distance = self.get_parameter('min_path_distance').value
        self.max_path_angle = self.get_parameter('max_path_angle').value
        self.hysteresis_threshold = self.get_parameter('hysteresis_threshold').value
        self.blend_time = self.get_parameter('blend_time').value
        
        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.use_mpc_publisher = self.create_publisher(Bool, '/use_mpc', 10)
        self.blend_factor_publisher = self.create_publisher(Float32, '/blend_factor', 10)
        
        # Create subscribers
        self.dwa_cmd_subscription = self.create_subscription(
            Twist,
            self.dwa_cmd_topic,
            self.dwa_cmd_callback,
            10)
            
        self.mpc_cmd_subscription = self.create_subscription(
            Twist,
            self.mpc_cmd_topic,
            self.mpc_cmd_callback,
            10)
            
        self.dwa_path_subscription = self.create_subscription(
            Path,
            self.dwa_path_topic,
            self.dwa_path_callback,
            10)
        
        # Initialize variables
        self.dwa_cmd = Twist()
        self.mpc_cmd = Twist()
        self.last_dwa_cmd_time = self.get_clock().now()
        self.last_mpc_cmd_time = self.get_clock().now()
        self.use_mpc = False
        self.blend_factor = 0.0  # 0.0 = DWA, 1.0 = MPC
        self.blend_start_time = None
        self.dwa_path_quality = 0.0  # 0.0 = poor, 1.0 = excellent
        
        # Create timer for control selection and blending
        self.timer = self.create_timer(1.0/self.update_rate, self.control_selection_callback)
        
        self.get_logger().info('DWA-MPC Bridge initialized - Using only DWA purple path')
    
    def dwa_cmd_callback(self, msg):
        """Callback for DWA command velocities"""
        self.dwa_cmd = msg
        self.last_dwa_cmd_time = self.get_clock().now()
    
    def mpc_cmd_callback(self, msg):
        """Callback for MPC command velocities"""
        self.mpc_cmd = msg
        self.last_mpc_cmd_time = self.get_clock().now()
    
    def dwa_path_callback(self, msg):
        """Callback for DWA planned path"""
        # Analyze path quality
        path_quality = self.analyze_path_quality(msg)
        self.dwa_path_quality = path_quality
        
        # Determine if we should use MPC based on path quality
        should_use_mpc = path_quality > (0.5 + self.hysteresis_threshold) if not self.use_mpc else path_quality > (0.5 - self.hysteresis_threshold)
        
        # If state is changing, start blending
        if should_use_mpc != self.use_mpc:
            self.blend_start_time = self.get_clock().now()
            self.get_logger().info(f"Switching to {'MPC' if should_use_mpc else 'DWA'} control")
        
        self.use_mpc = should_use_mpc
        
        # Publish use_mpc flag
        use_mpc_msg = Bool()
        use_mpc_msg.data = self.use_mpc
        self.use_mpc_publisher.publish(use_mpc_msg)
    
    def analyze_path_quality(self, path_msg):
        """Analyze the quality of the DWA path to determine if it's suitable for MPC"""
        if len(path_msg.poses) < self.min_path_length:
            self.get_logger().debug(f"Path too short: {len(path_msg.poses)} points")
            return 0.0
        
        # Calculate path length
        path_length = 0.0
        for i in range(1, len(path_msg.poses)):
            p1 = path_msg.poses[i-1].pose.position
            p2 = path_msg.poses[i].pose.position
            path_length += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        
        # Calculate maximum angle change along the path
        max_angle_change = 0.0
        if len(path_msg.poses) >= 3:
            for i in range(2, len(path_msg.poses)):
                p1 = path_msg.poses[i-2].pose.position
                p2 = path_msg.poses[i-1].pose.position
                p3 = path_msg.poses[i].pose.position
                
                # Calculate vectors
                v1 = np.array([p2.x - p1.x, p2.y - p1.y])
                v2 = np.array([p3.x - p2.x, p3.y - p2.y])
                
                # Normalize vectors
                v1_norm = np.linalg.norm(v1)
                v2_norm = np.linalg.norm(v2)
                
                if v1_norm > 0.001 and v2_norm > 0.001:
                    v1 = v1 / v1_norm
                    v2 = v2 / v2_norm
                    
                    # Calculate angle between vectors
                    dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
                    angle_change = math.acos(dot_product)
                    max_angle_change = max(max_angle_change, angle_change)
        
        # Calculate path quality score (0.0 to 1.0)
        length_score = min(1.0, path_length / self.min_path_distance)
        angle_score = 1.0 - min(1.0, max_angle_change / self.max_path_angle)
        
        # Combine scores
        quality = 0.6 * length_score + 0.4 * angle_score
        
        self.get_logger().debug(f"Path quality: {quality:.2f} (length: {path_length:.2f}m, max angle: {max_angle_change:.2f}rad)")
        return quality
    
    def control_selection_callback(self):
        """Select and blend control commands"""
        now = self.get_clock().now()
        
        # Check if we have recent commands
        dwa_age = (now.nanoseconds - self.last_dwa_cmd_time.nanoseconds) / 1e9
        mpc_age = (now.nanoseconds - self.last_mpc_cmd_time.nanoseconds) / 1e9
        
        if dwa_age > 1.0 and mpc_age > 1.0:
            # No recent commands, publish zero velocity
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
            self.get_logger().warn("No recent commands from either controller")
            return
        
        # Calculate blend factor
        if self.blend_start_time is not None:
            blend_time = (now.nanoseconds - self.blend_start_time.nanoseconds) / 1e9
            if blend_time < self.blend_time:
                # Smooth transition
                if self.use_mpc:
                    self.blend_factor = blend_time / self.blend_time
                else:
                    self.blend_factor = 1.0 - (blend_time / self.blend_time)
            else:
                # Transition complete
                self.blend_factor = 1.0 if self.use_mpc else 0.0
                self.blend_start_time = None
        else:
            # No transition in progress
            self.blend_factor = 1.0 if self.use_mpc else 0.0
        
        # Publish blend factor
        blend_factor_msg = Float32()
        blend_factor_msg.data = self.blend_factor
        self.blend_factor_publisher.publish(blend_factor_msg)
        
        # Blend commands
        cmd = Twist()
        cmd.linear.x = (1.0 - self.blend_factor) * self.dwa_cmd.linear.x + self.blend_factor * self.mpc_cmd.linear.x
        cmd.angular.z = (1.0 - self.blend_factor) * self.dwa_cmd.angular.z + self.blend_factor * self.mpc_cmd.angular.z
        
        # Publish blended command
        self.cmd_vel_publisher.publish(cmd)
        
        controller = "MPC" if self.blend_factor > 0.5 else "DWA"
        self.get_logger().debug(f"Using {controller} control (blend: {self.blend_factor:.2f})")
    
    def cleanup(self):
        """Cleanup resources"""
        # Publish zero velocity before shutting down
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info("Cleaning up DWA-MPC Bridge")


def main(args=None):
    rclpy.init(args=args)
    
    node = DWAMPCBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 