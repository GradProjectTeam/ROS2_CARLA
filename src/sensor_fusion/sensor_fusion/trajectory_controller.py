#!/usr/bin/env python3
# Trajectory Controller Node
# Follows a trajectory by outputting appropriate control commands

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # Declare parameters
        self.declare_parameter('trajectory_topic', '/planner/hybrid_astar/trajectory')
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_rate', 20.0)  # Hz
        self.declare_parameter('position_tolerance', 0.2)  # meters
        self.declare_parameter('heading_tolerance', 0.1)  # radians
        self.declare_parameter('lookahead_time', 0.5)  # seconds
        
        # Control gains
        self.declare_parameter('k_p_linear', 1.0)  # Proportional gain for linear velocity
        self.declare_parameter('k_i_linear', 0.1)  # Integral gain for linear velocity
        self.declare_parameter('k_d_linear', 0.05)  # Derivative gain for linear velocity
        self.declare_parameter('k_p_angular', 1.0)  # Proportional gain for angular velocity
        self.declare_parameter('k_i_angular', 0.1)  # Integral gain for angular velocity
        self.declare_parameter('k_d_angular', 0.05)  # Derivative gain for angular velocity
        
        # Get parameters
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.control_rate = self.get_parameter('control_rate').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.lookahead_time = self.get_parameter('lookahead_time').value
        
        # Get control gains
        self.k_p_linear = self.get_parameter('k_p_linear').value
        self.k_i_linear = self.get_parameter('k_i_linear').value
        self.k_d_linear = self.get_parameter('k_d_linear').value
        self.k_p_angular = self.get_parameter('k_p_angular').value
        self.k_i_angular = self.get_parameter('k_i_angular').value
        self.k_d_angular = self.get_parameter('k_d_angular').value
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.trajectory_callback,
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile
        )
        
        # Publisher
        self.control_pub = self.create_publisher(
            Twist,
            self.control_topic,
            qos_profile
        )
        
        # State variables
        self.current_trajectory = None
        self.current_odom = None
        self.last_control_time = None
        
        # PID controller state
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_callback
        )
        
        self.get_logger().info('Trajectory Controller initialized')
        self.get_logger().info(f'Subscribing to trajectory topic: {self.trajectory_topic}')
        self.get_logger().info(f'Publishing control commands to: {self.control_topic}')
    
    def trajectory_callback(self, msg):
        """Process incoming trajectory message"""
        if len(msg.points) == 0:
            self.get_logger().warn('Received empty trajectory')
            return
        
        self.current_trajectory = msg
        self.get_logger().debug(f'Received trajectory with {len(msg.points)} points')
    
    def odom_callback(self, msg):
        """Process incoming odometry message"""
        self.current_odom = msg
    
    def control_callback(self):
        """Main control loop that runs at control_rate frequency"""
        if self.current_trajectory is None or self.current_odom is None:
            self.get_logger().debug('Waiting for trajectory and odometry data')
            return
        
        # Get current time
        current_time = self.get_clock().now()
        
        # Get current pose from odometry
        current_pose = self.current_odom.pose.pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        
        # Get current heading from quaternion
        qx = current_pose.orientation.x
        qy = current_pose.orientation.y
        qz = current_pose.orientation.z
        qw = current_pose.orientation.w
        current_heading = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Get current velocities
        current_twist = self.current_odom.twist.twist
        current_linear_velocity = current_twist.linear.x
        current_angular_velocity = current_twist.angular.z
        
        # Find the closest point on the trajectory and the lookahead point
        target_point = self.get_lookahead_point(current_x, current_y, current_heading)
        
        if target_point is None:
            self.get_logger().warn('No valid target point found on trajectory')
            self.stop_vehicle()
            return
        
        # Extract target state
        target_x = target_point.positions[0]
        target_y = target_point.positions[1]
        target_heading = target_point.positions[2]
        target_velocity = target_point.velocities[3]
        target_steering = target_point.velocities[4]
        
        # Calculate errors
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        heading_error = self.normalize_angle(target_heading - current_heading)
        velocity_error = target_velocity - current_linear_velocity
        
        # Calculate time step
        if self.last_control_time is None:
            dt = 1.0 / self.control_rate
        else:
            dt = (current_time.nanoseconds - self.last_control_time.nanoseconds) / 1e9
            dt = max(dt, 0.001)  # Prevent division by zero
        
        self.last_control_time = current_time
        
        # PID control for linear velocity
        self.linear_error_sum += velocity_error * dt
        linear_error_derivative = (velocity_error - self.last_linear_error) / dt
        
        linear_cmd = (self.k_p_linear * velocity_error + 
                      self.k_i_linear * self.linear_error_sum +
                      self.k_d_linear * linear_error_derivative)
        
        # Cap the linear command based on the target velocity
        linear_cmd = max(0.0, min(linear_cmd, target_velocity * 1.1))
        
        # PID control for angular velocity
        # Using the bicycle model: angular_vel = velocity * tan(steering) / wheelbase
        target_angular = target_velocity * math.tan(target_steering) / 2.7  # Using default wheelbase
        angular_error = target_angular - current_angular_velocity
        
        self.angular_error_sum += angular_error * dt
        angular_error_derivative = (angular_error - self.last_angular_error) / dt
        
        angular_cmd = (self.k_p_angular * angular_error + 
                       self.k_i_angular * self.angular_error_sum +
                       self.k_d_angular * angular_error_derivative)
        
        # Update error history
        self.last_linear_error = velocity_error
        self.last_angular_error = angular_error
        
        # Create and publish control command
        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        
        self.control_pub.publish(cmd)
        
        self.get_logger().debug(f'Control: linear={linear_cmd:.2f}, angular={angular_cmd:.2f}')
        self.get_logger().debug(f'Errors: vel={velocity_error:.2f}, heading={heading_error:.2f}, dist={distance:.2f}')
    
    def get_lookahead_point(self, current_x, current_y, current_heading):
        """
        Find a point on the trajectory that is approximately lookahead_time seconds ahead
        """
        if len(self.current_trajectory.points) == 0:
            return None
        
        # Find the closest point first
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(self.current_trajectory.points):
            x = point.positions[0]
            y = point.positions[1]
            dist = math.sqrt((x - current_x)**2 + (y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # If we're at the end of the trajectory, use the final point
        if closest_idx >= len(self.current_trajectory.points) - 1:
            return self.current_trajectory.points[-1]
        
        # Look ahead by time
        current_time = self.get_clock().now()
        lookahead_time = current_time.nanoseconds / 1e9 + self.lookahead_time
        
        # Find the point that's approximately lookahead_time ahead
        for i in range(closest_idx + 1, len(self.current_trajectory.points)):
            point = self.current_trajectory.points[i]
            
            # Convert time_from_start to absolute time
            point_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            current_point_time = self.current_trajectory.points[closest_idx].time_from_start.sec + \
                                 self.current_trajectory.points[closest_idx].time_from_start.nanosec / 1e9
            
            if point_time - current_point_time >= self.lookahead_time:
                return point
        
        # If no suitable point found, use the last point
        return self.current_trajectory.points[-1]
    
    def stop_vehicle(self):
        """Send a command to stop the vehicle"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.control_pub.publish(cmd)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 