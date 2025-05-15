#!/usr/bin/env python3

import math
import numpy as np
from typing import List, Tuple, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class AutonomousDWAPlanner(Node):
    """
    Autonomous DWA Planner Node for ROS 2
    
    This node implements an autonomous navigation system using the DWA algorithm
    for obstacle avoidance without requiring a specific goal.
    It subscribes to sensor data (LaserScan, Odometry) and publishes velocity commands (Twist).
    It also provides visualization of the planning process in RViz2.
    """
    
    def __init__(self):
        super().__init__('autonomous_dwa_planner')
        
        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Robot parameters
                ('robot_radius', 0.3),                # Robot radius in meters
                ('max_linear_velocity', 0.5),         # Maximum linear velocity in m/s
                ('min_linear_velocity', 0.0),         # Minimum linear velocity in m/s
                ('max_angular_velocity', 1.0),        # Maximum angular velocity in rad/s
                ('min_angular_velocity', -1.0),       # Minimum angular velocity in rad/s
                ('max_linear_accel', 0.2),            # Maximum linear acceleration in m/s²
                ('max_angular_accel', 0.5),           # Maximum angular acceleration in rad/s²
                ('velocity_resolution', 0.01),        # Resolution for velocity sampling in m/s
                ('angular_velocity_resolution', 0.1),  # Resolution for angular velocity sampling in rad/s
                ('prediction_time', 3.0),             # Time to predict trajectory in seconds
                ('prediction_steps', 60),             # Number of steps to predict
                
                # Cost function weights
                ('obstacle_weight', 2.0),             # Weight for obstacle cost
                ('forward_weight', 1.0),              # Weight for forward motion preference
                ('heading_weight', 1.0),              # Weight for heading stability
                ('velocity_weight', 1.0),             # Weight for velocity preference
                
                # Navigation parameters
                ('preferred_direction', 0.0),         # Preferred direction in radians (0 = forward)
                ('clearance_threshold', 2.0),         # Minimum clearance to consider a path safe
                ('stop_threshold', 0.5),              # Distance to obstacle that triggers stopping
                
                # Update frequencies
                ('control_frequency', 10.0),          # Control loop frequency in Hz
                ('visualization_frequency', 5.0),     # Visualization update frequency in Hz
                
                # Frame IDs
                ('base_frame', 'base_link'),          # Robot base frame ID
                ('odom_frame', 'odom'),               # Odometry frame ID
                ('map_frame', 'map'),                 # Map frame ID
                
                # Visualization parameters
                ('show_trajectories', True),          # Whether to show predicted trajectories
                ('max_trajectories_shown', 20),       # Maximum number of trajectories to visualize
            ]
        )
        
        # Get parameters
        self.get_all_parameters()
        
        # Initialize robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0
        
        # Initialize laser scan data
        self.obstacle_distances = []
        self.obstacle_angles = []
        
        # Setup QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Setup subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Setup publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            reliable_qos
        )
        
        self.trajectory_pub = self.create_publisher(
            MarkerArray,
            'dwa/trajectories',
            reliable_qos
        )
        
        self.best_trajectory_pub = self.create_publisher(
            Path,
            'dwa/best_trajectory',
            reliable_qos
        )
        
        self.robot_pub = self.create_publisher(
            Marker,
            'dwa/robot',
            reliable_qos
        )
        
        self.clearance_pub = self.create_publisher(
            Marker,
            'dwa/clearance',
            reliable_qos
        )
        
        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Setup timers
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )
        
        self.visualization_timer = self.create_timer(
            1.0 / self.visualization_frequency,
            self.visualization_loop
        )
        
        self.get_logger().info('Autonomous DWA Planner node initialized')
        self.get_logger().info(f'Preferred direction: {self.preferred_direction} radians')
    
    def get_all_parameters(self):
        """Get all parameters from the parameter server"""
        # Robot parameters
        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.min_linear_velocity = self.get_parameter('min_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.min_angular_velocity = self.get_parameter('min_angular_velocity').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.velocity_resolution = self.get_parameter('velocity_resolution').value
        self.angular_velocity_resolution = self.get_parameter('angular_velocity_resolution').value
        self.prediction_time = self.get_parameter('prediction_time').value
        self.prediction_steps = self.get_parameter('prediction_steps').value
        
        # Cost function weights
        self.obstacle_weight = self.get_parameter('obstacle_weight').value
        self.forward_weight = self.get_parameter('forward_weight').value
        self.heading_weight = self.get_parameter('heading_weight').value
        self.velocity_weight = self.get_parameter('velocity_weight').value
        
        # Navigation parameters
        self.preferred_direction = self.get_parameter('preferred_direction').value
        self.clearance_threshold = self.get_parameter('clearance_threshold').value
        self.stop_threshold = self.get_parameter('stop_threshold').value
        
        # Update frequencies
        self.control_frequency = self.get_parameter('control_frequency').value
        self.visualization_frequency = self.get_parameter('visualization_frequency').value
        
        # Frame IDs
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Visualization parameters
        self.show_trajectories = self.get_parameter('show_trajectories').value
        self.max_trajectories_shown = self.get_parameter('max_trajectories_shown').value
    
    def laser_callback(self, msg: LaserScan):
        """Process incoming laser scan data"""
        # Convert laser scan to obstacle points in robot frame
        self.obstacle_distances = []
        self.obstacle_angles = []
        
        angle = msg.angle_min
        for i, distance in enumerate(msg.ranges):
            # Skip invalid measurements
            if distance < msg.range_min or distance > msg.range_max:
                angle += msg.angle_increment
                continue
            
            self.obstacle_distances.append(distance)
            self.obstacle_angles.append(angle)
            angle += msg.angle_increment
    
    def odom_callback(self, msg: Odometry):
        """Process incoming odometry data"""
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract orientation (yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Extract velocities
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
    
    def control_loop(self):
        """Main control loop for autonomous navigation"""
        if not self.obstacle_distances:
            self.get_logger().warn('No laser scan data received yet')
            return
        
        # Check for imminent collisions
        min_distance = min(self.obstacle_distances) if self.obstacle_distances else float('inf')
        if min_distance <= self.stop_threshold:
            self.get_logger().warn(f'Obstacle too close ({min_distance:.2f}m), stopping')
            self.publish_velocity(0.0, 0.0)
            return
        
        # Calculate dynamic window
        v_min = max(self.min_linear_velocity, self.v - self.max_linear_accel / self.control_frequency)
        v_max = min(self.max_linear_velocity, self.v + self.max_linear_accel / self.control_frequency)
        omega_min = max(self.min_angular_velocity, self.omega - self.max_angular_accel / self.control_frequency)
        omega_max = min(self.max_angular_velocity, self.omega + self.max_angular_accel / self.control_frequency)
        
        # Find best trajectory
        best_trajectory, best_v, best_omega = self.find_best_trajectory(v_min, v_max, omega_min, omega_max)
        
        # Publish velocity command
        if best_trajectory:
            self.publish_velocity(best_v, best_omega)
            
            # Publish best trajectory for visualization
            self.publish_best_trajectory(best_trajectory)
        else:
            # No valid trajectory found, stop the robot
            self.get_logger().warn('No valid trajectory found, stopping')
            self.publish_velocity(0.0, 0.0)
    
    def find_best_trajectory(self, v_min: float, v_max: float, omega_min: float, omega_max: float) -> Tuple[List[Tuple[float, float, float]], float, float]:
        """Find the best trajectory using DWA algorithm"""
        best_cost = float('inf')
        best_trajectory = None
        best_v = 0.0
        best_omega = 0.0
        
        # Sample velocities from the dynamic window
        all_trajectories = []
        
        for v in np.arange(v_min, v_max + self.velocity_resolution, self.velocity_resolution):
            for omega in np.arange(omega_min, omega_max + self.angular_velocity_resolution, self.angular_velocity_resolution):
                # Predict trajectory for this velocity pair
                trajectory = self.predict_trajectory(v, omega)
                
                # Calculate cost for this trajectory
                obstacle_cost = self.calculate_obstacle_cost(trajectory)
                forward_cost = self.calculate_forward_cost(v, omega)
                heading_cost = self.calculate_heading_cost(omega)
                velocity_cost = self.calculate_velocity_cost(v)
                
                # Skip trajectories that would lead to collision
                if obstacle_cost == float('inf'):
                    continue
                
                # Calculate total cost
                total_cost = (
                    self.obstacle_weight * obstacle_cost +
                    self.forward_weight * forward_cost +
                    self.heading_weight * heading_cost +
                    self.velocity_weight * velocity_cost
                )
                
                # Store trajectory for visualization
                all_trajectories.append((trajectory, total_cost, v, omega))
                
                # Update best trajectory if this one is better
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_trajectory = trajectory
                    best_v = v
                    best_omega = omega
        
        # Publish all trajectories for visualization
        if self.show_trajectories:
            self.publish_trajectories(all_trajectories)
        
        return best_trajectory, best_v, best_omega
    
    def predict_trajectory(self, v: float, omega: float) -> List[Tuple[float, float, float]]:
        """Predict trajectory for given velocity pair"""
        trajectory = []
        time_delta = self.prediction_time / self.prediction_steps
        
        # Start from current state
        x = 0.0  # Start from robot's local frame
        y = 0.0
        yaw = 0.0
        
        # Predict future positions
        for _ in range(self.prediction_steps):
            x += v * math.cos(yaw) * time_delta
            y += v * math.sin(yaw) * time_delta
            yaw += omega * time_delta
            trajectory.append((x, y, yaw))
        
        return trajectory
    
    def calculate_obstacle_cost(self, trajectory: List[Tuple[float, float, float]]) -> float:
        """Calculate cost based on distance to obstacles"""
        min_distance = float('inf')
        
        for x, y, _ in trajectory:
            # Check distance to each obstacle
            for i in range(len(self.obstacle_distances)):
                obstacle_x = self.obstacle_distances[i] * math.cos(self.obstacle_angles[i])
                obstacle_y = self.obstacle_distances[i] * math.sin(self.obstacle_angles[i])
                
                # Calculate distance between trajectory point and obstacle
                distance = math.sqrt((x - obstacle_x)**2 + (y - obstacle_y)**2)
                
                # Update minimum distance
                min_distance = min(min_distance, distance)
                
                # If collision detected, return infinite cost
                if distance <= self.robot_radius:
                    return float('inf')
        
        # Return cost inversely proportional to minimum distance
        return 1.0 / min_distance if min_distance < float('inf') else 0.0
    
    def calculate_forward_cost(self, v: float, omega: float) -> float:
        """Calculate cost based on alignment with preferred direction"""
        # Penalize trajectories that don't move in the preferred direction
        # Lower cost for trajectories that move in the preferred direction
        direction_diff = abs(omega)  # Penalize angular velocity
        
        # Encourage forward motion by penalizing negative velocity
        forward_penalty = 0.0 if v > 0.0 else 10.0
        
        return direction_diff + forward_penalty
    
    def calculate_heading_cost(self, omega: float) -> float:
        """Calculate cost based on heading stability"""
        # Penalize large changes in heading to promote smooth motion
        return abs(omega)
    
    def calculate_velocity_cost(self, v: float) -> float:
        """Calculate cost based on velocity (prefer higher velocities)"""
        # Return cost inversely proportional to velocity
        return self.max_linear_velocity - v
    
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def publish_velocity(self, v: float, omega: float):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)
    
    def publish_trajectories(self, trajectories: List[Tuple[List[Tuple[float, float, float]], float, float, float]]):
        """Publish trajectories for visualization"""
        # Sort trajectories by cost (ascending)
        sorted_trajectories = sorted(trajectories, key=lambda x: x[1])
        
        # Limit number of trajectories to visualize
        if len(sorted_trajectories) > self.max_trajectories_shown:
            sorted_trajectories = sorted_trajectories[:self.max_trajectories_shown]
        
        # Create marker array
        marker_array = MarkerArray()
        
        # Create a marker for each trajectory
        for i, (trajectory, cost, v, omega) in enumerate(sorted_trajectories):
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'trajectories'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02  # Line width
            
            # Color based on cost (green for low cost, red for high cost)
            # Normalize cost between 0 and 1
            if len(sorted_trajectories) > 1:
                normalized_cost = (cost - sorted_trajectories[0][1]) / (sorted_trajectories[-1][1] - sorted_trajectories[0][1])
            else:
                normalized_cost = 0.0
            
            marker.color.r = normalized_cost
            marker.color.g = 1.0 - normalized_cost
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            # Add points to the line strip
            for x, y, _ in trajectory:
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.1  # Slightly above ground for visibility
                marker.points.append(point)
            
            marker_array.markers.append(marker)
        
        # Publish marker array
        self.trajectory_pub.publish(marker_array)
    
    def publish_best_trajectory(self, trajectory: List[Tuple[float, float, float]]):
        """Publish best trajectory as Path message"""
        path = Path()
        path.header.frame_id = self.base_frame
        path.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, yaw in trajectory:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path.poses.append(pose)
        
        self.best_trajectory_pub.publish(path)
    
    def publish_robot_marker(self):
        """Publish robot marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2 * self.robot_radius
        marker.scale.y = 2 * self.robot_radius
        marker.scale.z = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        
        self.robot_pub.publish(marker)
    
    def publish_clearance_marker(self):
        """Publish clearance threshold marker"""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'clearance'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2 * self.clearance_threshold
        marker.scale.y = 2 * self.clearance_threshold
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.3
        
        self.clearance_pub.publish(marker)
    
    def visualization_loop(self):
        """Update visualization markers"""
        self.publish_robot_marker()
        self.publish_clearance_marker()


def main(args=None):
    rclpy.init(args=args)
    autonomous_dwa_planner = AutonomousDWAPlanner()
    rclpy.spin(autonomous_dwa_planner)
    autonomous_dwa_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
