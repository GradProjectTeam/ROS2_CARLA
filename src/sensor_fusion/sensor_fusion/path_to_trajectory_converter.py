#!/usr/bin/env python3
# Path to Trajectory Converter Node
# Converts path messages from path planners to trajectory messages with velocity and time information

import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Path
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

class PathToTrajectoryConverter(Node):
    def __init__(self):
        super().__init__('path_to_trajectory_converter')
        
        # Declare parameters
        self.declare_parameter('max_velocity', 2.0)  # m/s
        self.declare_parameter('max_acceleration', 1.0)  # m/s^2
        self.declare_parameter('target_dt', 0.1)  # seconds between trajectory points
        self.declare_parameter('wheelbase', 2.7)  # meters
        self.declare_parameter('vehicle_width', 2.0)  # meters
        self.declare_parameter('vehicle_length', 4.0)  # meters
        self.declare_parameter('min_radius', 3.0)  # minimum turning radius in meters
        
        # Get parameters
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.target_dt = self.get_parameter('target_dt').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.vehicle_width = self.get_parameter('vehicle_width').value
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.min_radius = self.get_parameter('min_radius').value
        
        # Set up subscribers and publishers for each planner
        planners = ['hybrid_astar', 'astar', 'dwa', 'mpc']
        
        self.path_subscribers = {}
        self.trajectory_publishers = {}
        
        for planner in planners:
            self.path_subscribers[planner] = self.create_subscription(
                Path,
                f'/planner/{planner}/path',
                lambda msg, p=planner: self.path_callback(msg, p),
                10
            )
            
            self.trajectory_publishers[planner] = self.create_publisher(
                JointTrajectory,
                f'/planner/{planner}/trajectory',
                10
            )
        
        # Create control velocity publisher for all trajectories
        self.control_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Path to Trajectory Converter initialized')
    
    def path_callback(self, msg, planner_name):
        """
        Convert a Path message to a JointTrajectory message
        
        Args:
            msg (Path): The input path message
            planner_name (str): The name of the planner that produced the path
        """
        if len(msg.poses) == 0:
            self.get_logger().debug(f'Received empty path from {planner_name}')
            return
        
        self.get_logger().debug(f'Converting path from {planner_name} with {len(msg.poses)} points')
        
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.header = msg.header
        
        # For vehicle control, we'll use these joint names to represent control dimensions
        trajectory.joint_names = ['x', 'y', 'heading', 'velocity', 'steering_angle']
        
        # Process the path points
        points = []
        total_time = 0.0
        prev_pose = None
        
        for i, pose in enumerate(msg.poses):
            point = JointTrajectoryPoint()
            
            # Basic position
            x = pose.pose.position.x 
            y = pose.pose.position.y
            
            # Extract heading from quaternion
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            
            # Convert quaternion to Euler angles (yaw/heading)
            heading = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # Calculate velocities and steering angles
            if prev_pose is not None:
                dx = x - prev_pose[0]
                dy = y - prev_pose[1]
                d_heading = self.normalize_angle(heading - prev_pose[2])
                
                # Distance between points
                distance = math.sqrt(dx**2 + dy**2)
                
                # Estimate time needed to traverse this segment based on distance
                # Use a trapezoidal velocity profile for natural acceleration/deceleration
                if i == 1:  # Accelerating from start
                    # Estimate time based on acceleration
                    segment_time = math.sqrt(2.0 * distance / self.max_acceleration)
                    segment_time = max(segment_time, self.target_dt)  # Ensure minimum time
                    
                    # Calculate velocity based on acceleration
                    velocity = self.max_acceleration * segment_time
                    velocity = min(velocity, self.max_velocity)  # Cap at max velocity
                    
                elif i == len(msg.poses) - 1:  # Decelerating to end
                    # Deceleration time
                    prev_velocity = points[-1].velocities[3]
                    segment_time = prev_velocity / self.max_acceleration
                    segment_time = max(segment_time, self.target_dt)  # Ensure minimum time
                    
                    # Final velocity is zero
                    velocity = 0.0
                    
                else:  # Cruising in the middle
                    # Calculate curvature to determine appropriate velocity
                    if i < len(msg.poses) - 1:
                        next_x = msg.poses[i+1].pose.position.x
                        next_y = msg.poses[i+1].pose.position.y
                        
                        # Simple curvature estimation using three points
                        curvature = self.estimate_curvature(
                            prev_pose[0], prev_pose[1], 
                            x, y, 
                            next_x, next_y
                        )
                        
                        # Adjust velocity based on curvature (slower in curves)
                        if abs(curvature) > 0.001:  # Avoid division by zero
                            radius = 1.0 / abs(curvature)
                            # Limit velocity in curves based on comfortable lateral acceleration
                            curve_max_velocity = min(
                                self.max_velocity,
                                math.sqrt(self.max_acceleration * radius)
                            )
                            velocity = min(self.max_velocity, curve_max_velocity)
                        else:
                            velocity = self.max_velocity
                    else:
                        velocity = self.max_velocity
                    
                    # Calculate time needed at this velocity
                    segment_time = distance / velocity
                    segment_time = max(segment_time, self.target_dt)  # Ensure minimum time
                
                # Calculate steering angle based on vehicle kinematics
                if abs(d_heading) > 0.01 and distance > 0.01:
                    # Approximate radius of turn
                    turn_radius = distance / abs(d_heading)
                    
                    # Apply minimum turning radius constraint
                    turn_radius = max(turn_radius, self.min_radius)
                    
                    # Calculate steering angle using bicycle model
                    steering_angle = math.atan(self.wheelbase / turn_radius)
                    
                    # Apply sign based on turning direction
                    if d_heading < 0:
                        steering_angle = -steering_angle
                else:
                    steering_angle = 0.0
                
                # Accumulate time
                total_time += segment_time
                
            else:
                # First point - set initial velocity and steering to zero
                velocity = 0.0
                steering_angle = 0.0
                total_time = 0.0

            # Store point data
            # Positions: x, y, heading, velocity_position, steering_position
            point.positions = [x, y, heading, 0.0, 0.0]  # Last two are placeholders
            
            # Velocities: dx, dy, d_heading, velocity, steering_rate
            if prev_pose is not None:
                point.velocities = [
                    (x - prev_pose[0]) / self.target_dt,
                    (y - prev_pose[1]) / self.target_dt,
                    self.normalize_angle(heading - prev_pose[2]) / self.target_dt,
                    velocity,
                    steering_angle
                ]
            else:
                point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Set time from start
            sec = int(total_time)
            nanosec = int((total_time - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            points.append(point)
            prev_pose = (x, y, heading)
        
        # Add points to trajectory
        trajectory.points = points
        
        # Publish the trajectory
        self.trajectory_publishers[planner_name].publish(trajectory)
        self.get_logger().debug(f'Published trajectory for {planner_name} with {len(points)} points')
        
        # For demonstration, publish velocity command based on first segment
        if planner_name == 'hybrid_astar' and len(points) > 1:  # Using Hybrid A* as primary planner
            self.publish_control_command(points[1])
    
    def publish_control_command(self, trajectory_point):
        """
        Publish a control command based on a trajectory point
        
        Args:
            trajectory_point (JointTrajectoryPoint): The trajectory point
        """
        cmd = Twist()
        
        # Linear velocity from the trajectory point
        cmd.linear.x = trajectory_point.velocities[3]  # velocity
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        
        # Angular velocity can be derived from steering angle and velocity
        # Using the bicycle model: angular_vel = velocity * tan(steering_angle) / wheelbase
        steering_angle = trajectory_point.velocities[4]
        linear_velocity = trajectory_point.velocities[3]
        
        if abs(linear_velocity) > 0.1:  # Only calculate when moving
            angular_velocity = linear_velocity * math.tan(steering_angle) / self.wheelbase
            cmd.angular.z = angular_velocity
        else:
            cmd.angular.z = 0.0
        
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        
        # Publish the command
        self.control_publisher.publish(cmd)
    
    def estimate_curvature(self, x1, y1, x2, y2, x3, y3):
        """
        Estimate curvature from three points
        
        Args:
            x1, y1: First point coordinates
            x2, y2: Second point coordinates
            x3, y3: Third point coordinates
            
        Returns:
            float: Estimated curvature (1/radius)
        """
        # Handle collinear or duplicate points
        area = abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)
        if area < 1e-10:
            return 0.0
            
        # Calculate side lengths
        a = math.sqrt((x2 - x3)**2 + (y2 - y3)**2)
        b = math.sqrt((x1 - x3)**2 + (y1 - y3)**2)
        c = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        
        # Avoid division by zero or numerical issues
        if a * b * c < 1e-10:
            return 0.0
            
        # Calculate curvature (1/radius)
        curvature = 4.0 * area / (a * b * c)
        
        # Determine sign of curvature (positive for counterclockwise)
        cross_product = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
        if cross_product < 0:
            curvature = -curvature
            
        return curvature
    
    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi] range
        
        Args:
            angle (float): The input angle in radians
            
        Returns:
            float: Normalized angle
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PathToTrajectoryConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 