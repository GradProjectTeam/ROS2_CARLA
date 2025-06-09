#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer, TransformException
import math
import numpy as np
from collections import deque

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Declare parameters - check if use_sim_time already exists
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
            
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('linear_velocity_gain', 1.0)
        self.declare_parameter('angular_velocity_gain', 1.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('enable_pure_pursuit', True)
        self.declare_parameter('enable_pid_control', True)
        self.declare_parameter('pid_p_gain', 1.0)
        self.declare_parameter('pid_i_gain', 0.1)
        self.declare_parameter('pid_d_gain', 0.1)
        
        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.local_path_topic = self.get_parameter('local_path_topic').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.linear_velocity_gain = self.get_parameter('linear_velocity_gain').get_parameter_value().double_value
        self.angular_velocity_gain = self.get_parameter('angular_velocity_gain').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.enable_pure_pursuit = self.get_parameter('enable_pure_pursuit').get_parameter_value().bool_value
        self.enable_pid_control = self.get_parameter('enable_pid_control').get_parameter_value().bool_value
        self.pid_p_gain = self.get_parameter('pid_p_gain').get_parameter_value().double_value
        self.pid_i_gain = self.get_parameter('pid_i_gain').get_parameter_value().double_value
        self.pid_d_gain = self.get_parameter('pid_d_gain').get_parameter_value().double_value
        
        # Initialize variables
        self.local_path = None
        self.robot_position = None
        self.robot_orientation = None
        self.goal_reached = False
        
        # PID controller variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscribers
        self.local_path_sub = self.create_subscription(
            Path,
            self.local_path_topic,
            self.local_path_callback,
            qos_profile
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            qos_profile
        )
        
        # Create visualization publisher
        self.target_vis_pub = self.create_publisher(
            Marker,
            '/robot_controller/target_point',
            qos_profile
        )
        
        # Create TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer for control
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_callback
        )
        
        self.get_logger().info('Robot controller node initialized')
    
    def local_path_callback(self, msg):
        """Callback for local path updates"""
        self.get_logger().debug(f'Received local path with {len(msg.poses)} points')
        self.local_path = msg
        self.goal_reached = False
    
    def control_callback(self):
        """Timer callback for robot control"""
        if self.local_path is None or len(self.local_path.poses) == 0:
            # No path to follow, stop the robot
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        try:
            # Get current robot position from TF
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time()
            )
            
            self.robot_position = (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
            
            # Extract orientation (yaw) from quaternion
            q = transform.transform.rotation
            self.robot_orientation = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            
            # Check if we've reached the goal
            goal = self.local_path.poses[-1]
            goal_pos = (goal.pose.position.x, goal.pose.position.y)
            distance_to_goal = math.sqrt(
                (self.robot_position[0] - goal_pos[0])**2 +
                (self.robot_position[1] - goal_pos[1])**2
            )
            
            if distance_to_goal < self.goal_tolerance:
                if not self.goal_reached:
                    self.get_logger().info('Goal reached!')
                    self.goal_reached = True
                
                # Stop the robot
                cmd_vel = Twist()
                self.cmd_vel_pub.publish(cmd_vel)
                return
            
            # Calculate velocity commands based on the control method
            if self.enable_pure_pursuit:
                cmd_vel = self.pure_pursuit_control()
            else:
                cmd_vel = self.simple_path_following()
            
            # Apply PID control if enabled
            if self.enable_pid_control:
                cmd_vel = self.apply_pid_control(cmd_vel)
            
            # Publish velocity commands
            self.cmd_vel_pub.publish(cmd_vel)
        
        except TransformException as e:
            self.get_logger().warn(f'Transform error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in robot control: {str(e)}')
    
    def pure_pursuit_control(self):
        """Implement pure pursuit control algorithm"""
        cmd_vel = Twist()
        
        # Find the lookahead point on the path
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            # No valid lookahead point, use the last point on the path
            if len(self.local_path.poses) > 0:
                last_pose = self.local_path.poses[-1].pose.position
                lookahead_point = (last_pose.x, last_pose.y)
            else:
                # No path points, stop the robot
                return cmd_vel
        
        # Visualize the lookahead point
        self.visualize_target_point(lookahead_point)
        
        # Transform the lookahead point to robot frame
        dx = lookahead_point[0] - self.robot_position[0]
        dy = lookahead_point[1] - self.robot_position[1]
        
        # Rotate the point to robot frame
        cos_theta = math.cos(-self.robot_orientation)
        sin_theta = math.sin(-self.robot_orientation)
        
        target_x = dx * cos_theta - dy * sin_theta
        target_y = dx * sin_theta + dy * cos_theta
        
        # Calculate the curvature (1/radius)
        lookahead_distance = math.sqrt(target_x**2 + target_y**2)
        
        if lookahead_distance < 0.001:
            # We're very close to the target point, move straight
            curvature = 0.0
        else:
            # Pure pursuit formula
            curvature = 2.0 * target_y / (lookahead_distance**2)
        
        # Calculate linear and angular velocity
        linear_velocity = self.max_linear_velocity
        angular_velocity = curvature * linear_velocity
        
        # Clamp angular velocity
        angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))
        
        # Set velocity commands
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        return cmd_vel
    
    def simple_path_following(self):
        """Implement simple path following control"""
        cmd_vel = Twist()
        
        if len(self.local_path.poses) == 0:
            return cmd_vel
        
        # Get the first point on the path
        target = self.local_path.poses[0].pose.position
        target_point = (target.x, target.y)
        
        # Calculate direction to target
        dx = target_point[0] - self.robot_position[0]
        dy = target_point[1] - self.robot_position[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate target angle
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(target_angle - self.robot_orientation)
        
        # Calculate linear and angular velocity
        linear_velocity = self.max_linear_velocity * (1.0 - abs(angle_diff) / math.pi) * self.linear_velocity_gain
        angular_velocity = self.max_angular_velocity * angle_diff * self.angular_velocity_gain
        
        # Clamp velocities
        linear_velocity = max(0.0, min(self.max_linear_velocity, linear_velocity))
        angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))
        
        # Set velocity commands
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        return cmd_vel
    
    def apply_pid_control(self, base_cmd_vel):
        """Apply PID control to refine the velocity commands"""
        if self.local_path is None or len(self.local_path.poses) == 0:
            return base_cmd_vel
        
        # Get the target point
        target = self.local_path.poses[0].pose.position
        target_point = (target.x, target.y)
        
        # Calculate cross-track error (distance from robot to path)
        error = self.calculate_cross_track_error(target_point)
        
        # Get current time
        current_time = self.get_clock().now()
        
        # Initialize PID variables if this is the first call
        if self.last_time is None:
            self.last_time = current_time
            self.prev_error = error
            self.integral = 0.0
            return base_cmd_vel
        
        # Calculate time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.001:  # Avoid division by very small numbers
            return base_cmd_vel
        
        # Calculate PID terms
        p_term = self.pid_p_gain * error
        
        # Update integral term with anti-windup
        self.integral += error * dt
        self.integral = max(-1.0, min(1.0, self.integral))  # Clamp integral to prevent windup
        i_term = self.pid_i_gain * self.integral
        
        # Calculate derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.pid_d_gain * derivative
        
        # Calculate PID output
        pid_output = p_term + i_term + d_term
        
        # Update previous values
        self.prev_error = error
        self.last_time = current_time
        
        # Apply PID output to angular velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = base_cmd_vel.linear.x
        cmd_vel.angular.z = base_cmd_vel.angular.z + pid_output
        
        # Clamp angular velocity
        cmd_vel.angular.z = max(-self.max_angular_velocity, min(self.max_angular_velocity, cmd_vel.angular.z))
        
        return cmd_vel
    
    def find_lookahead_point(self):
        """Find the lookahead point on the path"""
        if self.local_path is None or len(self.local_path.poses) == 0:
            return None
        
        # Check if there are at least two points to form a line
        if len(self.local_path.poses) < 2:
            # Return the only point
            point = self.local_path.poses[0].pose.position
            return (point.x, point.y)
        
        # Find the first point that is at least lookahead_distance away
        for i in range(len(self.local_path.poses)):
            point = self.local_path.poses[i].pose.position
            distance = math.sqrt(
                (self.robot_position[0] - point.x)**2 +
                (self.robot_position[1] - point.y)**2
            )
            
            if distance >= self.lookahead_distance:
                return (point.x, point.y)
        
        # If no point is far enough, return the last point
        last_point = self.local_path.poses[-1].pose.position
        return (last_point.x, last_point.y)
    
    def calculate_cross_track_error(self, target_point):
        """Calculate the cross-track error (perpendicular distance to path)"""
        if len(self.local_path.poses) < 2:
            # Not enough points to form a line, use direct distance
            dx = target_point[0] - self.robot_position[0]
            dy = target_point[1] - self.robot_position[1]
            return math.sqrt(dx*dx + dy*dy)
        
        # Find the closest line segment on the path
        min_distance = float('inf')
        closest_segment_start = None
        closest_segment_end = None
        
        for i in range(len(self.local_path.poses) - 1):
            p1 = self.local_path.poses[i].pose.position
            p2 = self.local_path.poses[i+1].pose.position
            
            # Calculate distance from robot to line segment
            distance = self.point_to_line_distance(
                self.robot_position,
                (p1.x, p1.y),
                (p2.x, p2.y)
            )
            
            if distance < min_distance:
                min_distance = distance
                closest_segment_start = (p1.x, p1.y)
                closest_segment_end = (p2.x, p2.y)
        
        # Calculate the sign of the error based on which side of the path the robot is on
        if closest_segment_start is not None and closest_segment_end is not None:
            # Vector from start to end of segment
            path_vector = (
                closest_segment_end[0] - closest_segment_start[0],
                closest_segment_end[1] - closest_segment_start[1]
            )
            
            # Vector from start of segment to robot
            robot_vector = (
                self.robot_position[0] - closest_segment_start[0],
                self.robot_position[1] - closest_segment_start[1]
            )
            
            # Cross product to determine side (positive = left, negative = right)
            cross_product = path_vector[0] * robot_vector[1] - path_vector[1] * robot_vector[0]
            
            # Return signed distance
            return min_distance if cross_product >= 0 else -min_distance
        
        return 0.0
    
    def point_to_line_distance(self, point, line_start, line_end):
        """Calculate the shortest distance from a point to a line segment"""
        # Vector from line start to end
        line_vector = (line_end[0] - line_start[0], line_end[1] - line_start[1])
        
        # Vector from line start to point
        point_vector = (point[0] - line_start[0], point[1] - line_start[1])
        
        # Calculate the length of the line segment
        line_length = math.sqrt(line_vector[0]**2 + line_vector[1]**2)
        
        if line_length < 0.0001:
            # Line segment is a point, return distance to that point
            return math.sqrt(point_vector[0]**2 + point_vector[1]**2)
        
        # Calculate the projection of point_vector onto line_vector
        projection = (point_vector[0] * line_vector[0] + point_vector[1] * line_vector[1]) / line_length
        
        # Normalize projection to the range [0, 1]
        projection = max(0.0, min(1.0, projection / line_length))
        
        # Calculate the closest point on the line segment
        closest_point = (
            line_start[0] + projection * line_vector[0],
            line_start[1] + projection * line_vector[1]
        )
        
        # Calculate the distance to the closest point
        return math.sqrt(
            (point[0] - closest_point[0])**2 +
            (point[1] - closest_point[1])**2
        )
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def visualize_target_point(self, point):
        """Visualize the target point as a marker"""
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.1  # Slightly above ground
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.target_vis_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 