#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import math
from tf2_ros import TransformListener, Buffer, TransformException
import time

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        
        # Declare parameters - check if use_sim_time already exists
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
            
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('obstacles_topic', '/obstacles')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('planning_frequency', 5.0)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('obstacle_avoidance_weight', 0.8)
        self.declare_parameter('path_following_weight', 0.6)
        self.declare_parameter('goal_attraction_weight', 0.4)
        self.declare_parameter('lookahead_distance', 1.0)
        self.declare_parameter('enable_visualization', True)
        
        # Get parameters
        self.global_path_topic = self.get_parameter('global_path_topic').get_parameter_value().string_value
        self.local_path_topic = self.get_parameter('local_path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.obstacles_topic = self.get_parameter('obstacles_topic').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.planning_frequency = self.get_parameter('planning_frequency').get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.obstacle_avoidance_weight = self.get_parameter('obstacle_avoidance_weight').get_parameter_value().double_value
        self.path_following_weight = self.get_parameter('path_following_weight').get_parameter_value().double_value
        self.goal_attraction_weight = self.get_parameter('goal_attraction_weight').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        
        # Initialize variables
        self.global_path = None
        self.local_path = None
        self.map_data = None
        self.obstacles = []
        self.robot_position = None
        self.robot_orientation = None
        self.goal_reached = False
        
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
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )
        
        self.obstacles_sub = self.create_subscription(
            MarkerArray,
            self.obstacles_topic,
            self.obstacles_callback,
            qos_profile
        )
        
        # Create publishers
        self.local_path_pub = self.create_publisher(
            Path,
            self.local_path_topic,
            qos_profile
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            qos_profile
        )
        
        # Visualization publishers
        if self.enable_visualization:
            self.vis_pub = self.create_publisher(
                MarkerArray,
                '/local_planner/visualization',
                qos_profile
            )
        
        # Create TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer for planning
        self.planning_timer = self.create_timer(
            1.0 / self.planning_frequency,
            self.planning_callback
        )
        
        self.get_logger().info('Local planner node initialized')
    
    def global_path_callback(self, msg):
        """Callback for global path updates"""
        self.get_logger().debug(f'Received global path with {len(msg.poses)} points')
        self.global_path = msg
        self.goal_reached = False
    
    def map_callback(self, msg):
        """Callback for map updates"""
        self.get_logger().debug('Received map update')
        self.map_data = msg
    
    def obstacles_callback(self, msg):
        """Callback for obstacle updates"""
        self.obstacles = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE:
                obstacle = {
                    'position': (marker.pose.position.x, marker.pose.position.y),
                    'size': (marker.scale.x, marker.scale.y)
                }
                self.obstacles.append(obstacle)
        
        self.get_logger().debug(f'Received {len(self.obstacles)} obstacles')
    
    def planning_callback(self):
        """Timer callback for planning"""
        if self.global_path is None:
            self.get_logger().debug('No global path available for planning')
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
            if self.global_path and len(self.global_path.poses) > 0:
                goal = self.global_path.poses[-1]
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
            
            # Find closest point on global path
            closest_point, closest_idx = self.find_closest_point_on_path(self.robot_position)
            
            # Generate local path
            local_path = self.generate_local_path(closest_idx)
            
            # Calculate velocity commands
            cmd_vel = self.calculate_velocity_commands(local_path)
            
            # Publish local path
            self.local_path = local_path
            self.local_path_pub.publish(local_path)
            
            # Publish velocity commands
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Visualize if enabled
            if self.enable_visualization:
                self.visualize_local_planning()
        
        except TransformException as e:
            self.get_logger().warn(f'Transform error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in local planning: {str(e)}')
    
    def find_closest_point_on_path(self, position):
        """Find the closest point on the global path"""
        min_distance = float('inf')
        closest_point = None
        closest_idx = 0
        
        for i, pose in enumerate(self.global_path.poses):
            point = (pose.pose.position.x, pose.pose.position.y)
            distance = math.sqrt(
                (position[0] - point[0])**2 +
                (position[1] - point[1])**2
            )
            
            if distance < min_distance:
                min_distance = distance
                closest_point = point
                closest_idx = i
        
        return closest_point, closest_idx
    
    def generate_local_path(self, start_idx):
        """Generate a local path from the global path"""
        local_path = Path()
        local_path.header.frame_id = self.map_frame_id
        local_path.header.stamp = self.get_clock().now().to_msg()
        
        # Add points from global path within lookahead distance
        current_distance = 0.0
        prev_point = None
        
        for i in range(start_idx, len(self.global_path.poses)):
            pose = self.global_path.poses[i]
            point = (pose.pose.position.x, pose.pose.position.y)
            
            if prev_point is not None:
                segment_distance = math.sqrt(
                    (point[0] - prev_point[0])**2 +
                    (point[1] - prev_point[1])**2
                )
                current_distance += segment_distance
            
            local_path.poses.append(pose)
            prev_point = point
            
            # Stop if we've reached the lookahead distance
            if current_distance >= self.lookahead_distance:
                break
        
        # If we don't have enough points, add the last point of the global path
        if len(local_path.poses) == 0 and len(self.global_path.poses) > 0:
            local_path.poses.append(self.global_path.poses[-1])
        
        return local_path
    
    def calculate_velocity_commands(self, local_path):
        """Calculate velocity commands based on the local path"""
        cmd_vel = Twist()
        
        if not local_path.poses:
            return cmd_vel
        
        # Get target point (first point on local path)
        target = local_path.poses[0].pose.position
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
        linear_velocity = self.max_linear_velocity * (1.0 - abs(angle_diff) / math.pi)
        angular_velocity = self.max_angular_velocity * angle_diff
        
        # Adjust for obstacle avoidance
        for obstacle in self.obstacles:
            obstacle_pos = obstacle['position']
            obstacle_size = obstacle['size']
            
            # Calculate distance to obstacle
            dx_obs = obstacle_pos[0] - self.robot_position[0]
            dy_obs = obstacle_pos[1] - self.robot_position[1]
            distance_to_obstacle = math.sqrt(dx_obs*dx_obs + dy_obs*dy_obs)
            
            # If obstacle is close, adjust velocity
            if distance_to_obstacle < 1.0:
                # Calculate repulsive force
                repulsive_force = 1.0 / max(0.1, distance_to_obstacle)
                
                # Calculate angle to obstacle
                obstacle_angle = math.atan2(dy_obs, dx_obs)
                
                # Calculate angle difference
                angle_diff_obs = self.normalize_angle(obstacle_angle - self.robot_orientation)
                
                # Adjust angular velocity to avoid obstacle
                if abs(angle_diff_obs) < math.pi / 2:
                    # Obstacle is in front, turn away
                    avoidance_turn = self.max_angular_velocity * (-1.0 if angle_diff_obs > 0 else 1.0)
                    angular_velocity += avoidance_turn * self.obstacle_avoidance_weight * repulsive_force
                
                # Reduce linear velocity near obstacles
                linear_velocity *= max(0.1, min(1.0, distance_to_obstacle - 0.5))
        
        # Clamp velocities
        linear_velocity = max(0.0, min(self.max_linear_velocity, linear_velocity))
        angular_velocity = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_velocity))
        
        # Set velocity commands
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        return cmd_vel
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def visualize_local_planning(self):
        """Visualize the local planning"""
        if not self.enable_visualization or not self.robot_position:
            return
        
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header.frame_id = self.map_frame_id
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(clear_marker)
        
        # Visualize robot position and orientation
        robot_marker = Marker()
        robot_marker.header.frame_id = self.map_frame_id
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "robot"
        robot_marker.id = 0
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        
        robot_marker.pose.position.x = self.robot_position[0]
        robot_marker.pose.position.y = self.robot_position[1]
        robot_marker.pose.position.z = 0.1
        
        # Set orientation from yaw
        robot_marker.pose.orientation.x = 0.0
        robot_marker.pose.orientation.y = 0.0
        robot_marker.pose.orientation.z = math.sin(self.robot_orientation / 2.0)
        robot_marker.pose.orientation.w = math.cos(self.robot_orientation / 2.0)
        
        robot_marker.scale.x = 0.5  # Arrow length
        robot_marker.scale.y = 0.1  # Arrow width
        robot_marker.scale.z = 0.1  # Arrow height
        
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0
        
        marker_array.markers.append(robot_marker)
        
        # Visualize lookahead distance
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = self.map_frame_id
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.ns = "lookahead"
        lookahead_marker.id = 0
        lookahead_marker.type = Marker.CYLINDER
        lookahead_marker.action = Marker.ADD
        
        lookahead_marker.pose.position.x = self.robot_position[0]
        lookahead_marker.pose.position.y = self.robot_position[1]
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.w = 1.0
        
        lookahead_marker.scale.x = self.lookahead_distance * 2
        lookahead_marker.scale.y = self.lookahead_distance * 2
        lookahead_marker.scale.z = 0.01
        
        lookahead_marker.color.r = 0.0
        lookahead_marker.color.g = 1.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 0.2
        
        marker_array.markers.append(lookahead_marker)
        
        # Publish markers
        self.vis_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 