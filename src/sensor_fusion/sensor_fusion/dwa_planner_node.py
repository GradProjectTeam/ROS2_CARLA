#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformListener, Buffer, TransformException
import math
import numpy as np
from collections import deque
import time

class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')
        
        # Declare parameters with existence check
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # Topics
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('visualization_topic', '/dwa_visualization')
        
        # Frames
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        
        # Planning parameters
        self.declare_parameter('planning_frequency', 10.0)
        self.declare_parameter('visualization_frequency', 2.0)  # New parameter for visualization update rate
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('safe_distance', 3.5)
        self.declare_parameter('min_parking_width', 2.5)
        self.declare_parameter('min_parking_length', 5.0)
        
        # DWA parameters
        self.declare_parameter('max_speed', 8.0)
        self.declare_parameter('min_speed', -2.0)
        self.declare_parameter('max_yaw_rate', 1.5)
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_delta_yaw_rate', 0.8)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 4.0)
        self.declare_parameter('to_goal_cost_gain', 1.0)
        self.declare_parameter('speed_cost_gain', 0.3)
        self.declare_parameter('obstacle_cost_gain', 1.0)
        self.declare_parameter('path_following_gain', 1.0)
        self.declare_parameter('lookahead_distance', 20.0)
        
        # Get parameters
        self.global_path_topic = self.get_parameter('global_path_topic').value
        self.local_path_topic = self.get_parameter('local_path_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.visualization_topic = self.get_parameter('visualization_topic').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.planning_frequency = self.get_parameter('planning_frequency').value
        self.visualization_frequency = self.get_parameter('visualization_frequency').value
        
        # Get obstacle and parking parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.min_parking_width = self.get_parameter('min_parking_width').value
        self.min_parking_length = self.get_parameter('min_parking_length').value
        
        # Get DWA parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_delta_yaw_rate = self.get_parameter('max_delta_yaw_rate').value
        self.dt = self.get_parameter('dt').value
        self.predict_time = self.get_parameter('predict_time').value
        self.to_goal_cost_gain = self.get_parameter('to_goal_cost_gain').value
        self.speed_cost_gain = self.get_parameter('speed_cost_gain').value
        self.obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').value
        self.path_following_gain = self.get_parameter('path_following_gain').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # Initialize state variables
        self.global_path = None
        self.local_path = None
        self.cost_map = None
        self.map_info = None
        self.current_pose = None
        self.current_velocity = [0.0, 0.0, 0.0]  # [vx, vy, omega]
        self.last_visualization_time = 0.0  # Track last visualization time
        self.latest_trajectories = []  # Store latest trajectories for visualization
        
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
        
        self.vis_pub = self.create_publisher(
            MarkerArray,
            self.visualization_topic,
            qos_profile
        )
        
        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Planning timer
        self.planning_timer = self.create_timer(
            1.0 / self.planning_frequency,
            self.planning_callback
        )
        
        # Visualization timer (separate from planning)
        self.visualization_timer = self.create_timer(
            1.0 / self.visualization_frequency,
            self.visualization_callback
        )
        
        self.get_logger().info('DWA Planner Node initialized with the following parameters:')
        self.get_logger().info(f'Max Speed: {self.max_speed} m/s')
        self.get_logger().info(f'Min Speed: {self.min_speed} m/s')
        self.get_logger().info(f'Max Yaw Rate: {self.max_yaw_rate} rad/s')
        self.get_logger().info(f'Prediction Time: {self.predict_time} s')
        self.get_logger().info(f'Lookahead Distance: {self.lookahead_distance} m')
        self.get_logger().info(f'Safe Distance: {self.safe_distance} m')
        self.get_logger().info(f'Planning Frequency: {self.planning_frequency} Hz')
        self.get_logger().info(f'Visualization Frequency: {self.visualization_frequency} Hz')
    
    def global_path_callback(self, msg):
        """Callback for global path updates"""
        self.get_logger().debug(f'Received global path with {len(msg.poses)} points')
        self.global_path = msg
    
    def map_callback(self, msg):
        """Callback for map updates"""
        self.get_logger().debug('Received map update')
        self.cost_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
    
    def get_robot_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time()
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract orientation (yaw)
            q = transform.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            return [x, y, yaw]
        
        except TransformException as e:
            self.get_logger().warn(f'Failed to get robot pose: {str(e)}')
            return None
    
    def planning_callback(self):
        """Main planning callback"""
        if self.global_path is None or self.cost_map is None:
            self.get_logger().debug('Waiting for global path and map data')
            return
        
        # Get current robot pose
        current_pose = self.get_robot_pose()
        if current_pose is None:
            return
        
        self.current_pose = current_pose
        
        # Find target point on global path
        target_point = self.find_target_point()
        if target_point is None:
            self.get_logger().warn('No valid target point found on global path')
            return
        
        # Run DWA to get optimal velocity
        optimal_v, optimal_omega, trajectories = self.dwa_planning(target_point)
        
        # Store trajectories for visualization
        self.latest_trajectories = trajectories
        
        # Create and publish control command
        cmd_vel = Twist()
        cmd_vel.linear.x = optimal_v
        cmd_vel.angular.z = optimal_omega
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Generate and publish local path
        local_path = self.generate_local_path(optimal_v, optimal_omega)
        self.local_path_pub.publish(local_path)
    
    def visualization_callback(self):
        """Separate callback for visualization at a lower frequency"""
        if not self.latest_trajectories:
            return
            
        self.visualize_trajectories(self.latest_trajectories)
    
    def find_target_point(self):
        """Find target point on global path based on lookahead distance"""
        if not self.global_path or not self.current_pose:
            return None
        
        x, y, _ = self.current_pose
        min_dist = float('inf')
        closest_idx = 0
        
        # Find closest point on path
        for i, pose in enumerate(self.global_path.poses):
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Look ahead on path
        lookahead_idx = closest_idx
        lookahead_dist = 0.0
        
        while lookahead_dist < self.lookahead_distance and lookahead_idx + 1 < len(self.global_path.poses):
            p1 = self.global_path.poses[lookahead_idx].pose.position
            p2 = self.global_path.poses[lookahead_idx + 1].pose.position
            segment_dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            lookahead_dist += segment_dist
            lookahead_idx += 1
        
        # Return target point
        target = self.global_path.poses[lookahead_idx].pose.position
        return [target.x, target.y]
    
    def dwa_planning(self, goal):
        """Dynamic Window Approach planning algorithm"""
        x, y, yaw = self.current_pose
        vx, _, omega = self.current_velocity
        
        # Dynamic window calculation
        vs = [self.min_speed, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]
        vd = [vx - self.max_accel * self.dt,
              vx + self.max_accel * self.dt,
              omega - self.max_delta_yaw_rate * self.dt,
              omega + self.max_delta_yaw_rate * self.dt]
        
        # Constrain dynamic window
        vmin = max(vs[0], vd[0])
        vmax = min(vs[1], vd[1])
        wmin = max(vs[2], vd[2])
        wmax = min(vs[3], vd[3])
        
        # Evaluate all trajectories
        best_cost = float('inf')
        best_v = 0.0
        best_omega = 0.0
        best_trajectory = None
        all_trajectories = []
        
        # Discretize velocity samples
        v_samples = 5
        w_samples = 7
        v_step = (vmax - vmin) / max(1, v_samples - 1)
        w_step = (wmax - wmin) / max(1, w_samples - 1)
        
        for v in np.arange(vmin, vmax + v_step/2, v_step):
            for w in np.arange(wmin, wmax + w_step/2, w_step):
                # Predict trajectory
                trajectory = self.predict_trajectory(x, y, yaw, v, w)
                
                # Skip if trajectory is empty
                if not trajectory:
                    continue
                
                all_trajectories.append(trajectory)
                
                # Calculate costs
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.calc_speed_cost(v)
                obstacle_cost = self.calc_obstacle_cost(trajectory)
                
                # Total cost
                total_cost = (self.to_goal_cost_gain * to_goal_cost +
                             self.speed_cost_gain * speed_cost +
                             self.obstacle_cost_gain * obstacle_cost)
                
                # Update best trajectory
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_omega = w
                    best_trajectory = trajectory
        
        # Update current velocity
        self.current_velocity = [best_v, 0.0, best_omega]
        
        return best_v, best_omega, all_trajectories
    
    def predict_trajectory(self, x, y, yaw, v, omega):
        """Predict trajectory for given velocity commands"""
        trajectory = []
        time = 0.0
        
        # Current state
        x_pred = x
        y_pred = y
        yaw_pred = yaw
        
        while time <= self.predict_time:
            # Update state with motion model
            x_pred = x_pred + v * math.cos(yaw_pred) * self.dt
            y_pred = y_pred + v * math.sin(yaw_pred) * self.dt
            yaw_pred = yaw_pred + omega * self.dt
            
            # Check if position is valid (not in obstacle)
            if not self.is_valid_position(x_pred, y_pred):
                return []
            
            # Add to trajectory
            trajectory.append([x_pred, y_pred, yaw_pred, v, omega])
            time += self.dt
        
        return trajectory
    
    def is_valid_position(self, x, y):
        """Check if position is valid (not in obstacle)"""
        if self.cost_map is None or self.map_info is None:
            return True
        
        # Convert world coordinates to map coordinates
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        # Check if within map bounds
        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return False
        
        # Check if cell is free (below obstacle threshold)
        return self.cost_map[my, mx] < self.obstacle_threshold
    
    def calc_to_goal_cost(self, trajectory, goal):
        """Calculate cost to goal (final heading and distance)"""
        if not trajectory:
            return float('inf')
        
        # Get final state
        x, y, yaw, _, _ = trajectory[-1]
        
        # Calculate distance to goal
        dx = goal[0] - x
        dy = goal[1] - y
        goal_dist = math.sqrt(dx*dx + dy*dy)
        
        # Calculate heading error to goal
        goal_angle = math.atan2(dy, dx)
        heading_error = abs(self.normalize_angle(goal_angle - yaw))
        
        # Combine distance and heading costs
        cost = goal_dist + 0.5 * heading_error
        return cost
    
    def calc_speed_cost(self, v):
        """Calculate speed cost (prefer higher speeds)"""
        # Prefer speeds closer to max_speed
        return self.max_speed - v
    
    def calc_obstacle_cost(self, trajectory):
        """Calculate obstacle cost (distance to nearest obstacle)"""
        if not trajectory:
            return float('inf')
        
        min_dist = float('inf')
        
        # Check each point in trajectory
        for x, y, _, _, _ in trajectory:
            # Convert to map coordinates
            mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
            my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
            
            # Skip if outside map
            if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
                continue
            
            # Get obstacle value
            obstacle_value = self.cost_map[my, mx]
            
            # If cell is occupied, return high cost
            if obstacle_value >= self.obstacle_threshold:
                return float('inf')
            
            # Calculate distance to nearest obstacle
            # This is a simplified version - in a real implementation,
            # you would search nearby cells for obstacles
            if obstacle_value > 0:
                dist = (self.obstacle_threshold - obstacle_value) / self.obstacle_threshold
                min_dist = min(min_dist, dist)
        
        # If no obstacles found, return 0 cost
        if min_dist == float('inf'):
            return 0.0
        
        # Return cost based on distance (closer = higher cost)
        return 1.0 / max(0.1, min_dist)
    
    def generate_local_path(self, v, omega):
        """Generate local path from optimal velocity"""
        if not self.current_pose:
            return Path()
        
        x, y, yaw = self.current_pose
        
        # Create path message
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame_id
        
        # Add current position
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        path.poses.append(pose)
        
        # Predict and add future positions
        time = 0.0
        x_pred = x
        y_pred = y
        yaw_pred = yaw
        
        while time <= self.predict_time:
            # Update state with motion model
            x_pred = x_pred + v * math.cos(yaw_pred) * self.dt
            y_pred = y_pred + v * math.sin(yaw_pred) * self.dt
            yaw_pred = yaw_pred + omega * self.dt
            
            # Create pose
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_pred
            pose.pose.position.y = y_pred
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            pose.pose.orientation.z = math.sin(yaw_pred / 2.0)
            pose.pose.orientation.w = math.cos(yaw_pred / 2.0)
            
            path.poses.append(pose)
            time += self.dt
        
        return path
    
    def visualize_trajectories(self, trajectories):
        """Visualize all evaluated trajectories"""
        marker_array = MarkerArray()
        
        # Delete previous markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Add trajectory markers
        for i, trajectory in enumerate(trajectories):
            if not trajectory:
                continue
            
            # Create line strip marker
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set color (blue for all trajectories)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.3
            
            # Set scale
            marker.scale.x = 0.05  # line width
            
            # Add points
            for x, y, _, _, _ in trajectory:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1  # slightly above ground
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        # Publish marker array
        self.vis_pub.publish(marker_array)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
