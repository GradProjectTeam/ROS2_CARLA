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

class HighwayFollowerNode(Node):
    def __init__(self):
        super().__init__('highway_follower_node')
        
        # Declare parameters with existence check
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # Topics
        self.declare_parameter('local_path_topic', '/highway_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('visualization_topic', '/highway_visualization')
        
        # Frames
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        
        # Planning parameters
        self.declare_parameter('planning_frequency', 10.0)
        self.declare_parameter('visualization_frequency', 2.0)
        self.declare_parameter('obstacle_threshold', 70)
        self.declare_parameter('safe_distance', 5.0)
        self.declare_parameter('lane_width', 3.5)  # Standard highway lane width
        self.declare_parameter('lane_detection_range', 30.0)  # How far to look for lane detection
        
        # Highway following parameters
        self.declare_parameter('forward_distance', 30.0)  # Distance to project forward
        self.declare_parameter('lateral_weight', 1.5)  # Weight for lateral positioning
        self.declare_parameter('forward_weight', 1.0)  # Weight for forward movement
        self.declare_parameter('obstacle_weight', 2.0)  # Weight for obstacle avoidance
        
        # DWA parameters
        self.declare_parameter('max_speed', 20.0)  # Highway speed
        self.declare_parameter('min_speed', 5.0)   # Minimum speed
        self.declare_parameter('max_yaw_rate', 0.8)  # Reduced for highway
        self.declare_parameter('max_accel', 2.0)
        self.declare_parameter('max_delta_yaw_rate', 0.5)  # Reduced for highway
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 5.0)  # Increased for highway
        self.declare_parameter('speed_cost_gain', 0.1)  # Prefer higher speeds
        self.declare_parameter('obstacle_cost_gain', 2.0)  # Higher obstacle avoidance
        self.declare_parameter('lane_following_gain', 1.5)  # Lane following importance
        self.declare_parameter('lookahead_distance', 20.0)  # Look far ahead
        
        # Get parameters
        self.local_path_topic = self.get_parameter('local_path_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.visualization_topic = self.get_parameter('visualization_topic').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.planning_frequency = self.get_parameter('planning_frequency').value
        self.visualization_frequency = self.get_parameter('visualization_frequency').value
        
        # Get obstacle parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.lane_width = self.get_parameter('lane_width').value
        self.lane_detection_range = self.get_parameter('lane_detection_range').value
        
        # Get highway following parameters
        self.forward_distance = self.get_parameter('forward_distance').value
        self.lateral_weight = self.get_parameter('lateral_weight').value
        self.forward_weight = self.get_parameter('forward_weight').value
        self.obstacle_weight = self.get_parameter('obstacle_weight').value
        
        # Get DWA parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_delta_yaw_rate = self.get_parameter('max_delta_yaw_rate').value
        self.dt = self.get_parameter('dt').value
        self.predict_time = self.get_parameter('predict_time').value
        self.speed_cost_gain = self.get_parameter('speed_cost_gain').value
        self.obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').value
        self.lane_following_gain = self.get_parameter('lane_following_gain').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        
        # Initialize state variables
        self.cost_map = None
        self.map_info = None
        self.current_pose = None
        self.current_velocity = [0.0, 0.0, 0.0]  # [vx, vy, omega]
        self.last_visualization_time = 0.0
        self.latest_trajectories = []
        self.detected_lane = None
        
        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscribers
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
        
        # Visualization timer
        self.visualization_timer = self.create_timer(
            1.0 / self.visualization_frequency,
            self.visualization_callback
        )
        
        self.get_logger().info('Highway Follower Node initialized with the following parameters:')
        self.get_logger().info(f'Max Speed: {self.max_speed} m/s')
        self.get_logger().info(f'Min Speed: {self.min_speed} m/s')
        self.get_logger().info(f'Forward Distance: {self.forward_distance} m')
        self.get_logger().info(f'Safe Distance: {self.safe_distance} m')
        self.get_logger().info(f'Lane Width: {self.lane_width} m')
        self.get_logger().info(f'Planning Frequency: {self.planning_frequency} Hz')
        self.get_logger().info(f'Visualization Frequency: {self.visualization_frequency} Hz')
    
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
        if self.cost_map is None:
            self.get_logger().debug('Waiting for map data')
            return
        
        # Get current robot pose
        current_pose = self.get_robot_pose()
        if current_pose is None:
            return
        
        self.current_pose = current_pose
        
        # Detect lane
        self.detected_lane = self.detect_lane()
        
        # Generate virtual target ahead on the lane
        target_point = self.generate_lane_target()
        if target_point is None:
            self.get_logger().warn('No valid lane target found')
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
        self.visualize_lane()
    
    def detect_lane(self):
        """Detect highway lane from occupancy grid"""
        if not self.current_pose or self.cost_map is None:
            return None
        
        x, y, yaw = self.current_pose
        
        # Define search parameters
        search_distance = self.lane_detection_range
        search_width = self.lane_width * 3  # Search area width
        search_resolution = 0.5  # Search resolution
        
        # Initialize lane points
        lane_points = []
        
        # Search for free space ahead
        for dist in np.arange(2.0, search_distance, search_resolution):
            # Look ahead in vehicle direction
            center_x = x + dist * math.cos(yaw)
            center_y = y + dist * math.sin(yaw)
            
            # Search perpendicular to vehicle direction for lane boundaries
            left_edge = None
            right_edge = None
            
            # Search for left edge
            for offset in np.arange(0, search_width, search_resolution):
                # Calculate point coordinates
                px = center_x + offset * math.cos(yaw + math.pi/2)
                py = center_y + offset * math.sin(yaw + math.pi/2)
                
                # Check if point is an obstacle
                if not self.is_valid_position(px, py):
                    left_edge = [px, py]
                    break
            
            # Search for right edge
            for offset in np.arange(0, search_width, search_resolution):
                # Calculate point coordinates
                px = center_x + offset * math.cos(yaw - math.pi/2)
                py = center_y + offset * math.sin(yaw - math.pi/2)
                
                # Check if point is an obstacle
                if not self.is_valid_position(px, py):
                    right_edge = [px, py]
                    break
            
            # If both edges found, calculate lane center
            if left_edge and right_edge:
                lane_center_x = (left_edge[0] + right_edge[0]) / 2
                lane_center_y = (left_edge[1] + right_edge[1]) / 2
                lane_points.append([lane_center_x, lane_center_y])
            else:
                # If no clear lane boundaries, use forward projection
                lane_points.append([center_x, center_y])
        
        return lane_points if lane_points else None
    
    def generate_lane_target(self):
        """Generate target point based on lane detection"""
        if not self.current_pose:
            return None
        
        x, y, yaw = self.current_pose
        
        # If lane detected, use it
        if self.detected_lane and len(self.detected_lane) > 0:
            # Find a point at lookahead distance
            target_idx = min(int(self.lookahead_distance / 0.5), len(self.detected_lane) - 1)
            return self.detected_lane[target_idx]
        
        # Fallback: just project forward
        forward_x = x + self.forward_distance * math.cos(yaw)
        forward_y = y + self.forward_distance * math.sin(yaw)
        return [forward_x, forward_y]
    
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
        v_samples = 7
        w_samples = 9
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
                lane_cost = self.calc_lane_cost(trajectory)
                
                # Total cost
                total_cost = (to_goal_cost +
                             self.speed_cost_gain * speed_cost +
                             self.obstacle_cost_gain * obstacle_cost +
                             self.lane_following_gain * lane_cost)
                
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
            if obstacle_value > 0:
                dist = (self.obstacle_threshold - obstacle_value) / self.obstacle_threshold
                min_dist = min(min_dist, dist)
        
        # If no obstacles found, return 0 cost
        if min_dist == float('inf'):
            return 0.0
        
        # Return cost based on distance (closer = higher cost)
        return 1.0 / max(0.1, min_dist)
    
    def calc_lane_cost(self, trajectory):
        """Calculate lane following cost"""
        if not trajectory or not self.detected_lane:
            return 0.0
        
        total_cost = 0.0
        
        # For each point in trajectory, find closest lane point
        for i, (x, y, _, _, _) in enumerate(trajectory):
            min_dist = float('inf')
            
            # Find closest lane point
            for lane_x, lane_y in self.detected_lane:
                dist = math.sqrt((x - lane_x)**2 + (y - lane_y)**2)
                min_dist = min(min_dist, dist)
            
            # Add distance cost (weighted by position in trajectory)
            # Points further in the future have more weight
            weight = (i + 1) / len(trajectory)
            total_cost += min_dist * weight
        
        return total_cost / len(trajectory)
    
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
    
    def visualize_lane(self):
        """Visualize detected lane"""
        if not self.detected_lane:
            return
        
        marker_array = MarkerArray()
        
        # Create lane marker
        lane_marker = Marker()
        lane_marker.header.frame_id = self.map_frame_id
        lane_marker.header.stamp = self.get_clock().now().to_msg()
        lane_marker.ns = "lane"
        lane_marker.id = 0
        lane_marker.type = Marker.LINE_STRIP
        lane_marker.action = Marker.ADD
        
        # Set color (green for lane)
        lane_marker.color.r = 0.0
        lane_marker.color.g = 1.0
        lane_marker.color.b = 0.0
        lane_marker.color.a = 0.8
        
        # Set scale
        lane_marker.scale.x = 0.2  # line width
        
        # Add lane points
        for x, y in self.detected_lane:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05  # slightly above ground
            lane_marker.points.append(p)
        
        marker_array.markers.append(lane_marker)
        
        # Add to visualization
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
    node = HighwayFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 