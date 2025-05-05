#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import time
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')
        
        # Declare parameters
        self.declare_parameter('min_linear_velocity', 0.0)
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('min_angular_velocity', -0.8)
        self.declare_parameter('max_angular_velocity', 0.8)
        self.declare_parameter('linear_acceleration', 0.5)
        self.declare_parameter('angular_acceleration', 1.0)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_horizon', 1.0)
        self.declare_parameter('obstacle_weight', 1.0)
        self.declare_parameter('goal_weight', 1.0)
        self.declare_parameter('heading_weight', 0.8)
        self.declare_parameter('velocity_weight', 0.2)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('path_topic', '/smooth_path')
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('safety_radius', 0.3)
        
        # Get parameters
        self.min_linear_vel = self.get_parameter('min_linear_velocity').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.min_angular_vel = self.get_parameter('min_angular_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.linear_accel = self.get_parameter('linear_acceleration').value
        self.angular_accel = self.get_parameter('angular_acceleration').value
        self.dt = self.get_parameter('dt').value
        self.predict_horizon = self.get_parameter('predict_horizon').value
        self.obstacle_weight = self.get_parameter('obstacle_weight').value
        self.goal_weight = self.get_parameter('goal_weight').value
        self.heading_weight = self.get_parameter('heading_weight').value
        self.velocity_weight = self.get_parameter('velocity_weight').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.path_topic = self.get_parameter('path_topic').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.safety_radius = self.get_parameter('safety_radius').value
        
        # State variables
        self.map_data = None
        self.current_pose = None
        self.current_vel = 0.0
        self.current_angular_vel = 0.0
        self.path = None
        self.goal = None
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            qos_profile
        )
        self.trajectory_viz_pub = self.create_publisher(
            MarkerArray,
            '/dwa_trajectories',
            qos_profile
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            qos_profile
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            qos_profile
        )
        
        # Timer for planning
        self.planning_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.planning_callback
        )
        
        self.get_logger().info('DWA Local Planner node initialized')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().debug('Received map update')
        
    def path_callback(self, msg):
        self.path = msg
        if msg.poses:
            self.goal = msg.poses[-1]  # Set the last pose in the path as the goal
            self.get_logger().debug(f'Received path with {len(msg.poses)} poses')
        else:
            self.get_logger().warning('Received empty path')
    
    def get_current_state(self):
        """Extract current robot state"""
        if not self.current_pose:
            return None
            
        # Extract position
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        # Extract orientation (yaw) from quaternion
        qx = self.current_pose.pose.orientation.x
        qy = self.current_pose.pose.orientation.y
        qz = self.current_pose.pose.orientation.z
        qw = self.current_pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        return [x, y, theta]
    
    def is_obstacle(self, x, y):
        """Check if a point is an obstacle in the map"""
        if self.map_data is None:
            return True
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check if in bounds
        if not (0 <= grid_x < self.map_data.info.width and 
                0 <= grid_y < self.map_data.info.height):
            return True
            
        # Get index in the occupancy grid data
        index = grid_y * self.map_data.info.width + grid_x
        if index >= len(self.map_data.data):
            return True
            
        return self.map_data.data[index] >= self.obstacle_threshold
    
    def calculate_dynamic_window(self):
        """Calculate the dynamic window based on current velocity and acceleration constraints"""
        # Robot dynamics constraints
        v_min = max(self.min_linear_vel, self.current_vel - self.linear_accel * self.dt)
        v_max = min(self.max_linear_vel, self.current_vel + self.linear_accel * self.dt)
        w_min = max(self.min_angular_vel, self.current_angular_vel - self.angular_accel * self.dt)
        w_max = min(self.max_angular_vel, self.current_angular_vel + self.angular_accel * self.dt)
        
        return [v_min, v_max, w_min, w_max]
    
    def predict_trajectory(self, v, w, horizon):
        """Predict the trajectory for given velocity commands"""
        if not self.current_pose:
            return []
            
        # Get current state
        current_state = self.get_current_state()
        if not current_state:
            return []
            
        x, y, theta = current_state
        
        # Number of prediction steps
        n_steps = int(horizon / self.dt)
        
        # Initialize trajectory
        traj = [(x, y)]
        
        # Predict future positions
        for _ in range(n_steps):
            # Update using kinematic bicycle model
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            
            # Normalize theta
            theta = math.atan2(math.sin(theta), math.cos(theta))
            
            traj.append((x, y))
            
            # Check for collision
            if self.is_obstacle(x, y):
                break
                
        return traj
    
    def calculate_obstacle_cost(self, trajectory):
        """Calculate cost related to obstacles"""
        if not trajectory:
            return float('inf')
            
        min_dist = float('inf')
        
        # Calculate minimum distance to obstacles
        for x, y in trajectory:
            # Check points in a small radius around the trajectory point
            collision = False
            for dx in np.linspace(-self.safety_radius, self.safety_radius, 5):
                for dy in np.linspace(-self.safety_radius, self.safety_radius, 5):
                    if dx*dx + dy*dy <= self.safety_radius*self.safety_radius:
                        if self.is_obstacle(x + dx, y + dy):
                            collision = True
                            break
                if collision:
                    break
                    
            if collision:
                return float('inf')
                
            # Find closest obstacle (optional, for smoother obstacle avoidance)
            for dx in np.linspace(-1.0, 1.0, 10):
                for dy in np.linspace(-1.0, 1.0, 10):
                    if self.is_obstacle(x + dx, y + dy):
                        dist = math.sqrt(dx*dx + dy*dy)
                        min_dist = min(min_dist, dist)
        
        if min_dist == float('inf'):
            return 0.0
            
        return 1.0 / min_dist
    
    def calculate_goal_cost(self, trajectory):
        """Calculate cost related to goal"""
        if not trajectory or not self.goal:
            return float('inf')
            
        # Get the end point of trajectory
        x, y = trajectory[-1]
        
        # Calculate distance to goal
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        
        return math.hypot(goal_x - x, goal_y - y)
    
    def calculate_heading_cost(self, trajectory):
        """Calculate cost related to heading alignment with the goal"""
        if not trajectory or not self.goal or len(trajectory) < 2:
            return float('inf')
            
        # Get the last two points to determine heading
        x2, y2 = trajectory[-1]
        x1, y1 = trajectory[-2]
        
        # Calculate trajectory heading
        traj_heading = math.atan2(y2 - y1, x2 - x1)
        
        # Calculate goal heading
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        goal_heading = math.atan2(goal_y - y2, goal_x - x2)
        
        # Calculate heading difference
        heading_diff = abs(traj_heading - goal_heading)
        heading_diff = min(heading_diff, 2 * math.pi - heading_diff)
        
        return heading_diff
    
    def dwa_planning(self):
        """Execute DWA planning algorithm"""
        if not self.current_pose or not self.goal or not self.map_data:
            return None, None
            
        # Calculate dynamic window
        dw = self.calculate_dynamic_window()
        
        # Initialize best trajectory and score
        best_traj = None
        min_cost = float('inf')
        
        # Number of samples for v and w
        v_samples = 5
        w_samples = 7
        
        # Search for the best trajectory
        all_trajectories = []
        
        for v in np.linspace(dw[0], dw[1], v_samples):
            for w in np.linspace(dw[2], dw[3], w_samples):
                # Predict trajectory
                traj = self.predict_trajectory(v, w, self.predict_horizon)
                
                if not traj:
                    continue
                    
                # Store for visualization
                all_trajectories.append((traj, v, w))
                
                # Calculate costs
                obstacle_cost = self.calculate_obstacle_cost(traj)
                goal_cost = self.calculate_goal_cost(traj)
                heading_cost = self.calculate_heading_cost(traj)
                velocity_cost = self.max_linear_vel - v  # Prefer higher velocities
                
                # Skip trajectories that would collide with obstacles
                if obstacle_cost == float('inf'):
                    continue
                    
                # Total cost is weighted sum
                total_cost = (
                    self.obstacle_weight * obstacle_cost +
                    self.goal_weight * goal_cost +
                    self.heading_weight * heading_cost +
                    self.velocity_weight * velocity_cost
                )
                
                # Update best trajectory
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_traj = traj
                    best_v = v
                    best_w = w
        
        # Visualize trajectories
        self.visualize_trajectories(all_trajectories, best_traj)
        
        if best_traj:
            return best_v, best_w
        return None, None
    
    def visualize_trajectories(self, all_trajectories, best_traj):
        """Visualize all considered trajectories and highlight the best one"""
        marker_array = MarkerArray()
        
        # Add marker for each trajectory
        for i, (traj, v, w) in enumerate(all_trajectories):
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Line width
            
            # Check if this is the best trajectory
            is_best = (best_traj and traj == best_traj)
            
            # Set color (green for best, gray for others)
            marker.color = ColorRGBA()
            if is_best:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.7
                marker.color.g = 0.7
                marker.color.b = 0.7
                marker.color.a = 0.3
            
            # Add points
            for x, y in traj:
                point = self.create_point(x, y, 0.1)  # Slightly above ground
                marker.points.append(point)
            
            marker_array.markers.append(marker)
        
        # Publish the marker array
        self.trajectory_viz_pub.publish(marker_array)
    
    def create_point(self, x, y, z):
        """Helper to create a point for visualization"""
        from geometry_msgs.msg import Point
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p
    
    def planning_callback(self):
        """Main planning loop"""
        if not self.current_pose or not self.goal or not self.map_data:
            return
            
        # Run DWA planning
        start_time = time.time()
        v, w = self.dwa_planning()
        planning_time = time.time() - start_time
        
        if v is None or w is None:
            self.get_logger().warning("Failed to find a valid trajectory")
            return
            
        # Create command message
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Update current velocity (for next iteration)
        self.current_vel = v
        self.current_angular_vel = w
        
        self.get_logger().debug(f"Planning completed in {planning_time:.3f} seconds, cmd: v={v:.2f}, w={w:.2f}")
        
def main(args=None):
    rclpy.init(args=args)
    node = DWALocalPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 