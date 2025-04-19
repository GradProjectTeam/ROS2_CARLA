#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import time
import math
from queue import PriorityQueue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Node2D:
    def __init__(self, x, y, theta, cost, parent=None):
        self.x = x
        self.y = y
        self.theta = theta  # orientation in radians
        self.cost = cost
        self.parent = parent
        
    def __lt__(self, other):
        return self.cost < other.cost

class HybridAStarPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_astar_planner')
        
        # Declare parameters
        self.declare_parameter('grid_size', 1.0)
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        
        # New tunable A* parameters
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('motion_resolution', 5)
        self.declare_parameter('angle_resolution', 18)  # 18 = 20 degree resolution
        self.declare_parameter('heuristic_weight', 1.0)
        
        # Get parameters
        self.grid_size = self.get_parameter('grid_size').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # Get new parameters
        self.max_iterations = self.get_parameter('max_iterations').value
        self.motion_resolution = self.get_parameter('motion_resolution').value
        self.angle_resolution = self.get_parameter('angle_resolution').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        
        # State variables
        self.map_data = None
        self.current_pose = None
        self.goal_pose = None
        self.latest_path = None
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path, 
            '/hybrid_astar_path', 
            qos_profile
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            qos_profile
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_profile
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )
        
        # Timer for planning
        self.planning_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.planning_callback
        )
        
        self.get_logger().info('Hybrid A* Planner node initialized')
        self.get_logger().info(f'Grid size: {self.grid_size}m')
        self.get_logger().info(f'Max iterations: {self.max_iterations}')
        self.get_logger().info(f'Motion resolution: {self.motion_resolution} angles')
        self.get_logger().info(f'Angle resolution: {360/self.angle_resolution} degrees')
        self.get_logger().info(f'Heuristic weight: {self.heuristic_weight}')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().debug('Received map update')
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between nodes"""
        # Apply heuristic weight to make A* more greedy (faster) or more optimal (slower)
        return self.heuristic_weight * np.hypot(a.x - b.x, a.y - b.y)
    
    def is_in_bounds(self, x, y):
        """Check if coordinates are within map boundaries"""
        if self.map_data is None:
            return False
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        return (0 <= grid_x < self.map_data.info.width and 
                0 <= grid_y < self.map_data.info.height)
    
    def is_obstacle(self, x, y):
        """Check if a point is an obstacle based on map data"""
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
    
    def kinematic_motion(self, node, steering, velocity, dt=1.0):
        """Apply vehicle kinematic model"""
        x = node.x + velocity * np.cos(node.theta) * dt
        y = node.y + velocity * np.sin(node.theta) * dt
        theta = node.theta + (velocity / self.wheelbase) * np.tan(steering) * dt
        
        # Normalize theta to [-pi, pi]
        theta = np.arctan2(np.sin(theta), np.cos(theta))
        
        return Node2D(x, y, theta, node.cost + dt, parent=node)
    
    def get_successors(self, node):
        """Generate successor nodes by applying different steering angles"""
        motions = []
        
        # Try different steering angles based on motion_resolution parameter
        for delta in np.linspace(-0.5, 0.5, self.motion_resolution):  # Steering angles
            # Forward motion
            next_node = self.kinematic_motion(node, delta, 1.0)
            if not self.is_obstacle(next_node.x, next_node.y):
                motions.append(next_node)
                
            # Reverse motion (optional, can be enabled for more complex maneuvers)
            # next_node = self.kinematic_motion(node, delta, -1.0)
            # if not self.is_obstacle(next_node.x, next_node.y):
            #     motions.append(next_node)
                
        return motions
    
    def plan(self, start_node, goal_node):
        """Hybrid A* path planning algorithm"""
        if self.map_data is None:
            self.get_logger().warning('No map data available for planning')
            return None
            
        open_set = PriorityQueue()
        open_set.put((0, start_node))
        
        # Dictionary to track visited nodes
        # The key is a tuple of (grid_x, grid_y, discretized_theta)
        # to handle continuous state space with discrete representation
        visited = {}
        
        # Use max_iterations parameter to control search time
        iterations = 0
        
        # Measure planning time for performance reporting
        start_time = time.time()
        
        while not open_set.empty() and iterations < self.max_iterations:
            iterations += 1
            
            _, current = open_set.get()
            
            # Discretize state for checking visited nodes
            x_grid = int(current.x / self.grid_size)
            y_grid = int(current.y / self.grid_size)
            
            # Use angle_resolution parameter to control how finely angles are discretized
            theta_grid = int(current.theta / (np.pi/self.angle_resolution))  # discretized angle
            state_key = (x_grid, y_grid, theta_grid)
            
            # Skip if this discretized state was already visited with lower cost
            if state_key in visited and visited[state_key] <= current.cost:
                continue
                
            # Mark as visited
            visited[state_key] = current.cost
            
            # Check if goal reached
            if np.hypot(current.x - goal_node.x, current.y - goal_node.y) < 1.0:
                planning_time = time.time() - start_time
                self.get_logger().info(f'Path found in {iterations} iterations ({planning_time:.3f} sec)')
                
                # Extract path
                path = []
                node = current
                while node:
                    path.append((node.x, node.y, node.theta))
                    node = node.parent
                    
                return path[::-1]  # Reverse to get path from start to goal
            
            # Generate successors
            for neighbor in self.get_successors(current):
                h = self.heuristic(neighbor, goal_node)
                open_set.put((neighbor.cost + h, neighbor))
        
        planning_time = time.time() - start_time
        self.get_logger().warning(f'Path planning failed after {iterations} iterations ({planning_time:.3f} sec)')
        return None
    
    def get_current_state(self):
        """Extract current vehicle position and orientation"""
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
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return Node2D(x, y, yaw, 0.0)
    
    def get_goal_state(self):
        """Extract goal position and orientation"""
        if not self.goal_pose:
            return None
            
        # Extract position
        x = self.goal_pose.pose.position.x
        y = self.goal_pose.pose.position.y
        
        # Extract orientation (yaw) from quaternion
        qx = self.goal_pose.pose.orientation.x
        qy = self.goal_pose.pose.orientation.y
        qz = self.goal_pose.pose.orientation.z
        qw = self.goal_pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return Node2D(x, y, yaw, 0.0)
    
    def path_to_msg(self, path):
        """Convert path to ROS Path message"""
        if not path:
            return None
            
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id
        
        for x, y, theta in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Convert theta to quaternion
            cy = math.cos(theta * 0.5)
            sy = math.sin(theta * 0.5)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sy
            pose.pose.orientation.w = cy
            
            path_msg.poses.append(pose)
            
        return path_msg
    
    def planning_callback(self):
        """Generate and publish path plans on timer"""
        if not (self.current_pose and self.goal_pose and self.map_data):
            return
            
        # Check if we already have a path
        if self.latest_path is not None:
            # Publish existing path
            self.path_pub.publish(self.latest_path)
            return
            
        # Get current and goal states
        start_node = self.get_current_state()
        goal_node = self.get_goal_state()
        
        if start_node and goal_node:
            # Plan path
            path = self.plan(start_node, goal_node)
            
            if path:
                # Convert to ROS message
                path_msg = self.path_to_msg(path)
                
                if path_msg:
                    # Save latest path
                    self.latest_path = path_msg
                    
                    # Publish path
                    self.path_pub.publish(path_msg)
                    
                    # Calculate path length
                    path_length = 0
                    for i in range(1, len(path_msg.poses)):
                        p1 = path_msg.poses[i-1].pose.position
                        p2 = path_msg.poses[i].pose.position
                        path_length += math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
                    
                    self.get_logger().info(f'Published path with {len(path_msg.poses)} points, length: {path_length:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 