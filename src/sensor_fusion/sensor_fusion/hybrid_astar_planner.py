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
    def __init__(self, x, y, theta, cost, parent=None, f_cost=0.0):
        self.x = x
        self.y = y
        self.theta = theta  # orientation in radians
        self.cost = cost    # g-cost (cost from start)
        self.f_cost = f_cost  # f-cost (g-cost + heuristic)
        self.parent = parent
        
    def __lt__(self, other):
        # Compare nodes based on f-cost for priority queue
        return self.f_cost < other.f_cost

class HybridAStarPlanner(Node):
    def __init__(self):
        super().__init__('hybrid_astar_planner')
        
        # Declare base parameters
        self.declare_parameter('grid_size', 0.5)
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        
        # A* tuning parameters
        self.declare_parameter('max_iterations', 10000)
        self.declare_parameter('motion_resolution', 10)
        self.declare_parameter('angle_resolution', 36)  # 36 = 10 degree resolution
        self.declare_parameter('heuristic_weight', 1.5)
        
        # Replanning parameters
        self.declare_parameter('replan_on_move', True)
        self.declare_parameter('position_change_threshold', 0.25)  # meters
        self.declare_parameter('orientation_change_threshold', 0.15)  # radians
        
        # NEW: Parameters for handling unexplored areas
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('unknown_cost_multiplier', 100.0)
        self.declare_parameter('publish_debug_viz', True)
        self.declare_parameter('min_explored_percentage', 90.0)  # Min % of path through explored cells
        
        # Get base parameters
        self.grid_size = self.get_parameter('grid_size').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # Get A* parameters
        self.max_iterations = self.get_parameter('max_iterations').value
        self.motion_resolution = self.get_parameter('motion_resolution').value
        self.angle_resolution = self.get_parameter('angle_resolution').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        
        # Get replanning parameters
        self.replan_on_move = self.get_parameter('replan_on_move').value
        self.position_change_threshold = self.get_parameter('position_change_threshold').value
        self.orientation_change_threshold = self.get_parameter('orientation_change_threshold').value
        
        # NEW: Get parameters for handling unexplored areas
        self.treat_unknown_as_obstacle = self.get_parameter('treat_unknown_as_obstacle').value
        self.unknown_cost_multiplier = self.get_parameter('unknown_cost_multiplier').value
        self.publish_debug_viz = self.get_parameter('publish_debug_viz').value
        self.min_explored_percentage = self.get_parameter('min_explored_percentage').value
        
        # Constants for motion model
        self.velocity = 1.0  # m/s
        self.dt = 1.0  # s
        
        # State variables
        self.map_data = None
        self.current_pose = None
        self.goal_pose = None
        self.latest_path = None
        
        # Internal map representation
        self.cost_map = {
            'resolution': 0.5,  # meters per grid cell
            'width': 500,
            'height': 500,
            'origin': (0.0, 0.0),  # [x, y]
            'data': np.zeros(500 * 500)  # Empty cost map to start
        }
        
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
        
        # NEW: Add debug visualization publishers
        if self.publish_debug_viz:
            self.debug_map_pub = self.create_publisher(
                OccupancyGrid,
                '/hybrid_astar_debug',
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
        self.get_logger().info(f'Planning with grid size: {self.grid_size}')
        
        # Display full configuration
        self.config()
        
        # Add variables to track previous position and orientation
        self.last_planned_position = None
        self.last_planned_orientation = None
    
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
    def map_callback(self, msg):
        """Process incoming map data and update internal cost map"""
        self.map_data = msg
        
        # Update our internal map representation
        self.cost_map['resolution'] = msg.info.resolution
        self.cost_map['width'] = msg.info.width
        self.cost_map['height'] = msg.info.height
        self.cost_map['origin'] = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.cost_map['data'] = np.array(msg.data).flatten()
        
        self.get_logger().debug('Received map update')
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between nodes with unexplored penalty"""
        # Base euclidean distance
        base_cost = self.heuristic_weight * np.hypot(a.x - b.x, a.y - b.y)
        
        # NEW: Add penalty for path through unexplored area
        if self.is_unexplored(a.x, a.y):
            base_cost *= self.unknown_cost_multiplier
            
        return base_cost
    
    # NEW: Add method to check if a cell is unexplored
    def is_unexplored(self, x, y):
        """Check if a point is unexplored (-1) in the map"""
        if self.map_data is None:
            return True
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check if in bounds
        if not (0 <= grid_x < self.map_data.info.width and 
                0 <= grid_y < self.map_data.info.height):
            return True
            
        # Get index in the occupancy grid
        index = grid_y * self.map_data.info.width + grid_x
        if index >= len(self.map_data.data):
            return True
            
        # -1 (255 in uint8) represents unexplored space
        return self.map_data.data[index] == -1
    
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
        # Try to use the internal cost_map if available and populated
        if len(self.cost_map['data']) > 0:
            # Convert world coordinates to grid coordinates
            grid_x = int((x - self.cost_map['origin'][0]) / self.cost_map['resolution'])
            grid_y = int((y - self.cost_map['origin'][1]) / self.cost_map['resolution'])
            
            # Check if in bounds
            if not (0 <= grid_x < self.cost_map['width'] and 
                    0 <= grid_y < self.cost_map['height']):
                return True
                
            # Get index in the cost map data
            index = grid_y * self.cost_map['width'] + grid_x
            if index >= len(self.cost_map['data']):
                return True
                
            # NEW: Check for unexplored cells
            cell_value = self.cost_map['data'][index]
            if cell_value == -1 and self.treat_unknown_as_obstacle:
                return True
                
            return cell_value >= self.obstacle_threshold
        
        # Fall back to ROS map_data if internal map isn't available
        if self.map_data is None:
            return True
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check if in bounds
        if not (0 <= grid_x < self.map_data.info.width and 
                0 <= grid_y < self.map_data.info.height):
            return True
            
        # Get index in the occupancy grid
        index = grid_y * self.map_data.info.width + grid_x
        if index >= len(self.map_data.data):
            return True
            
        # NEW: Check for unexplored cells
        cell_value = self.map_data.data[index]
        if cell_value == -1 and self.treat_unknown_as_obstacle:
            return True
            
        return cell_value >= self.obstacle_threshold
    
    def kinematic_motion(self, node, steering, velocity=None, dt=None):
        """Use bicycle model for vehicle motion"""
        if velocity is None:
            velocity = self.velocity
            
        if dt is None:
            dt = self.dt
            
        # Calculate next state using bicycle model
        x = node.x + velocity * np.cos(node.theta) * dt
        y = node.y + velocity * np.sin(node.theta) * dt
        theta = node.theta + velocity * np.tan(steering) / self.wheelbase * dt
        
        # Normalize theta to [-pi, pi]
        theta = self.normalize_angle(theta)
        
        return x, y, theta
    
    def get_successors(self, node):
        """Generate successor nodes for a given node"""
        successors = []
        
        # Define different steering angles
        steering_angles = np.linspace(-np.pi/4, np.pi/4, self.motion_resolution)
        
        for steering in steering_angles:
            # Apply motion model to get next state
            x, y, theta = self.kinematic_motion(node, steering)
            
            # Skip if out of bounds or in collision
            if not self.is_in_bounds(x, y) or self.is_obstacle(x, y):
                continue
                
            # Calculate cost (distance traveled plus steering penalty)
            cost = node.cost + np.hypot(x - node.x, y - node.y)
            cost += 0.1 * abs(steering)  # Small penalty for steering
            
            # Create new successor node
            successor = Node2D(x, y, theta, cost, parent=node)
            successors.append(successor)
            
        return successors
    
    def plan(self, start_node, goal_node):
        """Hybrid A* path planning algorithm"""
        if start_node is None or goal_node is None:
            self.get_logger().warn("Cannot plan: start or goal is None")
            return None
            
        # Initialize data structures
        open_set = PriorityQueue()
        closed_set = {}
        
        # Add start node to open set
        start_node.f_cost = start_node.cost + self.heuristic(start_node, goal_node)
        open_set.put((start_node.f_cost, id(start_node), start_node))
        
        # Initialize for debugging
        iterations = 0
        start_time = time.time()
        
        # Perform A* search
        while not open_set.empty() and iterations < self.max_iterations:
            iterations += 1
            
            # Get node with lowest f_cost
            _, _, current = open_set.get()
            
            # Check if goal reached
            goal_dist = np.hypot(current.x - goal_node.x, current.y - goal_node.y)
            if goal_dist < 2.0 * self.grid_size:
                self.get_logger().info(f"Path found in {iterations} iterations, {time.time() - start_time:.2f} seconds")
                
                # NEW: Verify that path doesn't go through too many unexplored cells
                path = self.backtrack_path(current)
                if path and self.is_path_safe(path):
                    return path
                else:
                    self.get_logger().warn("Path goes through too many unexplored cells, continuing search")
                    # Continue searching for a better path
            
            # Mark as visited
            grid_x = int(current.x / self.grid_size)
            grid_y = int(current.y / self.grid_size)
            grid_theta = int(current.theta / (2 * np.pi / self.angle_resolution))
            state_key = (grid_x, grid_y, grid_theta)
            
            # Skip if already visited
            if state_key in closed_set and closed_set[state_key] <= current.cost:
                continue
                
            closed_set[state_key] = current.cost
            
            # Generate successors
            for successor in self.get_successors(current):
                # Skip if in collision or already explored with lower cost
                grid_x = int(successor.x / self.grid_size)
                grid_y = int(successor.y / self.grid_size)
                grid_theta = int(successor.theta / (2 * np.pi / self.angle_resolution))
                state_key = (grid_x, grid_y, grid_theta)
                
                if state_key in closed_set and closed_set[state_key] <= successor.cost:
                    continue
                    
                # Update f_cost and add to open set
                successor.f_cost = successor.cost + self.heuristic(successor, goal_node)
                open_set.put((successor.f_cost, id(successor), successor))
        
        self.get_logger().warn(f"Failed to find path after {iterations} iterations")
        return None
    
    # NEW: Add method to check if path is safe (not too many unexplored cells)
    def is_path_safe(self, path):
        """Check if path goes through a reasonable amount of explored cells"""
        if not path:
            return False
            
        total_cells = len(path)
        unexplored_cells = 0
        
        for pose in path:
            x, y = pose.pose.position.x, pose.pose.position.y
            if self.is_unexplored(x, y):
                unexplored_cells += 1
        
        explored_percentage = 100 * (total_cells - unexplored_cells) / total_cells
        self.get_logger().info(f"Path explored percentage: {explored_percentage:.1f}%")
        
        return explored_percentage >= self.min_explored_percentage
    
    def backtrack_path(self, node):
        """Reconstruct path by following parent pointers"""
        if node is None:
            return None
            
        path = []
        current = node
        
        while current is not None:
            # Create PoseStamped for current node
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = current.x
            pose.pose.position.y = current.y
            pose.pose.position.z = 0.0
            
            # Convert theta to quaternion
            pose.pose.orientation.z = np.sin(current.theta / 2.0)
            pose.pose.orientation.w = np.cos(current.theta / 2.0)
            
            path.append(pose)
            current = current.parent
            
        # Reverse to get start-to-goal order
        path.reverse()
        return path
    
    def get_current_state(self):
        """Get current vehicle state as a Node2D"""
        if self.current_pose is None:
            return None
            
        # Extract position
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        # Extract orientation (assuming 2D rotation around z-axis)
        qz = self.current_pose.pose.orientation.z
        qw = self.current_pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        
        return Node2D(x, y, theta, 0.0)
    
    def get_goal_state(self):
        """Get goal state as a Node2D"""
        if self.goal_pose is None:
            return None
            
        # Extract position
        x = self.goal_pose.pose.position.x
        y = self.goal_pose.pose.position.y
        
        # Extract orientation (assuming 2D rotation around z-axis)
        qz = self.goal_pose.pose.orientation.z
        qw = self.goal_pose.pose.orientation.w
        theta = 2.0 * np.arctan2(qz, qw)
        
        return Node2D(x, y, theta, 0.0)
    
    def path_to_msg(self, path):
        """Convert path to ROS Path message"""
        if not path:
            return None
            
        # Create Path message
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add poses to path
        path_msg.poses = path
        
        return path_msg
    
    # NEW: Add debug visualization function
    def publish_debug_map(self):
        """Publish a debug map showing explored/unexplored areas"""
        if not self.publish_debug_viz or self.map_data is None:
            return
            
        # Create a copy of the map
        debug_map = OccupancyGrid()
        debug_map.header = self.map_data.header
        debug_map.info = self.map_data.info
        
        # Fill debug map - make unexplored areas clearly visible
        debug_data = np.array(self.map_data.data)
        
        # Mark unexplored cells with a distinctive value (e.g., 80)
        debug_data[debug_data == -1] = 80
        
        # If we have a path, mark it on the debug map
        if self.latest_path is not None:
            for pose in self.latest_path.poses:
                x, y = pose.pose.position.x, pose.pose.position.y
                
                # Convert to grid coordinates
                grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
                
                # Mark path cells with a distinctive value (e.g., 100)
                if 0 <= grid_x < self.map_data.info.width and 0 <= grid_y < self.map_data.info.height:
                    index = grid_y * self.map_data.info.width + grid_x
                    if index < len(debug_data):
                        debug_data[index] = 100
        
        # Set the data in the debug map
        debug_map.data = debug_data.tolist()
        
        # Publish the debug map
        self.debug_map_pub.publish(debug_map)
    
    def planning_callback(self):
        """Timer callback for path planning"""
        # Check if we have all necessary data
        if self.map_data is None:
            self.get_logger().warn("No map data available")
            return
            
        if self.current_pose is None:
            self.get_logger().warn("No current pose available")
            return
            
        if self.goal_pose is None:
            self.get_logger().warn("No goal pose available")
            return
        
        # Check if we need to replan
        need_replanning = False
        
        # Always plan on first run
        if self.latest_path is None:
            need_replanning = True
        
        # Check if we've moved significantly
        if self.replan_on_move and self.last_planned_position is not None:
            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            
            # Calculate position change
            position_change = np.hypot(
                current_x - self.last_planned_position[0],
                current_y - self.last_planned_position[1]
            )
            
            # Calculate orientation change
            current_qz = self.current_pose.pose.orientation.z
            current_qw = self.current_pose.pose.orientation.w
            current_theta = 2.0 * np.arctan2(current_qz, current_qw)
            
            orientation_change = abs(self.normalize_angle(
                current_theta - self.last_planned_orientation
            ))
            
            # Check if changes exceed thresholds
            if (position_change > self.position_change_threshold or
                orientation_change > self.orientation_change_threshold):
                need_replanning = True
        
        # Perform planning if needed
        if need_replanning:
            self.get_logger().info("Planning new path")
            
            # Get start and goal states
            start_node = self.get_current_state()
            goal_node = self.get_goal_state()
            
            # Plan path
            path = self.plan(start_node, goal_node)
            
            if path:
                # Convert to ROS message
                path_msg = self.path_to_msg(path)
                
                # Publish path
                self.path_pub.publish(path_msg)
                
                # Update latest path and planning position
                self.latest_path = path_msg
                self.last_planned_position = (
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y
                )
                current_qz = self.current_pose.pose.orientation.z
                current_qw = self.current_pose.pose.orientation.w
                self.last_planned_orientation = 2.0 * np.arctan2(current_qz, current_qw)
                
                self.get_logger().info("Published new path")
            else:
                self.get_logger().warn("Failed to find path")
        
        # NEW: Publish debug visualization
        self.publish_debug_map()
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
    
    def config(self):
        """Log configuration parameters"""
        self.get_logger().info("Hybrid A* Planner Configuration:")
        self.get_logger().info(f"  Grid size: {self.grid_size} m")
        self.get_logger().info(f"  Wheelbase: {self.wheelbase} m")
        self.get_logger().info(f"  Obstacle threshold: {self.obstacle_threshold}")
        self.get_logger().info(f"  Max iterations: {self.max_iterations}")
        self.get_logger().info(f"  Motion resolution: {self.motion_resolution}")
        self.get_logger().info(f"  Angle resolution: {self.angle_resolution}")
        self.get_logger().info(f"  Heuristic weight: {self.heuristic_weight}")
        self.get_logger().info(f"  Treat unknown as obstacle: {self.treat_unknown_as_obstacle}")
        self.get_logger().info(f"  Unknown cost multiplier: {self.unknown_cost_multiplier}")
        self.get_logger().info(f"  Min explored percentage: {self.min_explored_percentage}%")

def main(args=None):
    rclpy.init(args=args)
    planner = HybridAStarPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()