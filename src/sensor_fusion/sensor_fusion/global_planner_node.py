#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import heapq
from tf2_ros import TransformListener, Buffer
import math
import time

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        
        # Declare parameters - check if use_sim_time already exists
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
            
        self.declare_parameter('planner_type', 'astar')
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('planning_frequency', 1.0)
        self.declare_parameter('obstacle_inflation_radius', 1.5)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('heuristic_weight', 1.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('visualize_search', True)
        self.declare_parameter('visualize_heatmap', True)
        
        # Get parameters
        self.planner_type = self.get_parameter('planner_type').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.planning_frequency = self.get_parameter('planning_frequency').get_parameter_value().double_value
        self.obstacle_inflation_radius = self.get_parameter('obstacle_inflation_radius').get_parameter_value().double_value
        self.path_resolution = self.get_parameter('path_resolution').get_parameter_value().double_value
        self.heuristic_weight = self.get_parameter('heuristic_weight').get_parameter_value().double_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        self.visualize_search = self.get_parameter('visualize_search').get_parameter_value().bool_value
        self.visualize_heatmap = self.get_parameter('visualize_heatmap').get_parameter_value().bool_value
        
        # Initialize map and goal variables
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.goal_pose = None
        
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
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            qos_profile
        )
        
        # Create publishers
        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            qos_profile
        )
        
        # Visualization publishers
        if self.enable_visualization:
            self.search_vis_pub = self.create_publisher(
                MarkerArray,
                '/global_planner/search_visualization',
                qos_profile
            )
            
            self.heatmap_pub = self.create_publisher(
                OccupancyGrid,
                '/global_planner/cost_heatmap',
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
        
        self.get_logger().info(f'Global planner node initialized with {self.planner_type} algorithm')
    
    def map_callback(self, msg):
        """Callback for map updates"""
        self.get_logger().debug('Received map update')
        self.map_data = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
    
    def goal_callback(self, msg):
        """Callback for goal updates"""
        self.get_logger().info(f'Received new goal: ({msg.pose.position.x}, {msg.pose.position.y})')
        self.goal_pose = msg
    
    def planning_callback(self):
        """Timer callback for planning"""
        if self.map_data is None:
            self.get_logger().warn('No map data available for planning')
            return
        
        if self.goal_pose is None:
            self.get_logger().debug('No goal set for planning')
            return
        
        try:
            # Get current robot position from TF
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time()
            )
            
            start_x = transform.transform.translation.x
            start_y = transform.transform.translation.y
            
            # Convert to grid coordinates
            start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
            goal_grid_x, goal_grid_y = self.world_to_grid(
                self.goal_pose.pose.position.x,
                self.goal_pose.pose.position.y
            )
            
            self.get_logger().info(f'Planning path from ({start_x:.2f}, {start_y:.2f}) to ({self.goal_pose.pose.position.x:.2f}, {self.goal_pose.pose.position.y:.2f})')
            
            # Plan path
            if self.planner_type == 'astar':
                path, visited_cells, cost_grid = self.a_star_planning(
                    (start_grid_x, start_grid_y),
                    (goal_grid_x, goal_grid_y)
                )
            elif self.planner_type == 'dijkstra':
                path, visited_cells, cost_grid = self.dijkstra_planning(
                    (start_grid_x, start_grid_y),
                    (goal_grid_x, goal_grid_y)
                )
            else:
                self.get_logger().error(f'Unknown planner type: {self.planner_type}')
                return
            
            # Convert path to ROS message
            if path:
                path_msg = self.create_path_message(path)
                self.path_pub.publish(path_msg)
                self.get_logger().info(f'Published path with {len(path_msg.poses)} points')
                
                # Visualize search if enabled
                if self.enable_visualization and self.visualize_search:
                    self.visualize_search_space(visited_cells)
                
                # Visualize cost heatmap if enabled
                if self.enable_visualization and self.visualize_heatmap and cost_grid is not None:
                    self.visualize_cost_heatmap(cost_grid)
            else:
                self.get_logger().warn('No path found')
        
        except Exception as e:
            self.get_logger().error(f'Error in planning: {str(e)}')
    
    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y
    
    def is_valid_cell(self, x, y):
        """Check if a grid cell is valid and not an obstacle"""
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False
        
        idx = y * self.map_width + x
        # Check if cell is free (value < 50 in occupancy grid)
        return self.map_data.data[idx] < 90
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between two grid cells"""
        # Manhattan distance
        return self.heuristic_weight * (abs(a[0] - b[0]) + abs(a[1] - b[1]))
    
    def a_star_planning(self, start, goal):
        """A* path planning algorithm"""
        self.get_logger().info('Running A* algorithm...')
        start_time = time.time()
        
        # Initialize open and closed sets
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        open_set_hash = {start}
        visited_cells = []
        
        # Initialize cost grid for visualization
        cost_grid = np.full((self.map_height, self.map_width), np.inf)
        cost_grid[start[1], start[0]] = 0
        
        while open_set:
            # Get the cell with the lowest f_score
            current_f, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            visited_cells.append(current)
            
            # Check if we've reached the goal
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                self.get_logger().info(f'Path found with {len(path)} points in {time.time() - start_time:.3f} seconds')
                return path, visited_cells, cost_grid
            
            # Check all neighbors
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip invalid cells
                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue
                
                # Calculate tentative g_score
                # Diagonal movement costs more
                if abs(dx) + abs(dy) == 2:
                    tentative_g_score = g_score[current] + 1.414  # sqrt(2)
                else:
                    tentative_g_score = g_score[current] + 1.0
                
                # Update if this path is better
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    cost_grid[neighbor[1], neighbor[0]] = f_score[neighbor]
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        self.get_logger().warn('No path found')
        return [], visited_cells, cost_grid
    
    def dijkstra_planning(self, start, goal):
        """Dijkstra's path planning algorithm"""
        self.get_logger().info('Running Dijkstra\'s algorithm...')
        start_time = time.time()
        
        # Initialize open and closed sets
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
        visited_cells = []
        
        # Initialize cost grid for visualization
        cost_grid = np.full((self.map_height, self.map_width), np.inf)
        cost_grid[start[1], start[0]] = 0
        
        while open_set:
            # Get the cell with the lowest cost
            current_cost, current = heapq.heappop(open_set)
            visited_cells.append(current)
            
            # Check if we've reached the goal
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                
                self.get_logger().info(f'Path found with {len(path)} points in {time.time() - start_time:.3f} seconds')
                return path, visited_cells, cost_grid
            
            # Check all neighbors
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip invalid cells
                if not self.is_valid_cell(neighbor[0], neighbor[1]):
                    continue
                
                # Calculate new cost
                # Diagonal movement costs more
                if abs(dx) + abs(dy) == 2:
                    new_cost = cost_so_far[current] + 1.414  # sqrt(2)
                else:
                    new_cost = cost_so_far[current] + 1.0
                
                # Update if this path is better
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (new_cost, neighbor))
                    cost_grid[neighbor[1], neighbor[0]] = new_cost
        
        self.get_logger().warn('No path found')
        return [], visited_cells, cost_grid
    
    def create_path_message(self, path):
        """Convert path to ROS Path message"""
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_x, grid_y in path:
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation
            
            path_msg.poses.append(pose)
        
        return path_msg
    
    def visualize_search_space(self, visited_cells):
        """Visualize the search space"""
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header.frame_id = self.map_frame_id
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        marker_array.markers.append(clear_marker)
        
        # Create markers for visited cells
        for i, (grid_x, grid_y) in enumerate(visited_cells):
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "search_space"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = world_x
            marker.pose.position.y = world_y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = self.map_resolution * 0.8
            marker.scale.y = self.map_resolution * 0.8
            marker.scale.z = 0.02
            
            # Fade color based on visit order
            intensity = 1.0 - min(1.0, float(i) / len(visited_cells))
            marker.color.r = 0.0
            marker.color.g = intensity
            marker.color.b = 1.0
            marker.color.a = 0.3
            
            marker.lifetime.sec = 0  # Persistent
            
            marker_array.markers.append(marker)
        
        self.search_vis_pub.publish(marker_array)
    
    def visualize_cost_heatmap(self, cost_grid):
        """Visualize the cost heatmap"""
        heatmap_msg = OccupancyGrid()
        heatmap_msg.header.frame_id = self.map_frame_id
        heatmap_msg.header.stamp = self.get_clock().now().to_msg()
        heatmap_msg.info = self.map_data.info
        
        # Normalize costs to 0-100 range for visualization
        normalized_costs = np.zeros(cost_grid.shape, dtype=np.int8)
        finite_mask = np.isfinite(cost_grid)
        
        if np.any(finite_mask):
            min_cost = np.min(cost_grid[finite_mask])
            max_cost = np.max(cost_grid[finite_mask])
            
            if max_cost > min_cost:
                normalized_costs[finite_mask] = ((cost_grid[finite_mask] - min_cost) / (max_cost - min_cost) * 100).astype(np.int8)
            else:
                normalized_costs[finite_mask] = 50
        
        # Convert to 1D array
        heatmap_msg.data = normalized_costs.flatten().tolist()
        
        self.heatmap_pub.publish(heatmap_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 