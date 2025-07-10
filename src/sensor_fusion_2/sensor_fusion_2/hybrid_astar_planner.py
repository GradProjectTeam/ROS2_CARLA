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
from scipy.ndimage import distance_transform_edt, binary_dilation

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
        
        # NEW: Vehicle dimensions and safety parameters
        self.declare_parameter('vehicle_length', 4.5)
        self.declare_parameter('vehicle_width', 2.0)
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('obstacle_inflation_radius', 1.0)
        self.declare_parameter('use_distance_transform', True)
        self.declare_parameter('collision_check_resolution', 5)
        
        # A* tuning parameters
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('motion_resolution', 10)
        self.declare_parameter('angle_resolution', 36)  # 36 = 10 degree resolution
        self.declare_parameter('heuristic_weight', 1.5)
        
        # Replanning parameters
        self.declare_parameter('replan_on_move', True)
        self.declare_parameter('position_change_threshold', 0.25)  # meters
        self.declare_parameter('orientation_change_threshold', 0.15)  # radians
        self.declare_parameter('replan_on_map_change', True)
        
        # NEW: Parameters for handling unexplored areas
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('unknown_cost_multiplier', 100.0)
        self.declare_parameter('publish_debug_viz', True)
        self.declare_parameter('min_explored_percentage', 90.0)  # Min % of path through explored cells
        
        # NEW: Cost gradient parameters
        self.declare_parameter('distance_cost_weight', 50.0)
        self.declare_parameter('max_distance_cost', 100.0)
        self.declare_parameter('min_safe_distance', 2.0)
        
        # NEW: Parameters for fixed start and goal positions
        self.declare_parameter('use_fixed_start', False)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 20.0)
        self.declare_parameter('goal_y', 20.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('use_rviz_goal', True)
        self.declare_parameter('goal_topic', '/goal_pose')
        
        # NEW: Parameters for waypoint handling
        self.declare_parameter('waypoint_topic', '/carla/waypoints')
        self.declare_parameter('respect_waypoints', True)
        self.declare_parameter('waypoint_safety_distance', 1.0)
        self.declare_parameter('waypoint_cost_multiplier', 2.0)
        self.declare_parameter('follow_waypoints', True)
        
        # Get base parameters
        self.grid_size = self.get_parameter('grid_size').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # NEW: Get vehicle and safety parameters
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.vehicle_width = self.get_parameter('vehicle_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.obstacle_inflation_radius = self.get_parameter('obstacle_inflation_radius').value
        self.use_distance_transform = self.get_parameter('use_distance_transform').value
        self.collision_check_resolution = self.get_parameter('collision_check_resolution').value
        
        # Get A* parameters
        self.max_iterations = self.get_parameter('max_iterations').value
        self.motion_resolution = self.get_parameter('motion_resolution').value
        self.angle_resolution = self.get_parameter('angle_resolution').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        
        # Get replanning parameters
        self.replan_on_move = self.get_parameter('replan_on_move').value
        self.position_change_threshold = self.get_parameter('position_change_threshold').value
        self.orientation_change_threshold = self.get_parameter('orientation_change_threshold').value
        self.replan_on_map_change = self.get_parameter('replan_on_map_change').value
        
        # NEW: Get parameters for handling unexplored areas
        self.treat_unknown_as_obstacle = self.get_parameter('treat_unknown_as_obstacle').value
        self.unknown_cost_multiplier = self.get_parameter('unknown_cost_multiplier').value
        self.publish_debug_viz = self.get_parameter('publish_debug_viz').value
        self.min_explored_percentage = self.get_parameter('min_explored_percentage').value
        
        # NEW: Get cost gradient parameters
        self.distance_cost_weight = self.get_parameter('distance_cost_weight').value
        self.max_distance_cost = self.get_parameter('max_distance_cost').value
        self.min_safe_distance = self.get_parameter('min_safe_distance').value
        
        # NEW: Get parameters for fixed start and goal positions
        self.use_fixed_start = self.get_parameter('use_fixed_start').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.use_rviz_goal = self.get_parameter('use_rviz_goal').value
        self.goal_topic = self.get_parameter('goal_topic').value
        
        # NEW: Get parameters for waypoint handling
        self.waypoint_topic = self.get_parameter('waypoint_topic').value
        self.respect_waypoints = self.get_parameter('respect_waypoints').value
        self.waypoint_safety_distance = self.get_parameter('waypoint_safety_distance').value
        self.waypoint_cost_multiplier = self.get_parameter('waypoint_cost_multiplier').value
        self.follow_waypoints = self.get_parameter('follow_waypoints').value
        
        # Constants for motion model
        self.velocity = 1.0  # m/s
        self.dt = 1.0  # s
        
        # State variables
        self.map_data = None
        self.current_pose = None
        self.goal_pose = None
        self.latest_path = None
        
        # NEW: Distance transform and inflated map for better obstacle handling
        self.distance_transform = None
        self.inflated_map = None
        self.last_map_update_time = None
        
        # NEW: Vehicle footprint calculation
        self.vehicle_footprint = self.calculate_vehicle_footprint()
        
        # Create a fixed start pose if needed
        if self.use_fixed_start:
            self.current_pose = PoseStamped()
            self.current_pose.header.frame_id = self.map_frame_id
            self.current_pose.pose.position.x = self.start_x
            self.current_pose.pose.position.y = self.start_y
            self.current_pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            self.current_pose.pose.orientation.x = 0.0
            self.current_pose.pose.orientation.y = 0.0
            self.current_pose.pose.orientation.z = math.sin(self.start_yaw / 2.0)
            self.current_pose.pose.orientation.w = math.cos(self.start_yaw / 2.0)
            
            self.get_logger().info(f'Using fixed start position: ({self.start_x}, {self.start_y}, {self.start_yaw})')
        
        # Create a fixed goal pose if not using RViz goal
        if not self.use_rviz_goal:
            self.goal_pose = PoseStamped()
            self.goal_pose.header.frame_id = self.map_frame_id
            self.goal_pose.pose.position.x = self.goal_x
            self.goal_pose.pose.position.y = self.goal_y
            self.goal_pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            self.goal_pose.pose.orientation.x = 0.0
            self.goal_pose.pose.orientation.y = 0.0
            self.goal_pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
            self.goal_pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
            
            self.get_logger().info(f'Using fixed goal position: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
        
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
            self.inflated_map_pub = self.create_publisher(
                OccupancyGrid,
                '/hybrid_astar_inflated',
                qos_profile
            )
        
        # Subscribers - only subscribe if not using fixed values
        if not self.use_fixed_start:
            self.pose_sub = self.create_subscription(
                PoseStamped,
                '/current_pose',
                self.pose_callback,
                qos_profile
            )
        
        if self.use_rviz_goal:
            self.goal_sub = self.create_subscription(
                PoseStamped,
                self.goal_topic,
                self.goal_callback,
                qos_profile
            )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )
        
        # NEW: Add waypoint subscription if enabled
        if self.respect_waypoints:
            self.waypoints = []
            self.waypoint_sub = self.create_subscription(
                Path,
                self.waypoint_topic,
                self.waypoint_callback,
                qos_profile
            )
            self.get_logger().info(f'Subscribed to waypoints on topic: {self.waypoint_topic}')
        
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
    
    # NEW: Calculate vehicle footprint for collision checking
    def calculate_vehicle_footprint(self):
        """Calculate vehicle footprint points relative to vehicle center"""
        # Vehicle footprint as rectangle with safety margin
        length = self.vehicle_length + 2 * self.safety_margin
        width = self.vehicle_width + 2 * self.safety_margin
        
        # Points relative to vehicle center (origin at rear axle center)
        footprint = [
            [-length/4, -width/2],  # rear left
            [3*length/4, -width/2], # front left  
            [3*length/4, width/2],  # front right
            [-length/4, width/2]    # rear right
        ]
        
        return np.array(footprint)
    
    # NEW: Transform vehicle footprint to world coordinates
    def transform_footprint(self, x, y, theta):
        """Transform vehicle footprint to world coordinates"""
        # Rotation matrix
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        transformed_points = []
        for point in self.vehicle_footprint:
            # Rotate and translate
            rotated_x = point[0] * cos_theta - point[1] * sin_theta
            rotated_y = point[0] * sin_theta + point[1] * cos_theta
            
            world_x = x + rotated_x
            world_y = y + rotated_y
            
            transformed_points.append([world_x, world_y])
        
        return np.array(transformed_points)
    
    # NEW: Create inflated obstacle map
    def create_inflated_map(self):
        """Create an inflated obstacle map for better collision avoidance"""
        if self.map_data is None:
            return None
            
        # Convert occupancy grid to binary obstacle map
        grid_data = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )
        
        # Create binary obstacle map (1 = obstacle, 0 = free)
        obstacle_map = np.zeros_like(grid_data, dtype=np.uint8)
        
        # MODIFIED: Increased threshold from 0.7 to 0.8 to be less sensitive
        obstacle_map[grid_data >= self.obstacle_threshold * 0.8] = 1  # Changed from 0.7 to 0.8 to be less sensitive
        
        # Treat unknown areas as obstacles if configured
        if self.treat_unknown_as_obstacle:
            obstacle_map[grid_data == -1] = 1
        
        # NEW: Add waypoints to obstacle map if enabled
        if hasattr(self, 'respect_waypoints') and self.respect_waypoints and hasattr(self, 'waypoints') and self.waypoints:
            self.get_logger().info(f"Adding {len(self.waypoints)} waypoints to obstacle map")
            
            # Calculate waypoint inflation radius in grid cells
            waypoint_inflation = int(self.waypoint_safety_distance / self.map_data.info.resolution)
            
            # Add each waypoint to the obstacle map
            for wp_x, wp_y in self.waypoints:
                # Convert waypoint to grid coordinates
                grid_x = int((wp_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                grid_y = int((wp_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
                
                # Check if in bounds
                if (0 <= grid_x < self.map_data.info.width and 
                    0 <= grid_y < self.map_data.info.height):
                    # Mark waypoint as obstacle
                    obstacle_map[grid_y, grid_x] = 1
        
        # Calculate inflation radius in grid cells
        inflation_cells = int(self.obstacle_inflation_radius / self.map_data.info.resolution)
        
        # MODIFIED: Reduced extra inflation for waypoints
        waypoint_inflation = inflation_cells  # Removed the +1 to reduce inflation
        
        if inflation_cells > 0:
            # Create structuring element for dilation
            y, x = np.ogrid[-waypoint_inflation:waypoint_inflation+1, -waypoint_inflation:waypoint_inflation+1]
            kernel = x*x + y*y <= waypoint_inflation*waypoint_inflation
            
            # Dilate obstacles
            inflated_map = binary_dilation(obstacle_map, structure=kernel).astype(np.uint8)
        else:
            inflated_map = obstacle_map
        
        return inflated_map
    
    # NEW: Create distance transform for smooth cost gradients
    def create_distance_transform(self):
        """Create distance transform for smooth navigation around obstacles"""
        if self.inflated_map is None:
            return None
            
        # Create distance transform (distance to nearest obstacle)
        free_space = 1 - self.inflated_map  # Invert: 1 = free, 0 = obstacle
        distance_transform = distance_transform_edt(free_space)
        
        # Scale by map resolution to get distances in meters
        distance_transform *= self.map_data.info.resolution
        
        return distance_transform
    
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
    def map_callback(self, msg):
        """Process incoming map data and update internal cost map"""
        map_changed = (self.map_data is None or 
                      len(self.map_data.data) != len(msg.data) or
                      np.any(np.array(self.map_data.data) != np.array(msg.data)))
        
        self.map_data = msg
        
        # Update our internal map representation
        self.cost_map['resolution'] = msg.info.resolution
        self.cost_map['width'] = msg.info.width
        self.cost_map['height'] = msg.info.height
        self.cost_map['origin'] = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.cost_map['data'] = np.array(msg.data).flatten()
        
        # ENHANCED: Log map statistics for debugging
        if map_changed:
            # Count obstacles and free cells
            obstacle_count = np.sum(np.array(msg.data) >= self.obstacle_threshold)
            free_count = np.sum(np.array(msg.data) < self.obstacle_threshold)
            unknown_count = np.sum(np.array(msg.data) == -1)
            total_cells = len(msg.data)
            
            self.get_logger().info(f"Map statistics: {obstacle_count} obstacles, {free_count} free cells, "
                                  f"{unknown_count} unknown cells out of {total_cells} total")
            
            # Update inflated map and distance transform
            self.inflated_map = self.create_inflated_map()
            if self.use_distance_transform and self.inflated_map is not None:
                self.distance_transform = self.create_distance_transform()
            
            self.last_map_update_time = self.get_clock().now()
            self.get_logger().info('Map updated - recreated inflated map and distance transform')
        
        self.get_logger().debug('Received map update')
    
    def heuristic(self, a, b):
        """Calculate heuristic distance between nodes with unexplored penalty"""
        # Base euclidean distance
        base_cost = self.heuristic_weight * np.hypot(a.x - b.x, a.y - b.y)
        
        # NEW: Add penalty for path through unexplored area
        if self.is_unexplored(a.x, a.y):
            base_cost *= self.unknown_cost_multiplier
        
        # NEW: Add distance-based cost gradient
        if self.use_distance_transform and self.distance_transform is not None:
            distance_cost = self.get_distance_cost(a.x, a.y)
            base_cost += distance_cost
            
        # NEW: Add waypoint consideration
        if hasattr(self, 'respect_waypoints') and self.respect_waypoints and hasattr(self, 'waypoints') and self.waypoints:
            # If we're following waypoints, add cost for being far from waypoints
            if self.follow_waypoints:
                closest_wp = self.find_closest_waypoint(a.x, a.y)
                if closest_wp:
                    _, wp_x, wp_y = closest_wp
                    wp_dist = np.hypot(a.x - wp_x, a.y - wp_y)
                    # Add cost for being far from waypoints, but don't penalize too much
                    base_cost += wp_dist * 0.5
            
            # If we're near a waypoint, add cost to discourage cutting across waypoints
            if self.is_near_waypoint(a.x, a.y):
                base_cost *= self.waypoint_cost_multiplier
            
        return base_cost
    
    # NEW: Get cost based on distance to obstacles
    def get_distance_cost(self, x, y):
        """Calculate cost based on distance to nearest obstacle"""
        if self.distance_transform is None or self.map_data is None:
            return 0.0
            
        # Convert world coordinates to grid coordinates
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check bounds
        if not (0 <= grid_x < self.map_data.info.width and 
                0 <= grid_y < self.map_data.info.height):
            return self.max_distance_cost
            
        # Get distance to nearest obstacle
        distance = self.distance_transform[grid_y, grid_x]
        
        # Calculate cost - higher cost closer to obstacles
        if distance <= 0:
            return self.max_distance_cost
        elif distance >= self.min_safe_distance:
            return 0.0
        else:
            # Exponential cost increase as we get closer to obstacles
            normalized_distance = distance / self.min_safe_distance
            cost = self.distance_cost_weight * (1.0 - normalized_distance) ** 2
            return min(cost, self.max_distance_cost)
    
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
    
    # NEW: Enhanced collision checking with vehicle footprint
    def is_collision_footprint(self, x, y, theta):
        """Check collision using full vehicle footprint"""
        if self.inflated_map is None or self.map_data is None:
            base_collision = self.is_obstacle(x, y)
        else:
            # Get transformed vehicle footprint
            footprint_points = self.transform_footprint(x, y, theta)
            
            # Check collision for multiple points along the footprint
            for i in range(len(footprint_points)):
                # Current corner
                p1 = footprint_points[i]
                # Next corner (with wraparound)
                p2 = footprint_points[(i + 1) % len(footprint_points)]
                
                # Check points along the edge
                num_checks = self.collision_check_resolution
                for j in range(num_checks + 1):
                    t = j / num_checks
                    check_x = p1[0] * (1 - t) + p2[0] * t
                    check_y = p1[1] * (1 - t) + p2[1] * t
                    
                    if self.is_obstacle_inflated(check_x, check_y):
                        return True
            
            # Also check the center point
            base_collision = self.is_obstacle_inflated(x, y)
        
        # MODIFIED: Only check waypoints if base collision check passed and waypoints are available
        if base_collision:
            return True
            
        # MODIFIED: Make waypoint collision check less aggressive
        if hasattr(self, 'respect_waypoints') and self.respect_waypoints and hasattr(self, 'waypoints') and self.waypoints:
            # Only consider waypoints that are very close (reduced from waypoint_safety_distance)
            for wp_x, wp_y in self.waypoints:
                dist = np.hypot(x - wp_x, y - wp_y)
                if dist < self.waypoint_safety_distance * 0.7:  # Reduced sensitivity
                    return True
            
        return False
    
    # NEW: Check obstacle in inflated map
    def is_obstacle_inflated(self, x, y):
        """Check if point is obstacle in inflated map"""
        if self.inflated_map is None or self.map_data is None:
            return self.is_obstacle(x, y)
            
        # Convert world coordinates to grid coordinates  
        grid_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        grid_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        # Check if in bounds
        if not (0 <= grid_x < self.map_data.info.width and 
                0 <= grid_y < self.map_data.info.height):
            return True
            
        # Check inflated map with enhanced sensitivity
        # Use a more sensitive check to detect even faint obstacles in the unified map
        # MODIFIED: Increased threshold from 0.5 to 0.7 to be less sensitive
        return self.inflated_map[grid_y, grid_x] >= 0.7  # Changed from 0.5 to 0.7 to be less sensitive
    
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
                
            # Check for unexplored cells
            cell_value = self.cost_map['data'][index]
            if cell_value == -1 and self.treat_unknown_as_obstacle:
                return True
            
            # MODIFIED: Increased threshold from 0.7 to 0.8 to be less sensitive
            return cell_value >= self.obstacle_threshold * 0.8  # Changed from 0.7 to 0.8 to be less sensitive
        
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
            
        # Check for unexplored cells
        cell_value = self.map_data.data[index]
        if cell_value == -1 and self.treat_unknown_as_obstacle:
            return True
            
        # MODIFIED: Increased threshold from 0.7 to 0.8 to be less sensitive
        return cell_value >= self.obstacle_threshold * 0.8  # Changed from 0.7 to 0.8 to be less sensitive
    
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
            
            # Skip if out of bounds
            if not self.is_in_bounds(x, y):
                continue
            
            # ENHANCED: More thorough collision checking by checking intermediate points
            # This helps prevent cutting corners through obstacles
            collision = False
            
            # Check intermediate points along the path for collision
            num_checks = 5  # Check 5 intermediate points
            for i in range(1, num_checks + 1):
                t = i / (num_checks + 1)
                check_x = node.x + t * (x - node.x)
                check_y = node.y + t * (y - node.y)
                check_theta = node.theta + t * (theta - node.theta)
                
                if self.is_collision_footprint(check_x, check_y, check_theta):
                    collision = True
                    break
            
            if collision or self.is_collision_footprint(x, y, theta):
                continue
                
            # Calculate cost (distance traveled plus steering penalty)
            distance_cost = np.hypot(x - node.x, y - node.y)
            steering_cost = 0.1 * abs(steering)  # Small penalty for steering
            
            # Add distance-based cost for smooth navigation
            obstacle_cost = 0.0
            if self.use_distance_transform:
                obstacle_cost = self.get_distance_cost(x, y)
            
            total_cost = node.cost + distance_cost + steering_cost + obstacle_cost
            
            # Create new successor node
            successor = Node2D(x, y, theta, total_cost, parent=node)
            successors.append(successor)
            
        return successors
    
    def plan(self, start_node, goal_node):
        """Hybrid A* path planning algorithm"""
        if start_node is None or goal_node is None:
            self.get_logger().warn("Cannot plan: start or goal is None")
            return None
            
        # NEW: Check if start or goal positions are in collision
        start_collision = self.is_collision_footprint(start_node.x, start_node.y, start_node.theta)
        if start_collision:
            # Instead of immediately failing, try to find a nearby non-collision start point
            self.get_logger().warn("Start position is in collision! Attempting to find a valid start position nearby...")
            
            # Try to find a valid start position in a small area around the original start
            valid_start = self.find_valid_position_nearby(start_node.x, start_node.y, start_node.theta, is_start=True)
            if valid_start:
                self.get_logger().info(f"Found valid start position at ({valid_start.x}, {valid_start.y})")
                start_node = valid_start
            else:
                self.get_logger().error("Could not find a valid start position nearby!")
                return None
        
        goal_collision = self.is_collision_footprint(goal_node.x, goal_node.y, goal_node.theta)
        if goal_collision:
            # Try to find a valid goal position nearby
            self.get_logger().warn("Goal position is in collision! Attempting to find a valid goal position nearby...")
            valid_goal = self.find_valid_position_nearby(goal_node.x, goal_node.y, goal_node.theta, is_start=False)
            if valid_goal:
                self.get_logger().info(f"Found valid goal position at ({valid_goal.x}, {valid_goal.y})")
                goal_node = valid_goal
            else:
                self.get_logger().error("Could not find a valid goal position nearby!")
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
    
    # NEW: Add method to find a valid position nearby
    def find_valid_position_nearby(self, x, y, theta, is_start=True, search_radius=2.0, num_samples=20):
        """Find a valid (collision-free) position near the given position"""
        # First, try with reduced collision sensitivity
        if is_start:
            # For start positions, we're more lenient with collision checking
            original_threshold = self.obstacle_threshold
            original_safety_margin = self.safety_margin
            original_inflation_radius = self.obstacle_inflation_radius
            
            # Temporarily reduce sensitivity
            self.obstacle_threshold = self.obstacle_threshold * 1.5  # Higher threshold = fewer obstacles
            self.safety_margin = self.safety_margin * 0.5  # Smaller safety margin
            self.obstacle_inflation_radius = self.obstacle_inflation_radius * 0.5  # Smaller inflation radius
            
            # Recalculate vehicle footprint with reduced safety margin
            original_footprint = self.vehicle_footprint
            self.vehicle_footprint = self.calculate_vehicle_footprint()
            
            # Check if position is now valid with reduced sensitivity
            if not self.is_collision_footprint(x, y, theta):
                # Create a new node with the same position but reduced sensitivity
                valid_node = Node2D(x, y, theta, 0.0)
                
                # Restore original settings
                self.obstacle_threshold = original_threshold
                self.safety_margin = original_safety_margin
                self.obstacle_inflation_radius = original_inflation_radius
                self.vehicle_footprint = original_footprint
                
                return valid_node
            
            # Restore original settings
            self.obstacle_threshold = original_threshold
            self.safety_margin = original_safety_margin
            self.obstacle_inflation_radius = original_inflation_radius
            self.vehicle_footprint = original_footprint
        
        # If reduced sensitivity didn't work or it's a goal position, try different positions
        best_dist = float('inf')
        best_node = None
        
        # Try positions in a spiral pattern around the original position
        for i in range(num_samples):
            # Calculate radius and angle for spiral pattern
            radius = search_radius * (i / num_samples)
            angle = 2 * np.pi * i / num_samples
            
            # Calculate new position
            new_x = x + radius * np.cos(angle)
            new_y = y + radius * np.sin(angle)
            
            # Check if this position is valid
            if not self.is_collision_footprint(new_x, new_y, theta):
                # Calculate distance from original position
                dist = np.hypot(new_x - x, new_y - y)
                
                # Keep track of closest valid position
                if dist < best_dist:
                    best_dist = dist
                    best_node = Node2D(new_x, new_y, theta, 0.0)
        
        return best_node
    
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
    
    # NEW: Enhanced debug visualization function
    def publish_debug_map(self):
        """Publish debug maps showing explored/unexplored areas and inflated obstacles"""
        if not self.publish_debug_viz or self.map_data is None:
            return
            
        # Create a copy of the map for debug visualization
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
        
        # NEW: Publish inflated map for visualization
        if self.inflated_map is not None:
            inflated_map_msg = OccupancyGrid()
            inflated_map_msg.header = self.map_data.header
            inflated_map_msg.info = self.map_data.info
            
            # Convert inflated map to occupancy grid format
            inflated_data = self.inflated_map.flatten() * 100  # Convert to 0-100 scale
            inflated_map_msg.data = inflated_data.astype(np.int8).tolist()
            
            self.inflated_map_pub.publish(inflated_map_msg)
    
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
            if not self.use_rviz_goal:
                # Create a default goal if not using RViz goal
                self.goal_pose = PoseStamped()
                self.goal_pose.header.frame_id = self.map_frame_id
                self.goal_pose.pose.position.x = self.goal_x
                self.goal_pose.pose.position.y = self.goal_y
                self.goal_pose.pose.position.z = 0.0
                
                # Convert yaw to quaternion
                self.goal_pose.pose.orientation.x = 0.0
                self.goal_pose.pose.orientation.y = 0.0
                self.goal_pose.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
                self.goal_pose.pose.orientation.w = math.cos(self.goal_yaw / 2.0)
                
                self.get_logger().info(f'Using default goal position: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
            else:
                self.get_logger().warn("No goal pose available")
                return
        
        # Check if we need to replan
        need_replanning = False
        
        # Always plan on first run
        if self.latest_path is None:
            need_replanning = True
        
        # NEW: Check if map has changed significantly
        if self.replan_on_map_change and self.last_map_update_time is not None:
            # Replan if map was updated recently
            time_since_update = (self.get_clock().now() - self.last_map_update_time).nanoseconds / 1e9
            if time_since_update < 1.0:  # Map updated within last second
                need_replanning = True
                self.get_logger().info("Replanning due to map update")
        
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
        self.get_logger().info(f"  Vehicle: {self.vehicle_length}m x {self.vehicle_width}m")
        self.get_logger().info(f"  Safety margin: {self.safety_margin} m")
        self.get_logger().info(f"  Obstacle inflation: {self.obstacle_inflation_radius} m")
        self.get_logger().info(f"  Obstacle threshold: {self.obstacle_threshold}")
        self.get_logger().info(f"  Max iterations: {self.max_iterations}")
        self.get_logger().info(f"  Motion resolution: {self.motion_resolution}")
        self.get_logger().info(f"  Angle resolution: {self.angle_resolution}")
        self.get_logger().info(f"  Heuristic weight: {self.heuristic_weight}")
        self.get_logger().info(f"  Use distance transform: {self.use_distance_transform}")
        self.get_logger().info(f"  Distance cost weight: {self.distance_cost_weight}")
        self.get_logger().info(f"  Min safe distance: {self.min_safe_distance} m")
        self.get_logger().info(f"  Treat unknown as obstacle: {self.treat_unknown_as_obstacle}")
        self.get_logger().info(f"  Unknown cost multiplier: {self.unknown_cost_multiplier}")
        self.get_logger().info(f"  Min explored percentage: {self.min_explored_percentage}%")
        
        # Log fixed start and goal parameters
        if self.use_fixed_start:
            self.get_logger().info(f"  Using fixed start: ({self.start_x}, {self.start_y}, {self.start_yaw})")
        
        if not self.use_rviz_goal:
            self.get_logger().info(f"  Using fixed goal: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})")
        else:
            self.get_logger().info(f"  Using RViz goal from topic: {self.goal_topic}")
            
        # NEW: Log waypoint parameters
        self.get_logger().info(f"  Respect waypoints: {self.respect_waypoints}")
        if self.respect_waypoints:
            self.get_logger().info(f"  Waypoint topic: {self.waypoint_topic}")
            self.get_logger().info(f"  Waypoint safety distance: {self.waypoint_safety_distance} m")
            self.get_logger().info(f"  Waypoint cost multiplier: {self.waypoint_cost_multiplier}")
            self.get_logger().info(f"  Follow waypoints: {self.follow_waypoints}")

    # NEW: Add waypoint callback
    def waypoint_callback(self, msg):
        """Process incoming waypoint data"""
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')

    # NEW: Add method to check if a point is too close to waypoints
    def is_near_waypoint(self, x, y):
        """Check if a point is too close to a waypoint"""
        if not hasattr(self, 'waypoints') or not self.waypoints:
            return False
            
        for wp_x, wp_y in self.waypoints:
            dist = np.hypot(x - wp_x, y - wp_y)
            if dist < self.waypoint_safety_distance:
                return True
                
        return False
    
    # NEW: Add method to find closest waypoint
    def find_closest_waypoint(self, x, y):
        """Find the closest waypoint to a given position"""
        if not hasattr(self, 'waypoints') or not self.waypoints:
            return None
            
        min_dist = float('inf')
        closest_wp = None
        
        for i, (wp_x, wp_y) in enumerate(self.waypoints):
            dist = np.hypot(x - wp_x, y - wp_y)
            if dist < min_dist:
                min_dist = dist
                closest_wp = (i, wp_x, wp_y)
                
        return closest_wp

def main(args=None):
    rclpy.init(args=args)
    planner = HybridAStarPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()