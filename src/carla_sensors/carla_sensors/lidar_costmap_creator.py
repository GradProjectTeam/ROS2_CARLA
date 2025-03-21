#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import tf2_ros
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from threading import Lock

class LidarCostmapCreator(Node):
    """
    Converts LiDAR points to a costmap layer for navigation.
    
    This node:
    1. Subscribes to PointCloud2 from lidar_listener_clusters
    2. Filters out ground plane and noise
    3. Creates a static costmap with obstacles
    4. Publishes the costmap for use with Nav2
    """
    def __init__(self):
        super().__init__('lidar_costmap_creator')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.1)       # meters per cell
        self.declare_parameter('map_width', 1000)           # cells (100m x 100m with 0.1m resolution)
        self.declare_parameter('map_height', 1000)          # cells
        self.declare_parameter('map_origin_x', -50.0)       # meters
        self.declare_parameter('map_origin_y', -50.0)       # meters
        self.declare_parameter('publish_rate', 5.0)         # Hz
        self.declare_parameter('ground_threshold', 0.3)     # meters above ground to consider an obstacle
        self.declare_parameter('max_points_per_cell', 5)    # Maximum points per cell for confidence
        self.declare_parameter('min_height', -2.0)          # Minimum height to consider (m)
        self.declare_parameter('max_height', 3.0)           # Maximum height to consider (m)
        self.declare_parameter('obstacle_inflation', 0.4)   # meters to inflate obstacles
        self.declare_parameter('max_data_age', 1.0)         # seconds
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.ground_threshold = self.get_parameter('ground_threshold').value
        self.max_points_per_cell = self.get_parameter('max_points_per_cell').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.max_data_age = self.get_parameter('max_data_age').value
        
        # Initialize costmap layer and point tracking
        self.lidar_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.lidar_points = None
        self.lidar_timestamp = None
        
        # Thread safety
        self.costmap_lock = Lock()
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create reliable QoS profile for costmap publishing
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/lidar_costmap', 
            reliable_qos
        )
        
        # Debug publisher
        self.debug_publisher = self.create_publisher(
            OccupancyGrid,
            '/lidar_costmap_debug',
            10
        )
        
        # Subscribers - Updated to match topics from lidar_listener_clusters
        self.create_subscription(
            PointCloud2,
            '/lidar/points',  # Main topic from lidar_listener_clusters
            self.lidar_callback,
            10
        )
        
        # Also subscribe to marker data for additional information
        self.create_subscription(
            MarkerArray,
            '/lidar/markers',  # Marker array from lidar_listener_clusters
            self.marker_callback,
            10
        )
        
        # Create timer for costmap generation and publishing
        self.create_timer(1.0/self.publish_rate, self.update_and_publish_costmap)
        
        self.get_logger().info('LiDAR Costmap Creator initialized')
    
    def lidar_callback(self, msg):
        """Process LiDAR point cloud data from lidar_listener_clusters"""
        with self.costmap_lock:
            try:
                self.lidar_points = list(point_cloud2.read_points(msg))
                self.lidar_timestamp = self.get_clock().now()
                self.get_logger().debug(f'Received {len(self.lidar_points)} LiDAR points')
            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR data: {str(e)}')
    
    def marker_callback(self, msg):
        """Process marker data from lidar_listener_clusters for additional metadata"""
        # This callback can be used to extract additional information from markers
        # such as cluster IDs, convex hulls, etc.
        cluster_count = len(msg.markers)
        if cluster_count > 0:
            self.get_logger().debug(f'Received markers for {cluster_count} LiDAR clusters')
    
    def is_data_fresh(self):
        """Check if LiDAR data is fresh enough to use"""
        if self.lidar_points is None or self.lidar_timestamp is None:
            return False
        
        current_time = self.get_clock().now()
        lidar_age = (current_time - self.lidar_timestamp).nanoseconds / 1e9
        
        if lidar_age > self.max_data_age:
            self.get_logger().warn(f'LiDAR data too old: {lidar_age:.2f}s')
            return False
        
        return True
    
    def filter_ground_and_noise(self, points):
        """Filter out ground plane and noise points"""
        if not points:
            return []
        
        # This is a simple height-based filter
        # For production, a RANSAC-based ground plane estimation would be better
        filtered_points = []
        
        for point in points:
            # Extract x, y, z from point
            # Format depends on your PointCloud2 field configuration
            if len(point) >= 3:
                x, y, z = point[0], point[1], point[2]
                
                # Filter by height
                if self.min_height <= z <= self.max_height and abs(z) > self.ground_threshold:
                    filtered_points.append((x, y, z))
        
        return filtered_points
    
    def update_costmap(self):
        """Update costmap based on LiDAR data"""
        if not self.is_data_fresh():
            return False
        
        with self.costmap_lock:
            # Start with a fresh costmap
            self.lidar_costmap.fill(0)  # 0 = free space
            
            # Filter points
            filtered_points = self.filter_ground_and_noise(self.lidar_points)
            
            if not filtered_points:
                self.get_logger().warn('No valid LiDAR points after filtering')
                return False
            
            # Track how many points fall into each cell for confidence
            cell_point_count = {}
            
            # First pass: Count points per cell
            for x, y, z in filtered_points:
                # Convert world coordinates to grid coordinates
                grid_x = int((x - self.map_origin_x) / self.map_resolution)
                grid_y = int((y - self.map_origin_y) / self.map_resolution)
                
                # Skip if out of bounds
                if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                    continue
                
                # Count points per cell
                cell_key = (grid_y, grid_x)  # numpy uses (row, col) order
                if cell_key in cell_point_count:
                    cell_point_count[cell_key] += 1
                else:
                    cell_point_count[cell_key] = 1
            
            # Second pass: Assign cost values based on point density
            for (grid_y, grid_x), count in cell_point_count.items():
                # Calculate confidence level (0-100%)
                confidence = min(count / self.max_points_per_cell, 1.0)
                
                # Assign cost (60-90) based on confidence
                cost = int(60 + 30 * confidence)
                self.lidar_costmap[grid_y, grid_x] = cost
            
            # Third pass: Inflate obstacles
            if self.obstacle_inflation > 0:
                # Create a copy of the original costmap
                original_costmap = np.copy(self.lidar_costmap)
                
                # Calculate inflation radius in grid cells
                inflation_radius = int(self.obstacle_inflation / self.map_resolution)
                
                # Find all obstacle cells
                obstacle_cells = np.where(original_costmap > 0)
                
                # For each obstacle cell, inflate it
                for y, x in zip(obstacle_cells[0], obstacle_cells[1]):
                    original_cost = original_costmap[y, x]
                    
                    # Apply inflation to nearby cells
                    for dy in range(-inflation_radius, inflation_radius + 1):
                        for dx in range(-inflation_radius, inflation_radius + 1):
                            nx, ny = x + dx, y + dy
                            
                            # Skip if out of bounds
                            if not (0 <= nx < self.map_width and 0 <= ny < self.map_height):
                                continue
                            
                            # Calculate distance from obstacle
                            distance = math.sqrt(dx**2 + dy**2)
                            
                            # Only inflate within the radius
                            if distance <= inflation_radius:
                                # Cost decreases with distance from obstacle
                                decay_factor = max(0, 1.0 - (distance / inflation_radius))
                                inflated_cost = int(original_cost * decay_factor)
                                
                                # Only update if the new cost is higher
                                if inflated_cost > self.lidar_costmap[ny, nx]:
                                    self.lidar_costmap[ny, nx] = inflated_cost
            
            return True
    
    def update_and_publish_costmap(self):
        """Update and publish the LiDAR costmap"""
        # Update costmap
        if not self.update_costmap():
            return
        
        # Create occupancy grid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = 'map'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height = self.map_height
        costmap_msg.info.origin.position.x = self.map_origin_x
        costmap_msg.info.origin.position.y = self.map_origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        
        # Convert numpy array to 1D list
        costmap_msg.data = self.lidar_costmap.flatten().tolist()
        
        # Publish costmap
        self.costmap_publisher.publish(costmap_msg)
        self.debug_publisher.publish(costmap_msg)
        
        # Log statistics periodically
        if hasattr(self, 'log_count'):
            self.log_count += 1
            if self.log_count >= 50:  # Log every ~10 seconds at 5Hz
                self.log_count = 0
                occupied_cells = np.count_nonzero(self.lidar_costmap > 0)
                self.get_logger().info(f'LiDAR costmap: {occupied_cells} occupied cells')
        else:
            self.log_count = 0

def main(args=None):
    rclpy.init(args=args)
    
    node = LidarCostmapCreator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 