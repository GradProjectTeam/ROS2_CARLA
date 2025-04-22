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

class LidarCostmapEnhanced(Node):
    """
    Enhanced version of the LiDAR costmap creator with improved free space detection.
    
    This node:
    1. Subscribes to PointCloud2 from lidar_listener_clusters
    2. Implements a three-state system:
       - -1 = unknown (gray)
       - 0 = known free (black)
       - 1-100 = obstacles with varying confidence (white)
    3. Uses ray casting to mark free space between the sensor and obstacles
    4. Publishes an enhanced costmap for use with Nav2
    """
    def __init__(self):
        super().__init__('lidar_costmap_enhanced')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.1)       # meters per cell
        self.declare_parameter('map_width', 1000)           # cells
        self.declare_parameter('map_height', 1000)          # cells
        self.declare_parameter('map_origin_x', -50.0)       # meters
        self.declare_parameter('map_origin_y', -50.0)       # meters
        self.declare_parameter('publish_rate', 5.0)         # Hz
        self.declare_parameter('ground_threshold', 0.1)     # meters above ground to consider an obstacle
        self.declare_parameter('max_points_per_cell', 5)    # Maximum points per cell for confidence
        self.declare_parameter('min_height', -5.0)          # Minimum height to consider (m)
        self.declare_parameter('max_height', 5.0)           # Maximum height to consider (m)
        self.declare_parameter('obstacle_inflation', 0.4)   # meters to inflate obstacles
        self.declare_parameter('max_data_age', 60.0)        # seconds
        self.declare_parameter('sensor_height', 1.0)        # height of the LiDAR sensor above ground
        self.declare_parameter('sensor_x', 0.0)             # x position of sensor in map frame
        self.declare_parameter('sensor_y', 0.0)             # y position of sensor in map frame
        
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
        self.sensor_height = self.get_parameter('sensor_height').value
        self.sensor_x = self.get_parameter('sensor_x').value
        self.sensor_y = self.get_parameter('sensor_y').value
        
        # Initialize costmap layer and point tracking
        # For the enhanced version, we use a three-state system:
        # -1 = unknown
        # 0 = known free
        # 1-100 = obstacles with varying confidence
        self.lidar_costmap = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
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
            '/lidar_costmap_enhanced', 
            reliable_qos
        )
        
        # Debug publisher for visualization
        self.debug_publisher = self.create_publisher(
            OccupancyGrid,
            '/lidar_costmap_enhanced_debug',
            10
        )
        
        # Subscribers - Updated to match topics from lidar_listener_clusters
        self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            10
        )
        
        # Also subscribe to marker data for additional information
        self.create_subscription(
            MarkerArray,
            '/lidar/markers',
            self.marker_callback,
            10
        )
        
        # Create timer for costmap generation and publishing
        self.create_timer(1.0/self.publish_rate, self.update_and_publish_costmap)
        
        self.get_logger().info('Enhanced LiDAR Costmap Creator initialized')
    
    def lidar_callback(self, msg):
        """Process LiDAR point cloud data from lidar_listener_clusters"""
        with self.costmap_lock:
            try:
                self.lidar_points = list(point_cloud2.read_points(msg))
                self.lidar_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
                self.get_logger().debug(f'Received {len(self.lidar_points)} LiDAR points')
            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR data: {str(e)}')
    
    def marker_callback(self, msg):
        """Process marker data from lidar_listener_clusters for additional metadata"""
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
        
        # Enhanced filtering with more permissive settings
        filtered_points = []
        ground_points = 0
        outside_range_points = 0
        valid_points = 0
        
        for point in points:
            # Extract x, y, z from point
            if len(point) >= 3:
                x, y, z = point[0], point[1], point[2]
                
                # Accept points within the height range
                if not (self.min_height <= z <= self.max_height):
                    outside_range_points += 1
                    continue
                
                # Only filter exact ground points
                if abs(z) < 0.01:  # Almost exactly 0
                    ground_points += 1
                    continue
                
                # Point passed all filters
                valid_points += 1
                filtered_points.append((x, y, z))
        
        # Add debugging output to understand what's happening with filtering
        total_points = len(points)
        if total_points > 0:
            self.get_logger().info(
                f"Point filtering: {total_points} total, {valid_points} valid, "
                f"{ground_points} filtered as ground, {outside_range_points} outside height range. "
                f"min_height={self.min_height}, max_height={self.max_height}, ground_threshold={self.ground_threshold}"
            )
        
        return filtered_points
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    def bresenham_line(self, x0, y0, x1, y1):
        """
        Bresenham's line algorithm for ray tracing.
        Returns list of cells (grid_y, grid_x) along the line from (x0,y0) to (x1,y1).
        """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            # Only add points that are within the grid
            if (0 <= x0 < self.map_width and 0 <= y0 < self.map_height):
                points.append((y0, x0))  # numpy uses (row, col) order
                
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
        return points
    
    def ray_trace_to_obstacles(self, sensor_grid_x, sensor_grid_y, obstacle_cells):
        """
        Mark cells along rays from the sensor to obstacles as free space (0)
        Returns a set of cells marked as free (grid_y, grid_x)
        """
        free_cells = set()
        
        # For each obstacle cell, trace a ray from the sensor
        for grid_y, grid_x in obstacle_cells:
            # Get cells along the ray
            ray_cells = self.bresenham_line(sensor_grid_x, sensor_grid_y, grid_x, grid_y)
            
            # All cells along the ray except the last one (the obstacle) are free
            if len(ray_cells) > 1:
                for cell in ray_cells[:-1]:  # Exclude the last cell (obstacle)
                    free_cells.add(cell)
        
        return free_cells
    
    def update_costmap(self):
        """Update costmap based on LiDAR data with enhanced free space detection"""
        # If we have no data, return but don't log a warning
        if self.lidar_points is None or self.lidar_timestamp is None:
            return False
        
        # Temporarily disable freshness check to allow continuous processing
        # if not self.is_data_fresh():
        #     return False
        
        with self.costmap_lock:
            # Start with a fresh costmap where all cells are unknown (-1)
            self.lidar_costmap.fill(-1)
            
            # Filter points
            filtered_points = self.filter_ground_and_noise(self.lidar_points)
            
            # Use raw points as fallback if filtering removed everything
            if not filtered_points:
                self.get_logger().warn('No valid LiDAR points after filtering, using raw points as fallback')
                filtered_points = [(p[0], p[1], p[2]) for p in self.lidar_points if len(p) >= 3]
                
                if not filtered_points:
                    self.get_logger().error('No usable LiDAR points at all')
                    return False
            
            self.get_logger().info(f"Creating enhanced costmap with {len(filtered_points)} filtered points")
            
            # Track obstacle cells for ray tracing
            obstacle_cells = set()
            cell_point_count = {}
            
            # First pass: Count points per cell and track obstacle locations
            for x, y, z in filtered_points:
                # Convert world coordinates to grid coordinates
                grid_x, grid_y = self.world_to_grid(x, y)
                
                # Skip if out of bounds
                if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                    continue
                
                # Track this cell as an obstacle
                obstacle_cells.add((grid_y, grid_x))
                
                # Count points per cell
                cell_key = (grid_y, grid_x)  # numpy uses (row, col) order
                if cell_key in cell_point_count:
                    cell_point_count[cell_key] += 1
                else:
                    cell_point_count[cell_key] = 1
            
            # Get sensor position in grid coordinates
            sensor_grid_x, sensor_grid_y = self.world_to_grid(self.sensor_x, self.sensor_y)
            
            # Second pass: Ray tracing to mark free space
            free_cells = self.ray_trace_to_obstacles(sensor_grid_x, sensor_grid_y, obstacle_cells)
            
            # Mark free cells with value 0
            for grid_y, grid_x in free_cells:
                self.lidar_costmap[grid_y, grid_x] = 0
            
            # Third pass: Assign cost values to obstacle cells
            for (grid_y, grid_x), count in cell_point_count.items():
                # Make even a single point have significant weight
                confidence = min(count / max(1, self.max_points_per_cell * 0.5), 1.0)
                
                # Assign higher cost (70-100) to make obstacles more visible
                cost = int(70 + 30 * confidence)
                self.lidar_costmap[grid_y, grid_x] = cost
            
            # Fourth pass: Inflate obstacles
            if self.obstacle_inflation > 0:
                # Create a copy of the original costmap
                original_costmap = np.copy(self.lidar_costmap)
                
                # Calculate inflation radius in grid cells
                inflation_radius = max(4, int(self.obstacle_inflation / self.map_resolution))
                
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
                                # Use slower decay for sparse point clouds
                                decay_factor = max(0, 1.0 - (distance / inflation_radius) ** 0.7)
                                inflated_cost = int(original_cost * decay_factor)
                                
                                # Only update if the new cost is higher and the cell isn't free
                                # This prevents inflation from overwriting known free space
                                if inflated_cost > self.lidar_costmap[ny, nx] and original_costmap[ny, nx] != 0:
                                    self.lidar_costmap[ny, nx] = inflated_cost
            
            # Count and log occupied, free, and unknown cells
            occupied_cells = np.count_nonzero(self.lidar_costmap > 0)
            free_cells = np.count_nonzero(self.lidar_costmap == 0)
            unknown_cells = np.count_nonzero(self.lidar_costmap < 0)
            
            self.get_logger().info(
                f'Enhanced costmap created with {occupied_cells} occupied cells, '
                f'{free_cells} free cells, and {unknown_cells} unknown cells'
            )
            
            return True
    
    def update_and_publish_costmap(self):
        """Update and publish the enhanced LiDAR costmap"""
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
        # Convert our int8 values (-1 to 100) to the OccupancyGrid expected range (-1 to 100)
        costmap_msg.data = self.lidar_costmap.flatten().tolist()
        
        # Publish costmap
        self.costmap_publisher.publish(costmap_msg)
        self.debug_publisher.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = LidarCostmapEnhanced()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 