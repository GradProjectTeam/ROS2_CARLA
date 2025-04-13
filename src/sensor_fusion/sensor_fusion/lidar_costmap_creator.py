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
import traceback  # For better error diagnostics

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
        
        # Performance counters
        self.start_time = time.time()
        self.frames_processed = 0
        self.total_points_processed = 0
        
        # Declare parameters - Optimized for better performance
        self.declare_parameter('map_resolution', 0.1)      # Increased to 10cm for better performance (was 0.05)
        self.declare_parameter('map_width', 1000)          # Reduced from 2000 for better performance
        self.declare_parameter('map_height', 1000)         # Reduced from 2000 for better performance
        self.declare_parameter('map_width_meters', 100.0)  # meters - for proper scaling
        self.declare_parameter('map_height_meters', 100.0) # meters - for proper scaling
        self.declare_parameter('map_origin_x', -50.0)      # meters
        self.declare_parameter('map_origin_y', -50.0)      # meters
        self.declare_parameter('publish_rate', 5.0)        # Reduced to 5Hz for lower CPU usage (was 10)
        self.declare_parameter('process_rate', 10.0)       # New parameter to control processing frequency
        self.declare_parameter('ground_threshold', 0.15)   # Lower threshold to better detect low objects
        self.declare_parameter('max_points_per_cell', 3)   # Reduced for more sensitivity
        self.declare_parameter('min_height', -3.0)         # Lower min height for ditches/potholes
        self.declare_parameter('max_height', 4.0)          # Higher max height for tall objects
        self.declare_parameter('obstacle_inflation', 0.3)  # Smaller inflation for more precise object boundaries
        self.declare_parameter('max_data_age', 2.0)        # Accept slightly older data
        self.declare_parameter('raycast_skip', 2)          # New parameter: only raytrace every Nth point
        self.declare_parameter('max_points_to_process', 5000) # New parameter: limit points processed per cycle
        
        # Vehicle centering parameters
        self.declare_parameter('vehicle_x_offset', 0.0)     # X offset for vehicle position
        self.declare_parameter('vehicle_y_offset', 0.0)     # Y offset for vehicle position  
        self.declare_parameter('center_priority_radius', 5.0)  # Radius around vehicle to prioritize in costmap
        self.declare_parameter('vehicle_center_weight', 0.8)   # Weight for centering vehicle (0-1)
        
        # Costmap color values - Enhanced contrast for better visualization
        self.declare_parameter('free_space_value', 0)       # Value for free space (white)
        self.declare_parameter('obstacle_value', 100)       # Value for obstacles (black)
        self.declare_parameter('unknown_value', 50)         # Value for unknown areas (gray)
        self.declare_parameter('detection_radius', 35.0)    # Increased detection radius for better coverage
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_width_meters = self.get_parameter('map_width_meters').value
        self.map_height_meters = self.get_parameter('map_height_meters').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.process_rate = self.get_parameter('process_rate').value
        self.ground_threshold = self.get_parameter('ground_threshold').value
        self.max_points_per_cell = self.get_parameter('max_points_per_cell').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.max_data_age = self.get_parameter('max_data_age').value
        self.raycast_skip = self.get_parameter('raycast_skip').value
        self.max_points_to_process = self.get_parameter('max_points_to_process').value
        
        # Get vehicle centering parameters
        self.vehicle_x_offset = self.get_parameter('vehicle_x_offset').value
        self.vehicle_y_offset = self.get_parameter('vehicle_y_offset').value
        self.center_priority_radius = self.get_parameter('center_priority_radius').value
        self.vehicle_center_weight = self.get_parameter('vehicle_center_weight').value
        
        # Get costmap color parameters
        self.free_space_value = self.get_parameter('free_space_value').value
        self.obstacle_value = self.get_parameter('obstacle_value').value
        self.unknown_value = self.get_parameter('unknown_value').value
        self.detection_radius = self.get_parameter('detection_radius').value
        
        # Validate and adjust map dimensions if needed
        if self.map_width_meters > 0 and self.map_height_meters > 0:
            # Calculate map dimensions based on meters and resolution
            calculated_width = int(self.map_width_meters / self.map_resolution)
            calculated_height = int(self.map_height_meters / self.map_resolution)
            
            # Update map dimensions if they differ from calculation
            if calculated_width != self.map_width or calculated_height != self.map_height:
                self.get_logger().info(f"Adjusting map dimensions based on meter values: " 
                                     f"{self.map_width}x{self.map_height} cells -> "
                                     f"{calculated_width}x{calculated_height} cells")
                self.map_width = calculated_width
                self.map_height = calculated_height
                
            # Recalculate origin to center the map around (0,0) plus any offset
            self.map_origin_x = -self.map_width_meters/2 + self.vehicle_x_offset
            self.map_origin_y = -self.map_height_meters/2 + self.vehicle_y_offset
            self.get_logger().info(f"Map origin set to ({self.map_origin_x}, {self.map_origin_y}) " 
                                 f"with vehicle offset ({self.vehicle_x_offset}, {self.vehicle_y_offset})")
        
        # Initialize costmap layer and point tracking
        self.lidar_costmap = np.ones((self.map_height, self.map_width), dtype=np.int8) * self.unknown_value
        self.lidar_points = None
        self.lidar_timestamp = None
        self.new_data_available = False  # Flag to signal new data
        
        # Cache for common calculations
        self.detection_radius_cells = int(self.detection_radius / self.map_resolution)
        self.center_priority_radius_cells = int(self.center_priority_radius / self.map_resolution)
        
        # Processing timestamps for rate limiting
        self.last_process_time = 0
        self.process_interval = 1.0 / self.process_rate
        
        # Thread safety
        self.costmap_lock = Lock()
        self.data_lock = Lock()
        
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
        
        # Create separate timers for processing and publishing
        self.create_timer(1.0/self.process_rate, self.process_data)
        self.create_timer(1.0/self.publish_rate, self.publish_costmap)
        
        # Performance reporting timer
        self.create_timer(10.0, self.report_performance)
        
        self.get_logger().info(f'LiDAR Costmap Creator initialized with {self.map_width}x{self.map_height} grid at {self.map_resolution}m resolution')
        self.get_logger().info(f'Map dimensions: {self.map_width_meters}x{self.map_height_meters} meters')
        self.get_logger().info(f'Map origin: ({self.map_origin_x}, {self.map_origin_y}) meters')
        self.get_logger().info(f'Performance settings: process_rate={self.process_rate}Hz, publish_rate={self.publish_rate}Hz')
        self.get_logger().info(f'Point processing: max_points={self.max_points_to_process}, raycast_skip={self.raycast_skip}')
    
    def report_performance(self):
        """Report performance statistics"""
        elapsed = time.time() - self.start_time
        if elapsed > 0 and self.frames_processed > 0:
            fps = self.frames_processed / elapsed
            pps = self.total_points_processed / elapsed
            self.get_logger().info(
                f'Performance: {fps:.2f} frames/sec, {pps:.2f} points/sec, '
                f'processed {self.frames_processed} frames, {self.total_points_processed} points'
            )
        
        # Reset counters every 10 seconds for current statistics
        self.start_time = time.time()
        self.frames_processed = 0
        self.total_points_processed = 0
    
    def lidar_callback(self, msg):
        """Process LiDAR point cloud data from lidar_listener_clusters"""
        with self.data_lock:
            try:
                self.lidar_points = list(point_cloud2.read_points(msg))
                self.lidar_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
                self.new_data_available = True
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
            self.get_logger().debug(f'LiDAR data too old: {lidar_age:.2f}s')
            return False
        
        return True
    
    def process_data(self):
        """Process the LIDAR data at a controlled rate"""
        # Rate limiting 
        current_time = time.time()
        if (current_time - self.last_process_time) < self.process_interval:
            return
        
        with self.data_lock:
            # Check if we have new data to process
            if not self.new_data_available:
                return
            
            # Copy data for processing
            points_to_process = self.lidar_points
            self.new_data_available = False
        
        # Process the data and update the costmap
        if points_to_process and len(points_to_process) > 0:
            try:
                self.update_costmap(points_to_process)
                self.last_process_time = current_time
            except Exception as e:
                self.get_logger().error(f'Error in costmap processing: {str(e)}')
                self.get_logger().error(traceback.format_exc())
    
    def update_costmap(self, points):
        """Update costmap based on LiDAR data"""
        process_start = time.time()
        
        # Limit points to process for consistent performance
        if len(points) > self.max_points_to_process:
            # Subsample points evenly across the full set
            step = len(points) // self.max_points_to_process
            points = points[::step]
            self.get_logger().debug(f'Limited to {len(points)} points for processing')
        
        with self.costmap_lock:
            # Start with unknown space
            self.lidar_costmap.fill(self.unknown_value)  # Unknown space (gray)
            
            # Use raw points directly without filtering
            raw_points = [(p[0], p[1], p[2]) for p in points if len(p) >= 3]
            
            if not raw_points:
                self.get_logger().warn('No usable LiDAR points at all')
                return False
            
            # Update performance counters
            self.frames_processed += 1
            self.total_points_processed += len(raw_points)
            
            # Log how many points we're using
            self.get_logger().debug(f"Creating costmap with {len(raw_points)} raw points")
            
            # Identify the center of the sensor - account for LIDAR mounting position 
            # Use lidar mounting position from CARLA (1.5m forward, 2.0m high)
            sensor_x = 1.5 + self.vehicle_x_offset  # Forward offset from vehicle center
            sensor_y = 0.0 + self.vehicle_y_offset  # Apply any lateral offset
            
            # Look for the vehicle position in TF if available
            try:
                # Try to get the vehicle position from TF
                transform = self.tf_buffer.lookup_transform('map', 'ego_vehicle', rclpy.time.Time())
                vehicle_x = transform.transform.translation.x
                vehicle_y = transform.transform.translation.y
                
                # Update sensor position based on vehicle position
                sensor_x += vehicle_x
                sensor_y += vehicle_y
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass  # Just use default position, no need for logging
            
            # First pass: Mark detected free space and obstacles
            # Track cells that are known (detected)
            free_space_cells = set()
            obstacle_cells = set()
            
            # Track how many points fall into each cell for confidence
            cell_point_count = {}
            
            # First pass: Count points per cell and identify obstacles
            for x, y, z in raw_points:
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
                
                # Mark this cell as an obstacle
                obstacle_cells.add(cell_key)
            
            # Convert sensor position to grid coordinates
            sensor_grid_x = int((sensor_x - self.map_origin_x) / self.map_resolution)
            sensor_grid_y = int((sensor_y - self.map_origin_y) / self.map_resolution)
            
            # Ensure the sensor position is within map bounds for visualization
            if not (0 <= sensor_grid_x < self.map_width and 0 <= sensor_grid_y < self.map_height):
                # Clamp sensor position to map bounds for raytracing
                sensor_grid_x = max(0, min(self.map_width-1, sensor_grid_x))
                sensor_grid_y = max(0, min(self.map_height-1, sensor_grid_y))
            
            # Performance optimization: Only raytrace a subset of obstacles
            # This significantly reduces computation with minimal impact on visualization
            if self.raycast_skip > 1 and len(obstacle_cells) > 100:
                obstacle_list = list(obstacle_cells)
                # Choose every Nth obstacle for raytracing
                obstacles_to_raytrace = obstacle_list[::self.raycast_skip]
            else:
                obstacles_to_raytrace = obstacle_cells
            
            # Second pass: Mark free space based on sensor to obstacle raytracing
            for (obs_y, obs_x) in obstacles_to_raytrace:
                # Line from sensor to obstacle using Bresenham's algorithm
                line_points = self.bresenham_line(sensor_grid_y, sensor_grid_x, obs_y, obs_x)
                
                # All points along the line except the last are free space
                for i in range(len(line_points) - 1):  # Exclude the last point (obstacle)
                    y, x = line_points[i]
                    # Skip if out of bounds
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        free_space_cells.add((y, x))
            
            # Optimization: Use a smaller radius for nearby free space marking
            local_radius_cells = min(self.detection_radius_cells, 15)  # Limit to 15 cells radius for performance
            
            # Mark cells within reduced radius that aren't obstacles as free space
            row_indices, col_indices = np.indices((2*local_radius_cells+1, 2*local_radius_cells+1))
            row_indices = row_indices + sensor_grid_y - local_radius_cells
            col_indices = col_indices + sensor_grid_x - local_radius_cells
            
            # Calculate distances
            distances = np.sqrt((row_indices - sensor_grid_y)**2 + (col_indices - sensor_grid_x)**2)
            
            # Find cells within radius
            mask = distances <= local_radius_cells
            valid_rows = row_indices[mask]
            valid_cols = col_indices[mask]
            
            # Filter out of bounds
            in_bounds = (
                (valid_rows >= 0) & (valid_rows < self.map_height) &
                (valid_cols >= 0) & (valid_cols < self.map_width)
            )
            valid_rows = valid_rows[in_bounds]
            valid_cols = valid_cols[in_bounds]
            
            # Set as free space
            for y, x in zip(valid_rows, valid_cols):
                cell_key = (y, x)
                if cell_key not in obstacle_cells:
                    free_space_cells.add(cell_key)
            
            # Remove free space cells that are actually obstacles
            free_space_cells = free_space_cells - obstacle_cells
            
            # Apply the three-color scheme to the costmap
            # Unknown = self.unknown_value (50): already filled
            
            # Mark free space
            for y, x in free_space_cells:
                self.lidar_costmap[y, x] = self.free_space_value  # White (0)
            
            # Mark obstacles with confidence-based cost
            for (y, x), count in cell_point_count.items():
                # For sparse point clouds, even a single point should generate meaningful cost
                # Adjust confidence calculation to make even a single point have significant weight
                confidence = min(count / max(1, self.max_points_per_cell * 0.5), 1.0)
                
                # Scale between 70% and 100% of obstacle value for better visibility
                cost = int(self.obstacle_value * (0.7 + 0.3 * confidence))
                
                # Priority weighting for center area - if enabled with vehicle_center_weight
                if self.vehicle_center_weight > 0:
                    # Calculate distance from vehicle center
                    dist_from_center = math.sqrt((y - sensor_grid_y)**2 + (x - sensor_grid_x)**2)
                    
                    # If within priority radius, increase obstacle weight
                    if dist_from_center <= self.center_priority_radius_cells:
                        # Linear weight based on distance from center
                        center_factor = 1.0 - (dist_from_center / self.center_priority_radius_cells) * self.vehicle_center_weight
                        
                        # Increase obstacle cost to ensure visibility near vehicle
                        cost = min(self.obstacle_value, int(cost * (1.0 + center_factor)))
                
                self.lidar_costmap[y, x] = cost
            
            # Third pass: Inflation (simplify for performance)
            if self.obstacle_inflation > 0:
                # Create binary obstacle mask
                obstacle_mask = (self.lidar_costmap > self.unknown_value)
                
                # Optional: vectorized dilation for obstacle inflation
                if 1:  # Placeholder for conditional implementation
                    # Calculate inflation radius in grid cells
                    inflation_radius = max(2, int(self.obstacle_inflation / self.map_resolution))
                    
                    # Simple dilation approximation (much faster than per-cell calculation)
                    # This uses a square kernel instead of circle, but is much faster
                    from scipy import ndimage
                    dilated = ndimage.binary_dilation(
                        obstacle_mask, 
                        structure=np.ones((inflation_radius*2+1, inflation_radius*2+1)),
                        iterations=1
                    )
                    
                    # Fill all dilated areas with obstacle value unless they're free space
                    free_mask = (self.lidar_costmap == self.free_space_value)
                    inflation_mask = dilated & ~obstacle_mask & ~free_mask
                    self.lidar_costmap[inflation_mask] = self.obstacle_value * 0.7  # 70% obstacle value
            
            process_time = time.time() - process_start
            self.get_logger().debug(f'Costmap processing took {process_time:.3f} seconds')
            return True
    
    def bresenham_line(self, y0, x0, y1, x1):
        """Bresenham's line algorithm for raytracing from sensor to obstacle"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        # Pre-allocate points array for efficiency
        max_points = dx + dy + 1
        points = []
        points.append((y0, x0))
        
        # Exit early if start and end are the same
        if x0 == x1 and y0 == y1:
            return points
            
        while True:
            if x0 == x1 and y0 == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
                
            points.append((y0, x0))
            
            # Optimization: limit line length
            if len(points) > 100:  # Maximum line length in cells
                break
                
        return points
    
    def publish_costmap(self):
        """Publish the current costmap"""
        # Create occupancy grid message
        with self.costmap_lock:
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
            if self.log_count >= 20:  # Log every 4 seconds at 5Hz
                self.log_count = 0
                free_cells = np.count_nonzero(self.lidar_costmap == self.free_space_value)
                obstacle_cells = np.count_nonzero(self.lidar_costmap > self.unknown_value)
                unknown_cells = np.count_nonzero(self.lidar_costmap == self.unknown_value)
                total_cells = self.map_width * self.map_height
                
                self.get_logger().info(
                    f'LiDAR costmap stats: '
                    f'White (free): {100*free_cells/total_cells:.1f}%, '
                    f'Black (obstacles): {100*obstacle_cells/total_cells:.1f}%, '
                    f'Gray (unknown): {100*unknown_cells/total_cells:.1f}%, '
                    f'Map: {self.map_width_meters}x{self.map_height_meters}m'
                )
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