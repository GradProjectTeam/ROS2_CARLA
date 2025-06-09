#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
import tf2_ros
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
from threading import Lock
from geometry_msgs.msg import TransformStamped, Pose, Point

class LidarRealtimeMapper(Node):
    """
    Builds a real-time occupancy grid map from LiDAR data.
    
    This node:
    1. Subscribes to data from lidar_listener_clusters_2 node
    2. Maintains a dynamic occupancy grid map with frequent updates
    3. Focuses on recent data over historical consistency
    4. Updates the map based on new data and vehicle position
    5. Is designed for real-time navigation and obstacle avoidance
    """
    def __init__(self):
        super().__init__('lidar_realtime_mapper')
        
        # Performance counters
        self.start_time = time.time()
        self.frames_processed = 0
        self.total_points_processed = 0
        
        # Declare parameters with existence check
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.has_parameter('map_resolution'):
            self.declare_parameter('map_resolution', 0.1)       # Higher resolution for real-time map
        if not self.has_parameter('map_width'):
            self.declare_parameter('map_width', 400)            # Smaller area for real-time map
        if not self.has_parameter('map_height'):
            self.declare_parameter('map_height', 400)           # Smaller area for real-time map
        if not self.has_parameter('map_width_meters'):
            self.declare_parameter('map_width_meters', 40.0)    # 40m x 40m area around vehicle
        if not self.has_parameter('map_height_meters'):
            self.declare_parameter('map_height_meters', 40.0)   # 40m x 40m area around vehicle
        if not self.has_parameter('map_origin_x'):
            self.declare_parameter('map_origin_x', -20.0)       # Centered on vehicle
        if not self.has_parameter('map_origin_y'):
            self.declare_parameter('map_origin_y', -20.0)       # Centered on vehicle
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 10.0)        # Higher rate for real-time (10Hz)
        if not self.has_parameter('process_rate'):
            self.declare_parameter('process_rate', 20.0)        # Higher rate for processing (20Hz)
        if not self.has_parameter('ground_threshold'):
            self.declare_parameter('ground_threshold', 0.2)     # Lower to catch more obstacles
        if not self.has_parameter('max_points_to_process'):
            self.declare_parameter('max_points_to_process', 5000) # Process more points for detail
        if not self.has_parameter('min_height'):
            self.declare_parameter('min_height', -0.3)          # Capture lower obstacles
        if not self.has_parameter('max_height'):
            self.declare_parameter('max_height', 2.0)           # Lower ceiling to focus on obstacles
        if not self.has_parameter('raycast_skip'):
            self.declare_parameter('raycast_skip', 2)           # Process more points for detail
        if not self.has_parameter('decay_rate'):
            self.declare_parameter('decay_rate', 0.2)           # Rate at which old data decays from map
        if not self.has_parameter('center_on_vehicle'):
            self.declare_parameter('center_on_vehicle', True)   # Keep map centered on vehicle
        if not self.has_parameter('update_threshold'):
            self.declare_parameter('update_threshold', 0.01)    # Very low threshold for very quick updates
        if not self.has_parameter('use_cluster_data'):
            self.declare_parameter('use_cluster_data', True)    # Whether to use cluster data
        if not self.has_parameter('vehicle_frame_id'):
            self.declare_parameter('vehicle_frame_id', 'base_link') # Vehicle frame ID
        if not self.has_parameter('map_frame_id'):
            self.declare_parameter('map_frame_id', 'map')       # Map frame ID
        
        # New parameters for all-white map
        if not self.has_parameter('initialize_as_free'):
            self.declare_parameter('initialize_as_free', True)  # Initialize all cells as free space
        if not self.has_parameter('unknown_to_free'):
            self.declare_parameter('unknown_to_free', True)     # Treat unknown cells as free space
        
        # Bayesian update weights - more decisive for real-time
        if not self.has_parameter('hit_weight'):
            self.declare_parameter('hit_weight', 0.95)          # Very confident about hits
        if not self.has_parameter('miss_weight'):
            self.declare_parameter('miss_weight', 0.05)         # Confident about free space
        if not self.has_parameter('prior_weight'):
            self.declare_parameter('prior_weight', 0.5)         # Start with unknown
        if not self.has_parameter('count_threshold'):
            self.declare_parameter('count_threshold', 1.0)      # Lower threshold to mark cells faster
        
        # Add parameter for obstacle threshold
        if not self.has_parameter('obstacle_threshold'):
            self.declare_parameter('obstacle_threshold', 70)    # Threshold for marking cells as obstacles
        
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
        self.max_points_to_process = self.get_parameter('max_points_to_process').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.raycast_skip = self.get_parameter('raycast_skip').value
        self.decay_rate = self.get_parameter('decay_rate').value
        self.center_on_vehicle = self.get_parameter('center_on_vehicle').value
        self.update_threshold = self.get_parameter('update_threshold').value
        self.use_cluster_data = self.get_parameter('use_cluster_data').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # New parameters for all-white map
        self.initialize_as_free = self.get_parameter('initialize_as_free').value
        self.unknown_to_free = self.get_parameter('unknown_to_free').value
        
        # Bayesian update weights
        self.hit_weight = self.get_parameter('hit_weight').value
        self.miss_weight = self.get_parameter('miss_weight').value
        self.prior_weight = self.get_parameter('prior_weight').value
        self.count_threshold = self.get_parameter('count_threshold').value
        
        # Get obstacle threshold parameter
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        
        # Validate and adjust map dimensions
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
        
        # Initialize real-time map with all free (0) or unknown (50) based on parameters
        self.realtime_map = np.zeros((self.map_height, self.map_width), dtype=np.int8) if self.initialize_as_free else \
                          (np.ones((self.map_height, self.map_width), dtype=np.int8) * 50)
        
        # Hit count map - tracks confidence in each cell
        self.hit_count_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Time-based decay map - when cells were last updated
        self.last_update_time = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Data storage
        self.lidar_points = None
        self.lidar_markers = None
        self.lidar_cubes = None
        self.lidar_timestamp = None
        self.new_points_available = False
        self.new_markers_available = False
        self.new_cubes_available = False
        self.last_vehicle_position = None
        self.current_vehicle_position = None
        self.map_changed = False
        
        # Thread safety
        self.map_lock = Lock()
        self.data_lock = Lock()
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create reliable QoS profile for map publishing
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers - main realtime map and visualization
        self.map_publisher = self.create_publisher(
            OccupancyGrid, 
            '/realtime_map', 
            reliable_qos
        )
        
        # Visualization publisher
        self.vis_publisher = self.create_publisher(
            OccupancyGrid, 
            '/realtime_map_visualization',
            10
        )
        
        # Subscribers - Connect to lidar_listener_clusters_2 outputs
        self.create_subscription(
            PointCloud2,
            '/lidar/points',  # Main point cloud
            self.lidar_points_callback,
            10
        )
        
        self.create_subscription(
            MarkerArray,
            '/lidar/markers',  # Cluster centers 
            self.lidar_markers_callback,
            10
        )
        
        self.create_subscription(
            MarkerArray,
            '/lidar/cubes',  # 3D bounding boxes
            self.lidar_cubes_callback,
            10
        )
        
        # Timer for data processing
        self.create_timer(1.0 / self.process_rate, self.process_data)
        
        # Timer for map publishing
        self.create_timer(1.0 / self.publish_rate, self.publish_map)
        
        # Timer for map decay (to clear old obstacles)
        self.create_timer(1.0, self.apply_time_decay)
        
        # Timer for status printing
        self.create_timer(10.0, self.report_stats)
        
        self.get_logger().info(f"LiDAR Real-time Mapper initialized with {self.map_width}x{self.map_height} cells at {self.map_resolution}m resolution")
        self.get_logger().info(f"Map area: {self.map_width_meters}m x {self.map_height_meters}m")
        self.get_logger().info(f"Update rate: {self.process_rate}Hz, Publish rate: {self.publish_rate}Hz")
    
    def lidar_points_callback(self, msg):
        """Process incoming LiDAR point cloud"""
        with self.data_lock:
            self.lidar_points = msg
            self.lidar_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.new_points_available = True
    
    def lidar_markers_callback(self, msg):
        """Process incoming marker array (cluster centers)"""
        with self.data_lock:
            self.lidar_markers = msg
            self.new_markers_available = True
    
    def lidar_cubes_callback(self, msg):
        """Process incoming cube array (3D bounding boxes)"""
        with self.data_lock:
            self.lidar_cubes = msg
            self.new_cubes_available = True
    
    def get_vehicle_position(self):
        """Get the current vehicle position from TF"""
        try:
            # Get the transform from map to vehicle
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.vehicle_frame_id,
                rclpy.time.Time())
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            return (x, y)
        except Exception as e:
            self.get_logger().warning(f"Failed to get vehicle position: {str(e)}")
            return None
    
    def apply_time_decay(self):
        """Apply time-based decay to the map to forget old obstacles"""
        with self.map_lock:
            current_time = time.time()
            
            # Calculate the time difference for each cell
            time_diff = current_time - self.last_update_time
            
            # Apply decay to cells that haven't been updated recently
            # Only apply to cells that have some occupancy value (> 0)
            mask = (time_diff > 0) & (self.realtime_map > 0)
            
            if np.any(mask):
                # Get indices of cells to decay
                indices = np.where(mask)
                
                # Process each cell that needs decay
                for i in range(len(indices[0])):
                    idx_y, idx_x = indices[0][i], indices[1][i]
                    
                    # Calculate decay factor based on elapsed time
                    # More time = more decay
                    cell_time_diff = time_diff[idx_y, idx_x]
                    
                    # Skip cells that were just updated
                    if cell_time_diff < 0.1:  # 100ms threshold
                        continue
                    
                    # Calculate decay amount based on time difference and decay rate
                    # decay_amount ranges from 0 to 1.0 based on elapsed time
                    decay_amount = min(cell_time_diff * self.decay_rate, 1.0)
                    
                    # Apply decay to hit count
                    if self.hit_count_map[idx_y, idx_x] > 0:
                        # Gradually reduce hit count based on decay amount
                        self.hit_count_map[idx_y, idx_x] *= (1.0 - decay_amount)
                        
                        # If hit count falls below threshold, update the map value
                        if self.hit_count_map[idx_y, idx_x] < self.count_threshold:
                            # Calculate new occupancy value based on hit count
                            new_value = int((self.hit_count_map[idx_y, idx_x] / self.count_threshold) * 100)
                            
                            # Ensure value is within valid range
                            new_value = max(0, min(new_value, 100))
                            
                            # Update map if value changed
                            if self.realtime_map[idx_y, idx_x] != new_value:
                                self.realtime_map[idx_y, idx_x] = new_value
                                self.map_changed = True
                    
                    # If decay is strong enough, clear the cell completely
                    if decay_amount > 0.8:  # 80% decay threshold for complete clearing
                        self.realtime_map[idx_y, idx_x] = 0
                        self.hit_count_map[idx_y, idx_x] = 0
                        self.map_changed = True
    
    def process_data(self):
        """Process LiDAR data and update the real-time map"""
        # Get current vehicle position
        vehicle_position = self.get_vehicle_position()
        if vehicle_position is None:
            return
        
        # Update map origin if centering on vehicle
        if self.center_on_vehicle:
            x, y = vehicle_position
            self.map_origin_x = x - self.map_width_meters / 2
            self.map_origin_y = y - self.map_height_meters / 2
        
        # Store vehicle position
        self.current_vehicle_position = vehicle_position
        
        # Process points if available
        with self.data_lock:
            points_available = self.lidar_points is not None and self.new_points_available
            markers_available = self.lidar_markers is not None and self.new_markers_available
            cubes_available = self.lidar_cubes is not None and self.new_cubes_available
            
            # Reset flags
            self.new_points_available = False
            self.new_markers_available = False
            self.new_cubes_available = False
        
        # Update map based on available data
        if points_available:
            self.update_from_points(self.lidar_points, vehicle_position)
            self.frames_processed += 1
        
        if self.use_cluster_data and markers_available:
            self.update_from_markers(self.lidar_markers, vehicle_position)
        
        if self.use_cluster_data and cubes_available:
            self.update_from_cubes(self.lidar_cubes, vehicle_position)
        
        # Store last vehicle position
        self.last_vehicle_position = vehicle_position
    
    def update_from_points(self, point_cloud_msg, vehicle_position):
        """Update the map using raw point cloud data"""
        try:
            # Extract point cloud data
            pc_data = list(point_cloud2.read_points(
                point_cloud_msg, 
                field_names=("x", "y", "z"),
                skip_nans=True
            ))
            
            if not pc_data:
                return
            
            # Limit number of points to process
            if len(pc_data) > self.max_points_to_process:
                import random
                pc_data = random.sample(pc_data, self.max_points_to_process)
            
            # Cells to update
            cells_to_update = []
            
            # Vehicle position for raycasting
            if vehicle_position:
                vehicle_x, vehicle_y = vehicle_position
            else:
                # If no vehicle position, use (0,0)
                vehicle_x, vehicle_y = 0, 0
            
            # Convert vehicle position to map coordinates
            vehicle_map_x = int((vehicle_x - self.map_origin_x) / self.map_resolution)
            vehicle_map_y = int((vehicle_y - self.map_origin_y) / self.map_resolution)
            
            # Check if vehicle position is in map bounds
            if not (0 <= vehicle_map_x < self.map_width and 0 <= vehicle_map_y < self.map_height):
                self.get_logger().warn("Vehicle position outside map bounds")
                return
            
            # Group points by grid cell to handle multiple points in same cell
            grid_cell_points = {}
            
            # Process points and group by grid cell
            for point in pc_data:
                x, y, z = point
                
                # Filter out points by height
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Convert to map coordinates
                map_x = int((x - self.map_origin_x) / self.map_resolution)
                map_y = int((y - self.map_origin_y) / self.map_resolution)
                
                # Skip if outside map bounds
                if not (0 <= map_x < self.map_width and 0 <= map_y < self.map_height):
                    continue
                
                # Add point to grid cell
                cell_key = (map_y, map_x)
                if cell_key not in grid_cell_points:
                    grid_cell_points[cell_key] = []
                
                grid_cell_points[cell_key].append((x, y, z))
            
            # Process each occupied grid cell
            for cell_key, points in grid_cell_points.items():
                map_y, map_x = cell_key
                
                # Calculate average height and number of points in cell
                avg_z = sum(p[2] for p in points) / len(points)
                num_points = len(points)
                
                # Perform raycasting from vehicle to this cell
                ray_cells = self.bresenham_line(vehicle_map_y, vehicle_map_x, map_y, map_x)
                
                # Mark ray cells as free space (except the endpoint)
                for cell_y, cell_x in ray_cells[:-1]:
                    cells_to_update.append((cell_y, cell_x, False))  # False = free space
                
                # Mark endpoint as occupied with weight based on number of points
                # More points in a cell = more likely to be an obstacle
                confidence = min(1.0, num_points / 5.0)  # Scale confidence by number of points (max 1.0)
                
                # Cells with more points get higher confidence
                for _ in range(max(1, int(confidence * 3))):
                    cells_to_update.append((map_y, map_x, True))  # True = occupied
            
            # Apply updates to the map
            self.update_cells(cells_to_update)
            
            # Update statistics
            self.total_points_processed += len(pc_data)
            
        except Exception as e:
            self.get_logger().error(f"Error updating map from points: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def update_from_markers(self, markers_msg, vehicle_position):
        """Update the map using cluster center markers"""
        if not markers_msg.markers:
            return
        
        try:
            cells_to_update = []
            
            # Vehicle position for raycasting
            if vehicle_position:
                vehicle_x, vehicle_y = vehicle_position
            else:
                # If no vehicle position, use (0,0)
                vehicle_x, vehicle_y = 0, 0
            
            # Process each marker (cluster center)
            for marker in markers_msg.markers:
                x = marker.pose.position.x
                y = marker.pose.position.y
                z = marker.pose.position.z
                
                # Filter by height
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Convert to map coordinates
                map_x = int((x - self.map_origin_x) / self.map_resolution)
                map_y = int((y - self.map_origin_y) / self.map_resolution)
                
                # Check if point is within map bounds
                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    # Convert vehicle position to map coordinates
                    vehicle_map_x = int((vehicle_x - self.map_origin_x) / self.map_resolution)
                    vehicle_map_y = int((vehicle_y - self.map_origin_y) / self.map_resolution)
                    
                    # Perform raycasting from vehicle to marker
                    if 0 <= vehicle_map_x < self.map_width and 0 <= vehicle_map_y < self.map_height:
                        ray_cells = self.bresenham_line(vehicle_map_y, vehicle_map_x, map_y, map_x)
                        
                        # Mark ray cells as free space (except the endpoint)
                        for cell_y, cell_x in ray_cells[:-1]:
                            cells_to_update.append((cell_y, cell_x, False))
                        
                        # Mark endpoint as occupied
                        cells_to_update.append((map_y, map_x, True))
            
            # Apply updates to the map
            self.update_cells(cells_to_update)
            
        except Exception as e:
            self.get_logger().error(f"Error updating map from markers: {str(e)}")
    
    def update_from_cubes(self, cubes_msg, vehicle_position):
        """Update the map using 3D bounding box cubes"""
        if not cubes_msg.markers:
            return
        
        try:
            cells_to_update = []
            
            # Process each cube (bounding box)
            for cube in cubes_msg.markers:
                # Get cube position and scale
                x = cube.pose.position.x
                y = cube.pose.position.y
                z = cube.pose.position.z
                scale_x = cube.scale.x
                scale_y = cube.scale.y
                
                # Skip if too high or too low
                if z < self.min_height or z > self.max_height:
                    continue
                
                # Calculate box corners in map coordinates
                half_x = scale_x / 2.0
                half_y = scale_y / 2.0
                
                # Box corners
                corners = [
                    (x - half_x, y - half_y),
                    (x - half_x, y + half_y),
                    (x + half_x, y - half_y),
                    (x + half_x, y + half_y)
                ]
                
                # Convert corners to map coordinates
                map_corners = []
                for corner_x, corner_y in corners:
                    map_x = int((corner_x - self.map_origin_x) / self.map_resolution)
                    map_y = int((corner_y - self.map_origin_y) / self.map_resolution)
                    
                    # Check bounds
                    if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                        map_corners.append((map_y, map_x))
                
                # Fill box area in map
                if map_corners:
                    min_y = max(0, min(c[0] for c in map_corners))
                    max_y = min(self.map_height-1, max(c[0] for c in map_corners))
                    min_x = max(0, min(c[1] for c in map_corners))
                    max_x = min(self.map_width-1, max(c[1] for c in map_corners))
                    
                    # Mark all cells in box as occupied
                    for y in range(min_y, max_y + 1):
                        for x in range(min_x, max_x + 1):
                            cells_to_update.append((y, x, True))
            
            # Apply updates to the map
            self.update_cells(cells_to_update)
            
        except Exception as e:
            self.get_logger().error(f"Error updating map from cubes: {str(e)}")
    
    def update_cells(self, cells_to_update):
        """Update the map cells based on LiDAR data"""
        with self.map_lock:
            current_time = time.time()
            
            for cell in cells_to_update:
                y, x, is_occupied = cell
                
                # Skip out of bounds cells
                if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
                    continue
                
                # Update last update time
                self.last_update_time[y, x] = current_time
                
                # Get current values
                current_value = self.realtime_map[y, x]
                current_count = self.hit_count_map[y, x]
                
                if is_occupied:
                    # Cell is occupied - increase the hit count
                    new_count = current_count + 1.0
                    
                    # Mark as obstacle if count exceeds threshold
                    if new_count >= self.count_threshold:
                        new_value = 100  # Maximum occupancy
                    else:
                        # Scale the occupancy value based on hit count
                        new_value = min(int((new_count / self.count_threshold) * 100), 100)
                else:
                    # Cell is free - decrease the hit count
                    new_count = max(current_count - 0.3, 0.0)
                    
                    # If count is below threshold, mark as free
                    if new_count < self.count_threshold * 0.5:
                        new_value = 0
                    else:
                        # Scale down the occupancy value
                        new_value = max(int((new_count / self.count_threshold) * 100), 0)
                
                # Update maps only if value changed
                if current_value != new_value:
                    self.realtime_map[y, x] = int(new_value)
                    self.hit_count_map[y, x] = new_count
                    self.map_changed = True
    
    def bresenham_line(self, y0, x0, y1, x1):
        """Bresenham's line algorithm for raycasting"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            cells.append((y0, x0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return cells
    
    def publish_map(self):
        """Publish the current real-time map"""
        with self.map_lock:
            if not self.map_changed:
                return
            
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            
            # Set header
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = self.map_frame_id
            
            # Set metadata
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width
            map_msg.info.height = self.map_height
            
            # Set map origin
            map_msg.info.origin.position.x = self.map_origin_x
            map_msg.info.origin.position.y = self.map_origin_y
            map_msg.info.origin.orientation.w = 1.0
            
            # Create a copy of the map data
            map_data = self.realtime_map.copy()
            
            # Apply obstacle threshold to create binary representation
            # Values >= obstacle_threshold become 100 (obstacle)
            # Values < obstacle_threshold become 0 (free space)
            map_data[map_data >= self.obstacle_threshold] = 100
            map_data[map_data < self.obstacle_threshold] = 0
            
            # Convert unknown cells (-1 or 50) to free space (0) if unknown_to_free is True
            if self.unknown_to_free:
                map_data[map_data == 50] = 0
                map_data[map_data == -1] = 0
            
            # Set map data
            map_msg.data = map_data.flatten().tolist()
            
            # Publish map
            self.map_publisher.publish(map_msg)
            
            # Also publish to visualization topic (same data)
            self.vis_publisher.publish(map_msg)
            
            self.map_changed = False
    
    def report_stats(self):
        """Report node performance statistics"""
        elapsed = time.time() - self.start_time
        if elapsed > 0 and self.frames_processed > 0:
            fps = self.frames_processed / elapsed
            pps = self.total_points_processed / elapsed
            
            # Calculate map statistics
            with self.map_lock:
                # Count cells by type
                free_cells = np.sum(self.realtime_map == 0)
                obstacle_cells = np.sum(self.realtime_map >= self.obstacle_threshold)
                unknown_cells = np.sum((self.realtime_map > 0) & (self.realtime_map < self.obstacle_threshold))
                
                # Calculate percentages
                total_cells = self.map_width * self.map_height
                free_percent = (free_cells / total_cells) * 100
                obstacle_percent = (obstacle_cells / total_cells) * 100
                unknown_percent = (unknown_cells / total_cells) * 100
                
                # Calculate average hit count for occupied cells
                occupied_mask = self.realtime_map > 0
                avg_hit_count = 0.0
                if np.any(occupied_mask):
                    avg_hit_count = np.mean(self.hit_count_map[occupied_mask])
            
            # Log performance statistics
            self.get_logger().info(f"Performance: {fps:.2f} fps, {pps:.2f} points/sec")
            
            # Log map statistics
            self.get_logger().info(f"Map stats: {free_cells} free ({free_percent:.1f}%), "
                                 f"{obstacle_cells} obstacles ({obstacle_percent:.1f}%), "
                                 f"{unknown_cells} uncertain ({unknown_percent:.1f}%)")
            
            # Log hit count statistics
            self.get_logger().info(f"Avg hit count: {avg_hit_count:.2f}, "
                                 f"Threshold: {self.count_threshold}, "
                                 f"Obstacle threshold: {self.obstacle_threshold}")
            
            # Log memory usage
            import psutil
            process = psutil.Process()
            memory_info = process.memory_info()
            memory_mb = memory_info.rss / (1024 * 1024)
            self.get_logger().info(f"Memory usage: {memory_mb:.1f} MB")
            
            # Reset counters for next report
            self.start_time = time.time()
            self.frames_processed = 0
            self.total_points_processed = 0


def main(args=None):
    rclpy.init(args=args)
    
    node = LidarRealtimeMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 