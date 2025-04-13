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
import os
from datetime import datetime
from geometry_msgs.msg import TransformStamped, Pose, Point
from std_srvs.srv import Trigger
import pickle

class LidarPermanentMapper(Node):
    """
    Builds a permanent occupancy grid map from LiDAR data.
    
    This node:
    1. Subscribes to data from lidar_listener_clusters_2 node
    2. Builds and maintains a persistent occupancy grid map
    3. Provides services to save and load maps
    4. Updates the map based on new data and vehicle position
    """
    def __init__(self):
        super().__init__('lidar_permanent_mapper')
        
        # Performance counters
        self.start_time = time.time()
        self.frames_processed = 0
        self.total_points_processed = 0
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.2)       # Default 20cm resolution for global map
        self.declare_parameter('map_width', 1000)           # 1000 cells = 200m at 0.2m resolution
        self.declare_parameter('map_height', 1000)          # 1000 cells = 200m at 0.2m resolution
        self.declare_parameter('map_width_meters', 200.0)   # meters - larger area for global map
        self.declare_parameter('map_height_meters', 200.0)  # meters - larger area for global map
        self.declare_parameter('map_origin_x', -100.0)      # meters
        self.declare_parameter('map_origin_y', -100.0)      # meters
        self.declare_parameter('publish_rate', 1.0)         # Lower rate for permanent map (1Hz)
        self.declare_parameter('process_rate', 2.0)         # Lower rate for map building (2Hz)
        self.declare_parameter('ground_threshold', 0.15)    # Same as costmap creator
        self.declare_parameter('max_points_to_process', 2000) # Limit points per update for performance
        self.declare_parameter('min_height', -3.0)          # Same as costmap creator
        self.declare_parameter('max_height', 4.0)           # Same as costmap creator
        self.declare_parameter('raycast_skip', 5)           # Skip more points for permanent map
        self.declare_parameter('map_save_dir', '/home/mostafa/GP/ROS2/maps') # Directory to save maps
        self.declare_parameter('enable_auto_save', True)    # Auto-save map periodically
        self.declare_parameter('auto_save_interval', 60.0)  # Auto-save interval in seconds
        self.declare_parameter('update_threshold', 0.7)     # Threshold for updating cells (0.0-1.0)
        self.declare_parameter('use_cluster_data', True)    # Whether to use cluster data from lidar_listener_clusters_2
        
        # Cell count parameters and Bayesian update weights
        self.declare_parameter('hit_weight', 0.9)           # Weight for obstacle hits (0.0-1.0)
        self.declare_parameter('miss_weight', 0.3)          # Weight for empty space (0.0-1.0)
        self.declare_parameter('prior_weight', 0.5)         # Prior probability (0.0-1.0)
        self.declare_parameter('count_threshold', 10.0)     # Count threshold for "permanent" status
        
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
        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.enable_auto_save = self.get_parameter('enable_auto_save').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value
        self.update_threshold = self.get_parameter('update_threshold').value
        self.use_cluster_data = self.get_parameter('use_cluster_data').value
        
        # Bayesian update weights
        self.hit_weight = self.get_parameter('hit_weight').value
        self.miss_weight = self.get_parameter('miss_weight').value
        self.prior_weight = self.get_parameter('prior_weight').value
        self.count_threshold = self.get_parameter('count_threshold').value
        
        # Ensure map save directory exists
        if not os.path.exists(self.map_save_dir):
            try:
                os.makedirs(self.map_save_dir)
                self.get_logger().info(f"Created map save directory: {self.map_save_dir}")
            except Exception as e:
                self.get_logger().error(f"Failed to create map directory: {e}")
        
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
                
            # Calculate origin to center the map around (0,0)
            self.map_origin_x = -self.map_width_meters/2
            self.map_origin_y = -self.map_height_meters/2
        
        # Initialize permanent map - uses probability values (0-100)
        # 0 = definitely free, 100 = definitely occupied, 50 = unknown
        self.permanent_map = np.ones((self.map_height, self.map_width), dtype=np.int8) * 50
        
        # Hit count map - tracks confidence in each cell
        self.hit_count_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Data storage
        self.lidar_points = None
        self.lidar_markers = None
        self.lidar_cubes = None
        self.lidar_hulls = None
        self.lidar_timestamp = None
        self.new_points_available = False
        self.new_markers_available = False
        self.new_cubes_available = False
        self.new_hulls_available = False
        self.last_save_time = time.time()
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
        
        # Publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid, 
            '/permanent_map', 
            reliable_qos
        )
        
        # Visualization publisher
        self.vis_publisher = self.create_publisher(
            OccupancyGrid, 
            '/permanent_map_visualization',
            10
        )
        
        # Subscribers - Connect to lidar_listener_clusters_2 outputs
        self.create_subscription(
            PointCloud2,
            '/lidar/points',  # Main point cloud from lidar_listener_clusters_2
            self.lidar_points_callback,
            10
        )
        
        self.create_subscription(
            MarkerArray,
            '/lidar/markers',  # Cluster centers from lidar_listener_clusters_2
            self.lidar_markers_callback,
            10
        )
        
        self.create_subscription(
            MarkerArray,
            '/lidar/cubes',  # 3D bounding boxes from lidar_listener_clusters_2
            self.lidar_cubes_callback,
            10
        )
        
        self.create_subscription(
            MarkerArray,
            '/lidar/hulls',  # Convex hulls from lidar_listener_clusters_2
            self.lidar_hulls_callback,
            10
        )
        
        # Services
        self.save_service = self.create_service(
            Trigger, 
            'save_permanent_map', 
            self.save_map_callback
        )
        
        self.load_service = self.create_service(
            Trigger, 
            'load_permanent_map', 
            self.load_map_callback
        )
        
        # Create timers
        self.process_timer = self.create_timer(1.0/self.process_rate, self.process_data)
        self.publish_timer = self.create_timer(1.0/self.publish_rate, self.publish_map)
        self.stats_timer = self.create_timer(10.0, self.report_stats)
        
        # Auto-save timer if enabled
        if self.enable_auto_save:
            self.auto_save_timer = self.create_timer(self.auto_save_interval, self.auto_save_map)
        
        self.get_logger().info(f'Permanent Mapper initialized with {self.map_width}x{self.map_height} grid at {self.map_resolution}m resolution')
        self.get_logger().info(f'Map covers {self.map_width_meters}x{self.map_height_meters}m area')
        self.get_logger().info(f'Auto-save is {"enabled" if self.enable_auto_save else "disabled"}')
        self.get_logger().info(f'Listening to lidar_listener_clusters_2 data (use_cluster_data: {self.use_cluster_data})')
    
    def lidar_points_callback(self, msg):
        """Process incoming LiDAR point cloud data"""
        with self.data_lock:
            self.lidar_points = msg
            self.lidar_timestamp = self.get_clock().now()
            self.new_points_available = True
            self.get_logger().debug(f'Received point cloud with {len(list(point_cloud2.read_points(msg)))} points')
    
    def lidar_markers_callback(self, msg):
        """Process incoming cluster markers data"""
        with self.data_lock:
            self.lidar_markers = msg
            self.new_markers_available = True
            self.get_logger().debug(f'Received {len(msg.markers)} cluster markers')
    
    def lidar_cubes_callback(self, msg):
        """Process incoming cube markers data"""
        with self.data_lock:
            self.lidar_cubes = msg
            self.new_cubes_available = True
            self.get_logger().debug(f'Received {len(msg.markers)} cube markers')
    
    def lidar_hulls_callback(self, msg):
        """Process incoming hull markers data"""
        with self.data_lock:
            self.lidar_hulls = msg
            self.new_hulls_available = True
            self.get_logger().debug(f'Received {len(msg.markers)} hull markers')
    
    def get_vehicle_position(self):
        """Get current vehicle position from TF"""
        try:
            # Look up transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            
            # Extract position
            position = (transform.transform.translation.x, 
                       transform.transform.translation.y)
            
            return position
        except Exception as e:
            self.get_logger().warning(f'Failed to get vehicle position: {e}')
            return None
    
    def process_data(self):
        """Process LiDAR data and update permanent map"""
        # Skip if no new data
        if not self.new_points_available and not (self.use_cluster_data and self.new_markers_available):
            return
        
        # Get vehicle position
        vehicle_position = self.get_vehicle_position()
        if vehicle_position is None:
            return
        
        # Store vehicle positions for movement detection
        self.last_vehicle_position = self.current_vehicle_position
        self.current_vehicle_position = vehicle_position
        
        try:
            # Process based on available data sources
            if self.use_cluster_data and self.new_markers_available:
                # Process cluster data from markers
                with self.data_lock:
                    marker_data = self.lidar_markers
                    self.new_markers_available = False
                
                if marker_data is not None:
                    with self.map_lock:
                        self.update_from_markers(marker_data, vehicle_position)
                    
                    # Track statistics
                    self.frames_processed += 1
                    self.total_points_processed += len(marker_data.markers)
            
            # Always process point cloud data if available
            if self.new_points_available:
                # Lock to safely access data
                with self.data_lock:
                    points_msg = self.lidar_points
                    self.new_points_available = False
                
                if points_msg is not None:
                    # Extract point cloud data
                    points_list = list(point_cloud2.read_points(
                        points_msg, 
                        field_names=["x", "y", "z"],
                        skip_nans=True
                    ))
                    
                    # Limit points to process
                    if len(points_list) > self.max_points_to_process:
                        # Randomly sample points
                        indices = np.random.choice(
                            len(points_list), 
                            self.max_points_to_process, 
                            replace=False
                        )
                        points_list = [points_list[i] for i in indices]
                    
                    # Process points and update map
                    with self.map_lock:
                        self.update_permanent_map(points_list, vehicle_position)
                    
                    # Update stats
                    self.frames_processed += 1
                    self.total_points_processed += len(points_list)
            
            # Process 3D cube markers if available
            if self.use_cluster_data and self.new_cubes_available:
                with self.data_lock:
                    cube_data = self.lidar_cubes
                    self.new_cubes_available = False
                
                if cube_data is not None:
                    with self.map_lock:
                        self.update_from_cubes(cube_data, vehicle_position)
            
        except Exception as e:
            self.get_logger().error(f'Error processing data: {e}')
    
    def update_from_markers(self, markers, vehicle_position):
        """Update map using marker data from lidar_listener_clusters_2"""
        if not markers.markers:
            return
            
        # Extract vehicle position
        vehicle_x, vehicle_y = vehicle_position
        
        # Convert to grid coordinates
        vehicle_grid_x = int((vehicle_x - self.map_origin_x) / self.map_resolution)
        vehicle_grid_y = int((vehicle_y - self.map_origin_y) / self.map_resolution)
        
        # Skip if vehicle is outside map
        if not (0 <= vehicle_grid_x < self.map_width and 0 <= vehicle_grid_y < self.map_height):
            self.get_logger().warning(f'Vehicle position ({vehicle_x}, {vehicle_y}) is outside map boundaries')
            return
            
        # Process each marker (cluster center)
        cells_to_update = set()
        
        for marker in markers.markers:
            # Skip markers that aren't spheres or have been deleted
            if marker.type != Marker.SPHERE or marker.action == Marker.DELETE:
                continue
                
            # Extract coordinates
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            
            # Filter out markers outside height range
            if z < self.min_height or z > self.max_height:
                continue
                
            # Convert to grid coordinates
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)
            
            # Skip if outside map
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                continue
                
            # Perform ray tracing from vehicle to marker
            ray_cells = self.bresenham_line(vehicle_grid_y, vehicle_grid_x, grid_y, grid_x)
            
            # Mark all cells along ray as free except the endpoint
            for j, (ray_y, ray_x) in enumerate(ray_cells):
                # Skip if outside map
                if not (0 <= ray_x < self.map_width and 0 <= ray_y < self.map_height):
                    continue
                    
                if j < len(ray_cells) - 1:  # Free space
                    cells_to_update.add((ray_y, ray_x, False))  # False = free
                else:  # Obstacle at endpoint
                    cells_to_update.add((ray_y, ray_x, True))   # True = occupied
        
        # Apply Bayesian updates to all cells in batch
        self.update_cells(cells_to_update)
    
    def update_from_cubes(self, cubes, vehicle_position):
        """Update map using 3D cube data from lidar_listener_clusters_2"""
        if not cubes.markers:
            return
            
        # Extract vehicle position
        vehicle_x, vehicle_y = vehicle_position
        
        # Convert to grid coordinates
        vehicle_grid_x = int((vehicle_x - self.map_origin_x) / self.map_resolution)
        vehicle_grid_y = int((vehicle_y - self.map_origin_y) / self.map_resolution)
        
        # Skip if vehicle is outside map
        if not (0 <= vehicle_grid_x < self.map_width and 0 <= vehicle_grid_y < self.map_height):
            return
            
        # Process each cube marker
        cells_to_update = set()
        
        for marker in cubes.markers:
            # Skip deleted markers
            if marker.action == Marker.DELETE:
                continue
                
            # Extract cube corners (using scale and position)
            x = marker.pose.position.x
            y = marker.pose.position.y
            
            # Get half-dimensions from scale
            half_width = marker.scale.x / 2.0
            half_length = marker.scale.y / 2.0
            
            # Create bounding box corners
            corners = [
                (x - half_width, y - half_length),
                (x - half_width, y + half_length),
                (x + half_width, y - half_length),
                (x + half_width, y + half_length)
            ]
            
            # Convert corners to grid coordinates
            grid_corners = []
            for corner_x, corner_y in corners:
                grid_x = int((corner_x - self.map_origin_x) / self.map_resolution)
                grid_y = int((corner_y - self.map_origin_y) / self.map_resolution)
                
                # Skip if outside map
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    grid_corners.append((grid_y, grid_x))
            
            # Skip if all corners are outside the map
            if not grid_corners:
                continue
                
            # Mark all cells inside the cube as occupied
            min_y = max(0, min(y for y, _ in grid_corners))
            max_y = min(self.map_height - 1, max(y for y, _ in grid_corners))
            min_x = max(0, min(x for _, x in grid_corners))
            max_x = min(self.map_width - 1, max(x for _, x in grid_corners))
            
            for y in range(min_y, max_y + 1):
                for x in range(min_x, max_x + 1):
                    cells_to_update.add((y, x, True))  # True = occupied
                    
            # Ray trace from vehicle to cube center to mark free space
            grid_center_y = int((y - self.map_origin_y) / self.map_resolution)
            grid_center_x = int((x - self.map_origin_x) / self.map_resolution)
            
            if 0 <= grid_center_x < self.map_width and 0 <= grid_center_y < self.map_height:
                ray_cells = self.bresenham_line(
                    vehicle_grid_y, vehicle_grid_x, 
                    grid_center_y, grid_center_x
                )
                
                # Mark cells along ray as free (except the last one)
                for j, (ray_y, ray_x) in enumerate(ray_cells):
                    if not (0 <= ray_x < self.map_width and 0 <= ray_y < self.map_height):
                        continue
                        
                    if j < len(ray_cells) - 1:  # Free space
                        cells_to_update.add((ray_y, ray_x, False))
        
        # Apply Bayesian updates to all cells in batch
        self.update_cells(cells_to_update)
    
    def update_permanent_map(self, points, vehicle_position):
        """Update the permanent map with new point data"""
        # Skip if no vehicle position
        if vehicle_position is None:
            return
        
        # Extract vehicle position
        vehicle_x, vehicle_y = vehicle_position
        
        # Convert to grid coordinates
        vehicle_grid_x = int((vehicle_x - self.map_origin_x) / self.map_resolution)
        vehicle_grid_y = int((vehicle_y - self.map_origin_y) / self.map_resolution)
        
        # Skip if vehicle is outside map
        if not (0 <= vehicle_grid_x < self.map_width and 0 <= vehicle_grid_y < self.map_height):
            self.get_logger().warning(f'Vehicle position ({vehicle_x}, {vehicle_y}) is outside map boundaries')
            return
        
        # Initialize cells to be updated
        cells_to_update = set()
        
        # Process each point with stride for efficiency
        for i, point in enumerate(points):
            # Skip points based on raycast_skip
            if i % self.raycast_skip != 0:
                continue
                
            # Extract coordinates
            x, y, z = point
            
            # Filter out points outside height range or too close to ground
            if z < self.min_height or z > self.max_height or abs(z) < self.ground_threshold:
                continue
            
            # Convert to grid coordinates
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)
            
            # Skip if outside map
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                continue
            
            # Perform ray tracing from vehicle to point
            ray_cells = self.bresenham_line(vehicle_grid_y, vehicle_grid_x, grid_y, grid_x)
            
            # Mark all cells along ray as free except the endpoint
            for j, (ray_y, ray_x) in enumerate(ray_cells):
                # Skip if outside map
                if not (0 <= ray_x < self.map_width and 0 <= ray_y < self.map_height):
                    continue
                    
                if j < len(ray_cells) - 1:  # Free space
                    cells_to_update.add((ray_y, ray_x, False))  # False = free
                else:  # Obstacle at endpoint
                    cells_to_update.add((ray_y, ray_x, True))   # True = occupied
        
        # Apply Bayesian updates to all cells in batch
        self.update_cells(cells_to_update)
    
    def update_cells(self, cells_to_update):
        """Apply Bayesian updates to a batch of cells"""
        # Apply Bayesian updates to all cells in batch
        for y, x, is_occupied in cells_to_update:
            # Get current probability (0-100)
            current_prob = self.permanent_map[y, x]
            
            # Convert to log-odds representation
            if current_prob == 0:
                current_log_odds = -10  # Avoid log(0)
            elif current_prob == 100:
                current_log_odds = 10   # Avoid log(inf)
            else:
                current_log_odds = np.log(current_prob / (100 - current_prob))
            
            # Calculate update
            if is_occupied:
                # Obstacle hit - increase probability
                update = np.log(self.hit_weight / (1 - self.hit_weight))
                self.hit_count_map[y, x] += 1
            else:
                # Free space - decrease probability
                update = np.log(self.miss_weight / (1 - self.miss_weight))
                
            # Apply update
            new_log_odds = current_log_odds + update
            
            # Convert back to probability (0-100)
            new_prob = int(100 * (1 / (1 + np.exp(-new_log_odds))))
            
            # Only update if change is significant or confidence is high
            if (abs(new_prob - current_prob) > (100 * self.update_threshold) or 
                self.hit_count_map[y, x] > self.count_threshold):
                self.permanent_map[y, x] = new_prob
                self.map_changed = True
        
        # If map was updated, log it
        if self.map_changed:
            self.get_logger().debug('Permanent map updated')
    
    def bresenham_line(self, y0, x0, y1, x1):
        """Bresenham's line algorithm for ray tracing"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((y0, x0))
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
    
    def publish_map(self):
        """Publish the current permanent map"""
        with self.map_lock:
            # Create OccupancyGrid message
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            # Set map metadata
            msg.info.resolution = self.map_resolution
            msg.info.width = self.map_width
            msg.info.height = self.map_height
            msg.info.origin.position.x = self.map_origin_x
            msg.info.origin.position.y = self.map_origin_y
            
            # Convert map to ROS format (-1 to 100)
            # This is a bit different than the internal representation
            # Unknown = -1, Free = 0, Occupied = 100
            ros_map = np.zeros(self.permanent_map.shape, dtype=np.int8)
            
            # Apply thresholds for clear visualization
            ros_map[self.permanent_map < 30] = 0       # Definitely free
            ros_map[self.permanent_map > 70] = 100     # Definitely occupied
            ros_map[(self.permanent_map >= 30) & (self.permanent_map <= 70)] = -1  # Unknown
            
            # Flatten in row-major order (height, width) -> (height * width)
            msg.data = ros_map.flatten().tolist()
            
            # Publish
            self.map_publisher.publish(msg)
            
            # Also publish visualization map which includes grayscale for partially known areas
            vis_msg = OccupancyGrid()
            vis_msg.header = msg.header
            vis_msg.info = msg.info
            vis_msg.data = self.permanent_map.flatten().tolist()
            self.vis_publisher.publish(vis_msg)
    
    def save_map_callback(self, request, response):
        """Service callback to save the current map"""
        try:
            filename = self.save_map()
            response.success = True
            response.message = f"Map saved to {filename}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to save map: {str(e)}"
        return response
    
    def load_map_callback(self, request, response):
        """Service callback to load the most recent map"""
        try:
            # Find most recent map file
            files = [f for f in os.listdir(self.map_save_dir) if f.endswith('.map')]
            if not files:
                response.success = False
                response.message = "No map files found"
                return response
                
            files.sort(reverse=True)  # Sort by name (timestamp) descending
            latest_map = os.path.join(self.map_save_dir, files[0])
            
            self.load_map(latest_map)
            response.success = True
            response.message = f"Loaded map from {latest_map}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to load map: {str(e)}"
        return response
    
    def save_map(self):
        """Save the current map to a file"""
        with self.map_lock:
            # Create filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.map_save_dir, f"permanent_map_{timestamp}.map")
            
            # Save map data
            data = {
                'permanent_map': self.permanent_map,
                'hit_count_map': self.hit_count_map,
                'resolution': self.map_resolution,
                'width': self.map_width,
                'height': self.map_height,
                'origin_x': self.map_origin_x,
                'origin_y': self.map_origin_y,
                'timestamp': timestamp
            }
            
            with open(filename, 'wb') as f:
                pickle.dump(data, f)
            
            # Also save as PNG for easy viewing
            import matplotlib.pyplot as plt
            plt.figure(figsize=(12, 12))
            plt.imshow(self.permanent_map, cmap='gray', origin='lower')
            plt.colorbar(label='Occupancy (0-100)')
            plt.title(f'Permanent Map {timestamp}')
            plt.savefig(os.path.join(self.map_save_dir, f"permanent_map_{timestamp}.png"))
            plt.close()
            
            self.get_logger().info(f'Map saved to {filename}')
            self.last_save_time = time.time()
            self.map_changed = False
            
            return filename
    
    def load_map(self, filename):
        """Load a map from a file"""
        with self.map_lock:
            with open(filename, 'rb') as f:
                data = pickle.load(f)
            
            # Check if map dimensions match
            if (data['width'] != self.map_width or data['height'] != self.map_height or
                data['resolution'] != self.map_resolution):
                self.get_logger().warn(f'Map dimensions or resolution mismatch. Resizing map.')
                # Handle resizing if needed
            
            # Load map data
            self.permanent_map = data['permanent_map']
            self.hit_count_map = data['hit_count_map']
            self.map_origin_x = data['origin_x']
            self.map_origin_y = data['origin_y']
            
            self.get_logger().info(f'Loaded map from {filename}')
    
    def auto_save_map(self):
        """Automatically save the map if it has changed"""
        if self.map_changed:
            try:
                self.save_map()
                self.get_logger().info('Auto-saved map')
            except Exception as e:
                self.get_logger().error(f'Auto-save failed: {e}')
    
    def report_stats(self):
        """Report performance statistics"""
        elapsed = time.time() - self.start_time
        if elapsed > 0 and self.frames_processed > 0:
            fps = self.frames_processed / elapsed
            pps = self.total_points_processed / elapsed
            self.get_logger().info(
                f'Performance: {fps:.2f} frames/sec, {pps:.2f} points/sec, '
                f'processed {self.frames_processed} frames, {self.total_points_processed} points'
            )
            
            # Check map coverage
            unknown = np.sum(self.permanent_map == 50)
            free = np.sum(self.permanent_map < 30)
            occupied = np.sum(self.permanent_map > 70)
            total = self.map_width * self.map_height
            
            self.get_logger().info(
                f'Map coverage: {(total-unknown)/total*100:.1f}% mapped, '
                f'{free/total*100:.1f}% free, {occupied/total*100:.1f}% occupied'
            )
        
        # Reset counters for current statistics
        self.start_time = time.time()
        self.frames_processed = 0
        self.total_points_processed = 0

def main(args=None):
    rclpy.init(args=args)
    node = LidarPermanentMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Final map save on shutdown
        if node.map_changed:
            try:
                node.save_map()
                node.get_logger().info('Saved map before shutdown')
            except Exception as e:
                node.get_logger().error(f'Failed to save map on shutdown: {e}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
