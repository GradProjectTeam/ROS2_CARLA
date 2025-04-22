#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
import socket
import struct
import numpy as np
import time
import math
from threading import Lock
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Add this function at the module level (outside any class)
def process_cell_batch(cells_batch, sensor_y, sensor_x, width, height, bresenham_line_func):
    """Process a batch of cells for ray tracing in parallel.
    
    This function must be at module level (not inside a class method) for multiprocessing to work.
    """
    local_free_cells = set()
    for cell in cells_batch:
        grid_y, grid_x = cell
        # Call the bresenham_line function passed as argument
        line_points = bresenham_line_func(sensor_y, sensor_x, grid_y, grid_x)
        # Mark all points along the line except the last one as free space
        for i in range(len(line_points) - 1):
            y, x = line_points[i]
            if 0 <= x < width and 0 <= y < height:
                local_free_cells.add((y, x))
    return local_free_cells

class LidarRawCostmapCreator(Node):
    """
    A combined node that:
    1. Receives raw LIDAR data over TCP
    2. Processes the data directly (no intermediate PointCloud2 publication)
    3. Creates a costmap directly from the raw LIDAR points
    4. Optimized for wall detection and obstacle avoidance
    """
    def __init__(self):
        """Initialize the node with optimized parameters for vehicle and road object detection"""
        super().__init__('lidar_raw_costmap_creator')
        
        # VEHICLE DETECTION: Optimized parameters for detecting cars and road objects
        # -----------------------------------------------------------------------
        # Parameters chosen for reliable detection of vehicles and other road objects
        self.declare_parameter('publish_rate', 20.0)                # Hz - high update rate for responsive detection
        self.declare_parameter('point_lifetime', 0.5)               # seconds - slightly increased for smoother tracking
        self.declare_parameter('max_points_per_update', 8000)       # points - increased for better vehicle definition
        self.declare_parameter('max_points_per_batch', 500)         # larger batches for better object clustering
        self.declare_parameter('use_multiprocessing', True)         # enable for better performance on multi-core systems
        self.declare_parameter('parallel_ray_tracing', True)        # enable parallel processing for faster updates
        self.declare_parameter('tcp_ip', '127.0.0.1')               # TCP server IP
        self.declare_parameter('tcp_port', 12349)                   # TCP client connection port
        self.declare_parameter('tcp_server_port', 8912)             # TCP server listening port
        
        # ENHANCED MAP CONFIGURATION: Optimized for road environment
        # ------------------------------------------------------
        self.declare_parameter('map_width', 600)                    # cells - wider for better side detection
        self.declare_parameter('map_height', 500)                   # cells
        self.declare_parameter('map_resolution', 0.15)              # meters/cell - finer resolution for better details
        self.declare_parameter('map_origin_x', -50.0)               # meters
        self.declare_parameter('map_origin_y', -50.0)               # meters
        self.declare_parameter('obstacle_inflation', 3)             # cells - increased for safer planning
        self.declare_parameter('max_detection_radius', 50.0)        # meters - increased for earlier detection
        self.declare_parameter('wall_detection_range', 8.0)         # meters - increased for better close vehicle detection
        
        # VEHICLE DETECTION THRESHOLDS: Tuned for typical road objects
        # ---------------------------------------------------------
        self.declare_parameter('height_threshold_min', -1.5)        # meters - lower to catch curbs and road features
        self.declare_parameter('height_threshold_max', 3.0)         # meters - higher to catch trucks and tall vehicles
        self.declare_parameter('vehicle_min_height', 0.2)           # meters - minimum height for potential vehicles
        self.declare_parameter('vehicle_max_height', 2.5)           # meters - maximum height for typical vehicles
        self.declare_parameter('vehicle_inflation', 1.5)            # multiplier - vehicle-specific inflation factor
        
        # ADAPTIVE PARAMETERS: For optimal real-time performance
        # --------------------------------------------------
        self.declare_parameter('dynamic_resolution', True)          # dynamically adjust resolution for performance
        self.declare_parameter('center_map_on_vehicle', True)       # center map on vehicle for moving environments
        
        # VISUALIZATION PARAMETERS: For better visibility in RViz
        # ---------------------------------------------------
        self.declare_parameter('free_space_value', 0)               # Value for free space (white)
        self.declare_parameter('obstacle_value', 100)               # Value for obstacles (black)
        self.declare_parameter('unknown_value', 50)                 # Value for unknown areas (gray)
        self.declare_parameter('vehicle_cost_bonus', 20)            # Additional cost for vehicle cells
        
        # LOAD ALL PARAMETERS: Efficiently load all parameters once at startup
        # -----------------------------------------------------------------
        self.publish_rate = self.get_parameter('publish_rate').value
        self.point_lifetime = self.get_parameter('point_lifetime').value
        self.max_points_per_update = self.get_parameter('max_points_per_update').value
        self.max_points_per_batch = self.get_parameter('max_points_per_batch').value
        self.use_multiprocessing = self.get_parameter('use_multiprocessing').value
        self.parallel_ray_tracing = self.get_parameter('parallel_ray_tracing').value
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.tcp_server_port = self.get_parameter('tcp_server_port').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.max_detection_radius = self.get_parameter('max_detection_radius').value
        self.wall_detection_range = self.get_parameter('wall_detection_range').value
        self.height_threshold_min = self.get_parameter('height_threshold_min').value
        self.height_threshold_max = self.get_parameter('height_threshold_max').value
        self.vehicle_min_height = self.get_parameter('vehicle_min_height').value
        self.vehicle_max_height = self.get_parameter('vehicle_max_height').value
        self.vehicle_inflation = self.get_parameter('vehicle_inflation').value
        self.dynamic_resolution = self.get_parameter('dynamic_resolution').value
        self.center_map_on_vehicle = self.get_parameter('center_map_on_vehicle').value
        self.free_space_value = self.get_parameter('free_space_value').value
        self.obstacle_value = self.get_parameter('obstacle_value').value
        self.unknown_value = self.get_parameter('unknown_value').value
        self.vehicle_cost_bonus = self.get_parameter('vehicle_cost_bonus').value
        
        # Initialize costmap and data storage
        self.costmap = np.ones((self.map_height, self.map_width), dtype=np.int8) * self.unknown_value
        
        # Enhanced point storage with timestamps for realtime handling
        self.timed_lidar_points = []  # List of (timestamp, x, y, z) tuples
        self.current_vehicle_position = (0.0, 0.0)  # Current vehicle position (x, y)
        self.last_vehicle_position = (0.0, 0.0)     # Last vehicle position for movement detection
        self.last_update_time = time.time()
        self.last_map_reset_time = time.time()
        
        # Performance monitoring
        self.processing_times = []  # Track processing times for adaptive optimization
        self.frame_count = 0
        self.last_position_update = time.time()
        
        # Thread safety
        self.points_lock = Lock()
        self.costmap_lock = Lock()
        
        # Statistics
        self.received_points_count = 0
        self.filtered_points_count = 0
        self.points_in_costmap = 0
        
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
            '/lidar_raw_costmap', 
            reliable_qos
        )
        
        # Also publish the processed points for visualization
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/lidar/processed_raw_points',
            10
        )
        
        # TCP Server setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = None
        
        # Start TCP server
        self.get_logger().info(f'Starting TCP server on {self.tcp_ip}:{self.tcp_port}')
        try:
            self.server_socket.bind((self.tcp_ip, self.tcp_port))
            self.server_socket.listen(1)
            # Make socket non-blocking for the accept call
            self.server_socket.setblocking(False)
            self.get_logger().info('Waiting for client connection...')
        except Exception as e:
            self.get_logger().error(f'Server setup failed: {str(e)}')
            raise
        
        # Create timers
        self.connection_timer = self.create_timer(1.0, self.check_connection)  # Check connection every second
        self.receiver_timer = self.create_timer(0.01, self.receive_data)      # Try to receive data at 100Hz
        self.costmap_timer = self.create_timer(1.0/self.publish_rate, self.update_and_publish_costmap)
        self.stats_timer = self.create_timer(5.0, self.report_stats)           # Report stats every 5 seconds
        
        self.get_logger().info('LiDAR Raw Costmap Creator initialized')
        
    def check_connection(self):
        """Check for client connection if not already connected"""
        if self.client_socket is None:
            try:
                self.client_socket, addr = self.server_socket.accept()
                self.client_socket.setblocking(True)  # Make data reception blocking
                self.get_logger().info(f'Client connected from {addr}')
            except BlockingIOError:
                # No connection available, which is normal
                pass
            except Exception as e:
                self.get_logger().warn(f'Connection error: {str(e)}')
    
    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        if self.client_socket is None:
            return None
            
        data = b''
        try:
            while len(data) < size:
                packet = self.client_socket.recv(size - len(data))
                if not packet:
                    self.get_logger().warn('Connection closed by client')
                    self.client_socket.close()
                    self.client_socket = None
                    return None
                data += packet
            return data
        except Exception as e:
            self.get_logger().error(f'Error in receive_exact: {str(e)}')
            if self.client_socket is not None:
                self.client_socket.close()
                self.client_socket = None
            return None
    
    def receive_data(self):
        """Receive raw LIDAR data points from TCP connection with optimized filtering for realtime performance"""
        if self.client_socket is None:
            return
            
        try:
            # INCREASED DATA RECEPTION: Larger point collection per update
            # -----------------------------------------------------------
            # Significantly increase the number of points processed per update to match
            # the original lidar_listener_raw performance while maintaining real-time response
            new_points = []
            points_received_this_batch = 0
            max_reception_time = 0.05  # 50ms max time spent receiving to maintain real-time performance
            start_time = time.time()
            
            # AGGRESSIVE DATA COLLECTION: Non-blocking continuous reception
            # -----------------------------------------------------------
            # Continue receiving as long as data is available and we haven't exceeded our time budget
            while time.time() - start_time < max_reception_time and points_received_this_batch < self.max_points_per_update * 2:
                # Fast non-blocking check if data is available
                try:
                    self.client_socket.settimeout(0.0)  # Non-blocking
                    self.client_socket.recv(1, socket.MSG_PEEK)  # Check if data is available
                    self.client_socket.settimeout(0.01)  # Very short timeout for responsive operation
                except (socket.timeout, BlockingIOError):
                    # No data available right now, which is normal
                    break
                except Exception as e:
                    # Any other error is worth logging
                    self.get_logger().warn(f'Socket peek error: {str(e)}')
                    break
                    
                # IMPROVED DATA RECEPTION: Batch-oriented with fallback
                # --------------------------------------------------
                # Try to receive large batches first for efficiency, with fallback to individual point reception
                try:
                    # First try batch reception (more efficient - gets more points at once)
                    batch_size = min(self.max_points_per_batch, self.max_points_per_update - points_received_this_batch)
                    batch_bytes = batch_size * 12  # Each point is 12 bytes (3 floats * 4 bytes)
                    
                    batch_data = bytearray()
                    chunk_start = time.time()
                    
                    # Receive data in chunks
                    while len(batch_data) < batch_bytes and time.time() - chunk_start < 0.01:
                        try:
                            # Request a reasonable-sized chunk (1-4KB)
                            chunk_size = min(4096, batch_bytes - len(batch_data))
                            chunk = self.client_socket.recv(chunk_size)
                            
                            if not chunk:  # Connection closed
                                self.get_logger().warn("TCP connection closed during batch reception")
                                raise ConnectionError("Connection closed")
                                
                            batch_data.extend(chunk)
                        except (socket.timeout, BlockingIOError):
                            # This is expected with a short timeout
                            break
                    
                    # Process complete points from the batch
                    point_count = len(batch_data) // 12  # Each point is 12 bytes
                    
                    if point_count > 0:
                        self.get_logger().debug(f"Received batch of {point_count} points ({len(batch_data)} bytes)")
                    
                    current_time = time.time()
                    
                    # Process each complete point
                    for i in range(point_count):
                        start_idx = i * 12
                        point_data = batch_data[start_idx:start_idx+12]
                        
                        try:
                            x, y, z = struct.unpack('fff', point_data)
                            self.received_points_count += 1
                            points_received_this_batch += 1
                            
                            # Less aggressive filtering to keep more points
                            if self.validate_point(x, y, z):
                                new_points.append((current_time, x, y, z))
                            else:
                                self.filtered_points_count += 1
                        except struct.error:
                            # Skip incomplete data
                            continue
                        
                    # If we got a partial point at the end, handle it separately
                    leftover_bytes = len(batch_data) % 12
                    if leftover_bytes > 0:
                        self.get_logger().debug(f"Had {leftover_bytes} leftover bytes")
                        leftover = batch_data[-leftover_bytes:]
                        
                        try:
                            # Try to receive the rest of the point
                            remaining = self.receive_exact(12 - leftover_bytes)
                            if remaining:
                                full_point = leftover + remaining
                                x, y, z = struct.unpack('fff', full_point)
                                self.received_points_count += 1
                                points_received_this_batch += 1
                                
                                if self.validate_point(x, y, z):
                                    new_points.append((current_time, x, y, z))
                                else:
                                    self.filtered_points_count += 1
                        except Exception as e:
                            # Just skip this partial point
                            pass
                    
                    # If we got very few points in the batch, fall back to point-by-point
                    # This helps when data arrives more slowly
                    if point_count < 5 and points_received_this_batch < self.max_points_per_update:
                        # Fall back to individual point reception for better handling of slow data
                        for _ in range(min(50, self.max_points_per_update - points_received_this_batch)):
                            try:
                                # Check if there's still data available
                                self.client_socket.settimeout(0.0)
                                self.client_socket.recv(1, socket.MSG_PEEK)
                                self.client_socket.settimeout(0.01)
                                
                                # Receive one point (12 bytes)
                                point_data = self.receive_exact(12)
                                if not point_data:
                                    break
                                
                                x, y, z = struct.unpack('fff', point_data)
                                self.received_points_count += 1
                                points_received_this_batch += 1
                                
                                if self.validate_point(x, y, z):
                                    new_points.append((current_time, x, y, z))
                                else:
                                    self.filtered_points_count += 1
                            except (socket.timeout, BlockingIOError):
                                # No more data available
                                break
                            except Exception as e:
                                self.get_logger().warn(f"Error in fallback point reception: {str(e)}")
                                break
                                
                except Exception as e:
                    self.get_logger().warn(f'Error in batch reception: {str(e)}')
                    
                    # If batch reception failed, try individual point reception as a fallback
                    try:
                        # Reset socket to known state
                        self.client_socket.settimeout(0.01)
                        
                        # Try to get individual points
                        for _ in range(min(20, self.max_points_per_update - points_received_this_batch)):
                            try:
                                # Check for data availability
                                self.client_socket.settimeout(0.0)
                                self.client_socket.recv(1, socket.MSG_PEEK)
                                self.client_socket.settimeout(0.01)
                                
                                # Receive one point
                                point_data = self.receive_exact(12)
                                if not point_data:
                                    break
                                    
                                x, y, z = struct.unpack('fff', point_data)
                                self.received_points_count += 1
                                points_received_this_batch += 1
                                
                                if self.validate_point(x, y, z):
                                    new_points.append((time.time(), x, y, z))
                                else:
                                    self.filtered_points_count += 1
                            except (socket.timeout, BlockingIOError):
                                # No more data, which is normal
                                break
                            except Exception as inner_e:
                                self.get_logger().warn(f'Error in individual point reception: {str(inner_e)}')
                                break
                    except Exception as recovery_e:
                        self.get_logger().error(f'Failed to recover from batch reception error: {str(recovery_e)}')
                
                # Check if we've spent too much time receiving
                if time.time() - start_time >= max_reception_time:
                    self.get_logger().debug("Reception time limit reached")
                    break
                    
                # Also stop if we've collected enough points
                if points_received_this_batch >= self.max_points_per_update:
                    self.get_logger().debug("Maximum points limit reached")
                    break
            
            # IMPROVED THREAD SAFETY: Thread-safe data updates
            # ----------------------------------------------
            if new_points:
                with self.points_lock:
                    # Add new points
                    self.timed_lidar_points.extend(new_points)
                    
                    # More efficient filtering - only filter if we have accumulated a lot of points
                    if len(self.timed_lidar_points) > self.max_points_per_update * 2:
                        current_time = time.time()
                        cutoff_time = current_time - self.point_lifetime
                        
                        # Keep only recent points (highly optimized list comprehension)
                        self.timed_lidar_points = [p for p in self.timed_lidar_points if p[0] >= cutoff_time]
                        
                        # Also limit the total number of points to prevent unbounded growth
                        # For the highest performance, just keep the most recent points
                        if len(self.timed_lidar_points) > self.max_points_per_update * 3:
                            self.timed_lidar_points = self.timed_lidar_points[-self.max_points_per_update * 3:]
                
                # Enhanced debugging with more details on point batches
                if len(new_points) > 0:
                    self.get_logger().info(f'Received {len(new_points)} points (filtered {self.filtered_points_count}), ' 
                                          f'total points in buffer: {len(self.timed_lidar_points)}')
        
        except ConnectionError:
            # Handle connection problems (try to reconnect)
            self.get_logger().error("TCP connection lost, will try to reconnect")
            if self.client_socket is not None:
                self.client_socket.close()
                self.client_socket = None
        except Exception as e:
            self.get_logger().error(f'Error in receive_data: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            if self.client_socket is not None:
                self.client_socket.close()
                self.client_socket = None

    def validate_point(self, x, y, z):
        """Validate a LIDAR point with emphasis on vehicle and road object detection"""
        # Check for NaN or Inf values (these are never valid)
        if math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isinf(x) or math.isinf(y) or math.isinf(z):
            return False
            
        # VEHICLE DETECTION: Only filter extreme outliers
        # ----------------------------------------------
        # Use very lenient bounds checking to keep all potentially useful data points
        if abs(x) > 1000.0 or abs(y) > 1000.0 or abs(z) > 15.0:  # Increased z bounds for tall vehicles
            return False
        
        # ROAD OBJECT DETECTION: Special handling for road-level objects
        # ------------------------------------------------------------
        # Road objects (cars, pedestrians, etc.) typically have z values in specific ranges
        # The LIDAR is mounted 2m high, so objects like cars are typically -1m to 2m relative to sensor
        is_road_object_height = -1.5 <= z <= 3.5  # Expanded range to catch trucks and SUVs
        
        # CLOSE VEHICLE DETECTION: High priority for nearby objects
        # -------------------------------------------------------
        # Objects close to the vehicle are extremely important for collision avoidance
        # These would typically be other vehicles in front, beside, or cars passing by
        if abs(x) < 15.0 and abs(y) < 8.0:  # Typical road width plus margin
            # For very close objects, be very permissive with height
            if abs(z) < 4.0:  # Accept almost any height for close objects
                return True
            
            # For objects directly in our path, be extremely lenient
            if abs(x) < 10.0 and abs(y) < 3.0:  # Car directly ahead
                return True
        
        # OBJECT SIZE HEURISTIC: Cars and road objects have a certain height profile
        # ------------------------------------------------------------------------
        # Most road objects won't be extremely high or extremely low
        if is_road_object_height:
            return True
            
        # GROUND AND OVERHEAD FILTERING
        # ----------------------------
        # Filter points that are likely ground or high overhead objects
        # Ground points are typically near z=0 or below
        is_ground = z < -0.5 and abs(x) > 5.0 and abs(y) > 5.0  # Likely ground outside immediate vicinity
        
        # Overhead objects (signs, bridges, etc.) are typically high and not a concern for driving
        is_overhead = z > 4.0 and abs(x) > 10.0  # High objects in the distance
        
        if is_ground or is_overhead:
            return False
            
        # Accept most other points
        return True
    
    def update_and_publish_costmap(self):
        """Update the costmap with current LIDAR data and publish it, with realtime optimizations"""
        # REALTIME OPTIMIZATION: Performance tracking
        # -----------------------------------------
        # Track processing time to help with adaptive parameter tuning
        start_time = time.time()
        
        # REALTIME OPTIMIZATION: Thread-safe data access
        # --------------------------------------------
        # Safely access the point data that might be modified by the receiver thread
        with self.points_lock:
            if not self.timed_lidar_points:
                return
                
            # REALTIME OPTIMIZATION: Point limiting
            # -----------------------------------
            # Only process a controlled number of points to maintain consistent frame rates
            # If we have too many points, use the most recent ones (more relevant)
            points = self.timed_lidar_points[-self.max_points_per_update:] if len(self.timed_lidar_points) > self.max_points_per_update else self.timed_lidar_points
            
            # REALTIME OPTIMIZATION: Data preprocessing
            # ---------------------------------------
            # Extract just the position data, discarding timestamps which aren't needed for mapping
            # This reduces memory usage and simplifies subsequent processing
            extracted_points = [(p[1], p[2], p[3]) for p in points]
        
        # REALTIME OPTIMIZATION: Vehicle tracking
        # -------------------------------------
        # Update vehicle position to keep map centered on vehicle in realtime
        self.update_vehicle_position()
        
        # REALTIME OPTIMIZATION: Dynamic map repositioning
        # ----------------------------------------------
        # Recenter the map when the vehicle moves significantly
        # This ensures the vehicle stays within the mapped area without needing an enormous map
        if self.center_map_on_vehicle and self.has_vehicle_moved_significantly():
            self.recenter_map()
        
        # REALTIME OPTIMIZATION: Focused processing
        # ---------------------------------------
        # Only update the costmap with current data - the core of our processing
        self.update_costmap(extracted_points)
        
        # Publish both the point cloud (for debugging) and costmap (for navigation)
        self.publish_point_cloud(extracted_points)
        self.publish_costmap()
        
        # REALTIME OPTIMIZATION: Adaptive parameter tuning
        # ----------------------------------------------
        # Track processing time to adjust parameters dynamically
        process_time = time.time() - start_time
        self.processing_times.append(process_time)
        if len(self.processing_times) > 10:
            self.processing_times.pop(0)
        
        # REALTIME OPTIMIZATION: Self-tuning system
        # ---------------------------------------
        # Automatically adjust processing parameters based on performance
        self.adapt_parameters()
        
        self.frame_count += 1
    
    def update_vehicle_position(self):
        """Update the current vehicle position from TF if available"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'ego_vehicle', rclpy.time.Time())
            vehicle_x = transform.transform.translation.x
            vehicle_y = transform.transform.translation.y
            
            # Store the last known position before updating
            self.last_vehicle_position = self.current_vehicle_position
            self.current_vehicle_position = (vehicle_x, vehicle_y)
            self.last_position_update = time.time()
            
            self.get_logger().debug(f"Updated vehicle position: {vehicle_x}, {vehicle_y}")
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f"Could not get vehicle transform: {e}")
            return False

    def has_vehicle_moved_significantly(self):
        """Check if the vehicle has moved enough to warrant map recentering"""
        if not self.center_map_on_vehicle:
            return False
            
        # Get current and last positions
        current_x, current_y = self.current_vehicle_position
        last_x, last_y = self.last_vehicle_position
        
        # Calculate distance moved
        distance = math.sqrt((current_x - last_x)**2 + (current_y - last_y)**2)
        
        # If moved more than 20% of the map's radius, recenter
        map_radius = min(self.map_width, self.map_height) * self.map_resolution / 2
        significant_distance = map_radius * 0.2
        
        return distance > significant_distance

    def recenter_map(self):
        """Recenter the map on the vehicle's current position"""
        with self.costmap_lock:
            # Get current vehicle position
            vehicle_x, vehicle_y = self.current_vehicle_position
            
            # Calculate new map origin to center the vehicle
            new_origin_x = vehicle_x - (self.map_width * self.map_resolution / 2)
            new_origin_y = vehicle_y - (self.map_height * self.map_resolution / 2)
            
            # Update map origin
            self.map_origin_x = new_origin_x
            self.map_origin_y = new_origin_y
            
            # Clear the costmap for a fresh start
            self.costmap.fill(self.unknown_value)
            
            self.get_logger().info(f"Recentered map on vehicle at ({vehicle_x}, {vehicle_y})")

    def adapt_parameters(self):
        """Adapt parameters based on performance for optimal realtime operation"""
        if not self.dynamic_resolution or len(self.processing_times) < 5:
            return
            
        # Calculate average processing time
        avg_time = sum(self.processing_times) / len(self.processing_times)
        
        # Target time per frame (70% of our update interval)
        target_time = 0.7 / self.publish_rate
        
        # If we're processing too slowly
        if avg_time > target_time:
            # Reduce the number of points we process
            self.max_points_per_update = max(500, int(self.max_points_per_update * 0.9))
            
            # Could also reduce resolution if needed
            if avg_time > 1.5 * target_time and self.map_resolution < 0.2:
                new_resolution = min(0.2, self.map_resolution * 1.2)
                
                # Only log if there's a significant change
                if new_resolution / self.map_resolution > 1.1:
                    self.get_logger().info(
                        f"Reducing map resolution from {self.map_resolution:.3f}m to {new_resolution:.3f}m " 
                        f"for better performance (avg processing: {avg_time*1000:.1f}ms)"
                    )
                    self.map_resolution = new_resolution
        
        # If we're processing very quickly, we can improve quality
        elif avg_time < 0.5 * target_time and self.frame_count > 30:
            # Increase the number of points we process
            self.max_points_per_update = min(10000, int(self.max_points_per_update * 1.1))
            
            # Could also increase resolution for better quality
            if avg_time < 0.3 * target_time and self.map_resolution > 0.03:
                new_resolution = max(0.03, self.map_resolution * 0.9)
                
                # Only log if there's a significant change
                if self.map_resolution / new_resolution > 1.1:
                    self.get_logger().info(
                        f"Increasing map resolution from {self.map_resolution:.3f}m to {new_resolution:.3f}m " 
                        f"for better quality (avg processing: {avg_time*1000:.1f}ms)"
                    )
                    self.map_resolution = new_resolution

    def update_costmap(self, points):
        """Update the costmap based on the current LIDAR points with focus on vehicle detection"""
        with self.costmap_lock:
            # Skip if we don't have points
            if not points:
                self.get_logger().warn('No points available for costmap update')
                return
                
            # Process all points efficiently for better object detection
            points_used = 0
            
            # Pre-compute squared radius for faster distance checks (avoids sqrt operations)
            max_detection_radius_cells = int(self.max_detection_radius / self.map_resolution)
            max_radius_squared = max_detection_radius_cells * max_detection_radius_cells
            
            # Enhanced sensor position tracking - account for LIDAR mounting position
            vehicle_x, vehicle_y = self.current_vehicle_position
            sensor_x = vehicle_x + 1.5  # LIDAR is typically 1.5m forward from vehicle center
            sensor_y = vehicle_y
            
            # Convert to grid coordinates once (reused multiple times)
            sensor_grid_x = int((sensor_x - self.map_origin_x) / self.map_resolution)
            sensor_grid_y = int((sensor_y - self.map_origin_y) / self.map_resolution)
            
            # More efficient update area tracking using numpy's binary operations
            update_area = np.zeros((self.map_height, self.map_width), dtype=bool)
            
            # Enhanced object classification for better vehicle detection
            # Cars and road objects need special consideration
            obstacle_cells = set()                # Regular obstacles
            vehicle_cells = set()                 # Likely vehicle obstacles
            close_obstacle_cells = set()          # Very close obstacles (high priority)
            road_object_cells = {}                # Maps cells to their z-values for height-based classification
            
            # Wall detection range squared (avoids sqrt in distance calculation)
            wall_range_squared = self.wall_detection_range * self.wall_detection_range
            
            # ENHANCED OBJECT DETECTION: Identify different types of objects
            # ------------------------------------------------------------
            for x, y, z in points:
                # Convert world coordinates to grid coordinates
                grid_x = int((x - self.map_origin_x) / self.map_resolution)
                grid_y = int((y - self.map_origin_y) / self.map_resolution)
                
                # Skip if out of bounds
                if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                    continue
                
                # Calculate squared distance from sensor
                dist_squared = (x - sensor_x)**2 + (y - sensor_y)**2
                
                # VEHICLE DETECTION HEURISTICS
                # ---------------------------
                # Characteristics of vehicle points:
                # 1. Often appear as clusters at specific heights
                # 2. Typically 0.2 to 2.0m above ground level
                # 3. Often form rectangular shapes in top-down view
                
                # Track point height for cell to assist with vehicle classification
                cell_key = (grid_y, grid_x)
                if cell_key in road_object_cells:
                    # Store min and max z for each cell to analyze height profile
                    min_z, max_z = road_object_cells[cell_key]
                    road_object_cells[cell_key] = (min(min_z, z), max(max_z, z))
                else:
                    road_object_cells[cell_key] = (z, z)
                
                # Very close obstacles are highest priority (immediate collision risks)
                if dist_squared < wall_range_squared:
                    close_obstacle_cells.add(cell_key)
                    
                    # For very close objects, enhance detection by marking adjacent cells
                    # This makes narrow objects more visible and ensures they're not missed
                    if dist_squared < 16.0:  # 4m squared - immediate proximity
                        for dy in range(-1, 2):
                            for dx in range(-1, 2):
                                ny, nx = grid_y + dy, grid_x + dx
                                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                    close_obstacle_cells.add((ny, nx))
                
                # Detect potential vehicle points based on height profile
                # Vehicles typically extend above ground level by 1.5-2m
                elif self.vehicle_min_height <= z <= self.vehicle_max_height:
                    # Potential vehicle point
                    vehicle_cells.add(cell_key)
                else:
                    # Regular obstacle
                    obstacle_cells.add(cell_key)
                
                points_used += 1
            
            # Post-process road object cells to identify likely vehicles
            # A vehicle typically has a height span of at least 0.5-1.5m
            for cell, (min_z, max_z) in road_object_cells.items():
                # Check if cell has a height profile consistent with a vehicle
                height_span = max_z - min_z
                if height_span >= 0.5 and min_z >= -0.3 and max_z <= 3.0:
                    # Cell has a height profile consistent with a vehicle or road object
                    vehicle_cells.add(cell)
            
            # Define circular update area more efficiently using numpy operations
            y_indices, x_indices = np.ogrid[:self.map_height, :self.map_width]
            dist_squared = (y_indices - sensor_grid_y)**2 + (x_indices - sensor_grid_x)**2
            update_area = dist_squared <= max_radius_squared
            
            # Limit to map bounds for efficiency
            update_area[:max(0, sensor_grid_y - max_detection_radius_cells)] = False
            update_area[min(self.map_height, sensor_grid_y + max_detection_radius_cells + 1):] = False
            update_area[:, :max(0, sensor_grid_x - max_detection_radius_cells)] = False
            update_area[:, min(self.map_width, sensor_grid_x + max_detection_radius_cells + 1):] = False
            
            # Reset only the update area to unknown
            self.costmap[update_area] = self.unknown_value
            
            # ENHANCED RAY TRACING
            # ------------------
            # Perform raytracing to identify free space
            free_space_cells = set()
            
            # Combine all obstacles with vehicles prioritized
            all_obstacles = obstacle_cells.union(vehicle_cells).union(close_obstacle_cells)
            
            # Parallel processing for ray tracing if enabled and we have many obstacles
            if self.parallel_ray_tracing and len(all_obstacles) > 100 and self.use_multiprocessing:
                try:
                    import multiprocessing as mp
                    from functools import partial
                    
                    # Set up parallel processing
                    num_cores = mp.cpu_count() - 1 or 1  # Leave one core free
                    
                    # FIXED: Use a modified version of the bresenham line function that can be pickled
                    # By creating a partial function that includes the bresenham_line method
                    worker_func = partial(
                        process_cell_batch, 
                        sensor_y=sensor_grid_y, 
                        sensor_x=sensor_grid_x,
                        width=self.map_width,
                        height=self.map_height,
                        bresenham_line_func=self.bresenham_line
                    )
                    
                    # Divide obstacles into batches for parallel processing
                    batch_size = max(10, len(all_obstacles) // num_cores)
                    obstacle_batches = [list(all_obstacles)[i:i+batch_size] 
                                      for i in range(0, len(all_obstacles), batch_size)]
                    
                    with mp.Pool(num_cores) as pool:
                        # Process batches in parallel
                        results = pool.map(worker_func, obstacle_batches)
                    
                    # Combine results
                    for result_set in results:
                        free_space_cells.update(result_set)
                    
                    self.get_logger().debug(f"Used parallel processing with {num_cores} cores")
                        
                except Exception as e:
                    # Fall back to serial processing if parallel fails
                    self.get_logger().warn(f"Parallel processing failed, using serial: {str(e)}")
                    self.parallel_ray_tracing = False  # Disable for future updates to avoid repeated errors
                    
                    # Regular serial processing as fallback
                    for cell in all_obstacles:
                        grid_y, grid_x = cell
                        line_points = self.bresenham_line(sensor_grid_y, sensor_grid_x, grid_y, grid_x)
                        for i in range(len(line_points) - 1):
                            y, x = line_points[i]
                            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                                free_space_cells.add((y, x))
            else:
                # Standard serial processing
                for cell in all_obstacles:
                    grid_y, grid_x = cell
                    line_points = self.bresenham_line(sensor_grid_y, sensor_grid_x, grid_y, grid_x)
                    for i in range(len(line_points) - 1):
                        y, x = line_points[i]
                        if 0 <= x < self.map_width and 0 <= y < self.map_height:
                            free_space_cells.add((y, x))
            
            # Remove obstacle cells from free space
            free_space_cells = free_space_cells - all_obstacles
            
            # PRIORITY-BASED COSTMAP UPDATES
            # ----------------------------
            # Apply values to the costmap with proper priorities
            
            # Apply free space (with bounds and update area check)
            for y, x in free_space_cells:
                if update_area[y, x]:
                    self.costmap[y, x] = self.free_space_value
            
            # Apply regular obstacles
            for y, x in obstacle_cells:
                if 0 <= y < self.map_height and 0 <= x < self.map_width:
                    self.costmap[y, x] = self.obstacle_value
            
            # Apply vehicle cells with higher cost to ensure they're avoided
            vehicle_cost = min(self.obstacle_value + self.vehicle_cost_bonus, 100)  # Higher cost for vehicles, capped at 100
            for y, x in vehicle_cells:
                if 0 <= y < self.map_height and 0 <= x < self.map_width:
                    self.costmap[y, x] = vehicle_cost
            
            # Close obstacles get highest priority
            for y, x in close_obstacle_cells:
                if 0 <= y < self.map_height and 0 <= x < self.map_width:
                    # Maximum cost for close obstacles to ensure they're avoided
                    self.costmap[y, x] = self.obstacle_value
            
            # Apply vehicle-specific inflation to make cars more visible
            # Use larger inflation for vehicles to ensure safe passing distance
            self.inflate_objects(vehicle_cells, self.vehicle_inflation, update_area)
            
            # Apply general obstacle inflation
            if self.obstacle_inflation > 0:
                self.inflate_obstacles(update_area)
            
            # Update statistics
            self.points_in_costmap = points_used
            
            # Enhanced logging with vehicle count
            vehicle_count = len(vehicle_cells)
            if vehicle_count > 0:
                self.get_logger().info(f'Updated costmap with {points_used} points, detected {vehicle_count} potential vehicle cells')
            else:
                self.get_logger().debug(f'Updated costmap with {points_used} points')
                
    def inflate_objects(self, object_cells, inflation_factor, update_area=None):
        """Special inflation method for vehicles and larger objects"""
        if not object_cells:
            return
            
        # Calculate vehicle-specific inflation radius
        inflation_radius = max(3, int(inflation_factor / self.map_resolution))
        
        # For each vehicle cell, apply larger inflation
        for y, x in object_cells:
            # Apply inflation to nearby cells
            for dy in range(-inflation_radius, inflation_radius + 1):
                for dx in range(-inflation_radius, inflation_radius + 1):
                    ny, nx = y + dy, x + dx
                    
                    # Skip if out of bounds
                    if not (0 <= nx < self.map_width and 0 <= ny < self.map_height):
                        continue
                    
                    # Skip if not in update area (when provided)
                    if update_area is not None and not update_area[ny, nx]:
                        continue
                    
                    # Calculate distance from vehicle cell
                    distance = math.sqrt(dx**2 + dy**2)
                    
                    # Only inflate within the radius
                    if distance <= inflation_radius:
                        # Use more gradual decay for larger objects
                        decay_factor = max(0, 1.0 - (distance / inflation_radius) ** 0.8)
                        inflated_cost = int(self.obstacle_value * decay_factor)
                        
                        # Only update if the new cost is higher
                        if inflated_cost > self.costmap[ny, nx]:
                            # Don't overwrite free space with inflation
                            if self.costmap[ny, nx] != self.free_space_value:
                                self.costmap[ny, nx] = inflated_cost

    def inflate_obstacles(self, update_area=None):
        """Inflate obstacles to account for robot size and safety margin"""
        # Create a copy of the original costmap
        original_costmap = np.copy(self.costmap)
        
        # Calculate inflation radius in grid cells
        inflation_radius = max(2, int(self.obstacle_inflation / self.map_resolution))
        
        # Find all obstacle cells
        if update_area is not None:
            # Only inflate obstacles in the update area for realtime performance
            obstacle_mask = (original_costmap == self.obstacle_value) & update_area
        else:
            obstacle_mask = (original_costmap == self.obstacle_value)
            
        obstacle_cells = np.where(obstacle_mask)
        
        # For each obstacle cell, inflate it
        for y, x in zip(obstacle_cells[0], obstacle_cells[1]):
            # Apply inflation to nearby cells
            for dy in range(-inflation_radius, inflation_radius + 1):
                for dx in range(-inflation_radius, inflation_radius + 1):
                    ny, nx = y + dy, x + dx
                    
                    # Skip if out of bounds
                    if not (0 <= nx < self.map_width and 0 <= ny < self.map_height):
                        continue
                    
                    # Skip if not in update area (when provided)
                    if update_area is not None and not update_area[ny, nx]:
                        continue
                    
                    # Calculate distance from obstacle
                    distance = math.sqrt(dx**2 + dy**2)
                    
                    # Only inflate within the radius
                    if distance <= inflation_radius:
                        # Use distance-based decay for the inflated cost
                        decay_factor = max(0, 1.0 - (distance / inflation_radius))
                        inflated_cost = int(self.obstacle_value * decay_factor)
                        
                        # Only update if the new cost is higher
                        if inflated_cost > self.costmap[ny, nx]:
                            # Don't overwrite free space with inflation
                            if self.costmap[ny, nx] != self.free_space_value:
                                self.costmap[ny, nx] = inflated_cost
    
    def bresenham_line(self, y0, x0, y1, x1):
        """Bresenham's line algorithm for raytracing from sensor to obstacle"""
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
    
    def publish_costmap(self):
        """Create and publish occupancy grid message from the costmap"""
        with self.costmap_lock:
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
            costmap_msg.data = self.costmap.flatten().tolist()
            
            # Publish costmap
            self.costmap_publisher.publish(costmap_msg)
    
    def publish_point_cloud(self, points):
        """Publish the processed points as a PointCloud2 message for visualization"""
        if not points:
            return
            
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set up fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Convert points to numpy array
        points_array = np.array(points, dtype=np.float32)
        
        # Set message parameters
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = points_array.tobytes()
        
        # Publish the message
        self.point_cloud_publisher.publish(msg)
    
    def report_stats(self):
        """Report statistics about data reception and processing"""
        # Calculate time since last stats report
        now = time.time()
        elapsed = now - self.last_update_time
        self.last_update_time = now
        
        # Report statistics
        if elapsed > 0:
            points_per_sec = self.received_points_count / elapsed
            filtered_per_sec = self.filtered_points_count / elapsed
            
            self.get_logger().info(
                f'LIDAR Stats: Received {points_per_sec:.1f} points/s, '
                f'Filtered {filtered_per_sec:.1f} points/s, '
                f'Using {self.points_in_costmap} points in costmap'
            )
            
            # Reset counters
            self.received_points_count = 0
            self.filtered_points_count = 0
    
    def __del__(self):
        """Clean up resources on destruction"""
        if hasattr(self, 'client_socket') and self.client_socket is not None:
            self.client_socket.close()
        if hasattr(self, 'server_socket') and self.server_socket is not None:
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = LidarRawCostmapCreator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 