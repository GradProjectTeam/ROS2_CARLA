#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Point, PoseStamped
from scipy.spatial.transform import Rotation
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.exceptions import ParameterAlreadyDeclaredException

class ClusterToNav2MapConverter(Node):
    def __init__(self):
        super().__init__('cluster_to_nav2_map_converter')
        
        # Parameters for map dimensions and behavior
        self.declare_parameter('map_resolution', 0.05)       # meters per cell
        self.declare_parameter('map_width', 60.0)            # meters
        self.declare_parameter('map_height', 60.0)           # meters
        self.declare_parameter('obstacle_threshold', 70)     # 0-100 for occupancy
        self.declare_parameter('obstacle_inflation', 0.3)    # meters
        self.declare_parameter('free_space_range', 2.0)      # meters of free space to mark around obstacles
        self.declare_parameter('cluster_topic', '/lidar/cubes')
        self.declare_parameter('point_cloud_topic', '/lidar/points')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('publish_rate', 5.0)          # Hz
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('tf_buffer_duration', 4.0)    # seconds
        self.declare_parameter('tf_timeout', 0.2)            # seconds
        self.declare_parameter('transform_tolerance', 0.3)   # seconds
        self.declare_parameter('map_persistence', 5.0)       # seconds to keep obstacles on map
        self.declare_parameter('use_distance_filter', True)  # Whether to use distance-based obstacle filtering
        self.declare_parameter('max_obstacle_distance', 25.0) # Maximum distance to include obstacles
        self.declare_parameter('update_free_space_only', False) # Whether to only update free space between robot and obstacles
        
        # Handle case where use_sim_time might already be declared by ROS
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            # Parameter already declared, likely by a launch file, we can skip declaring it
            self.get_logger().debug('use_sim_time parameter already declared, using existing value')
        
        # Get parameter values
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.free_space_range = self.get_parameter('free_space_range').value
        self.cluster_topic = self.get_parameter('cluster_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.tf_buffer_duration = self.get_parameter('tf_buffer_duration').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.transform_tolerance = self.get_parameter('transform_tolerance').value
        self.map_persistence = self.get_parameter('map_persistence').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.use_distance_filter = self.get_parameter('use_distance_filter').value
        self.max_obstacle_distance = self.get_parameter('max_obstacle_distance').value
        self.update_free_space_only = self.get_parameter('update_free_space_only').value
        
        # Create map dimensions in cells
        self.width_cells = int(self.map_width / self.map_resolution)
        self.height_cells = int(self.map_height / self.map_resolution)
        
        # Calculate inflation radius in cells
        self.inflation_radius_cells = int(self.obstacle_inflation / self.map_resolution)
        
        # Calculate free space range in cells
        self.free_space_cells = int(self.free_space_range / self.map_resolution)
        
        # Configure QoS profile for Nav2 compatibility
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Publishers and subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.cluster_sub = self.create_subscription(
            MarkerArray, 
            self.cluster_topic, 
            self.cluster_callback,
            10
        )
        
        # We'll also subscribe to PointCloud2 as a fallback if needed
        # self.points_sub = self.create_subscription(
        #     PointCloud2, 
        #     self.point_cloud_topic, 
        #     self.pointcloud_callback,
        #     10
        # )
        
        # TF Buffer setup for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=self.tf_buffer_duration))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create empty map with all cells unknown (-1)
        self.map_data = np.full(self.width_cells * self.height_cells, -1, dtype=np.int8)
        
        # Store the last update time for each cell to implement temporal decay
        self.last_cell_update = np.zeros((self.height_cells, self.width_cells), dtype=float)
        self.current_time = self.get_clock().now().nanoseconds / 1e9  # Current time in seconds
        
        # Performance tracking
        self.processing_times = []
        self.max_times_to_track = 10
        
        # Timer for publishing map at a fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)
        
        # Timestamp of the last map update
        self.last_update_time = self.get_clock().now()
        
        # Pre-compute inflation kernel for faster inflation
        self.inflation_kernel = self.create_inflation_kernel(self.inflation_radius_cells)
        
        # Vehicle position in map frame
        self.vehicle_position_map = None
        
        self.get_logger().info(
            f'ClusterToNav2MapConverter initialized with map dimensions: '
            f'{self.width_cells}x{self.height_cells} cells ({self.map_width}x{self.map_height} meters)'
        )
    
    def create_inflation_kernel(self, radius):
        """Create a pre-computed kernel for faster obstacle inflation"""
        kernel_size = 2 * radius + 1
        kernel = np.zeros((kernel_size, kernel_size), dtype=bool)
        center = radius
        
        for y in range(kernel_size):
            for x in range(kernel_size):
                distance = math.sqrt((x - center)**2 + (y - center)**2)
                kernel[y, x] = (distance <= radius)
        
        return kernel
    
    def cluster_callback(self, msg):
        """Process incoming cluster markers and update the map"""
        start_time = time.time()
        self.current_time = self.get_clock().now().nanoseconds / 1e9  # Update current time
        
        # Try to get vehicle position in map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.vehicle_frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            self.vehicle_position_map = (transform.transform.translation.x, transform.transform.translation.y)
        except TransformException as e:
            self.get_logger().warning(f'Could not transform vehicle position: {e}')
            self.vehicle_position_map = None
        
        # Only reset cells that haven't been updated recently (temporal decay)
        # Convert map_data back to 2D for easier manipulation
        map_2d = self.map_data.reshape(self.height_cells, self.width_cells)
        
        # Apply temporal decay - reset cells that haven't been updated within map_persistence time
        expired_cells = (self.current_time - self.last_cell_update) > self.map_persistence
        map_2d[expired_cells] = -1  # Reset to unknown
        
        # Flatten back to 1D
        self.map_data = map_2d.flatten()
        
        cluster_count = 0
        cluster_points = []  # Store all cluster centers for ray-casting
        
        # Log information about received message
        self.get_logger().info(f'Received MarkerArray with {len(msg.markers)} markers on topic {self.cluster_topic}')
        
        # Process each marker in the MarkerArray
        for marker in msg.markers:
            # Log marker type information
            self.get_logger().debug(f'Marker type: {marker.type}, frame: {marker.header.frame_id}')
            
            # We're mainly interested in CUBE type markers which represent our clusters
            if marker.type == Marker.CUBE:
                # Apply distance filter if enabled
                if self.use_distance_filter and self.vehicle_position_map:
                    marker_position = marker.pose.position
                    distance = math.sqrt(
                        (marker_position.x - self.vehicle_position_map[0])**2 + 
                        (marker_position.y - self.vehicle_position_map[1])**2
                    )
                    
                    if distance > self.max_obstacle_distance:
                        self.get_logger().debug(f'Filtering out distant obstacle at {distance:.2f}m (max: {self.max_obstacle_distance}m)')
                        continue
                
                cluster_count += 1
                result = self.process_cube_marker(marker)
                if result:
                    x, y = result
                    cluster_points.append((x, y))
            else:
                self.get_logger().debug(f'Ignoring non-CUBE marker (type {marker.type})')
        
        # If we have vehicle position and cluster points, ray-cast to mark free space
        if self.vehicle_position_map and cluster_points and self.update_free_space_only:
            vehicle_grid_x = int((self.vehicle_position_map[0] + self.map_width/2) / self.map_resolution)
            vehicle_grid_y = int((self.vehicle_position_map[1] + self.map_height/2) / self.map_resolution)
            
            # Ensure vehicle position is within map bounds
            if 0 <= vehicle_grid_x < self.width_cells and 0 <= vehicle_grid_y < self.height_cells:
                for point_x, point_y in cluster_points:
                    self.trace_ray(vehicle_grid_x, vehicle_grid_y, point_x, point_y)
        
        self.last_update_time = self.get_clock().now()
        end_time = time.time()
        processing_time = end_time - start_time
        
        # Track processing time for performance monitoring
        self.processing_times.append(processing_time)
        if len(self.processing_times) > self.max_times_to_track:
            self.processing_times.pop(0)
        
        avg_time = sum(self.processing_times) / len(self.processing_times)
        self.get_logger().info(
            f'Processed {cluster_count} clusters in {processing_time*1000:.2f}ms '
            f'(avg: {avg_time*1000:.2f}ms)'
        )
    
    def trace_ray(self, start_x, start_y, end_x, end_y):
        """Trace a ray from start to end point and mark cells as free"""
        # Use Bresenham's line algorithm to trace the ray
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        sx = 1 if start_x < end_x else -1
        sy = 1 if start_y < end_y else -1
        err = dx - dy
        
        x, y = start_x, start_y
        
        while (x != end_x or y != end_y):
            # Mark cell as free if it's unknown
            idx = y * self.width_cells + x
            if 0 <= idx < len(self.map_data) and self.map_data[idx] == -1:
                self.map_data[idx] = 0  # Free
                self.last_cell_update[y, x] = self.current_time
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            
            # Don't trace through the endpoint (which is an obstacle)
            if x == end_x and y == end_y:
                break
    
    def process_cube_marker(self, marker):
        """Convert a cube marker to occupancy grid cells"""
        try:
            # Log marker details
            self.get_logger().debug(
                f'Processing cube marker: position=({marker.pose.position.x}, {marker.pose.position.y}), '
                f'scale=({marker.scale.x}, {marker.scale.y}), frame={marker.header.frame_id}'
            )
            
            # Transform the marker position from its frame to the map frame if needed
            if marker.header.frame_id != self.map_frame_id:
                try:
                    self.get_logger().debug(f'Attempting transform from {marker.header.frame_id} to {self.map_frame_id}')
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame_id,
                        marker.header.frame_id,
                        rclpy.time.Time(),
                        rclpy.duration.Duration(seconds=self.tf_timeout)
                    )
                    
                    # Apply transform to position
                    point_in_map = self.transform_point(marker.pose.position, transform)
                    x_meters = point_in_map.x
                    y_meters = point_in_map.y
                    
                    self.get_logger().debug(f'Transformed position: ({x_meters}, {y_meters})')
                except TransformException as e:
                    self.get_logger().warning(f'Transform error for marker: {e}')
                    return None
            else:
                # Already in map frame
                x_meters = marker.pose.position.x
                y_meters = marker.pose.position.y
            
            # Convert marker scale to grid cells for determining obstacle area
            x_scale_cells = int(marker.scale.x / self.map_resolution)
            y_scale_cells = int(marker.scale.y / self.map_resolution)
            
            # Ensure at least one cell is marked even for small clusters
            x_scale_cells = max(1, x_scale_cells)
            y_scale_cells = max(1, y_scale_cells)
            
            # Map from meters to cell coordinates, centering the map
            # Map center is considered (0,0) in the map frame
            x_grid = int((x_meters + self.map_width/2) / self.map_resolution)
            y_grid = int((y_meters + self.map_height/2) / self.map_resolution)
            
            # Check if coordinates are valid
            if x_grid < 0 or x_grid >= self.width_cells or y_grid < 0 or y_grid >= self.height_cells:
                self.get_logger().warning(
                    f'Marker at ({x_meters}, {y_meters}) meters maps to grid coordinates ({x_grid}, {y_grid}) '
                    f'which is outside the map bounds (0-{self.width_cells-1}, 0-{self.height_cells-1})'
                )
                return None
            
            self.get_logger().debug(
                f'Marker at ({x_meters}, {y_meters}) meters maps to grid coordinates ({x_grid}, {y_grid}) '
                f'with dimensions {x_scale_cells}x{y_scale_cells} cells'
            )
            
            # Mark cells as occupied, accounting for the cluster size
            self.mark_obstacle_area(x_grid, y_grid, x_scale_cells, y_scale_cells)
            
            return (x_grid, y_grid)
            
        except Exception as e:
            self.get_logger().error(f'Error processing marker: {e}')
            # Print the full traceback for better debugging
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
    
    def mark_obstacle_area(self, center_x, center_y, width_cells, height_cells):
        """Mark a rectangular area around the cluster center as occupied"""
        # Calculate the bounds of the cluster
        half_width = max(1, width_cells // 2)
        half_height = max(1, height_cells // 2)
        
        min_x = max(0, center_x - half_width)
        max_x = min(self.width_cells - 1, center_x + half_width)
        min_y = max(0, center_y - half_height)
        max_y = min(self.height_cells - 1, center_y + half_height)
        
        # Mark the cluster area as occupied
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                idx = y * self.width_cells + x
                self.map_data[idx] = 100  # Occupied (100 in Nav2)
                self.last_cell_update[y, x] = self.current_time  # Update cell timestamp
        
        # Apply inflation around the obstacles for safety - use kernel for faster inflation
        self.inflate_obstacles_fast(min_x, max_x, min_y, max_y)
        
        # Mark free space around obstacles for better path planning
        if not self.update_free_space_only:
            self.mark_free_space(min_x, max_x, min_y, max_y)
    
    def inflate_obstacles_fast(self, min_x, max_x, min_y, max_y):
        """Faster obstacle inflation using pre-computed kernel"""
        # Convert map to 2D for easier processing
        map_2d = self.map_data.reshape(self.height_cells, self.width_cells)
        
        # Create a mask for the original obstacle area
        obstacle_mask = np.zeros((self.height_cells, self.width_cells), dtype=bool)
        obstacle_mask[min_y:max_y+1, min_x:max_x+1] = True
        
        # Apply kernel at each obstacle point
        radius = self.inflation_radius_cells
        kernel = self.inflation_kernel
        kernel_size = 2 * radius + 1
        
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                # Calculate the region to update
                region_min_y = max(0, y - radius)
                region_max_y = min(self.height_cells - 1, y + radius)
                region_min_x = max(0, x - radius)
                region_max_x = min(self.width_cells - 1, x + radius)
                
                # Calculate kernel offsets
                k_min_y = max(0, radius - y)
                k_max_y = min(kernel_size - 1, radius + (self.height_cells - 1 - y))
                k_min_x = max(0, radius - x)
                k_max_x = min(kernel_size - 1, radius + (self.width_cells - 1 - x))
                
                # Apply kernel to region
                region_height = region_max_y - region_min_y + 1
                region_width = region_max_x - region_min_x + 1
                kernel_roi = kernel[k_min_y:k_max_y+1, k_min_x:k_max_x+1]
                
                # Skip if kernel ROI doesn't match region dimensions (can happen at map edges)
                if kernel_roi.shape[0] != region_height or kernel_roi.shape[1] != region_width:
                    continue
                
                # Only inflate cells that aren't occupied and are within the kernel
                inflation_mask = kernel_roi & ~obstacle_mask[region_min_y:region_max_y+1, region_min_x:region_max_x+1]
                
                if np.any(inflation_mask):
                    # Update inflated cells in map data
                    flat_indices = (np.arange(region_min_y, region_max_y + 1)[:, None] * self.width_cells + 
                                    np.arange(region_min_x, region_max_x + 1)).flatten()
                    
                    # Only update cells within the inflation mask
                    mask_flat = inflation_mask.flatten()
                    indices_to_update = flat_indices[mask_flat]
                    
                    # Set inflation values (90) and update timestamps
                    for idx in indices_to_update:
                        if 0 <= idx < len(self.map_data) and self.map_data[idx] != 100:
                            self.map_data[idx] = 90
                            y_idx, x_idx = divmod(idx, self.width_cells)
                            self.last_cell_update[y_idx, x_idx] = self.current_time
        
        # Update the original map_data from 2D representation
        self.map_data = map_2d.flatten()
    
    def mark_free_space(self, min_x, max_x, min_y, max_y):
        """Mark a ring of free space around obstacles to improve path planning"""
        # Calculate region to mark free space (outside the inflation area)
        inflation_radius = self.inflation_radius_cells
        free_space_radius = self.free_space_cells + inflation_radius
        
        # Extend the bounds by the free space radius
        free_min_x = max(0, min_x - free_space_radius)
        free_max_x = min(self.width_cells - 1, max_x + free_space_radius)
        free_min_y = max(0, min_y - free_space_radius)
        free_max_y = min(self.height_cells - 1, max_y + free_space_radius)
        
        # Calculate the inner bounds (the inflated obstacle area)
        inner_min_x = max(0, min_x - inflation_radius)
        inner_max_x = min(self.width_cells - 1, max_x + inflation_radius)
        inner_min_y = max(0, min_y - inflation_radius)
        inner_max_y = min(self.height_cells - 1, max_y + inflation_radius)
        
        # Mark cells in the free space ring as free (0)
        for y in range(free_min_y, free_max_y + 1):
            for x in range(free_min_x, free_max_x + 1):
                # Skip if it's in the inflated obstacle area
                if inner_min_x <= x <= inner_max_x and inner_min_y <= y <= inner_max_y:
                    continue
                
                # Only mark as free if it's currently unknown (-1)
                idx = y * self.width_cells + x
                if self.map_data[idx] == -1:
                    self.map_data[idx] = 0  # Free (0 in Nav2)
                    self.last_cell_update[y, x] = self.current_time  # Update cell timestamp
    
    def transform_point(self, point, transform):
        """Apply a transform to a point with proper rotation handling"""
        transformed_point = Point()
        
        # Extract transform components
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        # Convert geometry_msgs quaternion to scipy rotation
        q = [rotation.x, rotation.y, rotation.z, rotation.w]
        rot = Rotation.from_quat(q)
        
        # Apply rotation to the point
        point_vec = np.array([point.x, point.y, point.z])
        rotated_point = rot.apply(point_vec)
        
        # Apply translation
        transformed_point.x = rotated_point[0] + translation.x
        transformed_point.y = rotated_point[1] + translation.y
        transformed_point.z = rotated_point[2] + translation.z
        
        return transformed_point
    
    def publish_map(self):
        """Publish the current occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = self.map_frame_id
        
        # Set map metadata
        map_msg.info = MapMetaData()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.width_cells
        map_msg.info.height = self.height_cells
        
        # Set the origin (lower-left corner of the map)
        map_msg.info.origin.position.x = -self.map_width / 2
        map_msg.info.origin.position.y = -self.map_height / 2
        map_msg.info.origin.position.z = 0.0
        
        # Set orientation (identity quaternion = no rotation)
        map_msg.info.origin.orientation.w = 1.0
        
        # Set map data
        map_msg.data = self.map_data.tolist()
        
        # Publish the map
        self.map_pub.publish(map_msg)
        
        # Log information occasionally
        now = self.get_clock().now()
        if (now - self.last_update_time).nanoseconds / 1e9 > 5.0:  # Log every 5 seconds of no updates
            occupied_cells = np.count_nonzero(self.map_data == 100)
            inflated_cells = np.count_nonzero(self.map_data == 90)
            unknown_cells = np.count_nonzero(self.map_data == -1)
            free_cells = np.count_nonzero(self.map_data == 0)
            self.get_logger().info(
                f'Map status: {occupied_cells} occupied cells, {inflated_cells} inflated cells, '
                f'{unknown_cells} unknown cells, {free_cells} free cells'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ClusterToNav2MapConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()