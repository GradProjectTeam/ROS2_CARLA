#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import time
from collections import defaultdict
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Header

# Define a function to read points from PointCloud2 without using point_cloud2 module
def read_points_from_pc2(pc2_msg, field_names=None, skip_nans=False):
    """
    Read points from a PointCloud2 message without using sensor_msgs.point_cloud2
    
    Args:
        pc2_msg: The PointCloud2 message
        field_names: The names of fields to read
        skip_nans: Whether to skip NaN values
        
    Returns:
        Generator yielding tuples with field values
    """
    if field_names is None:
        field_names = [field.name for field in pc2_msg.fields]
    
    # Create a mapping from field names to offsets
    field_offsets = {}
    for field in pc2_msg.fields:
        if field.name in field_names:
            field_offsets[field.name] = field.offset
    
    # Calculate point step
    point_step = pc2_msg.point_step
    
    # Get the data as a numpy array
    data = np.frombuffer(pc2_msg.data, dtype=np.uint8).reshape(-1, point_step)
    
    # Process each point
    for i in range(data.shape[0]):
        point_data = {}
        for name in field_names:
            if name in field_offsets:
                offset = field_offsets[name]
                # Extract data based on datatype
                for field in pc2_msg.fields:
                    if field.name == name:
                        # Assuming float32 (datatype 7)
                        if field.datatype == 7:
                            value = np.frombuffer(data[i, offset:offset+4], dtype=np.float32)[0]
                            point_data[name] = value
                        break
        
        # Check for NaN values if requested
        if skip_nans:
            contains_nan = False
            for value in point_data.values():
                if math.isnan(value):
                    contains_nan = True
                    break
            if contains_nan:
                continue
        
        # Yield the point data as a tuple in the same order as field_names
        yield tuple(point_data.get(name, float('nan')) for name in field_names)

class RadarMapGenerator(Node):
    """
    Node that converts radar point cloud data into an occupancy grid map
    suitable for fusion with lidar-based maps.
    """
    
    def __init__(self):
        super().__init__('radar_map_generator')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.2)  # meters per cell
        self.declare_parameter('map_width', 60.0)      # meters
        self.declare_parameter('map_height', 60.0)     # meters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('radar_topic', '/radar/points')
        self.declare_parameter('map_topic', '/radar/map')
        self.declare_parameter('realtime_map_topic', '/realtime_map')  # Added realtime map topic
        self.declare_parameter('publish_to_realtime_map', True)  # Whether to publish to realtime map
        self.declare_parameter('publish_rate', 5.0)    # Hz
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('free_threshold', -0.2)
        self.declare_parameter('use_velocity_filter', True)
        self.declare_parameter('min_velocity', 0.5)    # m/s
        self.declare_parameter('use_temporal_filtering', True)
        self.declare_parameter('cell_memory', 1.5)     # seconds
        self.declare_parameter('enable_fusion_layer', True)
        # QoS parameters
        self.declare_parameter('use_reliable_qos', True)
        self.declare_parameter('use_transient_local_durability', True)
        self.declare_parameter('obstacle_value', 100)  # Value for obstacles in the map
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_frame = self.get_parameter('map_frame').value
        self.radar_topic = self.get_parameter('radar_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.realtime_map_topic = self.get_parameter('realtime_map_topic').value
        self.publish_to_realtime_map = self.get_parameter('publish_to_realtime_map').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.use_velocity_filter = self.get_parameter('use_velocity_filter').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.use_temporal_filtering = self.get_parameter('use_temporal_filtering').value
        self.cell_memory = self.get_parameter('cell_memory').value
        self.enable_fusion_layer = self.get_parameter('enable_fusion_layer').value
        self.obstacle_value = self.get_parameter('obstacle_value').value
        # Get QoS parameters
        self.use_reliable_qos = self.get_parameter('use_reliable_qos').value
        self.use_transient_local_durability = self.get_parameter('use_transient_local_durability').value
        
        # Initialize map dimensions
        self.map_width_cells = int(self.map_width / self.map_resolution)
        self.map_height_cells = int(self.map_height / self.map_resolution)
        self.map_origin_x = -self.map_width / 2.0
        self.map_origin_y = -self.map_height / 2.0
        
        # Initialize grid data
        self.initialize_map()
        
        # For temporal filtering
        self.cell_timestamps = defaultdict(float)
        self.last_cleanup_time = time.time()
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profile for reliable map publishing
        reliability = ReliabilityPolicy.RELIABLE if self.use_reliable_qos else ReliabilityPolicy.BEST_EFFORT
        durability = DurabilityPolicy.TRANSIENT_LOCAL if self.use_transient_local_durability else DurabilityPolicy.VOLATILE
        
        map_qos = QoSProfile(
            reliability=reliability,
            durability=durability,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Set up publishers and subscribers
        self.map_publisher = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        
        # Create realtime map publisher if enabled
        if self.publish_to_realtime_map:
            self.realtime_map_publisher = self.create_publisher(OccupancyGrid, self.realtime_map_topic, map_qos)
        
        self.radar_subscriber = self.create_subscription(
            PointCloud2,
            self.radar_topic,
            self.radar_callback,
            10
        )
        
        # Set up timers
        self.map_timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_old_cells)
        
        self.get_logger().info('Radar Map Generator initialized')
        self.get_logger().info(f'Map dimensions: {self.map_width_cells}x{self.map_height_cells} cells')
        self.get_logger().info(f'Map resolution: {self.map_resolution} meters/cell')
        self.get_logger().info(f'Publishing to radar map topic: {self.map_topic}')
        if self.publish_to_realtime_map:
            self.get_logger().info(f'Publishing to realtime map topic: {self.realtime_map_topic}')
        self.get_logger().info(f'QoS settings - Reliable: {self.use_reliable_qos}, Transient Local: {self.use_transient_local_durability}')
    
    def initialize_map(self):
        """Initialize an empty occupancy grid"""
        # Use numpy array for efficient storage and operations
        self.grid_data = np.zeros((self.map_height_cells, self.map_width_cells), dtype=np.float32)
        
        # Mark all cells as unknown (-1)
        self.grid_data.fill(-1)
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map cell coordinates"""
        cell_x = int((x - self.map_origin_x) / self.map_resolution)
        cell_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Check if within bounds
        if 0 <= cell_x < self.map_width_cells and 0 <= cell_y < self.map_height_cells:
            return cell_x, cell_y
        return None
    
    def radar_callback(self, msg):
        """Process incoming radar point cloud data"""
        try:
            # Get transform from radar frame to map frame
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Log the number of points received
            point_count = 0
            velocity_points = 0
            
            # Create a temporary grid to accumulate evidence
            temp_grid = np.zeros_like(self.grid_data)
            
            # Extract points from PointCloud2 using our custom function
            for point in read_points_from_pc2(msg, field_names=("x", "y", "z", "velocity"), skip_nans=True):
                x, y, z, velocity = point
                point_count += 1
                
                # Apply velocity filter if enabled, but with more lenient threshold for testing
                if self.use_velocity_filter and abs(velocity) < self.min_velocity * 0.8:  # 20% more lenient
                    continue
                
                velocity_points += 1
                
                # Transform point to map frame
                map_x, map_y = self.transform_point(x, y, transform)
                
                # Convert world coordinates to map cell coordinates
                cell = self.world_to_map(map_x, map_y)
                if cell:
                    cell_x, cell_y = cell
                    
                    # Calculate evidence based on velocity (higher velocity = stronger evidence)
                    velocity_weight = min(1.0, abs(velocity) / 3.0)  # More sensitive scaling
                    
                    # Much stronger evidence for radar points to ensure visibility
                    evidence = 0.4 + (0.6 * velocity_weight)  # Significantly increased evidence values
                    
                    # Accumulate evidence in temporary grid
                    temp_grid[cell_y, cell_x] += evidence
                    
                    # Also mark neighboring cells with reduced evidence for better visibility
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            nx, ny = cell_x + dx, cell_y + dy
                            if 0 <= nx < self.map_width_cells and 0 <= ny < self.map_height_cells:
                                # Add reduced evidence to neighboring cells
                                temp_grid[ny, nx] += evidence * 0.3  # 30% evidence for neighbors
            
            # Now update the actual grid with accumulated evidence
            for y in range(self.map_height_cells):
                for x in range(self.map_width_cells):
                    if temp_grid[y, x] > 0:
                        # Get current value
                        current_value = self.grid_data[y, x]
                        
                        # Calculate new value with accumulated evidence
                        new_value = min(1.0, current_value + temp_grid[y, x])
                        
                        # Update grid and timestamp
                        self.grid_data[y, x] = new_value
                        self.cell_timestamps[(x, y)] = time.time()
            
            # Log information about the points processed
            if point_count > 0:
                self.get_logger().info(f'Processed {point_count} radar points, {velocity_points} with sufficient velocity')
            else:
                self.get_logger().warning('No radar points received or all points were filtered out')
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f'TF Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing radar data: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def transform_point(self, x, y, transform):
        """Transform a point from radar frame to map frame"""
        # Extract transform components
        trans_x = transform.transform.translation.x
        trans_y = transform.transform.translation.y
        
        # Extract rotation quaternion
        q = transform.transform.rotation
        # Convert quaternion to yaw (assuming 2D transformation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Apply transformation
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        map_x = cos_yaw * x - sin_yaw * y + trans_x
        map_y = sin_yaw * x + cos_yaw * y + trans_y
        
        return map_x, map_y
    
    def cleanup_old_cells(self):
        """Remove evidence from old detections over time"""
        if not self.use_temporal_filtering:
            return
            
        current_time = time.time()
        
        # Only process if enough time has passed
        if current_time - self.last_cleanup_time < 1.0:
            return
            
        self.last_cleanup_time = current_time
        
        # Decay factor based on time passed
        decay_rate = 0.1  # Adjust to control decay speed
        
        # Cells to remove from tracking (those that have decayed to unknown)
        cells_to_remove = []
        
        for (cell_x, cell_y), timestamp in self.cell_timestamps.items():
            # Calculate time elapsed since last update
            elapsed = current_time - timestamp
            
            # If cell is older than memory window, decay it
            if elapsed > self.cell_memory:
                if 0 <= cell_y < self.map_height_cells and 0 <= cell_x < self.map_width_cells:
                    current_value = self.grid_data[cell_y, cell_x]
                    
                    # Decay the value towards unknown (-1)
                    decay_amount = decay_rate * elapsed / self.cell_memory
                    new_value = max(-1.0, current_value - decay_amount)
                    
                    self.grid_data[cell_y, cell_x] = new_value
                    
                    # If fully decayed, mark for removal from tracking
                    if new_value <= -0.9:
                        cells_to_remove.append((cell_x, cell_y))
        
        # Remove fully decayed cells from tracking
        for cell in cells_to_remove:
            if cell in self.cell_timestamps:
                del self.cell_timestamps[cell]
    
    def publish_map(self):
        """Publish the current occupancy grid map"""
        try:
            # Create occupancy grid message
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = self.map_frame
            
            # Set map metadata
            grid_msg.info.resolution = self.map_resolution
            grid_msg.info.width = self.map_width_cells
            grid_msg.info.height = self.map_height_cells
            grid_msg.info.origin.position.x = self.map_origin_x
            grid_msg.info.origin.position.y = self.map_origin_y
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0
            
            # Convert probability values to occupancy grid format (0-100)
            # Values < free_threshold become 0 (free)
            # Values > obstacle_threshold become 100 (occupied)
            # Values in between are scaled proportionally
            # Unknown cells (-1) remain as -1
            
            # Deep copy the grid data to avoid modifying the original
            grid_data_copy = self.grid_data.copy()
            
            # Apply thresholds and convert to occupancy grid format
            free_mask = grid_data_copy < self.free_threshold
            obstacle_mask = grid_data_copy > self.obstacle_threshold
            unknown_mask = grid_data_copy == -1
            
            # Initialize with scaled values
            occupancy_data = np.round((grid_data_copy - self.free_threshold) / 
                                     (self.obstacle_threshold - self.free_threshold) * 100).astype(np.int8)
            
            # Apply masks
            occupancy_data[free_mask] = 0
            occupancy_data[obstacle_mask] = self.obstacle_value  # Use parameter value
            occupancy_data[unknown_mask] = -1
            
            # Flatten the grid and convert to Python list
            grid_msg.data = occupancy_data.flatten().tolist()
            
            # Publish the map to the radar map topic
            self.map_publisher.publish(grid_msg)
            
            # Also publish to realtime map if enabled
            if self.publish_to_realtime_map:
                # Create a copy of the message for realtime map
                realtime_grid_msg = OccupancyGrid()
                realtime_grid_msg.header = grid_msg.header
                realtime_grid_msg.info = grid_msg.info
                realtime_grid_msg.data = grid_msg.data
                
                # Publish to realtime map topic
                self.realtime_map_publisher.publish(realtime_grid_msg)
                self.get_logger().debug(f'Published radar data to realtime map topic: {self.realtime_map_topic}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing map: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = RadarMapGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 