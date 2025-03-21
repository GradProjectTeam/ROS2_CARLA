#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
import numpy as np
import math
from threading import Lock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import time
from std_msgs.msg import String

class SensorFusionCostmap(Node):
    """
    Fuses LiDAR and Radar costmaps for navigation with Nav2.
    
    Data Flow:
    - Receives radar costmap from radar_costmap_creator (originally from enhanced_radar_visualizer)
    - Receives lidar costmap from lidar_costmap_creator (originally from lidar_listener_clusters)
    - Applies intelligent fusion based on object dynamics
    - Publishes fused costmap for Nav2 integration
    
    Features:
    - Intelligent fusion that uses radar for dynamic objects, lidar for static objects
    - Automatic adjustment based on IMU data
    - Nav2 costmap_2d integration
    - Support for multiple navigation sessions
    """
    def __init__(self):
        super().__init__('sensor_fusion_costmap')
        
        # Declare parameters with defaults
        self.declare_parameter('map_resolution', 0.1)       # meters per cell
        self.declare_parameter('map_width', 1000)           # cells
        self.declare_parameter('map_height', 1000)          # cells
        self.declare_parameter('map_origin_x', -50.0)       # meters
        self.declare_parameter('map_origin_y', -50.0)       # meters
        self.declare_parameter('publish_rate', 5.0)         # Hz
        self.declare_parameter('lidar_weight_static', 0.8)  # Weight for lidar data on static objects
        self.declare_parameter('radar_weight_static', 0.2)  # Weight for radar data on static objects
        self.declare_parameter('lidar_weight_dynamic', 0.3) # Weight for lidar data on dynamic objects
        self.declare_parameter('radar_weight_dynamic', 0.7) # Weight for radar data on dynamic objects
        self.declare_parameter('dynamic_threshold', 60)     # Cost threshold to consider object dynamic
        self.declare_parameter('max_data_age', 1.0)         # Maximum age of data in seconds
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.lidar_weight_static = self.get_parameter('lidar_weight_static').value
        self.radar_weight_static = self.get_parameter('radar_weight_static').value
        self.lidar_weight_dynamic = self.get_parameter('lidar_weight_dynamic').value
        self.radar_weight_dynamic = self.get_parameter('radar_weight_dynamic').value
        self.dynamic_threshold = self.get_parameter('dynamic_threshold').value
        self.max_data_age = self.get_parameter('max_data_age').value
        
        # Initialize costmap layers
        self.lidar_costmap = None
        self.radar_costmap = None
        self.fused_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Timestamp tracking
        self.lidar_costmap_timestamp = None
        self.radar_costmap_timestamp = None
        self.imu_data_timestamp = None
        
        # IMU data
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Thread safety
        self.costmap_lock = Lock()
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create reliable QoS profile for costmap publishing
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.fused_costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/fused_costmap', 
            reliable_qos
        )
        
        # For Nav2 integration - this topic can be added to Nav2's costmap layers
        self.nav2_costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/global_costmap/fused_sensor_layer', 
            reliable_qos
        )
        
        # Status publisher
        self.status_publisher = self.create_publisher(
            String,
            '/fusion_costmap/status',
            10
        )
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/radar_costmap',  # From radar_costmap_creator
            self.radar_costmap_callback,
            10
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/lidar_costmap',  # From lidar_costmap_creator
            self.lidar_costmap_callback,
            10
        )
        
        self.create_subscription(
            Imu,
            '/imu',  # IMU data
            self.imu_callback,
            10
        )
        
        # Timer for costmap fusion and publishing
        self.create_timer(1.0/self.publish_rate, self.fuse_and_publish_costmap)
        
        # Timer for status updates
        self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('Sensor Fusion Costmap node initialized')
        self.get_logger().info('Subscribed to radar costmap (from enhanced_radar_visualizer via radar_costmap_creator)')
        self.get_logger().info('Subscribed to lidar costmap (from lidar_listener_clusters via lidar_costmap_creator)')
    
    def radar_costmap_callback(self, msg):
        """Process radar costmap data (originally from enhanced_radar_visualizer)"""
        with self.costmap_lock:
            # Check if dimensions match
            if msg.info.width != self.map_width or msg.info.height != self.map_height:
                self.get_logger().warn(
                    f'Radar costmap dimensions ({msg.info.width}x{msg.info.height}) '
                    f'do not match expected dimensions ({self.map_width}x{self.map_height})'
                )
                return
            
            # Convert 1D list to 2D numpy array
            self.radar_costmap = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width
            )
            
            # Update timestamp
            self.radar_costmap_timestamp = self.get_clock().now()
            
            # Log received data
            occupied_cells = np.count_nonzero(self.radar_costmap > 0)
            self.get_logger().debug(f'Received radar costmap with {occupied_cells} occupied cells')
    
    def lidar_costmap_callback(self, msg):
        """Process lidar costmap data (originally from lidar_listener_clusters)"""
        with self.costmap_lock:
            # Check if dimensions match
            if msg.info.width != self.map_width or msg.info.height != self.map_height:
                self.get_logger().warn(
                    f'LiDAR costmap dimensions ({msg.info.width}x{msg.info.height}) '
                    f'do not match expected dimensions ({self.map_width}x{self.map_height})'
                )
                return
            
            # Convert 1D list to 2D numpy array
            self.lidar_costmap = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width
            )
            
            # Update timestamp
            self.lidar_costmap_timestamp = self.get_clock().now()
            
            # Log received data
            occupied_cells = np.count_nonzero(self.lidar_costmap > 0)
            self.get_logger().debug(f'Received lidar costmap with {occupied_cells} occupied cells')
    
    def imu_callback(self, msg):
        """Process IMU data for costmap orientation correction"""
        # Extract roll, pitch, yaw from quaternion
        # Simple conversion - for more accuracy, use tf2's quaternion utilities
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        
        # Convert quaternion to Euler angles
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update timestamp
        self.imu_data_timestamp = self.get_clock().now()
    
    def apply_imu_correction(self, costmap):
        """Apply IMU-based correction to costmap"""
        if costmap is None or self.imu_data_timestamp is None:
            return costmap
        
        # Skip if IMU data is too old
        current_time = self.get_clock().now()
        if (current_time - self.imu_data_timestamp).nanoseconds / 1e9 > self.max_data_age:
            return costmap
        
        # Apply correction based on pitch and roll
        # This is a simplified correction - for more accuracy, use a proper transformation
        
        # Skip small angles (optimization)
        if abs(self.pitch) < 0.05 and abs(self.roll) < 0.05:
            return costmap
        
        # Create a copy to avoid modifying the original
        corrected_costmap = np.copy(costmap)
        
        # Apply pitch correction (shift forward/backward)
        if abs(self.pitch) > 0.05:  # Only apply if pitch is significant
            shift_y = int(math.tan(self.pitch) * 10 / self.map_resolution)  # 10m distance approximation
            if shift_y != 0:
                if shift_y > 0:
                    corrected_costmap[shift_y:, :] = costmap[:-shift_y, :]
                    corrected_costmap[:shift_y, :] = 0
                else:
                    shift_y = abs(shift_y)
                    corrected_costmap[:-shift_y, :] = costmap[shift_y:, :]
                    corrected_costmap[-shift_y:, :] = 0
        
        # Apply roll correction (shift left/right)
        if abs(self.roll) > 0.05:  # Only apply if roll is significant
            shift_x = int(math.tan(self.roll) * 10 / self.map_resolution)  # 10m distance approximation
            if shift_x != 0:
                if shift_x > 0:
                    corrected_costmap[:, shift_x:] = corrected_costmap[:, :-shift_x]
                    corrected_costmap[:, :shift_x] = 0
                else:
                    shift_x = abs(shift_x)
                    corrected_costmap[:, :-shift_x] = corrected_costmap[:, shift_x:]
                    corrected_costmap[:, -shift_x:] = 0
        
        return corrected_costmap
    
    def is_data_fresh(self):
        """Check if costmap data is fresh enough to use"""
        if self.radar_costmap is None or self.lidar_costmap is None:
            return False
        
        current_time = self.get_clock().now()
        
        # Check radar data age
        if self.radar_costmap_timestamp is None:
            return False
        radar_age = (current_time - self.radar_costmap_timestamp).nanoseconds / 1e9
        if radar_age > self.max_data_age:
            self.get_logger().warn(f'Radar data too old: {radar_age:.2f}s')
            return False
        
        # Check lidar data age
        if self.lidar_costmap_timestamp is None:
            return False
        lidar_age = (current_time - self.lidar_costmap_timestamp).nanoseconds / 1e9
        if lidar_age > self.max_data_age:
            self.get_logger().warn(f'LiDAR data too old: {lidar_age:.2f}s')
            return False
        
        return True
    
    def fuse_costmaps(self):
        """Fuse radar and lidar costmaps using intelligent weighting"""
        if not self.is_data_fresh():
            return
        
        with self.costmap_lock:
            # Apply IMU correction if available
            corrected_radar = self.apply_imu_correction(self.radar_costmap)
            corrected_lidar = self.apply_imu_correction(self.lidar_costmap)
            
            # Initialize fused costmap
            self.fused_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
            
            # Identify dynamic areas from radar data
            dynamic_mask = corrected_radar > self.dynamic_threshold
            static_mask = ~dynamic_mask
            
            # Count dynamic and static cells
            dynamic_count = np.count_nonzero(dynamic_mask)
            static_count = np.count_nonzero(static_mask)
            self.get_logger().debug(f'Fusion: {dynamic_count} dynamic cells, {static_count} static cells')
            
            # Fuse with different weights for static and dynamic areas
            # For dynamic areas: favor radar data more
            self.fused_costmap[dynamic_mask] = np.clip(
                corrected_radar[dynamic_mask] * self.radar_weight_dynamic +
                corrected_lidar[dynamic_mask] * self.lidar_weight_dynamic,
                0, 100
            ).astype(np.int8)
            
            # For static areas: favor lidar data more
            self.fused_costmap[static_mask] = np.clip(
                corrected_radar[static_mask] * self.radar_weight_static +
                corrected_lidar[static_mask] * self.lidar_weight_static,
                0, 100
            ).astype(np.int8)
            
            # For unknown cells (-1) in either costmap, use the other costmap's value
            radar_unknown = corrected_radar == -1
            lidar_unknown = corrected_lidar == -1
            both_known = ~(radar_unknown | lidar_unknown)
            
            # If radar is unknown but lidar is known
            self.fused_costmap[radar_unknown & ~lidar_unknown] = corrected_lidar[radar_unknown & ~lidar_unknown]
            
            # If lidar is unknown but radar is known
            self.fused_costmap[lidar_unknown & ~radar_unknown] = corrected_radar[lidar_unknown & ~radar_unknown]
            
            # If both are unknown, mark as unknown
            self.fused_costmap[radar_unknown & lidar_unknown] = -1
    
    def fuse_and_publish_costmap(self):
        """Fuse costmaps and publish result"""
        # Fuse costmaps
        self.fuse_costmaps()
        
        # Create occupancy grid message
        fused_costmap_msg = OccupancyGrid()
        fused_costmap_msg.header.frame_id = 'map'
        fused_costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        fused_costmap_msg.info.resolution = self.map_resolution
        fused_costmap_msg.info.width = self.map_width
        fused_costmap_msg.info.height = self.map_height
        fused_costmap_msg.info.origin.position.x = self.map_origin_x
        fused_costmap_msg.info.origin.position.y = self.map_origin_y
        fused_costmap_msg.info.origin.position.z = 0.0
        fused_costmap_msg.info.origin.orientation.w = 1.0
        
        # Convert numpy array to 1D list
        fused_costmap_msg.data = self.fused_costmap.flatten().tolist()
        
        # Publish fused costmap
        self.fused_costmap_publisher.publish(fused_costmap_msg)
        
        # Publish to Nav2 costmap topic
        self.nav2_costmap_publisher.publish(fused_costmap_msg)
        
        # Log info occasionally
        occupied_cells = np.count_nonzero(self.fused_costmap > 0)
        if hasattr(self, 'publish_count'):
            self.publish_count += 1
            if self.publish_count >= 20:  # Log every ~4 seconds at 5Hz
                self.publish_count = 0
                self.get_logger().info(f'Published fused costmap with {occupied_cells} occupied cells')
        else:
            self.publish_count = 0
    
    def publish_status(self):
        """Publish status information"""
        status_msg = String()
        
        # Build status message
        current_time = self.get_clock().now()
        
        # Calculate data ages
        radar_age = "N/A"
        lidar_age = "N/A"
        imu_age = "N/A"
        
        if self.radar_costmap_timestamp:
            radar_age = f"{(current_time - self.radar_costmap_timestamp).nanoseconds / 1e9:.2f}s"
        
        if self.lidar_costmap_timestamp:
            lidar_age = f"{(current_time - self.lidar_costmap_timestamp).nanoseconds / 1e9:.2f}s"
        
        if self.imu_data_timestamp:
            imu_age = f"{(current_time - self.imu_data_timestamp).nanoseconds / 1e9:.2f}s"
        
        # Count cells in different categories
        dynamic_count = 0
        static_count = 0
        unknown_count = 0
        
        if self.fused_costmap is not None:
            dynamic_count = np.count_nonzero(self.fused_costmap > self.dynamic_threshold)
            static_count = np.count_nonzero((self.fused_costmap > 0) & (self.fused_costmap <= self.dynamic_threshold))
            unknown_count = np.count_nonzero(self.fused_costmap == -1)
        
        status = (
            f"Sensor Fusion Costmap Status:\n"
            f"Radar costmap age: {radar_age}\n"
            f"LiDAR costmap age: {lidar_age}\n"
            f"IMU data age: {imu_age}\n"
            f"IMU orientation: Roll={self.roll:.2f}, Pitch={self.pitch:.2f}, Yaw={self.yaw:.2f}\n"
            f"Dynamic cells: {dynamic_count}\n"
            f"Static cells: {static_count}\n"
            f"Unknown cells: {unknown_count}\n"
            f"Data source: enhanced_radar_visualizer + lidar_listener_clusters\n"
        )
        
        status_msg.data = status
        self.status_publisher.publish(status_msg)
        
        # Log status every 30 seconds
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    
    node = SensorFusionCostmap()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 