#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import numpy as np
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from threading import Lock
import time
import threading
from tf2_ros import TransformException

class ThreeSensorFusion(Node):
    """
    Fuses data from LiDAR, radar, and IMU sensors into a single coherent map.
    
    This node:
    1. Subscribes to LiDAR map data
    2. Subscribes to radar map data
    3. Subscribes to IMU orientation data (with compass values in degrees)
    4. Applies intelligent fusion algorithms to combine the data
    5. Publishes a unified map that leverages the strengths of each sensor
    6. Provides visualization for the fused data
    """
    def __init__(self):
        super().__init__('three_sensor_fusion')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.2)
        self.declare_parameter('map_width', 60.0)
        self.declare_parameter('map_height', 60.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_map_topic', '/lidar/map')
        self.declare_parameter('radar_map_topic', '/radar/map')
        self.declare_parameter('radar_points_topic', '/radar/points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('fused_map_topic', '/three_sensor_fused_map')
        self.declare_parameter('publish_rate', 5.0)  # Hz
        self.declare_parameter('lidar_weight', 0.6)
        self.declare_parameter('radar_weight', 0.3)
        self.declare_parameter('imu_weight', 0.1)
        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('max_timestamp_diff', 0.5)  # Maximum allowed difference between timestamps (seconds)
        self.declare_parameter('use_adaptive_weighting', True)  # Use adaptive weights based on data freshness
        self.declare_parameter('enable_debug_output', False)  # Enable debug output
        self.declare_parameter('unknown_cell_value', 50)  # Value for unknown cells in output map
        self.declare_parameter('min_confidence_threshold', 0.3)  # Minimum confidence to consider data valid
        self.declare_parameter('verbose_logging', False)  # Enable verbose logging
        self.declare_parameter('log_topic_info', False)  # Log topic information
        self.declare_parameter('use_imu_for_orientation', True)  # Use IMU data for map orientation
        self.declare_parameter('orientation_correction', True)  # Apply orientation correction based on IMU
        self.declare_parameter('dynamic_obstacle_tracking', True)  # Track dynamic obstacles using sensor fusion
        self.declare_parameter('radar_data_timeout', 2.0)  # Timeout for radar data in seconds
        self.declare_parameter('check_radar_points', True)  # Check if radar points are available
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_frame = self.get_parameter('map_frame').value
        self.lidar_map_topic = self.get_parameter('lidar_map_topic').value
        self.radar_map_topic = self.get_parameter('radar_map_topic').value
        self.radar_points_topic = self.get_parameter('radar_points_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.fused_map_topic = self.get_parameter('fused_map_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.lidar_weight = self.get_parameter('lidar_weight').value
        self.radar_weight = self.get_parameter('radar_weight').value
        self.imu_weight = self.get_parameter('imu_weight').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.max_timestamp_diff = self.get_parameter('max_timestamp_diff').value
        self.use_adaptive_weighting = self.get_parameter('use_adaptive_weighting').value
        self.enable_debug_output = self.get_parameter('enable_debug_output').value
        self.unknown_cell_value = self.get_parameter('unknown_cell_value').value
        self.min_confidence_threshold = self.get_parameter('min_confidence_threshold').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.log_topic_info = self.get_parameter('log_topic_info').value
        self.use_imu_for_orientation = self.get_parameter('use_imu_for_orientation').value
        self.orientation_correction = self.get_parameter('orientation_correction').value
        self.dynamic_obstacle_tracking = self.get_parameter('dynamic_obstacle_tracking').value
        self.radar_data_timeout = self.get_parameter('radar_data_timeout').value
        self.check_radar_points = self.get_parameter('check_radar_points').value
        
        # Create QoS profile for map
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Create QoS profile for sensor data (less strict)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Initialize map data
        self.lidar_map_data = None
        self.radar_map_data = None
        self.lidar_map_info = None
        self.radar_map_info = None
        self.lidar_map_received = False
        self.radar_map_received = False
        
        # Initialize IMU data
        self.imu_data = None
        self.imu_received = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Add timestamp tracking for temporal consistency
        self.lidar_timestamp = None
        self.radar_timestamp = None
        self.imu_timestamp = None
        self.last_fusion_time = None
        
        # Performance metrics
        self.fusion_count = 0
        self.last_stats_time = time.time()
        
        # Create locks for thread safety
        self.lidar_lock = threading.Lock()
        self.radar_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        
        # Log topic information if enabled
        if self.log_topic_info:
            self.get_logger().info(f"Subscribing to LiDAR map topic: {self.lidar_map_topic}")
            self.get_logger().info(f"Subscribing to radar map topic: {self.radar_map_topic}")
            self.get_logger().info(f"Subscribing to radar points topic: {self.radar_points_topic}")
            self.get_logger().info(f"Subscribing to IMU topic: {self.imu_topic}")
            self.get_logger().info(f"Publishing fused map to topic: {self.fused_map_topic}")
        
        # Create subscribers for LiDAR, Radar maps and IMU data
        self.lidar_map_sub = self.create_subscription(
            OccupancyGrid,
            self.lidar_map_topic,
            self.lidar_map_callback,
            map_qos
        )
        
        self.radar_map_sub = self.create_subscription(
            OccupancyGrid,
            self.radar_map_topic,
            self.radar_map_callback,
            map_qos
        )
        
        # Add subscription to radar points for additional validation
        if self.check_radar_points:
            self.radar_points_sub = self.create_subscription(
                PointCloud2,
                self.radar_points_topic,
                self.radar_points_callback,
                sensor_qos
            )
            self.radar_points_received = False
            self.radar_points_timestamp = None
        
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )
        
        # Create publisher for fused map
        self.fused_map_pub = self.create_publisher(
            OccupancyGrid,
            self.fused_map_topic,
            map_qos
        )
        
        # Debug publisher for fusion weights
        if self.enable_debug_output:
            self.debug_pub = self.create_publisher(
                OccupancyGrid,
                '/fusion_debug',
                map_qos
            )
        
        # Create timer for publishing fused map
        self.fusion_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_fused_map
        )
        
        # Create timer for performance reporting
        self.stats_timer = self.create_timer(
            10.0,  # Report every 10 seconds
            self.report_stats
        )
        
        # Create timer for checking topic status
        if self.log_topic_info:
            self.topic_check_timer = self.create_timer(
                5.0,  # Check every 5 seconds
                self.check_topic_status
            )
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Three Sensor Fusion node initialized')
        self.get_logger().info(f'Using adaptive weighting: {self.use_adaptive_weighting}')
        self.get_logger().info(f'Max timestamp difference: {self.max_timestamp_diff} seconds')
        self.get_logger().info('IMU data now includes compass values in degrees')
    
    def check_topic_status(self):
        """Check and log the status of subscribed topics"""
        if self.lidar_map_received:
            timestamp_ms = self.lidar_timestamp.nanosec // 1000000
            self.get_logger().info(f"LiDAR map data received, timestamp: {timestamp_ms/1000.0}")
        else:
            self.get_logger().warn(f"No LiDAR map data received on topic: {self.lidar_map_topic}")
            
        if self.radar_map_received:
            timestamp_ms = self.radar_timestamp.nanosec // 1000000
            self.get_logger().info(f"Radar map data received, timestamp: {timestamp_ms/1000.0}")
        else:
            self.get_logger().warn(f"No radar map data received on topic: {self.radar_map_topic}")
            
        if self.check_radar_points:
            if self.radar_points_received:
                timestamp_ms = self.radar_points_timestamp.nanosec // 1000000
                self.get_logger().info(f"Radar points data received, timestamp: {timestamp_ms/1000.0}")
            else:
                self.get_logger().warn(f"No radar points data received on topic: {self.radar_points_topic}")
            
        if self.imu_received:
            timestamp_ms = self.imu_timestamp.nanosec // 1000000
            self.get_logger().info(f"IMU data received, timestamp: {timestamp_ms/1000.0}")
        else:
            self.get_logger().warn(f"No IMU data received on topic: {self.imu_topic}")
    
    def report_stats(self):
        """Report performance statistics"""
        now = time.time()
        elapsed = now - self.last_stats_time
        if elapsed > 0 and self.fusion_count > 0:
            rate = self.fusion_count / elapsed
            self.get_logger().info(f'Fusion rate: {rate:.2f} Hz, total fusions: {self.fusion_count}')
            
            # Reset counters
            self.fusion_count = 0
            self.last_stats_time = now
    
    def lidar_map_callback(self, msg):
        with self.lidar_lock:
            # Convert 1D array to 2D grid
            width = msg.info.width
            height = msg.info.height
            self.lidar_map_data = np.array(msg.data).reshape(height, width)
            self.lidar_map_info = msg.info
            self.lidar_timestamp = msg.header.stamp
            self.lidar_map_received = True
            if self.verbose_logging:
                timestamp_ms = self.lidar_timestamp.nanosec // 1000000
                self.get_logger().info(f'Received LiDAR map, timestamp: {timestamp_ms/1000.0}, size: {width}x{height}')
            
    def radar_map_callback(self, msg):
        with self.radar_lock:
            # Convert 1D array to 2D grid
            width = msg.info.width
            height = msg.info.height
            self.radar_map_data = np.array(msg.data).reshape(height, width)
            self.radar_map_info = msg.info
            self.radar_timestamp = msg.header.stamp
            self.radar_map_received = True
            if self.verbose_logging:
                timestamp_ms = self.radar_timestamp.nanosec // 1000000
                self.get_logger().info(f'Received radar map, timestamp: {timestamp_ms/1000.0}, size: {width}x{height}')
    
    def imu_callback(self, msg):
        """
        Process incoming IMU data
        
        The orientation quaternion in the IMU message is created from compass values 
        that are in degrees but have been converted to radians for the quaternion.
        """
        with self.imu_lock:
            self.imu_data = msg
            self.imu_timestamp = msg.header.stamp
            self.imu_received = True
            
            # Extract orientation from quaternion
            q = msg.orientation
            self.roll, self.pitch, self.yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Log orientation in degrees for better readability
            if self.verbose_logging:
                timestamp_ms = self.imu_timestamp.nanosec // 1000000
                roll_deg = math.degrees(self.roll)
                pitch_deg = math.degrees(self.pitch)
                yaw_deg = math.degrees(self.yaw)
                self.get_logger().info(f'Received IMU data, timestamp: {timestamp_ms/1000.0}, '
                                      f'roll: {roll_deg:.2f}°, '
                                      f'pitch: {pitch_deg:.2f}°, '
                                      f'yaw: {yaw_deg:.2f}°')
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        
        This function extracts Euler angles from a quaternion. The quaternion is 
        created from compass values that were originally in degrees but converted 
        to radians for the quaternion. The output of this function is in radians.
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def calculate_timestamp_difference(self):
        """Calculate the maximum difference between sensor timestamps in seconds"""
        if self.lidar_timestamp is None or self.radar_timestamp is None or self.imu_timestamp is None:
            return None
            
        # Convert to seconds
        lidar_time = float(self.lidar_timestamp.sec) + float(self.lidar_timestamp.nanosec) / 1e9
        radar_time = float(self.radar_timestamp.sec) + float(self.radar_timestamp.nanosec) / 1e9
        imu_time = float(self.imu_timestamp.sec) + float(self.imu_timestamp.nanosec) / 1e9
        
        # Find maximum difference
        times = [lidar_time, radar_time, imu_time]
        max_diff = max(times) - min(times)
        
        return max_diff
    
    def calculate_adaptive_weights(self):
        """Calculate adaptive weights based on data freshness"""
        if not self.use_adaptive_weighting:
            return self.lidar_weight, self.radar_weight, self.imu_weight
            
        try:
            # Get current time
            now = self.get_clock().now().to_msg()
            
            # Calculate age of each sensor data in seconds
            if self.lidar_timestamp is None or self.radar_timestamp is None or self.imu_timestamp is None:
                return self.lidar_weight, self.radar_weight, self.imu_weight
                
            # Convert timestamps to seconds
            now_sec = float(now.sec) + float(now.nanosec) / 1e9
            lidar_sec = float(self.lidar_timestamp.sec) + float(self.lidar_timestamp.nanosec) / 1e9
            radar_sec = float(self.radar_timestamp.sec) + float(self.radar_timestamp.nanosec) / 1e9
            imu_sec = float(self.imu_timestamp.sec) + float(self.imu_timestamp.nanosec) / 1e9
            
            # Calculate age
            lidar_age = max(0.0, now_sec - lidar_sec)
            radar_age = max(0.0, now_sec - radar_sec)
            imu_age = max(0.0, now_sec - imu_sec)
            
            # Calculate freshness (inverse of age)
            lidar_freshness = 1.0 / (1.0 + lidar_age)
            radar_freshness = 1.0 / (1.0 + radar_age)
            imu_freshness = 1.0 / (1.0 + imu_age)
            
            # Calculate total freshness
            total_freshness = lidar_freshness + radar_freshness + imu_freshness
            
            if total_freshness > 0:
                # Calculate weights based on freshness
                lidar_weight = (lidar_freshness / total_freshness) * (self.lidar_weight + self.radar_weight + self.imu_weight)
                radar_weight = (radar_freshness / total_freshness) * (self.lidar_weight + self.radar_weight + self.imu_weight)
                imu_weight = (imu_freshness / total_freshness) * (self.lidar_weight + self.radar_weight + self.imu_weight)
                
                if self.verbose_logging:
                    self.get_logger().info(f'Adaptive weights: LiDAR={lidar_weight:.2f}, Radar={radar_weight:.2f}, IMU={imu_weight:.2f}')
                
                return lidar_weight, radar_weight, imu_weight
            else:
                return self.lidar_weight, self.radar_weight, self.imu_weight
                
        except Exception as e:
            self.get_logger().error(f'Error calculating adaptive weights: {str(e)}')
            return self.lidar_weight, self.radar_weight, self.imu_weight
    
    def apply_orientation_correction(self, map_data):
        """
        Apply orientation correction based on IMU data
        
        The yaw value is already in radians (converted from the quaternion). 
        The quaternion was originally created from compass values in degrees
        that were converted to radians.
        """
        if not self.orientation_correction or not self.imu_received:
            return map_data
            
        try:
            # Get map dimensions
            height, width = map_data.shape
            
            # Create rotation matrix from IMU orientation
            # This is a simplified rotation that only considers yaw (heading)
            cos_yaw = math.cos(-self.yaw)  # Negative yaw because we're rotating the map, not the vehicle
            sin_yaw = math.sin(-self.yaw)
            
            # Create rotated map
            rotated_map = np.ones((height, width), dtype=np.int8) * self.unknown_cell_value
            
            # Center of rotation (center of map)
            cx = width // 2
            cy = height // 2
            
            # Apply rotation to each cell
            for y in range(height):
                for x in range(width):
                    # Translate to origin
                    tx = x - cx
                    ty = y - cy
                    
                    # Rotate
                    rx = int(tx * cos_yaw - ty * sin_yaw + cx)
                    ry = int(tx * sin_yaw + ty * cos_yaw + cy)
                    
                    # Check if rotated coordinates are within bounds
                    if 0 <= rx < width and 0 <= ry < height:
                        rotated_map[y, x] = map_data[ry, rx]
            
            return rotated_map
            
        except Exception as e:
            self.get_logger().error(f'Error applying orientation correction: {str(e)}')
            return map_data
    
    def publish_fused_map(self):
        """Publish the fused map"""
        # Check if we have received data from all required sensors
        if not self.lidar_map_received:
            self.get_logger().warn('Waiting for LiDAR map data...')
            return
        
        # For radar, check either map or points depending on configuration
        radar_data_valid = self.radar_map_received
        if self.check_radar_points:
            radar_data_valid = radar_data_valid or self.radar_points_received
            
        if not radar_data_valid:
            self.get_logger().warn('Waiting for radar data...')
            return
            
        if not self.imu_received:
            self.get_logger().warn('Waiting for IMU data...')
            return
            
        # Check for data timeout
        current_time = self.get_clock().now()
        
        # Check radar data timeout if we're using radar data
        if self.radar_map_received:
            radar_time = self.radar_timestamp
            current_sec = current_time.seconds_nanoseconds()
            radar_sec = radar_time.sec
            radar_nsec = radar_time.nanosec
            radar_age = (current_sec[0] - radar_sec) + (current_sec[1] - radar_nsec) / 1e9
            if radar_age > self.radar_data_timeout:
                self.get_logger().warn(f'Radar map data is too old: {radar_age:.2f} seconds')
                return
        
        # Check if timestamps are close enough
        timestamp_diff = self.calculate_timestamp_difference()
        if timestamp_diff is not None and timestamp_diff > self.max_timestamp_diff:
            self.get_logger().warn(f'Timestamp difference too large: {timestamp_diff:.2f} seconds')
            return
            
        try:
            # Calculate adaptive weights
            lidar_weight, radar_weight, imu_weight = self.calculate_adaptive_weights()
            
            # Get map dimensions and info from LiDAR map (primary source)
            with self.lidar_lock:
                if self.lidar_map_data is None or self.lidar_map_info is None:
                    return
                
                lidar_map = self.lidar_map_data.copy()
                map_info = self.lidar_map_info
                
            # Get radar map data
            with self.radar_lock:
                if self.radar_map_data is None:
                    return
                
                radar_map = self.radar_map_data.copy()
                
            # Get IMU data
            with self.imu_lock:
                if self.imu_data is None:
                    return
                
                # We already extracted roll, pitch, yaw in the imu_callback
            
            # Apply orientation correction if enabled
            if self.orientation_correction:
                lidar_map = self.apply_orientation_correction(lidar_map)
                radar_map = self.apply_orientation_correction(radar_map)
            
            # Ensure maps have the same dimensions
            if lidar_map.shape != radar_map.shape:
                self.get_logger().error(f'Map dimensions do not match: LiDAR {lidar_map.shape}, Radar {radar_map.shape}')
                return
                
            # Create fused map
            fused_map = np.zeros_like(lidar_map)
            
            # Apply fusion algorithm
            for y in range(lidar_map.shape[0]):
                for x in range(lidar_map.shape[1]):
                    lidar_value = lidar_map[y, x]
                    radar_value = radar_map[y, x]
                    
                    # Skip unknown cells (-1 or 255 in OccupancyGrid)
                    if lidar_value == -1 or lidar_value == 255:
                        lidar_value = self.unknown_cell_value
                    if radar_value == -1 or radar_value == 255:
                        radar_value = self.unknown_cell_value
                    
                    # Apply weighted fusion
                    fused_value = (lidar_value * lidar_weight + 
                                  radar_value * radar_weight)
                    
                    # Normalize by the weights used
                    total_weight = lidar_weight + radar_weight
                    if total_weight > 0:
                        fused_value /= total_weight
                    
                    # Round to integer and ensure it's in the valid range [0, 100]
                    fused_map[y, x] = max(0, min(100, int(round(fused_value))))
            
            # Create OccupancyGrid message
            fused_map_msg = OccupancyGrid()
            fused_map_msg.header.stamp = self.get_clock().now().to_msg()
            fused_map_msg.header.frame_id = self.map_frame
            fused_map_msg.info = map_info
            
            # Convert 2D grid to 1D array
            fused_map_msg.data = fused_map.flatten().tolist()
            
            # Publish fused map
            self.fused_map_pub.publish(fused_map_msg)
            
            # Update fusion count
            self.fusion_count += 1
            self.last_fusion_time = time.time()
            
            # Publish debug info if enabled
            if self.enable_debug_output:
                self.publish_debug_info(lidar_weight, radar_weight, imu_weight)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing fused map: {str(e)}')
    
    def publish_debug_info(self, lidar_weight, radar_weight, imu_weight):
        """Publish debug information"""
        try:
            # Create debug message
            debug_msg = OccupancyGrid()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = self.map_frame
            
            with self.lidar_lock:
                if self.lidar_map_info is None:
                    return
                debug_msg.info = self.lidar_map_info
            
            # Create debug data
            height = debug_msg.info.height
            width = debug_msg.info.width
            debug_data = np.zeros((height, width), dtype=np.int8)
            
            # Fill debug data with weights
            # We'll use the center third of the map for each weight
            h_third = height // 3
            
            # LiDAR weight (top third)
            debug_data[0:h_third, :] = int(lidar_weight * 100)
            
            # Radar weight (middle third)
            debug_data[h_third:2*h_third, :] = int(radar_weight * 100)
            
            # IMU weight (bottom third)
            debug_data[2*h_third:, :] = int(imu_weight * 100)
            
            # Convert to 1D array
            debug_msg.data = debug_data.flatten().tolist()
            
            # Publish debug info
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing debug info: {str(e)}')

    def radar_points_callback(self, msg):
        """Callback for radar points data"""
        self.radar_points_timestamp = msg.header.stamp
        self.radar_points_received = True
        if self.verbose_logging:
            timestamp_ms = self.radar_points_timestamp.nanosec // 1000000
            self.get_logger().info(f'Received radar points, timestamp: {timestamp_ms/1000.0}')

def main(args=None):
    rclpy.init(args=args)
    node = ThreeSensorFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 