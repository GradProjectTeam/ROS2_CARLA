#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from threading import Lock
import time
import threading
from tf2_ros import TransformException


class FusionCostmapCreator(Node):
    """
    Fuses LiDAR and radar costmaps into a single coherent costmap for navigation.
    
    This node subscribes to costmaps from both sensors, applies intelligent fusion
    strategies, and produces a unified map that leverages the strengths of each sensor.
    """
    def __init__(self):
        super().__init__('fusion_costmap_creator')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.2)
        self.declare_parameter('map_width', 60.0)
        self.declare_parameter('map_height', 60.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lidar_map_topic', '/lidar/map')
        self.declare_parameter('radar_map_topic', '/radar/map')
        self.declare_parameter('fused_map_topic', '/fused_map')
        self.declare_parameter('publish_rate', 2.0)  # Hz
        self.declare_parameter('lidar_weight', 0.7)
        self.declare_parameter('radar_weight', 0.3)
        self.declare_parameter('obstacle_threshold', 50)
        # New parameters for improved fusion
        self.declare_parameter('max_timestamp_diff', 0.5)  # Maximum allowed difference between timestamps (seconds)
        self.declare_parameter('use_adaptive_weighting', True)  # Use adaptive weights based on data freshness
        self.declare_parameter('enable_debug_output', False)  # Enable debug output
        self.declare_parameter('unknown_cell_value', 50)  # Value for unknown cells in output map
        self.declare_parameter('min_confidence_threshold', 0.3)  # Minimum confidence to consider data valid
        # Logging parameters
        self.declare_parameter('verbose_logging', False)  # Enable verbose logging
        self.declare_parameter('log_topic_info', False)  # Log topic information
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_frame = self.get_parameter('map_frame').value
        self.lidar_map_topic = self.get_parameter('lidar_map_topic').value
        self.radar_map_topic = self.get_parameter('radar_map_topic').value
        self.fused_map_topic = self.get_parameter('fused_map_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.lidar_weight = self.get_parameter('lidar_weight').value
        self.radar_weight = self.get_parameter('radar_weight').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        # Get new parameters
        self.max_timestamp_diff = self.get_parameter('max_timestamp_diff').value
        self.use_adaptive_weighting = self.get_parameter('use_adaptive_weighting').value
        self.enable_debug_output = self.get_parameter('enable_debug_output').value
        self.unknown_cell_value = self.get_parameter('unknown_cell_value').value
        self.min_confidence_threshold = self.get_parameter('min_confidence_threshold').value
        # Get logging parameters
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.log_topic_info = self.get_parameter('log_topic_info').value
        
        # Create QoS profile for map
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Initialize map data
        self.lidar_map_data = None
        self.radar_map_data = None
        self.lidar_map_info = None
        self.radar_map_info = None
        self.lidar_map_received = False
        self.radar_map_received = False
        
        # Add timestamp tracking for temporal consistency
        self.lidar_timestamp = None
        self.radar_timestamp = None
        self.last_fusion_time = None
        
        # Performance metrics
        self.fusion_count = 0
        self.last_stats_time = time.time()
        
        # Create locks for thread safety
        self.lidar_lock = threading.Lock()
        self.radar_lock = threading.Lock()
        
        # Log topic information if enabled
        if self.log_topic_info:
            self.get_logger().info(f"Subscribing to LiDAR map topic: {self.lidar_map_topic}")
            self.get_logger().info(f"Subscribing to radar map topic: {self.radar_map_topic}")
            self.get_logger().info(f"Publishing fused map to topic: {self.fused_map_topic}")
        
        # Create subscribers for LiDAR and Radar maps
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
        
        self.get_logger().info('Fusion Costmap Creator node initialized')
        self.get_logger().info(f'Using adaptive weighting: {self.use_adaptive_weighting}')
        self.get_logger().info(f'Max timestamp difference: {self.max_timestamp_diff} seconds')
    
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
    
    def calculate_timestamp_difference(self):
        """Calculate the difference between LiDAR and radar timestamps in seconds"""
        if self.lidar_timestamp is None or self.radar_timestamp is None:
            return None
            
        # Convert to seconds
        lidar_time = float(self.lidar_timestamp.sec) + float(self.lidar_timestamp.nanosec) / 1e9
        radar_time = float(self.radar_timestamp.sec) + float(self.radar_timestamp.nanosec) / 1e9
        
        return abs(lidar_time - radar_time)
    
    def calculate_adaptive_weights(self):
        """Calculate adaptive weights based on data freshness"""
        if not self.use_adaptive_weighting:
            return self.lidar_weight, self.radar_weight
            
        try:
            # Get current time
            now = self.get_clock().now().to_msg()
            
            # Calculate age of each map in seconds
            if self.lidar_timestamp is None or self.radar_timestamp is None:
                return self.lidar_weight, self.radar_weight
                
            # Convert timestamps to seconds
            now_sec = float(now.sec) + float(now.nanosec) / 1e9
            lidar_sec = float(self.lidar_timestamp.sec) + float(self.lidar_timestamp.nanosec) / 1e9
            radar_sec = float(self.radar_timestamp.sec) + float(self.radar_timestamp.nanosec) / 1e9
            
            # Calculate age
            lidar_age = now_sec - lidar_sec
            radar_age = now_sec - radar_sec
            
            # Ensure ages are positive
            lidar_age = max(0.0, lidar_age)
            radar_age = max(0.0, radar_age)
            
            # Normalize ages (newer data gets higher weight)
            max_age = max(lidar_age, radar_age)
            if max_age <= 0.001:  # Avoid division by zero or very small values
                return self.lidar_weight, self.radar_weight
                
            lidar_freshness = 1.0 - (lidar_age / max_age) if max_age > 0 else 0.5
            radar_freshness = 1.0 - (radar_age / max_age) if max_age > 0 else 0.5
            
            # Apply base weights and normalize
            lidar_weight = self.lidar_weight * lidar_freshness
            radar_weight = self.radar_weight * radar_freshness
            
            # Normalize to ensure weights sum to 1.0
            total_weight = lidar_weight + radar_weight
            if total_weight > 0.001:  # Avoid division by zero
                lidar_weight /= total_weight
                radar_weight /= total_weight
            else:
                lidar_weight = self.lidar_weight
                radar_weight = self.radar_weight
                
            if self.verbose_logging:
                self.get_logger().info(f'Adaptive weights - LiDAR: {lidar_weight:.2f}, Radar: {radar_weight:.2f}')
                self.get_logger().info(f'Ages - LiDAR: {lidar_age:.2f}s, Radar: {radar_age:.2f}s')
            
            return lidar_weight, radar_weight
            
        except Exception as e:
            self.get_logger().error(f'Error calculating adaptive weights: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return self.lidar_weight, self.radar_weight
    
    def publish_fused_map(self):
        # Check if we have received both maps
        if not self.lidar_map_received or not self.radar_map_received:
            if self.verbose_logging:
                self.get_logger().info('Waiting for both maps to be received')
                self.get_logger().info(f'LiDAR map received: {self.lidar_map_received}, Radar map received: {self.radar_map_received}')
            return
        
        with self.lidar_lock, self.radar_lock:
            try:
                # Check timestamp difference for temporal consistency
                timestamp_diff = self.calculate_timestamp_difference()
                if timestamp_diff is not None and timestamp_diff > self.max_timestamp_diff:
                    self.get_logger().warning(f'Map timestamps differ by {timestamp_diff:.3f}s (> {self.max_timestamp_diff}s threshold)')
                    # Continue anyway but log the warning
                
                # Calculate adaptive weights if enabled
                if self.use_adaptive_weighting:
                    lidar_weight, radar_weight = self.calculate_adaptive_weights()
                else:
                    lidar_weight, radar_weight = self.lidar_weight, self.radar_weight
                
                # Use the LiDAR map info as the base for the fused map
                fused_map = OccupancyGrid()
                fused_map.header.stamp = self.get_clock().now().to_msg()
                fused_map.header.frame_id = self.map_frame
                fused_map.info = self.lidar_map_info
                
                # Create a copy of the LiDAR map data to avoid modifying the original
                lidar_data = self.lidar_map_data.copy()
                radar_data = self.radar_map_data.copy()
                
                # Ensure the maps have the same dimensions
                if lidar_data.shape != radar_data.shape:
                    self.get_logger().warn(f'Map dimensions mismatch: LiDAR {lidar_data.shape}, Radar {radar_data.shape}')
                    # Resize to the smaller of the two maps
                    min_height = min(lidar_data.shape[0], radar_data.shape[0])
                    min_width = min(lidar_data.shape[1], radar_data.shape[1])
                    lidar_data = lidar_data[:min_height, :min_width]
                    radar_data = radar_data[:min_height, :min_width]
                
                # Create confidence masks for each sensor
                # -1 in occupancy grid means unknown, so we should have lower confidence there
                lidar_confidence = np.ones_like(lidar_data, dtype=float)
                radar_confidence = np.ones_like(radar_data, dtype=float)
                
                # Lower confidence for unknown cells (-1 in occupancy grid)
                lidar_confidence[lidar_data == -1] = self.min_confidence_threshold
                radar_confidence[radar_data == -1] = self.min_confidence_threshold
                
                # Normalize maps from 0-100 to 0-1 for weighted average, handling unknown cells
                lidar_norm = np.where(lidar_data >= 0, lidar_data / 100.0, 0.5)  # Unknown = 0.5
                radar_norm = np.where(radar_data >= 0, radar_data / 100.0, 0.5)  # Unknown = 0.5
                
                # Apply confidence-weighted fusion
                fused_norm = (lidar_weight * lidar_norm * lidar_confidence + 
                             radar_weight * radar_norm * radar_confidence) / \
                            (lidar_weight * lidar_confidence + radar_weight * radar_confidence)
                
                # Convert back to 0-100 range
                fused_data = np.round(fused_norm * 100).astype(np.int8)
                
                # Apply obstacle threshold - values above threshold are considered obstacles
                fused_data = np.where(fused_data > self.obstacle_threshold, 100, 
                                     np.where(fused_data < 0, self.unknown_cell_value, 0))
                
                # Special case: if both sensors report unknown, keep as unknown
                both_unknown = (lidar_data == -1) & (radar_data == -1)
                fused_data[both_unknown] = -1
                
                # Flatten the grid back to 1D array
                fused_map.data = fused_data.flatten().tolist()
                
                # Publish the fused map
                self.fused_map_pub.publish(fused_map)
                
                if self.verbose_logging:
                    self.get_logger().info(f'Published fused map, size: {fused_map.info.width}x{fused_map.info.height}')
                
                # Publish debug info if enabled
                if self.enable_debug_output:
                    self.publish_debug_info(lidar_weight, radar_weight, lidar_confidence, radar_confidence)
                
                # Update fusion stats
                self.fusion_count += 1
                self.last_fusion_time = time.time()
                
            except Exception as e:
                self.get_logger().error(f'Error in publish_fused_map: {str(e)}')
                import traceback
                self.get_logger().error(traceback.format_exc())
    
    def publish_debug_info(self, lidar_weight, radar_weight, lidar_confidence, radar_confidence):
        """Publish debug information about the fusion process"""
        try:
            # Create a debug map showing the weights used
            debug_map = OccupancyGrid()
            debug_map.header.stamp = self.get_clock().now().to_msg()
            debug_map.header.frame_id = self.map_frame
            debug_map.info = self.lidar_map_info
            
            # Create a visualization of the weights (scaled to 0-100)
            # Higher values indicate more weight given to LiDAR
            weight_vis = np.round(lidar_weight * lidar_confidence / 
                                 (lidar_weight * lidar_confidence + 
                                  radar_weight * radar_confidence) * 100).astype(np.int8)
            
            debug_map.data = weight_vis.flatten().tolist()
            self.debug_pub.publish(debug_map)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing debug info: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FusionCostmapCreator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 