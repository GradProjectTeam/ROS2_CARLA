#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from threading import Lock
import time


class FusionCostmapCreator(Node):
    """
    Fuses LiDAR and radar costmaps into a single coherent costmap for navigation.
    
    This node subscribes to costmaps from both sensors, applies intelligent fusion
    strategies, and produces a unified map that leverages the strengths of each sensor.
    """
    def __init__(self):
        super().__init__('fusion_costmap_creator')
        
        # Declare fusion parameters
        self.declare_parameter('lidar_weight', 0.7)       # Weight for LiDAR data (0.0-1.0)
        self.declare_parameter('radar_weight', 0.3)       # Weight for radar data (0.0-1.0)
        self.declare_parameter('dynamic_weighting', True) # Use dynamic weighting based on confidence
        self.declare_parameter('use_imu_correction', True) # Use IMU data to correct costmaps
        self.declare_parameter('publish_rate', 5.0)       # Rate to publish fused costmap (Hz)
        self.declare_parameter('publish_debug', True)     # Whether to publish debug visualizations
        
        # Get parameters
        self.lidar_weight = self.get_parameter('lidar_weight').value
        self.radar_weight = self.get_parameter('radar_weight').value
        self.dynamic_weighting = self.get_parameter('dynamic_weighting').value
        self.use_imu_correction = self.get_parameter('use_imu_correction').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_debug = self.get_parameter('publish_debug').value
        
        # Normalize weights if they don't sum to 1.0
        total_weight = self.lidar_weight + self.radar_weight
        if abs(total_weight - 1.0) > 0.001:
            self.lidar_weight /= total_weight
            self.radar_weight /= total_weight
            self.get_logger().info(f'Normalized weights: LiDAR={self.lidar_weight:.2f}, Radar={self.radar_weight:.2f}')
            
        # Storage for costmap data
        self.lidar_costmap = None
        self.radar_costmap = None
        self.fused_costmap = None
        self.lidar_timestamp = None
        self.radar_timestamp = None
        
        # Locks for thread safety
        self.lidar_lock = Lock()
        self.radar_lock = Lock()
        self.fusion_lock = Lock()
        
        # Create reliable QoS profile for map publishing
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, 
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # TF buffer and listener for frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.fused_costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/fusion/costmap',
            reliable_qos
        )
        
        # Debug publishers
        if self.publish_debug:
            self.confidence_map_publisher = self.create_publisher(
                OccupancyGrid,
                '/fusion/confidence_map',
                10
            )
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/lidar_costmap',  # From LidarCostmapCreator
            self.lidar_costmap_callback,
            reliable_qos
        )
        
        self.create_subscription(
            OccupancyGrid,
            '/radar_costmap',  # From RadarCostmapCreator
            self.radar_costmap_callback,
            reliable_qos
        )
        
        # Create timer for periodic fusion and publishing
        self.create_timer(1.0/self.publish_rate, self.fusion_and_publish)
        
        # Performance monitoring
        self.fusion_count = 0
        self.last_performance_time = time.time()
        self.create_timer(10.0, self.report_performance)
        
        self.get_logger().info('Fusion costmap creator initialized')
        self.get_logger().info(f'LiDAR weight: {self.lidar_weight:.2f}, Radar weight: {self.radar_weight:.2f}')
        self.get_logger().info(f'Dynamic weighting: {self.dynamic_weighting}')
        self.get_logger().info(f'IMU correction: {self.use_imu_correction}')
    
    def lidar_costmap_callback(self, msg):
        """Process incoming LiDAR costmap"""
        with self.lidar_lock:
            self.lidar_costmap = msg
            self.lidar_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.get_logger().debug(f'Received LiDAR costmap: {msg.info.width}x{msg.info.height}')
    
    def radar_costmap_callback(self, msg):
        """Process incoming radar costmap"""
        with self.radar_lock:
            self.radar_costmap = msg
            self.radar_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
            self.get_logger().debug(f'Received radar costmap: {msg.info.width}x{msg.info.height}')
    
    def has_valid_data(self):
        """Check if we have valid data from both sensors"""
        if self.lidar_costmap is None or self.radar_costmap is None:
            return False
            
        # Check if the timestamps are reasonably close (within 0.5 seconds)
        current_time = self.get_clock().now()
        if self.lidar_timestamp is None or self.radar_timestamp is None:
            return False
            
        lidar_age = (current_time - self.lidar_timestamp).nanoseconds / 1e9
        radar_age = (current_time - self.radar_timestamp).nanoseconds / 1e9
        
        if lidar_age > 2.0 or radar_age > 2.0:
            self.get_logger().debug(f'Data too old: LiDAR={lidar_age:.2f}s, Radar={radar_age:.2f}s')
            return False
            
        return True
    
    def calculate_dynamic_weights(self, lidar_data, radar_data):
        """Calculate dynamic weights based on sensor confidence"""
        # This is a simple implementation - in a real system, you'd use more 
        # sophisticated confidence metrics based on sensor characteristics
        
        lidar_confidence = np.ones_like(lidar_data, dtype=float)
        radar_confidence = np.ones_like(radar_data, dtype=float)
        
        # Adjust confidence based on value certainty
        # -1 is unknown, 0 is free, 100 is occupied
        
        # LiDAR is more confident about definite free space and obstacles
        lidar_confidence[lidar_data == 0] = 0.9    # High confidence in free space
        lidar_confidence[lidar_data == 100] = 0.9  # High confidence in obstacles
        lidar_confidence[lidar_data == -1] = 0.3   # Low confidence in unknown space
        
        # Radar is more confident about moving objects and less about static ones
        # In a real implementation, you would need to know which cells contain moving objects
        radar_confidence[radar_data == 0] = 0.7    # Medium confidence in free space
        radar_confidence[radar_data == 100] = 0.8  # High confidence in obstacles (potentially moving)
        radar_confidence[radar_data == -1] = 0.2   # Very low confidence in unknown space
        
        # Normalize the confidence maps to create weights
        total_confidence = lidar_confidence + radar_confidence
        lidar_weights = np.divide(lidar_confidence, total_confidence, 
                                 out=np.ones_like(lidar_confidence)*0.5, 
                                 where=total_confidence>0)
        radar_weights = np.divide(radar_confidence, total_confidence,
                                 out=np.ones_like(radar_confidence)*0.5,
                                 where=total_confidence>0)
                                 
        return lidar_weights, radar_weights
    
    def fusion_and_publish(self):
        """Perform costmap fusion and publish the result"""
        if not self.has_valid_data():
            return
            
        # Acquire locks for both costmaps during fusion
        with self.lidar_lock, self.radar_lock, self.fusion_lock:
            # Check if dimensions match
            lidar_width = self.lidar_costmap.info.width
            lidar_height = self.lidar_costmap.info.height
            radar_width = self.radar_costmap.info.width
            radar_height = self.radar_costmap.info.height
            
            if lidar_width != radar_width or lidar_height != radar_height:
                self.get_logger().error(f'Costmap dimensions do not match: '
                                       f'LiDAR {lidar_width}x{lidar_height}, '
                                       f'Radar {radar_width}x{radar_height}')
                return
                
            # Check if resolutions are compatible
            lidar_resolution = self.lidar_costmap.info.resolution
            radar_resolution = self.radar_costmap.info.resolution
            
            if abs(lidar_resolution - radar_resolution) > 0.001:
                self.get_logger().error(f'Costmap resolutions do not match: '
                                       f'LiDAR {lidar_resolution:.3f}, '
                                       f'Radar {radar_resolution:.3f}')
                return
                
            # Convert costmap data to numpy arrays (0-100 scale where -1 is unknown)
            lidar_data = np.array(self.lidar_costmap.data).reshape(lidar_height, lidar_width)
            radar_data = np.array(self.radar_costmap.data).reshape(radar_height, radar_width)
            
            # Create output array
            fused_data = np.full((lidar_height, lidar_width), -1, dtype=np.int8)
            
            # Apply fusion algorithm
            if self.dynamic_weighting:
                # Dynamic weighting based on cell-by-cell confidence
                lidar_weights, radar_weights = self.calculate_dynamic_weights(lidar_data, radar_data)
                
                # Apply weighted fusion
                for y in range(lidar_height):
                    for x in range(lidar_width):
                        if lidar_data[y, x] == -1 and radar_data[y, x] == -1:
                            # If both are unknown, result is unknown
                            fused_data[y, x] = -1
                        elif lidar_data[y, x] == -1:
                            # If LiDAR is unknown, use radar
                            fused_data[y, x] = radar_data[y, x]
                        elif radar_data[y, x] == -1:
                            # If radar is unknown, use LiDAR
                            fused_data[y, x] = lidar_data[y, x]
                        else:
                            # Both have data, apply weighted fusion
                            lw = lidar_weights[y, x]
                            rw = radar_weights[y, x]
                            fused_value = lw * lidar_data[y, x] + rw * radar_data[y, x]
                            fused_data[y, x] = min(100, max(0, int(round(fused_value))))
            else:
                # Simple weighted fusion using global weights
                for y in range(lidar_height):
                    for x in range(lidar_width):
                        if lidar_data[y, x] == -1 and radar_data[y, x] == -1:
                            # If both are unknown, result is unknown
                            fused_data[y, x] = -1
                        elif lidar_data[y, x] == -1:
                            # If LiDAR is unknown, use radar
                            fused_data[y, x] = radar_data[y, x]
                        elif radar_data[y, x] == -1:
                            # If radar is unknown, use LiDAR
                            fused_data[y, x] = lidar_data[y, x]
                        else:
                            # Both have data, apply weighted fusion
                            fused_value = self.lidar_weight * lidar_data[y, x] + self.radar_weight * radar_data[y, x]
                            fused_data[y, x] = min(100, max(0, int(round(fused_value))))
            
            # Create and publish fused costmap message
            fused_msg = OccupancyGrid()
            fused_msg.header.stamp = self.get_clock().now().to_msg()
            fused_msg.header.frame_id = 'map'  # Use same frame as input costmaps
            
            # Copy metadata from LiDAR costmap
            fused_msg.info = self.lidar_costmap.info
            
            # Flatten the fused data and convert to list
            fused_msg.data = fused_data.flatten().tolist()
            
            # Publish the fused costmap
            self.fused_costmap_publisher.publish(fused_msg)
            self.fusion_count += 1
            
            # Publish debug visualizations if enabled
            if self.publish_debug and self.dynamic_weighting:
                # Create a visualization of the confidence weights
                confidence_vis = np.zeros((lidar_height, lidar_width), dtype=np.int8)
                
                # Scale lidar weights (0.0-1.0) to occupancy grid values (0-100)
                for y in range(lidar_height):
                    for x in range(lidar_width):
                        confidence_vis[y, x] = int(lidar_weights[y, x] * 100)
                
                confidence_msg = OccupancyGrid()
                confidence_msg.header.stamp = self.get_clock().now().to_msg()
                confidence_msg.header.frame_id = 'map'
                confidence_msg.info = self.lidar_costmap.info
                confidence_msg.data = confidence_vis.flatten().tolist()
                
                self.confidence_map_publisher.publish(confidence_msg)
    
    def report_performance(self):
        """Report fusion performance statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_performance_time
        
        if elapsed > 0 and self.fusion_count > 0:
            fusion_rate = self.fusion_count / elapsed
            self.get_logger().info(f'Fusion performance: {fusion_rate:.2f} Hz')
            
            self.fusion_count = 0
            self.last_performance_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = FusionCostmapCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 