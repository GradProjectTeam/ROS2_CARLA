#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from threading import Lock
import numpy as np
import math
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from std_srvs.srv import Trigger
import os
from datetime import datetime
import tf2_ros

# Convert quaternion to Euler angles
def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
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

class ImuLidarYawFusion(Node):
    """
    Fuses IMU yaw data with the permanent lidar map to improve mapping accuracy.
    
    This node:
    1. Subscribes to IMU data to get yaw information
    2. Subscribes to the permanent map from lidar_permanent_mapper
    3. Applies yaw orientation to transform/rotate the map
    4. Publishes the yaw-corrected map
    5. Provides services to save and load the fused map
    """
    def __init__(self):
        super().__init__('imu_lidar_yaw_fusion')
        
        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('map_topic', '/permanent_map')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('initial_yaw_offset', 0.0)  # Initial yaw offset in radians
        self.declare_parameter('use_filtered_yaw', True)  # Use filtered yaw
        self.declare_parameter('yaw_filter_size', 10)  # Size of yaw filter buffer
        self.declare_parameter('yaw_weight', 0.7)  # Weight of yaw in fusion (0-1)
        self.declare_parameter('enable_auto_save', True)  # Auto-save map periodically
        self.declare_parameter('auto_save_interval', 60.0)  # Auto-save interval in seconds
        self.declare_parameter('map_save_dir', '/home/mostafa/GP/ROS2/maps')  # Directory to save maps
        self.declare_parameter('publish_tf', True)  # Whether to publish TF
        self.declare_parameter('map_frame_id', 'map')  # Map frame ID
        self.declare_parameter('base_frame_id', 'base_link')  # Base frame ID
        self.declare_parameter('override_static_tf', True)  # Whether to override static TF
        self.declare_parameter('publish_tf_rate', 50.0)  # Higher rate for TF publishing
        
        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.initial_yaw_offset = self.get_parameter('initial_yaw_offset').value
        self.use_filtered_yaw = self.get_parameter('use_filtered_yaw').value
        self.yaw_filter_size = self.get_parameter('yaw_filter_size').value
        self.yaw_weight = self.get_parameter('yaw_weight').value
        self.enable_auto_save = self.get_parameter('enable_auto_save').value
        self.auto_save_interval = self.get_parameter('auto_save_interval').value
        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.override_static_tf = self.get_parameter('override_static_tf').value
        self.publish_tf_rate = self.get_parameter('publish_tf_rate').value
        
        # Ensure map save directory exists
        if not os.path.exists(self.map_save_dir):
            try:
                os.makedirs(self.map_save_dir)
                self.get_logger().info(f"Created map save directory: {self.map_save_dir}")
            except Exception as e:
                self.get_logger().error(f"Failed to create map directory: {e}")
        
        # State variables
        self.current_map = None
        self.map_metadata = None
        self.current_yaw = 0.0
        self.yaw_buffer = []
        self.filtered_yaw = 0.0
        self.initial_map_orientation = None
        self.map_transformed = False
        self.last_save_time = time.time()
        
        # Locks for thread safety
        self.map_lock = Lock()
        self.yaw_lock = Lock()
        
        # TF listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create reliable QoS profile for map publishing
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.fused_map_publisher = self.create_publisher(
            OccupancyGrid,
            '/fused_map',
            reliable_qos
        )
        
        # Subscribers
        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )
        
        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            reliable_qos
        )
        
        # Services
        self.save_service = self.create_service(
            Trigger,
            'save_fused_map',
            self.save_map_callback
        )
        
        self.load_service = self.create_service(
            Trigger,
            'load_fused_map',
            self.load_map_callback
        )
        
        # Timers
        self.create_timer(1.0 / self.publish_rate, self.process_and_publish)
        
        if self.enable_auto_save:
            self.create_timer(self.auto_save_interval, self.auto_save_map)
        
        # Performance monitoring
        self.frame_count = 0
        self.last_performance_time = time.time()
        self.create_timer(10.0, self.report_performance)
        
        # Create TF broadcaster and timer for TF publishing
        self.tf_broadcaster = TransformBroadcaster(self)
        if self.publish_tf:
            self.create_timer(1.0 / self.publish_tf_rate, self.publish_tf_transform)
        
        self.get_logger().info('IMU Lidar Yaw Fusion node initialized')
        self.get_logger().info(f'IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Map topic: {self.map_topic}')
        self.get_logger().info(f'Publishing rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Using filtered yaw: {self.use_filtered_yaw}')
        self.get_logger().info(f'Initial yaw offset: {math.degrees(self.initial_yaw_offset):.2f} degrees')
    
    def imu_callback(self, msg):
        """Process incoming IMU data to extract yaw information"""
        with self.yaw_lock:
            # Extract quaternion
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            
            # Convert to Euler angles
            roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
            
            # Apply initial offset (e.g., for calibration)
            yaw += self.initial_yaw_offset
            
            # Store the current yaw
            self.current_yaw = yaw
            
            # Apply moving average filter if enabled
            if self.use_filtered_yaw:
                self.yaw_buffer.append(yaw)
                if len(self.yaw_buffer) > self.yaw_filter_size:
                    self.yaw_buffer.pop(0)
                
                # Compute filtered yaw (handle wrap-around for angles)
                sin_sum = sum(math.sin(y) for y in self.yaw_buffer)
                cos_sum = sum(math.cos(y) for y in self.yaw_buffer)
                self.filtered_yaw = math.atan2(sin_sum, cos_sum)
    
    def map_callback(self, msg):
        """Process incoming permanent map data"""
        with self.map_lock:
            # Store the map and its metadata
            self.current_map = msg
            self.map_metadata = msg.info
            
            # Capture initial map orientation if not already set
            if self.initial_map_orientation is None and msg is not None:
                # Extract map orientation from quaternion
                qx = msg.info.origin.orientation.x
                qy = msg.info.origin.orientation.y
                qz = msg.info.origin.orientation.z
                qw = msg.info.origin.orientation.w
                
                _, _, map_yaw = euler_from_quaternion(qx, qy, qz, qw)
                self.initial_map_orientation = map_yaw
                self.get_logger().info(f'Initial map orientation captured: {math.degrees(map_yaw):.2f} degrees')
    
    def process_and_publish(self):
        """Process the map with yaw information and publish the fused result"""
        with self.map_lock, self.yaw_lock:
            if self.current_map is None:
                return
            
            # Get the yaw to use for fusion
            fusion_yaw = self.filtered_yaw if self.use_filtered_yaw else self.current_yaw
            
            # Create a copy of the current map for fusion
            fused_map = OccupancyGrid()
            fused_map.header = self.current_map.header
            fused_map.header.stamp = self.get_clock().now().to_msg()
            fused_map.header.frame_id = "map"
            fused_map.info = self.current_map.info
            
            # Apply yaw orientation to the map origin
            # This effectively rotates the map based on IMU yaw
            # For a real implementation, you'd want to apply the rotation to the map data itself
            
            # Convert map data to 2D array for processing
            width = self.current_map.info.width
            height = self.current_map.info.height
            map_data = np.array(self.current_map.data, dtype=np.int8).reshape(height, width)
            
            # Apply the rotation to the map data
            # Get the map center for rotation
            center_x = width // 2
            center_y = height // 2
            
            # Create rotation matrix
            rot_mat = cv2.getRotationMatrix2D((center_x, center_y), math.degrees(fusion_yaw), 1.0)
            
            # Apply rotation (OpenCV uses degrees for rotation angle)
            rotated_map = cv2.warpAffine(map_data, rot_mat, (width, height), 
                                         flags=cv2.INTER_NEAREST, 
                                         borderMode=cv2.BORDER_CONSTANT, 
                                         borderValue=-1)  # -1 for unknown space
            
            # Flatten and update the map data
            fused_map.data = rotated_map.flatten().tolist()
            
            # Update the map's origin orientation with the IMU yaw
            # This is a simplified approach - a more sophisticated method would be needed 
            # to properly update the map frame based on the IMU orientation
            
            # Publish the fused map
            self.fused_map_publisher.publish(fused_map)
            self.frame_count += 1
            self.map_transformed = True
    
    def save_map_callback(self, request, response):
        """Service callback to save the fused map"""
        success = self.save_map()
        response.success = success
        response.message = "Map saved successfully" if success else "Failed to save map"
        return response
    
    def load_map_callback(self, request, response):
        """Service callback to load a saved fused map"""
        success = self.load_map()
        response.success = success
        response.message = "Map loaded successfully" if success else "Failed to load map"
        return response
    
    def save_map(self):
        """Save the current fused map to disk"""
        try:
            with self.map_lock:
                if self.current_map is None or not self.map_transformed:
                    self.get_logger().warning("No transformed map available to save")
                    return False
                
                # Create timestamp for filename
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(self.map_save_dir, f"fused_map_{timestamp}.pgm")
                
                # Save map data as PGM file
                width = self.current_map.info.width
                height = self.current_map.info.height
                map_data = np.array(self.current_map.data, dtype=np.int8).reshape(height, width)
                
                # Convert occupancy values to grayscale (0-255)
                # -1 (unknown) -> 205 (gray)
                # 0 (free) -> 254 (white)
                # 100 (occupied) -> 0 (black)
                pgm_data = np.zeros_like(map_data, dtype=np.uint8)
                pgm_data[map_data == -1] = 205  # Unknown -> gray
                pgm_data[map_data == 0] = 254   # Free -> white
                pgm_data[map_data == 100] = 0   # Occupied -> black
                
                # Write PGM file
                cv2.imwrite(filename, pgm_data)
                
                # Save YAML metadata
                yaml_filename = os.path.splitext(filename)[0] + ".yaml"
                resolution = self.current_map.info.resolution
                origin_x = self.current_map.info.origin.position.x
                origin_y = self.current_map.info.origin.position.y
                
                with open(yaml_filename, 'w') as yaml_file:
                    yaml_file.write(f"image: {os.path.basename(filename)}\n")
                    yaml_file.write(f"resolution: {resolution}\n")
                    yaml_file.write(f"origin: [{origin_x}, {origin_y}, 0.0]\n")
                    yaml_file.write(f"negate: 0\n")
                    yaml_file.write(f"occupied_thresh: 0.65\n")
                    yaml_file.write(f"free_thresh: 0.196\n")
                    yaml_file.write(f"yaw_fusion: {math.degrees(self.filtered_yaw):.6f}\n")
                
                self.get_logger().info(f"Saved fused map to {filename} and {yaml_filename}")
                self.last_save_time = time.time()
                return True
                
        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")
            return False
    
    def load_map(self):
        """Load a previously saved fused map"""
        # This would load a map from disk
        # Implementation would depend on how you store and retrieve maps
        self.get_logger().warning("Map loading not yet implemented")
        return False
    
    def auto_save_map(self):
        """Automatically save the map at regular intervals"""
        if self.enable_auto_save and time.time() - self.last_save_time >= self.auto_save_interval:
            if self.save_map():
                self.get_logger().info("Auto-saved fused map")
    
    def report_performance(self):
        """Report node performance statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_performance_time
        rate = self.frame_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(f"Performance: {rate:.2f} maps/sec, {self.frame_count} maps processed")
        
        self.frame_count = 0
        self.last_performance_time = current_time

    def publish_tf_transform(self):
        """Publish the transform from map to base_link based on IMU orientation"""
        if not self.publish_tf:
            return
        
        with self.yaw_lock:
            # Get the yaw to use (filtered or raw)
            tf_yaw = self.filtered_yaw if self.use_filtered_yaw else self.current_yaw
            
            # Create transform message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame_id
            t.child_frame_id = self.base_frame_id
            
            # Set position (could be updated based on odometry)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            # Set rotation using quaternion from yaw
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(tf_yaw / 2.0)
            t.transform.rotation.w = math.cos(tf_yaw / 2.0)
            
            # Publish the transform
            self.tf_broadcaster.sendTransform(t)
            
            if not hasattr(self, 'last_tf_log_time') or time.time() - self.last_tf_log_time > 5.0:
                self.get_logger().info(f"Publishing TF: yaw={math.degrees(tf_yaw):.2f} degrees")
                self.last_tf_log_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    
    node = ImuLidarYawFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 