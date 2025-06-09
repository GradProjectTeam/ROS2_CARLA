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
    Convert quaternion to Euler angles (roll, pitch, yaw) for vehicle orientation
    
    For ground vehicles:
    - roll: rotation around X-axis (vehicle tilt left/right)
    - pitch: rotation around Y-axis (vehicle tilt forward/backward)
    - yaw: rotation around Z-axis (vehicle heading/direction)
    
    Returns angles in radians
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

# Convert Euler angles to quaternion
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion for vehicle orientation
    
    Args:
        roll: Rotation around X-axis in radians
        pitch: Rotation around Y-axis in radians
        yaw: Rotation around Z-axis in radians
        
    Returns:
        [x, y, z, w] quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = [0, 0, 0, 0]
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    
    return q

class ImuLidarYawFusion(Node):
    """
    Fuses IMU yaw data with the realtime lidar map to improve mapping accuracy.
    
    This node:
    1. Subscribes to IMU data to get yaw information
    2. Subscribes to the realtime map from lidar_realtime_mapper
    3. Applies yaw orientation to transform/rotate the map
    4. Publishes the yaw-corrected map
    5. Provides services to save and load the fused map
    6. Updates the TF tree with the current orientation
    """
    def __init__(self):
        super().__init__('imu_lidar_yaw_fusion')
        
        # Declare parameters with existence check
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.has_parameter('imu_topic'):
            self.declare_parameter('imu_topic', '/imu/data')
        if not self.has_parameter('map_topic'):
            self.declare_parameter('map_topic', '/realtime_map')  # Changed to use realtime_map instead of permanent_map
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 30.0)  # Increased rate for more responsiveness
        if not self.has_parameter('initial_yaw_offset'):
            self.declare_parameter('initial_yaw_offset', 0.0)  # Initial yaw offset in radians
        if not self.has_parameter('use_filtered_yaw'):
            self.declare_parameter('use_filtered_yaw', True)  # Use filtered yaw
        if not self.has_parameter('yaw_filter_size'):
            self.declare_parameter('yaw_filter_size', 5)  # Smaller filter size for quicker response
        if not self.has_parameter('yaw_weight'):
            self.declare_parameter('yaw_weight', 0.95)  # Higher weight of yaw in fusion (0-1)
        if not self.has_parameter('enable_auto_save'):
            self.declare_parameter('enable_auto_save', True)  # Auto-save map periodically
        if not self.has_parameter('auto_save_interval'):
            self.declare_parameter('auto_save_interval', 30.0)  # Auto-save interval in seconds
        if not self.has_parameter('map_save_dir'):
            self.declare_parameter('map_save_dir', '/home/mostafa/GP/ROS2/maps')  # Directory to save maps
        if not self.has_parameter('publish_tf'):
            self.declare_parameter('publish_tf', True)  # Whether to publish TF
        if not self.has_parameter('map_frame_id'):
            self.declare_parameter('map_frame_id', 'map')  # Map frame ID
        if not self.has_parameter('base_frame_id'):
            self.declare_parameter('base_frame_id', 'base_link')  # Base frame ID
        if not self.has_parameter('override_static_tf'):
            self.declare_parameter('override_static_tf', True)  # Whether to override static TF
        if not self.has_parameter('publish_tf_rate'):
            self.declare_parameter('publish_tf_rate', 100.0)  # Higher rate for TF publishing
        if not self.has_parameter('all_white_map'):
            self.declare_parameter('all_white_map', True)  # New parameter for all-white map mode
        if not self.has_parameter('invert_map_colors'):
            self.declare_parameter('invert_map_colors', False)  # New parameter to invert colors if needed
        if not self.has_parameter('road_plane_correction'):
            self.declare_parameter('road_plane_correction', True)  # Correct for road plane
        if not self.has_parameter('gravity_aligned'):
            self.declare_parameter('gravity_aligned', True)  # Align with gravity
        if not self.has_parameter('vehicle_forward_axis'):
            self.declare_parameter('vehicle_forward_axis', 'x')  # Vehicle forward axis (x, y, or z)
        
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
        self.all_white_map = self.get_parameter('all_white_map').value
        self.invert_map_colors = self.get_parameter('invert_map_colors').value
        self.road_plane_correction = self.get_parameter('road_plane_correction').value
        self.gravity_aligned = self.get_parameter('gravity_aligned').value
        self.vehicle_forward_axis = self.get_parameter('vehicle_forward_axis').value.lower()
        
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
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.yaw_buffer = []
        self.roll_buffer = []
        self.pitch_buffer = []
        self.filtered_yaw = 0.0
        self.filtered_roll = 0.0
        self.filtered_pitch = 0.0
        self.initial_map_orientation = None
        self.map_transformed = False
        self.last_save_time = time.time()
        
        # Store accelerometer data for gravity alignment
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        
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
        self.get_logger().info(f'All-white map mode: {self.all_white_map}')
        self.get_logger().info(f'Road plane correction: {self.road_plane_correction}')
        self.get_logger().info(f'Gravity aligned: {self.gravity_aligned}')
        self.get_logger().info(f'Vehicle forward axis: {self.vehicle_forward_axis}')
    
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
            
            # Store accelerometer data for gravity alignment
            self.accel_x = msg.linear_acceleration.x
            self.accel_y = msg.linear_acceleration.y
            self.accel_z = msg.linear_acceleration.z
            
            # Apply gravity alignment if enabled
            if self.gravity_aligned and (self.accel_x != 0.0 or self.accel_y != 0.0 or self.accel_z != 0.0):
                # Calculate magnitude of acceleration
                accel_magnitude = math.sqrt(self.accel_x**2 + self.accel_y**2 + self.accel_z**2)
                
                if accel_magnitude > 0.1:  # Ensure we have valid accelerometer data
                    # Normalize accelerometer values
                    accel_x_norm = self.accel_x / accel_magnitude
                    accel_y_norm = self.accel_y / accel_magnitude
                    accel_z_norm = self.accel_z / accel_magnitude
                    
                    # Calculate roll (rotation around X-axis) - positive is right tilt
                    accel_roll = math.atan2(accel_y_norm, math.sqrt(accel_x_norm**2 + accel_z_norm**2))
                    
                    # Calculate pitch (rotation around Y-axis) - positive is forward tilt
                    accel_pitch = -math.atan2(accel_x_norm, math.sqrt(accel_y_norm**2 + accel_z_norm**2))
                    
                    # Blend with gyro-derived values for stability (complementary filter)
                    # Use more weight from accelerometer for roll and pitch
                    roll = 0.8 * accel_roll + 0.2 * roll
                    pitch = 0.8 * accel_pitch + 0.2 * pitch
            
            # Apply road plane correction if enabled
            if self.road_plane_correction:
                # For vehicles on roads, we often want to minimize roll and pitch
                roll *= 0.8  # Reduce roll influence
                pitch *= 0.8  # Reduce pitch influence
            
            # Apply initial offset (e.g., for calibration)
            yaw += self.initial_yaw_offset
            
            # Normalize yaw to [-pi, pi]
            while yaw > math.pi:
                yaw -= 2.0 * math.pi
            while yaw < -math.pi:
                yaw += 2.0 * math.pi
            
            # Store the current orientation
            self.current_roll = roll
            self.current_pitch = pitch
            self.current_yaw = yaw
            
            # Apply moving average filter if enabled
            if self.use_filtered_yaw:
                # Roll filter
                self.roll_buffer.append(roll)
                if len(self.roll_buffer) > self.yaw_filter_size:
                    self.roll_buffer.pop(0)
                
                # Pitch filter
                self.pitch_buffer.append(pitch)
                if len(self.pitch_buffer) > self.yaw_filter_size:
                    self.pitch_buffer.pop(0)
                
                # Yaw filter
                self.yaw_buffer.append(yaw)
                if len(self.yaw_buffer) > self.yaw_filter_size:
                    self.yaw_buffer.pop(0)
                
                # Compute filtered roll (handle wrap-around for angles)
                sin_sum = sum(math.sin(r) for r in self.roll_buffer)
                cos_sum = sum(math.cos(r) for r in self.roll_buffer)
                self.filtered_roll = math.atan2(sin_sum, cos_sum)
                
                # Compute filtered pitch (handle wrap-around for angles)
                sin_sum = sum(math.sin(p) for p in self.pitch_buffer)
                cos_sum = sum(math.cos(p) for p in self.pitch_buffer)
                self.filtered_pitch = math.atan2(sin_sum, cos_sum)
                
                # Compute filtered yaw (handle wrap-around for angles)
                sin_sum = sum(math.sin(y) for y in self.yaw_buffer)
                cos_sum = sum(math.cos(y) for y in self.yaw_buffer)
                self.filtered_yaw = math.atan2(sin_sum, cos_sum)
    
    def map_callback(self, msg):
        """Process incoming realtime map data"""
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
            
            # Convert map data to 2D array for processing
            width = self.current_map.info.width
            height = self.current_map.info.height
            map_data = np.array(self.current_map.data, dtype=np.int8).reshape(height, width)
            
            # Process the map data for all-white map mode
            if self.all_white_map:
                # Convert unknown (-1 or 50) to free space (0)
                map_data[map_data == -1] = 0
                map_data[map_data == 50] = 0
                
                # In binary mode, make sure values are either 0 (free) or 100 (obstacle)
                map_data[map_data > 0] = 100
            
            # Apply the rotation to the map data
            # Get the map center for rotation
            center_x = width // 2
            center_y = height // 2
            
            # Detect if there's significant motion by checking the rate of yaw change
            # This helps determine how much to trust the IMU data
            current_time = time.time()
            if hasattr(self, 'last_yaw_time') and hasattr(self, 'last_fusion_yaw'):
                time_diff = current_time - self.last_yaw_time
                if time_diff > 0:
                    yaw_rate = abs(fusion_yaw - self.last_fusion_yaw) / time_diff
                    
                    # Adjust fusion weight based on motion
                    # Higher yaw rate = more weight to IMU (we trust it more during motion)
                    # Lower yaw rate = less weight to IMU (we trust the map more when stationary)
                    if hasattr(self, 'adaptive_fusion') and self.adaptive_fusion:
                        # Define thresholds for motion detection
                        motion_threshold = getattr(self, 'motion_threshold', 0.05)
                        
                        if yaw_rate > motion_threshold:
                            # Vehicle is likely moving - use moving fusion weight
                            effective_weight = getattr(self, 'moving_fusion_weight', 0.98)
                            self.get_logger().debug(f"Motion detected (yaw_rate={yaw_rate:.3f}), using weight={effective_weight:.2f}")
                        else:
                            # Vehicle is likely stationary - use stationary fusion weight
                            effective_weight = getattr(self, 'stationary_fusion_weight', 0.9)
                            self.get_logger().debug(f"Stationary (yaw_rate={yaw_rate:.3f}), using weight={effective_weight:.2f}")
                    else:
                        # Use default weight if adaptive fusion is not enabled
                        effective_weight = self.yaw_weight
                else:
                    effective_weight = self.yaw_weight
            else:
                effective_weight = self.yaw_weight
            
            # Store current values for next iteration
            self.last_yaw_time = current_time
            self.last_fusion_yaw = fusion_yaw
            
            # Create rotation matrix
            rot_mat = cv2.getRotationMatrix2D((center_x, center_y), math.degrees(fusion_yaw), 1.0)
            
            # Apply rotation (OpenCV uses degrees for rotation angle)
            rotated_map = cv2.warpAffine(map_data, rot_mat, (width, height), 
                                         flags=cv2.INTER_NEAREST, 
                                         borderMode=cv2.BORDER_CONSTANT, 
                                         borderValue=0 if self.all_white_map else -1)  # 0 for free space in all-white mode
            
            # Invert colors if specified (for debugging or visualization)
            if self.invert_map_colors:
                rotated_map[rotated_map == 0] = 200  # Temporarily store free as 200
                rotated_map[rotated_map == 100] = 0  # Obstacles become free
                rotated_map[rotated_map == 200] = 100  # Free becomes obstacle
            
            # Flatten and update the map data
            fused_map.data = rotated_map.flatten().tolist()
            
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
                pgm_data = np.zeros_like(map_data, dtype=np.uint8)
                
                if self.all_white_map:
                    # For all-white map, simply convert:
                    # 0 (free) -> 254 (white)
                    # 100 (occupied) -> 0 (black)
                    pgm_data[map_data == 0] = 254   # Free -> white
                    pgm_data[map_data == 100] = 0   # Occupied -> black
                else:
                    # Standard conversion:
                    # -1 (unknown) -> 205 (gray)
                    # 0 (free) -> 254 (white)
                    # 100 (occupied) -> 0 (black)
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
                    yaml_file.write(f"all_white_map: {self.all_white_map}\n")
                
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
            # Get the orientation values to use (filtered or raw)
            if self.use_filtered_yaw:
                roll = self.filtered_roll
                pitch = self.filtered_pitch
                yaw = self.filtered_yaw
            else:
                roll = self.current_roll
                pitch = self.current_pitch
                yaw = self.current_yaw
            
            # Adjust orientation based on vehicle forward axis
            # This ensures the vehicle's forward direction aligns with the expected axis
            if self.vehicle_forward_axis == 'y':
                # If vehicle forward is Y-axis, rotate 90 degrees around Z
                yaw += math.pi/2
            elif self.vehicle_forward_axis == '-x':
                # If vehicle forward is negative X-axis, rotate 180 degrees around Z
                yaw += math.pi
            elif self.vehicle_forward_axis == '-y':
                # If vehicle forward is negative Y-axis, rotate -90 degrees around Z
                yaw -= math.pi/2
            
            # Create quaternion from roll, pitch, yaw
            q = quaternion_from_euler(roll, pitch, yaw)
            
            # Create transform message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame_id
            t.child_frame_id = self.base_frame_id
            
            # Set position (could be updated based on odometry)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            # Set rotation using quaternion from yaw (only Z rotation for 2D navigation)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            # Publish the transform
            try:
                self.tf_broadcaster.sendTransform(t)
            except Exception as e:
                self.get_logger().error(f"Error publishing transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = ImuLidarYawFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 