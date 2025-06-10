#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from threading import Lock
import numpy as np
import math
import time
import cv2
import socket
import struct
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Our own implementation of quaternion_from_euler and euler_from_quaternion
# instead of importing from tf_transformations

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    
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

def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        x, y, z, w: Quaternion components
        
    Returns:
        Tuple of (roll, pitch, yaw) in radians
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

class ImuCameraLaneFusion(Node):
    """
    Fuses IMU compass data with camera lane detection to visualize road lanes in a map.
    
    This node:
    1. Connects to IMU data via TCP socket
    2. Subscribes to camera images to detect lanes
    3. Fuses the IMU compass heading with lane detection
    4. Projects the lanes onto a map
    5. Publishes visualization markers for the lanes
    """
    def __init__(self):
        super().__init__('imu_camera_lane_fusion')
        
        # Create parameter descriptors with descriptions
        heading_smoothing_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Smoothing factor for heading changes (0.0-1.0, lower = smoother)'
        )
        
        heading_rate_limit_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum heading change rate in degrees per second (0.0 = no limit)'
        )
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/carla/camera/rgb/image_raw')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('lane_projection_distance', 20.0)  # How far to project lanes in meters
        self.declare_parameter('lane_width', 3.5)  # Standard lane width in meters
        self.declare_parameter('use_filtered_heading', True)
        self.declare_parameter('heading_filter_size', 30)  # Increased for even smoother filtering
        self.declare_parameter('heading_smoothing_factor', 0.05, heading_smoothing_descriptor)  # Lower value for smoother changes
        self.declare_parameter('heading_rate_limit', 10.0, heading_rate_limit_descriptor)  # Max 10 degrees per second
        self.declare_parameter('lane_confidence_threshold', 0.6)  # Threshold for lane detection confidence
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12345)  # TCP port for IMU data
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('compass_offset', 0.0)  # Offset to align compass with vehicle forward direction
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.lane_projection_distance = self.get_parameter('lane_projection_distance').value
        self.lane_width = self.get_parameter('lane_width').value
        self.use_filtered_heading = self.get_parameter('use_filtered_heading').value
        self.heading_filter_size = self.get_parameter('heading_filter_size').value
        self.heading_smoothing_factor = self.get_parameter('heading_smoothing_factor').value
        self.heading_rate_limit = self.get_parameter('heading_rate_limit').value
        self.lane_confidence_threshold = self.get_parameter('lane_confidence_threshold').value
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.compass_offset = self.get_parameter('compass_offset').value
        
        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # State variables
        self.current_heading = 0.0
        self.heading_buffer = []
        self.filtered_heading = 0.0
        self.prev_filtered_heading = 0.0  # Store previous heading for smoothing
        self.prev_raw_compass = 0.0  # Store previous raw compass value
        self.last_heading_update_time = time.time()
        self.left_lane_points = []
        self.right_lane_points = []
        self.lane_center_points = []
        self.last_lane_update_time = None
        self.last_imu_update_time = None
        
        # IMU data from TCP
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        self.compass = 0.0  # We'll use this as our primary heading source
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # TCP socket related
        self.socket = None
        self.connected = False
        self.buffer = b''
        
        # Data size constants - expect 10 float values from C++ TCP server
        # 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
        self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')  # 40 bytes
        
        # Image processing
        self.bridge = CvBridge()
        self.current_image = None
        
        # Fixed camera resolution
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        
        # Lane detection parameters
        self.min_lane_points = 5     # Minimum points needed to fit a lane
        self.min_line_length = 20    # Minimum line length for Hough transform
        self.max_line_gap = 30       # Maximum line gap for Hough transform
        self.lane_detection_threshold = 20  # Hough transform threshold
        
        # Locks for thread safety
        self.imu_lock = Lock()
        self.camera_lock = Lock()
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.lane_marker_publisher = self.create_publisher(
            MarkerArray,
            '/lane_markers',
            10
        )
        
        self.lane_center_publisher = self.create_publisher(
            PointStamped,
            '/lane_center',
            10
        )
        
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/processed_lane_image',
            10
        )
        
        # Also publish standard IMU messages for compatibility
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )
        
        # Camera subscriber
        self.camera_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10
        )
        
        # Attempt initial TCP connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.process_imu_data)  # 100Hz for IMU processing
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
        # Timer for processing and publishing
        self.create_timer(1.0/self.publish_rate, self.process_and_publish)
        
        # Performance monitoring
        self.process_count = 0
        self.last_performance_time = time.time()
        self.create_timer(10.0, self.report_performance)
        
        self.get_logger().info('IMU Camera Lane Fusion node initialized')
        self.get_logger().info('Using only compass data for heading')
        self.get_logger().info(f'Connecting to IMU server at {self.tcp_ip}:{self.tcp_port}')
    
    def connect(self):
        """Attempt to connect to the TCP server"""
        if self.connected:
            return
            
        try:
            if self.socket:
                self.socket.close()
                
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(1.0)  # 1 second timeout
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.socket.setblocking(False)  # Non-blocking socket
            self.connected = True
            self.buffer = b''  # Clear buffer on new connection
            self.get_logger().info(f'Connected to IMU server at {self.tcp_ip}:{self.tcp_port}')
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f'Failed to connect to IMU server: {str(e)}')
    
    def check_connection(self):
        """Check if connection is still valid and reconnect if needed"""
        if not self.connected:
            self.get_logger().info('Attempting to reconnect to IMU server...')
            self.connect()
    
    def process_imu_data(self):
        """Process IMU data from TCP socket"""
        if not self.connected:
            return
            
        try:
            # Try to receive data (non-blocking)
            data = self.socket.recv(4096)
            if not data:
                self.get_logger().warn('Connection closed by server')
                self.connected = False
                return
                
            # Add to buffer
            self.buffer += data
            
            # Process complete packets
            while len(self.buffer) >= self.IMU_DATA_SIZE:
                # Extract one packet
                packet = self.buffer[:self.IMU_DATA_SIZE]
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
                # Unpack the 10 float values
                values = struct.unpack('ffffffffff', packet)
                
                with self.imu_lock:
                    self.accel_x, self.accel_y, self.accel_z = values[0:3]
                    self.gyro_x, self.gyro_y, self.gyro_z = values[3:6]
                    
                    # Store previous compass value
                    self.prev_raw_compass = self.compass
                    
                    # Get new compass value
                    raw_compass = values[6]
                    self.compass = raw_compass
                    self.roll, self.pitch, self.yaw = values[7:10]
                    
                    # Convert compass to radians if it's in degrees
                    # Check if the compass value is likely in degrees (values > 2π)
                    compass_in_radians = raw_compass
                    if abs(raw_compass) > (2 * math.pi):
                        # Value is likely in degrees, convert to radians
                        compass_in_radians = math.radians(raw_compass)
                    
                    # Normalize compass to [0, 2π] range
                    normalized_compass = compass_in_radians % (2 * math.pi)
                    
                    # Apply compass offset
                    self.current_heading = (normalized_compass + self.compass_offset) % (2 * math.pi)
                    
                    # Store previous filtered heading for smoothing
                    self.prev_filtered_heading = self.filtered_heading
                    current_time = time.time()
                    dt = current_time - self.last_heading_update_time
                    self.last_heading_update_time = current_time
                    
                    # Check for large discrepancy between current and filtered heading
                    if self.filtered_heading != 0.0:
                        # Calculate absolute angle difference
                        diff = abs(self.normalize_angle(self.current_heading - self.filtered_heading))
                        
                        # If difference is more than 30 degrees (reduced from 45), reset the filter
                        # This makes the system even more responsive to heading changes
                        if diff > math.radians(30):
                            self.get_logger().info(f'Large heading change detected ({math.degrees(diff):.1f}°), resetting filter')
                            self.heading_buffer = []
                            self.filtered_heading = self.current_heading
                            self.prev_filtered_heading = self.current_heading
                    
                    # Apply heading filter if enabled
                    if self.use_filtered_heading:
                        # Add to buffer
                        self.heading_buffer.append(self.current_heading)
                        if len(self.heading_buffer) > self.heading_filter_size:
                            self.heading_buffer.pop(0)
                            
                        # Calculate filtered heading using circular mean
                        sin_sum = 0.0
                        cos_sum = 0.0
                        for angle in self.heading_buffer:
                            sin_sum += math.sin(angle)
                            cos_sum += math.cos(angle)
                            
                        mean_heading = math.atan2(sin_sum, cos_sum)
                        if mean_heading < 0:
                            mean_heading += 2 * math.pi
                        
                        # Apply additional exponential smoothing
                        # This helps reduce rapid changes between readings
                        if self.prev_filtered_heading != 0.0:
                            # Handle angle wrapping (e.g., 359° to 1°)
                            diff = mean_heading - self.prev_filtered_heading
                            if diff > math.pi:
                                diff -= 2 * math.pi
                            elif diff < -math.pi:
                                diff += 2 * math.pi
                                
                            # Apply rate limiting if enabled
                            if self.heading_rate_limit > 0.0 and dt > 0.0:
                                max_change = math.radians(self.heading_rate_limit * dt)
                                if abs(diff) > max_change:
                                    diff = math.copysign(max_change, diff)
                            
                            # Apply smoothing factor - use a higher factor for larger differences
                            # Further increased base factor for even faster response
                            adaptive_factor = min(1.0, self.heading_smoothing_factor * (1 + abs(diff) * 15))
                            smooth_heading = self.prev_filtered_heading + adaptive_factor * diff
                            
                            # Normalize to [0, 2π]
                            self.filtered_heading = smooth_heading % (2 * math.pi)
                        else:
                            self.filtered_heading = mean_heading
                    else:
                        self.filtered_heading = self.current_heading
                
                self.last_imu_update_time = time.time()
                self.publish_imu_data()
                
        except BlockingIOError:
            # No data available right now, this is normal for non-blocking sockets
            pass
        except ConnectionResetError:
            self.get_logger().warn('Connection reset by server')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
            self.connected = False
    
    def publish_imu_data(self):
        """Publish standard IMU message"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.base_frame_id
        
        # Convert filtered compass heading to quaternion (only yaw, zero roll and pitch)
        q = quaternion_from_euler(0.0, 0.0, self.filtered_heading)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        
        # Set angular velocity
        imu_msg.angular_velocity.x = self.gyro_x
        imu_msg.angular_velocity.y = self.gyro_y
        imu_msg.angular_velocity.z = self.gyro_z
        
        # Set linear acceleration
        imu_msg.linear_acceleration.x = self.accel_x
        imu_msg.linear_acceleration.y = self.accel_y
        imu_msg.linear_acceleration.z = self.accel_z
        
        # Publish
        self.imu_publisher.publish(imu_msg)
    
    def camera_callback(self, msg):
        """Process incoming camera images to detect lanes"""
        with self.camera_lock:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.current_image = cv_image
                
                # Process image to detect lanes
                self.detect_lanes(cv_image)
                
                self.last_lane_update_time = self.get_clock().now()
            except Exception as e:
                self.get_logger().error(f'Error processing camera image: {e}')
    
    def detect_lanes(self, image):
        """Detect lanes in the camera image"""
        if image is None:
            return
            
        # Resize the image if needed
        if image.shape[1] != self.IMAGE_WIDTH or image.shape[0] != self.IMAGE_HEIGHT:
            image = cv2.resize(image, (self.IMAGE_WIDTH, self.IMAGE_HEIGHT))
        
        # Define the region of interest (ROI)
        roi = image[240:480, 108:532]  # Bottom half of the image
        roi_im = cv2.resize(roi, (424, 240))
        
        # Apply Gaussian Blur to the ROI
        blur_im = cv2.bilateralFilter(roi_im, d=-1, sigmaColor=5, sigmaSpace=5)
        
        # Convert to grayscale
        gray = cv2.cvtColor(blur_im, cv2.COLOR_BGR2GRAY)
        
        # Detect edges using Canny edge detector
        edges = cv2.Canny(gray, 50, 100)
        
        # Apply Hough Transformation to detect lines
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180.0, threshold=25, 
                               minLineLength=10, maxLineGap=20)
        
        # Process detected lines
        left_lane_points = []
        right_lane_points = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Skip vertical lines to avoid division by zero
                if x2 == x1:
                    continue
                
                # Calculate line slope
                slope = (y2 - y1) / (x2 - x1)
                angle = math.atan(slope) * 180.0 / math.pi
                
                # Adjust coordinates back to original image space
                pt1 = (x1 + 108, y1 + 240)
                pt2 = (x2 + 108, y2 + 240)
                
                # Classify as left or right lane based on angle
                if 20.0 < angle < 90.0:  # Right lane (positive slope)
                    right_lane_points.append((pt1, pt2))
                    cv2.line(image, pt1, pt2, (0, 0, 255), 2)  # Red for right lane
                elif -90.0 < angle < -20.0:  # Left lane (negative slope)
                    left_lane_points.append((pt1, pt2))
                    cv2.line(image, pt1, pt2, (255, 0, 0), 2)  # Blue for left lane
        
        # Fit lanes if enough points are detected
        left_lane_valid = len(left_lane_points) >= self.min_lane_points
        right_lane_valid = len(right_lane_points) >= self.min_lane_points
        
        # Store lane points for visualization
        self.left_lane_points = left_lane_points if left_lane_valid else []
        self.right_lane_points = right_lane_points if right_lane_valid else []
        
        # Calculate lane center if both lanes are detected
        if left_lane_valid and right_lane_valid:
            # Simple approach: average the x-coordinates at the bottom of the image
            left_x_avg = sum(pt1[0] for pt1, _ in left_lane_points) / len(left_lane_points)
            right_x_avg = sum(pt1[0] for pt1, _ in right_lane_points) / len(right_lane_points)
            center_x = (left_x_avg + right_x_avg) / 2
            
            # Draw lane center
            cv2.line(image, (int(center_x), self.IMAGE_HEIGHT), 
                    (int(center_x), self.IMAGE_HEIGHT - 50), (0, 255, 0), 2)
            
            # Store lane center for projection
            self.lane_center_points = [(int(center_x), self.IMAGE_HEIGHT)]
        
        # Publish processed image
        try:
            processed_img_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            processed_img_msg.header.stamp = self.get_clock().now().to_msg()
            processed_img_msg.header.frame_id = self.camera_frame_id
            self.processed_image_publisher.publish(processed_img_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing processed image: {e}')
    
    def process_and_publish(self):
        """Process fused data and publish visualization"""
        # We'll publish lane markers even if we don't have camera data yet
        # This allows us to see the heading changes
        with self.imu_lock:
            # Always create and publish lane markers with the latest heading
            # This ensures the visualization updates quickly in response to heading changes
            marker_array = self.create_lane_markers()
            if marker_array is not None:
                self.lane_marker_publisher.publish(marker_array)
            
            # Publish lane center point if we have lane data
            if self.lane_center_points:
                self.publish_lane_center()
            
        self.process_count += 1
    
    def create_lane_markers(self):
        """Create visualization markers for the detected lanes"""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Always use the most current filtered heading for lane projection
        # This ensures the visualization responds immediately to heading changes
        heading = self.filtered_heading
        
        # Create marker for left lane
        left_marker = Marker()
        left_marker.header.frame_id = self.map_frame_id
        left_marker.header.stamp = now
        left_marker.ns = "left_lane"
        left_marker.id = 0
        left_marker.type = Marker.LINE_STRIP
        left_marker.action = Marker.ADD
        left_marker.pose.orientation.w = 1.0
        left_marker.scale.x = 0.2  # Line width
        left_marker.color.r = 1.0
        left_marker.color.g = 1.0
        left_marker.color.b = 0.0
        left_marker.color.a = 1.0
        # Reduced marker lifetime for more frequent updates
        left_marker.lifetime.sec = 0
        left_marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        # Project lane points into 3D space using filtered compass heading
        for i in range(10):  # Create 10 points along the lane
            distance = i * (self.lane_projection_distance / 10.0)
            # Left lane is offset from center by half the lane width
            x = distance * math.cos(heading)
            y = distance * math.sin(heading) - self.lane_width / 2.0
            z = 0.0  # On the ground plane
            
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            left_marker.points.append(point)
        
        marker_array.markers.append(left_marker)
        
        # Create marker for right lane
        right_marker = Marker()
        right_marker.header.frame_id = self.map_frame_id
        right_marker.header.stamp = now
        right_marker.ns = "right_lane"
        right_marker.id = 1
        right_marker.type = Marker.LINE_STRIP
        right_marker.action = Marker.ADD
        right_marker.pose.orientation.w = 1.0
        right_marker.scale.x = 0.2  # Line width
        right_marker.color.r = 1.0
        right_marker.color.g = 1.0
        right_marker.color.b = 0.0
        right_marker.color.a = 1.0
        # Reduced marker lifetime for more frequent updates
        right_marker.lifetime.sec = 0
        right_marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        # Project lane points into 3D space using filtered compass heading
        for i in range(10):  # Create 10 points along the lane
            distance = i * (self.lane_projection_distance / 10.0)
            # Right lane is offset from center by half the lane width
            x = distance * math.cos(heading)
            y = distance * math.sin(heading) + self.lane_width / 2.0
            z = 0.0  # On the ground plane
            
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            right_marker.points.append(point)
        
        marker_array.markers.append(right_marker)
        
        # Create marker for lane center line
        center_marker = Marker()
        center_marker.header.frame_id = self.map_frame_id
        center_marker.header.stamp = now
        center_marker.ns = "lane_center"
        center_marker.id = 2
        center_marker.type = Marker.LINE_STRIP
        center_marker.action = Marker.ADD
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.1  # Line width
        center_marker.color.r = 0.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0
        # Reduced marker lifetime for more frequent updates
        center_marker.lifetime.sec = 0
        center_marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        # Project center line using filtered compass heading
        for i in range(10):  # Create 10 points along the center
            distance = i * (self.lane_projection_distance / 10.0)
            x = distance * math.cos(heading)
            y = distance * math.sin(heading)
            z = 0.0  # On the ground plane
            
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            center_marker.points.append(point)
        
        marker_array.markers.append(center_marker)
        
        # Create a heading indicator arrow
        arrow_marker = Marker()
        arrow_marker.header.frame_id = self.map_frame_id
        arrow_marker.header.stamp = now
        arrow_marker.ns = "heading_indicator"
        arrow_marker.id = 3
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.scale.x = 2.0  # Arrow length
        arrow_marker.scale.y = 0.2  # Arrow width
        arrow_marker.scale.z = 0.2  # Arrow height
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 1.0
        arrow_marker.color.a = 1.0
        # Reduced marker lifetime for more frequent updates
        arrow_marker.lifetime.sec = 0
        arrow_marker.lifetime.nanosec = 500000000  # 0.5 seconds
        
        # Set arrow position and orientation
        arrow_marker.pose.position.x = 0.0
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 0.1  # Slightly above ground
        
        # Convert heading to quaternion (only yaw rotation)
        q = quaternion_from_euler(0.0, 0.0, heading)
        arrow_marker.pose.orientation.x = q[0]
        arrow_marker.pose.orientation.y = q[1]
        arrow_marker.pose.orientation.z = q[2]
        arrow_marker.pose.orientation.w = q[3]
        
        marker_array.markers.append(arrow_marker)
        
        return marker_array
    
    def publish_lane_center(self):
        """Publish the lane center point"""
        if not self.lane_center_points:
            return
        
        # Create point message
        point_msg = PointStamped()
        point_msg.header.frame_id = self.map_frame_id
        point_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Project lane center forward using compass heading
        heading = self.filtered_heading
        distance = 5.0  # 5 meters ahead
        point_msg.point.x = distance * math.cos(heading)
        point_msg.point.y = distance * math.sin(heading)
        point_msg.point.z = 0.0
        
        self.lane_center_publisher.publish(point_msg)
    
    def report_performance(self):
        """Report performance statistics"""
        now = time.time()
        elapsed = now - self.last_performance_time
        rate = self.process_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(f'Processing rate: {rate:.1f} Hz')
        
        # Improved logging for compass and heading values
        raw_compass_deg = math.degrees(self.compass) if abs(self.compass) <= (2 * math.pi) else self.compass
        self.get_logger().info(f'Raw compass value: {raw_compass_deg:.1f}° ({self.compass:.6f} rad)')
        
        self.get_logger().info(f'Raw yaw value: {math.degrees(self.yaw):.1f}°')
        self.get_logger().info(f'Normalized compass heading: {math.degrees(self.current_heading - self.compass_offset):.1f}°')
        self.get_logger().info(f'Current compass heading (with offset): {math.degrees(self.current_heading):.1f}°')
        self.get_logger().info(f'Filtered heading: {math.degrees(self.filtered_heading):.1f}°')
        self.get_logger().info(f'Heading smoothing factor: {self.heading_smoothing_factor}')
        self.get_logger().info(f'Heading filter size: {self.heading_filter_size}')
        self.get_logger().info(f'Heading rate limit: {self.heading_rate_limit} deg/s')
        
        self.process_count = 0
        self.last_performance_time = now
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def __del__(self):
        """Cleanup resources"""
        if self.socket:
            self.socket.close()
    
    def parameters_callback(self, params):
        """Handle parameter updates during runtime"""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'heading_smoothing_factor':
                if param.value < 0.0 or param.value > 1.0:
                    result.successful = False
                    result.reason = 'heading_smoothing_factor must be between 0.0 and 1.0'
                else:
                    self.heading_smoothing_factor = param.value
                    self.get_logger().info(f'Updated heading_smoothing_factor to {param.value}')
            elif param.name == 'heading_filter_size':
                if param.value < 1:
                    result.successful = False
                    result.reason = 'heading_filter_size must be at least 1'
                else:
                    old_size = self.heading_filter_size
                    self.heading_filter_size = param.value
                    # Reset buffer if size decreased
                    if param.value < old_size and len(self.heading_buffer) > param.value:
                        self.heading_buffer = self.heading_buffer[-param.value:]
                    self.get_logger().info(f'Updated heading_filter_size to {param.value}')
            elif param.name == 'heading_rate_limit':
                if param.value < 0.0:
                    result.successful = False
                    result.reason = 'heading_rate_limit must be non-negative'
                else:
                    self.heading_rate_limit = param.value
                    self.get_logger().info(f'Updated heading_rate_limit to {param.value} deg/s')
            elif param.name == 'compass_offset':
                self.compass_offset = param.value
                self.get_logger().info(f'Updated compass_offset to {param.value} radians ({math.degrees(param.value):.1f}°)')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ImuCameraLaneFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 