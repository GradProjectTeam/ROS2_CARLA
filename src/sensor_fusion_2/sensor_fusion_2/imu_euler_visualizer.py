#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Point
from tf2_ros import TransformBroadcaster
import numpy as np
import socket
import struct
import time
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import ColorRGBA
from threading import Lock
import collections  # For more sophisticated filtering

# Our own implementation of quaternion_from_euler to replace tf_transformations
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion based on the ZYX rotation sequence.
    This is the standard for ground vehicles where:
    - roll: rotation around X-axis (vehicle tilt left/right)
    - pitch: rotation around Y-axis (vehicle tilt forward/backward)
    - yaw: rotation around Z-axis (vehicle heading/direction)
    
    Args:
        roll: Rotation around X-axis in radians
        pitch: Rotation around Y-axis in radians
        yaw: Rotation around Z-axis in radians
        
    Returns:
        [x, y, z, w] quaternion
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    q = [0, 0, 0, 0]
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    
    return q

# Our own implementation of euler_from_quaternion
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

# ANSI color codes for prettier output
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Improved complementary filter for sensor fusion
class ComplementaryFilter:
    def __init__(self, alpha=0.98, dt=0.01):
        """
        Initialize complementary filter
        
        Args:
            alpha: Weight for gyroscope data (0-1)
            dt: Time step in seconds
        """
        self.alpha = alpha
        self.dt = dt
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.initialized = False
        
    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, compass=None):
        """
        Update filter with new sensor data
        
        Returns:
            roll, pitch, yaw in radians
        """
        # Calculate roll and pitch from accelerometer
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # Only use accelerometer if magnitude is close to gravity
        if 8.0 < accel_magnitude < 11.0:
            accel_roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
            accel_pitch = -math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2))
            
            if not self.initialized:
                # First measurement, initialize with accelerometer
                self.roll = accel_roll
                self.pitch = accel_pitch
                if compass is not None:
                    self.yaw = compass
                self.initialized = True
            else:
                # Complementary filter
                # Integrate gyro rates
                gyro_roll = self.roll + gyro_x * self.dt
                gyro_pitch = self.pitch + gyro_y * self.dt
                
                # Combine with accelerometer (complementary filter)
                self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
                self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        else:
            # High acceleration, rely more on gyro
            self.roll += gyro_x * self.dt
            self.pitch += gyro_y * self.dt
            
        # Update yaw from gyro or compass if available
        if compass is not None:
            # Use compass for absolute yaw reference
            # Apply complementary filter to blend with gyro
            gyro_yaw = self.yaw + gyro_z * self.dt
            self.yaw = 0.8 * gyro_yaw + 0.2 * compass
        else:
            # No compass, use gyro only
            self.yaw += gyro_z * self.dt
            
        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi
            
        return self.roll, self.pitch, self.yaw

class ImuEulerVisualizer(Node):
    def __init__(self):
        super().__init__('imu_euler_visualizer')
        
        # Declare parameters with existence check to prevent ParameterAlreadyDeclaredException
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.has_parameter('tcp_ip'):
            self.declare_parameter('tcp_ip', '127.0.0.1')
        if not self.has_parameter('tcp_port'):
            self.declare_parameter('tcp_port', 12345)  # TCP port for IMU data
        if not self.has_parameter('reconnect_interval'):
            self.declare_parameter('reconnect_interval', 5.0)
        if not self.has_parameter('frame_id'):
            self.declare_parameter('frame_id', 'imu_link')
        if not self.has_parameter('world_frame_id'):
            self.declare_parameter('world_frame_id', 'world')
        if not self.has_parameter('filter_window_size'):
            self.declare_parameter('filter_window_size', 5)
        if not self.has_parameter('queue_size'):
            self.declare_parameter('queue_size', 20)
        if not self.has_parameter('publish_rate'):
            self.declare_parameter('publish_rate', 200.0)  # Hz
        if not self.has_parameter('socket_buffer_size'):
            self.declare_parameter('socket_buffer_size', 65536)
        if not self.has_parameter('enable_bias_correction'):
            self.declare_parameter('enable_bias_correction', True)
        if not self.has_parameter('enable_complementary_filter'):
            self.declare_parameter('enable_complementary_filter', True)
        if not self.has_parameter('zero_velocity_threshold'):
            self.declare_parameter('zero_velocity_threshold', 0.02)  # m/s^2
        if not self.has_parameter('yaw_offset'):
            self.declare_parameter('yaw_offset', 0.0)  # Offset to align IMU with vehicle forward direction
        if not self.has_parameter('road_plane_correction'):
            self.declare_parameter('road_plane_correction', True)  # Correct for road plane
        if not self.has_parameter('gravity_aligned'):
            self.declare_parameter('gravity_aligned', True)  # Align with gravity
        if not self.has_parameter('vehicle_forward_axis'):
            self.declare_parameter('vehicle_forward_axis', 'x')
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.queue_size = self.get_parameter('queue_size').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.socket_buffer_size = self.get_parameter('socket_buffer_size').value
        self.enable_bias_correction = self.get_parameter('enable_bias_correction').value
        self.enable_complementary_filter = self.get_parameter('enable_complementary_filter').value
        self.zero_velocity_threshold = self.get_parameter('zero_velocity_threshold').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
        self.road_plane_correction = self.get_parameter('road_plane_correction').value
        self.gravity_aligned = self.get_parameter('gravity_aligned').value
        self.vehicle_forward_axis = self.get_parameter('vehicle_forward_axis').value.lower()
        
        # TF broadcaster for publishing IMU transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create standard IMU publisher for backward compatibility
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', qos_profile)
        
        # Create marker publisher for Euler angle visualization
        self.marker_publisher = self.create_publisher(MarkerArray, 'imu/euler_markers', qos_profile)
        
        self.get_logger().info(f'Publishing IMU Euler angle visualization on topic: imu/euler_markers')
        
        # TCP socket related
        self.socket = None
        self.connected = False
        self.buffer = b''
        
        # Counters and statistics
        self.packet_count = 0
        self.last_print_time = time.time()
        self.print_interval = 5.0  # seconds between status prints
        
        # Data size constants - expect 10 float values from C++ TCP server
        # 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
        self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')  # 40 bytes
        self.get_logger().info(f'{Colors.GREEN}Receiving IMU data format with 10 values (acc, gyro, compass, roll, pitch, yaw){Colors.ENDC}')
        
        # Initialize complementary filter
        self.comp_filter = ComplementaryFilter(alpha=0.98, dt=1.0/self.publish_rate)
        
        # Initialize data buffers for moving average filter
        # Using deque for efficient fixed-size buffer
        self.buffer_size = max(10, self.filter_window_size)
        self.accel_x_buffer = collections.deque(maxlen=self.buffer_size)
        self.accel_y_buffer = collections.deque(maxlen=self.buffer_size)
        self.accel_z_buffer = collections.deque(maxlen=self.buffer_size)
        self.gyro_x_buffer = collections.deque(maxlen=self.buffer_size)
        self.gyro_y_buffer = collections.deque(maxlen=self.buffer_size)
        self.gyro_z_buffer = collections.deque(maxlen=self.buffer_size)
        self.roll_buffer = collections.deque(maxlen=self.buffer_size)
        self.pitch_buffer = collections.deque(maxlen=self.buffer_size)
        self.yaw_buffer = collections.deque(maxlen=self.buffer_size)
        
        # Bias estimation
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.bias_samples = 0
        self.bias_calibrated = False

        # Print welcome message
        self.print_welcome_message()
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(1.0/self.publish_rate, self.process_data)  # Process at specified rate
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
        # Timer for status printing
        self.create_timer(self.print_interval, self.print_status)
        
    def print_welcome_message(self):
        print("\n" + "="*80)
        print(f"{Colors.BOLD}{Colors.HEADER}IMU Euler Angle Visualizer Node{Colors.ENDC}")
        print(f"Connecting to IMU server at {Colors.CYAN}{self.tcp_ip}:{self.tcp_port}{Colors.ENDC}")
        print(f"IMU frame: {Colors.GREEN}{self.frame_id}{Colors.ENDC}")
        print(f"Publishing Euler angle visualization markers on topic: {Colors.CYAN}imu/euler_markers{Colors.ENDC}")
        print(f"Also publishing standard IMU data on topic: {Colors.CYAN}imu/data{Colors.ENDC}")
        print(f"Publishing TF transform between {Colors.GREEN}{self.world_frame_id}{Colors.ENDC} and {Colors.GREEN}{self.frame_id}{Colors.ENDC}")
        print(f"Processing rate: {Colors.YELLOW}{self.publish_rate:.1f} Hz{Colors.ENDC}")
        print(f"Filter window size: {Colors.YELLOW}{self.filter_window_size}{Colors.ENDC}")
        print(f"Bias correction: {Colors.YELLOW}{self.enable_bias_correction}{Colors.ENDC}")
        print(f"Complementary filter: {Colors.YELLOW}{self.enable_complementary_filter}{Colors.ENDC}")
        print(f"Yaw offset: {Colors.YELLOW}{math.degrees(self.yaw_offset):.2f}°{Colors.ENDC}")
        print(f"Road plane correction: {Colors.YELLOW}{self.road_plane_correction}{Colors.ENDC}")
        print(f"Gravity aligned: {Colors.YELLOW}{self.gravity_aligned}{Colors.ENDC}")
        print(f"Vehicle forward axis: {Colors.YELLOW}{self.vehicle_forward_axis}{Colors.ENDC}")
        print("="*80 + "\n")
        
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
            
            # Set socket buffer size
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.socket_buffer_size)
            
            self.connected = True
            self.buffer = b''  # Clear buffer on new connection
            
            # Reset bias calibration on new connection
            self.bias_calibrated = False
            self.bias_samples = 0
            
            self.get_logger().info(f'{Colors.GREEN}Connected to IMU server at {self.tcp_ip}:{self.tcp_port}{Colors.ENDC}')
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f'{Colors.RED}Failed to connect to IMU server: {str(e)}{Colors.ENDC}')
            
    def check_connection(self):
        """Check if connection is still valid and reconnect if needed"""
        if not self.connected:
            self.get_logger().info('Attempting to reconnect to IMU server...')
            self.connect()
    
    def print_status(self):
        """Print periodic status information"""
        if self.connected:
            self.get_logger().info(f'Processed {self.packet_count} IMU packets so far')
            
    def apply_moving_average(self, value, buffer):
        """Apply moving average filter to smooth out noise"""
        buffer.append(value)
        if len(buffer) > 0:
            return sum(buffer) / len(buffer)
        return value
    
    def estimate_gyro_bias(self, gyro_x, gyro_y, gyro_z):
        """Estimate gyro bias during stationary periods"""
        if not self.enable_bias_correction:
            return gyro_x, gyro_y, gyro_z
            
        # Check if we're stationary based on gyro values
        gyro_magnitude = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
        
        if gyro_magnitude < self.zero_velocity_threshold:
            # We're likely stationary, update bias estimate
            if not self.bias_calibrated:
                # Still calibrating
                self.gyro_bias_x = ((self.bias_samples * self.gyro_bias_x) + gyro_x) / (self.bias_samples + 1)
                self.gyro_bias_y = ((self.bias_samples * self.gyro_bias_y) + gyro_y) / (self.bias_samples + 1)
                self.gyro_bias_z = ((self.bias_samples * self.gyro_bias_z) + gyro_z) / (self.bias_samples + 1)
                self.bias_samples += 1
                
                if self.bias_samples >= 100:  # After 100 samples, consider bias calibrated
                    self.bias_calibrated = True
                    self.get_logger().info(f"Gyro bias calibrated: x={self.gyro_bias_x:.6f}, y={self.gyro_bias_y:.6f}, z={self.gyro_bias_z:.6f}")
            else:
                # Already calibrated, just update with small weight
                self.gyro_bias_x = 0.99 * self.gyro_bias_x + 0.01 * gyro_x
                self.gyro_bias_y = 0.99 * self.gyro_bias_y + 0.01 * gyro_y
                self.gyro_bias_z = 0.99 * self.gyro_bias_z + 0.01 * gyro_z
                
        # Apply bias correction
        return gyro_x - self.gyro_bias_x, gyro_y - self.gyro_bias_y, gyro_z - self.gyro_bias_z
            
    def process_data(self):
        """Process data from the TCP socket and visualize Euler angles in RViz2"""
        if not self.connected:
            return
            
        try:
            # Try to receive data
            try:
                data = self.socket.recv(4096)
                if not data:
                    # Empty data means disconnected
                    self.connected = False
                    self.get_logger().warning(f'{Colors.RED}IMU server disconnected{Colors.ENDC}')
                    return
                self.buffer += data
            except BlockingIOError:
                # No data available right now, that's normal for non-blocking
                pass
            except Exception as e:
                self.connected = False
                self.get_logger().warning(f'{Colors.RED}IMU socket error: {str(e)}{Colors.ENDC}')
                return
                
            # Process complete IMU packets
            while len(self.buffer) >= self.IMU_DATA_SIZE:
                self.packet_count += 1
                
                # Parse all 10 values from the C++ application
                imu_values = struct.unpack('ffffffffff', self.buffer[:self.IMU_DATA_SIZE])
                
                # Extract data
                accel_x, accel_y, accel_z = imu_values[0:3]     # Accelerometer data
                gyro_x, gyro_y, gyro_z = imu_values[3:6]        # Gyroscope data
                compass = imu_values[6]                          # Compass value
                roll_deg, pitch_deg, yaw_deg = imu_values[7:10]  # Orientation in degrees
                
                # Apply bias correction to gyroscope data
                if self.enable_bias_correction:
                    gyro_x, gyro_y, gyro_z = self.estimate_gyro_bias(gyro_x, gyro_y, gyro_z)
                
                # Apply moving average filter to raw sensor data
                accel_x = self.apply_moving_average(accel_x, self.accel_x_buffer)
                accel_y = self.apply_moving_average(accel_y, self.accel_y_buffer)
                accel_z = self.apply_moving_average(accel_z, self.accel_z_buffer)
                gyro_x = self.apply_moving_average(gyro_x, self.gyro_x_buffer)
                gyro_y = self.apply_moving_average(gyro_y, self.gyro_y_buffer)
                gyro_z = self.apply_moving_average(gyro_z, self.gyro_z_buffer)
                
                # Convert orientation to radians for ROS2
                roll_rad = math.radians(roll_deg)
                pitch_rad = math.radians(pitch_deg)
                yaw_rad = math.radians(yaw_deg)
                
                # Convert compass to radians
                compass_rad = math.radians(compass)
                
                # Apply our own sensor fusion if enabled
                if self.enable_complementary_filter:
                    # Use complementary filter to fuse accelerometer and gyroscope data
                    roll_rad, pitch_rad, yaw_rad = self.comp_filter.update(
                        accel_x, accel_y, accel_z, 
                        gyro_x, gyro_y, gyro_z,
                        compass_rad
                    )
                
                # Apply yaw offset to align with vehicle forward direction
                yaw_rad = yaw_rad + self.yaw_offset
                
                # Normalize yaw to [-pi, pi]
                while yaw_rad > math.pi:
                    yaw_rad -= 2.0 * math.pi
                while yaw_rad < -math.pi:
                    yaw_rad += 2.0 * math.pi
                
                # Apply road plane correction if enabled
                if self.road_plane_correction:
                    # For vehicles, we often want to minimize roll and pitch
                    # to represent orientation relative to the road plane
                    roll_rad *= 0.8  # Reduce roll influence
                    pitch_rad *= 0.8  # Reduce pitch influence
                
                # Apply final moving average filter to orientation angles
                roll_rad = self.apply_moving_average(roll_rad, self.roll_buffer)
                pitch_rad = self.apply_moving_average(pitch_rad, self.pitch_buffer)
                yaw_rad = self.apply_moving_average(yaw_rad, self.yaw_buffer)
                
                # Convert back to degrees for visualization
                roll_deg = math.degrees(roll_rad)
                pitch_deg = math.degrees(pitch_rad)
                yaw_deg = math.degrees(yaw_rad)
                
                # Publish the standard IMU data for backward compatibility
                self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll_rad, pitch_rad, yaw_rad)
                
                # Publish Euler angle markers for direct visualization
                self.publish_euler_markers(roll_rad, pitch_rad, yaw_rad, roll_deg, pitch_deg, yaw_deg)
                
                # Publish the TF transform for visualization
                self.publish_tf_transform(roll_rad, pitch_rad, yaw_rad)
                
                # Remove processed data from buffer
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error processing IMU data: {str(e)}{Colors.ENDC}')

    def publish_euler_markers(self, roll_rad, pitch_rad, yaw_rad, roll_deg, pitch_deg, yaw_deg):
        """Publish markers to visualize Euler angles directly in RViz2"""
        try:
            # Create MarkerArray message
            marker_array = MarkerArray()
            
            # Get current time for all markers
            now = self.get_clock().now().to_msg()
            
            # Create text markers for displaying the Euler angle values
            text_marker = Marker()
            text_marker.header.frame_id = self.world_frame_id
            text_marker.header.stamp = now
            text_marker.ns = "imu_euler_text"
            text_marker.id = 0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text slightly above the IMU
            text_marker.pose.position.x = 0.0
            text_marker.pose.position.y = 0.0
            text_marker.pose.position.z = 0.3
            
            # Set orientation (doesn't matter for TEXT_VIEW_FACING)
            text_marker.pose.orientation.w = 1.0
            
            # Set text content with Euler angles
            text_marker.text = f"Roll: {roll_deg:.2f}°\nPitch: {pitch_deg:.2f}°\nYaw: {yaw_deg:.2f}°"
            
            # Set scale and color
            text_marker.scale.z = 0.1  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            
            # Add text marker to array
            marker_array.markers.append(text_marker)
            
            # Create arrow markers for each Euler angle axis
            
            # Roll arrow (Red, around X-axis)
            roll_marker = self.create_arrow_marker(
                frame_id=self.frame_id,
                stamp=now,
                ns="roll_arrow",
                id=1,
                start=Point(x=0.0, y=0.0, z=0.0),
                end=Point(x=0.2, y=0.0, z=0.0),
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                width=0.02
            )
            marker_array.markers.append(roll_marker)
            
            # Pitch arrow (Green, around Y-axis)
            pitch_marker = self.create_arrow_marker(
                frame_id=self.frame_id,
                stamp=now,
                ns="pitch_arrow",
                id=2,
                start=Point(x=0.0, y=0.0, z=0.0),
                end=Point(x=0.0, y=0.2, z=0.0),
                color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                width=0.02
            )
            marker_array.markers.append(pitch_marker)
            
            # Yaw arrow (Blue, around Z-axis)
            yaw_marker = self.create_arrow_marker(
                frame_id=self.frame_id,
                stamp=now,
                ns="yaw_arrow",
                id=3,
                start=Point(x=0.0, y=0.0, z=0.0),
                end=Point(x=0.0, y=0.0, z=0.2),
                color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
                width=0.02
            )
            marker_array.markers.append(yaw_marker)
            
            # Add angle indicator arcs for each Euler angle
            # These circular arcs show the current angle visually
            
            # Roll angle indicator (X-axis rotation)
            roll_angle_marker = self.create_angle_indicator(
                frame_id=self.world_frame_id,
                stamp=now,
                ns="roll_angle",
                id=4,
                angle=roll_rad,
                axis=0,  # X-axis
                color=ColorRGBA(r=1.0, g=0.3, b=0.3, a=0.8),
                radius=0.25
            )
            marker_array.markers.append(roll_angle_marker)
            
            # Pitch angle indicator (Y-axis rotation)
            pitch_angle_marker = self.create_angle_indicator(
                frame_id=self.world_frame_id,
                stamp=now,
                ns="pitch_angle",
                id=5,
                angle=pitch_rad,
                axis=1,  # Y-axis
                color=ColorRGBA(r=0.3, g=1.0, b=0.3, a=0.8),
                radius=0.25
            )
            marker_array.markers.append(pitch_angle_marker)
            
            # Yaw angle indicator (Z-axis rotation)
            yaw_angle_marker = self.create_angle_indicator(
                frame_id=self.world_frame_id,
                stamp=now,
                ns="yaw_angle",
                id=6,
                angle=yaw_rad,
                axis=2,  # Z-axis
                color=ColorRGBA(r=0.3, g=0.3, b=1.0, a=0.8),
                radius=0.25
            )
            marker_array.markers.append(yaw_angle_marker)
            
            # Publish the marker array
            self.marker_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing Euler markers: {str(e)}{Colors.ENDC}')

    def create_arrow_marker(self, frame_id, stamp, ns, id, start, end, color, width):
        """Helper function to create an arrow marker"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [start, end]
        marker.scale.x = width  # Shaft diameter
        marker.scale.y = width * 2.0  # Head diameter
        marker.scale.z = 0.0  # Not used for ARROW
        marker.color = color
        return marker
        
    def create_angle_indicator(self, frame_id, stamp, ns, id, angle, axis, color, radius):
        """Helper function to create an angle indicator marker (arc)"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Number of points to use for the arc
        num_points = 30
        
        # Create points for the arc
        for i in range(num_points + 1):
            # Calculate angle for this point (from 0 to the current angle)
            point_angle = (i / num_points) * angle
            
            # Calculate point position based on axis
            point = Point()
            
            if axis == 0:  # X-axis rotation (roll)
                point.x = 0.0
                point.y = radius * math.cos(point_angle)
                point.z = radius * math.sin(point_angle)
            elif axis == 1:  # Y-axis rotation (pitch)
                point.x = radius * math.cos(point_angle)
                point.y = 0.0
                point.z = radius * math.sin(point_angle)
            else:  # Z-axis rotation (yaw)
                point.x = radius * math.cos(point_angle)
                point.y = radius * math.sin(point_angle)
                point.z = 0.0
            
            marker.points.append(point)
        
        # Set scale and color
        marker.scale.x = 0.01  # Line width
        marker.color = color
        
        return marker

    def publish_tf_transform(self, roll, pitch, yaw):
        """Publish TF transform for visualization"""
        try:
            # Adjust orientation based on vehicle forward axis
            adjusted_yaw = yaw
            
            # This ensures the vehicle's forward direction aligns with the expected axis
            if self.vehicle_forward_axis == 'y':
                # If vehicle forward is Y-axis, rotate 90 degrees around Z
                adjusted_yaw += math.pi/2
            elif self.vehicle_forward_axis == '-x':
                # If vehicle forward is negative X-axis, rotate 180 degrees around Z
                adjusted_yaw += math.pi
            elif self.vehicle_forward_axis == '-y':
                # If vehicle forward is negative Y-axis, rotate -90 degrees around Z
                adjusted_yaw -= math.pi/2
            
            # Normalize adjusted yaw
            while adjusted_yaw > math.pi:
                adjusted_yaw -= 2.0 * math.pi
            while adjusted_yaw < -math.pi:
                adjusted_yaw += 2.0 * math.pi
            
            # Create quaternion from roll, pitch, adjusted yaw
            q = quaternion_from_euler(roll, pitch, adjusted_yaw)
            
            # Create and publish transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame_id
            t.child_frame_id = self.frame_id
            
            # Set translation (fixed position for visualization)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            
            # Set rotation
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            # Publish transform
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing TF transform: {str(e)}{Colors.ENDC}')

    def publish_imu_data(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw):
        """Publish standard IMU data for backward compatibility"""
        try:
            # Create IMU message
            imu_msg = Imu()
            
            # Set header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Convert degrees/second to radians/second for gyroscope data if needed
            gyro_x_rad = gyro_x
            gyro_y_rad = gyro_y
            gyro_z_rad = gyro_z
            
            # Adjust orientation based on vehicle forward axis
            adjusted_yaw = yaw
            
            # This ensures the vehicle's forward direction aligns with the expected axis
            if self.vehicle_forward_axis == 'y':
                # If vehicle forward is Y-axis, rotate 90 degrees around Z
                adjusted_yaw += math.pi/2
            elif self.vehicle_forward_axis == '-x':
                # If vehicle forward is negative X-axis, rotate 180 degrees around Z
                adjusted_yaw += math.pi
            elif self.vehicle_forward_axis == '-y':
                # If vehicle forward is negative Y-axis, rotate -90 degrees around Z
                adjusted_yaw -= math.pi/2
            
            # Normalize adjusted yaw
            while adjusted_yaw > math.pi:
                adjusted_yaw -= 2.0 * math.pi
            while adjusted_yaw < -math.pi:
                adjusted_yaw += 2.0 * math.pi
            
            # Set orientation from Euler angles with adjusted yaw
            q = quaternion_from_euler(roll, pitch, adjusted_yaw)
            imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            # Set orientation covariance (small values for good confidence)
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # Set angular velocity (gyroscope data)
            imu_msg.angular_velocity = Vector3(x=gyro_x_rad, y=gyro_y_rad, z=gyro_z_rad)
            
            # Set angular velocity covariance
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # Set linear acceleration
            imu_msg.linear_acceleration = Vector3(x=accel_x, y=accel_y, z=accel_z)
            
            # Set linear acceleration covariance
            imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # Publish IMU message
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing IMU data: {str(e)}{Colors.ENDC}')

    def cleanup(self):
        """Clean up resources when node is destroyed"""
        if self.socket:
            self.socket.close()
        self.get_logger().info('IMU Euler visualizer node shutting down')


def main(args=None):
    rclpy.init(args=args)
    
    imu_visualizer = ImuEulerVisualizer()
    
    try:
        rclpy.spin(imu_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        imu_visualizer.cleanup()
        imu_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 