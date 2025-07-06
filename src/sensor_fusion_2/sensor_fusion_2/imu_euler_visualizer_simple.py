#!/usr/bin/env python3
"""
IMU Euler Angle Visualizer for Autonomous Vehicle Navigation
=========================================================

Authors: Shishtawy & Hendy
Project: TechZ Autonomous Driving System

OVERVIEW:
This node provides real-time visualization of IMU orientation data in ROS2,
specifically designed for autonomous vehicle applications. It processes raw IMU
data from a TCP connection and converts it into meaningful visual representations
in RViz2.

KEY FEATURES:
- Real-time IMU data processing via TCP
- Euler angle visualization (roll, pitch, yaw)
- TF tree integration for coordinate frame management
- RViz2 marker-based visualization
- Standard IMU message publishing
- Robust connection handling with auto-reconnect
- Configurable visualization parameters

VISUALIZATION COMPONENTS:
1. Text display of raw compass heading
2. 3D arrow showing vehicle heading
3. Compass rose showing North direction
4. Cardinal direction markers (N, E)
5. TF transforms for coordinate frame integration

USAGE:
- Launch with default parameters:
  ros2 run sensor_fusion_2 imu_euler_visualizer_simple
- Configure TCP connection:
  ros2 run sensor_fusion_2 imu_euler_visualizer_simple --ros-args -p tcp_ip:=<IP> -p tcp_port:=<PORT>

This node is a critical component in the TechZ autonomous driving system,
providing essential orientation feedback for navigation and control.
"""

# Standard library imports for basic functionality
import rclpy                                                                   # ROS2 Python client library
from rclpy.node import Node                                                    # Base class for ROS2 nodes
from sensor_msgs.msg import Imu                                                # Standard IMU message type
from visualization_msgs.msg import Marker, MarkerArray                         # For RViz visualization
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Point     # Geometric message types
from tf2_ros import TransformBroadcaster                                       # For publishing transforms
import numpy as np                                                             # Numerical computations
import socket                                                                  # TCP/IP communication
import struct                                                                  # Binary data handling
import time                                                                    # Time utilities
import math                                                                    # Mathematical functions
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy      # Quality of Service settings
from std_msgs.msg import ColorRGBA                                            # Color message type
from threading import Lock                                                     # Thread synchronization
import collections                                                             # For data structures

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
    # Calculate trigonometric components for quaternion conversion
    cy = np.cos(yaw * 0.5)                                                    # Cosine of half yaw
    sy = np.sin(yaw * 0.5)                                                    # Sine of half yaw
    cp = np.cos(pitch * 0.5)                                                  # Cosine of half pitch
    sp = np.sin(pitch * 0.5)                                                  # Sine of half pitch
    cr = np.cos(roll * 0.5)                                                   # Cosine of half roll
    sr = np.sin(roll * 0.5)                                                   # Sine of half roll
    
    # Calculate quaternion components using standard conversion formula
    q = [0, 0, 0, 0]                                                          # Initialize quaternion array
    q[0] = sr * cp * cy - cr * sp * sy                                        # x component
    q[1] = cr * sp * cy + sr * cp * sy                                        # y component
    q[2] = cr * cp * sy - sr * sp * cy                                        # z component
    q[3] = cr * cp * cy + sr * sp * sy                                        # w component
    
    return q

# ANSI color codes for terminal output formatting
class Colors:
    HEADER = '\033[95m'                                                       # Purple header text
    BLUE = '\033[94m'                                                         # Blue text
    CYAN = '\033[96m'                                                         # Cyan text
    GREEN = '\033[92m'                                                        # Green text
    YELLOW = '\033[93m'                                                       # Yellow text
    RED = '\033[91m'                                                          # Red text for errors
    ENDC = '\033[0m'                                                          # Reset color
    BOLD = '\033[1m'                                                          # Bold text
    UNDERLINE = '\033[4m'                                                     # Underlined text

class ImuEulerVisualizer(Node):
    def __init__(self):
        super().__init__('imu_euler_visualizer')                              # Initialize ROS2 node
        
        # Declare ROS2 parameters with default values
        self.declare_parameter('tcp_ip', '127.0.0.1')                         # TCP server IP
        self.declare_parameter('tcp_port', 12345)                             # TCP server port
        self.declare_parameter('reconnect_interval', 5.0)                     # Seconds between reconnection attempts
        self.declare_parameter('frame_id', 'imu_link')                        # IMU frame name
        self.declare_parameter('world_frame_id', 'world')                     # World frame name
        self.declare_parameter('publish_rate', 100.0)                         # Publishing frequency in Hz
        self.declare_parameter('socket_buffer_size', 65536)                   # TCP socket buffer size
        self.declare_parameter('yaw_offset', 0.0)                             # Heading offset correction
        self.declare_parameter('vehicle_forward_axis', 'x')                   # Vehicle's forward direction axis
        
        # Get parameter values
        self.tcp_ip = self.get_parameter('tcp_ip').value                      # Get TCP IP from parameter
        self.tcp_port = self.get_parameter('tcp_port').value                  # Get TCP port from parameter
        self.reconnect_interval = self.get_parameter('reconnect_interval').value  # Get reconnection interval
        self.frame_id = self.get_parameter('frame_id').value                  # Get IMU frame ID
        self.world_frame_id = self.get_parameter('world_frame_id').value      # Get world frame ID
        self.publish_rate = self.get_parameter('publish_rate').value          # Get publishing rate
        self.socket_buffer_size = self.get_parameter('socket_buffer_size').value  # Get socket buffer size
        self.yaw_offset = self.get_parameter('yaw_offset').value              # Get yaw offset
        self.vehicle_forward_axis = self.get_parameter('vehicle_forward_axis').value.lower()  # Get forward axis
        
        # Initialize TF broadcaster for coordinate transforms
        self.tf_broadcaster = TransformBroadcaster(self)                      # Create transform broadcaster
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(                                             # Configure Quality of Service
            reliability=QoSReliabilityPolicy.RELIABLE,                        # Ensure reliable delivery
            history=QoSHistoryPolicy.KEEP_LAST,                               # Keep only latest messages
            depth=10                                                          # Buffer size
        )
        
        # Create publishers for IMU data and visualization
        self.imu_publisher = self.create_publisher(                           # Create IMU message publisher
            Imu, 'imu/data', qos_profile)                                     # Standard IMU topic
        self.marker_publisher = self.create_publisher(                        # Create visualization marker publisher
            MarkerArray, 'imu/euler_markers', qos_profile)                    # Custom markers topic
        
        self.get_logger().info(                                              # Log initialization message
            f'Publishing IMU Euler angle visualization on topic: imu/euler_markers')
        
        # Initialize TCP socket variables
        self.socket = None                                                    # TCP socket object
        self.connected = False                                                # Connection status flag
        self.buffer = b''                                                     # Data buffer for TCP stream
        
        # Initialize statistics counters
        self.packet_count = 0                                                 # Count received packets
        self.last_print_time = time.time()                                    # Time of last status print
        self.print_interval = 5.0                                             # Status print interval
        
        # Set data format constants
        self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')                    # Size of IMU data packet (10 floats)
        self.get_logger().info(                                              # Log data format info
            f'{Colors.GREEN}Receiving IMU data format with 10 values (acc, gyro, compass, roll, pitch, yaw){Colors.ENDC}')

        # Print welcome message and setup information
        self.print_welcome_message()                                          # Display startup information
        
        # Initialize connection and timers
        self.connect()                                                        # Attempt initial connection
        self.create_timer(1.0/self.publish_rate, self.process_data)          # Create data processing timer
        self.create_timer(1.0, self.check_connection)                         # Create connection check timer
        self.create_timer(self.print_interval, self.print_status)            # Create status print timer
        
    def print_welcome_message(self):
        """Display formatted welcome message with configuration details"""
        print("\n" + "="*80)                                                   # Print separator line
        print(f"{Colors.BOLD}{Colors.HEADER}IMU Euler Angle Visualizer Node (RAW DATA MODE){Colors.ENDC}")  # Print title
        print(f"Connecting to IMU server at {Colors.CYAN}{self.tcp_ip}:{self.tcp_port}{Colors.ENDC}")  # Connection info
        print(f"IMU frame: {Colors.GREEN}{self.frame_id}{Colors.ENDC}")       # IMU frame ID
        print(f"Publishing Euler angle visualization markers on topic: {Colors.CYAN}imu/euler_markers{Colors.ENDC}")  # Output topic
        print(f"Also publishing standard IMU data on topic: {Colors.CYAN}imu/data{Colors.ENDC}")  # Standard IMU topic
        print(f"Publishing TF transform between {Colors.GREEN}{self.world_frame_id}{Colors.ENDC} and {Colors.GREEN}{self.frame_id}{Colors.ENDC}")  # Transform info
        print(f"Processing rate: {Colors.YELLOW}{self.publish_rate:.1f} Hz{Colors.ENDC}")  # Update frequency
        print(f"Yaw offset: {Colors.YELLOW}{math.degrees(self.yaw_offset):.2f}°{Colors.ENDC}")  # Heading correction
        print(f"Vehicle forward axis: {Colors.YELLOW}{self.vehicle_forward_axis}{Colors.ENDC}")  # Forward direction
        print(f"{Colors.BOLD}{Colors.RED}RAW DATA MODE: Using raw compass values in DEGREES directly from TCP without any filtering{Colors.ENDC}")  # Mode info
        print("="*80 + "\n")                                                  # Print separator line
        
    def connect(self):
        """Attempt to establish TCP connection with IMU server"""
        if self.connected:                                                    # Skip if already connected
            return
            
        try:
            if self.socket:                                                   # Close existing socket if any
                self.socket.close()
                
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # Create TCP socket
            self.socket.settimeout(1.0)                                       # Set 1-second timeout
            self.socket.connect((self.tcp_ip, self.tcp_port))                 # Connect to server
            self.socket.setblocking(False)                                    # Set non-blocking mode
            
            # Configure socket buffer size for performance
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.socket_buffer_size)  # Set receive buffer
            
            self.connected = True                                             # Mark as connected
            self.buffer = b''                                                 # Clear data buffer
            
            self.get_logger().info(f'{Colors.GREEN}Connected to IMU server at {self.tcp_ip}:{self.tcp_port}{Colors.ENDC}')
        except Exception as e:
            self.connected = False                                            # Mark as disconnected
            self.get_logger().warning(f'{Colors.RED}Failed to connect to IMU server: {str(e)}{Colors.ENDC}')
            
    def check_connection(self):
        """Periodically check connection status and reconnect if needed"""
        if not self.connected:                                               # Check if disconnected
            self.get_logger().info('Attempting to reconnect to IMU server...')
            self.connect()                                                    # Try to reconnect
    
    def print_status(self):
        """Print periodic status information about received packets"""
        if self.connected:                                                   # Only print if connected
            self.get_logger().info(f'Processed {self.packet_count} IMU packets so far')
            
    def calculate_orientation(self, compass_deg):
        """
        Calculate vehicle orientation from compass data for CARLA simulator
        
        Args:
            compass_deg: Raw compass heading in degrees from TCP
            
        Returns:
            tuple: (roll, pitch, yaw) in radians
        """
        roll_rad = 0.0                                                       # Set roll to 0 (no tilt)
        pitch_rad = 0.0                                                      # Set pitch to 0 (no tilt)
        yaw_rad = math.radians(compass_deg)                                 # Convert compass to radians
            
        return roll_rad, pitch_rad, yaw_rad
    
    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π] range (kept for reference)
        
        Args:
            angle: Input angle in radians
            
        Returns:
            float: Normalized angle in radians
        """
        return angle                                                        # Return as is (no normalization)
            
    def process_data(self):
        """Process incoming IMU data and publish visualizations"""
        if not self.connected:                                              # Skip if not connected
            return
            
        try:
            # Receive data from socket
            try:
                data = self.socket.recv(4096)                               # Read up to 4KB of data
                if not data:                                                # Check for disconnection
                    self.connected = False
                    self.get_logger().warning(f'{Colors.RED}IMU server disconnected{Colors.ENDC}')
                    return
                self.buffer += data                                         # Append to buffer
            except BlockingIOError:                                         # Handle non-blocking socket
                pass                                                        # No data available
            except Exception as e:                                          # Handle other errors
                self.connected = False
                self.get_logger().warning(f'{Colors.RED}IMU socket error: {str(e)}{Colors.ENDC}')
                return
                
            # Process complete packets from buffer
            while len(self.buffer) >= self.IMU_DATA_SIZE:                  # Check for complete packet
                self.packet_count += 1                                      # Increment packet counter
                
                # Parse IMU data packet (10 float values)
                imu_values = struct.unpack('ffffffffff', self.buffer[:self.IMU_DATA_SIZE])  # Unpack binary data
                
                # Extract individual sensor readings
                accel_x, accel_y, accel_z = imu_values[0:3]                # Accelerometer data (m/s²)
                gyro_x, gyro_y, gyro_z = imu_values[3:6]                   # Gyroscope data (rad/s)
                compass = imu_values[6]                                     # Raw compass heading (degrees)
                cpp_roll_deg, cpp_pitch_deg, cpp_yaw_deg = imu_values[7:10]  # Orientation from C++ (degrees)
                
                # Calculate orientation from compass
                roll_rad, pitch_rad, yaw_rad = self.calculate_orientation(compass)  # Get orientation in radians
                
                # Log every 100 packets for monitoring
                if self.packet_count % 100 == 0:                           # Periodic logging
                    self.get_logger().info(f"Raw compass: {compass:.2f}° (converted to {yaw_rad:.4f} rad)")
                
                # Publish data to ROS topics
                self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, yaw_rad)  # Standard IMU message
                self.publish_euler_markers(compass)                         # Visualization markers
                self.publish_tf_transform(yaw_rad)                         # Coordinate transform
                
                # Remove processed data from buffer
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]             # Advance buffer
                
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error processing IMU data: {str(e)}{Colors.ENDC}')

    def publish_euler_markers(self, compass_deg):
        """
        Publish visualization markers for compass heading
        
        Args:
            compass_deg: Compass heading in degrees
        """
        try:
            # Create marker array message
            marker_array = MarkerArray()                                    # Container for all markers
            
            # Get current time for marker timestamps
            now = self.get_clock().now().to_msg()                          # Current ROS time
            
            # Create text marker for compass heading display
            text_marker = Marker()                                         # Initialize text marker
            text_marker.header.frame_id = self.world_frame_id              # Set reference frame
            text_marker.header.stamp = now                                 # Set timestamp
            text_marker.ns = "compass_text"                                # Namespace for organization
            text_marker.id = 0                                             # Unique identifier
            text_marker.type = Marker.TEXT_VIEW_FACING                     # Always face camera
            text_marker.action = Marker.ADD                                # Add/modify marker
            
            # Position text above IMU
            text_marker.pose.position.x = 0.0                              # Center X
            text_marker.pose.position.y = 0.0                              # Center Y
            text_marker.pose.position.z = 0.3                              # Slightly above IMU
            
            # Set text orientation
            text_marker.pose.orientation.w = 1.0                           # No rotation
            
            # Set text content with compass heading
            text_marker.text = f"Raw Compass: {compass_deg:.1f}°"          # Display heading
            
            # Configure text appearance
            text_marker.scale.z = 0.1                                      # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)     # White color
            
            # Add text marker to array
            marker_array.markers.append(text_marker)                       # Add to collection
            
            # Create arrow marker for vehicle heading
            heading_marker = Marker()                                      # Initialize arrow marker
            heading_marker.header.frame_id = self.frame_id                 # Set reference frame
            heading_marker.header.stamp = now                              # Set timestamp
            heading_marker.ns = "vehicle_heading"                          # Namespace
            heading_marker.id = 1                                          # Unique identifier
            heading_marker.type = Marker.ARROW                             # Arrow shape
            heading_marker.action = Marker.ADD                             # Add/modify marker
            heading_marker.points = [Point(x=0.0, y=0.0, z=0.0),          # Arrow start point
                                   Point(x=0.8, y=0.0, z=0.0)]            # Arrow end point
            heading_marker.scale.x = 0.05                                  # Shaft diameter
            heading_marker.scale.y = 0.1                                   # Head diameter
            heading_marker.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)  # Green color
            marker_array.markers.append(heading_marker)                    # Add to collection
            
            # Create arrow marker for North direction
            north_marker = Marker()                                        # Initialize North marker
            north_marker.header.frame_id = self.world_frame_id             # Set reference frame
            north_marker.header.stamp = now                                # Set timestamp
            north_marker.ns = "north_direction"                            # Namespace
            north_marker.id = 2                                            # Unique identifier
            north_marker.type = Marker.ARROW                               # Arrow shape
            north_marker.action = Marker.ADD                               # Add/modify marker
            north_marker.points = [Point(x=0.0, y=0.0, z=0.0),            # Arrow start point
                                 Point(x=0.0, y=1.0, z=0.0)]              # Arrow end point
            north_marker.scale.x = 0.03                                    # Shaft diameter
            north_marker.scale.y = 0.06                                    # Head diameter
            north_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)    # Red color
            marker_array.markers.append(north_marker)                      # Add to collection
            
            # Create text label for North
            north_text = Marker()                                          # Initialize North label
            north_text.header.frame_id = self.world_frame_id               # Set reference frame
            north_text.header.stamp = now                                  # Set timestamp
            north_text.ns = "north_label"                                  # Namespace
            north_text.id = 3                                              # Unique identifier
            north_text.type = Marker.TEXT_VIEW_FACING                      # Always face camera
            north_text.action = Marker.ADD                                 # Add/modify marker
            north_text.pose.position.x = 0.0                               # Center X
            north_text.pose.position.y = 1.1                               # Above North arrow
            north_text.pose.position.z = 0.0                               # Ground level
            north_text.pose.orientation.w = 1.0                            # No rotation
            north_text.text = "N"                                          # North label
            north_text.scale.z = 0.1                                       # Text height
            north_text.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)      # Red color
            marker_array.markers.append(north_text)                        # Add to collection
            
            # Create arrow marker for East direction
            east_marker = Marker()                                         # Initialize East marker
            east_marker.header.frame_id = self.world_frame_id              # Set reference frame
            east_marker.header.stamp = now                                 # Set timestamp
            east_marker.ns = "east_direction"                              # Namespace
            east_marker.id = 4                                             # Unique identifier
            east_marker.type = Marker.ARROW                                # Arrow shape
            east_marker.action = Marker.ADD                                # Add/modify marker
            east_marker.points = [Point(x=0.0, y=0.0, z=0.0),             # Arrow start point
                                Point(x=1.0, y=0.0, z=0.0)]               # Arrow end point
            east_marker.scale.x = 0.02                                     # Shaft diameter
            east_marker.scale.y = 0.04                                     # Head diameter
            east_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)     # Orange color
            marker_array.markers.append(east_marker)                       # Add to collection
            
            # Create text label for East
            east_text = Marker()                                           # Initialize East label
            east_text.header.frame_id = self.world_frame_id                # Set reference frame
            east_text.header.stamp = now                                   # Set timestamp
            east_text.ns = "east_label"                                    # Namespace
            east_text.id = 5                                               # Unique identifier
            east_text.type = Marker.TEXT_VIEW_FACING                       # Always face camera
            east_text.action = Marker.ADD                                  # Add/modify marker
            east_text.pose.position.x = 1.1                                # Right of East arrow
            east_text.pose.position.y = 0.0                                # Center Y
            east_text.pose.position.z = 0.0                                # Ground level
            east_text.pose.orientation.w = 1.0                             # No rotation
            east_text.text = "E"                                           # East label
            east_text.scale.z = 0.1                                        # Text height
            east_text.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)       # Orange color
            marker_array.markers.append(east_text)                         # Add to collection
            
            # Publish all markers
            self.marker_publisher.publish(marker_array)                    # Send to RViz
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing compass markers: {str(e)}{Colors.ENDC}')

    def publish_tf_transform(self, yaw):
        """
        Publish TF transform for IMU orientation
        
        Args:
            yaw: Yaw angle in radians
        """
        try:
            # Create quaternion from IMU orientation
            q = quaternion_from_euler(0.0, 0.0, yaw)                      # Convert to quaternion
            
            # Create transform message
            t = TransformStamped()                                        # Initialize transform
            t.header.stamp = self.get_clock().now().to_msg()              # Set timestamp
            t.header.frame_id = self.world_frame_id                       # Parent frame
            t.child_frame_id = self.frame_id                              # Child frame
            
            # Set transform translation (fixed position)
            t.transform.translation.x = 0.0                               # Center X
            t.transform.translation.y = 0.0                               # Center Y
            t.transform.translation.z = 0.0                               # Ground level
            
            # Set transform rotation
            t.transform.rotation.x = q[0]                                 # Quaternion X
            t.transform.rotation.y = q[1]                                 # Quaternion Y
            t.transform.rotation.z = q[2]                                 # Quaternion Z
            t.transform.rotation.w = q[3]                                 # Quaternion W
            
            # Publish transform
            self.tf_broadcaster.sendTransform(t)                          # Send to TF tree
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing TF transform: {str(e)}{Colors.ENDC}')

    def publish_imu_data(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, yaw):
        """
        Publish standard IMU message
        
        Args:
            accel_x, accel_y, accel_z: Linear acceleration (m/s²)
            gyro_x, gyro_y, gyro_z: Angular velocity (rad/s)
            yaw: Yaw angle in radians
        """
        try:
            # Create IMU message
            imu_msg = Imu()                                               # Initialize IMU message
            
            # Set header
            imu_msg.header.stamp = self.get_clock().now().to_msg()        # Set timestamp
            imu_msg.header.frame_id = self.frame_id                       # Set frame ID
            
            # Set orientation from yaw angle
            q = quaternion_from_euler(0.0, 0.0, yaw)                     # Convert to quaternion
            imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])  # Set orientation
            
            # Set orientation covariance (confidence in measurement)
            # Make sure this is exactly 9 float values
            imu_msg.orientation_covariance = np.array([0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01], dtype=np.float64)  # Low uncertainty
            
            # Set angular velocity
            imu_msg.angular_velocity = Vector3(x=gyro_x, y=gyro_y, z=gyro_z)  # Gyroscope data
            
            # Set angular velocity covariance
            imu_msg.angular_velocity_covariance = np.array([0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01], dtype=np.float64)  # Low uncertainty
            
            # Set linear acceleration
            imu_msg.linear_acceleration = Vector3(x=accel_x, y=accel_y, z=accel_z)  # Accelerometer data
            
            # Set linear acceleration covariance
            imu_msg.linear_acceleration_covariance = np.array([0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01], dtype=np.float64)  # Low uncertainty
            
            # Publish IMU message
            self.imu_publisher.publish(imu_msg)                           # Send to ROS
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing IMU data: {str(e)}{Colors.ENDC}')
            import traceback
            self.get_logger().error(traceback.format_exc())              # Print full stack trace

    def cleanup(self):
        """Clean up resources when node is destroyed"""
        if self.socket:                                                  # Check for active socket
            self.socket.close()                                          # Close TCP connection
        self.get_logger().info('IMU Euler visualizer node shutting down')  # Log shutdown


def main(args=None):
    """Main entry point for the IMU visualizer node"""
    rclpy.init(args=args)                                               # Initialize ROS2
    
    imu_visualizer = ImuEulerVisualizer()                              # Create node instance
    
    try:
        rclpy.spin(imu_visualizer)                                      # Start node
    except KeyboardInterrupt:
        pass                                                            # Handle clean shutdown
    finally:
        imu_visualizer.cleanup()                                        # Clean up resources
        imu_visualizer.destroy_node()                                   # Destroy node
        rclpy.shutdown()                                                # Shutdown ROS2


if __name__ == '__main__':
    main()                                                             # Run main function 