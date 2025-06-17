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

class ImuEulerVisualizer(Node):
    def __init__(self):
        super().__init__('imu_euler_visualizer')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12345)  # TCP port for IMU data
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('socket_buffer_size', 65536)
        self.declare_parameter('yaw_offset', 0.0)  # Offset to align IMU with vehicle forward direction
        self.declare_parameter('vehicle_forward_axis', 'x')
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.socket_buffer_size = self.get_parameter('socket_buffer_size').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
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
        print(f"{Colors.BOLD}{Colors.HEADER}IMU Euler Angle Visualizer Node (RAW DATA MODE){Colors.ENDC}")
        print(f"Connecting to IMU server at {Colors.CYAN}{self.tcp_ip}:{self.tcp_port}{Colors.ENDC}")
        print(f"IMU frame: {Colors.GREEN}{self.frame_id}{Colors.ENDC}")
        print(f"Publishing Euler angle visualization markers on topic: {Colors.CYAN}imu/euler_markers{Colors.ENDC}")
        print(f"Also publishing standard IMU data on topic: {Colors.CYAN}imu/data{Colors.ENDC}")
        print(f"Publishing TF transform between {Colors.GREEN}{self.world_frame_id}{Colors.ENDC} and {Colors.GREEN}{self.frame_id}{Colors.ENDC}")
        print(f"Processing rate: {Colors.YELLOW}{self.publish_rate:.1f} Hz{Colors.ENDC}")
        print(f"Yaw offset: {Colors.YELLOW}{math.degrees(self.yaw_offset):.2f}°{Colors.ENDC}")
        print(f"Vehicle forward axis: {Colors.YELLOW}{self.vehicle_forward_axis}{Colors.ENDC}")
        print(f"{Colors.BOLD}{Colors.RED}RAW DATA MODE: Using raw compass values in DEGREES directly from TCP without any filtering{Colors.ENDC}")
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
            
    def calculate_orientation(self, compass_deg):
        """
        Get vehicle heading from compass data for a car in CARLA simulator
        
        The compass provides heading in degrees directly from the TCP connection.
        We convert it to radians for ROS quaternion creation.
        """
        # For simplicity, set roll and pitch to 0
        roll_rad = 0.0
        pitch_rad = 0.0
        
        # Convert compass from degrees to radians for ROS
        yaw_rad = math.radians(compass_deg)
            
        return roll_rad, pitch_rad, yaw_rad
    
    def normalize_angle(self, angle):
        """
        This function is kept for reference but not used anymore.
        We're using raw values directly without normalization.
        """
        return angle
            
    def process_data(self):
        """Process data from the TCP socket and visualize compass heading in RViz2"""
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
                compass = imu_values[6]                          # Raw compass value in DEGREES from C++
                cpp_roll_deg, cpp_pitch_deg, cpp_yaw_deg = imu_values[7:10]  # Orientation in degrees from C++
                
                # Get orientation directly from compass (in degrees)
                roll_rad, pitch_rad, yaw_rad = self.calculate_orientation(compass)
                
                # Log every 100 packets
                if self.packet_count % 100 == 0:
                    self.get_logger().info(f"Raw compass: {compass:.2f}° (converted to {yaw_rad:.4f} rad)")
                
                # Publish the standard IMU data with compass value converted to radians
                self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, yaw_rad)
                
                # Publish markers for visualization with compass value in degrees
                self.publish_euler_markers(compass)
                
                # Publish the TF transform for visualization with compass value converted to radians
                self.publish_tf_transform(yaw_rad)
                
                # Remove processed data from buffer
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error processing IMU data: {str(e)}{Colors.ENDC}')

    def publish_euler_markers(self, compass_deg):
        """Publish markers to visualize compass heading in RViz2"""
        try:
            # Create MarkerArray message
            marker_array = MarkerArray()
            
            # Get current time for all markers
            now = self.get_clock().now().to_msg()
            
            # Create text markers for displaying the compass heading
            text_marker = Marker()
            text_marker.header.frame_id = self.world_frame_id
            text_marker.header.stamp = now
            text_marker.ns = "compass_text"
            text_marker.id = 0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Position text slightly above the IMU
            text_marker.pose.position.x = 0.0
            text_marker.pose.position.y = 0.0
            text_marker.pose.position.z = 0.3
            
            # Set orientation (doesn't matter for TEXT_VIEW_FACING)
            text_marker.pose.orientation.w = 1.0
            
            # Set text content with raw compass heading in degrees
            text_marker.text = f"Raw Compass: {compass_deg:.1f}°"
            
            # Set scale and color
            text_marker.scale.z = 0.1  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            
            # Add text marker to array
            marker_array.markers.append(text_marker)
            
            # Add a direction arrow to show vehicle heading (forward direction)
            heading_marker = Marker()
            heading_marker.header.frame_id = self.frame_id
            heading_marker.header.stamp = now
            heading_marker.ns = "vehicle_heading"
            heading_marker.id = 1
            heading_marker.type = Marker.ARROW
            heading_marker.action = Marker.ADD
            heading_marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.8, y=0.0, z=0.0)]
            heading_marker.scale.x = 0.05  # Shaft diameter
            heading_marker.scale.y = 0.1   # Head diameter
            heading_marker.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=1.0)  # Green
            marker_array.markers.append(heading_marker)
            
            # Add a compass rose to show North direction (fixed in world frame)
            north_marker = Marker()
            north_marker.header.frame_id = self.world_frame_id
            north_marker.header.stamp = now
            north_marker.ns = "north_direction"
            north_marker.id = 2
            north_marker.type = Marker.ARROW
            north_marker.action = Marker.ADD
            north_marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=1.0, z=0.0)]
            north_marker.scale.x = 0.03  # Shaft diameter
            north_marker.scale.y = 0.06  # Head diameter
            north_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
            marker_array.markers.append(north_marker)
            
            # Add text label for North
            north_text = Marker()
            north_text.header.frame_id = self.world_frame_id
            north_text.header.stamp = now
            north_text.ns = "north_label"
            north_text.id = 3
            north_text.type = Marker.TEXT_VIEW_FACING
            north_text.action = Marker.ADD
            north_text.pose.position.x = 0.0
            north_text.pose.position.y = 1.1
            north_text.pose.position.z = 0.0
            north_text.pose.orientation.w = 1.0
            north_text.text = "N"
            north_text.scale.z = 0.1
            north_text.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker_array.markers.append(north_text)
            
            # Add East direction
            east_marker = Marker()
            east_marker.header.frame_id = self.world_frame_id
            east_marker.header.stamp = now
            east_marker.ns = "east_direction"
            east_marker.id = 4
            east_marker.type = Marker.ARROW
            east_marker.action = Marker.ADD
            east_marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=1.0, y=0.0, z=0.0)]
            east_marker.scale.x = 0.02  # Shaft diameter
            east_marker.scale.y = 0.04  # Head diameter
            east_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)  # Orange
            marker_array.markers.append(east_marker)
            
            # Add text label for East
            east_text = Marker()
            east_text.header.frame_id = self.world_frame_id
            east_text.header.stamp = now
            east_text.ns = "east_label"
            east_text.id = 5
            east_text.type = Marker.TEXT_VIEW_FACING
            east_text.action = Marker.ADD
            east_text.pose.position.x = 1.1
            east_text.pose.position.y = 0.0
            east_text.pose.position.z = 0.0
            east_text.pose.orientation.w = 1.0
            east_text.text = "E"
            east_text.scale.z = 0.1
            east_text.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
            marker_array.markers.append(east_text)
            
            # Publish the marker array
            self.marker_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error publishing compass markers: {str(e)}{Colors.ENDC}')

    def publish_tf_transform(self, yaw):
        """Publish TF transform for visualization of compass heading"""
        try:
            # Create quaternion from roll=0, pitch=0, and raw yaw
            q = quaternion_from_euler(0.0, 0.0, yaw)
            
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

    def publish_imu_data(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, yaw):
        """Publish standard IMU data with raw compass heading"""
        try:
            # Create IMU message
            imu_msg = Imu()
            
            # Set header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Create quaternion from roll=0, pitch=0, and raw yaw
            q = quaternion_from_euler(0.0, 0.0, yaw)
            imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            # Set orientation covariance (small values for good confidence)
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # Set angular velocity (gyroscope data)
            imu_msg.angular_velocity = Vector3(x=gyro_x, y=gyro_y, z=gyro_z)
            
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