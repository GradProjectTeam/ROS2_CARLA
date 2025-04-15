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

# Our own implementation of quaternion_from_euler to replace tf_transformations
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion based on the ZYX rotation sequence.
    
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
        self.declare_parameter('filter_window_size', 5)
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        
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
        
        # Initialize data buffers for moving average filter
        self.accel_x_buffer = []
        self.accel_y_buffer = []
        self.accel_z_buffer = []
        self.gyro_x_buffer = []
        self.gyro_y_buffer = []
        self.gyro_z_buffer = []
        self.roll_buffer = []
        self.pitch_buffer = []
        self.yaw_buffer = []

        # Print welcome message
        self.print_welcome_message()
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.process_data)  # 100Hz processing
        
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
            
    def apply_moving_average(self, value, buffer):
        """Apply moving average filter to smooth out noise"""
        buffer.append(value)
        if len(buffer) > self.filter_window_size:
            buffer.pop(0)
        return sum(buffer) / len(buffer)
            
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
                
                # Convert orientation to radians for ROS2
                roll_rad = math.radians(roll_deg)
                pitch_rad = math.radians(pitch_deg)
                yaw_rad = math.radians(yaw_deg)
                
                # Apply moving average filter to smooth data
                accel_x = self.apply_moving_average(accel_x, self.accel_x_buffer)
                accel_y = self.apply_moving_average(accel_y, self.accel_y_buffer)
                accel_z = self.apply_moving_average(accel_z, self.accel_z_buffer)
                gyro_x = self.apply_moving_average(gyro_x, self.gyro_x_buffer)
                gyro_y = self.apply_moving_average(gyro_y, self.gyro_y_buffer)
                gyro_z = self.apply_moving_average(gyro_z, self.gyro_z_buffer)
                roll_rad = self.apply_moving_average(roll_rad, self.roll_buffer)
                pitch_rad = self.apply_moving_average(pitch_rad, self.pitch_buffer)
                yaw_rad = self.apply_moving_average(yaw_rad, self.yaw_buffer)
                
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
            # Create quaternion from roll, pitch, yaw
            q = quaternion_from_euler(roll, pitch, yaw)
            
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
            
            # Set orientation from Euler angles
            q = quaternion_from_euler(roll, pitch, yaw)
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