#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import socket
import struct
import time
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

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

class ImuTcpNode(Node):
    def __init__(self):
        super().__init__('imu_tcp_node')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12345)  # TCP port for IMU data
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('filter_window_size', 5)   # Window size for moving average filter
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        
        # TF broadcaster for publishing IMU transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create ROS2 IMU publisher for RViz2 visualization
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', qos_profile)
        self.get_logger().info(f'Publishing IMU orientation data on topic: imu/data')
        
        # TCP socket related
        self.socket = None
        self.connected = False
        self.buffer = b''
        
        # Counters and statistics
        self.packet_count = 0
        self.last_print_time = time.time()
        self.print_interval = 1.0  # seconds between prints
        
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
        
        # Data size constants - Extended format with 10 values: 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
        self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')  # 40 bytes
        
        # Print welcome message
        self.print_welcome_message()
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.process_data)  # 100Hz processing
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
    def print_welcome_message(self):
        self.get_logger().info("=" * 40)
        self.get_logger().info("IMU TCP Node")
        self.get_logger().info(f"Connecting to IMU server at {self.tcp_ip}:{self.tcp_port}")
        self.get_logger().info(f"IMU frame: {self.frame_id}")
        self.get_logger().info("=" * 40)
        
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
            
    def apply_moving_average(self, value, buffer):
        """Apply moving average filter to smooth sensor data"""
        buffer.append(value)
        if len(buffer) > self.filter_window_size:
            buffer.pop(0)
        return sum(buffer) / len(buffer)
    
    def process_data(self):
        """Process incoming IMU data"""
        if not self.connected:
            return
            
        try:
            # Try to receive data
            data = self.socket.recv(4096)
            if not data:
                self.get_logger().warning('Connection closed by server')
                self.connected = False
                return
                
            # Add to buffer
            self.buffer += data
            
            # Process complete packets in buffer
            while len(self.buffer) >= self.IMU_DATA_SIZE:
                # Extract one packet
                packet = self.buffer[:self.IMU_DATA_SIZE]
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
                # Parse the data
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, compass, roll, pitch, yaw = struct.unpack('ffffffffff', packet)
                
                # Apply moving average filter
                accel_x = self.apply_moving_average(accel_x, self.accel_x_buffer)
                accel_y = self.apply_moving_average(accel_y, self.accel_y_buffer)
                accel_z = self.apply_moving_average(accel_z, self.accel_z_buffer)
                gyro_x = self.apply_moving_average(gyro_x, self.gyro_x_buffer)
                gyro_y = self.apply_moving_average(gyro_y, self.gyro_y_buffer)
                gyro_z = self.apply_moving_average(gyro_z, self.gyro_z_buffer)
                roll = self.apply_moving_average(roll, self.roll_buffer)
                pitch = self.apply_moving_average(pitch, self.pitch_buffer)
                yaw = self.apply_moving_average(yaw, self.yaw_buffer)
                
                # Publish TF transform
                self.publish_tf_transform(roll, pitch, yaw)
                
                # Publish IMU data
                self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw)
                
                # Count processed packets
                self.packet_count += 1
                
                # Print statistics periodically
                current_time = time.time()
                if current_time - self.last_print_time > self.print_interval:
                    packets_per_second = self.packet_count / (current_time - self.last_print_time)
                    self.get_logger().info(f'IMU data rate: {packets_per_second:.1f} packets/second')
                    self.packet_count = 0
                    self.last_print_time = current_time
                
        except BlockingIOError:
            # No data available, not an error for non-blocking socket
            pass
        except ConnectionResetError:
            self.get_logger().error('Connection reset by server')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Error processing data: {str(e)}')
            self.connected = False
    
    def publish_tf_transform(self, roll, pitch, yaw):
        """Publish TF transform for the IMU orientation"""
        q = quaternion_from_euler(roll, pitch, yaw)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.frame_id
        
        # Set translation (position)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Set rotation (orientation)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
    
    def publish_imu_data(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw):
        """Publish IMU data message"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Set linear acceleration
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # Set angular velocity
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        
        # Set orientation (converted from euler angles)
        q = quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)
    
    def cleanup(self):
        """Clean up resources before node shutdown"""
        if self.socket:
            self.socket.close()
            self.get_logger().info('Closed IMU TCP connection')

def main(args=None):
    rclpy.init(args=args)
    node = ImuTcpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 