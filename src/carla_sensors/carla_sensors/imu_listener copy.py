#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from tf_transformations import quaternion_from_euler
import socket
import struct
import numpy as np
import time
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImuClient(Node):
    def __init__(self):
        super().__init__('imu_client')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12331)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('extended_format', True)
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.extended_format = self.get_parameter('extended_format').value
        
        # Setup publisher with reliable QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher = self.create_publisher(Imu, '/imu/data', qos_profile)
        
        # TCP socket related
        self.socket = None
        self.connected = False
        self.buffer = b''
        
        # Data size constants - support both formats
        if self.extended_format:
            # Extended format with 10 values: 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
            self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')
            self.get_logger().info('Using extended IMU format (10 values including roll/pitch/yaw)')
        else:
            # Basic format with 7 values: 3 accel, 3 gyro, 1 compass
            self.IMU_DATA_SIZE = struct.calcsize('fffffff')
            self.get_logger().info('Using basic IMU format (7 values)')
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.process_data)  # 100Hz processing
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
        self.get_logger().info('IMU Client initialized')
        
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
            self.get_logger().info(f'Attempting to reconnect to IMU server...')
            self.connect()
            
    def process_data(self):
        """Process data from the TCP socket and publish IMU messages"""
        if not self.connected:
            return
            
        try:
            # Try to receive data
            try:
                data = self.socket.recv(4096)
                if not data:
                    # Empty data means disconnected
                    self.connected = False
                    self.get_logger().warning('IMU server disconnected')
                    return
                self.buffer += data
            except BlockingIOError:
                # No data available right now, that's normal for non-blocking
                pass
            except Exception as e:
                self.connected = False
                self.get_logger().warning(f'IMU socket error: {str(e)}')
                return
                
            # Process complete IMU packets
            while len(self.buffer) >= self.IMU_DATA_SIZE:
                # Extract a complete IMU data packet
                if self.extended_format:
                    # Parse 10 values: 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
                    imu_values = struct.unpack('ffffffffff', self.buffer[:self.IMU_DATA_SIZE])
                    accel_x, accel_y, accel_z = imu_values[0:3]  # Accelerometer data
                    gyro_x, gyro_y, gyro_z = imu_values[3:6]     # Gyroscope data
                    compass = imu_values[6]                       # Compass
                    roll = imu_values[7]                          # Roll
                    pitch = imu_values[8]                         # Pitch
                    yaw = imu_values[9]                           # Yaw
                    
                    # Use the provided orientation
                    self.publish_imu_data_with_orientation(accel_x, accel_y, accel_z, 
                                                           gyro_x, gyro_y, gyro_z, 
                                                           roll, pitch, yaw)
                else:
                    # Parse 7 values: 3 accel, 3 gyro, 1 compass
                    imu_data = struct.unpack('fffffff', self.buffer[:self.IMU_DATA_SIZE])
                    
                    # Parse the IMU data (3 accel, 3 gyro, 1 compass)
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, compass = imu_data
                    
                    # Create and publish ROS message
                    self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, compass)
                    
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
            
    def publish_imu_data(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, compass):
        """Create and publish an IMU message using only compass for orientation"""
        msg = Imu()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Set linear acceleration
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        
        # Set angular velocity (gyroscope)
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        
        # Calculate orientation quaternion from compass
        # Note: This is a simplified conversion, might need refinement based on how compass data is provided
        # In CARLA, compass is in radians from north (0) going clockwise
        roll = 0.0  # Assuming no roll
        pitch = 0.0  # Assuming no pitch
        yaw = compass  # Yaw from compass 
        
        quat = quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        
        # Set covariances (optional, set to unknown)
        # In a real system, these would be calibrated values
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Publish the message
        self.publisher.publish(msg)
        
    def publish_imu_data_with_orientation(self, accel_x, accel_y, accel_z, 
                                        gyro_x, gyro_y, gyro_z, 
                                        roll, pitch, yaw):
        """Create and publish an IMU message with explicit orientation values"""
        msg = Imu()
        
        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Set linear acceleration
        msg.linear_acceleration.x = accel_x
        msg.linear_acceleration.y = accel_y
        msg.linear_acceleration.z = accel_z
        
        # Set angular velocity (gyroscope)
        msg.angular_velocity.x = gyro_x
        msg.angular_velocity.y = gyro_y
        msg.angular_velocity.z = gyro_z
        
        # Set orientation quaternion from provided roll, pitch, yaw
        quat = quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        
        # Set covariances (optional, set to unknown)
        # In a real system, these would be calibrated values
        msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Publish the message
        self.publisher.publish(msg)
        
    def cleanup(self):
        """Clean up resources"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            
def main(args=None):
    rclpy.init(args=args)
    
    imu_client = ImuClient()
    
    try:
        rclpy.spin(imu_client)
    except KeyboardInterrupt:
        pass
    finally:
        imu_client.cleanup()
        imu_client.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()