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

class ImuRvizVisualizer(Node):
    def __init__(self):
        super().__init__('imu_rviz_visualizer')
        
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
        
        # Create ROS2 IMU publisher for RViz2 visualization
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', qos_profile)
        self.get_logger().info(f'Publishing IMU data on topic: imu/data')
        
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
        print(f"{Colors.BOLD}{Colors.HEADER}IMU RViz2 Visualizer Node{Colors.ENDC}")
        print(f"Connecting to IMU server at {Colors.CYAN}{self.tcp_ip}:{self.tcp_port}{Colors.ENDC}")
        print(f"IMU frame: {Colors.GREEN}{self.frame_id}{Colors.ENDC}")
        print(f"Publishing IMU data for RViz2 visualization on topic: {Colors.CYAN}imu/data{Colors.ENDC}")
        print(f"Also publishing TF transform between {Colors.GREEN}{self.world_frame_id}{Colors.ENDC} and {Colors.GREEN}{self.frame_id}{Colors.ENDC}")
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
        """Process data from the TCP socket and visualize in RViz2"""
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
                
                # Publish the IMU data for RViz2
                self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll_rad, pitch_rad, yaw_rad)
                
                # Publish the TF transform for visualization
                self.publish_tf_transform(roll_rad, pitch_rad, yaw_rad)
                
                # Remove processed data from buffer
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error processing IMU data: {str(e)}{Colors.ENDC}')

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
        """Publish IMU data for RViz2 visualization"""
        try:
            # Create IMU message
            imu_msg = Imu()
            
            # Set header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Convert degrees/second to radians/second for gyroscope data
            # (though our data should already be in the right units)
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
        self.get_logger().info('IMU visualizer node shutting down')


def main(args=None):
    rclpy.init(args=args)
    
    imu_visualizer = ImuRvizVisualizer()
    
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