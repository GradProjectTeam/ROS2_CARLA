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

# Create a rotation matrix from roll, pitch, yaw (in radians)
def rotation_matrix_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to 3x3 rotation matrix.
    
    Args:
        roll: Rotation around X-axis in radians
        pitch: Rotation around Y-axis in radians
        yaw: Rotation around Z-axis in radians
        
    Returns:
        3x3 rotation matrix as numpy array
    """
    # Roll (X-axis rotation)
    c_r, s_r = np.cos(roll), np.sin(roll)
    R_x = np.array([[1, 0, 0], [0, c_r, -s_r], [0, s_r, c_r]])
    
    # Pitch (Y-axis rotation)
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    R_y = np.array([[c_p, 0, s_p], [0, 1, 0], [-s_p, 0, c_p]])
    
    # Yaw (Z-axis rotation)
    c_y, s_y = np.cos(yaw), np.sin(yaw)
    R_z = np.array([[c_y, -s_y, 0], [s_y, c_y, 0], [0, 0, 1]])
    
    # Combined rotation (ZYX order - yaw, then pitch, then roll)
    R = R_z @ R_y @ R_x
    
    return R

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

class ImuLidarFusion(Node):
    def __init__(self):
        super().__init__('imu_lidar_fusion')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12345)  # TCP port for IMU data
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('lidar_frame_id', 'lidar_link')
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('extended_format', True)  # Keep this True to receive extended format
        self.declare_parameter('filter_window_size', 5)   # Window size for moving average filter
        self.declare_parameter('use_first_7_only', True)  # Use only first 7 parameters
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.extended_format = self.get_parameter('extended_format').value
        self.filter_window_size = self.get_parameter('filter_window_size').value
        self.use_first_7_only = self.get_parameter('use_first_7_only').value
        
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
        self.print_interval = 0.5  # seconds between prints
        self.data_history = []
        self.history_length = 10  # Store last 10 data points for min/max
        
        # Set initial position (can be updated based on odometry or GPS if available)
        self.position = [0.0, 0.0, 0.0]  # x, y, z in world frame
        
        # Data size constants - use extended format but only process first 7
        # Extended format with 10 values: 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
        self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')  # 40 bytes
        self.get_logger().info(f'{Colors.GREEN}IMU-LiDAR fusion mode: calculating orientation for point cloud registration{Colors.ENDC}')
        
        # Initialize data buffers for moving average filter
        self.accel_x_buffer = []
        self.accel_y_buffer = []
        self.accel_z_buffer = []
        self.gyro_x_buffer = []
        self.gyro_y_buffer = []
        self.gyro_z_buffer = []
        self.compass_buffer = []
        self.roll_buffer = []
        self.pitch_buffer = []
        self.yaw_buffer = []
        
        # Store the last calculated rotation matrix
        self.last_rotation_matrix = np.eye(3)

        # Print welcome message
        self.print_welcome_message()
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.process_data)  # 100Hz processing
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
    def print_welcome_message(self):
        print("\n" + "="*80)
        print(f"{Colors.BOLD}{Colors.HEADER}IMU-LiDAR Fusion Node{Colors.ENDC}")
        print(f"Connecting to IMU server at {Colors.CYAN}{self.tcp_ip}:{self.tcp_port}{Colors.ENDC}")
        print(f"IMU frame: {Colors.GREEN}{self.frame_id}{Colors.ENDC}, LiDAR frame: {Colors.GREEN}{self.lidar_frame_id}{Colors.ENDC}")
        print(f"Publishing TF transforms for LiDAR point cloud registration")
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
    
    def format_float(self, value):
        """Format float value with color based on magnitude"""
        abs_val = abs(value)
        if abs_val < 0.001:  # Very small value
            return f"{Colors.BLUE}{value:.6f}{Colors.ENDC}"
        elif abs_val > 10.0:  # Large value
            return f"{Colors.RED}{value:.2f}{Colors.ENDC}"
        else:  # Normal value
            return f"{Colors.GREEN}{value:.4f}{Colors.ENDC}"
            
    def process_data(self):
        """Process data from the TCP socket and generate transforms for LiDAR fusion"""
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
                
                # Parse all 10 values but only use the first 7 if specified
                imu_values = struct.unpack('ffffffffff', self.buffer[:self.IMU_DATA_SIZE])
                accel_x, accel_y, accel_z = imu_values[0:3]  # Accelerometer data
                gyro_x, gyro_y, gyro_z = imu_values[3:6]     # Gyroscope data
                compass = imu_values[6]                       # Compass value
                
                # These values are received but ignored when use_first_7_only is True
                received_roll, received_pitch, received_yaw = imu_values[7:10]
                
                if self.use_first_7_only:
                    # Calculate roll and pitch from accelerometer
                    accel_mag = math.sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z)
                    if accel_mag > 0.5:  # Only calculate if we have reasonable acceleration
                        roll = math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z)) * 180.0 / math.pi
                        pitch = math.atan2(-accel_x, accel_z) * 180.0 / math.pi
                    else:
                        # Not enough acceleration to determine orientation
                        roll = 0.0
                        pitch = 0.0
                    
                    # Map compass to standard degrees (0-360)
                    yaw = (compass / 6.28) * 360.0
                    while yaw < 0:
                        yaw += 360.0
                    while yaw >= 360.0:
                        yaw -= 360.0
                else:
                    # Use the received orientation values
                    roll, pitch, yaw = received_roll, received_pitch, received_yaw
                
                # Store in history for stats
                self.data_history.append(imu_values)
                if len(self.data_history) > self.history_length:
                    self.data_history.pop(0)
                
                # Apply moving average filter to orientation values
                filtered_roll = self.apply_moving_average(roll, self.roll_buffer)
                filtered_pitch = self.apply_moving_average(pitch, self.pitch_buffer)
                filtered_yaw = self.apply_moving_average(yaw, self.yaw_buffer)
                
                # Convert to radians for calculations
                roll_rad = filtered_roll * (math.pi / 180.0)
                pitch_rad = filtered_pitch * (math.pi / 180.0)
                yaw_rad = filtered_yaw * (math.pi / 180.0)
                
                # Calculate rotation matrix for LiDAR point cloud transformations
                self.last_rotation_matrix = rotation_matrix_from_euler(roll_rad, pitch_rad, yaw_rad)
                
                # Publish IMU data and transforms for LiDAR fusion
                self.publish_imu_data(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, 
                                     filtered_roll, filtered_pitch, filtered_yaw)
                
                # Publish TF transform for LiDAR point cloud registration
                self.publish_tf_transform(filtered_roll, filtered_pitch, filtered_yaw)
                
                # Visualize data at regular intervals
                current_time = time.time()
                if current_time - self.last_print_time >= self.print_interval:
                    self.last_print_time = current_time
                    
                    # Clear screen for better visibility
                    print("\033c", end="")
                    
                    # Print header
                    print(f"\n{Colors.BOLD}{Colors.HEADER}IMU-LiDAR Fusion{Colors.ENDC} - Packet #{self.packet_count}")
                    print(f"Time: {Colors.CYAN}{time.strftime('%H:%M:%S')}{Colors.ENDC}")
                    print("="*80)
                    
                    # Orientation data for LiDAR fusion
                    print(f"\n{Colors.BOLD}Orientation (for LiDAR fusion):{Colors.ENDC}")
                    print(f"  Roll:  {self.format_float(filtered_roll)}°")
                    print(f"  Pitch: {self.format_float(filtered_pitch)}°")
                    print(f"  Yaw:   {self.format_float(filtered_yaw)}°")
                    
                    # Display rotation matrix for LiDAR point cloud transformation
                    print(f"\n{Colors.BOLD}Rotation Matrix:{Colors.ENDC}")
                    for i in range(3):
                        print(f"  [ {self.format_float(self.last_rotation_matrix[i, 0])} {self.format_float(self.last_rotation_matrix[i, 1])} {self.format_float(self.last_rotation_matrix[i, 2])} ]")
                    
                    # Show quaternion values
                    q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
                    print(f"\n{Colors.BOLD}Quaternion:{Colors.ENDC}")
                    print(f"  x: {self.format_float(q[0])}, y: {self.format_float(q[1])}, z: {self.format_float(q[2])}, w: {self.format_float(q[3])}")
                    
                    # Print TF information
                    print(f"\n{Colors.BOLD}TF Transforms:{Colors.ENDC}")
                    print(f"  Publishing: {self.world_frame_id} → {self.frame_id} → {self.lidar_frame_id}")
                    print(f"  Use these transforms to register LiDAR point clouds in a common reference frame")
                    
                    # Print footer
                    print("\n" + "="*80)
                
                # Remove processed data from buffer
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error processing IMU data: {str(e)}{Colors.ENDC}')
            import traceback
            traceback.print_exc()
        
    def apply_moving_average(self, value, buffer):
        """Apply moving average filter to the value"""
        # Add new value to buffer
        buffer.append(value)
        
        # Keep buffer size limited
        if len(buffer) > self.filter_window_size:
            buffer.pop(0)
            
        # Calculate and return the average
        return sum(buffer) / len(buffer)
        
    def publish_tf_transform(self, roll, pitch, yaw):
        """Publish TF transforms for LiDAR-IMU fusion"""
        now = self.get_clock().now().to_msg()
        
        # Create transform from world to IMU
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.world_frame_id
        t.child_frame_id = self.frame_id
        
        # Set IMU position in world frame (could be updated from odometry/GPS)
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        
        # Set IMU orientation
        roll_rad = roll * (math.pi / 180.0)
        pitch_rad = pitch * (math.pi / 180.0)
        yaw_rad = yaw * (math.pi / 180.0)
        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Create transform from IMU to LiDAR (static transform, adjust values based on your setup)
        t_lidar = TransformStamped()
        t_lidar.header.stamp = now
        t_lidar.header.frame_id = self.frame_id
        t_lidar.child_frame_id = self.lidar_frame_id
        
        # Set LiDAR position relative to IMU (adjust based on your sensor placement)
        t_lidar.transform.translation.x = 0.0  # Forward offset
        t_lidar.transform.translation.y = 0.0  # Left/right offset
        t_lidar.transform.translation.z = 0.2  # Up/down offset
        
        # Set LiDAR orientation relative to IMU (usually identity unless sensors are misaligned)
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = 0.0
        t_lidar.transform.rotation.w = 1.0
        
        # Publish the LiDAR transform
        self.tf_broadcaster.sendTransform(t_lidar)

    def publish_imu_data(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw):
        """Publish IMU data to ROS2 for visualization in RViz2 and LiDAR fusion"""
        # Still filter acceleration and gyroscope data for proper IMU message
        filtered_accel_x = self.apply_moving_average(accel_x, self.accel_x_buffer)
        filtered_accel_y = self.apply_moving_average(accel_y, self.accel_y_buffer)
        filtered_accel_z = self.apply_moving_average(accel_z, self.accel_z_buffer)
        filtered_gyro_x = self.apply_moving_average(gyro_x, self.gyro_x_buffer)
        filtered_gyro_y = self.apply_moving_average(gyro_y, self.gyro_y_buffer)
        filtered_gyro_z = self.apply_moving_average(gyro_z, self.gyro_z_buffer)
        
        # Create IMU message
        imu_msg = Imu()
        
        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        
        # Set linear acceleration and angular velocity (required for ROS2 IMU message)
        imu_msg.linear_acceleration.x = filtered_accel_x
        imu_msg.linear_acceleration.y = filtered_accel_y
        imu_msg.linear_acceleration.z = filtered_accel_z
        
        # Convert gyroscope values from deg/s to rad/s as required by ROS2
        imu_msg.angular_velocity.x = filtered_gyro_x * (math.pi / 180.0)
        imu_msg.angular_velocity.y = filtered_gyro_y * (math.pi / 180.0)
        imu_msg.angular_velocity.z = filtered_gyro_z * (math.pi / 180.0)
        
        # Convert Euler angles (roll, pitch, yaw) to quaternion for ROS2
        roll_rad = roll * (math.pi / 180.0)
        pitch_rad = pitch * (math.pi / 180.0)
        yaw_rad = yaw * (math.pi / 180.0)
        
        # Calculate quaternion
        q = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        
        # Set orientation in quaternion format
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        
        # Set covariance matrices (required for RViz2)
        orientation_cov = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # rad^2
        angular_velocity_cov = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]  # rad^2/s^2
        linear_acceleration_cov = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]  # m^2/s^4
        
        imu_msg.orientation_covariance = orientation_cov
        imu_msg.angular_velocity_covariance = angular_velocity_cov
        imu_msg.linear_acceleration_covariance = linear_acceleration_cov
        
        # Publish the message
        self.imu_publisher.publish(imu_msg)
        
        # Log orientation values occasionally for LiDAR fusion debugging
        if self.packet_count % 100 == 0:
            self.get_logger().info(f"IMU Orientation for LiDAR fusion - Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")

    def cleanup(self):
        """Clean up resources"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            
def main(args=None):
    rclpy.init(args=args)
    
    imu_lidar_fusion = ImuLidarFusion()
    
    try:
        rclpy.spin(imu_lidar_fusion)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Shutting down IMU-LiDAR Fusion Node...{Colors.ENDC}")
    finally:
        imu_lidar_fusion.cleanup()
        imu_lidar_fusion.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()