#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
# Replace tf_transformations import with NumPy implementation
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

# Function to generate a simple ASCII bar chart
def generate_ascii_bar(value, max_value, width=20):
    """
    Generate a simple ASCII bar chart for visualization.
    """
    normalized = abs(value) / max_value if max_value != 0 else 0
    bar_width = int(normalized * width)
    bar = '█' * bar_width
    
    # Add color based on value
    if abs(value) > max_value * 0.8:
        return f"{Colors.RED}{bar}{Colors.ENDC}"
    elif abs(value) > max_value * 0.5:
        return f"{Colors.YELLOW}{bar}{Colors.ENDC}"
    else:
        return f"{Colors.GREEN}{bar}{Colors.ENDC}"

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

class ImuDataVisualizer(Node):
    def __init__(self):
        super().__init__('imu_data_visualizer')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12345)
        self.declare_parameter('reconnect_interval', 5.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('extended_format', False)  # Default to basic format
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.frame_id = self.get_parameter('frame_id').value
        self.extended_format = self.get_parameter('extended_format').value
        
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
        
        # Data size constants - support both formats but focus on basic
        if self.extended_format:
            # Extended format with 10 values: 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
            self.IMU_DATA_SIZE = struct.calcsize('ffffffffff')  # 40 bytes
            self.get_logger().info(f'{Colors.GREEN}Using extended IMU format (10 values including roll/pitch/yaw){Colors.ENDC}')
        else:
            # Basic format with 7 values: 3 accel, 3 gyro, 1 compass
            self.IMU_DATA_SIZE = struct.calcsize('fffffff')  # 28 bytes
            self.get_logger().info(f'{Colors.YELLOW}Using basic IMU format (7 values){Colors.ENDC}')
        
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
        print(f"{Colors.BOLD}{Colors.HEADER}IMU Data Visualizer{Colors.ENDC}")
        print(f"Connecting to IMU server at {Colors.CYAN}{self.tcp_ip}:{self.tcp_port}{Colors.ENDC}")
        print(f"Data format: {Colors.GREEN if self.extended_format else Colors.YELLOW}{'Extended (10 values)' if self.extended_format else 'Basic (7 values)'}{Colors.ENDC}")
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
        """Process data from the TCP socket and visualize IMU data"""
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
                
                if self.extended_format:
                    # Parse 10 values: 3 accel, 3 gyro, 1 compass, roll, pitch, yaw
                    imu_values = struct.unpack('ffffffffff', self.buffer[:self.IMU_DATA_SIZE])
                    accel_x, accel_y, accel_z = imu_values[0:3]  # Accelerometer data
                    gyro_x, gyro_y, gyro_z = imu_values[3:6]     # Gyroscope data
                    compass = imu_values[6]                       # Compass
                    roll, pitch, yaw = imu_values[7:10]           # Euler angles
                else:
                    # Parse 7 values: 3 accel, 3 gyro, 1 compass
                    imu_values = struct.unpack('fffffff', self.buffer[:self.IMU_DATA_SIZE])
                    accel_x, accel_y, accel_z = imu_values[0:3]  # Accelerometer data
                    gyro_x, gyro_y, gyro_z = imu_values[3:6]     # Gyroscope data
                    compass = imu_values[6]                       # Compass
                    
                    # Calculate roll and pitch from accelerometer
                    roll = math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z)) * 180.0 / math.pi
                    pitch = math.atan2(-accel_x, accel_z) * 180.0 / math.pi
                    yaw = compass  # Use compass as yaw
                
                # Store in history for stats
                self.data_history.append(imu_values)
                if len(self.data_history) > self.history_length:
                    self.data_history.pop(0)
                
                # Visualize data at regular intervals
                current_time = time.time()
                if current_time - self.last_print_time >= self.print_interval:
                    self.last_print_time = current_time
                    
                    # Calculate acceleration and gyro magnitudes
                    accel_mag = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
                    gyro_mag = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
                    
                    # Clear screen for better visibility
                    print("\033c", end="")
                    
                    # Print header
                    print(f"\n{Colors.BOLD}{Colors.HEADER}IMU Data Visualization{Colors.ENDC} - Packet #{self.packet_count}")
                    print(f"Time: {Colors.CYAN}{time.strftime('%H:%M:%S')}{Colors.ENDC}")
                    print("="*80)
                    
                    # Accelerometer data with bar graphs
                    print(f"{Colors.BOLD}Accelerometer (m/s²):{Colors.ENDC}")
                    print(f"  X: {self.format_float(accel_x)} │ {generate_ascii_bar(accel_x, 10.0)}")
                    print(f"  Y: {self.format_float(accel_y)} │ {generate_ascii_bar(accel_y, 10.0)}")
                    print(f"  Z: {self.format_float(accel_z)} │ {generate_ascii_bar(accel_z, 10.0)}")
                    print(f"  Magnitude: {self.format_float(accel_mag)} {'❌ INVALID' if accel_mag < 0.1 or accel_mag > 30.0 else '✅ OK'}")
                    
                    # Gyroscope data with bar graphs
                    print(f"\n{Colors.BOLD}Gyroscope (deg/s):{Colors.ENDC}")
                    print(f"  X: {self.format_float(gyro_x)} │ {generate_ascii_bar(gyro_x, 100.0)}")
                    print(f"  Y: {self.format_float(gyro_y)} │ {generate_ascii_bar(gyro_y, 100.0)}")
                    print(f"  Z: {self.format_float(gyro_z)} │ {generate_ascii_bar(gyro_z, 100.0)}")
                    print(f"  Magnitude: {self.format_float(gyro_mag)}")
                    
                    # Calculated orientation
                    if not self.extended_format:
                        print(f"\n{Colors.BOLD}Calculated Orientation (degrees):{Colors.ENDC}")
                    else:
                        print(f"\n{Colors.BOLD}Orientation (degrees):{Colors.ENDC}")
                    
                    # Create visual indicator for roll/pitch using ASCII
                    roll_indicator = "─" * 10 + "●" + "─" * 10
                    pitch_indicator = "─" * 10 + "●" + "─" * 10
                    
                    # Adjust position based on angle (-90 to +90 degrees range)
                    roll_pos = min(max(int((roll + 90) / 180 * 20), 0), 20)
                    pitch_pos = min(max(int((pitch + 90) / 180 * 20), 0), 20)
                    
                    # Replace character at position
                    roll_indicator = roll_indicator[:roll_pos] + "O" + roll_indicator[roll_pos+1:]
                    pitch_indicator = pitch_indicator[:pitch_pos] + "O" + pitch_indicator[pitch_pos+1:]
                    
                    print(f"  Roll:  {self.format_float(roll)}° │ {roll_indicator}")
                    print(f"  Pitch: {self.format_float(pitch)}° │ {pitch_indicator}")
                    print(f"  Yaw:   {self.format_float(yaw)}°")
                    
                    # Compass visualization - create circular indicator
                    print(f"\n{Colors.BOLD}Compass:{Colors.ENDC} {self.format_float(compass)}°")
                    
                    # Create simple compass visualization
                    compass_rad = yaw * math.pi / 180.0
                    compass_x = int(10 * math.sin(compass_rad)) + 10
                    compass_y = int(5 * math.cos(compass_rad)) + 5
                    
                    compass_display = []
                    for y in range(11):
                        line = ""
                        for x in range(21):
                            if x == 10 and y == 5:
                                line += "+"  # Center
                            elif x == compass_x and y == compass_y:
                                line += "◉"  # Current direction
                            elif (x == 10 and y < 5) or (y == 5 and x > 10):
                                line += "·"  # Axis
                            else:
                                line += " "
                        compass_display.append(line)
                    
                    # Add compass directions
                    compass_display[0] = compass_display[0][:10] + "N" + compass_display[0][11:]
                    compass_display[5] = "W" + compass_display[5][1:20] + "E"
                    compass_display[10] = compass_display[10][:10] + "S" + compass_display[10][11:]
                    
                    # Print compass
                    for line in compass_display:
                        print(f"  {line}")
                    
                    # Print footer
                    print("\n" + "="*80)
                
                # Remove processed data from buffer
                self.buffer = self.buffer[self.IMU_DATA_SIZE:]
                
        except Exception as e:
            self.get_logger().error(f'{Colors.RED}Error processing IMU data: {str(e)}{Colors.ENDC}')
            import traceback
            traceback.print_exc()
        
    def cleanup(self):
        """Clean up resources"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            
def main(args=None):
    rclpy.init(args=args)
    
    imu_visualizer = ImuDataVisualizer()
    
    try:
        rclpy.spin(imu_visualizer)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Shutting down IMU Data Visualizer...{Colors.ENDC}")
    finally:
        imu_visualizer.cleanup()
        imu_visualizer.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()