#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import socket
import struct
import time

class ImuRvizVisualizer(Node):
    def __init__(self):
        super().__init__('imu_rviz_visualizer')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12345)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('use_tcp', True)
        self.declare_parameter('arrow_scale', 1.0)  # Increased scale for better visibility
        self.declare_parameter('update_frequency', 100.0)  # Hz
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_tcp = self.get_parameter('use_tcp').value
        self.arrow_scale = self.get_parameter('arrow_scale').value
        self.update_frequency = self.get_parameter('update_frequency').value
        
        # TCP socket related
        self.socket = None
        self.connected = False
        self.buffer = b''
        
        # Publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'imu_markers', 10)
        
        # TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Data storage
        self.last_accel = [0.0, 0.0, 0.0]
        self.last_gyro = [0.0, 0.0, 0.0]
        self.last_compass = 0.0
        
        # Connection setup and timers
        self.get_logger().info(f'Starting IMU RViz Visualizer - Focused on Acc/Gyro/Compass')
        self.get_logger().info(f'Using TCP connection to {self.tcp_ip}:{self.tcp_port}')
        self.connect()
        
        # Process data at specified frequency
        period = 1.0 / self.update_frequency
        self.create_timer(period, self.process_tcp_data)
        self.create_timer(1.0, self.check_connection)
        self.create_timer(0.1, self.publish_visualization)  # Publish viz at 10Hz
            
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
            
    def process_tcp_data(self):
        """Process data from the TCP socket"""
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
            # First, check if we have at least one complete packet (7 floats = 28 bytes)
            while len(self.buffer) >= 28:
                # Parse 7 values: 3 accel, 3 gyro, 1 compass
                imu_values = struct.unpack('fffffff', self.buffer[:28])
                
                # Store the latest values
                self.last_accel = imu_values[0:3]  # Accelerometer data
                self.last_gyro = imu_values[3:6]   # Gyroscope data
                self.last_compass = imu_values[6]  # Compass in degrees
                
                # Log data occasionally for debugging
                self.log_data_debug()
                
                # Remove processed data from buffer
                self.buffer = self.buffer[28:]
                
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def log_data_debug(self):
        """Log data values occasionally for debugging"""
        # Only log every ~500 packets to avoid flooding
        if hasattr(self, 'packet_counter'):
            self.packet_counter += 1
        else:
            self.packet_counter = 0
            
        if self.packet_counter % 500 == 0:
            # Calculate magnitudes
            accel_mag = math.sqrt(sum(x*x for x in self.last_accel))
            gyro_mag = math.sqrt(sum(x*x for x in self.last_gyro))
            
            self.get_logger().info(
                f"IMU Data - Accel: [{self.last_accel[0]:.2f}, {self.last_accel[1]:.2f}, {self.last_accel[2]:.2f}] "
                f"mag: {accel_mag:.2f} - "
                f"Gyro: [{self.last_gyro[0]:.2f}, {self.last_gyro[1]:.2f}, {self.last_gyro[2]:.2f}] "
                f"mag: {gyro_mag:.2f} - "
                f"Compass: {self.last_compass:.2f}°"
            )
            
    def publish_visualization(self):
        """Publish markers for visualization in RViz"""
        # Create a marker array
        marker_array = MarkerArray()
        now = self.get_clock().now()
        
        # Extract the data values
        accel_x, accel_y, accel_z = self.last_accel
        gyro_x, gyro_y, gyro_z = self.last_gyro
        compass = self.last_compass * math.pi / 180.0  # Convert to radians
        
        # Calculate orientation from accelerometer for frame
        roll = math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))
        pitch = math.atan2(-accel_x, accel_z)
        yaw = compass  # Use compass reading as yaw
        
        # Add a reference coordinate frame
        frame_marker = self.create_frame_marker(roll, pitch, yaw, now)
        marker_array.markers.append(frame_marker)
        
        # Add acceleration vector marker
        accel_marker = self.create_accel_marker(accel_x, accel_y, accel_z, roll, pitch, yaw, now)
        if accel_marker:
            marker_array.markers.append(accel_marker)
            
        # Add gyroscope vector marker
        gyro_marker = self.create_gyro_marker(gyro_x, gyro_y, gyro_z, roll, pitch, yaw, now)
        if gyro_marker:
            marker_array.markers.append(gyro_marker)
        
        # Add compass visualization as a clear, colorful arrow
        compass_marker = self.create_compass_marker(compass, now)
        marker_array.markers.append(compass_marker)
        
        # Publish the marker array
        self.marker_pub.publish(marker_array)
        
        # Broadcast the transform
        self.broadcast_tf(roll, pitch, yaw, now)
        
    def create_frame_marker(self, roll, pitch, yaw, timestamp):
        """Create a coordinate frame marker"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp.to_msg()
        marker.ns = "imu_frame"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Calculate rotation matrix
        c1 = math.cos(roll)
        s1 = math.sin(roll)
        c2 = math.cos(pitch)
        s2 = math.sin(pitch)
        c3 = math.cos(yaw)
        s3 = math.sin(yaw)
        
        R = np.array([
            [c2*c3, -c2*s3, s2],
            [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
            [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]
        ])
        
        # Rotate axes
        origin = np.array([0, 0, 0])
        x_axis = np.array([self.arrow_scale, 0, 0])
        y_axis = np.array([0, self.arrow_scale, 0])
        z_axis = np.array([0, 0, self.arrow_scale])
        
        x_axis_rotated = np.dot(R, x_axis)
        y_axis_rotated = np.dot(R, y_axis)
        z_axis_rotated = np.dot(R, z_axis)
        
        # Add points to marker for coordinate frame
        marker.points = []
        
        # X-axis (red)
        p_origin = Point(x=0.0, y=0.0, z=0.0)
        p_x = Point(x=x_axis_rotated[0], y=x_axis_rotated[1], z=x_axis_rotated[2])
        marker.points.append(p_origin)
        marker.points.append(p_x)
        
        # Y-axis (green)
        p_y = Point(x=y_axis_rotated[0], y=y_axis_rotated[1], z=y_axis_rotated[2])
        marker.points.append(p_origin)
        marker.points.append(p_y)
        
        # Z-axis (blue)
        p_z = Point(x=z_axis_rotated[0], y=z_axis_rotated[1], z=z_axis_rotated[2])
        marker.points.append(p_origin)
        marker.points.append(p_z)
        
        # Colors for each axis
        marker.colors = []
        # X-axis (red)
        marker.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        marker.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        # Y-axis (green)
        marker.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        marker.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
        # Z-axis (blue)
        marker.colors.append(ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0))
        marker.colors.append(ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0))
        
        marker.scale.x = 0.03  # Line width
        marker.lifetime.sec = 0  # Forever
        
        return marker
        
    def create_accel_marker(self, accel_x, accel_y, accel_z, roll, pitch, yaw, timestamp):
        """Create an arrow marker for acceleration"""
        # Calculate rotation matrix for orientation
        c1 = math.cos(roll)
        s1 = math.sin(roll)
        c2 = math.cos(pitch)
        s2 = math.sin(pitch)
        c3 = math.cos(yaw)
        s3 = math.sin(yaw)
        
        R = np.array([
            [c2*c3, -c2*s3, s2],
            [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
            [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]
        ])
        
        # Calculate magnitude of acceleration
        accel_mag = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # Only draw if acceleration is significant
        if accel_mag < 0.1:  # Threshold to avoid noise
            return None
            
        # Normalize and scale acceleration for visualization
        # Enhanced scale for better visibility
        scale_factor = self.arrow_scale
        accel_vec = np.array([accel_x, accel_y, accel_z]) / max(accel_mag, 0.1) * scale_factor
        
        # Create marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp.to_msg()
        marker.ns = "imu_accel"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Set the arrow points
        marker.points = []
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(
            x=accel_vec[0], 
            y=accel_vec[1], 
            z=accel_vec[2]
        ))
        
        # Set visuals
        marker.scale.x = 0.03  # Shaft diameter
        marker.scale.y = 0.06  # Head diameter
        marker.scale.z = 0.09  # Head length
        
        # Use orange color for acceleration
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # Orange
        marker.lifetime.sec = 0  # Forever
        
        return marker

    def create_gyro_marker(self, gyro_x, gyro_y, gyro_z, roll, pitch, yaw, timestamp):
        """Create an arrow marker for gyroscope data"""
        # Calculate rotation matrix for orientation
        c1 = math.cos(roll)
        s1 = math.sin(roll)
        c2 = math.cos(pitch)
        s2 = math.sin(pitch)
        c3 = math.cos(yaw)
        s3 = math.sin(yaw)
        
        R = np.array([
            [c2*c3, -c2*s3, s2],
            [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
            [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]
        ])
        
        # Calculate magnitude of gyroscope
        gyro_mag = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
        
        # Only draw if gyro is significant
        if gyro_mag < 1.0:  # Threshold to avoid noise (in degrees/sec)
            return None
            
        # Normalize and scale gyro for visualization
        # Enhanced scale for better visibility
        scale_factor = self.arrow_scale * 0.02  # Scale to make it reasonable compared to accel
        gyro_vec = np.array([gyro_x, gyro_y, gyro_z]) / max(gyro_mag, 0.1) * scale_factor * gyro_mag
        
        # Create marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp.to_msg()
        marker.ns = "imu_gyro"
        marker.id = 2
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Set the arrow points
        marker.points = []
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(
            x=gyro_vec[0], 
            y=gyro_vec[1], 
            z=gyro_vec[2]
        ))
        
        # Set visuals - make gyro slightly thinner than accel
        marker.scale.x = 0.025  # Shaft diameter
        marker.scale.y = 0.05   # Head diameter
        marker.scale.z = 0.075  # Head length
        
        # Use purple for gyro to distinguish from accel
        marker.color = ColorRGBA(r=0.8, g=0.2, b=1.0, a=1.0)  # Purple
        marker.lifetime.sec = 0  # Forever
        
        return marker
        
    def create_compass_marker(self, compass_rad, timestamp):
        """Create a marker for compass visualization"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp.to_msg()
        marker.ns = "imu_compass"
        marker.id = 3
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Calculate the compass direction vector
        compass_x = math.sin(compass_rad) * self.arrow_scale * 1.5
        compass_y = math.cos(compass_rad) * self.arrow_scale * 1.5
        compass_z = 0.0  # Keep it in the horizontal plane
        
        # Set the arrow points
        marker.points = []
        marker.points.append(Point(x=0.0, y=0.0, z=0.0))
        marker.points.append(Point(x=compass_x, y=compass_y, z=compass_z))
        
        # Set visuals - make compass arrow more distinct
        marker.scale.x = 0.04   # Shaft diameter
        marker.scale.y = 0.08   # Head diameter
        marker.scale.z = 0.12   # Head length
        
        # Use bright cyan for compass
        marker.color = ColorRGBA(r=0.0, g=0.9, b=0.9, a=1.0)  # Cyan
        marker.lifetime.sec = 0  # Forever
        
        return marker
        
    def broadcast_tf(self, roll, pitch, yaw, timestamp):
        """Broadcast the IMU transform"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.frame_id
        
        # Set translation (fixed position)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Convert roll, pitch, yaw to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
        
    def cleanup(self):
        """Clean up resources"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    visualizer = ImuRvizVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print("Shutting down IMU RViz Visualizer...")
    finally:
        visualizer.cleanup()
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 