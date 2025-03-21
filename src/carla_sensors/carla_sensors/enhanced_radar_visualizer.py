#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import threading


class EnhancedRadarVisualizer(Node):
    """
    Advanced ROS2 node for visualizing radar clustering data from TCP stream.
    Features:
    - Automatic reconnection on connection failure
    - Enhanced visualization including velocity vectors
    - Performance metrics and diagnostics
    - Robust error handling and data parsing
    """
    def __init__(self):
        super().__init__('enhanced_radar_visualizer')
        
        # ROS2 Publishers
        self.point_cloud_publisher = self.create_publisher(PointCloud2, '/radar/points', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/radar/markers', 10)
        
        # Declare parameters with defaults
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12348)
        self.declare_parameter('reconnect_interval', 5.0)  # seconds
        self.declare_parameter('marker_lifetime', 0.5)     # seconds
        self.declare_parameter('max_buffer_size', 1048576) # 1MB
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.max_buffer_size = self.get_parameter('max_buffer_size').value
        
        # Initialize TCP socket and connection state
        self.socket = None
        self.connected = False
        self.data_buffer = bytearray()
        
        # Statistics and diagnostics
        self.stats = {
            'total_points_received': 0,
            'total_clusters_received': 0,
            'connection_attempts': 0,
            'connection_failures': 0,
            'parse_errors': 0,
            'last_data_time': None,
            'largest_cluster_size': 0,
        }
        
        # Thread lock for socket operations
        self.lock = threading.Lock()
        
        # Connect to TCP server
        self.connect()
        
        # Create timers for periodic tasks
        self.create_timer(0.01, self.receive_data)         # 100Hz for data receiving
        self.create_timer(10.0, self.print_statistics)     # 0.1Hz for stats
        self.create_timer(1.0, self.check_connection)      # 1Hz for connection check
        
        self.get_logger().info('Enhanced Radar Visualizer initialized')

    def connect(self):
        """Establish connection to the TCP server with error handling"""
        if self.connected:
            return True
            
        with self.lock:
            # Close any existing socket
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
            
            # Create new socket
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(2.0)  # 2 second timeout for connection
                self.get_logger().info(f'Attempting to connect to {self.tcp_ip}:{self.tcp_port}...')
                self.socket.connect((self.tcp_ip, self.tcp_port))
                self.socket.settimeout(0.1)  # Shorter timeout for data reception
                self.connected = True
                self.stats['connection_attempts'] += 1
                self.get_logger().info('Connected to radar data server successfully')
                return True
                
            except ConnectionRefusedError:
                self.stats['connection_failures'] += 1
                self.get_logger().error(f'Connection refused to {self.tcp_ip}:{self.tcp_port}. Server may not be running.')
                self.connected = False
                return False
                
            except socket.timeout:
                self.stats['connection_failures'] += 1
                self.get_logger().error(f'Connection timeout to {self.tcp_ip}:{self.tcp_port}')
                self.connected = False
                return False
                
            except Exception as e:
                self.stats['connection_failures'] += 1
                self.get_logger().error(f'Connection failed: {str(e)}')
                self.connected = False
                return False

    def check_connection(self):
        """Periodically check connection and attempt reconnection if needed"""
        if not self.connected:
            self.get_logger().info('Connection check: Not connected. Attempting to reconnect...')
            self.connect()
        else:
            # Check if we've received data recently
            if self.stats['last_data_time'] is not None:
                time_since_last_data = time.time() - self.stats['last_data_time']
                if time_since_last_data > 5.0:  # If no data for 5 seconds
                    self.get_logger().warn(f'No data received for {time_since_last_data:.1f} seconds. Connection may be stale.')
                    # Force reconnection
                    with self.lock:
                        self.connected = False
                        try:
                            self.socket.close()
                        except:
                            pass
                        self.socket = None
                        self.data_buffer.clear()

    def receive_data(self):
        """Receive and process data from TCP connection"""
        if not self.connected:
            return
            
        try:
            with self.lock:
                # Check if socket is valid
                if not self.socket:
                    self.connected = False
                    return
                    
                # Attempt to receive data
                try:
                    self.socket.settimeout(0.01)  # Short timeout so we don't block
                    chunk = self.socket.recv(4096)
                    
                    if not chunk:
                        # Empty data means connection closed
                        self.get_logger().warn('Connection closed by server')
                        self.connected = False
                        self.socket.close()
                        self.socket = None
                        return
                        
                    # Update stats
                    self.stats['last_data_time'] = time.time()
                    
                    # Add to buffer
                    self.data_buffer.extend(chunk)
                    
                    # Process buffer if it has enough data
                    self.process_buffer()
                    
                except socket.timeout:
                    # This is normal with non-blocking socket
                    pass
                    
                except ConnectionResetError:
                    self.get_logger().error('Connection reset by server')
                    self.connected = False
                    self.socket.close()
                    self.socket = None
                    
                except Exception as e:
                    self.get_logger().error(f'Socket receive error: {str(e)}')
                    self.connected = False
                    self.socket.close()
                    self.socket = None
        
        except Exception as e:
            self.get_logger().error(f'Unhandled exception in receive_data: {str(e)}')

    def process_buffer(self):
        """Process accumulated data in the buffer"""
        # Check buffer size limit
        if len(self.data_buffer) > self.max_buffer_size:
            self.get_logger().warn(f'Buffer size exceeded limit ({len(self.data_buffer)} bytes). Clearing buffer.')
            self.data_buffer.clear()
            self.stats['parse_errors'] += 1
            return
            
        # Process complete messages until buffer is exhausted or incomplete
        while len(self.data_buffer) >= 4:  # At least need header size
            try:
                # Parse number of clusters
                num_clusters = struct.unpack('!I', self.data_buffer[:4])[0]
                
                # Sanity check on cluster count
                if num_clusters > 100:  # Arbitrary limit to catch corrupt data
                    self.get_logger().warn(f'Invalid cluster count: {num_clusters}. Data may be corrupt.')
                    self.data_buffer.clear()
                    self.stats['parse_errors'] += 1
                    return
                    
                offset = 4  # Start after the header
                
                # Check if we have enough data for the full message
                # First, calculate expected message size
                # For each cluster: 4 bytes for point count + points data
                expected_size = 4  # Header
                
                complete_message = True
                for i in range(num_clusters):
                    # Need at least 4 bytes for point count
                    if len(self.data_buffer) < offset + 4:
                        complete_message = False
                        break
                        
                    # Get point count for this cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset+4])[0]
                    
                    # Sanity check point count
                    if num_points > 10000:  # Arbitrary limit to catch corrupt data
                        self.get_logger().warn(f'Invalid point count: {num_points}. Data may be corrupt.')
                        self.data_buffer.clear()
                        self.stats['parse_errors'] += 1
                        return
                        
                    # Calculate size for this cluster
                    cluster_size = 4 + (num_points * 16)  # 4 bytes header + (16 bytes per point)
                    expected_size += cluster_size
                    
                    # Check if we have enough data for this cluster
                    if len(self.data_buffer) < offset + cluster_size:
                        complete_message = False
                        break
                        
                    offset += cluster_size
                
                # If message is incomplete, wait for more data
                if not complete_message:
                    return
                    
                # If we got here, we have a complete message
                self.process_complete_message(num_clusters)
                
            except Exception as e:
                self.get_logger().error(f'Error processing buffer: {str(e)}')
                self.data_buffer.clear()  # Clear buffer on error
                self.stats['parse_errors'] += 1
                return

    def process_complete_message(self, num_clusters):
        """Process a complete radar data message"""
        try:
            # Update stats
            self.stats['total_clusters_received'] += num_clusters
            
            # Create marker array for visualization
            marker_array = MarkerArray()
            
            # Get the current ROS time for all markers
            current_time = self.get_clock().now().to_msg()
            
            # Predefined colors for clusters (add more if needed)
            cluster_colors = [
                (1.0, 0.0, 0.0),    # Red
                (0.0, 1.0, 0.0),    # Green
                (0.0, 0.0, 1.0),    # Blue
                (1.0, 1.0, 0.0),    # Yellow
                (1.0, 0.0, 1.0),    # Magenta
                (0.0, 1.0, 1.0),    # Cyan
                (1.0, 0.5, 0.0),    # Orange
                (0.5, 0.0, 1.0),    # Purple
                (0.0, 0.5, 0.0),    # Dark Green
                (0.5, 0.5, 1.0),    # Light Blue
                (0.7, 0.7, 0.7),    # Light Gray
                (0.3, 0.3, 0.3),    # Dark Gray
            ]
            
            # Skip the 4-byte header
            offset = 4
            
            # Process each cluster
            for cluster_id in range(num_clusters):
                # Get number of points in this cluster
                num_points = struct.unpack('!I', self.data_buffer[offset:offset+4])[0]
                offset += 4
                
                # Update stats
                self.stats['total_points_received'] += num_points
                if num_points > self.stats['largest_cluster_size']:
                    self.stats['largest_cluster_size'] = num_points
                
                # Get color for this cluster
                color_idx = cluster_id % len(cluster_colors)
                r, g, b = cluster_colors[color_idx]
                
                # Storage for cluster points and statistics
                cluster_points = []
                avg_velocity = 0.0
                max_velocity = 0.0
                
                # Process each point in the cluster
                for point_idx in range(num_points):
                    # Parse point data (altitude, azimuth, depth, velocity)
                    point_data = struct.unpack('!ffff', self.data_buffer[offset:offset+16])
                    altitude, azimuth, depth, velocity = point_data
                    offset += 16
                    
                    # Convert to radians if needed
                    azimuth_rad = azimuth * math.pi / 180.0 if abs(azimuth) > 6.28 else azimuth
                    
                    # Convert from polar to Cartesian coordinates
                    x = depth * math.cos(azimuth_rad)
                    y = depth * math.sin(azimuth_rad)
                    z = altitude
                    
                    # Store coordinates and velocity for statistics
                    cluster_points.append((x, y, z, velocity))
                    
                    # Update velocity statistics
                    avg_velocity += abs(velocity)
                    if abs(velocity) > max_velocity:
                        max_velocity = abs(velocity)
                    
                    # Create point marker
                    point_marker = Marker()
                    point_marker.header.frame_id = "map"
                    point_marker.header.stamp = current_time
                    point_marker.ns = f"radar_points"
                    point_marker.id = (cluster_id * 10000) + point_idx  # Unique ID
                    point_marker.type = Marker.SPHERE
                    point_marker.action = Marker.ADD
                    
                    # Set position
                    point_marker.pose.position.x = x
                    point_marker.pose.position.y = y
                    point_marker.pose.position.z = z
                    point_marker.pose.orientation.w = 1.0  # Identity quaternion
                    
                    # Size proportional to velocity
                    size_factor = 0.2 + (abs(velocity) * 0.05)  # Base size + velocity scaling
                    point_marker.scale.x = size_factor
                    point_marker.scale.y = size_factor
                    point_marker.scale.z = size_factor
                    
                    # Color based on cluster with alpha
                    point_marker.color.r = r
                    point_marker.color.g = g
                    point_marker.color.b = b
                    point_marker.color.a = 0.7
                    
                    # Set lifetime
                    point_marker.lifetime.sec = int(self.marker_lifetime)
                    point_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                    
                    marker_array.markers.append(point_marker)
                    
                    # Add velocity vector marker
                    if abs(velocity) > 0.5:  # Only show vectors for moving objects
                        vector_marker = Marker()
                        vector_marker.header.frame_id = "map"
                        vector_marker.header.stamp = current_time
                        vector_marker.ns = f"velocity_vectors"
                        vector_marker.id = (cluster_id * 10000) + point_idx
                        vector_marker.type = Marker.ARROW
                        vector_marker.action = Marker.ADD
                        
                        # Start position (radar point)
                        start = Point()
                        start.x = x
                        start.y = y
                        start.z = z
                        
                        # End position (based on velocity direction)
                        end = Point()
                        # Simple velocity mapping (assuming azimuth is direction of travel)
                        vel_scale = abs(velocity) * 0.5  # Scale for visualization
                        end.x = x + math.cos(azimuth_rad) * vel_scale
                        end.y = y + math.sin(azimuth_rad) * vel_scale
                        end.z = z
                        
                        vector_marker.points = [start, end]
                        
                        # Arrow thickness
                        vector_marker.scale.x = 0.05  # Shaft diameter
                        vector_marker.scale.y = 0.1   # Head diameter
                        vector_marker.scale.z = 0.2   # Head length
                        
                        # Use velocity-based color (red for approaching, blue for receding)
                        if velocity < 0:  # Approaching
                            vector_marker.color.r = 1.0
                            vector_marker.color.g = 0.0
                            vector_marker.color.b = 0.0
                        else:  # Receding
                            vector_marker.color.r = 0.0
                            vector_marker.color.g = 0.0
                            vector_marker.color.b = 1.0
                        vector_marker.color.a = 0.8
                        
                        # Set lifetime
                        vector_marker.lifetime.sec = int(self.marker_lifetime)
                        vector_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                        
                        marker_array.markers.append(vector_marker)
                
                # Skip if no points in cluster
                if not cluster_points:
                    continue
                    
                # Calculate cluster statistics
                avg_velocity /= num_points if num_points > 0 else 1
                
                # Calculate cluster centroid
                center_x = sum(p[0] for p in cluster_points) / len(cluster_points)
                center_y = sum(p[1] for p in cluster_points) / len(cluster_points)
                center_z = sum(p[2] for p in cluster_points) / len(cluster_points)
                
                # Add cluster hull marker (convex hull around the cluster)
                if len(cluster_points) >= 3:
                    hull_marker = Marker()
                    hull_marker.header.frame_id = "map"
                    hull_marker.header.stamp = current_time
                    hull_marker.ns = f"radar_hulls"
                    hull_marker.id = cluster_id
                    hull_marker.type = Marker.LINE_STRIP
                    hull_marker.action = Marker.ADD
                    
                    # Create a simplified 2D convex hull
                    points_2d = [(p[0], p[1]) for p in cluster_points]
                    
                    # Sort points by angle from center (simple approximation of convex hull)
                    hull_points = sorted(points_2d, 
                        key=lambda p: math.atan2(p[1] - center_y, p[0] - center_x))
                    
                    # Add hull vertices
                    for px, py in hull_points:
                        p = Point()
                        p.x = px
                        p.y = py
                        p.z = center_z  # Use cluster center Z
                        hull_marker.points.append(p)
                    
                    # Close the hull
                    if hull_points:
                        p = Point()
                        p.x = hull_points[0][0]
                        p.y = hull_points[0][1]
                        p.z = center_z
                        hull_marker.points.append(p)
                    
                    # Set line width and color
                    hull_marker.scale.x = 0.05  # Line width
                    hull_marker.color.r = r
                    hull_marker.color.g = g
                    hull_marker.color.b = b
                    hull_marker.color.a = 0.6  # Semi-transparent
                    
                    # Set lifetime
                    hull_marker.lifetime.sec = int(self.marker_lifetime)
                    hull_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                    
                    marker_array.markers.append(hull_marker)
                
                # Add cluster label and info
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = current_time
                text_marker.ns = f"radar_labels"
                text_marker.id = cluster_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                
                # Position above cluster center
                text_marker.pose.position.x = center_x
                text_marker.pose.position.y = center_y
                text_marker.pose.position.z = center_z + 1.0  # 1m above cluster
                
                # Cluster info text
                classification = "Unknown"
                if avg_velocity > 15.0:
                    classification = "Fast Vehicle"
                elif avg_velocity > 5.0:
                    classification = "Vehicle"
                elif avg_velocity > 1.0:
                    classification = "Slow Vehicle/Bicycle"
                else:
                    classification = "Stationary Object"
                
                text_marker.text = f"Cluster {cluster_id}\n{num_points} points\nAvg V: {avg_velocity:.1f} m/s\nMax V: {max_velocity:.1f} m/s\n{classification}"
                
                # Text size and color
                text_marker.scale.z = 0.5  # Text height
                text_marker.color.r = r
                text_marker.color.g = g
                text_marker.color.b = b
                text_marker.color.a = 1.0
                
                # Set lifetime
                text_marker.lifetime.sec = int(self.marker_lifetime)
                text_marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                
                marker_array.markers.append(text_marker)
            
            # Publish the marker array
            self.marker_publisher.publish(marker_array)
            
            # Remove processed data from buffer
            self.data_buffer = self.data_buffer[offset:]
            
        except Exception as e:
            self.get_logger().error(f'Error processing message: {str(e)}')
            self.data_buffer.clear()
            self.stats['parse_errors'] += 1

    def print_statistics(self):
        """Print node statistics and performance metrics"""
        if not self.stats['last_data_time']:
            self.get_logger().info("No data received yet.")
            return
            
        time_since_last_data = time.time() - self.stats['last_data_time']
        
        self.get_logger().info(
            f"\n=== Radar Visualizer Statistics ===\n"
            f"Connection status: {'Connected' if self.connected else 'Disconnected'}\n"
            f"Last data received: {time_since_last_data:.1f} seconds ago\n"
            f"Total points received: {self.stats['total_points_received']}\n"
            f"Total clusters received: {self.stats['total_clusters_received']}\n"
            f"Largest cluster size: {self.stats['largest_cluster_size']} points\n"
            f"Connection attempts: {self.stats['connection_attempts']}\n"
            f"Connection failures: {self.stats['connection_failures']}\n"
            f"Parse errors: {self.stats['parse_errors']}\n"
            f"Current buffer size: {len(self.data_buffer)} bytes\n"
            f"==============================="
        )

    def cleanup(self):
        """Clean up resources before node shutdown"""
        self.get_logger().info('Shutting down Enhanced Radar Visualizer')
        
        # Clear all markers by publishing empty marker array with DELETE action
        try:
            marker_array = MarkerArray()
            
            # Create deletion markers for all namespaces
            for ns in ["radar_points", "velocity_vectors", "radar_hulls", "radar_labels"]:
                delete_marker = Marker()
                delete_marker.header.frame_id = "map"
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.ns = ns
                delete_marker.id = 0
                delete_marker.action = Marker.DELETEALL
                marker_array.markers.append(delete_marker)
            
            self.marker_publisher.publish(marker_array)
            self.get_logger().info('Published marker cleanup message')
        except Exception as e:
            self.get_logger().error(f'Error during marker cleanup: {str(e)}')
        
        # Close socket connection
        if self.socket:
            try:
                self.socket.close()
                self.get_logger().info('Closed TCP connection')
            except Exception as e:
                self.get_logger().error(f'Error closing socket: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = EnhancedRadarVisualizer()
    
    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {str(e)}')
    finally:
        # Clean up
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 