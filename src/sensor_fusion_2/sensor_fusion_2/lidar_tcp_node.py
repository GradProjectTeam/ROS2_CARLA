#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
import socket
import struct
import numpy as np
import colorsys
import time
import math

class LidarTcpNode(Node):
    def __init__(self):
        super().__init__('lidar_tcp_node')
        
        # ROS2 Publishers
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/lidar/markers', 10)
        self.hull_publisher = self.create_publisher(MarkerArray, '/lidar/hulls', 10)
        self.cube_publisher = self.create_publisher(MarkerArray, '/lidar/cubes', 10)
        
        # Parameter declaration
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12350)
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('point_size', 2.0)
        self.declare_parameter('center_size', 3.0)
        self.declare_parameter('use_convex_hull', True)
        self.declare_parameter('use_point_markers', True)
        self.declare_parameter('use_cluster_stats', True)
        self.declare_parameter('verbose_logging', False)
        self.declare_parameter('cube_alpha', 0.3)  # Transparency for cubes
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.frame_id = self.get_parameter('frame_id').value  # Use this frame ID consistently
        self.point_size = self.get_parameter('point_size').value
        self.center_size = self.get_parameter('center_size').value
        self.use_convex_hull = self.get_parameter('use_convex_hull').value
        self.use_point_markers = self.get_parameter('use_point_markers').value
        self.use_cluster_stats = self.get_parameter('use_cluster_stats').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.cube_alpha = self.get_parameter('cube_alpha').value
        
        # Connection statistics
        self.socket = None
        self.connected = False
        self.last_receive_time = time.time()
        self.points_received = 0
        self.clusters_received = 0
        self.data_buffer = bytearray()
        self.max_buffer_size = 4194304  # 4MB buffer size limit (LiDAR data can be large)
        
        # Print welcome message
        self.print_welcome_message()
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.receive_data)  # 100Hz processing
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
        # Timer for statistics reporting
        self.create_timer(1.0, self.report_stats)

    def print_welcome_message(self):
        self.get_logger().info("=" * 40)
        self.get_logger().info("LiDAR TCP Node - Direct Cluster Processing")
        self.get_logger().info(f"Connecting to LiDAR server at {self.tcp_ip}:{self.tcp_port}")
        self.get_logger().info(f"LiDAR frame: {self.frame_id}")
        self.get_logger().info("No filtering applied - directly processing raw clusters")
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
            self.data_buffer.clear()  # Clear buffer on new connection
            self.get_logger().info(f'Connected to LiDAR server at {self.tcp_ip}:{self.tcp_port}')
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f'Failed to connect to LiDAR server: {str(e)}')
    
    def check_connection(self):
        """Check if connection is still valid and reconnect if needed"""
        if not self.connected:
            self.get_logger().info('Attempting to reconnect to LiDAR server...')
            self.connect()
    
    def report_stats(self):
        """Report statistics about data reception"""
        now = time.time()
        elapsed = now - self.last_receive_time
        if elapsed > 0:
            points_per_sec = self.points_received / elapsed
            clusters_per_sec = self.clusters_received / elapsed
            
            stats_msg = f'Stats: {points_per_sec:.1f} points/s, {clusters_per_sec:.1f} clusters/s'
            self.get_logger().info(stats_msg)
            
            self.points_received = 0
            self.clusters_received = 0
            self.last_receive_time = now
    
    def generate_colors(self, n):
        """Generate visually distinct colors using HSV color space"""
        colors = []
        for i in range(n):
            # Use golden ratio to create well-distributed hues
            h = (i * 0.618033988749895) % 1.0
            s = 0.8 + 0.2 * (i % 2)  # Alternate between 0.8 and 1.0 saturation
            v = 0.9  # Keep value high for visibility
            
            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            colors.append((r, g, b))
        return colors
    
    def calculate_convex_hull_2d(self, points):
        """Calculate the 2D convex hull for a set of points (Graham scan algorithm)"""
        if len(points) < 3:
            return points
            
        # Find the lowest point
        lowest = min(range(len(points)), key=lambda i: (points[i][1], points[i][0]))
        
        # Sort points by polar angle with respect to the lowest point
        def polar_angle(p):
            return math.atan2(p[1] - points[lowest][1], p[0] - points[lowest][0])
            
        sorted_points = sorted(points, key=lambda p: (polar_angle(p), p[0], p[1]))
        
        # Build the hull
        hull = [sorted_points[0], sorted_points[1]]
        
        for i in range(2, len(sorted_points)):
            while len(hull) > 1:
                # Cross product to determine turn direction
                x1, y1 = hull[-1][0] - hull[-2][0], hull[-1][1] - hull[-2][1]
                x2, y2 = sorted_points[i][0] - hull[-1][0], sorted_points[i][1] - hull[-1][1]
                cross_product = x1 * y2 - y1 * x2
                
                # If not a left turn, remove the last point
                if cross_product <= 0:
                    hull.pop()
                else:
                    break
                    
            hull.append(sorted_points[i])
            
        return hull
    
    def has_complete_data(self, num_clusters):
        """Check if we have complete data for all clusters"""
        buffer_len = len(self.data_buffer)
        
        # Start after the 4 bytes that store the number of clusters
        current_offset = 4
        
        for _ in range(num_clusters):
            # Need 4 bytes for number of points
            if current_offset + 4 > buffer_len:
                return False
            
            # Get number of points in this cluster
            num_points = struct.unpack('!I', self.data_buffer[current_offset:current_offset + 4])[0]
            current_offset += 4
            
            # Each point is 16 bytes (x, y, z, intensity as floats)
            points_size = num_points * 16
            if current_offset + points_size > buffer_len:
                return False
            
            current_offset += points_size
        
        return True
    
    def receive_data(self):
        """Process incoming LiDAR data directly without filtering"""
        if not self.connected:
            return
            
        try:
            # Try to receive data
            chunk = self.socket.recv(65536)  # Receive data in larger chunks for LiDAR
            if not chunk:
                self.get_logger().warning('Connection closed by server')
                self.connected = False
                return
                
            # Append new data to the buffer
            self.data_buffer.extend(chunk)
            
            # Limit the buffer size
            if len(self.data_buffer) > self.max_buffer_size:
                self.get_logger().warning('Data buffer size exceeded, clearing buffer.')
                self.data_buffer.clear()  # Clear the buffer if it exceeds the limit
            
            # Process complete packets in buffer
            while len(self.data_buffer) >= 4:  # Ensure we have at least 4 bytes for the number of clusters
                # Unpack the number of clusters
                if len(self.data_buffer) < 4:
                    break
                    
                num_clusters = struct.unpack('!I', self.data_buffer[:4])[0]
                self.get_logger().debug(f'Received data for {num_clusters} clusters')
                
                # Check if we have enough data for the full message
                if not self.has_complete_data(num_clusters):
                    break
                
                # Create marker arrays for visualization
                point_markers = MarkerArray()
                hull_markers = MarkerArray()
                cube_markers = MarkerArray()
                
                # Create a list to store all points for the point cloud
                cloud_points = []
                
                # Generate colors for clusters
                colors = self.generate_colors(num_clusters)
                
                # Move past the number of clusters
                offset = 4
                
                # Process each cluster
                for cluster_id in range(num_clusters):
                    # Get number of points in this cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    offset += 4
                    
                    # Skip empty clusters
                    if num_points == 0:
                        continue
                        
                    self.clusters_received += 1
                    
                    # Prepare lists for this cluster
                    cluster_points = []
                    cluster_points_2d = []  # For 2D hull calculation
                    cluster_intensities = []
                    
                    # Calculate cluster color
                    r, g, b = colors[cluster_id % len(colors)]
                    
                    # Track cluster bounds for cube calculation
                    min_x, max_x = float('inf'), float('-inf')
                    min_y, max_y = float('inf'), float('-inf')
                    min_z, max_z = float('inf'), float('-inf')
                    
                    # Process each point in the cluster - DIRECT PROCESSING WITHOUT FILTERING
                    for i in range(num_points):
                        # Unpack point data: x, y, z, intensity
                        x, y, z, intensity = struct.unpack('!ffff', self.data_buffer[offset:offset + 16])
                        offset += 16
                        
                        # Update cluster bounds
                        min_x = min(min_x, x)
                        max_x = max(max_x, x)
                        min_y = min(min_y, y)
                        max_y = max(max_y, y)
                        min_z = min(min_z, z)
                        max_z = max(max_z, z)
                        
                        # Add the point to cluster lists
                        cluster_points.append((x, y, z))
                        cluster_points_2d.append((x, y))
                        cluster_intensities.append(intensity)
                        
                        # Add the point to the point cloud
                        cloud_points.append((x, y, z, intensity))
                        self.points_received += 1
                    
                    # Skip if no points are in the cluster
                    if not cluster_points:
                        continue
                    
                    # Add point markers if enabled
                    if self.use_point_markers:
                        for i, (x, y, z) in enumerate(cluster_points):
                            marker = Marker()
                            marker.header.frame_id = self.frame_id
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.ns = f"lidar_cluster_{cluster_id}"
                            marker.id = i
                            marker.type = Marker.SPHERE
                            marker.action = Marker.ADD
                            marker.pose.position.x = x
                            marker.pose.position.y = y
                            marker.pose.position.z = z
                            marker.scale.x = self.point_size
                            marker.scale.y = self.point_size
                            marker.scale.z = self.point_size
                            marker.color.r = r
                            marker.color.g = g
                            marker.color.b = b
                            marker.color.a = 0.7
                            point_markers.markers.append(marker)
                    
                    # Calculate the centroid of the cluster
                    centroid_x = sum(p[0] for p in cluster_points) / len(cluster_points)
                    centroid_y = sum(p[1] for p in cluster_points) / len(cluster_points)
                    centroid_z = sum(p[2] for p in cluster_points) / len(cluster_points)
                    
                    # Add a marker for the centroid (larger sphere)
                    centroid_marker = Marker()
                    centroid_marker.header.frame_id = self.frame_id
                    centroid_marker.header.stamp = self.get_clock().now().to_msg()
                    centroid_marker.ns = f"lidar_cluster_centroid"
                    centroid_marker.id = cluster_id
                    centroid_marker.type = Marker.SPHERE
                    centroid_marker.action = Marker.ADD
                    centroid_marker.pose.position.x = centroid_x
                    centroid_marker.pose.position.y = centroid_y
                    centroid_marker.pose.position.z = centroid_z
                    centroid_marker.scale.x = self.center_size
                    centroid_marker.scale.y = self.center_size
                    centroid_marker.scale.z = self.center_size
                    centroid_marker.color.r = 1.0
                    centroid_marker.color.g = 1.0
                    centroid_marker.color.b = 1.0
                    centroid_marker.color.a = 0.9
                    point_markers.markers.append(centroid_marker)
                    
                    # Add convex hull markers if enabled
                    if self.use_convex_hull:
                        hull_points = self.calculate_convex_hull_2d(cluster_points_2d)
                        
                        # Only draw hull if we have at least 3 points
                        if len(hull_points) >= 3:
                            hull_marker = Marker()
                            hull_marker.header.frame_id = self.frame_id
                            hull_marker.header.stamp = self.get_clock().now().to_msg()
                            hull_marker.ns = "lidar_hulls"
                            hull_marker.id = cluster_id
                            hull_marker.type = Marker.LINE_STRIP
                            hull_marker.action = Marker.ADD
                            hull_marker.scale.x = 0.05  # Line width
                            hull_marker.color.r = r * 0.8
                            hull_marker.color.g = g * 0.8
                            hull_marker.color.b = b * 0.8
                            hull_marker.color.a = 1.0
                            
                            # Add hull points to line strip (project to ground plane z=0)
                            for x, y in hull_points:
                                p = Point()
                                p.x = x
                                p.y = y
                                p.z = 0.05  # Slightly above ground
                                hull_marker.points.append(p)
                            
                            # Close the loop
                            if hull_points:
                                p = Point()
                                p.x = hull_points[0][0]
                                p.y = hull_points[0][1]
                                p.z = 0.05
                                hull_marker.points.append(p)
                            
                            hull_markers.markers.append(hull_marker)
                    
                    # Add a cube marker for the cluster
                    cube_marker = Marker()
                    cube_marker.header.frame_id = self.frame_id
                    cube_marker.header.stamp = self.get_clock().now().to_msg()
                    cube_marker.ns = "lidar_cubes"
                    cube_marker.id = cluster_id
                    cube_marker.type = Marker.CUBE
                    cube_marker.action = Marker.ADD
                    
                    # Set cube position to the center of the bounds
                    cube_marker.pose.position.x = (min_x + max_x) / 2
                    cube_marker.pose.position.y = (min_y + max_y) / 2
                    cube_marker.pose.position.z = (min_z + max_z) / 2
                    
                    # Set cube scale (dimensions)
                    cube_marker.scale.x = max(0.1, max_x - min_x)  # Ensure minimum size
                    cube_marker.scale.y = max(0.1, max_y - min_y)
                    cube_marker.scale.z = max(0.1, max_z - min_z)
                    
                    # Set cube color (same as cluster but transparent)
                    cube_marker.color.r = r
                    cube_marker.color.g = g
                    cube_marker.color.b = b
                    cube_marker.color.a = self.cube_alpha  # Transparent
                    
                    cube_markers.markers.append(cube_marker)
                    
                    # Add text label with cluster information
                    if self.use_cluster_stats:
                        text_marker = Marker()
                        text_marker.header.frame_id = self.frame_id
                        text_marker.header.stamp = self.get_clock().now().to_msg()
                        text_marker.ns = f"cluster_label_{cluster_id}"
                        text_marker.id = 0
                        text_marker.type = Marker.TEXT_VIEW_FACING
                        text_marker.text = f"Cluster {cluster_id}\n{len(cluster_points)} points"
                        text_marker.scale.z = 0.5  # Text size
                        text_marker.pose.position.x = centroid_x
                        text_marker.pose.position.y = centroid_y
                        text_marker.pose.position.z = centroid_z + 1.0  # Position above centroid
                        text_marker.color.r = 1.0
                        text_marker.color.g = 1.0
                        text_marker.color.b = 1.0
                        text_marker.color.a = 1.0
                        point_markers.markers.append(text_marker)
                
                # Publish markers
                if self.use_point_markers and point_markers.markers:
                    self.marker_publisher.publish(point_markers)
                
                # Publish hulls
                if self.use_convex_hull and hull_markers.markers:
                    self.hull_publisher.publish(hull_markers)
                
                # Publish cubes
                if cube_markers.markers:
                    self.cube_publisher.publish(cube_markers)
                
                # Publish point cloud if we have points
                if cloud_points:
                    self.publish_point_cloud(cloud_points)
                
                # Remove processed data from the buffer
                self.data_buffer = self.data_buffer[offset:]
                
        except BlockingIOError:
            # No data available, not an error for non-blocking socket
            pass
        except ConnectionResetError:
            self.get_logger().error('Connection reset by server')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Error processing data: {str(e)}')
            self.connected = False
    
    def publish_point_cloud(self, points_data):
        """Publish LiDAR data as a PointCloud2 message"""
        # Create point cloud message
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = Header()
        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        point_cloud_msg.header.frame_id = self.frame_id
        
        # Point field definitions
        point_cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Set other message properties
        point_cloud_msg.point_step = 16  # 4 floats * 4 bytes
        point_cloud_msg.height = 1
        point_cloud_msg.width = len(points_data)
        point_cloud_msg.is_dense = True
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width
        
        # Convert points data to bytes
        data_bytes = bytearray()
        for point in points_data:
            for value in point:
                data_bytes.extend(struct.pack('!f', value))
        
        point_cloud_msg.data = bytes(data_bytes)
        
        # Publish the point cloud
        self.publisher.publish(point_cloud_msg)
    
    def cleanup(self):
        """Clean up resources before node shutdown"""
        if self.socket:
            self.socket.close()
            self.get_logger().info('Closed LiDAR TCP connection')

def main(args=None):
    rclpy.init(args=args)
    node = LidarTcpNode()
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