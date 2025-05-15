#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import socket
import struct
import numpy as np
import colorsys
import time
import math


class LidarClientSimple(Node):
    def __init__(self):
        super().__init__('lidar_client_simple')
        
        # ROS2 Publishers
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/lidar/markers', 10)
        self.cube_publisher = self.create_publisher(MarkerArray, '/lidar/cubes', 10)
        
        # Parameter declaration with defaults
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12350)
        self.declare_parameter('point_size', 2.0)
        self.declare_parameter('center_size', 3.0)
        self.declare_parameter('cube_alpha', 0.3)
        self.declare_parameter('verbose_logging', False)
        
        # CARLA LiDAR configuration parameters
        self.declare_parameter('lidar_channels', 32)
        self.declare_parameter('lidar_points_per_second', 120000)
        self.declare_parameter('lidar_rotation_frequency', 25.0)
        self.declare_parameter('lidar_range', 80.0)
        self.declare_parameter('lidar_upper_fov', -5.0)
        self.declare_parameter('lidar_lower_fov', -15.0)
        self.declare_parameter('lidar_height', 3.0)  # Height from ground in meters
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.point_size = self.get_parameter('point_size').value
        self.center_size = self.get_parameter('center_size').value
        self.cube_alpha = self.get_parameter('cube_alpha').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        
        # Get LiDAR configuration
        self.lidar_channels = self.get_parameter('lidar_channels').value
        self.lidar_points_per_second = self.get_parameter('lidar_points_per_second').value
        self.lidar_rotation_frequency = self.get_parameter('lidar_rotation_frequency').value
        self.lidar_range = self.get_parameter('lidar_range').value
        self.lidar_upper_fov = self.get_parameter('lidar_upper_fov').value
        self.lidar_lower_fov = self.get_parameter('lidar_lower_fov').value
        self.lidar_height = self.get_parameter('lidar_height').value
        
        # Log LiDAR configuration
        self.get_logger().info(f"LiDAR Configuration:")
        self.get_logger().info(f"  Channels: {self.lidar_channels}")
        self.get_logger().info(f"  Points per second: {self.lidar_points_per_second}")
        self.get_logger().info(f"  Rotation frequency: {self.lidar_rotation_frequency} Hz")
        self.get_logger().info(f"  Range: {self.lidar_range} meters")
        self.get_logger().info(f"  FOV: {self.lidar_upper_fov}° to {self.lidar_lower_fov}°")
        self.get_logger().info(f"  Height: {self.lidar_height} meters")
        
        # Connection statistics
        self.last_receive_time = time.time()
        self.points_received = 0
        self.clusters_received = 0
        
        # TCP Client setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Connect to server
        self.get_logger().info(f'Attempting to connect to {self.tcp_ip}:{self.tcp_port}...')
        try:
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info('Connected to server successfully')
        except ConnectionRefusedError:
            self.get_logger().error('Connection refused. Make sure the server is running.')
            raise
        except Exception as e:
            self.get_logger().error(f'Connection failed: {str(e)}')
            raise

        # Create timers
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz
        self.stats_timer = self.create_timer(1.0, self.report_stats)  # 1Hz stats

    def report_stats(self):
        """Report statistics about data reception"""
        now = time.time()
        elapsed = now - self.last_receive_time
        if elapsed > 0:
            points_per_sec = self.points_received / elapsed
            clusters_per_sec = self.clusters_received / elapsed
            
            # Calculate theoretical vs actual points
            expected_points = self.lidar_points_per_second
            efficiency = (points_per_sec / expected_points) * 100 if expected_points > 0 else 0
            
            stats_msg = f'Stats: {points_per_sec:.1f} points/s ({efficiency:.1f}% of capacity), {clusters_per_sec:.1f} clusters/s'
            self.get_logger().info(stats_msg)
            
            self.points_received = 0
            self.clusters_received = 0
            self.last_receive_time = now

    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        data = b''
        while len(data) < size:
            packet = self.socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

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

    def receive_data(self):
        try:
            # Receive number of clusters
            data = self.receive_exact(4)
            if not data:
                return
            num_clusters = struct.unpack('!I', data)[0]
            
            if self.verbose_logging:
                self.get_logger().info(f'Receiving {num_clusters} clusters')
            
            self.clusters_received += num_clusters
            
            # Create marker arrays
            point_marker_array = MarkerArray()
            cube_marker_array = MarkerArray()
            
            # Generate cluster colors - more visually distinct
            cluster_colors = self.generate_colors(max(8, num_clusters))

            all_points = []
            total_points = 0
            
            # Clean up previous markers when needed
            if num_clusters == 0:
                # Clear all markers by publishing empty arrays
                self.marker_publisher.publish(MarkerArray())
                self.cube_publisher.publish(MarkerArray())
                return
            
            # Process each cluster
            for cluster_id in range(num_clusters):
                # Get number of points in this cluster
                size_data = self.receive_exact(4)
                if not size_data:
                    return
                cluster_size = struct.unpack('!I', size_data)[0]
                
                if self.verbose_logging:
                    self.get_logger().info(f'Cluster {cluster_id}: expecting {cluster_size} points')
                
                # Get color for this cluster
                color = cluster_colors[cluster_id % len(cluster_colors)]
                
                cluster_points = []
                # Process each point in the cluster
                for point_id in range(cluster_size):
                    point_data = self.receive_exact(12)  # 3 * float32
                    if not point_data:
                        return
                    x, y, z = struct.unpack('!fff', point_data)
                    
                    # Very basic validation - just check for NaN and Inf
                    if (math.isnan(x) or math.isnan(y) or math.isnan(z) or
                        math.isinf(x) or math.isinf(y) or math.isinf(z)):
                        continue
                        
                    # Check range only based on configuration
                    distance_xy = math.sqrt(x*x + y*y)
                    if distance_xy > self.lidar_range:
                        continue
                    
                    # Add the point - NO OTHER FILTERING
                    cluster_points.append([x, y, z])
                    all_points.append([x, y, z])
                    total_points += 1
                
                # Process the cluster if it has points
                if cluster_points:
                    self.points_received += len(cluster_points)
                    
                    # Calculate cluster center and dimensions
                    center_x = sum(p[0] for p in cluster_points) / len(cluster_points)
                    center_y = sum(p[1] for p in cluster_points) / len(cluster_points)
                    center_z = sum(p[2] for p in cluster_points) / len(cluster_points)
                    
                    # Calculate dimensions
                    min_x = min(p[0] for p in cluster_points)
                    max_x = max(p[0] for p in cluster_points)
                    min_y = min(p[1] for p in cluster_points)
                    max_y = max(p[1] for p in cluster_points)
                    min_z = min(p[2] for p in cluster_points)
                    max_z = max(p[2] for p in cluster_points)
                    
                    width = max_x - min_x
                    length = max_y - min_y
                    height = max_z - min_z
                    
                    # Calculate distance from LiDAR
                    dist_from_lidar = math.sqrt(center_x*center_x + center_y*center_y)
                    
                    # Use POINTS type for efficient visualization of all cluster points
                    points_marker = Marker()
                    points_marker.header.frame_id = "map"
                    points_marker.header.stamp = self.get_clock().now().to_msg()
                    points_marker.ns = f"cluster_points_{cluster_id}"
                    points_marker.id = 0
                    points_marker.type = Marker.POINTS
                    points_marker.action = Marker.ADD
                    
                    # Larger points for better visibility
                    points_marker.scale.x = self.point_size * 0.7
                    points_marker.scale.y = self.point_size * 0.7
                    
                    # Add all points to the POINTS marker
                    for point in cluster_points:
                        ros_point = Point()
                        ros_point.x = point[0]
                        ros_point.y = point[1]
                        ros_point.z = point[2]
                        points_marker.points.append(ros_point)
                        
                        # Add matching color for each point
                        color_rgba = ColorRGBA()
                        color_rgba.r = color[0]
                        color_rgba.g = color[1]
                        color_rgba.b = color[2]
                        color_rgba.a = 1.0  # Fully opaque points
                        points_marker.colors.append(color_rgba)
                    
                    point_marker_array.markers.append(points_marker)
                    
                    # Add cluster center marker (larger sphere)
                    center_marker = Marker()
                    center_marker.header.frame_id = "map"
                    center_marker.header.stamp = self.get_clock().now().to_msg()
                    center_marker.ns = f"cluster_center_{cluster_id}"
                    center_marker.id = 0
                    center_marker.type = Marker.SPHERE
                    center_marker.action = Marker.ADD
                    center_marker.pose.position.x = center_x
                    center_marker.pose.position.y = center_y
                    center_marker.pose.position.z = center_z
                    center_marker.scale.x = self.center_size
                    center_marker.scale.y = self.center_size
                    center_marker.scale.z = self.center_size
                    center_marker.color.r = color[0]
                    center_marker.color.g = color[1]
                    center_marker.color.b = color[2]
                    center_marker.color.a = 0.9
                    point_marker_array.markers.append(center_marker)
                    
                    # Add solid cube for the cluster 
                    cube_marker = Marker()
                    cube_marker.header.frame_id = "map"
                    cube_marker.header.stamp = self.get_clock().now().to_msg()
                    cube_marker.ns = f"cluster_cube_{cluster_id}"
                    cube_marker.id = 0
                    cube_marker.type = Marker.CUBE
                    cube_marker.action = Marker.ADD
                    
                    # Set cube position to cluster center
                    cube_marker.pose.position.x = center_x
                    cube_marker.pose.position.y = center_y
                    cube_marker.pose.position.z = center_z
                    
                    # Set orientation (identity quaternion)
                    cube_marker.pose.orientation.w = 1.0
                    
                    # Set cube dimensions
                    cube_marker.scale.x = width
                    cube_marker.scale.y = length
                    cube_marker.scale.z = height
                    
                    # Set cube color (semi-transparent to see points inside)
                    cube_marker.color.r = color[0]
                    cube_marker.color.g = color[1]
                    cube_marker.color.b = color[2]
                    cube_marker.color.a = self.cube_alpha  # Adjustable transparency
                    
                    cube_marker_array.markers.append(cube_marker)
                    
                    # Add text label with enhanced information
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = f"cluster_label_{cluster_id}"
                    text_marker.id = 0
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    
                    # Create more informative label including distance
                    text_marker.text = f"Cluster {cluster_id}\n" \
                                    f"Points: {len(cluster_points)}\n" \
                                    f"Size: {width:.1f}x{length:.1f}x{height:.1f}m\n" \
                                    f"Distance: {dist_from_lidar:.1f}m"
                    
                    text_marker.scale.z = 0.5  # Larger text
                    text_marker.pose.position.x = center_x
                    text_marker.pose.position.y = center_y
                    text_marker.pose.position.z = center_z + height/2 + 0.5  # Position above the object
                    text_marker.color.r = color[0]
                    text_marker.color.g = color[1]
                    text_marker.color.b = color[2]
                    text_marker.color.a = 1.0
                    point_marker_array.markers.append(text_marker)

            # Print summary (only if verbose logging is enabled)
            if self.verbose_logging:
                self.get_logger().info(f'Total points received: {total_points}')

            # Publish markers
            self.marker_publisher.publish(point_marker_array)
            self.cube_publisher.publish(cube_marker_array)
            
            # Publish point cloud
            if all_points:
                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                
                points_array = np.array(all_points, dtype=np.float32)
                msg.height = 1
                msg.width = len(all_points)
                msg.is_bigendian = False
                msg.point_step = 12
                msg.row_step = msg.point_step * len(all_points)
                msg.is_dense = True
                msg.data = points_array.tobytes()
                
                self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    def __del__(self):
        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarClientSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 