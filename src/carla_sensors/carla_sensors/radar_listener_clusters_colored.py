#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import math
from geometry_msgs.msg import Point


class RadarClient_clusters_colored(Node):
    def __init__(self):
        super().__init__('radar_client_clusters_colored')
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(PointCloud2, '/radar/points', 100)
        self.marker_publisher = self.create_publisher(MarkerArray, '/radar/markers', 100)
        
        # TCP Client setup
        self.tcp_ip = '127.0.0.1'
        self.tcp_port = 12348
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Initialize data buffer and max buffer size
        self.data_buffer = bytearray()
        self.max_buffer_size = 1048576  # 1MB buffer size limit
        
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

        # Create timer for receiving data
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz

    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        data = b''
        while len(data) < size:
            packet = self.socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

    def receive_data(self):
        try:
            # Read data in smaller chunks
            chunk = self.socket.recv(4096)  # Receive data in chunks
            if not chunk:
                self.get_logger().warning('No data received from TCP server.')
                return

            # Append new data to the buffer
            self.data_buffer.extend(chunk)

            # Limit the buffer size
            if len(self.data_buffer) > self.max_buffer_size:
                self.get_logger().warning('Data buffer size exceeded, clearing buffer.')
                self.data_buffer.clear()  # Clear the buffer if it exceeds the limit

            while len(self.data_buffer) >= 4:  # Ensure we have at least 4 bytes for the number of clusters
                # Unpack the number of clusters
                num_clusters = struct.unpack('!I', self.data_buffer[:4])[0]
                self.get_logger().info(f'Received data for {num_clusters} clusters')
                offset = 4  # Start after the number of clusters

                # Create a MarkerArray to hold the markers for this update
                marker_array = MarkerArray()
                
                # Generate a fixed color for each cluster
                cluster_colors = [
                    (1.0, 0.0, 0.0),  # Red
                    (0.0, 1.0, 0.0),  # Green
                    (0.0, 0.0, 1.0),  # Blue
                    (1.0, 1.0, 0.0),  # Yellow
                    (1.0, 0.0, 1.0),  # Magenta
                    (0.0, 1.0, 1.0),  # Cyan
                    (1.0, 0.5, 0.0),  # Orange
                    (0.5, 0.0, 1.0),  # Purple
                    (0.0, 0.5, 0.0),  # Dark Green
                    (0.5, 0.5, 1.0),  # Light Blue
                ]

                for cluster_id in range(num_clusters):
                    if len(self.data_buffer) < offset + 4:
                        return  # Not enough data for the number of points

                    # Unpack the number of points in the current cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    self.get_logger().info(f'Cluster {cluster_id} has {num_points} points')
                    offset += 4  # Move to the point data

                    if len(self.data_buffer) < offset + num_points * 16:
                        return  # Not enough data for all points in the cluster

                    # Get color for this cluster
                    color_idx = cluster_id % len(cluster_colors)
                    r, g, b = cluster_colors[color_idx]
                    
                    # Store points for hull calculation
                    cluster_points = []

                    for point_index in range(num_points):
                        # Unpack the point data (altitude, azimuth, depth, velocity)
                        altitude, azimuth, depth, velocity = struct.unpack('!ffff', self.data_buffer[offset:offset + 16])
                        
                        # Print the received data
                        self.get_logger().info(
                            f'Point {point_index}: '
                            f'Altitude={altitude:.2f}m, '
                            f'Azimuth={azimuth:.2f}°, '
                            f'Depth={depth:.2f}m, '
                            f'Velocity={velocity:.2f}m/s'
                        )
                        
                        offset += 16  # Move to the next point

                        # Convert polar coordinates to Cartesian coordinates
                        x = depth * np.cos(azimuth)
                        y = depth * np.sin(azimuth)
                        z = altitude

                        # Point marker
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = f"radar_cluster_{cluster_id}"
                        marker.id = point_index
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = x
                        marker.pose.position.y = y
                        marker.pose.position.z = z
                        marker.scale.x = 0.3  # Smaller spheres
                        marker.scale.y = 0.3
                        marker.scale.z = 0.3
                        marker.color.r = r
                        marker.color.g = g
                        marker.color.b = b
                        marker.color.a = 1.0
                        marker_array.markers.append(marker)
                        
                        cluster_points.append((x, y, z))

                    # Add cluster hull if we have enough points
                    if len(cluster_points) >= 3:
                        hull_marker = Marker()
                        hull_marker.header.frame_id = "map"
                        hull_marker.header.stamp = self.get_clock().now().to_msg()
                        hull_marker.ns = f"radar_cluster_hull_{cluster_id}"
                        hull_marker.id = 0
                        hull_marker.type = Marker.LINE_STRIP
                        hull_marker.action = Marker.ADD
                        hull_marker.scale.x = 0.1  # Line width
                        hull_marker.color.r = r
                        hull_marker.color.g = g
                        hull_marker.color.b = b
                        hull_marker.color.a = 0.5  # Semi-transparent

                        # Create hull points (simplified 2D hull)
                        points_2d = [(p[0], p[1]) for p in cluster_points]
                        center_x = sum(p[0] for p in points_2d) / len(points_2d)
                        center_y = sum(p[1] for p in points_2d) / len(points_2d)
                        
                        # Sort points by angle from center
                        hull_points = sorted(points_2d, 
                            key=lambda p: math.atan2(p[1] - center_y, p[0] - center_x))
                        
                        # Add hull vertices
                        for px, py in hull_points:
                            p = Point()
                            p.x = px
                            p.y = py
                            p.z = sum(p[2] for p in cluster_points) / len(cluster_points)  # Average height
                            hull_marker.points.append(p)
                        
                        # Close the hull
                        if hull_points:
                            p = Point()
                            p.x = hull_points[0][0]
                            p.y = hull_points[0][1]
                            p.z = sum(p[2] for p in cluster_points) / len(cluster_points)
                            hull_marker.points.append(p)
                        
                        marker_array.markers.append(hull_marker)

                    # Add cluster label
                    if cluster_points:
                        text_marker = Marker()
                        text_marker.header.frame_id = "map"
                        text_marker.header.stamp = self.get_clock().now().to_msg()
                        text_marker.ns = f"radar_cluster_label_{cluster_id}"
                        text_marker.id = 0
                        text_marker.type = Marker.TEXT_VIEW_FACING
                        text_marker.action = Marker.ADD
                        text_marker.pose.position.x = sum(p[0] for p in cluster_points) / len(cluster_points)
                        text_marker.pose.position.y = sum(p[1] for p in cluster_points) / len(cluster_points)
                        text_marker.pose.position.z = sum(p[2] for p in cluster_points) / len(cluster_points) + 1.0
                        text_marker.text = f"Cluster {cluster_id}"
                        text_marker.scale.z = 0.5  # Text height
                        text_marker.color.r = r
                        text_marker.color.g = g
                        text_marker.color.b = b
                        text_marker.color.a = 1.0
                        marker_array.markers.append(text_marker)

                # Publish the MarkerArray
                self.marker_publisher.publish(marker_array)

                # Remove processed data from the buffer
                self.data_buffer = self.data_buffer[offset:]
                
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    def __del__(self):
        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = RadarClient_clusters_colored()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()