#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray


class RadarClient_clusters(Node):
    def __init__(self):
        super().__init__('radar_client_clusters')
        
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

                for cluster_id in range(num_clusters):
                    if len(self.data_buffer) < offset + 4:
                        return  # Not enough data for the number of points

                    # Unpack the number of points in the current cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    self.get_logger().info(f'Cluster {cluster_id} has {num_points} points')
                    offset += 4  # Move to the point data

                    if len(self.data_buffer) < offset + num_points * 16:
                        return  # Not enough data for all points in the cluster

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

                        # Create a marker for each point in the cluster
                        marker = Marker()
                        marker.header.frame_id = "map"  # Set the frame of reference
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = f"radar_cluster_{cluster_id}"  # Namespace for the cluster
                        marker.id = point_index  # Unique ID for each point
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = x  # Set x position
                        marker.pose.position.y = y  # Set y position
                        marker.pose.position.z = z  # Set z position
                        marker.scale.x = 0.5  # Increased size of the sphere
                        marker.scale.y = 0.5
                        marker.scale.z = 0.5
                        marker.color.r = np.random.rand()  # Random color for each cluster
                        marker.color.g = np.random.rand()
                        marker.color.b = np.random.rand()
                        marker.color.a = 1.0  # Alpha (transparency)

                        # Add the marker to the MarkerArray
                        marker_array.markers.append(marker)

                    # Add a delete marker action for old markers
                    for old_index in range(num_points, 100):  # Assuming a maximum of 100 points
                        delete_marker = Marker()
                        delete_marker.header.frame_id = "map"
                        delete_marker.header.stamp = self.get_clock().now().to_msg()
                        delete_marker.ns = f"radar_cluster_{cluster_id}"
                        delete_marker.id = old_index  # ID of the old marker to delete
                        delete_marker.action = Marker.DELETE  # Action to delete the marker
                        marker_array.markers.append(delete_marker)

                # Publish the MarkerArray for the current cluster
                self.marker_publisher.publish(marker_array)

                # Remove processed data from the buffer
                self.data_buffer = self.data_buffer[offset:]
                
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    def __del__(self):
        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = RadarClient_clusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()