#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import std_msgs.msg


class RadarClient_clusters(Node):
    def __init__(self):
        super().__init__('radar_client_clusters')
        
        # ROS2 Publishers
        self.points_publisher = self.create_publisher(PointCloud2, '/radar/points', 10)
        self.clusters_publisher = self.create_publisher(MarkerArray, '/radar/clusters', 10)
        self.markers_publisher = self.create_publisher(MarkerArray, '/radar/markers', 10)
        self.debug_publisher = self.create_publisher(std_msgs.msg.String, '/radar/debug', 10)
        self.monitor_publisher = self.create_publisher(std_msgs.msg.String, '/radar/monitor_info', 10)
        
        # TCP Client setup
        self.tcp_ip = '127.0.0.1'
        self.tcp_port = 12348 # ros port
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
                debug_msg = std_msgs.msg.String()
                debug_msg.data = f'Received data for {num_clusters} clusters'
                self.debug_publisher.publish(debug_msg)
                self.get_logger().info(debug_msg.data)
                offset = 4  # Start after the number of clusters

                # Create MarkerArrays to hold the markers for this update
                marker_array = MarkerArray()
                clusters_array = MarkerArray()

                # Create a PointCloud2 message for all points
                points = []

                for cluster_id in range(num_clusters):
                    if len(self.data_buffer) < offset + 4:
                        return  # Not enough data for the number of points

                    # Unpack the number of points in the current cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    debug_msg = std_msgs.msg.String()
                    debug_msg.data = f'Cluster {cluster_id} has {num_points} points'
                    self.debug_publisher.publish(debug_msg)
                    self.get_logger().info(debug_msg.data)
                    offset += 4  # Move to the point data

                    if len(self.data_buffer) < offset + num_points * 16:
                        return  # Not enough data for all points in the cluster

                    for point_index in range(num_points):
                        # Unpack the point data (altitude, azimuth, depth, velocity)
                        altitude, azimuth, depth, velocity = struct.unpack('!ffff', self.data_buffer[offset:offset + 16])
                        
                        # Print the received data
                        debug_msg = std_msgs.msg.String()
                        debug_msg.data = f'Point {point_index}: Altitude={altitude:.2f}m, Azimuth={azimuth:.2f}°, Depth={depth:.2f}m, Velocity={velocity:.2f}m/s'
                        self.debug_publisher.publish(debug_msg)
                        self.get_logger().info(debug_msg.data)
                        
                        offset += 16  # Move to the next point

                        # Convert polar coordinates to Cartesian coordinates
                        x = depth * np.cos(np.radians(azimuth))
                        y = depth * np.sin(np.radians(azimuth))
                        z = altitude

                        # Add point to the points list for PointCloud2
                        points.append((x, y, z, velocity))

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
                        marker.scale.x = 0.5  # Size of the sphere
                        marker.scale.y = 0.5
                        marker.scale.z = 0.5
                        marker.color.r = 0.2 + 0.8 * (cluster_id / max(1, num_clusters))  # Color based on cluster ID
                        marker.color.g = 0.2 + 0.8 * (1 - cluster_id / max(1, num_clusters))
                        marker.color.b = 0.5
                        marker.color.a = 1.0  # Alpha (transparency)

                        # Add the marker to the MarkerArray
                        marker_array.markers.append(marker)
                        
                        # Add to clusters array with different namespace
                        cluster_marker = Marker()
                        cluster_marker.header.frame_id = "map"
                        cluster_marker.header.stamp = self.get_clock().now().to_msg()
                        cluster_marker.ns = "radar_clusters"  # Different namespace for clusters
                        cluster_marker.id = cluster_id * 100 + point_index  # Unique ID
                        cluster_marker.type = Marker.SPHERE
                        cluster_marker.action = Marker.ADD
                        cluster_marker.pose.position.x = x
                        cluster_marker.pose.position.y = y
                        cluster_marker.pose.position.z = z
                        cluster_marker.scale.x = 0.5
                        cluster_marker.scale.y = 0.5
                        cluster_marker.scale.z = 0.5
                        cluster_marker.color.r = 1.0 if velocity > 0 else 0.0
                        cluster_marker.color.g = 0.0
                        cluster_marker.color.b = 1.0 if velocity < 0 else 0.0
                        cluster_marker.color.a = 1.0
                        clusters_array.markers.append(cluster_marker)

                    # Add a delete marker action for old markers
                    for old_index in range(num_points, 100):  # Assuming a maximum of 100 points
                        delete_marker = Marker()
                        delete_marker.header.frame_id = "map"
                        delete_marker.header.stamp = self.get_clock().now().to_msg()
                        delete_marker.ns = f"radar_cluster_{cluster_id}"
                        delete_marker.id = old_index  # ID of the old marker to delete
                        delete_marker.action = Marker.DELETE  # Action to delete the marker
                        marker_array.markers.append(delete_marker)

                # Create and publish PointCloud2 message if we have points
                if points:
                    pc2_msg = self.create_point_cloud2(points)
                    self.points_publisher.publish(pc2_msg)
                    monitor_msg = std_msgs.msg.String()
                    monitor_msg.data = f"Published PointCloud2 with {len(points)} points"
                    self.monitor_publisher.publish(monitor_msg)
                
                # Publish the MarkerArrays
                self.markers_publisher.publish(marker_array)
                self.clusters_publisher.publish(clusters_array)

                # Remove processed data from the buffer
                self.data_buffer = self.data_buffer[offset:]
                
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    def create_point_cloud2(self, points):
        """Create a PointCloud2 message from a list of points"""
        # Create the PointCloud2 message
        pc2 = PointCloud2()
        pc2.header.stamp = self.get_clock().now().to_msg()
        pc2.header.frame_id = "map"
        
        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Set the fields in the message
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 16  # 4 fields * 4 bytes
        pc2.row_step = pc2.point_step * len(points)
        
        # Convert points to bytes
        pc2.data = b''
        for point in points:
            pc2.data += struct.pack('ffff', *point)
        
        pc2.height = 1
        pc2.width = len(points)
        pc2.is_dense = True
        
        return pc2

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