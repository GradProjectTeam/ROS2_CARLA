#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import socket
import struct
import numpy as np


class LidarClient_clusters(Node):
    def __init__(self):
        super().__init__('lidar_client_clusters')
        
        # ROS2 Publishers
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 100)
        self.marker_publisher = self.create_publisher(MarkerArray, '/lidar/markers', 100)
        
        # TCP Client setup
        self.tcp_ip = '127.0.0.1'
        self.tcp_port = 12350
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
            # Receive number of clusters
            data = self.receive_exact(4)
            if not data:
                return
            num_clusters = struct.unpack('!I', data)[0]
            self.get_logger().info(f'Receiving {num_clusters} clusters')
            
            # Create MarkerArray message
            marker_array = MarkerArray()
            
            # Define cluster colors with more distinct colors
            cluster_colors = [
                (1.0, 0.0, 0.0),  # Red
                (0.0, 1.0, 0.0),  # Green
                (0.0, 0.0, 1.0),  # Blue
                (1.0, 1.0, 0.0),  # Yellow
                (1.0, 0.0, 1.0),  # Magenta
                (0.0, 1.0, 1.0),  # Cyan
                (1.0, 0.5, 0.0),  # Orange
                (0.5, 0.0, 1.0),  # Purple
            ]

            all_points = []
            total_points = 0
            
            # Process each cluster
            for cluster_id in range(num_clusters):
                # Get number of points in this cluster
                size_data = self.receive_exact(4)
                if not size_data:
                    return
                cluster_size = struct.unpack('!I', size_data)[0]
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
                    
                    # Verify the data
                    if not (isinstance(x, float) and isinstance(y, float) and isinstance(z, float)):
                        self.get_logger().error(f'Invalid point data type: {type(x)}, {type(y)}, {type(z)}')
                        continue
                    
                    if not (abs(x) < 1000 and abs(y) < 1000 and abs(z) < 1000):  # Sanity check
                        self.get_logger().warn(f'Suspicious point values: {x}, {y}, {z}')
                        continue
                    
                    self.get_logger().info(f'Point {point_id}: x={x:.3f}, y={y:.3f}, z={z:.3f}')
                    
                    cluster_points.append([x, y, z])
                    all_points.append([x, y, z])
                    total_points += 1

                    # Create marker for this point
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f"lidar_cluster_{cluster_id}"
                    marker.id = total_points  # Unique ID for each point
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = x
                    marker.pose.position.y = y
                    marker.pose.position.z = z
                    
                    # Make points more visible
                    marker.scale.x = 2.0  # Increased from 1.0 to 2.0
                    marker.scale.y = 2.0
                    marker.scale.z = 2.0
                    
                    marker.color.r = color[0]
                    marker.color.g = color[1]
                    marker.color.b = color[2]
                    marker.color.a = 1.0
                    
                    marker_array.markers.append(marker)

                # Add cluster label if cluster has points
                if cluster_points:
                    # Calculate cluster center
                    center_x = sum(p[0] for p in cluster_points) / len(cluster_points)
                    center_y = sum(p[1] for p in cluster_points) / len(cluster_points)
                    center_z = sum(p[2] for p in cluster_points) / len(cluster_points)
                    
                    # Add cluster center marker
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
                    center_marker.scale.x = 3.0  # Increased from 1.0 to 3.0
                    center_marker.scale.y = 3.0
                    center_marker.scale.z = 3.0
                    center_marker.color.r = color[0]
                    center_marker.color.g = color[1]
                    center_marker.color.b = color[2]
                    center_marker.color.a = 0.8
                    marker_array.markers.append(center_marker)
                    
                    # Add text label
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = f"cluster_label_{cluster_id}"
                    text_marker.id = 0
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.text = f"Cluster {cluster_id}\n{len(cluster_points)} points"
                    text_marker.scale.z = 0.3  # Text height
                    text_marker.pose.position.x = center_x
                    text_marker.pose.position.y = center_y
                    text_marker.pose.position.z = center_z + 0.5
                    text_marker.color = center_marker.color
                    marker_array.markers.append(text_marker)

            # Print summary
            self.get_logger().info(f'Total points received: {total_points}')

            # Publish markers
            self.marker_publisher.publish(marker_array)
            
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
    node = LidarClient_clusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()