#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np


class LidarClient(Node):
    def __init__(self):
        super().__init__('lidar_client')
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 30)
        
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
            
            all_points = []
            
            # Process each cluster
            for cluster_idx in range(num_clusters):
                # Receive number of points in this cluster
                data = self.receive_exact(4)
                if not data:
                    return
                num_points = struct.unpack('!I', data)[0]
                self.get_logger().info(f'Cluster {cluster_idx + 1}: {num_points} points')
                
                # Receive all points in this cluster
                for point_idx in range(num_points):
                    point_data = self.receive_exact(12)  # 3 floats * 4 bytes
                    if not point_data:
                        return
                    
                    # Unpack the point data (network byte order)
                    x, y, z = struct.unpack('!fff', point_data)
                    all_points.append([x, y, z])
                    
                    # Print the received point data
                    self.get_logger().info(f'Received LiDAR Data - X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}')
                
            if all_points:  # Only publish if we have points
                # Create PointCloud2 message
                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # Set up the fields
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                
                # Convert points to numpy array
                points = np.array(all_points, dtype=np.float32)
                
                # Set message parameters
                msg.height = 1
                msg.width = len(points)
                msg.is_bigendian = False
                msg.point_step = 12  # 3 * float32 (4 bytes each)
                msg.row_step = msg.point_step * len(points)
                msg.is_dense = True
                msg.data = points.tobytes()
                
                # Publish the message
                self.publisher.publish(msg)
                self.get_logger().debug(f'Published {len(points)} points from {num_clusters} clusters')
                
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    def __del__(self):
        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()