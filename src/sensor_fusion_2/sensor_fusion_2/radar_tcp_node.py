#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import socket
import struct
import numpy as np
from std_msgs.msg import Header

class RadarTcpNode(Node):
    def __init__(self):
        super().__init__('radar_tcp_node')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12348)  # TCP port for radar data
        self.declare_parameter('frame_id', 'radar_link')
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        
        # ROS2 Publishers
        self.points_publisher = self.create_publisher(PointCloud2, '/radar/points', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/radar/markers', 10)
        
        # TCP socket related
        self.socket = None
        self.connected = False
        self.data_buffer = bytearray()
        self.max_buffer_size = 1048576  # 1MB buffer size limit
        
        # Print welcome message
        self.print_welcome_message()
        
        # Attempt initial connection
        self.connect()
        
        # Timer for data processing
        self.create_timer(0.01, self.receive_data)  # 100Hz processing
        
        # Timer for connection check
        self.create_timer(1.0, self.check_connection)
        
    def print_welcome_message(self):
        self.get_logger().info("=" * 40)
        self.get_logger().info("Radar TCP Node")
        self.get_logger().info(f"Connecting to radar server at {self.tcp_ip}:{self.tcp_port}")
        self.get_logger().info(f"Radar frame: {self.frame_id}")
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
            self.get_logger().info(f'Connected to radar server at {self.tcp_ip}:{self.tcp_port}')
        except Exception as e:
            self.connected = False
            self.get_logger().warning(f'Failed to connect to radar server: {str(e)}')
            
    def check_connection(self):
        """Check if connection is still valid and reconnect if needed"""
        if not self.connected:
            self.get_logger().info('Attempting to reconnect to radar server...')
            self.connect()
    
    def receive_data(self):
        """Process incoming radar data"""
        if not self.connected:
            return
            
        try:
            # Try to receive data
            chunk = self.socket.recv(4096)  # Receive data in chunks
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
                num_clusters = struct.unpack('!I', self.data_buffer[:4])[0]
                self.get_logger().debug(f'Received data for {num_clusters} clusters')
                
                # Check if we have enough data for all clusters
                offset = 4  # Start after the number of clusters
                
                # Break if we don't have enough data to process all clusters
                if not self.has_complete_data(offset, num_clusters):
                    break
                
                # Create a MarkerArray to hold all markers for this update
                marker_array = MarkerArray()
                
                # Create a list to store point cloud data
                point_cloud_data = []
                
                for cluster_id in range(num_clusters):
                    # Unpack the number of points in the current cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    offset += 4  # Move to the point data
                    
                    for point_index in range(num_points):
                        # Unpack the point data (altitude, azimuth, depth, velocity)
                        altitude, azimuth, depth, velocity = struct.unpack('!ffff', self.data_buffer[offset:offset + 16])
                        offset += 16  # Move to the next point
                        
                        # Convert polar coordinates to Cartesian coordinates
                        x = depth * np.cos(azimuth)
                        y = depth * np.sin(azimuth)
                        z = altitude
                        
                        # Add point to the point cloud data
                        point_cloud_data.append([x, y, z, velocity])
                        
                        # Create a marker for each point in the cluster
                        marker = Marker()
                        marker.header.frame_id = self.frame_id
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = f"radar_cluster_{cluster_id}"  # Namespace for the cluster
                        marker.id = point_index  # Unique ID for each point
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = x
                        marker.pose.position.y = y
                        marker.pose.position.z = z
                        marker.scale.x = 0.5
                        marker.scale.y = 0.5
                        marker.scale.z = 0.5
                        
                        # Set color based on velocity (red-to-green gradient)
                        marker.color.r = max(0.0, min(1.0, -velocity / 5.0 if velocity < 0 else 0.0))
                        marker.color.g = max(0.0, min(1.0, velocity / 5.0 if velocity > 0 else 0.0))
                        marker.color.b = 0.2
                        marker.color.a = 0.8  # Semi-transparent
                        
                        # Add the marker to the MarkerArray
                        marker_array.markers.append(marker)
                    
                    # Add delete markers for old points
                    for old_index in range(num_points, 100):  # Assuming a maximum of 100 points
                        delete_marker = Marker()
                        delete_marker.header.frame_id = self.frame_id
                        delete_marker.header.stamp = self.get_clock().now().to_msg()
                        delete_marker.ns = f"radar_cluster_{cluster_id}"
                        delete_marker.id = old_index
                        delete_marker.action = Marker.DELETE
                        marker_array.markers.append(delete_marker)
                
                # Publish the MarkerArray
                if marker_array.markers:
                    self.marker_publisher.publish(marker_array)
                    
                # Publish PointCloud2 data if we have points
                if point_cloud_data:
                    self.publish_point_cloud(point_cloud_data)
                
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
    
    def has_complete_data(self, offset, num_clusters):
        """Check if we have complete data for all clusters"""
        buffer_len = len(self.data_buffer)
        
        current_offset = offset
        for _ in range(num_clusters):
            if current_offset + 4 > buffer_len:
                return False  # Not enough data for num_points
            
            num_points = struct.unpack('!I', self.data_buffer[current_offset:current_offset + 4])[0]
            current_offset += 4
            
            # Each point is 16 bytes (4 floats)
            if current_offset + (num_points * 16) > buffer_len:
                return False  # Not enough data for all points
            
            current_offset += num_points * 16
        
        return True
    
    def publish_point_cloud(self, points_data):
        """Publish radar data as a PointCloud2 message"""
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
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
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
        self.points_publisher.publish(point_cloud_msg)
    
    def cleanup(self):
        """Clean up resources before node shutdown"""
        if self.socket:
            self.socket.close()
            self.get_logger().info('Closed radar TCP connection')

def main(args=None):
    rclpy.init(args=args)
    node = RadarTcpNode()
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