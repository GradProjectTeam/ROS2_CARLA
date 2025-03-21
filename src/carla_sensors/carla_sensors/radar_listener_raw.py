#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np


class RadarClient_raw(Node):
    def __init__(self):
        super().__init__('radar_client_raw')
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(PointCloud2, '/radar/points', 100)
        
        # TCP Server setup
        self.tcp_ip = '127.0.0.1'
        self.tcp_port = 12350
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = None
        
        # Start server
        self.get_logger().info('Starting server on {}:{}'.format(self.tcp_ip, self.tcp_port))
        try:
            self.server_socket.bind((self.tcp_ip, self.tcp_port))
            self.server_socket.listen(1)
            self.get_logger().info('Waiting for client connection...')
            self.client_socket, addr = self.server_socket.accept()
            self.get_logger().info('Client connected from {}'.format(addr))
        except Exception as e:
            self.get_logger().error('Server setup failed: {}'.format(str(e)))
            raise

        # Create timer for receiving data
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz

    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        data = b''
        while len(data) < size:
            packet = self.client_socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

    def receive_data(self):
        try:
            all_points = []
            
            while True:
                # Each radar point is 16 bytes (4 floats * 4 bytes: altitude, azimuth, depth, velocity)
                point_data = self.receive_exact(16)  # Changed to 16 bytes
                if not point_data:
                    self.get_logger().warn('No point data received')
                    break
                
                # Unpack the radar point data (native byte order)
                try:
                    altitude, azimuth, depth, velocity = struct.unpack('ffff', point_data)  # Changed to unpack 4 floats
                    
                    # Convert spherical to cartesian coordinates
                    x = depth * np.cos(altitude) * np.cos(azimuth)
                    y = depth * np.cos(altitude) * np.sin(azimuth)
                    z = depth * np.sin(altitude)
                    
                except struct.error as e:
                    self.get_logger().error('Failed to unpack point data: {}'.format(e))
                    continue
                
                # Print raw values before filtering
                self.get_logger().info('Raw Radar Point - Altitude: {:.3f}, Azimuth: {:.3f}, Depth: {:.3f}, Velocity: {:.3f}'.format(
                    altitude, azimuth, depth, velocity))
                
                # Filter out nonsense values
                if depth > 100:  # Adjusted range for radar
                    self.get_logger().debug('Point filtered (range): {:.3f}'.format(depth))
                    continue
                
                # Store point with velocity
                all_points.append([x, y, z, velocity])
                
                # Print accepted point
                self.get_logger().info('Accepted Radar Point - X: {:.3f}, Y: {:.3f}, Z: {:.3f}, Vel: {:.3f}'.format(x, y, z, velocity))
                
                # Check socket for more data
                try:
                    if not self.client_socket.recv(1, socket.MSG_PEEK):
                        self.get_logger().info('No more data in socket')
                        break
                except Exception as e:
                    self.get_logger().warn('Socket peek failed: {}'.format(e))
                    break
            
           
                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                # Set up the fields for radar data
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                
                # Convert points to numpy array
                points = np.array(all_points, dtype=np.float32)
                
                # Set message parameters
                msg.height = 1
                msg.width = len(points)
                msg.is_bigendian = False
                msg.point_step = 16  # Changed to 16 bytes per point
                msg.row_step = msg.point_step * len(points)
                msg.is_dense = True
                msg.data = points.tobytes()
                
                # Publish the message
                self.publisher.publish(msg)
                self.get_logger().info('Published {} points in radar data'.format(len(points)))
            else:
                self.get_logger().warn('No valid points to publish.')
        except Exception as e:
            self.get_logger().error('Error in receive_data: {}'.format(str(e)))
            import traceback
            self.get_logger().error(traceback.format_exc())

    def __del__(self):
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = RadarClient_raw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()