import socket
import struct
import time
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.node import Node
import rclpy


class LidarClient(Node):
    def __init__(self):
        super().__init__('lidar_client')
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 30)
        self.tcp_ip = '127.0.0.1'  # Server IP
        self.tcp_port = 12345       # Server port (should match LISTEN_PORT in C++)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.tcp_ip, self.tcp_port))  # Bind the socket to the IP and port
        self.socket.listen(1)  # Listen for incoming connections

        self.get_logger().info('Waiting for a connection...')  # Log waiting for a connection
        self.connection, client_address = self.socket.accept()  # Accept a connection from a client
        self.get_logger().info(f'Connected to client at {client_address}')  # Log the client connection

        # Start receiving data
        self.timer = self.create_timer(0.01, self.receive_data)

    def receive_data(self):
        while True:
            try:
                data = self.connection.recv(12)  # Expecting 12 bytes for each point (3 floats)
                if not data:
                    self.get_logger().error('Connection closed by the server.')
                    self.reconnect()  # Attempt to reconnect
                    continue  # Skip to the next iteration to wait for data

                if len(data) < 12:
                    self.get_logger().error('Incomplete point data received.')
                    continue

                # Unpack the point data
                point = struct.unpack('fff', data)  # Each point is 3 floats
                self.get_logger().info(f'Received point: x={point[0]}, y={point[1]}, z={point[2]}')

                # Create a PointCloud2 message
                point_msg = PointCloud2()
                point_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
                point_msg.header.frame_id = 'lidar_frame'  # Set the frame ID
                point_msg.height = 1
                point_msg.width = 1
                point_msg.is_dense = True
                point_msg.is_bigendian = False
                point_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                point_msg.data = struct.pack('fff', *point)  # Pack the point data
                point_msg.point_step = 12  # Size of a point (3 floats)
                point_msg.row_step = point_msg.point_step
                point_msg.width = 1
                point_msg.height = 1

                self.publisher.publish(point_msg)
                self.get_logger().info('Point published to /lidar/points')

            except (socket.error, struct.error) as e:
                self.get_logger().error(f'Error receiving point data: {e}')
                self.reconnect()  # Attempt to reconnect on error

    def reconnect(self):
        self.get_logger().info('Attempting to reconnect...')
        self.socket.close()  # Close the current socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Create a new socket
        self.socket.bind((self.tcp_ip, self.tcp_port))  # Rebind to the IP and port
        self.socket.listen(1)  # Listen for incoming connections
        self.get_logger().info('Waiting for a new connection...')
        self.connection, client_address = self.socket.accept()  # Accept a new connection
        self.get_logger().info(f'Connected to client at {client_address}')  # Log the new client connection

    def shutdown(self):
        if self.socket:
            self.socket.close()
            self.get_logger().info('Socket closed.')

def main(args=None):
    rclpy.init(args=args)
    lidar_client = LidarClient()
    try:
        rclpy.spin(lidar_client)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_client.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()