import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class LidarListener(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.tcp_ip = '127.0.0.1'  # CARLA IP
        self.tcp_port = 12347  # CARLA LiDAR TCP port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.tcp_ip, self.tcp_port))
        self.get_logger().info('Connected to CARLA LiDAR TCP stream.')

        self.timer = self.create_timer(0.1, self.receive_data)

    def receive_data(self):
        try:
            data = self.socket.recv(4096)
            if data:
                lidar_msg = PointCloud2()
                # Process the LiDAR data and fill in the PointCloud2 message fields
                self.publisher.publish(lidar_msg)
                self.get_logger().info('LiDAR data received and published.')
        except Exception as e:
            self.get_logger().error(f'Error receiving LiDAR data: {e}')

def main(args=None):
    rclpy.init(args=args)
    lidar_listener = LidarListener()
    rclpy.spin(lidar_listener)
    lidar_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()