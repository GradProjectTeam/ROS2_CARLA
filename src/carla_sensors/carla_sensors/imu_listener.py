import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)  # Corrected topic name
        self.tcp_ip = '0.0.0.0'  # Listen on all interfaces
        self.tcp_port = 12346  # CARLA IMU TCP port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Set socket options
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Bind and listen for incoming connections
        self.socket.bind((self.tcp_ip, self.tcp_port))
        self.socket.listen(1)
        self.get_logger().info(f'Listening for connections on {self.tcp_ip}:{self.tcp_port}')
        
        # Accept a connection
        self.client_socket, addr = self.socket.accept()
        self.get_logger().info(f'Connection accepted from {addr}')

        self.timer = self.create_timer(0.1, self.receive_data)

    def receive_data(self):
        while rclpy.ok():  # Keep receiving data while the node is active
            try:
                data = self.client_socket.recv(28)  # Expecting 28 bytes (7 floats)
                if len(data) == 28:  # Check if the received data is of the expected length
                    self.process_imu_data(data)
                else:
                    self.get_logger().warning(f'Unexpected data length: {len(data)}. Expected: 28 bytes.')
                    continue  # Continue to the next iteration to receive more data
            except Exception as e:
                self.get_logger().error(f'Error receiving IMU data: {e}')
                break  # Exit loop on error

    def process_imu_data(self, data):
        try:
            # Unpack the data assuming it is in the format of 7 floats
            ax, ay, az, gx, gy, gz, compass = struct.unpack('!7f', data)
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Linear acceleration
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            
            # Angular velocity
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Convert compass to quaternion (simple example)
            roll = 0  # You might want to derive this from your data
            pitch = 0
            yaw = compass  # Using compass as yaw

            # Convert to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            imu_msg.orientation = Quaternion(
                x = sr * cp * cy - cr * sp * sy,
                y = cr * sp * cy + sr * cp * sy,
                z = cr * cp * sy - sr * sp * cy,
                w = cr * cp * cy + sr * sp * sy
            )
            
            # Publish the message
            self.publisher.publish(imu_msg)
            self.get_logger().info(f'Published IMU Data: {ax}, {ay}, {az}, {gx}, {gy}, {gz}, {compass}')
        except struct.error as e:
            self.get_logger().error(f'Error unpacking IMU data: {e}')

def main(args=None):
    rclpy.init(args=args)
    imu_listener = ImuListener()
    rclpy.spin(imu_listener)
    imu_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()