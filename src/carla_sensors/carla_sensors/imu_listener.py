import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')

        # TCP connection parameters
        self.tcp_ip = '127.0.0.1'  # IP address of the TCP server
        self.tcp_port = 12331  # Port number of the TCP server

        # Create a TCP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info('Connected to TCP server for IMU data.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to TCP server: {e}')
            return

        # Publisher for IMU data
        self.publisher = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Create a timer to periodically receive data
        self.timer = self.create_timer(0.1, self.receive_data)

    def receive_data(self):
        try:
            # Receive data from the socket
            data = self.socket.recv(40)  # Expecting 40 bytes (10 floats)
            if len(data) == 40:  # Check if the received data is of the expected length
                self.process_imu_data(data)
            else:
                self.get_logger().warning(f'Unexpected data length: {len(data)}. Expected: 40 bytes.')
        except Exception as e:
            self.get_logger().error(f'Error receiving IMU data: {e}')

    def process_imu_data(self, data):
        try:
            # Unpack the data assuming it is in the format of 10 floats
            imu_values = struct.unpack('!10f', data)

            # Extract values
            ax, ay, az = imu_values[0:3]  # Accelerometer data
            gx, gy, gz = imu_values[3:6]  # Gyroscope data
            compass = imu_values[6]        # Compass
            roll = imu_values[7]           # Roll
            pitch = imu_values[8]          # Pitch
            yaw = imu_values[9]            # Yaw

           
            
            print(f'Received IMU Data - Acceleration: ({ax}, {ay}, {az}), '
                                   f'Gyroscope: ({gx}, {gy}, {gz}), Compass: {compass}, '
                                   f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw} \n')

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
            # Using compass as yaw
            # Convert to quaternion
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
            imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
            imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
            imu_msg.orientation.w = cr * cp * cy + sr * sp * sy

            # Publish the message
            self.publisher.publish(imu_msg)
            self.get_logger().info(f'Published IMU Data: {ax}, {ay}, {az}, {gx}, {gy}, {gz}, {compass}, {roll}, {pitch}, {yaw}')
        except struct.error as e:
            self.get_logger().error(f'Error unpacking IMU data: {e}')

    def destroy_node(self):
        self.socket.close()  # Close the socket when the node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    imu_listener = ImuListener()
    rclpy.spin(imu_listener)
    imu_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()