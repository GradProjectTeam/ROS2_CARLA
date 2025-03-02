import socket
import struct
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class RadarDataReceiver(Node):
    def __init__(self):
        super().__init__('radar_data_receiver')

        # TCP connection parameters
        self.tcp_ip = '127.0.0.1'  # IP address of the TCP server
        self.tcp_port = 12348  # Port number of the TCP server

        # Create a TCP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info('Connected to TCP server for radar data.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to TCP server: {e}')
            return

        self.data_buffer = bytearray()
        self.max_buffer_size = 1048576  # 1 MB buffer size limit
        self.timer = self.create_timer(0.1, self.receive_data)

        # Publisher for radar markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/radar_markers', 10)

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
                offset = 4  # Start after the number of clusters

                # Create a MarkerArray to hold the markers for this update
                marker_array = MarkerArray()

                for cluster_id in range(num_clusters):
                    if len(self.data_buffer) < offset + 4:
                        return  # Not enough data for the number of points

                    # Unpack the number of points in the current cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    offset += 4  # Move to the point data

                    if len(self.data_buffer) < offset + num_points * 16:
                        return  # Not enough data for all points in the cluster

                    for point_index in range(num_points):
                        # Unpack the point data (altitude, azimuth, depth, velocity)
                        altitude, azimuth, depth, velocity = struct.unpack('!ffff', self.data_buffer[offset:offset + 16])
                        offset += 16  # Move to the next point

                        # Convert polar coordinates to Cartesian coordinates
                        x = depth * np.cos(azimuth)  # x = depth * cos(azimuth)
                        y = depth * np.sin(azimuth)  # y = depth * sin(azimuth)
                        z = altitude  # z = altitude

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

        except struct.error as e:
            self.get_logger().error(f'Error unpacking radar data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error receiving radar data: {e}')

    def destroy_node(self):
        self.socket.close()  # Close the socket when the node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    radar_data_receiver = RadarDataReceiver()
    rclpy.spin(radar_data_receiver)
    radar_data_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()