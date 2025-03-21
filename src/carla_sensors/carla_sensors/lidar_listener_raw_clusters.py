import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import socket
import struct
import numpy as np
from queue import Queue
import threading
from sklearn.cluster import DBSCAN

class LidarClient_raw_clusters(Node):
    def __init__(self):
        super().__init__('lidar_client_raw_clusters')
        
        # ROS2 Publishers
        self.point_publisher = self.create_publisher(PointCloud2, '/lidar/points', 30)
        self.marker_publisher = self.create_publisher(MarkerArray, '/detected_cars', 30)
        
        # TCP Server setup
        self.tcp_ip = '127.0.0.1'
        self.tcp_port = 12349
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

        # Queue for storing points
        self.point_queue = Queue(maxsize=100)  # Adjust size as needed
        self.running = True
        
        # Start a thread for receiving data
        self.receiver_thread = threading.Thread(target=self.receive_data)
        self.receiver_thread.start()

        # Create a timer for publishing data
        self.create_timer(0.1, self.publish_data)  # Adjust frequency as needed

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
        while self.running:
            # Each point is 12 bytes (3 floats * 4 bytes)
            point_data = self.receive_exact(12)
            if not point_data:
                self.get_logger().warn('No point data received')
                break
            
            # Unpack the point data (native byte order)
            try:
                x, y, z = struct.unpack('fff', point_data)
            except struct.error as e:
                self.get_logger().error('Failed to unpack point data: {}'.format(e))
                continue
            
            # Filter out nonsense values
            if abs(x) > 1000 or abs(y) > 1000 or abs(z) > 1000:
                continue
            
            # For 2D map, we'll set Z to 0
            self.point_queue.put((x, y, 0.0))  # Add point to the queue

    def publish_data(self):
        all_points = []  # Initialize a list to hold all points

        # Collect all points from the queue
        while not self.point_queue.empty():
            point = self.point_queue.get()
            # No need to reverse coordinates here anymore
            all_points.append(point)

        # Only proceed if we have points to publish
        if all_points:
            # Create and publish point cloud message
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
            msg.point_step = 12
            msg.row_step = msg.point_step * len(points)
            msg.is_dense = True
            msg.data = points.tobytes()
            
            # Publish the PointCloud2 message
            self.point_publisher.publish(msg)
            
            # Perform clustering to detect cars
            self.detect_cars(points)

    def detect_cars(self, points):
        # Convert points to a 2D array for clustering
        point_array = np.array(points)
        
        # Use DBSCAN for clustering
        clustering = DBSCAN(eps=0.5, min_samples=5).fit(point_array[:, :2])  # Only use x, y for clustering
        
        # Create marker array
        marker_array = MarkerArray()

        # First, visualize all raw points
        for i, point in enumerate(points):
            point_marker = Marker()
            point_marker.header.frame_id = 'map'
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = 'raw_points'
            point_marker.id = self.get_clock().now().nanoseconds + i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = float(point[0])
            point_marker.pose.position.y = float(point[1])
            point_marker.pose.position.z = float(point[2])
            point_marker.scale.x = 0.1
            point_marker.scale.y = 0.1
            point_marker.scale.z = 0.1
            point_marker.color.a = 1.0
            point_marker.color.b = 1.0  # Blue for raw points
            point_marker.lifetime.sec = 5
            marker_array.markers.append(point_marker)

        # Then visualize clusters with bounding boxes
        for cluster_id in set(clustering.labels_):
            if cluster_id == -1:
                continue  # Skip noise points
            
            # Get points in this cluster
            cluster_points = point_array[clustering.labels_ == cluster_id]
            if len(cluster_points) < 5:  # Skip very small clusters
                continue
            
            # Calculate the bounding box for the cluster
            min_x = float(np.min(cluster_points[:, 0]))
            max_x = float(np.max(cluster_points[:, 0]))
            min_y = float(np.min(cluster_points[:, 1]))
            max_y = float(np.max(cluster_points[:, 1]))
            
            # Create a marker for the bounding box
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'clusters'
            marker.id = int(cluster_id)
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.color.r = 1.0  # Red for clusters
            marker.color.a = 1.0
            marker.lifetime.sec = 1
            
            # Define the corners of the bounding box
            corners = [
                (min_x, min_y, 0.0),
                (max_x, min_y, 0.0),
                (max_x, max_y, 0.0),
                (min_x, max_y, 0.0),
                (min_x, min_y, 0.0)
            ]
            
            # Add points to the marker
            for corner in corners:
                point = Point()
                point.x = float(corner[0])
                point.y = float(corner[1])
                point.z = float(corner[2])
                marker.points.append(point)
            
            marker_array.markers.append(marker)

        # Publish all markers
        self.marker_publisher.publish(marker_array)

    def __del__(self):
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarClient_raw_clusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()