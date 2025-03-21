#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import socket
import struct
import threading
import math

class LidarMapCreator(Node):
    def __init__(self):
        super().__init__('lidar_map_creator')
        
        # TCP Connection settings
        self.TCP_IP = '0.0.0.0'
        self.TCP_PORT = 12345
        self.BUFFER_SIZE = 1024
        
        # Map parameters
        self.map_resolution = 0.1  # meters per pixel
        self.map_width = 2000     # Increased size for larger area
        self.map_height = 2000    # Increased size for larger area
        self.map_origin_x = -100.0  # Adjusted for larger area
        self.map_origin_y = -100.0  # Adjusted for larger area
        
        # Vehicle pose tracking
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        
        # Create empty map with unknown cells (-1)
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Ray tracing parameters
        self.max_range = 50.0  # maximum sensor range in meters
        
        # Publishers and Subscribers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vehicle_pose',
            self.pose_callback,
            10)
        
        # Start TCP server
        self.tcp_thread = threading.Thread(target=self.tcp_server)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()
        
        self.create_timer(0.1, self.publish_map)  # Increased update rate
        
        self.get_logger().info('Lidar Map Creator Node Started')

    def pose_callback(self, msg):
        """Update vehicle pose"""
        self.vehicle_x = msg.pose.position.x
        self.vehicle_y = msg.pose.position.y
        
        # Extract yaw from quaternion
        siny_cosp = 2 * (msg.pose.orientation.w * msg.pose.orientation.z + 
                        msg.pose.orientation.x * msg.pose.orientation.y)
        cosy_cosp = 1 - 2 * (msg.pose.orientation.y * msg.pose.orientation.y + 
                            msg.pose.orientation.z * msg.pose.orientation.z)
        self.vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)

    def transform_point(self, x, y):
        """Transform point from vehicle frame to world frame"""
        # Rotate
        cos_yaw = math.cos(self.vehicle_yaw)
        sin_yaw = math.sin(self.vehicle_yaw)
        x_world = x * cos_yaw - y * sin_yaw
        y_world = x * sin_yaw + y * cos_yaw
        
        # Translate
        x_world += self.vehicle_x
        y_world += self.vehicle_y
        
        return x_world, y_world

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
                
        points.append((x, y))
        return points

    def update_map_with_ray(self, start_x, start_y, end_x, end_y):
        """Update map using ray tracing"""
        ray_points = self.bresenham_line(start_x, start_y, end_x, end_y)
        
        # Mark all points along the ray as free space (0)
        for x, y in ray_points[:-1]:  # All points except the last one
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.map_data[y, x] != 100:  # Don't overwrite occupied cells
                    self.map_data[y, x] = 0
        
        # Mark the endpoint as occupied (100)
        if ray_points:
            x, y = ray_points[-1]
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                self.map_data[y, x] = 100

    def process_lidar_point(self, x, y, z):
        """Process a single LiDAR point"""
        # Transform to world coordinates
        x_world, y_world = self.transform_point(x, y)
        
        # Convert vehicle position to map coordinates
        vehicle_map_x, vehicle_map_y = self.world_to_map(self.vehicle_x, self.vehicle_y)
        
        # Convert point to map coordinates
        point_map_x, point_map_y = self.world_to_map(x_world, y_world)
        
        # Calculate distance
        distance = math.sqrt(x*x + y*y)
        
        # Only process points within max range
        if distance <= self.max_range:
            self.update_map_with_ray(vehicle_map_x, vehicle_map_y, point_map_x, point_map_y)

    def tcp_server(self):
        """TCP server to receive LiDAR data"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((self.TCP_IP, self.TCP_PORT))
        sock.listen(1)
        
        self.get_logger().info(f'TCP Server listening on port {self.TCP_PORT}')
        
        while True:
            conn, addr = sock.accept()
            self.get_logger().info(f'Connection from {addr}')
            
            try:
                while True:
                    data = conn.recv(self.BUFFER_SIZE)
                    if not data:
                        break
                    
                    point_size = struct.calcsize('fff')
                    points_count = len(data) // point_size
                    
                    for i in range(points_count):
                        point_data = data[i*point_size:(i+1)*point_size]
                        x, y, z = struct.unpack('fff', point_data)
                        self.process_lidar_point(x, y, z)
                        
            except Exception as e:
                self.get_logger().error(f'Error processing data: {str(e)}')
            finally:
                conn.close()

    def world_to_map(self, x, y):
        """Convert world coordinates to map cell indices"""
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        
        # Flatten map data for message
        map_msg.data = self.map_data.flatten().tolist()
        
        self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    lidar_map_creator = LidarMapCreator()
    rclpy.spin(lidar_map_creator)
    lidar_map_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 