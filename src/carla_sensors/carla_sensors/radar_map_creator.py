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

class RadarMapCreator(Node):
    def __init__(self):
        super().__init__('radar_map_creator')
        
        # TCP Connection settings
        self.TCP_IP = '0.0.0.0'
        self.TCP_PORT = 12345
        self.BUFFER_SIZE = 1024
        
        # Map parameters - adjusted for better coverage
        self.map_resolution = 0.1  # 10cm per pixel for finer detail
        self.map_width = 3000     # 300m width (3000 * 0.1)
        self.map_height = 3000    # 300m height
        self.map_origin_x = -150.0  # Center the map
        self.map_origin_y = -150.0
        
        # Create empty map with free space (0)
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Vehicle position tracking
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_z = 0.0
        self.vehicle_yaw = 0.0
        
        # Radar mounting parameters (matching updated configuration)
        self.radar_offset_x = 2.5  # 2.5m forward from vehicle center
        self.radar_offset_z = 1.0  # 1m up from vehicle center
        self.radar_pitch = math.radians(-10)  # -10 degrees pitch
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/radar_map', 10)
        
        # Start TCP servers
        self.tcp_thread = threading.Thread(target=self.tcp_server)
        self.tcp_thread.daemon = True
        self.tcp_thread.start()
        
        # Publish map frequently
        self.create_timer(0.1, self.publish_map)
        
        self.get_logger().info('2D Radar Map Creator Started')

    def transform_point(self, altitude, azimuth, depth):
        """Transform radar point from spherical to 2D cartesian coordinates"""
        # Altitude and azimuth are already in radians
        adjusted_altitude = altitude + self.radar_pitch
        
        # Calculate coordinates (radar's forward is along X-axis)
        x = depth * math.cos(adjusted_altitude) * math.cos(azimuth)
        y = depth * math.cos(adjusted_altitude) * math.sin(azimuth)
        z = depth * math.sin(adjusted_altitude)
        
        # Apply radar mounting offset
        x += self.radar_offset_x
        z += self.radar_offset_z
        
        # Transform to world coordinates
        # CARLA's coordinate system: x=forward, y=right
        # Map coordinate system: x=right, y=forward
        # So we swap x and y, and negate the new y to match map orientation
        x_world = y + self.vehicle_x  # right is positive x in map
        y_world = -x + self.vehicle_y  # forward is positive y in map
        
        return x_world, y_world

    def world_to_map(self, x, y):
        """Convert world coordinates to map cell indices"""
        # Adjust coordinate system to match map orientation
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def add_point_to_map(self, x, y, velocity):
        """Add a single point to the map with improved intensity scaling"""
        map_x, map_y = self.world_to_map(x, y)
        
        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            # Enhanced velocity-based intensity mapping
            # Negative velocity means approaching objects
            velocity_intensity = min(abs(velocity) * 10, 100)  # Scale velocity to intensity
            
            if velocity < 0:  # Approaching
                base_intensity = min(90 + velocity_intensity/10, 100)  # Higher intensity
            else:  # Moving away
                base_intensity = max(60 - velocity_intensity/10, 30)   # Lower intensity
            
            # Create a gaussian-like intensity distribution
            kernel_size = 5
            kernel_center = kernel_size // 2
            
            for dx in range(-kernel_center, kernel_center + 1):
                for dy in range(-kernel_center, kernel_center + 1):
                    nx, ny = map_x + dx, map_y + dy
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        # Calculate distance from center for intensity falloff
                        distance = math.sqrt(dx*dx + dy*dy)
                        intensity = base_intensity * math.exp(-distance)
                        
                        # Update the cell if new intensity is higher
                        self.map_data[ny, nx] = max(
                            self.map_data[ny, nx],
                            int(intensity)
                        )

    def tcp_server(self):
        """TCP server to receive both position and radar data"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((self.TCP_IP, self.TCP_PORT))
        sock.listen(1)
        
        self.get_logger().info(f'TCP Server listening on port {self.TCP_PORT}')
        
        while True:
            try:
                conn, addr = sock.accept()
                self.get_logger().info(f'Connection from {addr}')
                
                while True:
                    try:
                        # Read data type identifier (1 byte)
                        type_data = conn.recv(1)
                        if not type_data:
                            break
                            
                        data_type = struct.unpack('!B', type_data)[0]
                        
                        if data_type == 1:  # Position data
                            # Receive position (3 floats)
                            pos_data = conn.recv(struct.calcsize('!3f'))
                            x, y, z = struct.unpack('!3f', pos_data)
                            self.vehicle_x = x
                            self.vehicle_y = y
                            self.vehicle_z = z
                            self.get_logger().info(f'Updated position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
                            self.clear_old_points()  # Clear old points when position updates
                            
                        elif data_type == 2:  # Radar data
                            # Receive number of points
                            num_points_data = conn.recv(4)
                            num_points = struct.unpack('!I', num_points_data)[0]
                            
                            # Receive radar points
                            point_size = struct.calcsize('!4f')
                            data = conn.recv(point_size * num_points)
                            
                            # Process each point
                            for i in range(num_points):
                                point_data = data[i*point_size:(i+1)*point_size]
                                altitude, azimuth, depth, velocity = struct.unpack('!4f', point_data)
                                
                                if not (np.isnan(depth) or np.isnan(azimuth)):
                                    x_world, y_world = self.transform_point(altitude, azimuth, depth)
                                    self.add_point_to_map(x_world, y_world, velocity)
                            
                    except Exception as e:
                        self.get_logger().error(f'Error processing data: {str(e)}')
                        continue
                        
            except Exception as e:
                self.get_logger().error(f'Server error: {str(e)}')
            finally:
                if 'conn' in locals():
                    conn.close()

    def publish_map(self):
        """Publish the occupancy grid map"""
        try:
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'  # Make sure RViz2 has this frame
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.info.resolution = self.map_resolution
            msg.info.width = self.map_width
            msg.info.height = self.map_height
            msg.info.origin.position.x = self.map_origin_x
            msg.info.origin.position.y = self.map_origin_y
            msg.info.origin.position.z = 0.0
            
            # Set the orientation (identity quaternion)
            msg.info.origin.orientation.w = 1.0
            msg.info.origin.orientation.x = 0.0
            msg.info.origin.orientation.y = 0.0
            msg.info.origin.orientation.z = 0.0
            
            # Convert map data to list
            msg.data = self.map_data.flatten().tolist()
            
            self.map_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing map: {str(e)}')

    def clear_old_points(self):
        """Clear old points from the map periodically"""
        # Create a small area around vehicle that's always cleared
        vehicle_map_x, vehicle_map_y = self.world_to_map(self.vehicle_x, self.vehicle_y)
        clear_radius = int(5.0 / self.map_resolution)  # 5 meters radius
        
        for dx in range(-clear_radius, clear_radius + 1):
            for dy in range(-clear_radius, clear_radius + 1):
                map_x = vehicle_map_x + dx
                map_y = vehicle_map_y + dy
                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    self.map_data[map_y, map_x] = 0

def main(args=None):
    rclpy.init(args=args)
    radar_map_creator = RadarMapCreator()
    rclpy.spin(radar_map_creator)
    radar_map_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 