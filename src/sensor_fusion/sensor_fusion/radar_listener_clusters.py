#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import socket
import struct
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, TransformStamped
import std_msgs.msg
import time
import array
import geometry_msgs.msg
from tf2_ros import TransformBroadcaster
from std_msgs.msg import ColorRGBA
import math
from nav_msgs.msg import OccupancyGrid
import re
from std_msgs.msg import Float32MultiArray

# CARLA Radar Configuration
CARLA_RADAR_HORIZONTAL_FOV = 60.0  # degrees
CARLA_RADAR_VERTICAL_FOV = 20.0    # degrees
CARLA_RADAR_RANGE = 100.0          # meters
CARLA_RADAR_POINTS_PER_SECOND = 2000
CARLA_RADAR_POSITION = [1.5, 0.5, 0.0]  # x, y, z relative to vehicle


class RadarClient_clusters(Node):
    def __init__(self):
        super().__init__('radar_client_clusters')
        
        # ROS2 Publishers
        self.points_publisher = self.create_publisher(PointCloud2, '/radar/points', 10)
        self.clusters_publisher = self.create_publisher(MarkerArray, '/radar/clusters', 10)
        self.markers_publisher = self.create_publisher(MarkerArray, '/radar/markers', 10)
        self.velocity_publisher = self.create_publisher(MarkerArray, '/radar/velocity_vectors', 10)
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/radar/occupancy_grid', 10)
        self.debug_publisher = self.create_publisher(std_msgs.msg.String, '/radar/debug', 10)
        self.monitor_publisher = self.create_publisher(std_msgs.msg.String, '/radar/monitor_info', 10)
        self.distance_publisher = self.create_publisher(std_msgs.msg.String, '/radar/distance_info', 10)
        self.raw_distance_publisher = self.create_publisher(Float32MultiArray, '/radar/raw_distance_info', 10)
        
        # TCP-specific publishers (for separating TCP data visualization)
        self.tcp_points_publisher = self.create_publisher(PointCloud2, '/radar/tcp/points', 10)
        self.tcp_markers_publisher = self.create_publisher(MarkerArray, '/radar/tcp/markers', 10)
        self.tcp_velocity_publisher = self.create_publisher(MarkerArray, '/radar/tcp/velocity_vectors', 10)
        
        # CARLA-specific publishers
        self.carla_radar_points_publisher = self.create_publisher(PointCloud2, '/carla/radar/points', 10)
        self.carla_radar_markers_publisher = self.create_publisher(MarkerArray, '/carla/radar/markers', 10)
        
        # TCP Client setup
        self.tcp_ip = '127.0.0.1'
        self.tcp_port = 12348 # ros port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Optimize socket settings
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)  # 128KB receive buffer (increased)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)   # Enable keepalive
        
        # Initialize data buffer and max buffer size
        self.data_buffer = bytearray()
        self.max_buffer_size = 2097152  # 2MB buffer size limit (increased)
        
        # Statistics counters
        self.total_clusters_received = 0
        self.total_points_received = 0
        self.last_data_time = None
        self.connection_time = time.time()
        self.processing_times = []  # Track processing times for performance monitoring
        
        # Data storage for TCP points (separate from CARLA data)
        self.tcp_points_data = []
        self.max_tcp_points = 1000
        
        # Visualization parameters
        self.use_advanced_coloring = True
        self.show_velocity_vectors = True
        self.marker_lifetime = 0.2  # seconds
        self.grid_resolution = 0.2  # meters per cell
        self.grid_width = 40.0  # meters
        self.grid_height = 40.0  # meters
        
        # Data processing parameters
        self.velocity_threshold = 0.5  # m/s - minimum velocity to consider an object moving
        self.cluster_history = {}  # Track clusters over time for velocity estimation
        self.recent_measurements = []  # Track recent measurements for distance summary
        self.max_recent_measurements = 20  # Maximum number of recent measurements to store
        
        # CARLA radar specific parameters
        self.carla_radar_data = []  # Store CARLA radar detections
        self.max_carla_points = 1000  # Maximum number of CARLA radar points to store
        
        # Subscribe to CARLA radar topic if available
        self.carla_radar_subscription = self.create_subscription(
            Float32MultiArray,
            '/carla/radar/raw',
            self.carla_radar_callback,
            10
        )
        
        # Connect to server
        self.get_logger().info(f'[RADAR] Attempting to connect to {self.tcp_ip}:{self.tcp_port}...')
        try:
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info('[RADAR] Connected to server successfully')
            self.last_data_time = time.time()
        except ConnectionRefusedError:
            self.get_logger().error('[RADAR] Connection refused. Make sure the server is running.')
            raise
        except Exception as e:
            self.get_logger().error(f'[RADAR] Connection failed: {str(e)}')
            raise

        # Create timer for receiving data
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz
        
        # Create timer for stats reporting
        self.stats_timer = self.create_timer(5.0, self.report_stats)  # Report stats every 5 seconds
        
        # Create timer for publishing CARLA radar data
        self.carla_radar_timer = self.create_timer(0.1, self.publish_carla_radar_data)  # 10Hz
        
        # Create timer for publishing TCP-only radar visualization
        self.tcp_visualization_timer = self.create_timer(0.05, self.publish_tcp_radar_visualization)  # 20Hz
        
        self.get_logger().info('[RADAR] Radar processing node initialized successfully')

    def store_tcp_point(self, x, y, z, velocity):
        """Store a point received from TCP for separate visualization"""
        point_data = {
            'x': x,
            'y': y,
            'z': z,
            'velocity': velocity,
            'timestamp': self.get_clock().now().to_msg()
        }
        
        # Add to TCP points data
        self.tcp_points_data.append(point_data)
        
        # Limit the number of stored points
        if len(self.tcp_points_data) > self.max_tcp_points:
            self.tcp_points_data.pop(0)
    
    def publish_tcp_radar_visualization(self):
        """Publish visualization of TCP-only radar data"""
        if not self.tcp_points_data:
            return
            
        try:
            # Create point cloud from TCP radar data
            points = []
            for point in self.tcp_points_data:
                points.append((
                    point['x'],
                    point['y'], 
                    point['z'], 
                    point['velocity']
                ))
                
            # Create and publish point cloud
            pc2_msg = self.create_point_cloud2(points, frame_id="radar_link")
            self.tcp_points_publisher.publish(pc2_msg)
            
            # Create markers for visualization
            marker_array = MarkerArray()
            
            # Create a single marker with all points
            marker = Marker()
            marker.header.frame_id = "radar_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tcp_radar_points"
            marker.id = 0
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.3  # Point size
            marker.scale.y = 0.3
            marker.lifetime.sec = 0  # Persistent
            
            # Add points with velocity-based coloring
            for point in self.tcp_points_data:
                p = geometry_msgs.msg.Point()
                p.x = point['x']
                p.y = point['y']
                p.z = point['z']
                marker.points.append(p)
                
                # Add color for each point based on velocity
                color = ColorRGBA()
                velocity = point['velocity']
                
                if abs(velocity) < self.velocity_threshold:
                    # Stationary object - gray
                    color.r = 0.7
                    color.g = 0.7
                    color.b = 0.7
                elif velocity > 0:
                    # Approaching (red to yellow based on speed)
                    intensity = min(1.0, velocity / 10.0)
                    color.r = 1.0
                    color.g = 1.0 - intensity
                    color.b = 0.0
                else:
                    # Moving away (blue to cyan based on speed)
                    intensity = min(1.0, abs(velocity) / 10.0)
                    color.r = 0.0
                    color.g = intensity
                    color.b = 1.0
                    
                color.a = 1.0
                marker.colors.append(color)
                
            marker_array.markers.append(marker)
            
            # Add distance rings for reference
            for distance in [5.0, 10.0, 20.0, 50.0]:
                ring_marker = self.create_distance_ring_marker(distance)
                marker_array.markers.append(ring_marker)
            
            # Publish markers
            self.tcp_markers_publisher.publish(marker_array)
            
            # Create velocity vectors
            self.publish_tcp_velocity_vectors()
            
        except Exception as e:
            self.get_logger().error(f'[TCP RADAR] Error publishing radar visualization: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def publish_tcp_velocity_vectors(self):
        """Publish velocity vectors for TCP radar data"""
        if not self.tcp_points_data:
            return
            
        velocity_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # Only show vectors for points with significant velocity
        velocity_points = [p for p in self.tcp_points_data if abs(p['velocity']) > self.velocity_threshold]
        
        # Limit to max 50 vectors to avoid clutter
        if len(velocity_points) > 50:
            # Sort by absolute velocity (show fastest ones)
            velocity_points.sort(key=lambda p: abs(p['velocity']), reverse=True)
            velocity_points = velocity_points[:50]
        
        for i, point in enumerate(velocity_points):
            vel_vector = self.create_velocity_vector_marker(
                point['x'], point['y'], point['z'], 
                point['velocity'], i, current_time
            )
            velocity_array.markers.append(vel_vector)
        
        if velocity_array.markers:
            self.tcp_velocity_publisher.publish(velocity_array)

    def report_stats(self):
        """Report statistics about received data"""
        now = time.time()
        uptime = now - self.connection_time
        
        if self.last_data_time:
            time_since_last = now - self.last_data_time
            status = f"Last data received {time_since_last:.1f} seconds ago"
        else:
            status = "No data received yet"
        
        # Calculate average processing time if available
        avg_processing_time = 0
        if self.processing_times:
            avg_processing_time = sum(self.processing_times) / len(self.processing_times)
            self.processing_times = self.processing_times[-100:]  # Keep only last 100 measurements
            
        stats_msg = std_msgs.msg.String()
        stats_msg.data = (f"[RADAR STATS] Uptime: {uptime:.1f}s, "
                         f"Total clusters: {self.total_clusters_received}, "
                         f"Total points: {self.total_points_received}, "
                         f"Avg processing time: {avg_processing_time*1000:.1f}ms, "
                         f"{status}")
        
        self.monitor_publisher.publish(stats_msg)
        self.get_logger().info(stats_msg.data)

    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        data = b''
        while len(data) < size:
            packet = self.socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

    def radar_polar_to_distance(self, altitude, azimuth, depth, velocity):
        """
        Convert radar polar coordinates to distance measurements and Cartesian coordinates
        
        Args:
            altitude (float): Elevation angle in degrees
            azimuth (float): Azimuth angle in degrees
            depth (float): Radial distance in meters
            velocity (float): Radial velocity in m/s
            
        Returns:
            tuple: (x, y, z, distance, horizontal_distance, velocity)
        """
        # Convert angles to radians
        azimuth_rad = np.radians(azimuth)
        altitude_rad = np.radians(altitude)
        
        # Calculate Cartesian coordinates
        # For automotive radars, the convention is typically:
        # x: forward (depth direction)
        # y: left/right (lateral)
        # z: up/down (altitude)
        
        # Convert spherical to Cartesian coordinates
        x = depth * np.cos(altitude_rad) * np.cos(azimuth_rad)
        y = depth * np.cos(altitude_rad) * np.sin(azimuth_rad)
        z = depth * np.sin(altitude_rad)
        
        # Calculate horizontal distance (distance in xy-plane)
        horizontal_distance = np.sqrt(x*x + y*y)
        
        # The direct line-of-sight distance is the same as depth
        distance = depth
        
        # Adjust velocity sign if needed (positive = approaching, negative = moving away)
        # Note: In radar systems, velocity is typically positive when target is moving away
        # and negative when approaching, but we might want to flip it for intuitive visualization
        # adjusted_velocity = -velocity  # Uncomment if you need to flip the velocity convention
        
        return (x, y, z, distance, horizontal_distance, velocity)
        
    def process_radar_parameters(self, altitude, azimuth, depth, velocity):
        """
        Process the 4 raw radar parameters (Altitude, Azimuth, Depth, Velocity) and 
        calculate distance metrics from them.
        
        Args:
            altitude (float): Elevation angle in degrees
            azimuth (float): Azimuth angle in degrees
            depth (float): Radial distance in meters
            velocity (float): Radial velocity in m/s
            
        Returns:
            dict: Dictionary containing all calculated distance metrics and raw parameters
        """
        # Calculate Cartesian coordinates and distance metrics
        x, y, z, distance, horizontal_distance, velocity = self.radar_polar_to_distance(
            altitude, azimuth, depth, velocity)
        
        # Compute additional metrics
        angle_to_target = np.degrees(np.arctan2(y, x))  # Angle in degrees in the XY plane
        
        # Time to collision (if approaching, otherwise infinity)
        if velocity < 0:  # Object is approaching (negative velocity)
            time_to_collision = abs(distance / velocity)
        else:
            time_to_collision = float('inf')
        
        # Calculate distance metrics - will add more as needed
        distance_metrics = {
            # Raw parameters
            'altitude': altitude,             # degrees
            'azimuth': azimuth,               # degrees
            'depth': depth,                   # meters (raw radar distance)
            'velocity': velocity,             # m/s
            
            # Calculated coordinates
            'x': x,                           # meters (forward distance)
            'y': y,                           # meters (lateral distance)
            'z': z,                           # meters (height)
            
            # Derived distance metrics
            'direct_distance': distance,                  # meters (straight line)
            'horizontal_distance': horizontal_distance,   # meters (ground plane)
            'angle_to_target': angle_to_target,           # degrees
            'time_to_collision': time_to_collision,       # seconds
            
            # Additional useful metrics
            'forward_distance': x,            # meters (forward distance - same as x)
            'lateral_distance': y,            # meters (left/right distance - same as y)
            'vertical_distance': z,           # meters (up/down distance - same as z)
            'is_approaching': velocity < 0,   # Boolean (true if object is approaching)
        }
        
        # Log this processing for debugging
        debug_msg = std_msgs.msg.String()
        debug_msg.data = (
            f"[RADAR DISTANCE METRICS]\n"
            f"  Raw Parameters:\n"
            f"    Altitude: {altitude:.4f}°, Azimuth: {azimuth:.4f}°, Depth: {depth:.4f}m, Velocity: {velocity:.4f}m/s\n"
            f"  Distance Metrics:\n"
            f"    Direct Distance: {distance:.4f}m\n"
            f"    Horizontal Distance: {horizontal_distance:.4f}m\n"
            f"    Forward Distance (X): {x:.4f}m\n"
            f"    Lateral Distance (Y): {y:.4f}m\n"
            f"    Vertical Distance (Z): {z:.4f}m\n"
            f"  Motion Analysis:\n"
            f"    Angle to Target: {angle_to_target:.2f}°\n"
            f"    {'Approaching' if velocity < 0 else 'Moving Away'} at {abs(velocity):.2f}m/s\n"
            f"    {'Time to Collision: ' + f'{time_to_collision:.2f}s' if velocity < 0 else 'No collision risk'}"
        )
        self.debug_publisher.publish(debug_msg)
        
        return distance_metrics
        
    def radar_parameters_to_point_cloud(self, altitude, azimuth, depth, velocity):
        """
        Convert a single radar measurement (altitude, azimuth, depth, velocity) to a PointCloud2 message
        
        Args:
            altitude (float): Elevation angle in degrees
            azimuth (float): Azimuth angle in degrees
            depth (float): Radial distance in meters
            velocity (float): Radial velocity in m/s
            
        Returns:
            sensor_msgs.msg.PointCloud2: PointCloud2 message with a single point
        """
        # Convert to Cartesian coordinates
        x, y, z, distance, horizontal_distance, velocity = self.radar_polar_to_distance(
            altitude, azimuth, depth, velocity)
        
        # Create a PointCloud2 message with a single point
        pc2 = PointCloud2()
        pc2.header.stamp = self.get_clock().now().to_msg()
        pc2.header.frame_id = "radar_link"
        
        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='azimuth', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='altitude', offset=24, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Set the fields in the message
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 28  # 7 fields * 4 bytes
        pc2.row_step = pc2.point_step
        
        # Pack point data
        point_data = np.array([x, y, z, velocity, depth, azimuth, altitude], dtype=np.float32)
        pc2.data = point_data.tobytes()
        
        pc2.height = 1
        pc2.width = 1
        pc2.is_dense = True
        
        return pc2
    
    def multi_radar_parameters_to_point_cloud(self, params_list):
        """
        Convert multiple radar measurements to a single PointCloud2 message
        
        Args:
            params_list (list): List of (altitude, azimuth, depth, velocity) tuples
            
        Returns:
            sensor_msgs.msg.PointCloud2: PointCloud2 message with multiple points
        """
        if not params_list:
            return None
            
        # Create a PointCloud2 message
        pc2 = PointCloud2()
        pc2.header.stamp = self.get_clock().now().to_msg()
        pc2.header.frame_id = "radar_link"
        
        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='azimuth', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='altitude', offset=24, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Set the fields in the message
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 28  # 7 fields * 4 bytes
        
        # Process all points
        all_points = []
        for altitude, azimuth, depth, velocity in params_list:
            # Convert to Cartesian coordinates
            x, y, z, distance, horizontal_distance, velocity = self.radar_polar_to_distance(
                altitude, azimuth, depth, velocity)
                
            # Add to point list
            all_points.append([x, y, z, velocity, depth, azimuth, altitude])
        
        # Set data
        points_array = np.array(all_points, dtype=np.float32).flatten()
        pc2.data = points_array.tobytes()
        
        pc2.height = 1
        pc2.width = len(all_points)
        pc2.row_step = pc2.point_step * len(all_points)
        pc2.is_dense = True
        
        return pc2
        
    def process_cpp_debug(self, msg):
        """Process debug messages from C++ code containing radar measurements"""
        try:
            # Example debug format: 
            # [CPP DEBUG] Processed: Altitude: 0.0000 meters, Azimuth: 0.1717 degrees, Depth: 48.5033 meters, Velocity: 0.0000 m/s
            
            pattern = r"Altitude:\s+(-?\d+\.\d+)\s+meters,\s+Azimuth:\s+(-?\d+\.\d+)\s+degrees,\s+Depth:\s+(-?\d+\.\d+)\s+meters,\s+Velocity:\s+(-?\d+\.\d+)\s+m/s"
            match = re.search(pattern, msg.data)
            
            if match:
                altitude = float(match.group(1))
                azimuth = float(match.group(2))
                depth = float(match.group(3))
                velocity = float(match.group(4))
                
                # Process the 4 parameters directly
                distance_metrics = self.process_radar_parameters(altitude, azimuth, depth, velocity)
                
                # Store the measurement
                self.recent_measurements.append(distance_metrics)
                
                # Limit the number of stored measurements
                if len(self.recent_measurements) > self.max_recent_measurements:
                    self.recent_measurements.pop(0)
                
                # Convert to point cloud and publish
                point_cloud = self.radar_parameters_to_point_cloud(altitude, azimuth, depth, velocity)
                self.points_publisher.publish(point_cloud)
                
                # Publish distance information
                self.publish_radar_distances()
                
        except Exception as e:
            self.get_logger().error(f'[RADAR] Error processing debug message: {str(e)}')
            import traceback
            traceback.print_exc()

    def receive_data(self):
        try:
            start_time = time.time()
            
            # Read data in larger chunks
            chunk = self.socket.recv(32768)  # 32KB chunk size for better performance
            if not chunk:
                return  # No data received, just return

            # Update last data time
            self.last_data_time = time.time()
            
            # Append new data to the buffer
            self.data_buffer.extend(chunk)
            
            # Limit the buffer size
            if len(self.data_buffer) > self.max_buffer_size:
                self.get_logger().warning('[RADAR] Data buffer size exceeded, clearing buffer.')
                self.data_buffer.clear()  # Clear the buffer if it exceeds the limit

            while len(self.data_buffer) >= 4:  # Ensure we have at least 4 bytes for the number of clusters
                # Unpack the number of clusters
                num_clusters = struct.unpack('!I', self.data_buffer[:4])[0]
                
                # Sanity check for corrupt data
                if num_clusters > 1000:  # Unreasonable number of clusters
                    self.get_logger().error(f'[RADAR] Corrupt data detected: {num_clusters} clusters. Clearing buffer.')
                    self.data_buffer.clear()
                    return
                    
                self.total_clusters_received += num_clusters
                
                # Only log if we have clusters
                if num_clusters > 0:
                    debug_msg = std_msgs.msg.String()
                    debug_msg.data = f'[RADAR] Received data for {num_clusters} clusters'
                    self.debug_publisher.publish(debug_msg)
                
                offset = 4  # Start after the number of clusters

                # Create MarkerArrays to hold the markers for this update
                marker_array = MarkerArray()
                clusters_array = MarkerArray()
                velocity_array = MarkerArray()

                # Create a PointCloud2 message for all points
                all_points = []
                
                # Prepare grid data
                grid_data = self.create_empty_grid()
                
                # Pre-calculate current time to avoid repeated calls
                current_time = self.get_clock().now().to_msg()
                
                # Store distance data for reporting
                min_distance = float('inf')
                max_distance = 0
                avg_distances = []
                cluster_distances = []

                for cluster_id in range(num_clusters):
                    if len(self.data_buffer) < offset + 4:
                        return  # Not enough data for the number of points

                    # Unpack the number of points in the current cluster
                    num_points = struct.unpack('!I', self.data_buffer[offset:offset + 4])[0]
                    
                    # Sanity check for corrupt data
                    if num_points > 10000:  # Unreasonable number of points
                        self.get_logger().error(f'[RADAR] Corrupt data detected: {num_points} points in cluster. Clearing buffer.')
                        self.data_buffer.clear()
                        return
                        
                    self.total_points_received += num_points
                    offset += 4  # Move to the point data

                    if len(self.data_buffer) < offset + num_points * 16:
                        return  # Not enough data for all points in the cluster

                    # Process all points in the cluster at once
                    cluster_points = []
                    cluster_velocities = []
                    cluster_min_distance = float('inf')
                    cluster_total_distance = 0
                    
                    # Dictionary to track unique points (for duplicate detection)
                    unique_points = {}
                    duplicate_count = 0
                    
                    for point_index in range(num_points):
                        # Unpack the point data (altitude, azimuth, depth, velocity)
                        altitude, azimuth, depth, velocity = struct.unpack('!ffff', self.data_buffer[offset:offset + 16])
                        offset += 16  # Move to the next point

                        # Convert polar coordinates to distance and Cartesian coordinates
                        x, y, z, distance, horizontal_distance, velocity = self.radar_polar_to_distance(
                            altitude, azimuth, depth, velocity)
                        
                        # Store the point for TCP-only visualization
                        self.store_tcp_point(x, y, z, velocity)
                        
                        # Update distance statistics
                        min_distance = min(min_distance, distance)
                        max_distance = max(max_distance, distance)
                        avg_distances.append(distance)
                        cluster_min_distance = min(cluster_min_distance, distance)
                        cluster_total_distance += distance
                        
                        # Check for duplicates with a small tolerance
                        # Round to 2 decimal places for duplicate detection
                        point_key = (round(x, 2), round(y, 2), round(z, 2))
                        
                        if point_key in unique_points:
                            # Found a duplicate - average the velocities
                            prev_velocity = unique_points[point_key][3]
                            avg_velocity = (prev_velocity + velocity) / 2.0
                            
                            # Update the velocity in the existing point
                            idx = unique_points[point_key][4]  # Get index of the original point
                            cluster_points[idx] = (x, y, z, avg_velocity)
                            cluster_velocities[idx] = avg_velocity
                            
                            duplicate_count += 1
                            continue  # Skip adding this as a new point
                        
                        # Add point to the points list
                        cluster_points.append((x, y, z, velocity))
                        cluster_velocities.append(velocity)
                        
                        # Store in unique points dict with index
                        unique_points[point_key] = (x, y, z, velocity, len(cluster_points) - 1)
                        
                        # Add to all points for PointCloud2
                        all_points.append((x, y, z, velocity))
                        
                        # Update occupancy grid with non-duplicate points
                        self.update_grid(grid_data, x, y, velocity)
                    
                    # Log duplicate statistics if any found
                    if duplicate_count > 0:
                        self.get_logger().debug(f'[RADAR] Removed {duplicate_count} duplicate points from cluster {cluster_id}')
                    
                    # Calculate cluster centroid and average velocity
                    if cluster_points:
                        centroid_x = sum(p[0] for p in cluster_points) / len(cluster_points)
                        centroid_y = sum(p[1] for p in cluster_points) / len(cluster_points)
                        centroid_z = sum(p[2] for p in cluster_points) / len(cluster_points)
                        avg_velocity = sum(cluster_velocities) / len(cluster_velocities)
                        
                        # Calculate cluster average distance
                        avg_cluster_distance = cluster_total_distance / len(cluster_points)
                        cluster_distances.append((cluster_id, cluster_min_distance, avg_cluster_distance, len(cluster_points)))
                        
                        # Store cluster data for tracking
                        cluster_key = f"cluster_{cluster_id}"
                        self.cluster_history[cluster_key] = {
                            'centroid': (centroid_x, centroid_y, centroid_z),
                            'velocity': avg_velocity,
                            'timestamp': time.time(),
                            'points': len(cluster_points),
                            'min_distance': cluster_min_distance,
                            'avg_distance': avg_cluster_distance
                        }
                    
                    # Create visualization markers
                    if cluster_points:
                        # Create cluster marker with advanced coloring
                        cluster_marker = self.create_cluster_marker(cluster_points, cluster_id, num_clusters, current_time)
                        marker_array.markers.append(cluster_marker)
                        
                        # Create velocity-colored marker
                        vel_marker = self.create_velocity_marker(cluster_points, cluster_id, current_time)
                        clusters_array.markers.append(vel_marker)
                        
                        # Create velocity vector marker if enabled
                        if self.show_velocity_vectors and abs(avg_velocity) > self.velocity_threshold:
                            vel_vector = self.create_velocity_vector_marker(
                                centroid_x, centroid_y, centroid_z, avg_velocity, cluster_id, current_time)
                            velocity_array.markers.append(vel_vector)

                # Create and publish PointCloud2 message if we have points
                if all_points:
                    # Remove duplicates from all_points before creating PointCloud2
                    all_points = self.remove_duplicates_from_points(all_points)
                    pc2_msg = self.create_point_cloud2(all_points)
                    self.points_publisher.publish(pc2_msg)
                
                # Publish the MarkerArrays
                self.markers_publisher.publish(marker_array)
                self.clusters_publisher.publish(clusters_array)
                
                # Publish velocity vectors if we have any
                if velocity_array.markers:
                    self.velocity_publisher.publish(velocity_array)
                
                # Publish occupancy grid
                grid_msg = self.create_occupancy_grid_msg(grid_data, current_time)
                self.grid_publisher.publish(grid_msg)
                
                # Publish distance information
                if avg_distances:
                    avg_distance = sum(avg_distances) / len(avg_distances)
                    distance_msg = std_msgs.msg.String()
                    
                    # Sort clusters by minimum distance
                    sorted_clusters = sorted(cluster_distances, key=lambda x: x[1])
                    
                    # Build the message
                    distance_info = (f"[RADAR DISTANCE] Min: {min_distance:.2f}m, Max: {max_distance:.2f}m, Avg: {avg_distance:.2f}m\n")
                    distance_info += "Clusters by distance:\n"
                    
                    for i, (cluster_id, min_dist, avg_dist, num_points) in enumerate(sorted_clusters[:5]):  # Show top 5 closest
                        distance_info += f"  #{i+1}: Cluster {cluster_id} - Min: {min_dist:.2f}m, Avg: {avg_dist:.2f}m, Points: {num_points}\n"
                    
                    distance_msg.data = distance_info
                    self.distance_publisher.publish(distance_msg)

                # Remove processed data from the buffer
                self.data_buffer = self.data_buffer[offset:]
                
                # Record processing time
                end_time = time.time()
                self.processing_times.append(end_time - start_time)
                
        except Exception as e:
            self.get_logger().error(f'[RADAR] Error receiving data: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def remove_duplicates_from_points(self, points_list):
        """Remove duplicate points from a list of points based on x,y,z coordinates"""
        if not points_list:
            return []
            
        # Convert to numpy array for faster processing
        points_array = np.array(points_list)
        
        # Round coordinates to reduce floating point comparison issues
        rounded_xyz = np.round(points_array[:, :3], 2)
        
        # Find unique rows based on x,y,z (first 3 columns)
        # We use a structured array to find unique rows
        unique_indices = np.unique(rounded_xyz.view([('', rounded_xyz.dtype)] * 3), 
                                 return_index=True)[1]
        
        # Sort indices to maintain original order
        unique_indices.sort()
        
        # Return the unique points
        return [points_list[i] for i in unique_indices]
    
    def create_empty_grid(self):
        """Create an empty occupancy grid"""
        width = int(self.grid_width / self.grid_resolution)
        height = int(self.grid_height / self.grid_resolution)
        return np.full((width, height), -1, dtype=np.int8)  # -1 = unknown
    
    def update_grid(self, grid, x, y, velocity):
        """Update occupancy grid with radar point"""
        # Convert world coordinates to grid coordinates
        grid_x = int((x + self.grid_width/2) / self.grid_resolution)
        grid_y = int((y + self.grid_height/2) / self.grid_resolution)
        
        # Check if within grid bounds
        if 0 <= grid_x < grid.shape[0] and 0 <= grid_y < grid.shape[1]:
            # Assign occupancy value based on velocity
            # Higher velocity = higher cost (more dangerous)
            cost = 50  # Base cost for static objects
            
            if abs(velocity) > self.velocity_threshold:
                # Scale velocity to cost (50-100)
                # Higher velocity = higher cost
                vel_cost = min(100, 50 + int(abs(velocity) * 5))
                cost = vel_cost
            
            # Only update if the new cost is higher than existing cost
            # This ensures that moving objects take precedence over static ones
            if grid[grid_x, grid_y] < cost:
                grid[grid_x, grid_y] = cost
    
    def create_occupancy_grid_msg(self, grid_data, timestamp):
        """Create occupancy grid message from numpy array"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = timestamp
        grid_msg.header.frame_id = "radar_link"
        
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = grid_data.shape[0]
        grid_msg.info.height = grid_data.shape[1]
        
        # Set origin to bottom-left corner (ROS convention)
        grid_msg.info.origin.position.x = -self.grid_width / 2
        grid_msg.info.origin.position.y = -self.grid_height / 2
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Flatten grid data to 1D array (row-major order)
        grid_msg.data = grid_data.flatten().tolist()
        
        return grid_msg
    
    def create_cluster_marker(self, cluster_points, cluster_id, num_clusters, timestamp):
        """Create marker for cluster visualization with advanced coloring"""
        marker = Marker()
        marker.header.frame_id = "radar_link"
        marker.header.stamp = timestamp
        marker.ns = f"radar_cluster_{cluster_id}"
        marker.id = cluster_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # Point size
        marker.scale.y = 0.3
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
        
        # Set color based on cluster ID with improved scheme
        if self.use_advanced_coloring:
            # Use HSV color space for better visual distinction
            hue = cluster_id / max(1, num_clusters)  # 0.0 to 1.0
            # Convert HSV to RGB (simplified conversion)
            h = hue * 6.0
            i = int(h)
            f = h - i
            if i % 2 == 0:
                f = 1 - f
            
            # Generate RGB from hue
            if i == 0:
                marker.color.r, marker.color.g, marker.color.b = 1.0, f, 0.0
            elif i == 1:
                marker.color.r, marker.color.g, marker.color.b = f, 1.0, 0.0
            elif i == 2:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, f
            elif i == 3:
                marker.color.r, marker.color.g, marker.color.b = 0.0, f, 1.0
            elif i == 4:
                marker.color.r, marker.color.g, marker.color.b = f, 0.0, 1.0
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, f
        else:
            # Simple coloring scheme
            marker.color.r = 0.2 + 0.8 * (cluster_id / max(1, num_clusters))
            marker.color.g = 0.2 + 0.8 * (1 - cluster_id / max(1, num_clusters))
            marker.color.b = 0.5
            
        marker.color.a = 1.0
        
        # Add all points to the marker
        for x, y, z, _ in cluster_points:
            p = geometry_msgs.msg.Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)
        
        return marker
    
    def create_velocity_marker(self, cluster_points, cluster_id, timestamp):
        """Create velocity-colored marker for the cluster"""
        marker = Marker()
        marker.header.frame_id = "radar_link"
        marker.header.stamp = timestamp
        marker.ns = "radar_velocity"
        marker.id = cluster_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
        
        # Add all points with velocity-based coloring
        for x, y, z, velocity in cluster_points:
            p = geometry_msgs.msg.Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)
            
            # Add color for each point based on velocity
            color = ColorRGBA()
            
            # Improved velocity coloring:
            # - Green to red gradient for positive velocity (approaching)
            # - Blue to cyan gradient for negative velocity (moving away)
            # - Gray for stationary
            
            if abs(velocity) < self.velocity_threshold:
                # Stationary object
                color.r = 0.7
                color.g = 0.7
                color.b = 0.7
            elif velocity > 0:
                # Approaching (red to yellow based on speed)
                intensity = min(1.0, velocity / 10.0)  # Scale velocity to 0-1
                color.r = 1.0
                color.g = 1.0 - intensity
                color.b = 0.0
            else:
                # Moving away (blue to cyan based on speed)
                intensity = min(1.0, abs(velocity) / 10.0)  # Scale velocity to 0-1
                color.r = 0.0
                color.g = intensity
                color.b = 1.0
                
            color.a = 1.0
            marker.colors.append(color)
        
        return marker
    
    def create_velocity_vector_marker(self, x, y, z, velocity, cluster_id, timestamp):
        """Create an arrow marker showing velocity direction and magnitude"""
        marker = Marker()
        marker.header.frame_id = "radar_link"
        marker.header.stamp = timestamp
        marker.ns = "velocity_vectors"
        marker.id = cluster_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set arrow start point
        marker.points.append(Point(x=x, y=y, z=z))
        
        # Calculate arrow end point based on velocity
        # Assuming positive velocity is toward the sensor (in x-direction)
        # Scale velocity for better visualization
        scale = 1.0  # Adjust scale factor as needed
        end_x = x + (velocity * scale)
        
        marker.points.append(Point(x=end_x, y=y, z=z))
        
        # Set arrow properties
        marker.scale.x = 0.1  # Shaft diameter
        marker.scale.y = 0.2  # Head diameter
        marker.scale.z = 0.3  # Head length
        
        # Set color based on velocity
        if velocity > 0:
            # Approaching - red
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            # Moving away - blue
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            
        marker.color.a = 1.0
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
        
        return marker

    def create_point_cloud2(self, points, frame_id="radar_link"):
        """
        Create a PointCloud2 message from a list of points
        
        Args:
            points: List of tuples (x, y, z, velocity, ...)
            frame_id: Frame ID for the message
        """
        pc2 = PointCloud2()
        pc2.header.stamp = self.get_clock().now().to_msg()
        pc2.header.frame_id = frame_id
        
        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Add optional fields if provided
        field_count = 4
        point_step = 16  # 4 fields * 4 bytes
        
        # Check if we have depth/azimuth/altitude
        if points and len(points[0]) > 4:
            fields.append(PointField(name='depth', offset=point_step, datatype=PointField.FLOAT32, count=1))
            point_step += 4
            field_count += 1
            
            if len(points[0]) > 5:
                fields.append(PointField(name='azimuth', offset=point_step, datatype=PointField.FLOAT32, count=1))
                point_step += 4
                field_count += 1
                
                if len(points[0]) > 6:
                    fields.append(PointField(name='altitude', offset=point_step, datatype=PointField.FLOAT32, count=1))
                    point_step += 4
                    field_count += 1
        
        # Set the fields in the message
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = point_step
        pc2.row_step = pc2.point_step * len(points)
        
        # Use numpy for more efficient packing
        if points:
            points_array = np.array(points, dtype=np.float32).flatten()
            pc2.data = points_array.tobytes()
        else:
            pc2.data = b''
        
        pc2.height = 1
        pc2.width = len(points)
        pc2.is_dense = True
        
        return pc2

    def publish_radar_distances(self):
        """Publish the radar distance measurements"""
        if not self.recent_measurements:
            return
            
        # Create a Float32MultiArray message for the raw distances
        distances_msg = Float32MultiArray()
        
        # Format: [depth, azimuth, velocity, x, y, z, ...]
        data = []
        for measurement in self.recent_measurements[-10:]:  # Use most recent 10 measurements
            data.extend([
                measurement['depth'],                # Radial distance
                measurement['azimuth'],              # Azimuth angle
                measurement['velocity'],             # Radial velocity
                measurement['x'],                    # X position (forward)
                measurement['y'],                    # Y position (lateral)
                measurement['z'],                    # Z position (vertical)
                measurement['horizontal_distance'],  # Distance in XY plane
                measurement['angle_to_target']       # Angle to target in degrees
            ])
            
        distances_msg.data = data
        self.raw_distance_publisher.publish(distances_msg)
        
        # Also publish a formatted text message with detailed metrics
        distance_msg = std_msgs.msg.String()
        distance_info = "[RADAR DISTANCE SUMMARY]\n"
        
        # Sort measurements by direct distance (closest first)
        sorted_measurements = sorted(self.recent_measurements[-5:], key=lambda x: x['depth'])
        
        for i, measurement in enumerate(sorted_measurements):
            distance_info += (
                f"Target #{i+1}:\n"
                f"  Distance: {measurement['depth']:.2f}m at {measurement['azimuth']:.2f}°\n"
                f"  Position (x,y,z): ({measurement['x']:.2f}, {measurement['y']:.2f}, {measurement['z']:.2f})m\n"
                f"  Velocity: {measurement['velocity']:.2f}m/s "
                f"({'approaching' if measurement['velocity'] < 0 else 'receding'})\n"
            )
            
            # Add time to collision if approaching
            if measurement['velocity'] < 0:
                distance_info += f"  Time to collision: {measurement['time_to_collision']:.2f}s\n"
            
            distance_info += "\n"
            
        distance_msg.data = distance_info
        self.distance_publisher.publish(distance_msg)

    def carla_radar_callback(self, msg):
        """
        Handle raw CARLA radar data from Float32MultiArray format
        Format expected: [altitude, azimuth, depth, velocity, ...]
        """
        try:
            # Check if we have valid data (must be multiple of 4)
            if len(msg.data) % 4 != 0:
                self.get_logger().warn(f'[CARLA RADAR] Invalid data length: {len(msg.data)}')
                return
                
            # Process the radar points (each point is 4 consecutive values)
            for i in range(0, len(msg.data), 4):
                if i+3 < len(msg.data):
                    altitude = msg.data[i]
                    azimuth = msg.data[i+1]
                    depth = msg.data[i+2]
                    velocity = msg.data[i+3]
                    
                    # Convert to Cartesian coordinates and check validity
                    x, y, z, distance, horizontal_distance, velocity, is_valid = self.carla_radar_to_distance(
                        altitude, azimuth, depth, velocity)
                    
                    if is_valid:
                        # Store the point for later processing
                        point_data = {
                            'altitude': altitude,
                            'azimuth': azimuth,
                            'depth': depth,
                            'velocity': velocity,
                            'x': x,
                            'y': y,
                            'z': z,
                            'horizontal_distance': horizontal_distance,
                            'timestamp': self.get_clock().now().to_msg(),
                        }
                        
                        # Add to CARLA radar data buffer
                        self.carla_radar_data.append(point_data)
                        
                        # Limit buffer size
                        if len(self.carla_radar_data) > self.max_carla_points:
                            self.carla_radar_data.pop(0)
                            
            # Log reception of data
            self.get_logger().debug(f'[CARLA RADAR] Received {len(msg.data)//4} radar points')
            
        except Exception as e:
            self.get_logger().error(f'[CARLA RADAR] Error processing radar data: {str(e)}')
            import traceback
            traceback.print_exc()

    def publish_carla_radar_data(self):
        """Publish the CARLA radar data as point cloud and markers"""
        if not self.carla_radar_data:
            return
            
        try:
            # Create point cloud from CARLA radar data
            points = []
            for point in self.carla_radar_data:
                points.append((
                    point['x'], 
                    point['y'], 
                    point['z'], 
                    point['velocity'],
                    point['depth'],
                    point['azimuth'],
                    point['altitude']
                ))
                
            # Create and publish point cloud
            pc2_msg = self.create_point_cloud2(points, frame_id="radar_link")
            self.carla_radar_points_publisher.publish(pc2_msg)
            
            # Create markers for visualization
            marker_array = MarkerArray()
            
            # Create a single marker with all points
            marker = Marker()
            marker.header.frame_id = "radar_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "carla_radar_points"
            marker.id = 0
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.3  # Point size
            marker.scale.y = 0.3
            marker.lifetime.sec = 0  # Persistent
            
            # Add points with velocity-based coloring
            for point in self.carla_radar_data:
                p = geometry_msgs.msg.Point()
                p.x = point['x']
                p.y = point['y']
                p.z = point['z']
                marker.points.append(p)
                
                # Add color for each point based on velocity
                color = ColorRGBA()
                velocity = point['velocity']
                
                if abs(velocity) < self.velocity_threshold:
                    # Stationary object - gray
                    color.r = 0.7
                    color.g = 0.7
                    color.b = 0.7
                elif velocity > 0:
                    # Approaching (red to yellow based on speed)
                    intensity = min(1.0, velocity / 10.0)
                    color.r = 1.0
                    color.g = 1.0 - intensity
                    color.b = 0.0
                else:
                    # Moving away (blue to cyan based on speed)
                    intensity = min(1.0, abs(velocity) / 10.0)
                    color.r = 0.0
                    color.g = intensity
                    color.b = 1.0
                    
                color.a = 1.0
                marker.colors.append(color)
                
            marker_array.markers.append(marker)
            
            # Add distance rings
            for distance in [5.0, 10.0, 20.0, 50.0, 100.0]:
                if distance <= CARLA_RADAR_RANGE:
                    ring_marker = self.create_distance_ring_marker(distance)
                    marker_array.markers.append(ring_marker)
            
            # Publish markers
            self.carla_radar_markers_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'[CARLA RADAR] Error publishing radar data: {str(e)}')
            import traceback
            traceback.print_exc()

    def create_distance_ring_marker(self, radius):
        """Create a ring marker at specified radius for distance visualization"""
        marker = Marker()
        marker.header.frame_id = "radar_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "distance_rings"
        marker.id = int(radius)  # Use radius as ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8
        marker.color.a = 0.5
        marker.lifetime.sec = 0  # Persistent
        
        # Create ring points
        segments = 50  # Number of segments in the ring
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            p = geometry_msgs.msg.Point()
            p.x = radius * math.cos(angle)
            p.y = radius * math.sin(angle)
            p.z = 0.0
            marker.points.append(p)
        
        return marker

    def carla_radar_to_distance(self, altitude, azimuth, depth, velocity):
        """
        Convert CARLA radar polar coordinates to distance measurements and Cartesian coordinates.
        Uses CARLA's specific coordinate system (x-forward, y-right, z-up)
        
        Args:
            altitude (float): Elevation angle in degrees
            azimuth (float): Azimuth angle in degrees
            depth (float): Radial distance in meters
            velocity (float): Radial velocity in m/s
            
        Returns:
            tuple: (x, y, z, distance, horizontal_distance, velocity, is_valid)
        """
        # Convert angles to radians
        azimuth_rad = np.radians(azimuth)
        altitude_rad = np.radians(altitude)
        
        # Calculate Cartesian coordinates
        # For CARLA radar, the convention is:
        # x: forward, y: right, z: up
        x = depth * np.cos(altitude_rad) * np.cos(azimuth_rad)
        y = depth * np.cos(altitude_rad) * np.sin(azimuth_rad)
        z = depth * np.sin(altitude_rad)
        
        # Calculate horizontal distance (distance in xy-plane)
        horizontal_distance = np.sqrt(x*x + y*y)
        
        # Check if within the radar's FOV limits
        in_horizontal_fov = abs(azimuth) <= CARLA_RADAR_HORIZONTAL_FOV / 2
        in_vertical_fov = abs(altitude) <= CARLA_RADAR_VERTICAL_FOV / 2
        in_range = depth <= CARLA_RADAR_RANGE
        
        is_valid = in_horizontal_fov and in_vertical_fov and in_range
        
        return (x, y, z, depth, horizontal_distance, velocity, is_valid)

    def __del__(self):
        self.get_logger().info('[RADAR] Closing socket connection')
        try:
            self.socket.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RadarClient_clusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()