#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import ColorRGBA
import socket
import struct
import numpy as np
import colorsys
import time
import math


class LidarClient_clusters_2(Node):
    def __init__(self):
        super().__init__('lidar_client_clusters_2')
        
        # ROS2 Publishers
        self.publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/lidar/markers', 10)
        self.hull_publisher = self.create_publisher(MarkerArray, '/lidar/hulls', 10)
        self.cube_publisher = self.create_publisher(MarkerArray, '/lidar/cubes', 10)  # New publisher for cubes
        self.vehicle_publisher = self.create_publisher(Marker, '/lidar/vehicle', 10)  # Publisher for vehicle visualization
        
        # Parameter declaration
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12350)
        self.declare_parameter('point_size', 2.0)
        self.declare_parameter('center_size', 3.0)
        self.declare_parameter('use_convex_hull', True)
        self.declare_parameter('use_point_markers', True)
        self.declare_parameter('use_cluster_stats', True)
        self.declare_parameter('verbose_logging', False)
        self.declare_parameter('cube_alpha', 0.3)  # Transparency for cubes
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.point_size = self.get_parameter('point_size').value
        self.center_size = self.get_parameter('center_size').value
        self.use_convex_hull = self.get_parameter('use_convex_hull').value
        self.use_point_markers = self.get_parameter('use_point_markers').value
        self.use_cluster_stats = self.get_parameter('use_cluster_stats').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.cube_alpha = self.get_parameter('cube_alpha').value
        
        # Add parameters for filtering points that hit the car itself
        self.declare_parameter('filter_vehicle_points', True)   # Whether to filter out points hitting the car
        self.declare_parameter('vehicle_length', 5.0)           # Length of vehicle in meters (x direction)
        self.declare_parameter('vehicle_width', 2.5)            # Width of vehicle in meters (y direction)
        self.declare_parameter('vehicle_height', 2.2)           # Height of vehicle in meters (z direction)
        self.declare_parameter('vehicle_x_offset', 0.0)         # Offset of vehicle center in x direction
        self.declare_parameter('vehicle_y_offset', 0.0)         # Offset of vehicle center in y direction
        self.declare_parameter('vehicle_z_offset', -1.0)        # Offset of vehicle z-coordinate
        self.declare_parameter('vehicle_safety_margin', 0.5)    # Extra margin around vehicle to filter
        self.declare_parameter('vehicle_visualization', True)   # Whether to visualize the vehicle filter zone

        # LiDAR specific configuration parameters
        self.declare_parameter('lidar_upper_fov', 15.0)         # Upper field of view in degrees
        self.declare_parameter('lidar_lower_fov', -25.0)        # Lower field of view in degrees
        self.declare_parameter('lidar_pitch_angle', 5.0)        # Upward pitch of LiDAR in degrees
        self.declare_parameter('min_point_distance', 3.0)       # Minimum distance filter (meters)
        self.declare_parameter('max_negative_z', -0.5)          # Filter points below this Z value
        
        # Get vehicle filter parameters
        self.filter_vehicle_points = self.get_parameter('filter_vehicle_points').value
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.vehicle_width = self.get_parameter('vehicle_width').value
        self.vehicle_height = self.get_parameter('vehicle_height').value
        self.vehicle_x_offset = self.get_parameter('vehicle_x_offset').value
        self.vehicle_y_offset = self.get_parameter('vehicle_y_offset').value
        self.vehicle_z_offset = self.get_parameter('vehicle_z_offset').value
        self.vehicle_safety_margin = self.get_parameter('vehicle_safety_margin').value
        self.vehicle_visualization = self.get_parameter('vehicle_visualization').value
        
        # Get LiDAR specific parameters
        self.lidar_upper_fov = self.get_parameter('lidar_upper_fov').value
        self.lidar_lower_fov = self.get_parameter('lidar_lower_fov').value
        self.lidar_pitch_angle = self.get_parameter('lidar_pitch_angle').value
        self.min_point_distance = self.get_parameter('min_point_distance').value
        self.max_negative_z = self.get_parameter('max_negative_z').value
        
        # Calculate vehicle filter boundaries with safety margin
        self.vehicle_x_min = self.vehicle_x_offset - (self.vehicle_length / 2) - self.vehicle_safety_margin
        self.vehicle_x_max = self.vehicle_x_offset + (self.vehicle_length / 2) + self.vehicle_safety_margin
        self.vehicle_y_min = self.vehicle_y_offset - (self.vehicle_width / 2) - self.vehicle_safety_margin
        self.vehicle_y_max = self.vehicle_y_offset + (self.vehicle_width / 2) + self.vehicle_safety_margin
        self.vehicle_z_min = self.vehicle_z_offset - self.vehicle_safety_margin  # Adjusted to use z_offset
        self.vehicle_z_max = self.vehicle_z_offset + self.vehicle_height + self.vehicle_safety_margin
        
        # Log vehicle filter settings
        if self.filter_vehicle_points:
            self.get_logger().info(f"Vehicle point filtering enabled")
            self.get_logger().info(f"Vehicle filter zone: X [{self.vehicle_x_min:.2f}, {self.vehicle_x_max:.2f}], " 
                                   f"Y [{self.vehicle_y_min:.2f}, {self.vehicle_y_max:.2f}], "
                                   f"Z [{self.vehicle_z_min:.2f}, {self.vehicle_z_max:.2f}]")
            self.get_logger().info(f"LiDAR settings: Upper FOV: {self.lidar_upper_fov}°, Lower FOV: {self.lidar_lower_fov}°, "
                                   f"Pitch: {self.lidar_pitch_angle}°")
            self.get_logger().info(f"Point filtering: Min distance: {self.min_point_distance}m, Max negative Z: {self.max_negative_z}m")
        
        # Connection statistics
        self.last_receive_time = time.time()
        self.points_received = 0
        self.clusters_received = 0
        self.points_filtered = 0  # Track how many points are filtered out
        self.distance_filtered = 0  # Track points filtered by distance
        self.z_filtered = 0  # Track points filtered by z-value
        self.vehicle_filtered = 0  # Track points filtered by vehicle boundary
        
        # TCP Client setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Connect to server
        self.get_logger().info(f'Attempting to connect to {self.tcp_ip}:{self.tcp_port}...')
        try:
            self.socket.connect((self.tcp_ip, self.tcp_port))
            self.get_logger().info('Connected to server successfully')
        except ConnectionRefusedError:
            self.get_logger().error('Connection refused. Make sure the server is running.')
            raise
        except Exception as e:
            self.get_logger().error(f'Connection failed: {str(e)}')
            raise

        # Create timers
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz
        self.stats_timer = self.create_timer(1.0, self.report_stats)  # 1Hz stats

    def report_stats(self):
        """Report statistics about data reception"""
        now = time.time()
        elapsed = now - self.last_receive_time
        if elapsed > 0:
            points_per_sec = self.points_received / elapsed
            clusters_per_sec = self.clusters_received / elapsed
            
            stats_msg = f'Stats: {points_per_sec:.1f} points/s, {clusters_per_sec:.1f} clusters/s'
            if self.filter_vehicle_points:
                total_filtered = self.distance_filtered + self.z_filtered + self.vehicle_filtered
                filtered_per_sec = total_filtered / elapsed
                filter_percentage = 0.0
                if self.points_received + total_filtered > 0:
                    filter_percentage = (total_filtered / (self.points_received + total_filtered)) * 100
                stats_msg += f', filtered {filtered_per_sec:.1f} points/s ({filter_percentage:.1f}%)'
                
                # Add detailed breakdown of filtered points
                if total_filtered > 0:
                    dist_percent = (self.distance_filtered / total_filtered) * 100
                    z_percent = (self.z_filtered / total_filtered) * 100
                    veh_percent = (self.vehicle_filtered / total_filtered) * 100
                    stats_msg += f' [dist: {dist_percent:.1f}%, z: {z_percent:.1f}%, veh: {veh_percent:.1f}%]'
                
            self.get_logger().info(stats_msg)
            self.points_received = 0
            self.clusters_received = 0
            self.distance_filtered = 0
            self.z_filtered = 0
            self.vehicle_filtered = 0
            self.last_receive_time = now

    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        data = b''
        while len(data) < size:
            packet = self.socket.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data

    def generate_colors(self, n):
        """Generate visually distinct colors using HSV color space"""
        colors = []
        for i in range(n):
            # Use golden ratio to create well-distributed hues
            h = (i * 0.618033988749895) % 1.0
            s = 0.8 + 0.2 * (i % 2)  # Alternate between 0.8 and 1.0 saturation
            v = 0.9  # Keep value high for visibility
            
            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            colors.append((r, g, b))
        return colors

    def calculate_convex_hull_2d(self, points):
        """Calculate the 2D convex hull for a set of points (Graham scan algorithm)"""
        if len(points) < 3:
            return points
            
        # Find the lowest point
        lowest = min(range(len(points)), key=lambda i: (points[i][1], points[i][0]))
        
        # Sort points by polar angle with respect to the lowest point
        def polar_angle(p):
            return math.atan2(p[1] - points[lowest][1], p[0] - points[lowest][0])
            
        sorted_points = sorted(points, key=lambda p: (polar_angle(p), p[0], p[1]))
        
        # Build the hull
        hull = [sorted_points[0], sorted_points[1]]
        
        for i in range(2, len(sorted_points)):
            while len(hull) > 1:
                # Cross product to determine turn direction
                x1, y1 = hull[-1][0] - hull[-2][0], hull[-1][1] - hull[-2][1]
                x2, y2 = sorted_points[i][0] - hull[-1][0], sorted_points[i][1] - hull[-1][1]
                cross_product = x1 * y2 - y1 * x2
                
                # If not a left turn, remove the last point
                if cross_product <= 0:
                    hull.pop()
                else:
                    break
                    
            hull.append(sorted_points[i])
            
        return hull

    def receive_data(self):
        try:
            # First, publish the vehicle visualization if filtering is enabled
            if self.filter_vehicle_points and self.vehicle_visualization:
                self.publish_vehicle_visualization()
                
            # Receive number of clusters
            data = self.receive_exact(4)
            if not data:
                return
            num_clusters = struct.unpack('!I', data)[0]
            
            if self.verbose_logging:
                self.get_logger().info(f'Receiving {num_clusters} clusters')
            
            self.clusters_received += num_clusters
            
            # Create marker arrays
            point_marker_array = MarkerArray()
            hull_marker_array = MarkerArray()
            cube_marker_array = MarkerArray()  # New marker array for cubes
            
            # Generate cluster colors - more visually distinct
            cluster_colors = self.generate_colors(max(8, num_clusters))

            all_points = []
            total_points = 0
            
            # Clean up previous markers when needed
            if num_clusters == 0:
                # Clear all markers by publishing empty arrays
                self.marker_publisher.publish(MarkerArray())
                self.hull_publisher.publish(MarkerArray())
                self.cube_publisher.publish(MarkerArray())  # Clear cubes
                return
            
            # Process each cluster
            for cluster_id in range(num_clusters):
                # Get number of points in this cluster
                size_data = self.receive_exact(4)
                if not size_data:
                    return
                cluster_size = struct.unpack('!I', size_data)[0]
                
                if self.verbose_logging:
                    self.get_logger().info(f'Cluster {cluster_id}: expecting {cluster_size} points')
                
                # Get color for this cluster
                color = cluster_colors[cluster_id % len(cluster_colors)]
                
                cluster_points = []
                # Process each point in the cluster
                for point_id in range(cluster_size):
                    point_data = self.receive_exact(12)  # 3 * float32
                    if not point_data:
                        return
                    x, y, z = struct.unpack('!fff', point_data)
                    
                    # Verify the data with more tolerant checks for valid LIDAR data
                    if not (isinstance(x, float) and isinstance(y, float) and isinstance(z, float)):
                        self.get_logger().error(f'Invalid point data type: {type(x)}, {type(y)}, {type(z)}')
                        continue
                    
                    # Check for NaN and Inf
                    if (math.isnan(x) or math.isnan(y) or math.isnan(z) or
                        math.isinf(x) or math.isinf(y) or math.isinf(z)):
                        continue
                        
                    # Very generous bounds checking (±10km) - just to catch truly absurd values
                    if (abs(x) > 10000 or abs(y) > 10000 or abs(z) > 10000):
                        if point_id % 1000 == 0:  # Only log occasionally
                            self.get_logger().warn(f'Filtered extreme value: {x}, {y}, {z}')
                        continue
                    
                    # Apply minimum distance filter
                    if self.filter_vehicle_points:
                        # Calculate distance from LiDAR origin (0,0,0)
                        distance = math.sqrt(x**2 + y**2 + z**2)
                        if distance < self.min_point_distance:
                            self.distance_filtered += 1
                            continue
                        
                        # Filter points with severe negative Z values (hitting car roof/hood)
                        if z < self.max_negative_z:
                            self.z_filtered += 1
                            continue
                        
                        # Filter out points that are within the vehicle boundary
                        if (self.vehicle_x_min <= x <= self.vehicle_x_max and
                            self.vehicle_y_min <= y <= self.vehicle_y_max and
                            self.vehicle_z_min <= z <= self.vehicle_z_max):
                            self.vehicle_filtered += 1
                            continue
                    
                    # Add the point
                    cluster_points.append([x, y, z])
                    all_points.append([x, y, z])
                    total_points += 1

                    # Create individual point markers if enabled
                    if self.use_point_markers:
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = f"lidar_cluster_{cluster_id}"
                        marker.id = total_points  # Unique ID for each point
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        
                        marker.pose.position.x = x
                        marker.pose.position.y = y
                        marker.pose.position.z = z
                        
                        # Make points more visible
                        marker.scale.x = self.point_size
                        marker.scale.y = self.point_size
                        marker.scale.z = self.point_size
                        
                        marker.color.r = color[0]
                        marker.color.g = color[1]
                        marker.color.b = color[2]
                        marker.color.a = 0.9  # More opaque for better visibility
                        
                        point_marker_array.markers.append(marker)

                # Process the cluster if it has points
                if cluster_points:
                    self.points_received += len(cluster_points)
                    
                    # Calculate cluster center and dimensions
                    center_x = sum(p[0] for p in cluster_points) / len(cluster_points)
                    center_y = sum(p[1] for p in cluster_points) / len(cluster_points)
                    center_z = sum(p[2] for p in cluster_points) / len(cluster_points)
                    
                    # Calculate dimensions and statistics
                    min_x = min(p[0] for p in cluster_points)
                    max_x = max(p[0] for p in cluster_points)
                    min_y = min(p[1] for p in cluster_points)
                    max_y = max(p[1] for p in cluster_points)
                    min_z = min(p[2] for p in cluster_points)
                    max_z = max(p[2] for p in cluster_points)
                    
                    width = max_x - min_x
                    length = max_y - min_y
                    height = max_z - min_z
                    volume = width * length * height
                    
                    # Use POINTS type for efficient visualization of all cluster points
                    points_marker = Marker()
                    points_marker.header.frame_id = "map"
                    points_marker.header.stamp = self.get_clock().now().to_msg()
                    points_marker.ns = f"cluster_points_{cluster_id}"
                    points_marker.id = 0
                    points_marker.type = Marker.POINTS
                    points_marker.action = Marker.ADD
                    
                    # Larger points for better visibility
                    points_marker.scale.x = self.point_size * 0.7
                    points_marker.scale.y = self.point_size * 0.7
                    
                    # Add all points to the POINTS marker
                    for point in cluster_points:
                        ros_point = Point()
                        ros_point.x = point[0]
                        ros_point.y = point[1]
                        ros_point.z = point[2]
                        points_marker.points.append(ros_point)
                        
                        # Add matching color for each point
                        color_rgba = ColorRGBA()
                        color_rgba.r = color[0]
                        color_rgba.g = color[1]
                        color_rgba.b = color[2]
                        color_rgba.a = 1.0  # Fully opaque points
                        points_marker.colors.append(color_rgba)
                    
                    point_marker_array.markers.append(points_marker)
                    
                    # Add cluster center marker (larger sphere)
                    center_marker = Marker()
                    center_marker.header.frame_id = "map"
                    center_marker.header.stamp = self.get_clock().now().to_msg()
                    center_marker.ns = f"cluster_center_{cluster_id}"
                    center_marker.id = 0
                    center_marker.type = Marker.SPHERE
                    center_marker.action = Marker.ADD
                    center_marker.pose.position.x = center_x
                    center_marker.pose.position.y = center_y
                    center_marker.pose.position.z = center_z
                    center_marker.scale.x = self.center_size
                    center_marker.scale.y = self.center_size
                    center_marker.scale.z = self.center_size
                    center_marker.color.r = color[0]
                    center_marker.color.g = color[1]
                    center_marker.color.b = color[2]
                    center_marker.color.a = 0.9
                    point_marker_array.markers.append(center_marker)
                    
                    # Add solid cube for the cluster 
                    cube_marker = Marker()
                    cube_marker.header.frame_id = "map"
                    cube_marker.header.stamp = self.get_clock().now().to_msg()
                    cube_marker.ns = f"cluster_cube_{cluster_id}"
                    cube_marker.id = 0
                    cube_marker.type = Marker.CUBE
                    cube_marker.action = Marker.ADD
                    
                    # Set cube position to cluster center
                    cube_marker.pose.position.x = center_x
                    cube_marker.pose.position.y = center_y
                    cube_marker.pose.position.z = center_z
                    
                    # Set orientation (identity quaternion)
                    cube_marker.pose.orientation.w = 1.0
                    
                    # Set cube dimensions
                    cube_marker.scale.x = width
                    cube_marker.scale.y = length
                    cube_marker.scale.z = height
                    
                    # Set cube color (semi-transparent to see points inside)
                    cube_marker.color.r = color[0]
                    cube_marker.color.g = color[1]
                    cube_marker.color.b = color[2]
                    cube_marker.color.a = self.cube_alpha  # Adjustable transparency
                    
                    cube_marker_array.markers.append(cube_marker)
                    
                    # Add bounding box as a LINE_LIST
                    box_marker = Marker()
                    box_marker.header.frame_id = "map"
                    box_marker.header.stamp = self.get_clock().now().to_msg()
                    box_marker.ns = f"cluster_box_{cluster_id}"
                    box_marker.id = 0
                    box_marker.type = Marker.LINE_LIST
                    box_marker.action = Marker.ADD
                    box_marker.scale.x = 0.1  # Line width
                    box_marker.color.r = color[0]
                    box_marker.color.g = color[1]
                    box_marker.color.b = color[2]
                    box_marker.color.a = 1.0
                    
                    # Define the 8 corners of the box
                    corners = [
                        (min_x, min_y, min_z), (max_x, min_y, min_z),
                        (max_x, max_y, min_z), (min_x, max_y, min_z),
                        (min_x, min_y, max_z), (max_x, min_y, max_z),
                        (max_x, max_y, max_z), (min_x, max_y, max_z)
                    ]
                    
                    # Define the 12 lines of the box (each line connects 2 points)
                    lines = [
                        (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
                        (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
                        (0, 4), (1, 5), (2, 6), (3, 7)   # Connecting lines
                    ]
                    
                    # Add the points and lines to the marker
                    for start_idx, end_idx in lines:
                        start = corners[start_idx]
                        end = corners[end_idx]
                        
                        p1 = Point()
                        p1.x, p1.y, p1.z = start
                        box_marker.points.append(p1)
                        
                        p2 = Point()
                        p2.x, p2.y, p2.z = end
                        box_marker.points.append(p2)
                    
                    hull_marker_array.markers.append(box_marker)
                    
                    # Calculate and display 2D convex hull if enabled
                    if self.use_convex_hull and len(cluster_points) >= 3:
                        # Extract 2D points for hull calculation (using x and y only)
                        points_2d = [[p[0], p[1]] for p in cluster_points]
                        hull_points = self.calculate_convex_hull_2d(points_2d)
                        
                        if len(hull_points) >= 3:
                            hull_marker = Marker()
                            hull_marker.header.frame_id = "map"
                            hull_marker.header.stamp = self.get_clock().now().to_msg()
                            hull_marker.ns = f"cluster_hull_{cluster_id}"
                            hull_marker.id = 0
                            hull_marker.type = Marker.LINE_STRIP
                            hull_marker.action = Marker.ADD
                            hull_marker.scale.x = 0.2  # Thicker line for hull
                            hull_marker.color.r = color[0]
                            hull_marker.color.g = color[1]
                            hull_marker.color.b = color[2]
                            hull_marker.color.a = 1.0
                            
                            # Calculate the average z-value for the hull
                            avg_z = center_z - 0.2  # Slightly below center for visibility
                            
                            # Add hull points to the marker
                            for point in hull_points:
                                p = Point()
                                p.x = point[0]
                                p.y = point[1]
                                p.z = avg_z
                                hull_marker.points.append(p)
                            
                            # Close the loop
                            if hull_points:
                                p = Point()
                                p.x = hull_points[0][0]
                                p.y = hull_points[0][1]
                                p.z = avg_z
                                hull_marker.points.append(p)
                            
                            hull_marker_array.markers.append(hull_marker)
                    
                    # Add text label with comprehensive information
                    text_marker = Marker()
                    text_marker.header.frame_id = "map"
                    text_marker.header.stamp = self.get_clock().now().to_msg()
                    text_marker.ns = f"cluster_label_{cluster_id}"
                    text_marker.id = 0
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    
                    # Create more informative label
                    if self.use_cluster_stats:
                        text_marker.text = f"Cluster {cluster_id}\n" \
                                        f"Points: {len(cluster_points)}\n" \
                                        f"Size: {width:.1f}x{length:.1f}x{height:.1f}m\n" \
                                        f"Vol: {volume:.1f}m³"
                    else:
                        text_marker.text = f"Cluster {cluster_id}\n{len(cluster_points)} points"
                    
                    text_marker.scale.z = 0.5  # Larger text
                    text_marker.pose.position.x = center_x
                    text_marker.pose.position.y = center_y
                    text_marker.pose.position.z = center_z + height/2 + 0.5  # Position above the object
                    text_marker.color.r = color[0]
                    text_marker.color.g = color[1]
                    text_marker.color.b = color[2]
                    text_marker.color.a = 1.0
                    point_marker_array.markers.append(text_marker)

            # Print summary (only if verbose logging is enabled)
            if self.verbose_logging:
                self.get_logger().info(f'Total points received: {total_points}')

            # Publish markers
            self.marker_publisher.publish(point_marker_array)
            self.hull_publisher.publish(hull_marker_array)
            self.cube_publisher.publish(cube_marker_array)  # Publish the cube markers
            
            # Publish point cloud
            if all_points:
                msg = PointCloud2()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                
                points_array = np.array(all_points, dtype=np.float32)
                msg.height = 1
                msg.width = len(all_points)
                msg.is_bigendian = False
                msg.point_step = 12
                msg.row_step = msg.point_step * len(all_points)
                msg.is_dense = True
                msg.data = points_array.tobytes()
                
                self.publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    def publish_vehicle_visualization(self):
        """Publish a visualization of the vehicle filter zone"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle_filter_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position to vehicle center, with z offset
        marker.pose.position.x = self.vehicle_x_offset
        marker.pose.position.y = self.vehicle_y_offset
        marker.pose.position.z = self.vehicle_z_offset + (self.vehicle_height / 2)  # Center in Z with offset
        
        # Set orientation (identity quaternion)
        marker.pose.orientation.w = 1.0
        
        # Set dimensions - using the filter boundaries
        marker.scale.x = self.vehicle_length + (2 * self.vehicle_safety_margin)
        marker.scale.y = self.vehicle_width + (2 * self.vehicle_safety_margin)
        marker.scale.z = self.vehicle_height + (2 * self.vehicle_safety_margin)
        
        # Set color - red, semi-transparent
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3  # Semi-transparent
        
        # Set lifetime (2 seconds is enough since we publish at 1Hz stats)
        marker.lifetime.sec = 2
        
        # Publish
        self.vehicle_publisher.publish(marker)

    def __del__(self):
        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = LidarClient_clusters_2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()