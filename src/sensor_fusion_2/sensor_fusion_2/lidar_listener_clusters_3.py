#!/usr/bin/env python3
"""
LiDAR Point Cloud Clustering and Visualization Node
================================================

Authors: Shishtawy & Hendy
Project: TechZ Autonomous Driving System

OVERVIEW:
This node processes clustered LiDAR point cloud data received via TCP/IP connection
and provides comprehensive visualization in RViz. It handles real-time point cloud
data, organizes points into clusters, and generates multiple visualization markers
for better understanding of the environment.

KEY FEATURES:
- Real-time LiDAR data processing via TCP
- Point cloud clustering visualization
- Convex hull generation for clusters
- Bounding box visualization
- Cluster statistics and metrics
- Color-coded cluster representation
- RViz integration with multiple marker types

VISUALIZATION COMPONENTS:
1. Individual point markers
2. Cluster centers
3. Convex hulls
4. Bounding boxes
5. Text labels with cluster statistics
6. Solid cubes for volume representation

This node is essential for environmental perception in the TechZ autonomous
driving system, providing detailed visualization of detected objects and obstacles.
"""

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
        
        # Parameter declaration with existence checks
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.has_parameter('tcp_ip'):
            self.declare_parameter('tcp_ip', '127.0.0.1')
        if not self.has_parameter('tcp_port'):
            self.declare_parameter('tcp_port', 12350)
        if not self.has_parameter('point_size'):
            self.declare_parameter('point_size', 2.0)
        if not self.has_parameter('center_size'):
            self.declare_parameter('center_size', 3.0)
        if not self.has_parameter('use_convex_hull'):
            self.declare_parameter('use_convex_hull', True)
        if not self.has_parameter('use_point_markers'):
            self.declare_parameter('use_point_markers', True)
        if not self.has_parameter('use_cluster_stats'):
            self.declare_parameter('use_cluster_stats', True)
        if not self.has_parameter('verbose_logging'):
            self.declare_parameter('verbose_logging', False)
        if not self.has_parameter('cube_alpha'):
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
        
        # Add parameters for filtering points that hit the car itself - DISABLED but kept for compatibility
        if not self.has_parameter('filter_vehicle_points'):
            self.declare_parameter('filter_vehicle_points', False)   # DISABLED: Whether to filter out points hitting the car
        if not self.has_parameter('vehicle_length'):
            self.declare_parameter('vehicle_length', 5.0)           
        if not self.has_parameter('vehicle_width'):
            self.declare_parameter('vehicle_width', 2.5)            
        if not self.has_parameter('vehicle_height'):
            self.declare_parameter('vehicle_height', 2.2)            
        if not self.has_parameter('vehicle_x_offset'):
            self.declare_parameter('vehicle_x_offset', 0.0)            
        if not self.has_parameter('vehicle_y_offset'):
            self.declare_parameter('vehicle_y_offset', 0.0)            
        if not self.has_parameter('vehicle_z_offset'):
            self.declare_parameter('vehicle_z_offset', -1.0)            
        if not self.has_parameter('vehicle_safety_margin'):
            self.declare_parameter('vehicle_safety_margin', 0.5)            
        if not self.has_parameter('vehicle_visualization'):
            self.declare_parameter('vehicle_visualization', False)   # DISABLED: Whether to visualize the vehicle filter zone

        # LiDAR specific configuration parameters - DISABLED but kept for compatibility
        if not self.has_parameter('lidar_upper_fov'):
            self.declare_parameter('lidar_upper_fov', 15.0)            
        if not self.has_parameter('lidar_lower_fov'):
            self.declare_parameter('lidar_lower_fov', -25.0)            
        if not self.has_parameter('lidar_pitch_angle'):
            self.declare_parameter('lidar_pitch_angle', 5.0)            
        if not self.has_parameter('min_point_distance'):
            self.declare_parameter('min_point_distance', 0.0)       # Set to 0 to disable distance filtering
        if not self.has_parameter('max_negative_z'):
            self.declare_parameter('max_negative_z', -100.0)        # Set to extreme value to disable Z filtering
        
        # Get vehicle filter parameters - DISABLED
        self.filter_vehicle_points = False  # Explicitly disable filtering
        self.vehicle_visualization = False  # Explicitly disable visualization
        
        # Connection statistics
        self.last_receive_time = time.time()
        self.points_received = 0
        self.clusters_received = 0
        
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
        now = time.time()                                                   # Get current time
        elapsed = now - self.last_receive_time                             # Calculate time since last report
        if elapsed > 0:                                                    # Avoid division by zero
            points_per_sec = self.points_received / elapsed                # Calculate points per second
            clusters_per_sec = self.clusters_received / elapsed            # Calculate clusters per second
            
            stats_msg = f'Stats: {points_per_sec:.1f} points/s, {clusters_per_sec:.1f} clusters/s'  # Format stats message
            self.get_logger().info(stats_msg)                             # Log statistics
            
            self.points_received = 0                                       # Reset points counter
            self.clusters_received = 0                                     # Reset clusters counter
            self.last_receive_time = now                                   # Update timing reference

    def receive_exact(self, size):
        """Helper function to receive exact number of bytes"""
        data = b''                                                         # Initialize empty byte string
        while len(data) < size:                                           # Keep reading until we have enough data
            packet = self.socket.recv(size - len(data))                    # Read remaining bytes
            if not packet:                                                # Check for connection closure
                return None
            data += packet                                                # Append received data
        return data                                                       # Return complete data chunk

    def generate_colors(self, n):
        """Generate visually distinct colors using HSV color space"""
        colors = []                                                       # Initialize color list
        for i in range(n):                                               # Generate n colors
            h = (i * 0.618033988749895) % 1.0                            # Golden ratio for hue distribution
            s = 0.8 + 0.2 * (i % 2)                                      # Alternate saturation values
            v = 0.9                                                       # Constant brightness value
            
            r, g, b = colorsys.hsv_to_rgb(h, s, v)                       # Convert HSV to RGB
            colors.append((r, g, b))                                      # Store color tuple
        return colors                                                     # Return color list

    def calculate_convex_hull_2d(self, points):
        """Calculate the 2D convex hull for a set of points (Graham scan algorithm)"""
        if len(points) < 3:                                              # Handle degenerate cases
            return points
            
        lowest = min(range(len(points)), key=lambda i: (points[i][1], points[i][0]))  # Find bottom-most point
        
        def polar_angle(p):                                              # Calculate polar angle for sorting
            return math.atan2(p[1] - points[lowest][1], p[0] - points[lowest][0])
            
        sorted_points = sorted(points, key=lambda p: (polar_angle(p), p[0], p[1]))  # Sort by polar angle
        
        hull = [sorted_points[0], sorted_points[1]]                      # Initialize hull with first two points
        
        for i in range(2, len(sorted_points)):                          # Process remaining points
            while len(hull) > 1:                                         # Maintain convex property
                x1, y1 = hull[-1][0] - hull[-2][0], hull[-1][1] - hull[-2][1]  # Vector from second-last to last point
                x2, y2 = sorted_points[i][0] - hull[-1][0], sorted_points[i][1] - hull[-1][1]  # Vector to current point
                cross_product = x1 * y2 - y1 * x2                        # Cross product for turn direction
                
                if cross_product <= 0:                                   # Remove points making right/straight turns
                    hull.pop()
                else:
                    break                                                # Left turn found, keep point
                    
            hull.append(sorted_points[i])                                # Add current point to hull
            
        return hull                                                      # Return completed convex hull

    def receive_data(self):
        try:
            # Receive number of clusters
            data = self.receive_exact(4)                                    # Read 4 bytes for cluster count
            if not data:
                return
            num_clusters = struct.unpack('!I', data)[0]                     # Unpack unsigned integer
            
            if self.verbose_logging:                                        # Optional debug output
                self.get_logger().info(f'Receiving {num_clusters} clusters')
            
            self.clusters_received += num_clusters                          # Update cluster statistics
            
            # Create marker arrays for visualization
            point_marker_array = MarkerArray()                             # For individual points
            hull_marker_array = MarkerArray()                              # For convex hulls
            cube_marker_array = MarkerArray()                              # For bounding cubes
            
            # Generate visually distinct colors for clusters
            cluster_colors = self.generate_colors(max(8, num_clusters))     # At least 8 colors for distinction

            all_points = []                                                # Store all received points
            total_points = 0                                               # Track total point count
            
            # Handle empty data case
            if num_clusters == 0:                                          # No clusters received
                # Clear all markers by publishing empty arrays
                self.marker_publisher.publish(MarkerArray())                # Clear point markers
                self.hull_publisher.publish(MarkerArray())                  # Clear hull markers
                self.cube_publisher.publish(MarkerArray())                  # Clear cube markers
                return
            
            # Process each cluster
            for cluster_id in range(num_clusters):                         # Iterate through clusters
                # Get number of points in this cluster
                size_data = self.receive_exact(4)                          # Read 4 bytes for point count
                if not size_data:
                    return
                cluster_size = struct.unpack('!I', size_data)[0]           # Unpack unsigned integer
                
                if self.verbose_logging:                                    # Optional debug output
                    self.get_logger().info(f'Cluster {cluster_id}: expecting {cluster_size} points')
                
                # Get color for this cluster
                color = cluster_colors[cluster_id % len(cluster_colors)]    # Cycle through colors
                
                cluster_points = []                                           # Initialize points for this cluster
                # Process each point in the cluster
                for point_id in range(cluster_size):                        # Iterate through points
                    point_data = self.receive_exact(12)                     # Read 12 bytes (3 float32s)
                    if not point_data:
                        return
                    x, y, z = struct.unpack('!fff', point_data)            # Unpack XYZ coordinates
                    
                    # Verify the data with more tolerant checks for valid LIDAR data
                    if not (isinstance(x, float) and isinstance(y, float) and isinstance(z, float)):  # Type check
                        self.get_logger().error(f'Invalid point data type: {type(x)}, {type(y)}, {type(z)}')
                        continue
                    
                    # Check for NaN and Inf values
                    if (math.isnan(x) or math.isnan(y) or math.isnan(z) or
                        math.isinf(x) or math.isinf(y) or math.isinf(z)):  # Invalid value check
                        continue
                        
                    # Very generous bounds checking (±10km) - just to catch truly absurd values
                    if (abs(x) > 10000 or abs(y) > 10000 or abs(z) > 10000):  # Range validation
                        if point_id % 1000 == 0:                           # Limit warning frequency
                            self.get_logger().warn(f'Filtered extreme value: {x}, {y}, {z}')
                        continue
                    
                    # Add the valid point to collections
                    cluster_points.append([x, y, z])                        # Add to cluster points
                    all_points.append([x, y, z])                           # Add to all points
                    total_points += 1                                      # Increment total count

                    # Create individual point markers if enabled
                    if self.use_point_markers:                             # Optional point visualization
                        marker = Marker()                                  # Initialize marker
                        marker.header.frame_id = "map"                     # Set coordinate frame
                        marker.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
                        marker.ns = f"lidar_cluster_{cluster_id}"          # Namespace for organization
                        marker.id = total_points                           # Unique marker ID
                        marker.type = Marker.SPHERE                        # Sphere representation
                        marker.action = Marker.ADD                         # Add/modify marker
                        
                        marker.pose.position.x = x                         # Set X coordinate
                        marker.pose.position.y = y                         # Set Y coordinate
                        marker.pose.position.z = z                         # Set Z coordinate
                        
                        # Configure marker appearance
                        marker.scale.x = self.point_size                   # Set X scale
                        marker.scale.y = self.point_size                   # Set Y scale
                        marker.scale.z = self.point_size                   # Set Z scale
                        
                        marker.color.r = color[0]                          # Set red component
                        marker.color.g = color[1]                          # Set green component
                        marker.color.b = color[2]                          # Set blue component
                        marker.color.a = 0.9                               # Set transparency
                        
                        point_marker_array.markers.append(marker)          # Add to visualization

                # Process the cluster if it has points
                if cluster_points:                                          # Check for valid points
                    self.points_received += len(cluster_points)             # Update point statistics
                    
                    # Calculate cluster center and dimensions
                    center_x = sum(p[0] for p in cluster_points) / len(cluster_points)  # Average X
                    center_y = sum(p[1] for p in cluster_points) / len(cluster_points)  # Average Y
                    center_z = sum(p[2] for p in cluster_points) / len(cluster_points)  # Average Z
                    
                    # Calculate dimensions and statistics
                    min_x = min(p[0] for p in cluster_points)              # Minimum X coordinate
                    max_x = max(p[0] for p in cluster_points)              # Maximum X coordinate
                    min_y = min(p[1] for p in cluster_points)              # Minimum Y coordinate
                    max_y = max(p[1] for p in cluster_points)              # Maximum Y coordinate
                    min_z = min(p[2] for p in cluster_points)              # Minimum Z coordinate
                    max_z = max(p[2] for p in cluster_points)              # Maximum Z coordinate
                    
                    width = max_x - min_x                                  # Calculate cluster width
                    length = max_y - min_y                                 # Calculate cluster length
                    height = max_z - min_z                                 # Calculate cluster height
                    volume = width * length * height                       # Calculate cluster volume
                    
                    # Use POINTS type for efficient visualization of all cluster points
                    points_marker = Marker()                               # Initialize points marker
                    points_marker.header.frame_id = "map"                  # Set coordinate frame
                    points_marker.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
                    points_marker.ns = f"cluster_points_{cluster_id}"      # Set namespace
                    points_marker.id = 0                                   # Set marker ID
                    points_marker.type = Marker.POINTS                     # Use points visualization
                    points_marker.action = Marker.ADD                      # Add/modify marker
                    
                    # Larger points for better visibility
                    points_marker.scale.x = self.point_size * 0.7          # Set point width
                    points_marker.scale.y = self.point_size * 0.7          # Set point height
                    
                    # Add all points to the POINTS marker
                    for point in cluster_points:                           # Process each point
                        ros_point = Point()                                # Create point message
                        ros_point.x = point[0]                             # Set X coordinate
                        ros_point.y = point[1]                             # Set Y coordinate
                        ros_point.z = point[2]                             # Set Z coordinate
                        points_marker.points.append(ros_point)             # Add point to marker
                        
                        # Add matching color for each point
                        color_rgba = ColorRGBA()                           # Create color message
                        color_rgba.r = color[0]                            # Set red component
                        color_rgba.g = color[1]                            # Set green component
                        color_rgba.b = color[2]                            # Set blue component
                        color_rgba.a = 1.0                                 # Set full opacity
                        points_marker.colors.append(color_rgba)            # Add color to marker
                    
                    point_marker_array.markers.append(points_marker)       # Add to visualization
                    
                    # Add cluster center marker (larger sphere)
                    center_marker = Marker()                                # Initialize center marker
                    center_marker.header.frame_id = "map"                   # Set coordinate frame
                    center_marker.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
                    center_marker.ns = f"cluster_center_{cluster_id}"       # Set namespace
                    center_marker.id = 0                                    # Set marker ID
                    center_marker.type = Marker.SPHERE                      # Use sphere visualization
                    center_marker.action = Marker.ADD                       # Add/modify marker
                    center_marker.pose.position.x = center_x                # Set X coordinate
                    center_marker.pose.position.y = center_y                # Set Y coordinate
                    center_marker.pose.position.z = center_z                # Set Z coordinate
                    center_marker.scale.x = self.center_size                # Set sphere diameter
                    center_marker.scale.y = self.center_size                # Set sphere diameter
                    center_marker.scale.z = self.center_size                # Set sphere diameter
                    center_marker.color.r = color[0]                        # Set red component
                    center_marker.color.g = color[1]                        # Set green component
                    center_marker.color.b = color[2]                        # Set blue component
                    center_marker.color.a = 0.9                             # Set transparency
                    point_marker_array.markers.append(center_marker)        # Add to visualization
                    
                    # Add solid cube for the cluster 
                    cube_marker = Marker()                                  # Initialize cube marker
                    cube_marker.header.frame_id = "map"                     # Set coordinate frame
                    cube_marker.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
                    cube_marker.ns = f"cluster_cube_{cluster_id}"           # Set namespace
                    cube_marker.id = 0                                      # Set marker ID
                    cube_marker.type = Marker.CUBE                          # Use cube visualization
                    cube_marker.action = Marker.ADD                         # Add/modify marker
                    
                    # Set cube position to cluster center
                    cube_marker.pose.position.x = center_x                  # Set X coordinate
                    cube_marker.pose.position.y = center_y                  # Set Y coordinate
                    cube_marker.pose.position.z = center_z                  # Set Z coordinate
                    
                    # Set orientation (identity quaternion)
                    cube_marker.pose.orientation.w = 1.0                    # No rotation
                    
                    # Set cube dimensions
                    cube_marker.scale.x = width                             # Set width
                    cube_marker.scale.y = length                            # Set length
                    cube_marker.scale.z = height                            # Set height
                    
                    # Set cube color (semi-transparent to see points inside)
                    cube_marker.color.r = color[0]                          # Set red component
                    cube_marker.color.g = color[1]                          # Set green component
                    cube_marker.color.b = color[2]                          # Set blue component
                    cube_marker.color.a = self.cube_alpha                   # Set transparency
                    
                    cube_marker_array.markers.append(cube_marker)           # Add to visualization
                    
                    # Add bounding box as a LINE_LIST
                    box_marker = Marker()                                   # Initialize box marker
                    box_marker.header.frame_id = "map"                      # Set coordinate frame
                    box_marker.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
                    box_marker.ns = f"cluster_box_{cluster_id}"             # Set namespace
                    box_marker.id = 0                                       # Set marker ID
                    box_marker.type = Marker.LINE_LIST                      # Use line list visualization
                    box_marker.action = Marker.ADD                          # Add/modify marker
                    box_marker.scale.x = 0.1                                # Set line width
                    box_marker.color.r = color[0]                           # Set red component
                    box_marker.color.g = color[1]                           # Set green component
                    box_marker.color.b = color[2]                           # Set blue component
                    box_marker.color.a = 1.0                                # Set full opacity
                    
                    # Define the 8 corners of the box
                    corners = [                                             # Define box vertices
                        (min_x, min_y, min_z), (max_x, min_y, min_z),      # Bottom face corners
                        (max_x, max_y, min_z), (min_x, max_y, min_z),      # Bottom face corners
                        (min_x, min_y, max_z), (max_x, min_y, max_z),      # Top face corners
                        (max_x, max_y, max_z), (min_x, max_y, max_z)       # Top face corners
                    ]
                    
                    # Define the 12 lines of the box (each line connects 2 points)
                    lines = [                                               # Define box edges
                        (0, 1), (1, 2), (2, 3), (3, 0),                    # Bottom face edges
                        (4, 5), (5, 6), (6, 7), (7, 4),                    # Top face edges
                        (0, 4), (1, 5), (2, 6), (3, 7)                     # Vertical edges
                    ]
                    
                    # Add the points and lines to the marker
                    for start_idx, end_idx in lines:                        # Process each line
                        start = corners[start_idx]                          # Get start point
                        end = corners[end_idx]                              # Get end point
                        
                        p1 = Point()                                        # Create start point
                        p1.x, p1.y, p1.z = start                           # Set coordinates
                        box_marker.points.append(p1)                        # Add to marker
                        
                        p2 = Point()                                        # Create end point
                        p2.x, p2.y, p2.z = end                             # Set coordinates
                        box_marker.points.append(p2)                        # Add to marker
                    
                    hull_marker_array.markers.append(box_marker)            # Add to visualization
                    
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
            
            # Create and publish point cloud message if points exist
            if all_points:
                msg = PointCloud2()                                        # Initialize point cloud message
                msg.header.stamp = self.get_clock().now().to_msg()         # Set current timestamp
                msg.header.frame_id = 'map'                                # Set reference frame
                
                # Define point cloud structure (x,y,z floating point coordinates)
                msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),    # X coordinate field
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),    # Y coordinate field
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),    # Z coordinate field
                ]
                
                points_array = np.array(all_points, dtype=np.float32)      # Convert points to numpy array
                msg.height = 1                                             # Unorganized cloud (single row)
                msg.width = len(all_points)                                # Number of points
                msg.is_bigendian = False                                   # Use little-endian byte order
                msg.point_step = 12                                        # Size of each point (3 * float32)
                msg.row_step = msg.point_step * len(all_points)            # Size of point cloud data
                msg.is_dense = True                                        # No invalid points
                msg.data = points_array.tobytes()                          # Serialize point data
                
                self.publisher.publish(msg)                                # Publish to point cloud topic
            
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {str(e)}')

    # REMOVED: publish_vehicle_visualization method as it's no longer needed

    def __del__(self):
        """Clean up resources by closing the TCP socket connection"""
        self.socket.close()                                              # Close socket connection

def main(args=None):
    """
    Main entry point for the LiDAR cluster visualization node.
    Initializes ROS2, creates the node instance, and handles the main processing loop.
    
    Args:
        args: Command line arguments passed to ROS2 (optional)
    """
    rclpy.init(args=args)                                               # Initialize ROS2 Python client library
    node = LidarClient_clusters_2()                                     # Create instance of our LiDAR client node
    rclpy.spin(node)                                                    # Process callbacks until shutdown is called
    node.destroy_node()                                                 # Clean up node resources
    rclpy.shutdown()                                                    # Shut down ROS2 client library

if __name__ == '__main__':
    main()                                                             # Execute main function when script is run directly