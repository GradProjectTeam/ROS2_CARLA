#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Time
import numpy as np
import struct
import socket
import time
import math
import colorsys

class LidarVehicleFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_vehicle_filter_node')
        
        # TCP Parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')
        self.declare_parameter('tcp_port', 12350)
        self.declare_parameter('point_size', 2.0)
        self.declare_parameter('verbose_logging', False)
        
        # Vehicle Parameters - Configured for Carla LIDAR setup
        # LIDAR is mounted 1.5m forward and 2.0m high on the vehicle
        self.declare_parameter('vehicle_length', 4.5)           # Length of vehicle in meters (x direction)
        self.declare_parameter('vehicle_width', 2.0)            # Width of vehicle in meters (y direction)
        self.declare_parameter('vehicle_height', 1.8)           # Height of vehicle in meters (z direction)
        self.declare_parameter('vehicle_x_offset', -0.75)       # Offset of vehicle center in x direction, negative because LIDAR is forward
        self.declare_parameter('vehicle_y_offset', 0.0)         # Offset of vehicle center in y direction
        self.declare_parameter('vehicle_safety_margin', 0.2)    # Extra margin around vehicle to filter
        self.declare_parameter('visualize_vehicle', True)       # Whether to visualize the vehicle filter zone
        self.declare_parameter('output_topic', '/lidar/filtered_points') # Output topic for filtered point cloud
        
        # New comparison parameters
        self.declare_parameter('visualize_removed_points', True)   # Show points that would be filtered out
        self.declare_parameter('show_comparison_stats', True)      # Print detailed comparison statistics
        self.declare_parameter('use_box_mode', True)               # Use box-shaped filter instead of cylinder
        self.declare_parameter('filter_height_min', -1.5)          # Minimum height for filtering
        self.declare_parameter('filter_height_max', 2.0)           # Maximum height for filtering
        self.declare_parameter('debug_mode', True)                 # Enable debugging visualizations
        self.declare_parameter('comparison_update_rate', 0.5)      # Update rate for comparison visualization in seconds
        
        # LIDAR Frame Parameters - Must match the Carla configuration
        self.declare_parameter('lidar_frame_id', 'lidar')       # Frame ID for the LIDAR point cloud messages
        self.declare_parameter('map_frame_id', 'map')           # Frame ID for visualization in the map frame
        
        # Get TCP parameters
        self.tcp_ip = self.get_parameter('tcp_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.point_size = self.get_parameter('point_size').value
        self.verbose_logging = self.get_parameter('verbose_logging').value
        
        # Get vehicle parameters
        self.vehicle_length = self.get_parameter('vehicle_length').value
        self.vehicle_width = self.get_parameter('vehicle_width').value
        self.vehicle_height = self.get_parameter('vehicle_height').value
        self.vehicle_x_offset = self.get_parameter('vehicle_x_offset').value
        self.vehicle_y_offset = self.get_parameter('vehicle_y_offset').value
        self.vehicle_safety_margin = self.get_parameter('vehicle_safety_margin').value
        self.visualize_vehicle = self.get_parameter('visualize_vehicle').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Get new comparison parameters
        self.visualize_removed_points = self.get_parameter('visualize_removed_points').value
        self.show_comparison_stats = self.get_parameter('show_comparison_stats').value
        self.use_box_mode = self.get_parameter('use_box_mode').value
        self.filter_height_min = self.get_parameter('filter_height_min').value
        self.filter_height_max = self.get_parameter('filter_height_max').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.comparison_update_rate = self.get_parameter('comparison_update_rate').value
        
        # Get frame IDs
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # Calculate vehicle filter boundaries with safety margin
        self.vehicle_x_min = self.vehicle_x_offset - (self.vehicle_length / 2) - self.vehicle_safety_margin
        self.vehicle_x_max = self.vehicle_x_offset + (self.vehicle_length / 2) + self.vehicle_safety_margin
        self.vehicle_y_min = self.vehicle_y_offset - (self.vehicle_width / 2) - self.vehicle_safety_margin
        self.vehicle_y_max = self.vehicle_y_offset + (self.vehicle_width / 2) + self.vehicle_safety_margin
        self.vehicle_z_min = self.filter_height_min if self.use_box_mode else -self.vehicle_safety_margin
        self.vehicle_z_max = self.filter_height_max if self.use_box_mode else self.vehicle_height + self.vehicle_safety_margin
        
        # Stats
        self.points_received = 0
        self.clusters_received = 0
        self.points_filtered = 0
        self.last_stats_time = time.time()
        
        # Detailed comparison stats
        self.total_original_points = 0
        self.total_filtered_points = 0
        self.total_removed_points = 0
        self.filter_efficiency = 0.0  # percentage of points removed
        self.frames_processed = 0
        self.points_per_frame_avg = 0.0
        self.filter_time_ms_avg = 0.0
        self.filter_time_accumulator = 0.0
        
        # Store recent point data for visualization
        self.recent_original_points = []
        self.recent_filtered_points = []
        self.recent_removed_points = []
        
        # Publishers
        self.points_publisher = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.filtered_publisher = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.vehicle_publisher = self.create_publisher(Marker, '/lidar/vehicle_filter_zone', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/lidar/markers', 10)
        
        # New publishers for comparison visualization
        self.removed_points_publisher = self.create_publisher(
            MarkerArray, '/lidar/removed_points', 10)
        self.filter_stats_marker_publisher = self.create_publisher(
            Marker, '/lidar/filter_stats', 10)
        
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
        
        # Log vehicle filter configuration
        self.get_logger().info("Lidar Vehicle Filter Node initialized with COMPARISON MODE")
        self.get_logger().info(f"Vehicle filter zone: X [{self.vehicle_x_min:.2f}, {self.vehicle_x_max:.2f}], " 
                             f"Y [{self.vehicle_y_min:.2f}, {self.vehicle_y_max:.2f}], "
                             f"Z [{self.vehicle_z_min:.2f}, {self.vehicle_z_max:.2f}]")
        if self.use_box_mode:
            self.get_logger().info(f"Using BOX MODE for filtering with height range: [{self.filter_height_min}, {self.filter_height_max}]")
        
        # Create timers
        self.receiver_timer = self.create_timer(0.01, self.receive_data)  # 100Hz
        self.stats_timer = self.create_timer(1.0, self.report_stats)  # 1Hz stats
        if self.visualize_vehicle:
            self.visualization_timer = self.create_timer(0.5, self.publish_vehicle_visualization)
        
        # New timer for comparison visualization
        if self.visualize_removed_points:
            self.comparison_timer = self.create_timer(
                self.comparison_update_rate, self.publish_comparison_visualization)
    
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

    def point_in_vehicle(self, x, y, z):
        """Check if a point is inside the vehicle boundaries"""
        if self.use_box_mode:
            return (self.vehicle_x_min <= x <= self.vehicle_x_max and
                    self.vehicle_y_min <= y <= self.vehicle_y_max and
                    self.vehicle_z_min <= z <= self.vehicle_z_max)
        else:
            # Cylindrical filtering
            dx = x - self.vehicle_x_offset
            dy = y - self.vehicle_y_offset
            distance_xy = math.sqrt(dx*dx + dy*dy)
            radius = max(self.vehicle_length, self.vehicle_width) / 2 + self.vehicle_safety_margin
            return (distance_xy <= radius and 
                    self.vehicle_z_min <= z <= self.vehicle_z_max)

    def receive_data(self):
        """Receive LIDAR data over TCP and apply vehicle filtering"""
        try:
            # First, publish the vehicle visualization if filtering is enabled
            if self.visualize_vehicle:
                self.publish_vehicle_visualization()
                
            # Receive number of clusters
            data = self.receive_exact(4)
            if not data:
                return
            num_clusters = struct.unpack('!I', data)[0]
            
            if self.verbose_logging:
                self.get_logger().info(f'Receiving {num_clusters} clusters')
            
            self.clusters_received += num_clusters
            
            # Create marker array for visualization
            marker_array = MarkerArray()
            
            # Generate cluster colors
            cluster_colors = self.generate_colors(max(8, num_clusters))
            
            # Process all points from all clusters
            all_points = []
            filtered_points = []
            removed_points = []
            
            # Start timing filter process
            filter_start_time = time.time()
            
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
                
                # Create marker for this cluster
                marker = Marker()
                marker.header.frame_id = self.map_frame_id  # Use the map frame for visualization
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"cluster_{cluster_id}"
                marker.id = cluster_id
                marker.type = Marker.POINTS
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.point_size
                marker.scale.y = self.point_size
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1.0
                
                # Process each point in the cluster
                for point_id in range(cluster_size):
                    point_data = self.receive_exact(12)  # 3 * float32
                    if not point_data:
                        return
                    x, y, z = struct.unpack('!fff', point_data)
                    
                    # Verify the data
                    if not (isinstance(x, float) and isinstance(y, float) and isinstance(z, float)):
                        self.get_logger().error(f'Invalid point data type: {type(x)}, {type(y)}, {type(z)}')
                        continue
                    
                    # Check for NaN and Inf
                    if (math.isnan(x) or math.isnan(y) or math.isnan(z) or
                        math.isinf(x) or math.isinf(y) or math.isinf(z)):
                        continue
                    
                    # Add the point to the cluster marker
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = z
                    marker.points.append(p)
                    
                    # Add color to match the cluster
                    if self.debug_mode:
                        c = ColorRGBA()
                        c.r = color[0]
                        c.g = color[1]
                        c.b = color[2]
                        c.a = 1.0
                        marker.colors.append(c)
                    
                    # Add the point to overall arrays
                    all_points.append((x, y, z, color))
                    
                    # Check if point is inside vehicle boundaries
                    if not self.point_in_vehicle(x, y, z):
                        # Add to filtered points
                        filtered_points.append((x, y, z, color))
                    else:
                        # Count filtered points and add to removed points
                        self.points_filtered += 1
                        removed_points.append((x, y, z, color))
                
                # Add the marker to the array
                marker_array.markers.append(marker)
            
            # End timing filter process
            filter_end_time = time.time()
            filter_time_ms = (filter_end_time - filter_start_time) * 1000.0
            
            # Update comparison stats
            if len(all_points) > 0:
                self.frames_processed += 1
                self.total_original_points += len(all_points)
                self.total_filtered_points += len(filtered_points)
                self.total_removed_points += len(removed_points)
                
                if self.total_original_points > 0:
                    self.filter_efficiency = (self.total_removed_points / self.total_original_points) * 100.0
                
                self.points_per_frame_avg = self.total_original_points / self.frames_processed
                self.filter_time_accumulator += filter_time_ms
                self.filter_time_ms_avg = self.filter_time_accumulator / self.frames_processed
            
            # Store recent points for visualization
            self.recent_original_points = all_points
            self.recent_filtered_points = filtered_points
            self.recent_removed_points = removed_points
            
            # Publish marker array for visualization
            self.marker_publisher.publish(marker_array)
            
            # Create point cloud message for all points
            if all_points:
                self.points_received += len(all_points)
                points_msg = self.create_point_cloud_msg([(p[0], p[1], p[2]) for p in all_points], self.lidar_frame_id)
                self.points_publisher.publish(points_msg)
            
            # Create point cloud message for filtered points
            if filtered_points:
                filtered_msg = self.create_point_cloud_msg([(p[0], p[1], p[2]) for p in filtered_points], self.lidar_frame_id)
                self.filtered_publisher.publish(filtered_msg)
            
            # If in debug mode, show additional stats with current frame info
            if self.debug_mode and self.show_comparison_stats:
                self.get_logger().info(
                    f'Frame {self.frames_processed}: {len(all_points)} points → '
                    f'{len(filtered_points)} kept, {len(removed_points)} removed ({len(removed_points)/max(1, len(all_points))*100:.1f}%), '
                    f'Time: {filter_time_ms:.2f}ms'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in receive_data: {str(e)}')
    
    def create_point_cloud_msg(self, points, frame_id):
        """Create a PointCloud2 message from a list of (x,y,z) points"""
        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Set other message properties
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32 (12 bytes)
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        
        # Create byte array
        if len(points) > 0:
            buffer = bytearray(msg.row_step)
            for i, point in enumerate(points):
                offset = i * msg.point_step
                struct.pack_into('fff', buffer, offset, point[0], point[1], point[2])
            msg.data = bytes(buffer)
        else:
            msg.data = b''
        
        return msg

    def report_stats(self):
        """Report statistics about filtering performance"""
        now = time.time()
        elapsed = now - self.last_stats_time
        
        if elapsed > 0:
            points_per_sec = self.points_received / elapsed
            clusters_per_sec = self.clusters_received / elapsed
            filter_percentage = 0.0
            if self.points_received > 0:
                filter_percentage = (self.points_filtered / self.points_received) * 100
            
            # Basic stats logging
            self.get_logger().info(
                f'Basic Stats: Received {points_per_sec:.1f} points/sec across {clusters_per_sec:.1f} clusters/sec, '
                f'Filtered {self.points_filtered/elapsed:.1f} points/sec ({filter_percentage:.1f}%)'
            )
            
            # If comparison stats enabled, show additional info
            if self.show_comparison_stats and self.total_original_points > 0:
                self.get_logger().info(
                    f'Comparison Stats: '
                    f'Overall Efficiency: {self.filter_efficiency:.1f}% points removed, '
                    f'Avg {self.points_per_frame_avg:.1f} points/frame, '
                    f'Filter time: {self.filter_time_ms_avg:.2f}ms'
                )
            
            # Reset counters
            self.points_received = 0
            self.clusters_received = 0
            self.points_filtered = 0
            self.last_stats_time = now
            
            # Publish filter stats visualization
            self.publish_filter_stats_visualization()

    def publish_vehicle_visualization(self):
        """Publish a visualization of the vehicle filter zone"""
        marker = Marker()
        marker.header.frame_id = self.map_frame_id  # Use the map frame for visualization
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle_filter_zone"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position to vehicle center
        marker.pose.position.x = self.vehicle_x_offset
        marker.pose.position.y = self.vehicle_y_offset
        marker.pose.position.z = self.vehicle_z_max / 2  # Center in Z
        
        # Set orientation (identity quaternion)
        marker.pose.orientation.w = 1.0
        
        # Set dimensions - using the filter boundaries
        marker.scale.x = self.vehicle_x_max - self.vehicle_x_min
        marker.scale.y = self.vehicle_y_max - self.vehicle_y_min
        marker.scale.z = self.vehicle_z_max - self.vehicle_z_min
        
        # Set color - red, semi-transparent
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3  # Semi-transparent
        
        # Set lifetime
        marker.lifetime.sec = 2
        
        # Publish
        self.vehicle_publisher.publish(marker)
    
    def publish_comparison_visualization(self):
        """Publish visualization of removed points for comparison"""
        if not self.recent_removed_points:
            return
            
        # Create marker for removed points
        marker_array = MarkerArray()
        
        # Points that were removed (vehicle hits)
        removed_marker = Marker()
        removed_marker.header.frame_id = self.map_frame_id
        removed_marker.header.stamp = self.get_clock().now().to_msg()
        removed_marker.ns = "removed_points"
        removed_marker.id = 0
        removed_marker.type = Marker.POINTS
        removed_marker.action = Marker.ADD
        removed_marker.pose.orientation.w = 1.0
        removed_marker.scale.x = self.point_size * 1.5  # Make removed points larger for visibility
        removed_marker.scale.y = self.point_size * 1.5
        
        # Add all removed points
        for point in self.recent_removed_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            removed_marker.points.append(p)
            
            # Use the point's original color but make it more red to indicate removal
            c = ColorRGBA()
            c.r = 1.0  # Force red component high
            c.g = 0.0  # Zero out green
            c.b = 0.2  # Minimal blue
            c.a = 0.8  # Make slightly transparent
            removed_marker.colors.append(c)
        
        marker_array.markers.append(removed_marker)
        
        # Add a text marker showing how many points were removed
        if self.debug_mode:
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "removed_points_text"
            text_marker.id = 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = self.vehicle_x_offset
            text_marker.pose.position.y = self.vehicle_y_offset
            text_marker.pose.position.z = self.vehicle_z_max + 0.5  # Above the vehicle
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.5  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Info text about removed points
            removed_percent = 0.0
            if len(self.recent_original_points) > 0:
                removed_percent = (len(self.recent_removed_points) / len(self.recent_original_points)) * 100.0
                
            text_marker.text = f"Removed: {len(self.recent_removed_points)} points ({removed_percent:.1f}%)"
            marker_array.markers.append(text_marker)
        
        # Publish
        self.removed_points_publisher.publish(marker_array)
    
    def publish_filter_stats_visualization(self):
        """Publish visualization of filter statistics"""
        if not self.show_comparison_stats or self.total_original_points == 0:
            return
        
        # Create text marker with stats
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filter_stats"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.vehicle_x_offset + 3.0  # Offset to the right of vehicle
        marker.pose.position.y = self.vehicle_y_offset
        marker.pose.position.z = self.vehicle_z_max + 1.0  # Above the vehicle
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.5  # Text size
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Format multi-line text with stats
        marker.text = (
            f"Filter Statistics:\n"
            f"Total Points: {self.total_original_points}\n"
            f"Kept Points: {self.total_filtered_points} ({(self.total_filtered_points/max(1,self.total_original_points))*100:.1f}%)\n"
            f"Removed Points: {self.total_removed_points} ({self.filter_efficiency:.1f}%)\n"
            f"Avg Processing: {self.filter_time_ms_avg:.2f}ms\n"
            f"Filter Mode: {'BOX' if self.use_box_mode else 'CYLINDER'}\n"
            f"Height Range: [{self.filter_height_min:.1f}, {self.filter_height_max:.1f}]"
        )
        
        # Publish
        self.filter_stats_marker_publisher.publish(marker)
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        try:
            self.socket.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = LidarVehicleFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 