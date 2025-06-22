#!/usr/bin/env python3
"""
Waypoint Map Generator Node

This node subscribes to waypoint markers and generates a binary occupancy map
representing the waypoints for path planning and visualization.

Author: Mostafa
"""

import numpy as np
import math
import threading
import time  # Add time import
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros import Buffer, TransformListener

class WaypointMapGenerator(Node):
    def __init__(self):
        super().__init__('waypoint_map_generator')
        
        # Declare parameters
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_resolution', 0.5)  # meters per cell
        self.declare_parameter('map_width_meters', 120.0)  # meters
        self.declare_parameter('map_height_meters', 120.0)  # meters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('waypoint_marker_topic', '/carla/waypoint_markers')
        self.declare_parameter('binary_topic', '/waypoint_map/binary')
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('waypoint_width', 1.0)  # meters
        self.declare_parameter('use_vehicle_frame', False)  # If true, map is centered on vehicle
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').get_parameter_value().string_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_width_meters = self.get_parameter('map_width_meters').get_parameter_value().double_value
        self.map_height_meters = self.get_parameter('map_height_meters').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.waypoint_marker_topic = self.get_parameter('waypoint_marker_topic').get_parameter_value().string_value
        self.binary_topic = self.get_parameter('binary_topic').get_parameter_value().string_value
        self.occupied_value = self.get_parameter('occupied_value').get_parameter_value().integer_value
        self.free_value = self.get_parameter('free_value').get_parameter_value().integer_value
        self.waypoint_width = self.get_parameter('waypoint_width').get_parameter_value().double_value
        self.use_vehicle_frame = self.get_parameter('use_vehicle_frame').get_parameter_value().bool_value
        
        # Calculate map dimensions
        self.map_width = int(self.map_width_meters / self.map_resolution)
        self.map_height = int(self.map_height_meters / self.map_resolution)
        
        # Set up QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.binary_map_publisher = self.create_publisher(
            OccupancyGrid,
            self.binary_topic,
            qos_profile
        )
        
        # Create subscribers
        self.waypoint_subscriber = self.create_subscription(
            MarkerArray,
            self.waypoint_marker_topic,
            self.waypoint_callback,
            qos_profile
        )
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize map data
        self.map_data = np.full((self.map_height, self.map_width), self.free_value, dtype=np.int8)
        self.map_lock = threading.Lock()
        self.waypoint_queue = Queue(maxsize=1)
        self.has_new_waypoints = False
        
        # Create timer for publishing map
        self.map_timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)
        
        # Start processing thread
        self.running = True
        self.process_thread = threading.Thread(target=self.process_waypoints)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        self.get_logger().info("Waypoint map generator initialized")
    
    def waypoint_callback(self, marker_array):
        """Process incoming waypoint markers"""
        try:
            # Extract waypoints from marker array
            waypoints = []
            for marker in marker_array.markers:
                if marker.ns == "waypoint_points" and marker.type == 8:  # SPHERE_LIST
                    for point in marker.points:
                        waypoints.append({
                            'x': point.x,
                            'y': point.y,
                            'z': point.z
                        })
                elif marker.ns == "waypoint_lines" and marker.type == 4:  # LINE_STRIP
                    for point in marker.points:
                        waypoints.append({
                            'x': point.x,
                            'y': point.y,
                            'z': point.z
                        })
            
            if waypoints:
                # Replace old waypoints in queue
                if self.waypoint_queue.full():
                    try:
                        self.waypoint_queue.get_nowait()
                    except Empty:
                        pass
                self.waypoint_queue.put(waypoints)
                self.has_new_waypoints = True
                self.get_logger().debug(f"Received {len(waypoints)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint markers: {e}")
    
    def process_waypoints(self):
        """Process waypoints from queue and update map"""
        while self.running:
            try:
                if self.has_new_waypoints and not self.waypoint_queue.empty():
                    waypoints = self.waypoint_queue.get()
                    self.update_map(waypoints)
                    self.has_new_waypoints = False
                else:
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.01)  # Use standard Python time.sleep instead of rclpy.time.sleep_for
            except Exception as e:
                self.get_logger().error(f"Error in process_waypoints: {e}")
    
    def update_map(self, waypoints):
        """Update map with waypoints"""
        if not waypoints:
            return
            
        try:
            # Create a new map
            new_map = np.full((self.map_height, self.map_width), self.free_value, dtype=np.int8)
            
            # Get vehicle position if using vehicle frame
            vehicle_x, vehicle_y = 0.0, 0.0
            if self.use_vehicle_frame:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.map_frame_id,
                        self.vehicle_frame_id,
                        rclpy.time.Time()
                    )
                    vehicle_x = transform.transform.translation.x
                    vehicle_y = transform.transform.translation.y
                except Exception as e:
                    self.get_logger().warn(f"Could not get vehicle transform: {e}")
            
            # Calculate map origin (center of the map)
            origin_x = vehicle_x - (self.map_width_meters / 2.0)
            origin_y = vehicle_y - (self.map_height_meters / 2.0)
            
            # Draw waypoints on map
            waypoint_radius_cells = max(1, int(self.waypoint_width / (2.0 * self.map_resolution)))
            
            for wp in waypoints:
                # Convert waypoint to grid coordinates
                grid_x = int((wp['x'] - origin_x) / self.map_resolution)
                grid_y = int((wp['y'] - origin_y) / self.map_resolution)
                
                # Check if waypoint is within map bounds
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    # Draw a circle around the waypoint
                    for dx in range(-waypoint_radius_cells, waypoint_radius_cells + 1):
                        for dy in range(-waypoint_radius_cells, waypoint_radius_cells + 1):
                            if dx*dx + dy*dy <= waypoint_radius_cells*waypoint_radius_cells:
                                x = grid_x + dx
                                y = grid_y + dy
                                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                                    new_map[y, x] = self.occupied_value
            
            # Update map with lock
            with self.map_lock:
                self.map_data = new_map
                self.map_origin_x = origin_x
                self.map_origin_y = origin_y
                
        except Exception as e:
            self.get_logger().error(f"Error updating map: {e}")
    
    def publish_map(self):
        """Publish binary occupancy map"""
        try:
            # Create occupancy grid message
            grid_msg = OccupancyGrid()
            
            # Set header
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = self.map_frame_id
            
            # Set map metadata
            grid_msg.info.resolution = self.map_resolution
            grid_msg.info.width = self.map_width
            grid_msg.info.height = self.map_height
            
            # Set map origin
            with self.map_lock:
                grid_msg.info.origin.position.x = getattr(self, 'map_origin_x', 0.0)
                grid_msg.info.origin.position.y = getattr(self, 'map_origin_y', 0.0)
                grid_msg.info.origin.position.z = 0.0
                grid_msg.info.origin.orientation.w = 1.0
                
                # Flatten map data (row-major order)
                grid_msg.data = self.map_data.flatten().tolist()
            
            # Publish map
            self.binary_map_publisher.publish(grid_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing map: {e}")
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down waypoint map generator")
        self.running = False
        
        if self.process_thread and self.process_thread.is_alive():
            self.process_thread.join(timeout=1.0)
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    waypoint_map_generator = WaypointMapGenerator()
    
    try:
        rclpy.spin(waypoint_map_generator)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_map_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 