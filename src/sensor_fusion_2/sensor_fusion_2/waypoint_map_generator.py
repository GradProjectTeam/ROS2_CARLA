#!/usr/bin/env python3

"""
Waypoint Map Generator for Autonomous Vehicle Navigation
=====================================================

Authors: Shishtawy & Hendy
Project: TechZ Autonomous Driving System

This node generates a binary occupancy grid map from waypoint markers for autonomous navigation.
The generated map represents drivable paths and navigation waypoints in a format suitable for
path planning algorithms and visualization in RViz.

Key Features:
- Converts waypoint markers to occupancy grid representation
- Supports both point-based and line-based waypoint visualization
- Configurable map resolution and dimensions
- Dynamic map centering on vehicle position
- Real-time map updates with configurable publish rate
- Support for both VOLATILE and TRANSIENT_LOCAL durability

Frame Structure:
- map: Global reference frame for waypoint visualization
- base_link: Vehicle reference frame for dynamic map centering

Dependencies:
- ROS2 Humble
- NumPy for efficient grid operations
- TF2 for coordinate transformations
- Navigation Messages for map publishing
"""

import numpy as np                                   # For efficient array operations and grid manipulation
import math                                         # For mathematical calculations
import threading                                    # For thread-safe operations
import time                                         # For timing and delay operations
from queue import Queue, Empty                      # For thread-safe queue operations

import rclpy                                        # ROS2 Python client library
from rclpy.node import Node                         # Base class for ROS2 nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # QoS configuration
from visualization_msgs.msg import MarkerArray      # For waypoint marker messages
from nav_msgs.msg import OccupancyGrid             # For publishing occupancy grid maps
from geometry_msgs.msg import Point, TransformStamped  # For geometric transformations
from tf2_ros import Buffer, TransformListener       # For handling coordinate transforms

class WaypointMapGenerator(Node):                   # Main node class for waypoint map generation
    def __init__(self):
        super().__init__('waypoint_map_generator')  # Initialize the ROS2 node with name
        
        # Declare parameters
        self.declare_parameter('map_frame_id', 'map')                          # Global reference frame for the map
        self.declare_parameter('vehicle_frame_id', 'base_link')               # Vehicle's reference frame
        self.declare_parameter('map_resolution', 0.5)                         # Size of each grid cell in meters
        self.declare_parameter('map_width_meters', 120.0)                     # Total width of map in meters
        self.declare_parameter('map_height_meters', 120.0)                    # Total height of map in meters
        self.declare_parameter('publish_rate', 10.0)                          # Map update frequency in Hz
        self.declare_parameter('waypoint_marker_topic', '/carla/waypoint_markers')  # Topic for waypoint markers
        self.declare_parameter('binary_topic', '/waypoint_map/binary')        # Topic for publishing binary map
        self.declare_parameter('occupied_value', 100)                         # Value for occupied cells (100 = black)
        self.declare_parameter('free_value', 0)                              # Value for free cells (0 = white)
        self.declare_parameter('waypoint_width', 0.5)                        # Width of waypoint visualization in meters
        self.declare_parameter('use_vehicle_frame', False)                   # Whether to center map on vehicle
        self.declare_parameter('use_transient_local_durability', True)       # QoS setting for map persistence
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value                # Get global frame ID
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').get_parameter_value().string_value        # Get vehicle frame ID
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value            # Get cell size
        self.map_width_meters = self.get_parameter('map_width_meters').get_parameter_value().double_value        # Get map width
        self.map_height_meters = self.get_parameter('map_height_meters').get_parameter_value().double_value      # Get map height
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value                # Get update rate
        self.waypoint_marker_topic = self.get_parameter('waypoint_marker_topic').get_parameter_value().string_value  # Get marker topic
        self.binary_topic = self.get_parameter('binary_topic').get_parameter_value().string_value                # Get binary map topic
        self.occupied_value = self.get_parameter('occupied_value').get_parameter_value().integer_value           # Get occupied cell value
        self.free_value = self.get_parameter('free_value').get_parameter_value().integer_value                  # Get free cell value
        self.waypoint_width = self.get_parameter('waypoint_width').get_parameter_value().double_value           # Get waypoint width
        self.use_vehicle_frame = self.get_parameter('use_vehicle_frame').get_parameter_value().bool_value       # Get frame centering flag
        self.use_transient_local_durability = self.get_parameter('use_transient_local_durability').get_parameter_value().bool_value  # Get QoS setting
        
        # Calculate map dimensions
        self.map_width = int(self.map_width_meters / self.map_resolution)    # Convert width from meters to cells
        self.map_height = int(self.map_height_meters / self.map_resolution)  # Convert height from meters to cells
        
        # Set up QoS profiles for publisher
        pub_qos_profile = QoSProfile(                                        # Configure publisher quality of service
            reliability=ReliabilityPolicy.RELIABLE,                          # Ensure reliable message delivery
            history=HistoryPolicy.KEEP_LAST,                                 # Keep only most recent messages
            depth=10                                                         # Store up to 10 messages in history
        )
        
        # Set durability based on parameter for publisher
        if self.use_transient_local_durability:                             # Check if transient local durability is enabled
            pub_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL   # Late subscribers get last message
            self.get_logger().info("Using TRANSIENT_LOCAL durability for binary map publisher")
        else:
            pub_qos_profile.durability = DurabilityPolicy.VOLATILE          # Messages not stored for late subscribers
            self.get_logger().info("Using VOLATILE durability for binary map publisher")
        
        # Set up QoS profile for subscriber - typically VOLATILE is fine for marker arrays
        sub_qos_profile = QoSProfile(                                       # Configure subscriber quality of service
            reliability=ReliabilityPolicy.RELIABLE,                         # Request reliable message delivery
            history=HistoryPolicy.KEEP_LAST,                                # Keep only most recent messages
            depth=10,                                                       # Store up to 10 messages in history
            durability=DurabilityPolicy.VOLATILE                           # Don't request old messages
        )
        
        # Create publishers with explicit QoS
        self.binary_map_publisher = self.create_publisher(                  # Create publisher for binary occupancy grid
            OccupancyGrid,                                                 # Message type for grid map
            self.binary_topic,                                            # Topic name from parameters
            pub_qos_profile                                               # QoS settings for reliability
        )
        
        # Create subscribers with appropriate QoS
        self.waypoint_subscriber = self.create_subscription(               # Create subscriber for waypoint markers
            MarkerArray,                                                  # Message type for markers
            self.waypoint_marker_topic,                                   # Topic name from parameters
            self.waypoint_callback,                                       # Callback function for processing markers
            sub_qos_profile                                              # QoS settings for subscription
        )
        
        # Set up TF listener
        self.tf_buffer = Buffer()                                           # Create buffer for storing transforms
        self.tf_listener = TransformListener(self.tf_buffer, self)          # Create listener for transform updates
        
        # Initialize map data
        self.map_data = np.full((self.map_height, self.map_width), self.free_value, dtype=np.int8)  # Create empty map grid
        self.map_lock = threading.Lock()                                    # Create mutex for thread-safe map access
        self.waypoint_queue = Queue(maxsize=1)                             # Queue for storing latest waypoints
        self.has_new_waypoints = False                                     # Flag for waypoint processing
        
        # Create timer for publishing map
        self.map_timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)  # Timer for regular map updates
        
        # Start processing thread
        self.running = True                                                # Flag for thread control
        self.process_thread = threading.Thread(target=self.process_waypoints)  # Create waypoint processing thread
        self.process_thread.daemon = True                                  # Set as daemon thread to exit with main thread
        self.process_thread.start()                                        # Start the processing thread
        
        self.get_logger().info("Waypoint map generator initialized")       # Log initialization success
        self.get_logger().info(f"Publishing waypoint binary map to: {self.binary_topic}")  # Log publishing topic
        self.get_logger().info(f"Map dimensions: {self.map_width}x{self.map_height} cells, {self.map_width_meters}x{self.map_height_meters} meters")  # Log map size
        self.get_logger().info(f"Map resolution: {self.map_resolution} meters/cell")  # Log map resolution
        self.get_logger().info(f"Waypoint width: {self.waypoint_width} meters")  # Log waypoint visualization width
        self.get_logger().info(f"Map frame ID: {self.map_frame_id}")      # Log reference frame
    
    def waypoint_callback(self, marker_array):
        """Process incoming waypoint markers"""
        try:
            # Extract waypoints from marker array
            waypoints = []                                                 # List to store extracted waypoints
            for marker in marker_array.markers:                           # Process each marker in the array
                if marker.ns == "waypoint_points" and marker.type == 8:  # Handle sphere list markers (points)
                    for point in marker.points:                          # Extract each point from sphere list
                        waypoints.append({                               # Store point coordinates
                            'x': point.x,
                            'y': point.y,
                            'z': point.z
                        })
                elif marker.ns == "waypoint_lines" and marker.type == 4:  # Handle line strip markers (paths)
                    for point in marker.points:                          # Extract each point from line strip
                        waypoints.append({                               # Store point coordinates
                            'x': point.x,
                            'y': point.y,
                            'z': point.z
                        })
            
            if waypoints:                                               # If waypoints were found
                # Replace old waypoints in queue
                if self.waypoint_queue.full():                          # If queue is full
                    try:
                        self.waypoint_queue.get_nowait()                # Remove old waypoints
                    except Empty:
                        pass
                self.waypoint_queue.put(waypoints)                      # Add new waypoints to queue
                self.has_new_waypoints = True                           # Set flag for processing
                self.get_logger().debug(f"Received {len(waypoints)} waypoints")  # Log waypoint count
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint markers: {e}")  # Log any errors
    
    def process_waypoints(self):
        """Process waypoints from queue and update map"""
        while self.running:                                            # Run until node is shutdown
            try:
                if self.has_new_waypoints and not self.waypoint_queue.empty():  # Check for new waypoints
                    waypoints = self.waypoint_queue.get()              # Get waypoints from queue
                    self.update_map(waypoints)                         # Update map with new waypoints
                    self.has_new_waypoints = False                     # Reset new waypoints flag
                else:
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.01)                                   # Brief sleep when no new data
            except Exception as e:
                self.get_logger().error(f"Error in process_waypoints: {e}")  # Log any errors
    
    def update_map(self, waypoints):
        """Update map with waypoints"""
        if not waypoints:
            return
            
        try:
            # Create a new map
            new_map = np.full((self.map_height, self.map_width), self.free_value, dtype=np.int8)  # Initialize empty map
            
            # Get vehicle position if using vehicle frame
            vehicle_x, vehicle_y = 0.0, 0.0                                # Default vehicle position at origin
            if self.use_vehicle_frame:                                     # If map should be vehicle-centered
                try:
                    transform = self.tf_buffer.lookup_transform(           # Get vehicle transform
                        self.map_frame_id,                                # Target frame (map)
                        self.vehicle_frame_id,                            # Source frame (vehicle)
                        rclpy.time.Time()                                 # Latest transform
                    )
                    vehicle_x = transform.transform.translation.x         # Get vehicle X position
                    vehicle_y = transform.transform.translation.y         # Get vehicle Y position
                except Exception as e:
                    self.get_logger().warn(f"Could not get vehicle transform: {e}")  # Log transform errors
            
            # Calculate map origin (center of the map)
            origin_x = vehicle_x - (self.map_width_meters / 2.0)         # Calculate X origin for centering
            origin_y = vehicle_y - (self.map_height_meters / 2.0)        # Calculate Y origin for centering
            
            # Draw waypoints on map
            waypoint_radius_cells = max(1, int(self.waypoint_width / (2.0 * self.map_resolution)))  # Convert width to cells
            
            for wp in waypoints:                                         # Process each waypoint
                # Convert waypoint to grid coordinates
                grid_x = int((wp['x'] - origin_x) / self.map_resolution)  # Convert X to grid coordinate
                grid_y = int((wp['y'] - origin_y) / self.map_resolution)  # Convert Y to grid coordinate
                
                # Check if waypoint is within map bounds
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:  # If within bounds
                    # Draw a circle around the waypoint
                    for dx in range(-waypoint_radius_cells, waypoint_radius_cells + 1):  # Iterate X radius
                        for dy in range(-waypoint_radius_cells, waypoint_radius_cells + 1):  # Iterate Y radius
                            if dx*dx + dy*dy <= waypoint_radius_cells*waypoint_radius_cells:  # If within circle
                                x = grid_x + dx                          # Calculate absolute X
                                y = grid_y + dy                          # Calculate absolute Y
                                if 0 <= x < self.map_width and 0 <= y < self.map_height:  # If within bounds
                                    new_map[y, x] = self.occupied_value  # Mark cell as occupied
            
            # Update map with lock
            with self.map_lock:                                         # Thread-safe map update
                self.map_data = new_map                                 # Replace old map
                self.map_origin_x = origin_x                            # Update X origin
                self.map_origin_y = origin_y                            # Update Y origin
                
            self.get_logger().debug(f"Updated waypoint map with {len(waypoints)} waypoints")  # Log update
                
        except Exception as e:
            self.get_logger().error(f"Error updating map: {e}")         # Log any errors
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map cell coordinates"""
        cell_x = int((x - self.map_origin_x) / self.map_resolution)    # Convert X to cell coordinate
        cell_y = int((y - self.map_origin_y) / self.map_resolution)    # Convert Y to cell coordinate
        
        # Check if within bounds
        if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:  # If within map bounds
            return cell_x, cell_y                                       # Return cell coordinates
        return None, None                                              # Return None if out of bounds
    
    def map_to_world(self, cell_x, cell_y):
        """Convert map cell coordinates to world coordinates"""
        world_x = cell_x * self.map_resolution + self.map_origin_x     # Convert X to world coordinate
        world_y = cell_y * self.map_resolution + self.map_origin_y     # Convert Y to world coordinate
        return world_x, world_y                                        # Return world coordinates
    
    def publish_map(self):
        """Publish binary occupancy map"""
        try:
            # Create occupancy grid message
            grid_msg = OccupancyGrid()                                  # Create new grid message
            
            # Set header
            grid_msg.header.stamp = self.get_clock().now().to_msg()    # Set current timestamp
            grid_msg.header.frame_id = self.map_frame_id               # Set reference frame
            
            # Set map metadata
            grid_msg.info.resolution = self.map_resolution             # Set cell size
            grid_msg.info.width = self.map_width                      # Set map width
            grid_msg.info.height = self.map_height                    # Set map height
            
            # Set map origin
            with self.map_lock:                                       # Thread-safe access to map data
                grid_msg.info.origin.position.x = getattr(self, 'map_origin_x', 0.0)  # Set X origin
                grid_msg.info.origin.position.y = getattr(self, 'map_origin_y', 0.0)  # Set Y origin
                grid_msg.info.origin.position.z = 0.0                 # Set Z origin
                grid_msg.info.origin.orientation.w = 1.0              # Set orientation (no rotation)
                
                # Flatten map data (row-major order)
                grid_msg.data = self.map_data.flatten().tolist()     # Convert map to 1D list
            
            # Publish map
            self.binary_map_publisher.publish(grid_msg)               # Send map to subscribers
            
        except Exception as e:
            self.get_logger().error(f"Error publishing map: {e}")     # Log any errors
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down waypoint map generator")  # Log shutdown
        self.running = False                                          # Stop processing thread
        
        if self.process_thread and self.process_thread.is_alive():   # If thread is running
            self.process_thread.join(timeout=1.0)                    # Wait for thread to finish
            
        super().destroy_node()                                       # Call parent cleanup

def main(args=None):
    rclpy.init(args=args)                                           # Initialize ROS2 Python client
    waypoint_map_generator = WaypointMapGenerator()                 # Create node instance
    
    try:
        rclpy.spin(waypoint_map_generator)                          # Start node processing
    except KeyboardInterrupt:
        pass                                                        # Handle graceful shutdown
    finally:
        waypoint_map_generator.destroy_node()                       # Clean up node
        rclpy.shutdown()                                           # Shutdown ROS2 client

if __name__ == '__main__':
    main()                                                         # Run main function when script is executed 