#!/usr/bin/env python3
"""
Binary Map Combiner Node

This node subscribes to two binary occupancy grid maps:
1. The semantic binary map from the semantic costmap visualizer
2. The waypoint binary map from the waypoint map generator

It combines these maps, giving priority to waypoints if specified,
and publishes a new combined binary map for navigation.

Author: Mostafa
"""

import numpy as np
import threading
import time
import os
from datetime import datetime
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class BinaryMapCombiner(Node):
    def __init__(self):
        super().__init__('binary_map_combiner')
        
        # Declare parameters
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('semantic_binary_topic', '/semantic_costmap/binary')
        self.declare_parameter('waypoint_binary_topic', '/waypoint_map/binary')
        self.declare_parameter('combined_binary_topic', '/combined_binary_map')
        self.declare_parameter('combined_binary_updates_topic', '/combined_binary_map_updates')
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('prioritize_waypoints', True)
        self.declare_parameter('use_transient_local_durability', True)
        
        # Add new parameters for lane crossing detection
        self.declare_parameter('lane_width', 3.5)  # meters
        self.declare_parameter('detect_lane_crossing_objects', True)
        self.declare_parameter('lane_safety_margin', 1.0)  # meters
        self.declare_parameter('lane_crossing_enhancement_radius', 2)  # cells
        
        # Add new parameters for unified map functionality
        self.declare_parameter('save_combined_map', True)
        self.declare_parameter('save_directory', '/home/mostafa/GP/ROS2/maps/NewMaps')
        self.declare_parameter('save_interval', 5.0)  # seconds
        self.declare_parameter('map_name_prefix', 'unified_map_')
        self.declare_parameter('add_timestamp_to_filename', True)
        self.declare_parameter('unified_map_topic', '/unified_map')
        self.declare_parameter('unified_map_quality', 'high')
        self.declare_parameter('unified_map_format', 'pgm')
        self.declare_parameter('enhanced_radar_integration', True)
        self.declare_parameter('enhanced_lidar_integration', True)
        self.declare_parameter('include_all_data_sources', True)
        self.declare_parameter('include_lidar_data', True)
        self.declare_parameter('include_radar_data', True)
        self.declare_parameter('include_waypoints', True)
        self.declare_parameter('include_all_objects', True)
        self.declare_parameter('enhance_map_with_all_sensors', True)
        self.declare_parameter('use_high_quality_mode', True)
        self.declare_parameter('radar_confidence_threshold', 0.1)
        self.declare_parameter('lidar_confidence_threshold', 0.1)
        self.declare_parameter('combine_close_detections', True)
        self.declare_parameter('filter_noise', True)
        self.declare_parameter('preserve_thin_obstacles', True)
        self.declare_parameter('preserve_distant_obstacles', True)
        self.declare_parameter('force_save_on_shutdown', True)
        self.declare_parameter('max_map_age_before_save', 60.0)
        self.declare_parameter('extra_safety_expansion', 1.0)
        self.declare_parameter('add_map_metadata', True)
        self.declare_parameter('metadata_format', 'yaml')
        self.declare_parameter('save_as_pgm', True)
        self.declare_parameter('save_as_yaml', True)
        self.declare_parameter('save_as_black_white', True)
        self.declare_parameter('binary_threshold', 50)
        self.declare_parameter('enforce_binary_values', True)
        self.declare_parameter('waypoint_value', 50)  # Value between 0 (free/white) and 100 (occupied/black)
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.semantic_binary_topic = self.get_parameter('semantic_binary_topic').get_parameter_value().string_value
        self.waypoint_binary_topic = self.get_parameter('waypoint_binary_topic').get_parameter_value().string_value
        self.combined_binary_topic = self.get_parameter('combined_binary_topic').get_parameter_value().string_value
        self.combined_binary_updates_topic = self.get_parameter('combined_binary_updates_topic').get_parameter_value().string_value if self.has_parameter('combined_binary_updates_topic') else self.combined_binary_topic + '_updates'
        self.occupied_value = self.get_parameter('occupied_value').get_parameter_value().integer_value
        self.free_value = self.get_parameter('free_value').get_parameter_value().integer_value
        self.prioritize_waypoints = self.get_parameter('prioritize_waypoints').get_parameter_value().bool_value
        self.use_transient_local_durability = self.get_parameter('use_transient_local_durability').get_parameter_value().bool_value
        
        # Get lane crossing parameters
        self.lane_width = self.get_parameter('lane_width').get_parameter_value().double_value
        self.detect_lane_crossing_objects = self.get_parameter('detect_lane_crossing_objects').get_parameter_value().bool_value
        self.lane_safety_margin = self.get_parameter('lane_safety_margin').get_parameter_value().double_value
        self.lane_crossing_enhancement_radius = self.get_parameter('lane_crossing_enhancement_radius').get_parameter_value().integer_value
        
        self.get_logger().info(f"Lane crossing detection enabled: {self.detect_lane_crossing_objects}")
        self.get_logger().info(f"Lane safety margin: {self.lane_safety_margin} meters")
        self.get_logger().info(f"Lane crossing enhancement radius: {self.lane_crossing_enhancement_radius} cells")
        
        # Get unified map parameters
        self.save_combined_map = self.get_parameter('save_combined_map').get_parameter_value().bool_value
        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value
        self.map_name_prefix = self.get_parameter('map_name_prefix').get_parameter_value().string_value
        self.add_timestamp_to_filename = self.get_parameter('add_timestamp_to_filename').get_parameter_value().bool_value
        self.unified_map_topic = self.get_parameter('unified_map_topic').get_parameter_value().string_value
        self.unified_map_quality = self.get_parameter('unified_map_quality').get_parameter_value().string_value
        self.unified_map_format = self.get_parameter('unified_map_format').get_parameter_value().string_value
        self.include_all_data_sources = self.get_parameter('include_all_data_sources').get_parameter_value().bool_value
        self.include_waypoints = self.get_parameter('include_waypoints').get_parameter_value().bool_value
        self.include_all_objects = self.get_parameter('include_all_objects').get_parameter_value().bool_value
        self.force_save_on_shutdown = self.get_parameter('force_save_on_shutdown').get_parameter_value().bool_value
        self.max_map_age_before_save = self.get_parameter('max_map_age_before_save').get_parameter_value().double_value
        self.save_as_pgm = self.get_parameter('save_as_pgm').get_parameter_value().bool_value
        self.save_as_yaml = self.get_parameter('save_as_yaml').get_parameter_value().bool_value
        self.binary_threshold = self.get_parameter('binary_threshold').get_parameter_value().integer_value
        self.enforce_binary_values = self.get_parameter('enforce_binary_values').get_parameter_value().bool_value
        self.use_high_quality_mode = self.get_parameter('use_high_quality_mode').get_parameter_value().bool_value
        self.extra_safety_expansion = self.get_parameter('extra_safety_expansion').get_parameter_value().double_value
        self.waypoint_value = self.get_parameter('waypoint_value').get_parameter_value().integer_value
        self.get_logger().info(f"Waypoint color value: {self.waypoint_value} (gray)")
        
        self.get_logger().info(f"High quality mode enabled: {self.use_high_quality_mode}")
        self.get_logger().info(f"Include all data sources: {self.include_all_data_sources}")
        self.get_logger().info(f"Extra safety expansion: {self.extra_safety_expansion}")
        
        # Initialize map saving variables
        self.last_save_time = time.time()
        
        # Create save directory if it doesn't exist
        if self.save_combined_map:
            os.makedirs(self.save_directory, exist_ok=True)
            self.get_logger().info(f"Map saving enabled. Maps will be saved to {self.save_directory}")
            self.get_logger().info(f"Map save interval: {self.save_interval} seconds")
            self.get_logger().info(f"Map name prefix: {self.map_name_prefix}")
            self.get_logger().info(f"Add timestamp to filename: {self.add_timestamp_to_filename}")
        
        # Set up QoS profiles for all publishers and subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Set durability based on parameter - explicitly for all publishers and subscribers
        if self.use_transient_local_durability:
            qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("Using TRANSIENT_LOCAL durability for all publishers and subscribers")
        else:
            qos_profile.durability = DurabilityPolicy.VOLATILE
            self.get_logger().info("Using VOLATILE durability for all publishers and subscribers")
        
        # Create publisher for combined binary map
        self.combined_map_publisher = self.create_publisher(
            OccupancyGrid,
            self.combined_binary_topic,
            qos_profile
        )
        
        # Create publisher for map updates (if needed)
        self.combined_map_updates_publisher = self.create_publisher(
            OccupancyGrid,
            self.combined_binary_updates_topic,
            qos_profile
        )
        
        # Create publisher for unified map
        self.unified_map_publisher = self.create_publisher(
            OccupancyGrid,
            self.unified_map_topic,
            qos_profile
        )
        
        # Create subscribers for input maps with same QoS profile
        self.semantic_map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.semantic_binary_topic,
            self.semantic_map_callback,
            qos_profile
        )
        
        self.waypoint_map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.waypoint_binary_topic,
            self.waypoint_map_callback,
            qos_profile
        )
        
        # Initialize map data
        self.semantic_map = None
        self.waypoint_map = None
        self.combined_map = None
        self.map_lock = threading.Lock()
        
        # Create timer for publishing combined map
        self.map_timer = self.create_timer(1.0 / self.publish_rate, self.publish_combined_map)
        
        # Create timer for saving maps
        if self.save_combined_map:
            self.save_timer = self.create_timer(self.save_interval, self.save_map_timer_callback)
        
        # Flag to track if we have received both maps at least once
        self.have_semantic_map = False
        self.have_waypoint_map = False
        
        self.get_logger().info("Binary map combiner initialized")
        self.get_logger().info(f"Subscribing to semantic map: {self.semantic_binary_topic}")
        self.get_logger().info(f"Subscribing to waypoint map: {self.waypoint_binary_topic}")
        self.get_logger().info(f"Publishing combined map to: {self.combined_binary_topic}")
        self.get_logger().info(f"Publishing combined map updates to: {self.combined_binary_updates_topic}")
        self.get_logger().info(f"Publishing unified map to: {self.unified_map_topic}")
        self.get_logger().info(f"Prioritize waypoints: {self.prioritize_waypoints}")
        self.get_logger().info(f"Map frame ID: {self.map_frame_id}")
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"QoS durability: {qos_profile.durability}")
    
    def semantic_map_callback(self, map_msg):
        """Process incoming semantic binary map"""
        try:
            with self.map_lock:
                self.semantic_map = map_msg
                self.have_semantic_map = True
                self.get_logger().info(f"Received semantic binary map with frame_id: {map_msg.header.frame_id}")
                
                # Debug info about the semantic map
                occupied_cells = sum(1 for cell in map_msg.data if cell == self.occupied_value)
                self.get_logger().info(f"Semantic map has {occupied_cells} occupied cells out of {len(map_msg.data)} total cells")
                self.get_logger().info(f"Semantic map dimensions: {map_msg.info.width}x{map_msg.info.height}, resolution: {map_msg.info.resolution}")
        except Exception as e:
            self.get_logger().error(f"Error processing semantic map: {e}")
    
    def waypoint_map_callback(self, map_msg):
        """Process incoming waypoint binary map"""
        try:
            with self.map_lock:
                self.waypoint_map = map_msg
                self.have_waypoint_map = True
                self.get_logger().info(f"Received waypoint binary map with frame_id: {map_msg.header.frame_id}")
                
                # Debug info about the waypoint map
                occupied_cells = sum(1 for cell in map_msg.data if cell == self.occupied_value)
                self.get_logger().info(f"Waypoint map has {occupied_cells} occupied cells out of {len(map_msg.data)} total cells")
                self.get_logger().info(f"Waypoint map dimensions: {map_msg.info.width}x{map_msg.info.height}, resolution: {map_msg.info.resolution}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint map: {e}")
    
    def process_maps(self):
        """Process maps and generate combined map with three-color scheme"""
        if not self.semantic_map or not self.waypoint_map:
            self.get_logger().warn("Missing maps: Cannot process without both semantic and waypoint maps")
            return None
        
        # Create unified map with same metadata as semantic map
        self.combined_map = OccupancyGrid()
        self.combined_map.header = self.semantic_map.header
        self.combined_map.info = self.semantic_map.info
        
        # Verify maps have the same dimensions
        if (self.semantic_map.info.width != self.waypoint_map.info.width or
            self.semantic_map.info.height != self.waypoint_map.info.height):
            self.get_logger().error("Map dimension mismatch: Cannot combine maps of different sizes")
            return None
        
        # Process cell by cell with priority to obstacles
        combined_data = []
        for i in range(len(self.semantic_map.data)):
            obstacle_cell = self.semantic_map.data[i]
            waypoint_cell = self.waypoint_map.data[i]
            
            # Priority logic:
            # 1. If cell is an obstacle in semantic map (100), mark as obstacle (100)
            # 2. If cell is a waypoint in waypoint map (100), mark as waypoint (50)
            # 3. Otherwise, mark as free space (0)
            if obstacle_cell == self.occupied_value:
                # Obstacle takes priority
                combined_data.append(self.occupied_value)  # 100 = obstacle (black)
            elif waypoint_cell == self.occupied_value:
                # Waypoint if no obstacle
                combined_data.append(self.waypoint_value)  # 50 = waypoint (gray)
            else:
                # Free space if neither obstacle nor waypoint
                combined_data.append(self.free_value)      # 0 = free space (white)
        
        self.combined_map.data = combined_data
        self.get_logger().info(f"Created unified map with {len(combined_data)} cells using three-color scheme")
        
        # Publish the combined map
        self.unified_map_publisher.publish(self.combined_map)
        self.get_logger().info("Published unified map")
        
        return self.combined_map
    
    def publish_combined_map(self):
        """Publish combined binary map"""
        try:
            # Try to combine maps
            combined_map = None
            try:
                # Process maps now returns the combined map object directly
                combined_map = self.process_maps()
                # Note: process_maps already publishes to unified_map_publisher
            except Exception as e:
                self.get_logger().error(f"Error processing maps: {e}")
                combined_map = None
            
            # If processing failed, try to use a fallback approach
            if combined_map is None:
                with self.map_lock:
                    self.get_logger().warn("Failed to create combined map, falling back to individual maps")
                    
                    # Priority: combined (if available), then waypoint, then semantic
                    if self.waypoint_map and self.semantic_map:
                        # Create a very simple combined map by manual overlay
                        self.get_logger().info("Creating manual combined map from waypoint and semantic maps")
                        try:
                            combined_map = OccupancyGrid()
                            combined_map.header.stamp = self.get_clock().now().to_msg()
                            combined_map.header.frame_id = self.map_frame_id
                            
                            # Use waypoint map as base since it's our priority
                            combined_map.info = self.waypoint_map.info
                            
                            # Manual combination with three-color scheme
                            waypoint_data = np.array(self.waypoint_map.data, dtype=np.int8)
                            semantic_data = np.array(self.semantic_map.data, dtype=np.int8)
                            
                            # Three-color scheme logic:
                            # - If semantic map cell is occupied (100), use occupied value (100)
                            # - If waypoint map cell is occupied (100), use waypoint value (50)
                            # - Otherwise, use free value (0)
                            if len(waypoint_data) == len(semantic_data):
                                combined_data = np.zeros_like(waypoint_data)
                                
                                # Set obstacles (priority)
                                obstacle_mask = (semantic_data == self.occupied_value)
                                combined_data[obstacle_mask] = self.occupied_value
                                
                                # Set waypoints (where there are no obstacles)
                                waypoint_mask = (waypoint_data == self.occupied_value) & ~obstacle_mask
                                combined_data[waypoint_mask] = self.waypoint_value
                                
                                # Free space is already 0
                                
                                combined_map.data = combined_data.tolist()
                                self.get_logger().info("Successfully created manual combined map with three-color scheme")
                            else:
                                # If dimensions don't match, just use waypoint map data
                                combined_map.data = list(self.waypoint_map.data)
                                self.get_logger().warn("Dimension mismatch, using waypoint map only for fallback")
                        except Exception as e:
                            self.get_logger().error(f"Error in manual map combination: {e}")
                            combined_map = None
                        
                        # If the manual combination failed, use one of the individual maps
                        if combined_map is None:
                            if self.waypoint_map:
                                self.get_logger().info("Using waypoint map only")
                                combined_map = self.waypoint_map
                            elif self.semantic_map:
                                self.get_logger().info("Using semantic map only")
                                combined_map = self.semantic_map
            
            # Now publish the map, whether it's combined or individual
            if combined_map:
                # Store for later use in save_maps
                self.combined_map = combined_map
                
                # Publish to combined map topic
                try:
                    self.combined_map_publisher.publish(combined_map)
                    self.get_logger().debug("Published to combined map topic")
                except Exception as e:
                    self.get_logger().error(f"Error publishing to combined map topic: {e}")
                    
                # Publish to combined map updates topic
                try:
                    self.combined_map_updates_publisher.publish(combined_map)
                    self.get_logger().debug("Published to combined map updates topic")
                except Exception as e:
                    self.get_logger().error(f"Error publishing to combined map updates topic: {e}")
                
                # If this is a fallback map (process_maps failed), publish to unified map topic
                if combined_map is self.waypoint_map or combined_map is self.semantic_map:
                    try:
                        self.unified_map_publisher.publish(combined_map)
                        self.get_logger().debug("Published to unified map topic (fallback)")
                    except Exception as e:
                        self.get_logger().error(f"Error publishing to unified map topic: {e}")
                
                # Check if it's time to save maps
                current_time = time.time()
                if self.save_combined_map and (current_time - self.last_save_time) >= self.max_map_age_before_save:
                    self.save_maps()
            else:
                self.get_logger().warn("No combined map available for publishing")
                
                # If only one map is available, publish that instead
                with self.map_lock:
                    if self.waypoint_map and not self.semantic_map:
                        self.get_logger().info("Publishing waypoint map only")
                        self.unified_map_publisher.publish(self.waypoint_map)
                        self.combined_map_publisher.publish(self.waypoint_map)
                        self.combined_map = self.waypoint_map
                    elif self.semantic_map and not self.waypoint_map:
                        self.get_logger().info("Publishing semantic map only")
                        self.unified_map_publisher.publish(self.semantic_map)
                        self.combined_map_publisher.publish(self.semantic_map)
                        self.combined_map = self.semantic_map
            
        except Exception as e:
            self.get_logger().error(f"Error publishing combined map: {e}")
            # Try to publish individual maps as fallback
            with self.map_lock:
                if self.waypoint_map:
                    self.get_logger().info("Publishing waypoint map as fallback")
                    try:
                        self.unified_map_publisher.publish(self.waypoint_map)
                        self.combined_map_publisher.publish(self.waypoint_map) 
                    except Exception as publish_error:
                        self.get_logger().error(f"Failed to publish waypoint map: {publish_error}")
                elif self.semantic_map:
                    self.get_logger().info("Publishing semantic map as fallback")
                    try:
                        self.unified_map_publisher.publish(self.semantic_map)
                        self.combined_map_publisher.publish(self.semantic_map)
                    except Exception as publish_error:
                        self.get_logger().error(f"Failed to publish semantic map: {publish_error}")
    
    def save_map_timer_callback(self):
        """Callback for timer to save maps"""
        if self.save_combined_map:
            self.save_maps()
    
    def save_maps(self):
        """Save maps to disk"""
        if not self.save_combined_map or not self.combined_map:
            if not self.save_combined_map:
                self.get_logger().warn("Map saving is disabled")
            if not self.combined_map:
                self.get_logger().warn("No combined map available to save")
            return
        
        try:
            # Create filename with timestamp if requested
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = self.map_name_prefix
            if self.add_timestamp_to_filename:
                filename += timestamp
            
            # Full path for the map file
            map_path = os.path.join(self.save_directory, filename)
            
            # Count occupied and waypoint cells in the map being saved
            occupied_cells = sum(1 for cell in self.combined_map.data if cell == self.occupied_value)
            waypoint_cells = sum(1 for cell in self.combined_map.data if cell == self.waypoint_value)
            total_cells = len(self.combined_map.data)
            
            # Calculate percentages
            percent_occupied = (occupied_cells / total_cells) * 100 if total_cells > 0 else 0
            percent_waypoints = (waypoint_cells / total_cells) * 100 if total_cells > 0 else 0
            
            self.get_logger().info(f"Saving unified map with:")
            self.get_logger().info(f"  - {occupied_cells} obstacle cells ({percent_occupied:.2f}% of map)")
            self.get_logger().info(f"  - {waypoint_cells} waypoint cells ({percent_waypoints:.2f}% of map)")
            self.get_logger().info(f"Map dimensions: {self.combined_map.info.width}x{self.combined_map.info.height}, resolution: {self.combined_map.info.resolution}")
            
            # Save as PGM format
            if self.save_as_pgm:
                self.save_map_to_pgm(self.combined_map, f"{map_path}.pgm")
                self.get_logger().info(f"Saved unified map to {map_path}.pgm")
            
            # Save metadata as YAML
            if self.save_as_yaml:
                self.save_map_metadata(self.combined_map, f"{map_path}.yaml")
                self.get_logger().info(f"Saved unified map metadata to {map_path}.yaml")
            
            # Update last save time
            self.last_save_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error saving maps: {str(e)}")
    
    def save_map_to_pgm(self, map_msg, filename):
        """Save map data as PGM file with three-color scheme"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            # Get map dimensions
            width = map_msg.info.width
            height = map_msg.info.height
            
            # Open file for writing binary data
            with open(filename, 'wb') as f:
                # Write PGM header
                header = f"P5\n{width} {height}\n255\n"
                f.write(header.encode())
                
                # Process map data - convert from 0-100 scale to 0-255 scale
                # In the unified map:
                # - 100 = obstacle (black, 0)
                # - 50 = waypoint (gray, 127)
                # - 0 = free space (white, 255)
                map_data = []
                for cell in map_msg.data:
                    if cell == 100:  # Obstacle
                        map_data.append(0)  # Black
                    elif cell == 50:  # Waypoint
                        map_data.append(127)  # Gray
                    else:  # Free space or unknown
                        map_data.append(255)  # White
                
                # Write binary data
                f.write(bytes(map_data))
            
            self.get_logger().info(f"Saved map to {filename} with three-color scheme (0=obstacle, 127=waypoint, 255=free)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error saving map to PGM: {str(e)}")
            return False
    
    def save_map_metadata(self, map_msg, filename):
        """Save map metadata as YAML file for three-color map"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            # Create PGM filename (just the basename)
            pgm_filename = os.path.basename(filename.replace('.yaml', '.pgm'))
            
            # Write YAML metadata
            with open(filename, 'w') as f:
                f.write(f"image: {pgm_filename}\n")
                f.write(f"resolution: {map_msg.info.resolution}\n")
                f.write(f"origin: [{map_msg.info.origin.position.x}, {map_msg.info.origin.position.y}, 0.0]\n")
                f.write(f"negate: 0\n")
                # Set thresholds for three-color interpretation:
                # - Values below 0.25 (close to 0/black) are considered occupied
                # - Values above 0.75 (close to 255/white) are considered free
                # - Values in between (around 127/gray) are considered waypoints
                f.write(f"occupied_thresh: 0.25\n")  # Values below this are occupied (black)
                f.write(f"free_thresh: 0.75\n")      # Values above this are free (white)
                f.write(f"mode: scale\n")
                f.write(f"# Three-color scheme:\n")
                f.write(f"# - Black (0) represents obstacles\n")
                f.write(f"# - Gray (127) represents waypoints\n")
                f.write(f"# - White (255) represents free space\n")
            
            self.get_logger().info(f"Saved map metadata to {filename} with thresholds for three-color scheme")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error saving map metadata: {str(e)}")
            return False
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down binary map combiner")
        
        # Save map on shutdown if requested
        if self.force_save_on_shutdown and self.save_combined_map and self.combined_map:
            self.get_logger().info("Saving final map before shutdown...")
            self.save_maps()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    binary_map_combiner = BinaryMapCombiner()
    
    try:
        rclpy.spin(binary_map_combiner)
    except KeyboardInterrupt:
        pass
    finally:
        binary_map_combiner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()