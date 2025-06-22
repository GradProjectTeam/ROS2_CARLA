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
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('prioritize_waypoints', True)
        self.declare_parameter('use_transient_local_durability', True)
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.semantic_binary_topic = self.get_parameter('semantic_binary_topic').get_parameter_value().string_value
        self.waypoint_binary_topic = self.get_parameter('waypoint_binary_topic').get_parameter_value().string_value
        self.combined_binary_topic = self.get_parameter('combined_binary_topic').get_parameter_value().string_value
        self.occupied_value = self.get_parameter('occupied_value').get_parameter_value().integer_value
        self.free_value = self.get_parameter('free_value').get_parameter_value().integer_value
        self.prioritize_waypoints = self.get_parameter('prioritize_waypoints').get_parameter_value().bool_value
        self.use_transient_local_durability = self.get_parameter('use_transient_local_durability').get_parameter_value().bool_value
        
        # Set up QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        if self.use_transient_local_durability:
            qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Create publisher for combined binary map
        self.combined_map_publisher = self.create_publisher(
            OccupancyGrid,
            self.combined_binary_topic,
            qos_profile
        )
        
        # Create subscribers for input maps
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
        
        # Flag to track if we have received both maps at least once
        self.have_semantic_map = False
        self.have_waypoint_map = False
        
        self.get_logger().info("Binary map combiner initialized")
        self.get_logger().info(f"Subscribing to semantic map: {self.semantic_binary_topic}")
        self.get_logger().info(f"Subscribing to waypoint map: {self.waypoint_binary_topic}")
        self.get_logger().info(f"Publishing combined map to: {self.combined_binary_topic}")
    
    def semantic_map_callback(self, map_msg):
        """Process incoming semantic binary map"""
        try:
            with self.map_lock:
                self.semantic_map = map_msg
                self.have_semantic_map = True
                self.get_logger().debug("Received semantic binary map")
        except Exception as e:
            self.get_logger().error(f"Error processing semantic map: {e}")
    
    def waypoint_map_callback(self, map_msg):
        """Process incoming waypoint binary map"""
        try:
            with self.map_lock:
                self.waypoint_map = map_msg
                self.have_waypoint_map = True
                self.get_logger().debug("Received waypoint binary map")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint map: {e}")
    
    def combine_maps(self):
        """Combine semantic and waypoint binary maps"""
        try:
            with self.map_lock:
                # Check if we have both maps
                if not self.have_semantic_map or not self.have_waypoint_map:
                    return None
                
                # Use the most recent map as the base for metadata
                base_map = self.semantic_map if self.semantic_map.header.stamp > self.waypoint_map.header.stamp else self.waypoint_map
                
                # Create a new map with the same metadata
                combined_map = OccupancyGrid()
                combined_map.header.stamp = self.get_clock().now().to_msg()
                combined_map.header.frame_id = self.map_frame_id
                combined_map.info = base_map.info
                
                # Check if maps have compatible dimensions
                if self.semantic_map.info.width != self.waypoint_map.info.width or \
                   self.semantic_map.info.height != self.waypoint_map.info.height or \
                   self.semantic_map.info.resolution != self.waypoint_map.info.resolution:
                    self.get_logger().warn("Maps have incompatible dimensions, using semantic map only")
                    combined_map.data = list(self.semantic_map.data)
                    return combined_map
                
                # Convert map data to numpy arrays for easier processing
                semantic_data = np.array(self.semantic_map.data, dtype=np.int8)
                waypoint_data = np.array(self.waypoint_map.data, dtype=np.int8)
                
                # Combine maps
                if self.prioritize_waypoints:
                    # Where waypoint map has occupied cells, use those
                    # Otherwise, use semantic map data
                    combined_data = np.copy(semantic_data)
                    waypoint_occupied = waypoint_data == self.occupied_value
                    combined_data[waypoint_occupied] = self.occupied_value
                else:
                    # Combine maps with OR operation (if either map has occupied cell, mark as occupied)
                    semantic_occupied = semantic_data == self.occupied_value
                    waypoint_occupied = waypoint_data == self.occupied_value
                    combined_data = np.full_like(semantic_data, self.free_value)
                    combined_data[semantic_occupied | waypoint_occupied] = self.occupied_value
                
                # Set combined map data
                combined_map.data = combined_data.tolist()
                return combined_map
                
        except Exception as e:
            self.get_logger().error(f"Error combining maps: {e}")
            return None
    
    def publish_combined_map(self):
        """Publish combined binary map"""
        try:
            # Combine maps
            combined_map = self.combine_maps()
            
            # Publish if we have a combined map
            if combined_map:
                self.combined_map_publisher.publish(combined_map)
                self.get_logger().debug("Published combined binary map")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing combined map: {e}")
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down binary map combiner")
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