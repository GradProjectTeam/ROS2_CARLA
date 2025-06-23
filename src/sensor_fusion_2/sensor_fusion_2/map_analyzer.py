#!/usr/bin/env python3
"""
Map Analyzer Node

This node subscribes to the unified map topic and analyzes the map contents,
printing detailed statistics about what objects are included in the map.
It helps diagnose what's being saved to the unified map.

Author: Claude
"""

import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
from datetime import datetime

class MapAnalyzer(Node):
    def __init__(self):
        super().__init__('map_analyzer')
        
        # Declare parameters
        self.declare_parameter('unified_map_topic', '/unified_map')
        self.declare_parameter('semantic_map_topic', '/semantic_costmap/binary')
        self.declare_parameter('waypoint_map_topic', '/waypoint_map/binary')
        self.declare_parameter('analysis_interval', 5.0)  # seconds
        self.declare_parameter('save_visualizations', True)
        self.declare_parameter('visualization_path', os.path.expanduser('~/Robot_local/map_analysis'))
        
        # Get parameters
        self.unified_map_topic = self.get_parameter('unified_map_topic').get_parameter_value().string_value
        self.semantic_map_topic = self.get_parameter('semantic_map_topic').get_parameter_value().string_value
        self.waypoint_map_topic = self.get_parameter('waypoint_map_topic').get_parameter_value().string_value
        self.analysis_interval = self.get_parameter('analysis_interval').get_parameter_value().double_value
        self.save_visualizations = self.get_parameter('save_visualizations').get_parameter_value().bool_value
        self.visualization_path = self.get_parameter('visualization_path').get_parameter_value().string_value
        
        # Create visualization directory if needed
        if self.save_visualizations:
            os.makedirs(self.visualization_path, exist_ok=True)
            self.get_logger().info(f"Visualizations will be saved to {self.visualization_path}")
        
        # Initialize map data
        self.unified_map = None
        self.semantic_map = None
        self.waypoint_map = None
        self.last_analysis_time = time.time()
        
        # Create subscribers
        self.unified_map_sub = self.create_subscription(
            OccupancyGrid,
            self.unified_map_topic,
            self.unified_map_callback,
            10
        )
        
        self.semantic_map_sub = self.create_subscription(
            OccupancyGrid,
            self.semantic_map_topic,
            self.semantic_map_callback,
            10
        )
        
        self.waypoint_map_sub = self.create_subscription(
            OccupancyGrid,
            self.waypoint_map_topic,
            self.waypoint_map_callback,
            10
        )
        
        # Create timer for periodic analysis
        self.analysis_timer = self.create_timer(self.analysis_interval, self.analyze_maps)
        
        self.get_logger().info(f"Map Analyzer started")
        self.get_logger().info(f"Subscribing to unified map: {self.unified_map_topic}")
        self.get_logger().info(f"Subscribing to semantic map: {self.semantic_map_topic}")
        self.get_logger().info(f"Subscribing to waypoint map: {self.waypoint_map_topic}")
    
    def unified_map_callback(self, msg):
        """Process incoming unified map"""
        self.unified_map = msg
        self.get_logger().debug("Received unified map")
    
    def semantic_map_callback(self, msg):
        """Process incoming semantic map"""
        self.semantic_map = msg
        self.get_logger().debug("Received semantic map")
    
    def waypoint_map_callback(self, msg):
        """Process incoming waypoint map"""
        self.waypoint_map = msg
        self.get_logger().debug("Received waypoint map")
    
    def analyze_maps(self):
        """Analyze map contents and print statistics"""
        # Check if we have all maps
        if not self.unified_map:
            self.get_logger().warn("No unified map received yet")
            return
        
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("UNIFIED MAP ANALYSIS")
        self.get_logger().info("="*50)
        
        # Basic unified map statistics
        unified_data = np.array(self.unified_map.data, dtype=np.int8)
        unified_width = self.unified_map.info.width
        unified_height = self.unified_map.info.height
        unified_resolution = self.unified_map.info.resolution
        
        unified_occupied = np.sum(unified_data > 0)
        unified_free = np.sum(unified_data == 0)
        unified_unknown = np.sum(unified_data < 0)
        unified_total = len(unified_data)
        
        self.get_logger().info(f"Unified Map Stats:")
        self.get_logger().info(f"  - Dimensions: {unified_width}x{unified_height} cells ({unified_width*unified_resolution:.1f}x{unified_height*unified_resolution:.1f} meters)")
        self.get_logger().info(f"  - Resolution: {unified_resolution} meters/cell")
        self.get_logger().info(f"  - Occupied cells: {unified_occupied} ({unified_occupied/unified_total*100:.1f}%)")
        self.get_logger().info(f"  - Free cells: {unified_free} ({unified_free/unified_total*100:.1f}%)")
        self.get_logger().info(f"  - Unknown cells: {unified_unknown} ({unified_unknown/unified_total*100:.1f}%)")
        
        # Compare with source maps if available
        if self.semantic_map and self.waypoint_map:
            # Check if maps have compatible dimensions
            if (self.semantic_map.info.width == self.waypoint_map.info.width and
                self.semantic_map.info.height == self.waypoint_map.info.height and
                self.semantic_map.info.resolution == self.waypoint_map.info.resolution):
                
                semantic_data = np.array(self.semantic_map.data, dtype=np.int8)
                waypoint_data = np.array(self.waypoint_map.data, dtype=np.int8)
                
                # Count occupied cells in each map
                semantic_occupied = np.sum(semantic_data > 0)
                waypoint_occupied = np.sum(waypoint_data > 0)
                
                # Find cells that are exclusive to each map or shared
                semantic_only = np.sum((semantic_data > 0) & (waypoint_data == 0))
                waypoint_only = np.sum((waypoint_data > 0) & (semantic_data == 0))
                both_occupied = np.sum((semantic_data > 0) & (waypoint_data > 0))
                
                self.get_logger().info("\nSource Maps Comparison:")
                self.get_logger().info(f"  - Semantic map occupied cells: {semantic_occupied}")
                self.get_logger().info(f"  - Waypoint map occupied cells: {waypoint_occupied}")
                self.get_logger().info(f"  - Cells occupied only in semantic map: {semantic_only}")
                self.get_logger().info(f"  - Cells occupied only in waypoint map: {waypoint_only}")
                self.get_logger().info(f"  - Cells occupied in both maps: {both_occupied}")
                
                # Check what's in unified map
                unified_has_semantic = np.sum((unified_data > 0) & (semantic_data > 0))
                unified_has_waypoints = np.sum((unified_data > 0) & (waypoint_data > 0))
                unified_has_both = np.sum((unified_data > 0) & (semantic_data > 0) & (waypoint_data > 0))
                unified_missing_semantic = np.sum((semantic_data > 0) & (unified_data == 0))
                unified_missing_waypoints = np.sum((waypoint_data > 0) & (unified_data == 0))
                
                self.get_logger().info("\nUnified Map Content Analysis:")
                self.get_logger().info(f"  - Unified map includes {unified_has_semantic} semantic cells ({unified_has_semantic/semantic_occupied*100:.1f}% of semantic cells)")
                self.get_logger().info(f"  - Unified map includes {unified_has_waypoints} waypoint cells ({unified_has_waypoints/waypoint_occupied*100:.1f}% of waypoint cells)")
                self.get_logger().info(f"  - Unified map includes {unified_has_both} cells that are in both source maps")
                
                if unified_missing_semantic > 0:
                    self.get_logger().warn(f"  - WARNING: Unified map is missing {unified_missing_semantic} semantic cells ({unified_missing_semantic/semantic_occupied*100:.1f}% of semantic cells)")
                
                if unified_missing_waypoints > 0:
                    self.get_logger().warn(f"  - WARNING: Unified map is missing {unified_missing_waypoints} waypoint cells ({unified_missing_waypoints/waypoint_occupied*100:.1f}% of waypoint cells)")
            else:
                self.get_logger().error("Maps have incompatible dimensions - can't perform detailed comparison")
                self.get_logger().info(f"  - Semantic map: {self.semantic_map.info.width}x{self.semantic_map.info.height}, resolution: {self.semantic_map.info.resolution}")
                self.get_logger().info(f"  - Waypoint map: {self.waypoint_map.info.width}x{self.waypoint_map.info.height}, resolution: {self.waypoint_map.info.resolution}")
        else:
            if not self.semantic_map:
                self.get_logger().warn("No semantic map received yet - can't compare")
            if not self.waypoint_map:
                self.get_logger().warn("No waypoint map received yet - can't compare")
        
        # Save visualizations if requested
        if self.save_visualizations:
            self.visualize_maps()
        
        self.get_logger().info("="*50)
    
    def visualize_maps(self):
        """Create visualizations of the maps"""
        if not self.unified_map:
            return
            
        try:
            # Create timestamp for filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Prepare unified map for visualization
            unified_data = np.array(self.unified_map.data, dtype=np.int8)
            unified_map_2d = unified_data.reshape((self.unified_map.info.height, self.unified_map.info.width))
            
            # Create figure
            fig = plt.figure(figsize=(15, 10))
            
            # Plot unified map
            ax1 = fig.add_subplot(1, 3, 1)
            ax1.imshow(unified_map_2d, cmap='gray_r', vmin=-1, vmax=100)
            ax1.set_title('Unified Map')
            
            # Plot semantic map if available
            if self.semantic_map:
                semantic_data = np.array(self.semantic_map.data, dtype=np.int8)
                semantic_map_2d = semantic_data.reshape((self.semantic_map.info.height, self.semantic_map.info.width))
                
                ax2 = fig.add_subplot(1, 3, 2)
                ax2.imshow(semantic_map_2d, cmap='gray_r', vmin=-1, vmax=100)
                ax2.set_title('Semantic Map (Obstacles)')
            
            # Plot waypoint map if available
            if self.waypoint_map:
                waypoint_data = np.array(self.waypoint_map.data, dtype=np.int8)
                waypoint_map_2d = waypoint_data.reshape((self.waypoint_map.info.height, self.waypoint_map.info.width))
                
                ax3 = fig.add_subplot(1, 3, 3)
                ax3.imshow(waypoint_map_2d, cmap='gray_r', vmin=-1, vmax=100)
                ax3.set_title('Waypoint Map')
            
            # Add timestamp and other info to figure
            plt.suptitle(f'Map Analysis - {timestamp}', fontsize=16)
            plt.figtext(0.5, 0.01, f"Resolution: {self.unified_map.info.resolution} m/cell", ha='center')
            
            # Save figure
            plt.tight_layout()
            save_path = os.path.join(self.visualization_path, f"map_analysis_{timestamp}.png")
            plt.savefig(save_path)
            plt.close(fig)
            
            self.get_logger().info(f"Saved map visualization to {save_path}")
            
        except Exception as e:
            self.get_logger().error(f"Error creating visualization: {str(e)}")
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down map analyzer")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    map_analyzer = MapAnalyzer()
    
    try:
        rclpy.spin(map_analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        map_analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 