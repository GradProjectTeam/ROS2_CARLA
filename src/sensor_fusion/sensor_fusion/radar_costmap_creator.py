#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class RadarCostmapCreator(Node):
    """
    Converts radar cluster data from enhanced_radar_visualizer to a costmap layer for navigation.
    
    This node:
    1. Subscribes to MarkerArray from enhanced_radar_visualizer
    2. Extracts points and velocity information
    3. Creates a dynamic costmap that assigns higher costs to moving objects
    4. Publishes the costmap for use with Nav2
    """
    def __init__(self):
        super().__init__('radar_costmap_creator')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.1)       # meters per cell
        self.declare_parameter('map_width', 1000)           # cells (100m x 100m with 0.1m resolution)
        self.declare_parameter('map_height', 1000)          # cells
        self.declare_parameter('map_origin_x', -50.0)       # meters
        self.declare_parameter('map_origin_y', -50.0)       # meters
        self.declare_parameter('publish_rate', 5.0)         # Hz
        self.declare_parameter('velocity_impact_factor', 20.0)  # Cost increase per m/s
        self.declare_parameter('max_marker_age', 1.0)       # seconds
        self.declare_parameter('radar_weight', 0.7)         # Weight for radar data in fusion
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.velocity_impact_factor = self.get_parameter('velocity_impact_factor').value
        self.max_marker_age = self.get_parameter('max_marker_age').value
        self.radar_weight = self.get_parameter('radar_weight').value
        
        # Initialize costmap layer
        self.radar_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Track received markers with timestamps
        self.markers_with_timestamps = {}  # {marker_id: (marker, timestamp)}
        
        # Publishers
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/radar_costmap', 
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5
            )
        )
        
        # Debug publisher to visualize the costmap
        self.debug_costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/radar_costmap_debug', 
            10
        )
        
        # Subscribers - Updated to specify it's from enhanced_radar_visualizer
        self.create_subscription(
            MarkerArray,
            '/radar/markers',  # From enhanced_radar_visualizer
            self.marker_callback,
            10
        )
        
        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create timers
        self.create_timer(1.0/self.publish_rate, self.publish_costmap)
        self.create_timer(1.0, self.cleanup_old_markers)
        
        self.get_logger().info('Radar Costmap Creator initialized')
        self.get_logger().info('Listening for radar markers from enhanced_radar_visualizer...')
    
    def marker_callback(self, msg):
        """Process radar markers from enhanced_radar_visualizer"""
        current_time = self.get_clock().now()
        
        marker_count = len(msg.markers)
        if marker_count > 0:
            self.get_logger().debug(f'Received {marker_count} radar markers')
        
        for marker in msg.markers:
            # Store only point and arrow markers with timestamp
            if marker.type in [Marker.SPHERE, Marker.ARROW]:
                self.markers_with_timestamps[f"{marker.ns}_{marker.id}"] = (marker, current_time)
    
    def cleanup_old_markers(self):
        """Remove markers that are older than max_marker_age"""
        current_time = self.get_clock().now()
        
        # Identify old markers
        old_markers = []
        for marker_id, (_, timestamp) in self.markers_with_timestamps.items():
            if (current_time - timestamp).nanoseconds / 1e9 > self.max_marker_age:
                old_markers.append(marker_id)
        
        # Remove old markers
        for marker_id in old_markers:
            del self.markers_with_timestamps[marker_id]
            
        if old_markers:
            self.get_logger().debug(f'Removed {len(old_markers)} old markers')
    
    def update_costmap(self):
        """Update costmap based on current radar markers"""
        # Reset costmap
        self.radar_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Process point and velocity data
        point_markers = {}
        velocity_markers = {}
        
        # First, categorize markers by type
        for marker_id, (marker, _) in self.markers_with_timestamps.items():
            if marker.type == Marker.SPHERE and marker.ns == "radar_points":
                point_markers[marker_id] = marker
            elif marker.type == Marker.ARROW and marker.ns == "velocity_vectors":
                velocity_markers[marker_id] = marker
        
        # Then update costmap with point and velocity information
        processed_points = 0
        
        for marker_id, marker in point_markers.items():
            # Get point position
            x, y, z = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
            
            # Convert to grid coordinates
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)
            
            # Skip if out of bounds
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                continue
            
            # Calculate base cost (50 is default occupied)
            base_cost = 50
            
            # Find corresponding velocity vector if any
            # Look for an arrow marker with the same ID number
            vel_marker_id = marker_id.replace("radar_points", "velocity_vectors")
            velocity_magnitude = 0.0
            
            if vel_marker_id in velocity_markers and len(velocity_markers[vel_marker_id].points) >= 2:
                # Get velocity vector start and end points
                start = velocity_markers[vel_marker_id].points[0]
                end = velocity_markers[vel_marker_id].points[1]
                
                # Calculate velocity magnitude (length of arrow)
                velocity_magnitude = math.sqrt(
                    (end.x - start.x)**2 + 
                    (end.y - start.y)**2 + 
                    (end.z - start.z)**2
                )
                
                # Use color to determine approaching vs receding (red = approaching)
                if velocity_markers[vel_marker_id].color.r > velocity_markers[vel_marker_id].color.b:
                    # Approaching (red), higher cost
                    velocity_magnitude *= 1.5
            
            # Increase cost based on velocity
            dynamic_cost = min(int(base_cost + velocity_magnitude * self.velocity_impact_factor), 100)
            
            # Apply cost to grid and surrounding cells (inflation)
            inflation_radius = int(1.0 / self.map_resolution)  # 1 meter radius
            
            for dx in range(-inflation_radius, inflation_radius + 1):
                for dy in range(-inflation_radius, inflation_radius + 1):
                    nx, ny = grid_x + dx, grid_y + dy
                    
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        # Distance-based cost decay
                        distance = math.sqrt(dx**2 + dy**2)
                        if distance <= inflation_radius:
                            decay_factor = 1.0 - (distance / inflation_radius)
                            cell_cost = int(dynamic_cost * decay_factor)
                            
                            # Only update if new cost is higher than existing
                            if cell_cost > self.radar_costmap[ny, nx]:
                                self.radar_costmap[ny, nx] = cell_cost
            
            processed_points += 1
            
        if processed_points > 0:
            self.get_logger().debug(f'Updated costmap with {processed_points} radar points')
    
    def publish_costmap(self):
        """Publish the radar costmap for navigation and visualization"""
        # Update costmap
        self.update_costmap()
        
        # Create occupancy grid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = 'map'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height = self.map_height
        costmap_msg.info.origin.position.x = self.map_origin_x
        costmap_msg.info.origin.position.y = self.map_origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        
        # Convert numpy array to 1D list
        costmap_msg.data = self.radar_costmap.flatten().tolist()
        
        # Publish costmap
        self.costmap_publisher.publish(costmap_msg)
        self.debug_costmap_publisher.publish(costmap_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = RadarCostmapCreator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 