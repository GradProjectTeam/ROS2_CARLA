#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np
import math

class FusionMapCreator(Node):
    def __init__(self):
        super().__init__('fusion_map_creator')
        
        # Map parameters
        self.map_resolution = 0.1
        self.map_width = 3000
        self.map_height = 3000
        self.map_origin_x = -150.0
        self.map_origin_y = -150.0
        
        # Separate layers for fusion
        self.radar_layer = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.lidar_layer = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.fusion_map = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Confidence weights for fusion
        self.radar_weight = 0.4  # Lower weight for radar
        self.lidar_weight = 0.6  # Higher weight for lidar's precision
        
        # Dynamic object tracking
        self.dynamic_objects = {}  # Track moving objects
        
        # Publishers
        self.fusion_pub = self.create_publisher(OccupancyGrid, '/fusion_map', 10)
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/radar_map', self.radar_callback, 10)
        self.create_subscription(PointCloud2, '/lidar_points', self.lidar_callback, 10)
        
        # Timer for map fusion and publishing
        self.create_timer(0.1, self.fuse_and_publish)

    def radar_callback(self, msg):
        """Process radar data with focus on dynamic objects"""
        radar_data = np.array(msg.data).reshape(self.map_height, self.map_width)
        
        # Update radar layer
        self.radar_layer = radar_data
        
        # Track high-intensity points as potentially dynamic objects
        dynamic_threshold = 80
        dynamic_points = np.where(radar_data > dynamic_threshold)
        
        for y, x in zip(dynamic_points[0], dynamic_points[1]):
            world_x = x * self.map_resolution + self.map_origin_x
            world_y = y * self.map_resolution + self.map_origin_y
            self.dynamic_objects[(world_x, world_y)] = self.get_clock().now()

    def lidar_callback(self, msg):
        """Process lidar data with focus on static obstacles"""
        # Convert PointCloud2 to occupancy data
        # (Implementation depends on your lidar data format)
        self.lidar_layer = self.process_lidar_points(msg)

    def fuse_and_publish(self):
        """Fuse radar and lidar data with intelligent weighting"""
        # Clear old dynamic objects (older than 1 second)
        current_time = self.get_clock().now()
        self.dynamic_objects = {
            k: v for k, v in self.dynamic_objects.items() 
            if (current_time - v).nanoseconds < 1e9
        }
        
        # Basic fusion with weighted average
        self.fusion_map = np.zeros_like(self.radar_layer)
        
        for y in range(self.map_height):
            for x in range(self.map_width):
                world_x = x * self.map_resolution + self.map_origin_x
                world_y = y * self.map_resolution + self.map_origin_y
                
                # Check if point is near a dynamic object
                is_dynamic = any(
                    math.sqrt((world_x - dx)**2 + (world_y - dy)**2) < 1.0
                    for dx, dy in self.dynamic_objects.keys()
                )
                
                if is_dynamic:
                    # Prefer radar data for dynamic objects
                    self.fusion_map[y, x] = self.radar_layer[y, x]
                else:
                    # Weighted average for static objects
                    self.fusion_map[y, x] = int(
                        self.radar_layer[y, x] * self.radar_weight +
                        self.lidar_layer[y, x] * self.lidar_weight
                    )
        
        # Create cost map for path planning
        self.publish_fusion_map()

    def publish_fusion_map(self):
        """Publish the fused occupancy grid"""
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        
        msg.data = self.fusion_map.flatten().tolist()
        self.fusion_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    fusion_map_creator = FusionMapCreator()
    rclpy.spin(fusion_map_creator)
    fusion_map_creator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 