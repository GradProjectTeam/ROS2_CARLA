#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2 as pc2

class LidarSlamNode(Node):
    def __init__(self):
        super().__init__('lidar_slam_node')
        
        # Map parameters optimized for path planning
        self.map_resolution = 0.5  # meters per cell (coarser for planning)
        self.map_width = 200  # cells (100m x 100m area)
        self.map_height = 200  # cells
        self.map_origin = [-50.0, -50.0, 0.0]  # meters
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        self.height_threshold = 0.5  # meters above ground to detect obstacles
        
        # Publishers
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            '/slam/map',
            30
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            30
        )
        
        self.get_logger().info('LiDAR SLAM Node initialized')

    def update_map(self, points):
        """Update map with obstacles for path planning"""
        try:
            for point in points:
                # Convert point to map coordinates
                x = int((point[0] - self.map_origin[0]) / self.map_resolution)
                y = int((point[1] - self.map_origin[1]) / self.map_resolution)
                
                # Check if point is within map bounds
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    if abs(point[2]) > self.height_threshold:
                        self.map_data[y, x] = 100  # Mark as obstacle
                    else:
                        self.map_data[y, x] = 0  # Mark as free space
        except Exception as e:
            self.get_logger().error(f'Error updating map: {str(e)}')

    def publish_map(self, timestamp):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = timestamp
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin[0]
        grid_msg.info.origin.position.y = self.map_origin[1]
        grid_msg.info.origin.position.z = self.map_origin[2]
        grid_msg.data = self.map_data.flatten().tolist()
        self.map_publisher.publish(grid_msg)

    def lidar_callback(self, msg):
        """Process incoming LiDAR data"""
        try:
            # Convert PointCloud2 to numpy array
            points_list = []
            for data in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([data[0], data[1], data[2]])
            points = np.array(points_list)
            
            if len(points) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
            
            # Update map with new points
            self.update_map(points)
            
            # Publish updated map
            self.publish_map(msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()