#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
import message_filters

class SpaceDetectionNode(Node):
    def __init__(self):
        super().__init__('space_detection_node')
        
        # Detection parameters
        self.min_height = -0.2  # meters from ground
        self.max_height = 2.0   # meters from ground
        self.car_min_points = 10  # minimum points to consider as vehicle
        self.road_grid_size = 0.5  # meters per cell
        
        # Publishers
        self.free_space_pub = self.create_publisher(
            MarkerArray,
            '/detection/free_space',
            10
        )
        self.vehicle_markers_pub = self.create_publisher(
            MarkerArray,
            '/detection/vehicles',
            10
        )
        
        # Subscribers
        self.lidar_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            '/carla/lidar'
        )
        self.radar_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            '/carla/radar'
        )
        
        # Synchronize LiDAR and radar data
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.radar_sub],
            10,
            0.1
        )
        self.ts.registerCallback(self.sensor_callback)
        
        self.get_logger().info('Space Detection Node initialized')

    def detect_vehicles(self, lidar_points, radar_points):
        """Detect vehicles using both LiDAR and radar data"""
        vehicles = []
        
        # Cluster LiDAR points
        clusters = self.cluster_points(lidar_points)
        
        # Match with radar data for velocity
        for cluster in clusters:
            if len(cluster) > self.car_min_points:
                # Calculate cluster centroid
                centroid = np.mean(cluster, axis=0)
                
                # Find nearest radar point
                if len(radar_points) > 0:
                    distances = np.linalg.norm(radar_points[:, :3] - centroid, axis=1)
                    nearest_radar = radar_points[np.argmin(distances)]
                    
                    vehicles.append({
                        'position': centroid,
                        'velocity': nearest_radar[3:6] if len(nearest_radar) >= 6 else np.zeros(3),
                        'points': cluster
                    })
        
        return vehicles

    def detect_free_space(self, lidar_points):
        """Detect free space on the road"""
        # Filter ground-level points
        road_points = lidar_points[
            (lidar_points[:, 2] >= self.min_height) & 
            (lidar_points[:, 2] <= self.max_height)
        ]
        
        # Create grid cells
        x_cells = np.floor(road_points[:, 0] / self.road_grid_size)
        y_cells = np.floor(road_points[:, 1] / self.road_grid_size)
        
        # Identify free cells
        occupied_cells = set(zip(x_cells, y_cells))
        
        return occupied_cells

    def cluster_points(self, points, cluster_tolerance=0.5):
        """Simple clustering based on distance"""
        clusters = []
        processed = set()
        
        for i, point in enumerate(points):
            if i in processed:
                continue
                
            cluster = []
            queue = [i]
            
            while queue:
                idx = queue.pop(0)
                if idx not in processed:
                    processed.add(idx)
                    cluster.append(points[idx])
                    
                    # Find neighbors
                    distances = np.linalg.norm(points - points[idx], axis=1)
                    neighbors = np.where(distances < cluster_tolerance)[0]
                    
                    for neighbor in neighbors:
                        if neighbor not in processed:
                            queue.append(neighbor)
            
            if len(cluster) > 0:
                clusters.append(np.array(cluster))
        
        return clusters

    def publish_detections(self, vehicles, free_space, timestamp):
        """Publish visualization markers"""
        # Vehicle markers
        vehicle_markers = MarkerArray()
        for i, vehicle in enumerate(vehicles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = vehicle['position'][0]
            marker.pose.position.y = vehicle['position'][1]
            marker.pose.position.z = vehicle['position'][2]
            
            # Set size
            marker.scale.x = 4.0  # Length
            marker.scale.y = 2.0  # Width
            marker.scale.z = 1.5  # Height
            
            # Set color (red for vehicles)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            vehicle_markers.markers.append(marker)
        
        # Free space markers
        free_space_markers = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set points
        for cell in free_space:
            point = Point()
            point.x = cell[0] * self.road_grid_size
            point.y = cell[1] * self.road_grid_size
            point.z = 0
            marker.points.append(point)
        
        # Set appearance
        marker.scale.x = self.road_grid_size
        marker.scale.y = self.road_grid_size
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        
        free_space_markers.markers.append(marker)
        
        # Publish markers
        self.vehicle_markers_pub.publish(vehicle_markers)
        self.free_space_pub.publish(free_space_markers)

    def sensor_callback(self, lidar_msg, radar_msg):
        """Process incoming sensor data"""
        try:
            # Convert LiDAR points
            lidar_points = []
            for data in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True):
                lidar_points.append([data[0], data[1], data[2]])
            lidar_points = np.array(lidar_points)
            
            # Convert radar points (including velocity)
            radar_points = []
            for data in pc2.read_points(radar_msg, field_names=("x", "y", "z", "vx", "vy", "vz"), skip_nans=True):
                radar_points.append([data[0], data[1], data[2], data[3], data[4], data[5]])
            radar_points = np.array(radar_points)
            
            # Detect vehicles and free space
            vehicles = self.detect_vehicles(lidar_points, radar_points)
            free_space = self.detect_free_space(lidar_points)
            
            # Publish results
            self.publish_detections(vehicles, free_space, lidar_msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SpaceDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 