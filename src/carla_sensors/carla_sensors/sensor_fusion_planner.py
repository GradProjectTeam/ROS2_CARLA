#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math
import struct
from sensor_msgs_py import point_cloud2

class SensorFusionPlanner(Node):
    def __init__(self):
        super().__init__('sensor_fusion_planner')
        
        # Publishers
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.grid_publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.fusion_publisher = self.create_publisher(MarkerArray, '/fusion_visualization', 10)
        
        # Subscribers
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            '/lidar/clusters',  # Preprocessed LiDAR clusters
            self.lidar_callback,
            10)
        self.radar_subscription = self.create_subscription(
            PointCloud2,
            '/radar/clusters',  # Preprocessed radar clusters
            self.radar_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
            
        # Data storage
        self.lidar_clusters = None  # After noise filtering, RANSAC, and clustering
        self.radar_clusters = None  # After clutter removal, MTI, and clustering
        self.imu_orientation = None # Pitch, yaw, roll
        
        # Initialize tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Path planning parameters
        self.grid_resolution = 0.5  # meters per cell
        self.grid_width = 100  # cells
        self.grid_height = 100  # cells
        
        # Create timer for fusion processing
        self.create_timer(0.1, self.fuse_sensor_data)  # 10Hz processing

        # Add timestamp tracking
        self.last_lidar_time = None
        self.last_radar_time = None
        self.last_imu_time = None
        
        # Add maximum time difference threshold
        self.max_time_diff = 0.1  # 100ms threshold

    def check_data_sync(self):
        if not all([self.last_lidar_time, self.last_radar_time, self.last_imu_time]):
            self.get_logger().warn('Missing sensor data timestamps')
            return False
            
        # Check time differences between sensors
        time_diffs = [
            abs((self.last_lidar_time - self.last_radar_time).nanoseconds / 1e9),
            abs((self.last_lidar_time - self.last_imu_time).nanoseconds / 1e9),
            abs((self.last_radar_time - self.last_imu_time).nanoseconds / 1e9)
        ]
        
        if max(time_diffs) > self.max_time_diff:
            self.get_logger().warn(f'Sensor data out of sync: {max(time_diffs)}s difference')
            return False
            
        return True

    def lidar_callback(self, msg):
        self.last_lidar_time = msg.header.stamp
        self.lidar_clusters = msg
        
    def radar_callback(self, msg):
        self.last_radar_time = msg.header.stamp
        self.radar_clusters = msg
        
    def imu_callback(self, msg):
        self.last_imu_time = msg.header.stamp
        self.imu_orientation = msg.orientation

    def create_occupancy_grid(self, points):
        if points is None or len(points) == 0:
            return None
            
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        
        grid.info.resolution = self.grid_resolution
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height
        grid.info.origin.position.x = -self.grid_width * self.grid_resolution / 2
        grid.info.origin.position.y = -self.grid_height * self.grid_resolution / 2
        
        # Initialize grid data
        grid.data = [-1] * (self.grid_width * self.grid_height)  # -1 for unknown
        
        # Mark occupied cells from sensor data
        for point in points:
            x = int((point[0] - grid.info.origin.position.x) / self.grid_resolution)
            y = int((point[1] - grid.info.origin.position.y) / self.grid_resolution)
            
            if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                grid.data[y * self.grid_width + x] = 100  # 100 for occupied
                
                # Mark surrounding cells as free
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                            if grid.data[ny * self.grid_width + nx] != 100:
                                grid.data[ny * self.grid_width + nx] = 0  # 0 for free
        
        return grid

    def plan_path(self, start, goal, grid):
        if grid is None:
            return None
            
        def heuristic(a, b):
            return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
        
        def get_neighbors(pos):
            neighbors = []
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,1), (1,-1), (-1,-1)]:
                new_pos = (pos[0] + dx, pos[1] + dy)
                if (0 <= new_pos[0] < self.grid_width and 
                    0 <= new_pos[1] < self.grid_height and 
                    grid.data[new_pos[1] * self.grid_width + new_pos[0]] == 0):
                    neighbors.append(new_pos)
            return neighbors
        
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        # A* implementation
        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            current = frontier.pop(0)[1]
            
            if current == goal:
                break
                
            for next_pos in get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + heuristic(goal, next_pos)
                    frontier.append((priority, next_pos))
                    frontier.sort()
                    came_from[next_pos] = current
        
        # Reconstruct path
        current = goal
        while current is not None:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = current[0] * self.grid_resolution + grid.info.origin.position.x
            pose.pose.position.y = current[1] * self.grid_resolution + grid.info.origin.position.y
            path.poses.insert(0, pose)
            current = came_from.get(current)
        
        return path

    def validate_sensor_data(self):
        if self.lidar_clusters is None:
            self.get_logger().warn('No LiDAR data available')
            return False
            
        if self.radar_clusters is None:
            self.get_logger().warn('No Radar data available')
            return False
            
        if self.imu_orientation is None:
            self.get_logger().warn('No IMU data available')
            return False
            
        # Check for valid point counts
        lidar_points = list(point_cloud2.read_points(self.lidar_clusters))
        radar_points = list(point_cloud2.read_points(self.radar_clusters))
        
        if len(lidar_points) == 0:
            self.get_logger().warn('Empty LiDAR point cloud')
            return False
            
        if len(radar_points) == 0:
            self.get_logger().warn('Empty Radar point cloud')
            return False
            
        return True

    def fuse_sensor_data(self):
        # Check data synchronization
        if not self.check_data_sync():
            return

        # Validate sensor data
        if not self.validate_sensor_data():
            return

        try:
            marker_array = MarkerArray()
            marker_id = 0

            # Extract points with error handling
            try:
                lidar_points = list(point_cloud2.read_points(self.lidar_clusters))
                radar_points = list(point_cloud2.read_points(self.radar_clusters))
            except Exception as e:
                self.get_logger().error(f'Error reading point clouds: {str(e)}')
                return

            # Apply IMU orientation correction
            if self.imu_orientation:
                q = self.imu_orientation
                # Convert quaternion to rotation matrix
                rotation_matrix = np.array([
                    [1 - 2*q.y*q.y - 2*q.z*q.z, 2*q.x*q.y - 2*q.z*q.w, 2*q.x*q.z + 2*q.y*q.w],
                    [2*q.x*q.y + 2*q.z*q.w, 1 - 2*q.x*q.x - 2*q.z*q.z, 2*q.y*q.z - 2*q.x*q.w],
                    [2*q.x*q.z - 2*q.y*q.w, 2*q.y*q.z + 2*q.x*q.w, 1 - 2*q.x*q.x - 2*q.y*q.y]
                ])
                
                # Apply rotation to sensor data points
                lidar_points = [tuple(rotation_matrix.dot(np.array(point[:3]))) for point in lidar_points]
                radar_points = [tuple(rotation_matrix.dot(np.array(point[:3]))) for point in radar_points]

            # Process each detection with error checking
            for lidar_cluster in lidar_points:
                try:
                    matching_radar = None
                    min_distance = float('inf')
                    
                    for radar_point in radar_points:
                        try:
                            dist = np.linalg.norm(np.array(lidar_cluster[:3]) - np.array(radar_point[:3]))
                            if dist < min_distance and dist < 1.0:
                                min_distance = dist
                                matching_radar = radar_point
                        except Exception as e:
                            self.get_logger().warn(f'Error matching points: {str(e)}')
                            continue

                    # Create visualization marker
                    marker = self.create_detection_marker(
                        lidar_cluster, 
                        matching_radar, 
                        marker_id
                    )
                    
                    if marker is not None:
                        marker_array.markers.append(marker)
                        marker_id += 1

                except Exception as e:
                    self.get_logger().warn(f'Error processing cluster: {str(e)}')
                    continue

            # Publish results if we have any
            if marker_array.markers:
                self.fusion_publisher.publish(marker_array)
            else:
                self.get_logger().warn('No markers generated from fusion')

        except Exception as e:
            self.get_logger().error(f'Fusion process failed: {str(e)}')

    def process_sensor_data(self):
        # Fuse sensor data
        self.fuse_sensor_data()
        
        # Create occupancy grid
        grid = self.create_occupancy_grid(self.lidar_clusters)
        if grid is not None:
            self.grid_publisher.publish(grid)
        
        # Plan path (example: from center to edge)
        start = (self.grid_width//2, self.grid_height//2)
        goal = (self.grid_width-1, self.grid_height-1)
        path = self.plan_path(start, goal, grid)
        if path is not None:
            self.path_publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 