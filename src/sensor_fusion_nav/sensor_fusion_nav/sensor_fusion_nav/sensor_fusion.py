import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
from sensor_msgs.msg import Imu
import threading
import math

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Create subscriptions to the grid maps from LiDAR and Radar
        self.lidar_sub = self.create_subscription(
            OccupancyGrid,
            '/lidar_grid',
            self.lidar_callback,
            10
        )
        
        self.radar_sub = self.create_subscription(
            OccupancyGrid,
            '/radar_grid',
            self.radar_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Create publisher for the fused costmap
        self.fused_pub = self.create_publisher(
            OccupancyGrid,
            '/fused_costmap',
            10
        )
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Store latest grid maps
        self.lidar_grid = None
        self.radar_grid = None
        self.imu_data = None
        
        # Locks for thread safety
        self.lidar_lock = threading.Lock()
        self.radar_lock = threading.Lock()
        self.imu_lock = threading.Lock()
        
        # Create a timer for periodic fusion
        self.fusion_timer = self.create_timer(0.1, self.fuse_sensor_data)  # 10 Hz
        
        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        """
        Callback for receiving LiDAR grid data.
        
        Args:
            msg (OccupancyGrid): LiDAR occupancy grid
        """
        with self.lidar_lock:
            self.lidar_grid = msg
            self.get_logger().debug('Received LiDAR grid')

    def radar_callback(self, msg):
        """
        Callback for receiving Radar grid data.
        
        Args:
            msg (OccupancyGrid): Radar occupancy grid
        """
        with self.radar_lock:
            self.radar_grid = msg
            self.get_logger().debug('Received Radar grid')

    def imu_callback(self, msg):
        """
        Callback for receiving IMU data.
        
        Args:
            msg (Imu): IMU message
        """
        with self.imu_lock:
            self.imu_data = msg
            self.get_logger().debug('Received IMU data')

    def fuse_sensor_data(self):
        """
        Fuse LiDAR and Radar data using a simplified Kalman Filter approach.
        In a real application, you would use a more sophisticated sensor fusion algorithm.
        """
        # Check if we have data from both sensors
        with self.lidar_lock and self.radar_lock and self.imu_lock:
            if self.lidar_grid is None or self.radar_grid is None:
                return
            
            # Ensure both grids have the same dimensions
            if (self.lidar_grid.info.width != self.radar_grid.info.width or
                self.lidar_grid.info.height != self.radar_grid.info.height):
                self.get_logger().warn('LiDAR and Radar grids have different dimensions - cannot fuse')
                return
            
            # Create a new occupancy grid for the fusion result
            fused_grid = OccupancyGrid()
            fused_grid.header.stamp = self.get_clock().now().to_msg()
            fused_grid.header.frame_id = "map"
            fused_grid.info = self.lidar_grid.info  # Use LiDAR grid metadata
            
            # Initialize grid with unknown values (-1)
            width = fused_grid.info.width
            height = fused_grid.info.height
            fused_grid.data = [-1] * (width * height)
            
            # Perform a simple fusion:
            # - If both LiDAR and Radar detect an obstacle, take the higher cost
            # - If only one sensor detects an obstacle, use that value
            # - If both sensors report free or unknown, keep as unknown or free
            for i in range(width * height):
                lidar_value = self.lidar_grid.data[i]
                radar_value = self.radar_grid.data[i]
                
                # Both detect obstacles - take the maximum value (higher cost)
                if lidar_value > 0 and radar_value > 0:
                    fused_grid.data[i] = max(lidar_value, radar_value)
                
                # LiDAR detects an obstacle
                elif lidar_value > 0:
                    fused_grid.data[i] = lidar_value
                
                # Radar detects an obstacle (possibly moving)
                elif radar_value > 0:
                    fused_grid.data[i] = radar_value
                
                # Both report free space
                elif lidar_value == 0 and radar_value == 0:
                    fused_grid.data[i] = 0
                
                # One reports free, the other unknown
                elif lidar_value == 0 or radar_value == 0:
                    fused_grid.data[i] = 0
                
                # Both unknown
                else:
                    fused_grid.data[i] = -1
            
            # Apply IMU correction (tilt compensation) if available
            if self.imu_data is not None:
                # In a real application, you would use the IMU data to adjust the grid
                # based on the vehicle's orientation. This could involve transforming
                # the grid or adjusting individual cell values.
                pass
            
            # Publish the fused costmap
            self.fused_pub.publish(fused_grid)
            self.get_logger().debug('Published fused costmap')

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion = SensorFusion()
    
    try:
        rclpy.spin(sensor_fusion)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 