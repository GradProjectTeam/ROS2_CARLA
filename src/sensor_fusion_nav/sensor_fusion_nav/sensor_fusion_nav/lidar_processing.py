import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import std_msgs.msg

class LiDARProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Create a subscription to the LiDAR clusters
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/clusters',
            self.lidar_callback,
            10
        )
        
        # Create a publisher for the occupancy grid
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            '/lidar_grid',
            10
        )
        
        # Parameters for the occupancy grid
        self.grid_size_x = 20.0  # meters
        self.grid_size_y = 20.0  # meters
        self.grid_resolution = 0.05  # meters per cell
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('LiDAR Processor Node initialized')

    def lidar_callback(self, msg):
        """
        Process LiDAR point cloud clusters and convert to occupancy grid.
        
        Args:
            msg (PointCloud2): Point cloud message containing LiDAR clusters
        """
        try:
            # Create a new occupancy grid message
            grid = OccupancyGrid()
            grid.header = msg.header
            grid.info.resolution = self.grid_resolution
            
            # Calculate grid dimensions
            grid.info.width = int(self.grid_size_x / self.grid_resolution)
            grid.info.height = int(self.grid_size_y / self.grid_resolution)
            
            # Set origin (bottom-left corner of the grid)
            grid.info.origin.position.x = -self.grid_size_x / 2.0
            grid.info.origin.position.y = -self.grid_size_y / 2.0
            grid.info.origin.position.z = 0.0
            grid.info.origin.orientation.w = 1.0
            
            # Initialize grid with unknown values (-1)
            grid.data = [-1] * (grid.info.width * grid.info.height)
            
            # Process point cloud data to fill the grid
            # This is a simplified implementation - you would need to 
            # convert the point cloud to occupancy grid based on your specific needs
            self.process_point_cloud_to_grid(msg, grid)
            
            # Publish the occupancy grid
            self.grid_pub.publish(grid)
            self.get_logger().debug('Published LiDAR occupancy grid')
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {str(e)}')

    def process_point_cloud_to_grid(self, point_cloud, grid):
        """
        Convert PointCloud2 data to OccupancyGrid.
        
        This is a placeholder implementation. In a real application, you would:
        1. Extract point cloud data
        2. Project points onto the 2D grid
        3. Mark cells as occupied (100), free (0), or unknown (-1)
        
        Args:
            point_cloud (PointCloud2): The input point cloud
            grid (OccupancyGrid): The output occupancy grid to be filled
        """
        # In a full implementation, you would:
        # 1. Convert the PointCloud2 to a numpy array of points
        # 2. For each point, calculate its cell in the grid
        # 3. Mark those cells as occupied (100)
        # 4. Optionally, use raycasting to mark cells between the sensor and obstacles as free (0)
        
        # For demonstration, just mark the center as free and add some sample obstacles
        center_x = grid.info.width // 2
        center_y = grid.info.height // 2
        
        # Mark center area as free (0)
        for y in range(center_y - 10, center_y + 10):
            for x in range(center_x - 10, center_x + 10):
                if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                    index = y * grid.info.width + x
                    grid.data[index] = 0
        
        # Add some sample obstacles (100)
        for y in range(center_y - 20, center_y + 20, 5):
            for x in range(center_x - 20, center_x + 20, 5):
                if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                    if abs(x - center_x) > 12 or abs(y - center_y) > 12:
                        index = y * grid.info.width + x
                        grid.data[index] = 100

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LiDARProcessor()
    
    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 