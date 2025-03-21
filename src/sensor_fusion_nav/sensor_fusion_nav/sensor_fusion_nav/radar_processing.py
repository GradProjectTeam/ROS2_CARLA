import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import std_msgs.msg

class RadarProcessor(Node):
    def __init__(self):
        super().__init__('radar_processor')
        
        # Create a subscription to the Radar clusters
        self.radar_sub = self.create_subscription(
            PointCloud2,
            '/radar/clusters',
            self.radar_callback,
            10
        )
        
        # Create a publisher for the occupancy grid
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            '/radar_grid',
            10
        )
        
        # Parameters for the occupancy grid
        self.grid_size_x = 20.0  # meters
        self.grid_size_y = 20.0  # meters
        self.grid_resolution = 0.05  # meters per cell
        
        # Store the previous radar data for tracking movement
        self.previous_data = None
        self.previous_timestamp = None
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Radar Processor Node initialized')

    def radar_callback(self, msg):
        """
        Process Radar point cloud clusters and convert to occupancy grid,
        with higher costs assigned to moving objects.
        
        Args:
            msg (PointCloud2): Point cloud message containing Radar clusters
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
            
            # Process radar data to fill the grid with dynamic cost values
            self.process_radar_data_to_grid(msg, grid)
            
            # Publish the occupancy grid
            self.grid_pub.publish(grid)
            self.get_logger().debug('Published Radar occupancy grid')
            
            # Store data for the next callback
            self.previous_data = msg
            self.previous_timestamp = msg.header.stamp
            
        except Exception as e:
            self.get_logger().error(f'Error processing Radar data: {str(e)}')

    def process_radar_data_to_grid(self, radar_data, grid):
        """
        Convert Radar PointCloud2 data to OccupancyGrid, with different
        costs assigned based on whether objects are static or moving.
        
        In a real implementation, you would analyze the velocity component
        of radar returns and assign higher costs to faster-moving objects.
        
        Args:
            radar_data (PointCloud2): The input radar data
            grid (OccupancyGrid): The output occupancy grid to be filled
        """
        # In a full implementation, you would:
        # 1. Extract radar points and velocities from the PointCloud2
        # 2. Identify moving objects based on Doppler velocity
        # 3. Assign higher costs to moving objects, based on speed
        
        # For this demonstration, we'll create some simulated moving objects
        center_x = grid.info.width // 2
        center_y = grid.info.height // 2
        
        # Mark regular static obstacles (cost 70)
        for y in range(center_y - 15, center_y + 15, 5):
            for x in range(center_x - 15, center_x + 15, 5):
                if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                    if abs(x - center_x) > 8 or abs(y - center_y) > 8:
                        index = y * grid.info.width + x
                        grid.data[index] = 70
        
        # Simulate fast-moving objects (cost 90-100)
        # These would represent cars, bicycles, pedestrians, etc.
        offset = int(self.get_clock().now().nanoseconds / 1e8) % 10
        for i in range(3):  # Create 3 moving objects
            # Create movement pattern
            x = center_x + 10 + offset + i * 5
            y = center_y + 5 - i * 3
            
            if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                # Higher cost (100) for the moving object center
                index = y * grid.info.width + x
                grid.data[index] = 100
                
                # Lower cost (90) for the area around the moving object
                for dx in range(-1, 2):
                    for dy in range(-1, 2):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < grid.info.width and 0 <= ny < grid.info.height:
                            index = ny * grid.info.width + nx
                            if grid.data[index] < 90:  # Only override if current cost is lower
                                grid.data[index] = 90

def main(args=None):
    rclpy.init(args=args)
    radar_processor = RadarProcessor()
    
    try:
        rclpy.spin(radar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        radar_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 