import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time

class CostmapCreator(Node):
    def __init__(self):
        super().__init__('costmap_creator')
        
        # Create a subscription to the fused grid
        self.fused_grid_sub = self.create_subscription(
            OccupancyGrid,
            '/fused_costmap',
            self.fused_grid_callback,
            10
        )
        
        # Create publishers for the local costmap and the inflation layer
        self.local_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/local_costmap',
            10
        )
        
        self.inflation_layer_pub = self.create_publisher(
            OccupancyGrid,
            '/inflation_layer',
            10
        )
        
        # Configuration parameters
        self.inflation_radius = 10  # cells (for the inflation layer)
        self.cost_scaling_factor = 3.0  # controls how quickly cost decreases with distance
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Costmap Creator Node initialized')

    def fused_grid_callback(self, msg):
        """
        Process the fused grid to create the local costmap with inflation layer.
        
        Args:
            msg (OccupancyGrid): Fused occupancy grid
        """
        try:
            # First, create the local costmap based on the fused grid
            local_costmap = self.create_local_costmap(msg)
            
            # Then, create the inflation layer
            inflation_layer = self.create_inflation_layer(local_costmap)
            
            # Publish both costmaps
            self.local_costmap_pub.publish(local_costmap)
            self.inflation_layer_pub.publish(inflation_layer)
            
            self.get_logger().debug('Published local costmap and inflation layer')
            
        except Exception as e:
            self.get_logger().error(f'Error creating costmap: {str(e)}')

    def create_local_costmap(self, fused_grid):
        """
        Create a local costmap from the fused grid.
        
        Args:
            fused_grid (OccupancyGrid): The fused grid from multiple sensors
            
        Returns:
            OccupancyGrid: The local costmap
        """
        # For now, we'll just pass through the fused grid as our local costmap
        # In a more complex implementation, you might extract a region around the robot
        # or perform additional processing
        
        local_costmap = OccupancyGrid()
        local_costmap.header = fused_grid.header
        local_costmap.info = fused_grid.info
        local_costmap.data = list(fused_grid.data)  # Make a copy of the data
        
        return local_costmap

    def create_inflation_layer(self, local_costmap):
        """
        Create an inflation layer based on the local costmap.
        The inflation layer expands obstacles to create a safety buffer.
        
        Args:
            local_costmap (OccupancyGrid): The local costmap
            
        Returns:
            OccupancyGrid: The inflation layer
        """
        # Create a new grid for the inflation layer
        inflation_layer = OccupancyGrid()
        inflation_layer.header = local_costmap.header
        inflation_layer.info = local_costmap.info
        
        width = local_costmap.info.width
        height = local_costmap.info.height
        
        # Initialize the inflation layer with zeros (free space)
        inflation_data = [0] * (width * height)
        
        # For each cell in the local costmap
        for y in range(height):
            for x in range(width):
                index = y * width + x
                
                # If this is an obstacle in the local costmap (cost > 0)
                if local_costmap.data[index] > 0:
                    # Set this cell to the obstacle's cost in the inflation layer
                    inflation_data[index] = local_costmap.data[index]
                    
                    # Inflate around this obstacle
                    self.inflate_cell(x, y, local_costmap.data[index], inflation_data, width, height)
        
        inflation_layer.data = inflation_data
        return inflation_layer

    def inflate_cell(self, x, y, cost, inflation_data, width, height):
        """
        Inflate the cost around a cell, decreasing with distance.
        
        Args:
            x (int): Cell x-coordinate
            y (int): Cell y-coordinate
            cost (int): The cost of the obstacle cell
            inflation_data (list): The inflation layer data
            width (int): Width of the grid
            height (int): Height of the grid
        """
        # Calculate inflation range based on cost scaling
        for dy in range(-self.inflation_radius, self.inflation_radius + 1):
            for dx in range(-self.inflation_radius, self.inflation_radius + 1):
                # Skip the center cell (the obstacle itself)
                if dx == 0 and dy == 0:
                    continue
                    
                nx, ny = x + dx, y + dy
                
                # Check if the cell is within the grid
                if 0 <= nx < width and 0 <= ny < height:
                    # Calculate distance to obstacle
                    distance = np.sqrt(dx*dx + dy*dy)
                    
                    # Skip cells outside the inflation radius
                    if distance > self.inflation_radius:
                        continue
                        
                    # Calculate cost based on distance and scaling factor
                    # The formula is a common exponential decay function
                    # Higher cost_scaling_factor = faster cost decay with distance
                    new_cost = int(cost * np.exp(-self.cost_scaling_factor * distance))
                    
                    # Get index for the cell
                    idx = ny * width + nx
                    
                    # Update the cost if it's higher than the current value
                    inflation_data[idx] = max(inflation_data[idx], new_cost)

def main(args=None):
    rclpy.init(args=args)
    costmap_creator = CostmapCreator()
    
    try:
        rclpy.spin(costmap_creator)
    except KeyboardInterrupt:
        pass
    finally:
        costmap_creator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 