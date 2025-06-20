#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

class BinaryMapNavigation(Node):
    """
    Processes binary maps for navigation use.
    
    This node:
    1. Subscribes to the binary map from the semantic costmap
    2. Applies processing like inflation or filtering if needed
    3. Republishes the map with the appropriate formats for navigation
    4. Ensures compatibility with ROS2 Navigation and other planning stacks
    """
    def __init__(self):
        super().__init__('binary_map_navigation')
        
        # Declare parameters
        self.declare_parameter(
            'input_topic',
            '/semantic_costmap/binary',
            ParameterDescriptor(description='Input binary map topic')
        )
        
        self.declare_parameter(
            'output_topic',
            '/map',
            ParameterDescriptor(description='Output map topic for navigation')
        )
        
        self.declare_parameter(
            'apply_inflation',
            True,
            ParameterDescriptor(description='Apply inflation radius to obstacles')
        )
        
        self.declare_parameter(
            'inflation_radius',
            0.5,
            ParameterDescriptor(
                description='Inflation radius in meters',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1,
                    to_value=2.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'occupied_value',
            100,
            ParameterDescriptor(description='Value for occupied cells')
        )
        
        self.declare_parameter(
            'free_value',
            0,
            ParameterDescriptor(description='Value for free cells')
        )
        
        self.declare_parameter(
            'unknown_value',
            -1,
            ParameterDescriptor(description='Value for unknown cells')
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.apply_inflation = self.get_parameter('apply_inflation').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.occupied_value = self.get_parameter('occupied_value').value
        self.free_value = self.get_parameter('free_value').value
        self.unknown_value = self.get_parameter('unknown_value').value
        
        # QoS profiles
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Create publisher and subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.input_topic,
            self.map_callback,
            map_qos
        )
        
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.output_topic,
            map_qos
        )
        
        self.latest_map = None
        self.map_resolution = None
        
        self.get_logger().info(f'Binary Map Navigation Node initialized')
        self.get_logger().info(f'Subscribing to {self.input_topic} and publishing on {self.output_topic}')
    
    def map_callback(self, msg):
        """Process incoming binary map and republish for navigation"""
        self.get_logger().debug('Received binary map')
        
        # Store the map resolution for inflation calculations
        self.map_resolution = msg.info.resolution
        
        # Convert map data to numpy array for processing
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data, dtype=np.int8).reshape(height, width)
        
        # Process the map (inflation, etc.)
        processed_map = self.process_map(map_data)
        
        # Create new map message
        nav_map = OccupancyGrid()
        nav_map.header = msg.header
        nav_map.info = msg.info
        nav_map.data = processed_map.flatten().tolist()
        
        # Publish the processed map
        self.map_pub.publish(nav_map)
        self.get_logger().debug('Published navigation map')
    
    def process_map(self, map_data):
        """Process the binary map for navigation use"""
        # Create a copy of the map for processing
        processed_map = map_data.copy()
        
        if self.apply_inflation:
            processed_map = self.inflate_obstacles(processed_map)
        
        return processed_map
    
    def inflate_obstacles(self, map_data):
        """Inflate obstacles to create a safety margin"""
        try:
            import cv2
            
            # Convert to binary image for processing
            binary = np.zeros_like(map_data)
            binary[map_data == self.occupied_value] = 255
            
            # Calculate kernel size based on inflation radius and map resolution
            kernel_size = int(self.inflation_radius / self.map_resolution)
            if kernel_size < 1:
                kernel_size = 1
            
            # Create circular kernel for inflation
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, 
                (kernel_size*2+1, kernel_size*2+1)
            )
            
            # Dilate the binary image
            inflated = cv2.dilate(binary, kernel, iterations=1)
            
            # Apply inflation to the map
            inflated_map = map_data.copy()
            inflated_map[inflated > 0] = self.occupied_value
            
            self.get_logger().debug(f'Inflated obstacles with radius {self.inflation_radius}m ({kernel_size} pixels)')
            return inflated_map
            
        except ImportError:
            self.get_logger().warning('OpenCV not available. Inflation disabled.')
            return map_data

def main(args=None):
    rclpy.init(args=args)
    node = BinaryMapNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 