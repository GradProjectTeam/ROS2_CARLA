#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import ColorRGBA, String
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
import math
import time
from threading import Lock
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import SetBool
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
import os
from datetime import datetime

class SemanticCostmapVisualizer(Node):
    """
    Creates a multi-layer semantic costmap visualization from LiDAR and radar data.
    
    This node:
    1. Subscribes to LiDAR point clouds and cluster data
    2. Classifies clusters into semantic categories (ground, obstacle, vegetation, building, dynamic)
    3. Creates separate costmap layers for each semantic category
    4. Provides interactive visualization with layer toggling
    5. Integrates with RViz for enhanced visualization
    6. Produces a binary (black/white) map output for navigation planning
    """
    def __init__(self):
        super().__init__('semantic_costmap_visualizer')
        
        # Declare parameters with descriptors for dynamic reconfigure
        self.add_dynamic_parameters()
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width_meters = self.get_parameter('map_width_meters').value
        self.map_height_meters = self.get_parameter('map_height_meters').value
        self.map_frame = self.get_parameter('map_frame').value
        self.lidar_points_topic = self.get_parameter('lidar_points_topic').value
        self.lidar_clusters_topic = self.get_parameter('lidar_clusters_topic').value
        self.radar_points_topic = self.get_parameter('radar_points_topic').value
        self.radar_clusters_topic = self.get_parameter('radar_clusters_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.temporal_filtering = self.get_parameter('temporal_filtering').value
        self.motion_prediction = self.get_parameter('motion_prediction').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.decay_time = self.get_parameter('decay_time').value
        self.dynamic_decay_time = self.get_parameter('dynamic_decay_time').value
        self.enable_3d_visualization = self.get_parameter('enable_3d_visualization').value
        self.enable_text_labels = self.get_parameter('enable_text_labels').value
        
        # Add new parameters for classification
        self.ground_height_threshold = self.get_parameter('ground_height_threshold').value
        self.vegetation_height_ratio = self.get_parameter('vegetation_height_ratio').value
        self.building_width_threshold = self.get_parameter('building_width_threshold').value
        self.dynamic_velocity_threshold = self.get_parameter('dynamic_velocity_threshold').value
        
        # Add binary map parameters
        self.enable_binary_output = self.get_parameter('enable_binary_output').value
        self.binary_topic = self.get_parameter('binary_topic').value
        self.occupied_value = self.get_parameter('occupied_value').value
        self.free_value = self.get_parameter('free_value').value
        self.binary_threshold = self.get_parameter('binary_threshold').value
        self.convert_vegetation_to_occupied = self.get_parameter('convert_vegetation_to_occupied').value
        self.convert_all_non_ground_to_occupied = self.get_parameter('convert_all_non_ground_to_occupied').value
        
        # Add parameters for ground inclusion in binary map
        self.include_ground_in_binary = self.get_parameter('include_ground_in_binary').value
        self.ground_as_occupied = self.get_parameter('ground_as_occupied').value
        
        # Add marker lifetime parameter
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        
        # Map saving parameters
        self.enable_map_saving = self.get_parameter('enable_map_saving').value
        self.save_directory = self.get_parameter('save_directory').value
        self.save_interval = self.get_parameter('save_interval').value
        self.save_binary_map = self.get_parameter('save_binary_map').value
        self.save_combined_map = self.get_parameter('save_combined_map').value
        self.save_layer_maps = self.get_parameter('save_layer_maps').value
        self.last_save_time = time.time()
        
        # Add new parameters for low car detection
        self.enhance_low_cars = self.get_parameter('enhance_low_cars').value
        self.low_car_height_threshold = self.get_parameter('low_car_height_threshold').value
        self.car_detection_width = self.get_parameter('car_detection_width').value
        
        # Get the expansion radius parameters
        self.car_expansion_radius = self.get_parameter('car_expansion_radius').value
        self.obstacle_expansion_radius = self.get_parameter('obstacle_expansion_radius').value
        self.dynamic_expansion_radius = self.get_parameter('dynamic_expansion_radius').value
        
        # Create save directory if it doesn't exist
        if self.enable_map_saving:
            os.makedirs(self.save_directory, exist_ok=True)
            self.get_logger().info(f'Map saving enabled. Maps will be saved to {self.save_directory}')
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Calculate map dimensions
        self.map_width = int(self.map_width_meters / self.map_resolution)
        self.map_height = int(self.map_height_meters / self.map_resolution)
        self.map_origin_x = -self.map_width_meters / 2.0
        self.map_origin_y = -self.map_height_meters / 2.0
        
        # Initialize multi-layer costmap
        self.initialize_costmap()
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create QoS profile for map
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Create QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publishers for costmap layers
        self.layer_publishers = {}
        self.layer_visibility = {}
        
        for layer in ['ground', 'obstacle', 'vegetation', 'building', 'dynamic', 'combined']:
            self.layer_publishers[layer] = self.create_publisher(
                OccupancyGrid,
                f'/semantic_costmap/{layer}',
                map_qos
            )
            self.layer_visibility[layer] = True
        
        # Create publisher for semantic markers
        self.semantic_marker_pub = self.create_publisher(
            MarkerArray,
            '/semantic_costmap/markers',
            10
        )
        
        # Create publisher for height map visualization
        if self.enable_3d_visualization:
            self.height_map_pub = self.create_publisher(
                PointCloud2,
                '/semantic_costmap/height_map',
                10
            )
        
        # Create publisher for binary map
        if self.enable_binary_output:
            self.binary_map_pub = self.create_publisher(
                OccupancyGrid,
                self.binary_topic,
                map_qos
            )
            self.get_logger().info(f'Binary map will be published on {self.binary_topic}')
            self.get_logger().info(f'Binary threshold: {self.binary_threshold}, Occupied value: {self.occupied_value}, Free value: {self.free_value}')
        
        # Create subscribers
        self.lidar_points_sub = self.create_subscription(
            PointCloud2,
            self.lidar_points_topic,
            self.lidar_points_callback,
            sensor_qos
        )
        
        self.lidar_clusters_sub = self.create_subscription(
            MarkerArray,
            self.lidar_clusters_topic,
            self.lidar_clusters_callback,
            sensor_qos
        )
        
        self.radar_points_sub = self.create_subscription(
            PointCloud2,
            self.radar_points_topic,
            self.radar_points_callback,
            sensor_qos
        )
        
        self.radar_clusters_sub = self.create_subscription(
            MarkerArray,
            self.radar_clusters_topic,
            self.radar_clusters_callback,
            sensor_qos
        )
        
        # Create service for toggling layers
        self.toggle_layer_srv = self.create_service(
            SetBool,
            '/semantic_costmap/toggle_layer',
            self.toggle_layer_callback
        )
        
        # Create service for saving maps on demand
        self.save_maps_srv = self.create_service(
            SetBool,
            '/semantic_costmap/save_maps',
            self.save_maps_callback
        )
        
        # Create timers
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_costmap_layers
        )
        
        if self.temporal_filtering:
            # Use a more reasonable frequency (20 Hz) for temporal filtering to balance responsiveness and stability
            self.filtering_timer = self.create_timer(
                0.05,  # 20 Hz - reduced from 50 Hz for more stable behavior
                self.apply_temporal_filtering
            )
        
        # Create save timer if enabled
        if self.enable_map_saving:
            self.save_timer = self.create_timer(
                self.save_interval,
                self.save_maps
            )
            self.get_logger().info(f'Maps will be saved every {self.save_interval} seconds')
        
        # Data storage
        self.lidar_clusters = []
        self.radar_clusters = []
        self.dynamic_objects = {}  # For tracking dynamic objects
        self.last_update_time = time.time()
        self.height_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Thread safety
        self.costmap_lock = Lock()
        
        self.get_logger().info('Semantic Costmap Visualizer initialized')
        self.get_logger().info(f'Map dimensions: {self.map_width}x{self.map_height} cells')
        self.get_logger().info(f'Using balanced decay settings: decay_time={self.decay_time}s, dynamic_decay_time={self.dynamic_decay_time}s')
        self.get_logger().info(f'Temporal filtering running at 20 Hz to balance responsiveness and stability')
    
    def add_dynamic_parameters(self):
        """Add dynamic parameters that can be reconfigured at runtime"""
        # Map parameters
        self.declare_parameter(
            'map_resolution', 
            0.2,
            ParameterDescriptor(
                description='Resolution of the costmap in meters per cell',
                floating_point_range=[FloatingPointRange(
                    from_value=0.05, 
                    to_value=1.0, 
                    step=0.05
                )]
            )
        )
        
        self.declare_parameter(
            'map_width_meters', 
            60.0,
            ParameterDescriptor(
                description='Width of the costmap in meters',
                floating_point_range=[FloatingPointRange(
                    from_value=10.0, 
                    to_value=200.0, 
                    step=10.0
                )]
            )
        )
        
        self.declare_parameter(
            'map_height_meters', 
            60.0,
            ParameterDescriptor(
                description='Height of the costmap in meters',
                floating_point_range=[FloatingPointRange(
                    from_value=10.0, 
                    to_value=200.0, 
                    step=10.0
                )]
            )
        )
        
        self.declare_parameter('map_frame', 'map')
        
        # Topic parameters
        self.declare_parameter('lidar_points_topic', '/lidar/points')
        self.declare_parameter('lidar_clusters_topic', '/lidar/cubes')
        self.declare_parameter('radar_points_topic', '/radar/points')
        self.declare_parameter('radar_clusters_topic', '/radar/clusters')
        
        # Performance parameters
        self.declare_parameter(
            'publish_rate', 
            10.0,
            ParameterDescriptor(
                description='Rate at which to publish costmap layers (Hz)',
                floating_point_range=[FloatingPointRange(
                    from_value=1.0, 
                    to_value=100.0,  # Changed from 30.0 to 100.0 to support ultra-fast publishing
                    step=1.0
                )]
            )
        )
        
        # Feature flags
        self.declare_parameter('temporal_filtering', True)
        self.declare_parameter('motion_prediction', True)
        self.declare_parameter('enable_3d_visualization', True)
        self.declare_parameter('enable_text_labels', True)
        
        # Classification parameters
        self.declare_parameter(
            'min_confidence', 
            0.5,
            ParameterDescriptor(
                description='Minimum confidence for classification',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=1.0, 
                    step=0.05
                )]
            )
        )
        
        self.declare_parameter(
            'decay_time', 
            0.01,  # Ultra-fast decay (default was 0.05)
            ParameterDescriptor(
                description='Base time for cell decay in seconds',
                floating_point_range=[FloatingPointRange(
                    from_value=0.001,  # Changed lower bound for ultra-fast decay
                    to_value=10.0,
                    step=0.001
                )]
            )
        )
        
        # Add dynamic decay time parameter
        self.declare_parameter(
            'dynamic_decay_time', 
            0.005,  # Ultra-fast decay for dynamic objects (default was 0.02)
            ParameterDescriptor(
                description='Decay time specifically for dynamic objects in seconds',
                floating_point_range=[FloatingPointRange(
                    from_value=0.001,  # Changed lower bound for ultra-fast decay
                    to_value=5.0,
                    step=0.001
                )]
            )
        )
        
        # Add marker lifetime parameter
        self.declare_parameter(
            'marker_lifetime', 
            0.01,  # Ultra-fast marker lifetime
            ParameterDescriptor(
                description='Lifetime of visualization markers in seconds',
                floating_point_range=[FloatingPointRange(
                    from_value=0.001,
                    to_value=5.0,
                    step=0.001
                )]
            )
        )
        
        # Add new classification parameters
        self.declare_parameter(
            'ground_height_threshold', 
            0.3,
            ParameterDescriptor(
                description='Maximum height for ground classification (m)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.05,  # Changed from 0.1 to 0.05 to allow lower values for better low car detection
                    to_value=1.0, 
                    step=0.05
                )]
            )
        )
        
        self.declare_parameter(
            'vegetation_height_ratio', 
            3.0,
            ParameterDescriptor(
                description='Height to width ratio for vegetation classification',
                floating_point_range=[FloatingPointRange(
                    from_value=1.0, 
                    to_value=10.0, 
                    step=0.5
                )]
            )
        )
        
        self.declare_parameter(
            'building_width_threshold', 
            5.0,
            ParameterDescriptor(
                description='Minimum width for building classification (m)',
                floating_point_range=[FloatingPointRange(
                    from_value=2.0, 
                    to_value=20.0, 
                    step=0.5
                )]
            )
        )
        
        self.declare_parameter(
            'dynamic_velocity_threshold', 
            0.5,
            ParameterDescriptor(
                description='Minimum velocity for dynamic classification (m/s)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1, 
                    to_value=10.0, 
                    step=0.1
                )]
            )
        )
        
        # Layer weight parameters
        self.declare_parameter(
            'ground_weight', 
            0.1,
            ParameterDescriptor(
                description='Weight of ground layer in combined map',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'obstacle_weight', 
            1.0,
            ParameterDescriptor(
                description='Weight of obstacle layer in combined map',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'vegetation_weight', 
            0.7,
            ParameterDescriptor(
                description='Weight of vegetation layer in combined map',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'building_weight', 
            0.9,
            ParameterDescriptor(
                description='Weight of building layer in combined map',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'dynamic_weight', 
            1.0,
            ParameterDescriptor(
                description='Weight of dynamic layer in combined map',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
        
        # Binary map parameters
        self.declare_parameter(
            'enable_binary_output',
            True,
            ParameterDescriptor(
                description='Enable binary (black/white) output map for navigation'
            )
        )
        
        self.declare_parameter(
            'binary_topic',
            '/semantic_costmap/binary',
            ParameterDescriptor(
                description='Topic name for publishing binary obstacle map'
            )
        )
        
        self.declare_parameter(
            'occupied_value',
            100,
            ParameterDescriptor(
                description='Value for occupied cells in binary map (black)',
                integer_range=[IntegerRange(
                    from_value=1, 
                    to_value=100, 
                    step=1
                )]
            )
        )
        
        self.declare_parameter(
            'free_value',
            0,
            ParameterDescriptor(
                description='Value for free cells in binary map (white/transparent)',
                integer_range=[IntegerRange(
                    from_value=0, 
                    to_value=99, 
                    step=1
                )]
            )
        )
        
        self.declare_parameter(
            'binary_threshold',
            0.1,
            ParameterDescriptor(
                description='Threshold value for binary obstacle classification',
                floating_point_range=[FloatingPointRange(
                    from_value=0.01, 
                    to_value=1.0, 
                    step=0.01
                )]
            )
        )
        
        self.declare_parameter(
            'convert_vegetation_to_occupied',
            True,
            ParameterDescriptor(
                description='Convert vegetation cells to occupied in binary map'
            )
        )
        
        self.declare_parameter(
            'convert_all_non_ground_to_occupied',
            True,
            ParameterDescriptor(
                description='Convert all non-ground objects to occupied in binary map'
            )
        )
        
        # Add parameters for ground inclusion in binary map
        self.declare_parameter(
            'include_ground_in_binary',
            True,
            ParameterDescriptor(
                description='Include ground layer in the binary map'
            )
        )
        
        self.declare_parameter(
            'ground_as_occupied',
            True,
            ParameterDescriptor(
                description='Mark ground cells as occupied in binary map'
            )
        )
        
        # Add map saving parameters
        self.declare_parameter(
            'enable_map_saving',
            False,
            ParameterDescriptor(
                description='Enable saving maps to local files'
            )
        )
        
        self.declare_parameter(
            'save_directory',
            '/home/shishtawy/Carla/ROS2/NewMaps'
            # '/home/mostafa/GP/ROS2/maps/NewMaps',
            ParameterDescriptor(
                description='Directory to save map files'
            )
        )
        
        self.declare_parameter(
            'save_interval',
            3.0,
            ParameterDescriptor(
                description='Interval in seconds between map saves',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=3600.0, 
                    step=5.0
                )]
            )
        )
        
        self.declare_parameter(
            'save_binary_map',
            True,
            ParameterDescriptor(
                description='Save the binary map'
            )
        )
        
        self.declare_parameter(
            'save_combined_map',
            True,
            ParameterDescriptor(
                description='Save the combined map'
            )
        )
        
        self.declare_parameter(
            'save_layer_maps',
            False,
            ParameterDescriptor(
                description='Save individual layer maps'
            )
        )
        
        # Add parameters for low car detection
        self.declare_parameter(
            'enhance_low_cars', 
            True,
            ParameterDescriptor(
                description='Enable enhanced detection of low-profile cars'
            )
        )
        
        self.declare_parameter(
            'low_car_height_threshold', 
            0.1,
            ParameterDescriptor(
                description='Minimum height for low-profile car detection (meters)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.01, 
                    to_value=0.5, 
                    step=0.01
                )]
            )
        )
        
        self.declare_parameter(
            'car_detection_width', 
            0.5,
            ParameterDescriptor(
                description='Minimum width for car detection (meters)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.3, 
                    to_value=3.0, 
                    step=0.1
                )]
            )
        )
        
        # Add parameters for object expansion control
        self.declare_parameter(
            'car_expansion_radius', 
            2.0,
            ParameterDescriptor(
                description='Expansion radius for cars in cells',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=10.0, 
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'obstacle_expansion_radius', 
            0.5,
            ParameterDescriptor(
                description='Expansion radius for obstacles in cells',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=5.0, 
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'dynamic_expansion_radius', 
            1.0,
            ParameterDescriptor(
                description='Expansion radius for dynamic objects in cells',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, 
                    to_value=5.0, 
                    step=0.1
                )]
            )
        )
    
    def parameters_callback(self, params):
        """Callback for parameter changes"""
        result = SetParametersResult()
        result.successful = True
        
        map_changed = False
        weights_changed = False
        
        for param in params:
            if param.name == 'map_resolution' or \
               param.name == 'map_width_meters' or \
               param.name == 'map_height_meters':
                map_changed = True
            
            if param.name == 'publish_rate' and param.value != self.publish_rate:
                # Update publish timer
                self.publish_rate = param.value
                self.publish_timer.cancel()
                self.publish_timer = self.create_timer(
                    1.0 / self.publish_rate,
                    self.publish_costmap_layers
                )
                self.get_logger().info(f'Updated publish rate to {self.publish_rate} Hz')
            
            if param.name == 'temporal_filtering' and param.value != self.temporal_filtering:
                self.temporal_filtering = param.value
                if self.temporal_filtering and not hasattr(self, 'filtering_timer'):
                    # Use balanced frequency for more stable behavior
                    self.filtering_timer = self.create_timer(
                        0.05,  # 20 Hz - balanced for stability and responsiveness
                        self.apply_temporal_filtering
                    )
                elif not self.temporal_filtering and hasattr(self, 'filtering_timer'):
                    self.filtering_timer.cancel()
                    delattr(self, 'filtering_timer')
                self.get_logger().info(f'Temporal filtering set to {self.temporal_filtering}')
            
            if param.name == 'ground_height_threshold':
                self.ground_height_threshold = param.value
            
            if param.name == 'vegetation_height_ratio':
                self.vegetation_height_ratio = param.value
            
            if param.name == 'building_width_threshold':
                self.building_width_threshold = param.value
            
            if param.name == 'dynamic_velocity_threshold':
                self.dynamic_velocity_threshold = param.value
            
            # Add handling for decay time parameters
            if param.name == 'decay_time':
                self.decay_time = param.value
                self.get_logger().info(f'Updated decay time to {self.decay_time:.3f}s')
                
            if param.name == 'dynamic_decay_time':
                self.dynamic_decay_time = param.value
                self.get_logger().info(f'Updated dynamic decay time to {self.dynamic_decay_time:.3f}s')
                
            # Add handling for marker lifetime parameter
            if param.name == 'marker_lifetime':
                self.marker_lifetime = param.value
                self.get_logger().info(f'Updated marker lifetime to {self.marker_lifetime:.3f}s')
            
            if param.name.endswith('_weight'):
                weights_changed = True
            
            # Binary map parameters
            if param.name == 'enable_binary_output':
                self.enable_binary_output = param.value
                if self.enable_binary_output and not hasattr(self, 'binary_map_pub'):
                    map_qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        depth=1
                    )
                    self.binary_map_pub = self.create_publisher(
                        OccupancyGrid,
                        self.binary_topic,
                        map_qos
                    )
                    self.get_logger().info(f'Created binary map publisher on {self.binary_topic}')
            
            if param.name == 'binary_topic':
                self.binary_topic = param.value
                if hasattr(self, 'binary_map_pub'):
                    self.get_logger().warn(f'Binary map topic changed to {self.binary_topic}, but topic change requires restart to take effect')
            
            if param.name == 'occupied_value':
                self.occupied_value = param.value
                self.get_logger().info(f'Binary map occupied value set to {self.occupied_value}')
            
            if param.name == 'free_value':
                self.free_value = param.value
                self.get_logger().info(f'Binary map free value set to {self.free_value}')
            
            if param.name == 'binary_threshold':
                self.binary_threshold = param.value
                self.get_logger().info(f'Binary map threshold set to {self.binary_threshold}')
            
            if param.name == 'convert_vegetation_to_occupied':
                self.convert_vegetation_to_occupied = param.value
                self.get_logger().info(f'Converting vegetation to occupied: {self.convert_vegetation_to_occupied}')
            
            if param.name == 'convert_all_non_ground_to_occupied':
                self.convert_all_non_ground_to_occupied = param.value
                self.get_logger().info(f'Converting all non-ground to occupied: {self.convert_all_non_ground_to_occupied}')
            
            # Handle map saving parameters
            if param.name == 'enable_map_saving':
                self.enable_map_saving = param.value
                if self.enable_map_saving:
                    os.makedirs(self.save_directory, exist_ok=True)
                    if not hasattr(self, 'save_timer'):
                        self.save_timer = self.create_timer(
                            self.save_interval,
                            self.save_maps
                        )
                    self.get_logger().info(f'Map saving enabled. Maps will be saved to {self.save_directory}')
                else:
                    if hasattr(self, 'save_timer'):
                        self.save_timer.cancel()
                        delattr(self, 'save_timer')
                    self.get_logger().info('Map saving disabled')
            
            if param.name == 'save_directory':
                self.save_directory = param.value
                if self.enable_map_saving:
                    os.makedirs(self.save_directory, exist_ok=True)
                    self.get_logger().info(f'Map save directory changed to {self.save_directory}')
            
            if param.name == 'save_interval':
                self.save_interval = param.value
                if self.enable_map_saving and hasattr(self, 'save_timer'):
                    self.save_timer.cancel()
                    self.save_timer = self.create_timer(
                        self.save_interval,
                        self.save_maps
                    )
                self.get_logger().info(f'Map save interval updated to {self.save_interval} seconds')
                
            if param.name == 'save_binary_map':
                self.save_binary_map = param.value
                
            if param.name == 'save_combined_map':
                self.save_combined_map = param.value
                
            if param.name == 'save_layer_maps':
                self.save_layer_maps = param.value
            
            # Add handling for new parameters
            if param.name == 'enhance_low_cars':
                self.enhance_low_cars = param.value
                self.get_logger().info(f'Enhanced low car detection: {self.enhance_low_cars}')
                
            if param.name == 'low_car_height_threshold':
                self.low_car_height_threshold = param.value
                self.get_logger().info(f'Low car height threshold: {self.low_car_height_threshold}')
                
            if param.name == 'car_detection_width':
                self.car_detection_width = param.value
                self.get_logger().info(f'Car detection width: {self.car_detection_width}')
            
            # Handle expansion radius parameters
            if param.name == 'car_expansion_radius':
                self.car_expansion_radius = param.value
                self.get_logger().info(f'Car expansion radius set to {self.car_expansion_radius}')
                
            if param.name == 'obstacle_expansion_radius':
                self.obstacle_expansion_radius = param.value
                self.get_logger().info(f'Obstacle expansion radius set to {self.obstacle_expansion_radius}')
                
            if param.name == 'dynamic_expansion_radius':
                self.dynamic_expansion_radius = param.value
                self.get_logger().info(f'Dynamic expansion radius set to {self.dynamic_expansion_radius}')
        
        # If map dimensions changed, reinitialize the costmap
        if map_changed:
            with self.costmap_lock:
                self.map_resolution = self.get_parameter('map_resolution').value
                self.map_width_meters = self.get_parameter('map_width_meters').value
                self.map_height_meters = self.get_parameter('map_height_meters').value
                
                self.map_width = int(self.map_width_meters / self.map_resolution)
                self.map_height = int(self.map_height_meters / self.map_resolution)
                self.map_origin_x = -self.map_width_meters / 2.0
                self.map_origin_y = -self.map_height_meters / 2.0
                
                self.initialize_costmap()
                self.height_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
                
                self.get_logger().info(f'Map dimensions updated: {self.map_width}x{self.map_height} cells')
                self.get_logger().info(f'Map resolution updated: {self.map_resolution} meters/cell')
        
        # If layer weights changed, update them
        if weights_changed:
            with self.costmap_lock:
                self.layer_weights = {
                    'ground': self.get_parameter('ground_weight').value if self.has_parameter('ground_weight') else 0.1,
                    'obstacle': self.get_parameter('obstacle_weight').value if self.has_parameter('obstacle_weight') else 1.0,
                    'vegetation': self.get_parameter('vegetation_weight').value if self.has_parameter('vegetation_weight') else 0.7,
                    'building': self.get_parameter('building_weight').value if self.has_parameter('building_weight') else 0.9,
                    'dynamic': self.get_parameter('dynamic_weight').value if self.has_parameter('dynamic_weight') else 1.0
                }
                self.get_logger().info('Layer weights updated')
        
        return result
    
    def initialize_costmap(self):
        """Initialize the multi-layer costmap"""
        self.layers = {
            'ground': np.zeros((self.map_height, self.map_width), dtype=np.int8),
            'obstacle': np.zeros((self.map_height, self.map_width), dtype=np.int8),
            'vegetation': np.zeros((self.map_height, self.map_width), dtype=np.int8),
            'building': np.zeros((self.map_height, self.map_width), dtype=np.int8),
            'dynamic': np.zeros((self.map_height, self.map_width), dtype=np.int8),
            'combined': np.zeros((self.map_height, self.map_width), dtype=np.int8)
        }
        
        # Layer weights for combining into final costmap
        self.layer_weights = {
            'ground': self.get_parameter('ground_weight').value if self.has_parameter('ground_weight') else 0.1,
            'obstacle': self.get_parameter('obstacle_weight').value if self.has_parameter('obstacle_weight') else 1.0,
            'vegetation': self.get_parameter('vegetation_weight').value if self.has_parameter('vegetation_weight') else 0.7,
            'building': self.get_parameter('building_weight').value if self.has_parameter('building_weight') else 0.9,
            'dynamic': self.get_parameter('dynamic_weight').value if self.has_parameter('dynamic_weight') else 1.0
        }
        
        # Confidence values for each cell
        self.confidence_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        
        # Last update time for each cell
        self.cell_update_time = np.zeros((self.map_height, self.map_width), dtype=np.float32)
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map cell coordinates"""
        cell_x = int((x - self.map_origin_x) / self.map_resolution)
        cell_y = int((y - self.map_origin_y) / self.map_resolution)
        
        # Check if within bounds
        if 0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height:
            return cell_x, cell_y
        return None, None
    
    def map_to_world(self, cell_x, cell_y):
        """Convert map cell coordinates to world coordinates"""
        world_x = cell_x * self.map_resolution + self.map_origin_x
        world_y = cell_y * self.map_resolution + self.map_origin_y
        return world_x, world_y
    
    def lidar_points_callback(self, msg):
        """Process LiDAR point cloud data"""
        # This is just a placeholder - we're mainly using the cluster data
        self.get_logger().debug('Received LiDAR points')
    
    def extract_cluster_info(self, marker):
        """Extract cluster information from marker"""
        cluster = {
            'id': marker.id,
            'centroid': [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z],
            'dimensions': [marker.scale.x, marker.scale.y, marker.scale.z],
            'min_x': marker.pose.position.x - marker.scale.x/2,
            'max_x': marker.pose.position.x + marker.scale.x/2,
            'min_y': marker.pose.position.y - marker.scale.y/2,
            'max_y': marker.pose.position.y + marker.scale.y/2,
            'min_z': marker.pose.position.z - marker.scale.z/2,
            'max_z': marker.pose.position.z + marker.scale.z/2,
            'color': [marker.color.r, marker.color.g, marker.color.b, marker.color.a],
            'confidence': 0.9,  # Default confidence
            'velocity': 0.0,    # Default velocity
            'source': 'lidar'
        }
        return cluster
    
    def lidar_clusters_callback(self, msg):
        """Process LiDAR cluster data"""
        self.get_logger().debug(f'Received {len(msg.markers)} LiDAR clusters')
        
        # Extract clusters from markers
        clusters = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE:
                cluster = self.extract_cluster_info(marker)
                clusters.append(cluster)
        
        # Update lidar clusters
        with self.costmap_lock:
            self.lidar_clusters = clusters
            
            # Process each cluster
            for cluster in clusters:
                # Classify cluster
                cluster_class = self.classify_cluster(cluster)
                
                # Update costmap layer
                self.update_layer_from_cluster(cluster, cluster_class)
                
                # Update height map
                self.update_height_map(cluster)
    
    def radar_points_callback(self, msg):
        """Process radar point cloud data"""
        # This is just a placeholder - we're mainly using the cluster data
        self.get_logger().debug('Received radar points')
    
    def radar_clusters_callback(self, msg):
        """Process radar cluster data"""
        self.get_logger().info(f'Received {len(msg.markers)} radar clusters')
        
        # Extract clusters from markers
        clusters = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE:
                cluster = self.extract_cluster_info(marker)
                cluster['source'] = 'radar'
                
                # CRITICAL: Treat ALL radar detections as high-priority dynamic objects
                # Set extremely high velocity to ensure they're classified as dynamic
                cluster['velocity'] = 20.0  # Extremely high velocity to ensure dynamic classification
                
                # Set maximum confidence for all radar clusters
                cluster['confidence'] = 1.0  # Maximum confidence
                
                # Get dimensions
                width = marker.scale.x
                length = marker.scale.y
                height = marker.scale.z
                
                # CRITICAL: Treat EVERY radar detection as a car
                # This ensures maximum visibility for all detected objects
                is_car = True
                
                # Log detection based on dimensions
                if max(width, length) >= 1.0:
                    self.get_logger().info(f'LARGE OBJECT DETECTED: size=({width:.2f}x{length:.2f}x{height:.2f}), position=({cluster["centroid"][0]:.2f}, {cluster["centroid"][1]:.2f})')
                else:
                    self.get_logger().info(f'SMALL OBJECT DETECTED: size=({width:.2f}x{length:.2f}x{height:.2f}), position=({cluster["centroid"][0]:.2f}, {cluster["centroid"][1]:.2f})')
                
                # Determine if object is seen from side (width > length)
                is_side_view = width > length
                cluster['is_side_view'] = is_side_view
                
                cluster['is_car'] = is_car
                clusters.append(cluster)
        
        # Update radar clusters
        with self.costmap_lock:
            self.radar_clusters = clusters
            
            # Process each cluster
            for cluster in clusters:
                # All radar clusters are classified as dynamic objects
                cluster_class = 'dynamic'
                
                # Update costmap layer with higher priority
                self.update_layer_from_cluster(cluster, cluster_class)
                
                # Track dynamic object
                if self.motion_prediction:
                    self.track_dynamic_object(cluster)
                    
            # Log total number of dynamic objects being tracked
            self.get_logger().info(f'Currently tracking {len(self.dynamic_objects)} dynamic objects')
    
    def classify_cluster(self, cluster):
        """Classify a cluster based on its properties"""
        # Extract dimensions
        width = cluster['dimensions'][0]
        length = cluster['dimensions'][1]
        height = cluster['max_z'] - cluster['min_z'] if 'max_z' in cluster and 'min_z' in cluster else 0
        
        # Get velocity if available - but don't use it as a threshold
        velocity = cluster.get('velocity', 0.0)
        
        # Calculate distance from sensor (approximation)
        distance = math.sqrt(cluster['centroid'][0]**2 + cluster['centroid'][1]**2)
        
        # Scale detection thresholds based on distance
        # Objects appear smaller when further away, so we reduce the minimum width threshold
        distance_factor = max(1.0, min(3.0, 1.0 + distance / 30.0))  # Scale factor increases with distance
        scaled_car_detection_width = self.car_detection_width / distance_factor
        
        # Check if this is from radar (all radar objects are considered important)
        if cluster.get('source') == 'radar':
            # All radar detections are marked as cars for better visibility
            cluster['is_car'] = True
            # Mark as dynamic to ensure high visibility
            return 'dynamic'
        
        # Special handling for low-profile cars
        if self.enhance_low_cars:
            # Check if object has car-like dimensions (wide but low)
            is_car_like = (width >= scaled_car_detection_width or length >= scaled_car_detection_width) and \
                          height >= self.low_car_height_threshold and \
                          height <= self.ground_height_threshold * 3  # Increased from 2 to 3 for better detection
            
            # Additional check for very low but wide objects that might be cars
            is_very_low_car = (width >= scaled_car_detection_width * 1.5 or length >= scaled_car_detection_width * 1.5) and \
                             height >= self.low_car_height_threshold * 0.5  # Even lower threshold for wide objects
            
            # Special handling for distant objects - be more lenient with dimensions
            if distance > 20.0:  # For objects more than 20m away
                is_distant_car = (width >= scaled_car_detection_width * 0.7 or length >= scaled_car_detection_width * 0.7) and \
                                height >= self.low_car_height_threshold * 0.3
                if is_distant_car:
                    self.get_logger().info(f"Detected distant car: w={width:.2f}m, l={length:.2f}m, h={height:.2f}m, dist={distance:.2f}m")
                    cluster['is_car'] = True
                    return 'obstacle'
            
            if is_car_like or is_very_low_car:
                self.get_logger().info(f"Detected low-profile car: w={width:.2f}m, l={length:.2f}m, h={height:.2f}m, dist={distance:.2f}m")
                # Mark as car for special handling
                cluster['is_car'] = True
                # Return as obstacle instead of ground
                return 'obstacle'
        
        # Check for dynamic objects - REMOVE VELOCITY THRESHOLD
        # Consider ANY object with ANY velocity as potentially dynamic
        if velocity > 0.0 or 'is_moving' in cluster and cluster['is_moving']:
            # Any moving object is dynamic
            return 'dynamic'
        
        # If object has significant size in any dimension, treat as dynamic
        # Scale the size threshold based on distance but keep it very low
        min_size_threshold = 0.1 / distance_factor  # Reduced from 0.3 to 0.1
        if max(width, length, height) >= min_size_threshold:
            # Any significant object is treated as dynamic for better visibility
            return 'dynamic'
        
        # Very inclusive ground classification
        if height < self.ground_height_threshold:
            # Check if this might be a low car based on width/length ratio
            if width > scaled_car_detection_width * 0.8 or length > scaled_car_detection_width * 0.8:
                self.get_logger().info(f"Potential low car detected: w={width:.2f}m, l={length:.2f}m, h={height:.2f}m, dist={distance:.2f}m")
                # Mark as obstacle instead of ground
                return 'obstacle'
            # Any low object is ground
            return 'ground'
        elif height > width * 1.5:
            # Tall and narrow objects are likely vegetation
            return 'vegetation'
        elif width > self.building_width_threshold and height > 2.0:
            # Very wide and tall objects are buildings
            return 'building'
        else:
            # Everything else is an obstacle
            return 'obstacle'
    
    def update_layer_from_cluster(self, cluster, cluster_class):
        """Update costmap layer from a classified cluster"""
        # Get cluster bounds in grid coordinates
        min_x, min_y = self.world_to_map(cluster['min_x'], cluster['min_y'])
        max_x, max_y = self.world_to_map(cluster['max_x'], cluster['max_y'])
        
        if min_x is None or min_y is None or max_x is None or max_y is None:
            return
        
        # Calculate distance from sensor (approximation)
        distance = math.sqrt(cluster['centroid'][0]**2 + cluster['centroid'][1]**2)
        
        # Set cost in appropriate layer
        # Increase cost for all detected objects to ensure visibility
        base_cost = int(min(100, cluster['confidence'] * 100))
        
        # Special handling for cars and radar objects
        is_car = cluster.get('is_car', False)
        is_radar = cluster.get('source') == 'radar'
        
        # Special handling for low-profile cars
        is_low_car = False
        if self.enhance_low_cars and 'dimensions' in cluster:
            width = cluster['dimensions'][0]
            length = cluster['dimensions'][1]
            height = cluster['max_z'] - cluster['min_z'] if 'max_z' in cluster and 'min_z' in cluster else 0
            
            # Check if object has car-like dimensions (wide but low)
            is_low_car = (width >= self.car_detection_width or length >= self.car_detection_width) and \
                         height >= self.low_car_height_threshold and \
                         height <= self.ground_height_threshold * 3
            
            # Additional check for very low but wide objects
            is_very_low_car = (width >= self.car_detection_width * 1.5 or length >= self.car_detection_width * 1.5) and \
                             height >= self.low_car_height_threshold * 0.5
            
            if (is_low_car or is_very_low_car) and not is_car:
                is_car = True
                cluster['is_car'] = True
                self.get_logger().info(f"Enhanced low-profile car detection: w={width:.2f}m, l={length:.2f}m, h={height:.2f}m, dist={distance:.2f}m")
        
        # Set cost based on object type - ENSURE ALL OBJECTS HAVE HIGH VISIBILITY
        # All objects get high cost to ensure visibility
        cost = 100  # Maximum possible cost for all objects
        
        if is_car or is_radar:
            self.get_logger().info(f'Setting maximum cost (100) for car/radar at ({cluster["centroid"][0]:.2f}, {cluster["centroid"][1]:.2f}), dist={distance:.2f}m')
        else:
            self.get_logger().info(f'Setting high cost (100) for object at ({cluster["centroid"][0]:.2f}, {cluster["centroid"][1]:.2f}), dist={distance:.2f}m')
        
        current_time = time.time()
        
        # Use configurable expansion radius based on object type
        # ENSURE MINIMUM EXPANSION FOR ALL OBJECTS
        if is_car or is_radar:
            # Use car expansion radius parameter - ensure it's at least 2
            radius_boost = max(2, int(self.car_expansion_radius))
            
            # For distant cars, ensure a minimum expansion radius
            if distance > 30.0:  # For objects more than 30m away
                radius_boost = max(radius_boost, 4)  # Ensure at least 4 cells expansion for distant cars
                self.get_logger().info(f'Distant car/radar object detected at {distance:.2f}m - using enhanced radius_boost={radius_boost}')
            elif distance > 20.0:  # For objects more than 20m away
                radius_boost = max(radius_boost, 3)  # Ensure at least 3 cells expansion for medium-distance cars
                
            # Check if the object is likely seen from the side (width > length)
            is_side_view = cluster.get('is_side_view', False)
            if is_side_view:
                # Add a boost for side-view objects
                radius_boost += 2  # Increased from 1 to 2
                self.get_logger().info(f'Object detected from SIDE VIEW - using radius_boost={radius_boost}')
            
            # Add special logging
            height = cluster['max_z'] - cluster['min_z'] if 'max_z' in cluster and 'min_z' in cluster else 0
            width = max(cluster['dimensions'][0], cluster['dimensions'][1]) if 'dimensions' in cluster else 0
            self.get_logger().info(f'Car/radar object detected with h={height:.2f}m, w={width:.2f}m - using radius_boost={radius_boost}')
        elif cluster_class == 'dynamic':
            # Use dynamic expansion radius parameter - ensure it's at least 2
            radius_boost = max(2, int(self.dynamic_expansion_radius))
            
            # For distant dynamic objects, increase visibility
            if distance > 20.0:
                radius_boost = max(radius_boost, 3)  # Ensure at least 3 cells expansion for distant dynamic objects
        else:
            # Use obstacle expansion radius parameter for all other objects - ensure it's at least 1
            radius_boost = max(1, int(self.obstacle_expansion_radius))
            
            # For distant obstacles, ensure minimum visibility
            if distance > 25.0:
                radius_boost = max(radius_boost, 2)  # Ensure at least 2 cells expansion for distant obstacles
        
        # Special handling for low cars - make them visible with sufficient expansion
        if is_low_car or is_car:
            # Use car expansion radius - ensure it's at least 2
            car_radius = max(2, int(self.car_expansion_radius))
            
            # For distant cars, ensure a minimum expansion radius
            if distance > 30.0:
                car_radius = max(car_radius, 4)  # Ensure at least 4 cells expansion for distant cars
            elif distance > 20.0:
                car_radius = max(car_radius, 3)  # Ensure at least 3 cells expansion for medium-distance cars
                
            self.get_logger().info(f'Car detected at distance {distance:.2f}m - using car expansion radius={car_radius}')
            
            # Explicitly update the obstacle layer for low cars - only the actual car and minimal surroundings
            for y in range(min_y - car_radius, max_y + car_radius + 1):
                for x in range(min_x - car_radius, max_x + car_radius + 1):
                    if 0 <= x < self.map_width and 0 <= y < self.map_height:
                        # Calculate distance from cluster bounds
                        if min_x <= x <= max_x and min_y <= y <= max_y:
                            # Inside the car's actual bounds - maximum cost
                            self.layers['obstacle'][y, x] = 100
                        else:
                            # Outside bounds - calculate distance-based cost with less steep falloff
                            dx = max(0, min_x - x, x - max_x)
                            dy = max(0, min_y - y, y - max_y)
                            distance_to_edge = math.sqrt(dx*dx + dy*dy)
                            
                            # Use a higher minimum cost for better visibility
                            min_cost_factor = 0.6  # Increased from 0.5 to 0.6
                            # Use linear distance for less steep falloff
                            falloff = min(1.0, distance_to_edge/car_radius)
                            cell_cost = int(100 * max(min_cost_factor, 1.0 - falloff))
                            self.layers['obstacle'][y, x] = max(self.layers['obstacle'][y, x], cell_cost)
        
        # Expand the object's footprint with sufficient expansion
        for y in range(min_y - radius_boost, max_y + radius_boost + 1):
            for x in range(min_x - radius_boost, max_x + radius_boost + 1):
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    # For points outside the original bounds, reduce cost based on distance
                    if min_x <= x <= max_x and min_y <= y <= max_y:
                        cell_cost = cost
                    else:
                        # Calculate distance from cluster bounds
                        dx = max(0, min_x - x, x - max_x)
                        dy = max(0, min_y - y, y - max_y)
                        distance_to_edge = math.sqrt(dx*dx + dy*dy)
                        
                        # Use less steep falloff for all objects
                        # Reduce cost based on distance, but keep a higher minimum
                        min_cost_factor = 0.5  # Increased from 0.3 to 0.5
                        if is_car or is_radar:
                            # Higher minimum cost factor
                            if is_side_view:
                                min_cost_factor = 0.7  # Increased from 0.5 to 0.7
                            else:
                                min_cost_factor = 0.6  # Increased from 0.4 to 0.6
                        elif cluster_class == 'dynamic':
                            min_cost_factor = 0.6  # Increased from 0.4 to 0.6
                        
                        # Use linear distance for less steep falloff
                        falloff = min(1.0, distance_to_edge/radius_boost)
                        cell_cost = int(cost * max(min_cost_factor, 1.0 - falloff))
                    
                    # Update layer with maximum cost
                    self.layers[cluster_class][y, x] = max(self.layers[cluster_class][y, x], cell_cost)
                    
                    # CRITICAL: Mark EVERY object in ALL layers with high cost
                    # This ensures it will be included in the binary map regardless of classification
                    
                    # Mark in ground layer with significant cost
                    ground_cost = max(60, int(cell_cost * 0.7))  # Increased from 50 to 60 and from 0.6 to 0.7
                    self.layers['ground'][y, x] = max(self.layers['ground'][y, x], ground_cost)
                    
                    # Mark in obstacle layer
                    obstacle_cost = max(70, int(cell_cost * 0.8))  # Increased from 50 to 70 and from 0.6 to 0.8
                    self.layers['obstacle'][y, x] = max(self.layers['obstacle'][y, x], obstacle_cost)
                    
                    # Mark in dynamic layer
                    dynamic_cost = max(80, int(cell_cost * 0.9))  # Increased from 50 to 80 and from 0.6 to 0.9
                    self.layers['dynamic'][y, x] = max(self.layers['dynamic'][y, x], dynamic_cost)
                    
                    # Update confidence and timestamp
                    self.confidence_map[y, x] = max(self.confidence_map[y, x], cluster['confidence'])
                    self.cell_update_time[y, x] = current_time
    
    def update_height_map(self, cluster):
        """Update height map from cluster"""
        if not self.enable_3d_visualization:
            return
            
        # Get cluster bounds in grid coordinates
        min_x, min_y = self.world_to_map(cluster['min_x'], cluster['min_y'])
        max_x, max_y = self.world_to_map(cluster['max_x'], cluster['max_y'])
        
        if min_x is None or min_y is None or max_x is None or max_y is None:
            return
        
        # Update height map with maximum height
        height = cluster['max_z']
        
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                self.height_map[y, x] = max(self.height_map[y, x], height)
    
    def track_dynamic_object(self, cluster):
        """Track dynamic objects for motion prediction"""
        # Simple object tracking based on position
        obj_id = f"{cluster['source']}_{cluster['id']}"
        centroid = cluster['centroid']
        
        if obj_id in self.dynamic_objects:
            # Object already tracked - update and calculate velocity
            prev = self.dynamic_objects[obj_id]
            dt = time.time() - prev['timestamp']
            
            if dt > 0:
                vx = (centroid[0] - prev['x']) / dt
                vy = (centroid[1] - prev['y']) / dt
                
                # Update with new position and calculated velocity
                self.dynamic_objects[obj_id] = {
                    'x': centroid[0],
                    'y': centroid[1],
                    'z': centroid[2],
                    'vx': vx,
                    'vy': vy,
                    'confidence': cluster['confidence'],
                    'timestamp': time.time()
                }
        else:
            # New object - initialize tracking
            self.dynamic_objects[obj_id] = {
                'x': centroid[0],
                'y': centroid[1],
                'z': centroid[2],
                'vx': 0.0,
                'vy': 0.0,
                'confidence': cluster['confidence'],
                'timestamp': time.time()
            }
    
    def apply_temporal_filtering(self):
        """Apply temporal filtering to costmap layers"""
        with self.costmap_lock:
            current_time = time.time()
            
            # Calculate decay rates from decay time parameters
            # decay_rate = exp(-time_diff/decay_time) gives proper exponential decay
            # Lower decay_time means faster decay rate (closer to 0)
            # Layer-specific decay rates based on decay_time and dynamic_decay_time parameters
            time_diff = current_time - self.last_update_time
            
            # Calculate decay rates for each layer type
            # decay_rate closer to 0 means faster decay
            decay_rates = {
                'ground': math.exp(-time_diff / self.decay_time),
                'obstacle': math.exp(-time_diff / self.decay_time),
                'vegetation': math.exp(-time_diff / self.decay_time),
                'building': math.exp(-time_diff / self.decay_time),
                'dynamic': math.exp(-time_diff / self.dynamic_decay_time)  # Dynamic objects decay faster
            }
            
            self.get_logger().debug(f'Temporal filtering: time_diff={time_diff:.3f}s, decay_time={self.decay_time:.3f}s, dynamic_decay_time={self.dynamic_decay_time:.3f}s')
            self.get_logger().debug(f'Decay rates: ground={decay_rates["ground"]:.3f}, obstacle={decay_rates["obstacle"]:.3f}, dynamic={decay_rates["dynamic"]:.3f}')
            
            # Apply decay to each layer
            for layer_name, decay_rate in decay_rates.items():
                # Ensure decay_rate is between 0 and 1
                decay_rate = max(0.0, min(1.0, decay_rate))
                
                # Apply decay to entire layer
                self.layers[layer_name] = (self.layers[layer_name] * decay_rate).astype(np.int8)
            
            # Update dynamic object positions based on tracked velocities
            if self.motion_prediction and self.dynamic_objects:
                # Create a copy of the dynamic layer for prediction
                dynamic_layer_copy = self.layers['dynamic'].copy()
                
                for obj_id, obj_data in self.dynamic_objects.items():
                    # Skip old objects
                    if current_time - obj_data['timestamp'] > self.dynamic_decay_time * 5:  # Use dynamic_decay_time parameter
                        continue
                        
                    # Predict new position based on velocity and time difference
                    dt = current_time - obj_data['timestamp']
                    new_x = obj_data['x'] + (obj_data['vx'] * dt)
                    new_y = obj_data['y'] + (obj_data['vy'] * dt)
                    
                    # Update dynamic layer with predicted position
                    grid_x, grid_y = self.world_to_map(new_x, new_y)
                    
                    if grid_x is not None and grid_y is not None:
                        # Mark predicted position with slightly lower confidence
                        confidence = obj_data['confidence'] * decay_rates['dynamic']  # Apply dynamic decay
                        cost = int(min(100, confidence * 100))
                        
                        # Create a small area around the predicted position
                        radius = int(1.0 / self.map_resolution)  # 1-meter radius
                        for y in range(grid_y - radius, grid_y + radius + 1):
                            for x in range(grid_x - radius, grid_x + radius + 1):
                                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                                    dist = math.sqrt((x - grid_x)**2 + (y - grid_y)**2)
                                    if dist <= radius:
                                        # Decrease cost with distance from center
                                        cell_cost = int(cost * (1.0 - dist/radius))
                                        dynamic_layer_copy[y, x] = max(
                                            dynamic_layer_copy[y, x],
                                            cell_cost
                                        )
                
                # Update dynamic layer with predictions
                self.layers['dynamic'] = dynamic_layer_copy
            
            # Clean up old dynamic objects
            current_objects = {}
            for obj_id, obj_data in self.dynamic_objects.items():
                if current_time - obj_data['timestamp'] <= self.dynamic_decay_time * 5:  # Use dynamic_decay_time parameter
                    current_objects[obj_id] = obj_data
            
            self.dynamic_objects = current_objects
            self.last_update_time = current_time
    
    def compute_combined_costmap(self):
        """Compute combined costmap from all layers"""
        # Reset combined layer
        self.layers['combined'].fill(0)
        
        # Combine layers using weights
        for layer_name, weight in self.layer_weights.items():
            self.layers['combined'] = np.maximum(
                self.layers['combined'],
                np.minimum((self.layers[layer_name] * weight).astype(np.int8), 100)
            )
        
        return self.layers['combined']
    
    def create_occupancy_grid_msg(self, layer_name):
        """Create OccupancyGrid message from layer"""
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.map_frame
        
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        
        grid_msg.data = self.layers[layer_name].flatten().tolist()
        
        return grid_msg
    
    def publish_costmap_layers(self):
        """Publish all visible costmap layers"""
        with self.costmap_lock:
            # Compute combined costmap
            self.compute_combined_costmap()
            
            # Publish each visible layer
            for layer_name, visible in self.layer_visibility.items():
                if visible:
                    grid_msg = self.create_occupancy_grid_msg(layer_name)
                    self.layer_publishers[layer_name].publish(grid_msg)
            
            # Publish binary map if enabled
            if self.enable_binary_output and hasattr(self, 'binary_map_pub'):
                self.publish_binary_map()
            
            # Publish semantic markers
            self.publish_semantic_markers()
            
            # Publish height map if enabled
            if self.enable_3d_visualization:
                self.publish_height_map()
    
    def publish_semantic_markers(self):
        """Publish semantic markers for visualization"""
        marker_array = MarkerArray()
        
        # Define semantic classes with distinct visual properties
        semantic_classes = {
            'ground': {'color': [0.2, 0.8, 0.2, 0.7], 'height': 0.1},
            'obstacle': {'color': [0.8, 0.2, 0.2, 0.9], 'height': 1.0},
            'vegetation': {'color': [0.0, 0.6, 0.0, 0.8], 'height': 1.5},
            'building': {'color': [0.6, 0.6, 0.6, 1.0], 'height': 2.5},
            'dynamic': {'color': [1.0, 0.6, 0.0, 1.0], 'height': 1.2},
            'vehicle': {'color': [1.0, 0.0, 1.0, 1.0], 'height': 1.8}  # Magenta for vehicles
        }
        
        # Process all clusters
        marker_id = 0
        all_clusters = self.lidar_clusters + self.radar_clusters
        
        for cluster in all_clusters:
            # Check if cluster is a vehicle based on dimensions
            is_vehicle = False
            height = cluster['max_z'] - cluster['min_z']
            width = max(cluster['dimensions'][0], cluster['dimensions'][1])
            length = max(cluster['dimensions'])
            
            # Vehicle detection based on typical dimensions
            if 1.0 <= height <= 2.5 and 1.5 <= width <= 2.5 and 2.0 <= length <= 6.0:
                is_vehicle = True
            
            # Classify cluster
            if is_vehicle:
                cluster_class = 'vehicle'
            elif cluster['source'] == 'radar':
                cluster_class = 'dynamic'
            else:
                cluster_class = self.classify_cluster(cluster)
            
            visual_props = semantic_classes.get(cluster_class, semantic_classes['obstacle'])
            
            # Create cube marker for the cluster
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "semantic_clusters"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position to cluster centroid
            marker.pose.position.x = cluster['centroid'][0]
            marker.pose.position.y = cluster['centroid'][1]
            marker.pose.position.z = cluster['centroid'][2] / 2.0  # Half height for ground alignment
            
            # Set size based on cluster dimensions
            marker.scale.x = cluster['dimensions'][0]
            marker.scale.y = cluster['dimensions'][1]
            marker.scale.z = visual_props['height']
            
            # Set color based on semantic class
            marker.color.r = visual_props['color'][0]
            marker.color.g = visual_props['color'][1]
            marker.color.b = visual_props['color'][2]
            marker.color.a = visual_props['color'][3]
            
            # Set lifetime based on marker_lifetime parameter
            # Convert from seconds to ROS duration
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = int(self.marker_lifetime * 1e9)  # Convert to nanoseconds
            
            marker_array.markers.append(marker)
            
            # Add text label with classification if enabled
            if self.enable_text_labels:
                text_marker = Marker()
                text_marker.header = marker.header
                text_marker.ns = "cluster_labels"
                text_marker.id = marker_id
                marker_id += 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = cluster['centroid'][0]
                text_marker.pose.position.y = cluster['centroid'][1]
                text_marker.pose.position.z = cluster['centroid'][2] + 0.5  # Above the cluster
                text_marker.text = f"{cluster_class}"
                text_marker.scale.z = 0.5  # Text size
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                # Set lifetime based on marker_lifetime parameter
                text_marker.lifetime.sec = 0
                text_marker.lifetime.nanosec = int(self.marker_lifetime * 1e9)  # Convert to nanoseconds
                
                marker_array.markers.append(text_marker)
        
        # Publish markers
        self.semantic_marker_pub.publish(marker_array)
    
    def publish_height_map(self):
        """Publish height map as point cloud"""
        # TODO: Implement height map visualization
        pass
    
    def publish_binary_map(self):
        """Create and publish binary (black/white) map for navigation"""
        # Create binary map from combined costmap
        binary_map = np.zeros_like(self.layers['combined'])
        
        # CRITICAL: Include ANY non-zero value from ANY layer
        # This ensures ALL detected objects appear in the binary map regardless of classification
        total_cells = 0
        
        # Use an extremely low threshold to ensure all objects are included
        distant_threshold = 1  # Any value above 0 will be included
        normal_threshold = 10  # Very low threshold for normal detection
        
        self.get_logger().info(f'Using ultra-inclusive binary map thresholds: normal={normal_threshold}, distant={distant_threshold}')
        
        # First include obstacle layer cells (which include low-profile cars) - highest priority
        obstacle_layer = self.layers['obstacle']
        binary_map[obstacle_layer >= normal_threshold] = self.occupied_value
        obstacle_cells = np.sum(obstacle_layer >= normal_threshold)
        
        # Also include distant obstacles with lower threshold
        distant_mask = (obstacle_layer >= distant_threshold) & (obstacle_layer < normal_threshold) & (binary_map != self.occupied_value)
        binary_map[distant_mask] = self.occupied_value
        distant_obstacle_cells = np.sum(distant_mask)
        
        total_cells += obstacle_cells + distant_obstacle_cells
        self.get_logger().info(f'Binary map: Obstacle layer cells: {obstacle_cells} normal + {distant_obstacle_cells} distant')
        
        # Next include dynamic objects (high priority)
        dynamic_layer = self.layers['dynamic']
        mask = (dynamic_layer >= normal_threshold) & (binary_map != self.occupied_value)
        binary_map[mask] = self.occupied_value
        dynamic_cells = np.sum(mask)
        
        # Include distant dynamic objects with lower threshold
        distant_mask = (dynamic_layer >= distant_threshold) & (dynamic_layer < normal_threshold) & (binary_map != self.occupied_value)
        binary_map[distant_mask] = self.occupied_value
        distant_dynamic_cells = np.sum(distant_mask)
        
        total_cells += dynamic_cells + distant_dynamic_cells
        self.get_logger().info(f'Binary map: Dynamic layer cells: {dynamic_cells} normal + {distant_dynamic_cells} distant')
        
        # Include ALL vegetation layer cells
        vegetation_layer = self.layers['vegetation']
        mask = (vegetation_layer >= normal_threshold) & (binary_map != self.occupied_value)
        binary_map[mask] = self.occupied_value
        vegetation_cells = np.sum(mask)
        
        # Include distant vegetation with lower threshold
        distant_mask = (vegetation_layer >= distant_threshold) & (vegetation_layer < normal_threshold) & (binary_map != self.occupied_value)
        binary_map[distant_mask] = self.occupied_value
        distant_vegetation_cells = np.sum(distant_mask)
        
        total_cells += vegetation_cells + distant_vegetation_cells
        self.get_logger().info(f'Binary map: Vegetation layer cells: {vegetation_cells} normal + {distant_vegetation_cells} distant')
        
        # Include ALL building layer cells
        building_layer = self.layers['building']
        mask = (building_layer >= normal_threshold) & (binary_map != self.occupied_value)
        binary_map[mask] = self.occupied_value
        building_cells = np.sum(mask)
        
        # Include distant buildings with lower threshold
        distant_mask = (building_layer >= distant_threshold) & (building_layer < normal_threshold) & (binary_map != self.occupied_value)
        binary_map[distant_mask] = self.occupied_value
        distant_building_cells = np.sum(distant_mask)
        
        total_cells += building_cells + distant_building_cells
        self.get_logger().info(f'Binary map: Building layer cells: {building_cells} normal + {distant_building_cells} distant')
        
        # Only include ground if configured to do so
        if self.include_ground_in_binary and self.ground_as_occupied:
            ground_layer = self.layers['ground']
            mask = (ground_layer >= normal_threshold) & (binary_map != self.occupied_value)
            binary_map[mask] = self.occupied_value
            ground_cells = np.sum(mask)
            
            # Include distant ground with lower threshold
            distant_mask = (ground_layer >= distant_threshold) & (ground_layer < normal_threshold) & (binary_map != self.occupied_value)
            binary_map[distant_mask] = self.occupied_value
            distant_ground_cells = np.sum(distant_mask)
            
            total_cells += ground_cells + distant_ground_cells
            self.get_logger().info(f'Binary map: Ground layer cells: {ground_cells} normal + {distant_ground_cells} distant')
        
        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.map_frame
        
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        
        # Convert binary map to list with appropriate values
        grid_msg.data = binary_map.flatten().tolist()
        
        # Publish binary map
        self.binary_map_pub.publish(grid_msg)
        
        # Log the number of occupied cells for debugging
        occupied_cells = np.sum(binary_map == self.occupied_value)
        self.get_logger().info(f'Binary map published with {occupied_cells} total occupied cells from {total_cells} detected cells')
    
    def toggle_layer_callback(self, request, response):
        """Service callback to toggle layer visibility"""
        layer_name = request.data
        if layer_name in self.layer_visibility:
            self.layer_visibility[layer_name] = not self.layer_visibility[layer_name]
            response.success = True
            response.message = f"Layer {layer_name} visibility toggled to {self.layer_visibility[layer_name]}"
        else:
            response.success = False
            response.message = f"Layer {layer_name} not found"
        
        return response
    
    def save_maps_callback(self, request, response):
        """Service callback to save maps on demand"""
        if request.data:  # If request is true, save maps
            try:
                # Save maps immediately
                self.get_logger().info("Saving maps on demand...")
                self.save_maps()
                response.success = True
                response.message = f"Maps saved to {self.save_directory}"
            except Exception as e:
                response.success = False
                response.message = f"Failed to save maps: {str(e)}"
        else:
            # Return current status
            response.success = True
            if self.enable_map_saving:
                response.message = f"Map saving is enabled. Maps are saved every {self.save_interval} seconds to {self.save_directory}"
            else:
                response.message = "Map saving is disabled"
        
        return response
    
    def save_maps(self):
        """Save maps as PGM files"""
        if not self.enable_map_saving:
            return
        
        try:
            # Create timestamp for filenames
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_path = os.path.join(self.save_directory, timestamp)
            
            with self.costmap_lock:
                # Save binary map if enabled
                if self.save_binary_map and hasattr(self, 'binary_map_pub'):
                    # Create binary map
                    binary_map = np.zeros_like(self.layers['combined'])
                    
                    # CRITICAL: Include ANY non-zero value from ANY layer
                    # This ensures ALL detected objects appear in the binary map regardless of classification
                    total_cells = 0
                    
                    # First include obstacle layer cells (which include low-profile cars) - highest priority
                    obstacle_layer = self.layers['obstacle']
                    binary_map[obstacle_layer > 0] = self.occupied_value
                    obstacle_cells = np.sum(obstacle_layer > 0)
                    total_cells += obstacle_cells
                    self.get_logger().info(f'Save map: Obstacle layer cells: {obstacle_cells}')
                    
                    # Next include dynamic objects (high priority)
                    dynamic_layer = self.layers['dynamic']
                    mask = (dynamic_layer > 0) & (binary_map != self.occupied_value)
                    binary_map[mask] = self.occupied_value
                    dynamic_cells = np.sum(mask)
                    total_cells += dynamic_cells
                    self.get_logger().info(f'Save map: Dynamic layer cells: {dynamic_cells}')
                    
                    # Include ALL vegetation layer cells
                    vegetation_layer = self.layers['vegetation']
                    mask = (vegetation_layer > 0) & (binary_map != self.occupied_value)
                    binary_map[mask] = self.occupied_value
                    vegetation_cells = np.sum(mask)
                    total_cells += vegetation_cells
                    self.get_logger().info(f'Save map: Vegetation layer cells: {vegetation_cells}')
                    
                    # Include ALL building layer cells
                    building_layer = self.layers['building']
                    mask = (building_layer > 0) & (binary_map != self.occupied_value)
                    binary_map[mask] = self.occupied_value
                    building_cells = np.sum(mask)
                    total_cells += building_cells
                    self.get_logger().info(f'Save map: Building layer cells: {building_cells}')
                    
                    # Save binary map
                    self.save_pgm(binary_map, f"{base_path}_binary.pgm", self.occupied_value)
                    self.get_logger().info(f"Binary map saved to {base_path}_binary.pgm with {np.sum(binary_map == self.occupied_value)} total occupied cells")
                
                # Save combined map if enabled
                if self.save_combined_map:
                    # Compute combined map
                    self.compute_combined_costmap()
                    # Save combined map
                    self.save_pgm(self.layers['combined'], f"{base_path}_combined.pgm")
                    print(f"Combined map saved to {base_path}_combined.pgm")
                    self.get_logger().info(f"Combined map saved to {base_path}_combined.pgm")
                
                # Save individual layer maps if enabled
                if self.save_layer_maps:
                    for layer_name in ['ground', 'obstacle', 'vegetation', 'building', 'dynamic']:
                        # Save layer map
                        self.save_pgm(self.layers[layer_name], f"{base_path}_{layer_name}.pgm")
                        self.get_logger().info(f"{layer_name.capitalize()} layer map saved to {base_path}_{layer_name}.pgm")
                        
            # Update last save time
            self.last_save_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error saving maps: {str(e)}")
    
    def save_pgm(self, map_data, filename, max_value=100):
        """Save map data as PGM file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            # Open file for writing
            with open(filename, 'wb') as f:
                # Write PGM header
                f.write(f"P5\n".encode())
                f.write(f"# Created by Semantic Costmap Visualizer\n".encode())
                f.write(f"{self.map_width} {self.map_height}\n".encode())
                f.write(f"{max_value}\n".encode())
                
                # Write map data
                # Convert 0-100 scale to 0-255 scale and flip vertically for PGM format
                data = ((map_data.astype(np.float32) / max_value) * 255).astype(np.uint8)
                data = np.flipud(data)  # Flip vertically
                f.write(data.tobytes())
                
            # Also save a YAML metadata file for the map
            yaml_filename = filename.replace('.pgm', '.yaml')
            with open(yaml_filename, 'w') as f:
                f.write(f"image: {os.path.basename(filename)}\n")
                f.write(f"resolution: {self.map_resolution}\n")
                f.write(f"origin: [{self.map_origin_x}, {self.map_origin_y}, 0.0]\n")
                f.write(f"negate: 0\n")
                f.write(f"occupied_thresh: {self.binary_threshold}\n")
                f.write(f"free_thresh: 0.196\n")  # Standard ROS value
                
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving PGM file {filename}: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = SemanticCostmapVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 