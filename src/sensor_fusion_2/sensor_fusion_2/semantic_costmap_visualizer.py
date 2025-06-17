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

class SemanticCostmapVisualizer(Node):
    """
    Creates a multi-layer semantic costmap visualization from LiDAR and radar data.
    
    This node:
    1. Subscribes to LiDAR point clouds and cluster data
    2. Classifies clusters into semantic categories (ground, obstacle, vegetation, building, dynamic)
    3. Creates separate costmap layers for each semantic category
    4. Provides interactive visualization with layer toggling
    5. Integrates with RViz for enhanced visualization
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
        self.enable_3d_visualization = self.get_parameter('enable_3d_visualization').value
        self.enable_text_labels = self.get_parameter('enable_text_labels').value
        
        # Add new parameters for classification
        self.ground_height_threshold = self.get_parameter('ground_height_threshold').value
        self.vegetation_height_ratio = self.get_parameter('vegetation_height_ratio').value
        self.building_width_threshold = self.get_parameter('building_width_threshold').value
        self.dynamic_velocity_threshold = self.get_parameter('dynamic_velocity_threshold').value
        
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
        
        # Create timers
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_costmap_layers
        )
        
        if self.temporal_filtering:
            self.filtering_timer = self.create_timer(
                0.1,  # 10 Hz
                self.apply_temporal_filtering
            )
        
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
        self.get_logger().info(f'Map resolution: {self.map_resolution} meters/cell')
    
    def add_dynamic_parameters(self):
        """Add parameters with descriptors for dynamic reconfigure"""
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
                    to_value=30.0, 
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
            1.0,
            ParameterDescriptor(
                description='Base time for cell decay in seconds',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1, 
                    to_value=10.0, 
                    step=0.1
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
                    from_value=0.1, 
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
                    self.filtering_timer = self.create_timer(
                        0.1,  # 10 Hz
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
            
            if param.name.endswith('_weight'):
                weights_changed = True
        
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
        self.get_logger().debug(f'Received {len(msg.markers)} radar clusters')
        
        # Extract clusters from markers
        clusters = []
        for marker in msg.markers:
            if marker.type == Marker.CUBE:
                cluster = self.extract_cluster_info(marker)
                cluster['source'] = 'radar'
                
                # Radar clusters are likely dynamic objects
                cluster['velocity'] = 1.0  # Assume some velocity for radar objects
                clusters.append(cluster)
        
        # Update radar clusters
        with self.costmap_lock:
            self.radar_clusters = clusters
            
            # Process each cluster
            for cluster in clusters:
                # Radar clusters are typically dynamic objects
                cluster_class = 'dynamic'
                
                # Update costmap layer
                self.update_layer_from_cluster(cluster, cluster_class)
                
                # Track dynamic object
                if self.motion_prediction:
                    self.track_dynamic_object(cluster)
    
    def classify_cluster(self, cluster):
        """Classify LiDAR cluster based on properties"""
        # Extract cluster features
        height = cluster['max_z'] - cluster['min_z']
        width = max(cluster['dimensions'][0], cluster['dimensions'][1])
        length = cluster['dimensions'][0]  # Assuming x is the forward direction
        height_ratio = height / width if width > 0 else 0
        
        # Check for velocity first - moving objects are likely vehicles
        if cluster['velocity'] > self.dynamic_velocity_threshold:
            return 'dynamic'
            
        # Car detection based on typical dimensions
        # Most cars are 1.5-2m high, 1.7-2m wide, and 4-5m long
        if 1.0 <= height <= 2.5 and 1.5 <= width <= 2.5 and 2.0 <= length <= 6.0:
            return 'dynamic'  # Classify as dynamic even if stationary for vehicles
            
        # Original classification logic
        if height < self.ground_height_threshold:
            return 'ground'
        elif height_ratio > self.vegetation_height_ratio:
            return 'vegetation'
        elif width > self.building_width_threshold and height > 2.0:
            return 'building'
        else:
            return 'obstacle'
    
    def update_layer_from_cluster(self, cluster, cluster_class):
        """Update costmap layer from a classified cluster"""
        # Get cluster bounds in grid coordinates
        min_x, min_y = self.world_to_map(cluster['min_x'], cluster['min_y'])
        max_x, max_y = self.world_to_map(cluster['max_x'], cluster['max_y'])
        
        if min_x is None or min_y is None or max_x is None or max_y is None:
            return
        
        # Set cost in appropriate layer
        cost = int(min(100, cluster['confidence'] * 100))
        current_time = time.time()
        
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                # Update layer with maximum cost
                self.layers[cluster_class][y, x] = max(self.layers[cluster_class][y, x], cost)
                
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
            
            # Apply different decay rates to different layers
            decay_rates = {
                'ground': 0.01,     # Slow decay for static features
                'obstacle': 0.05,
                'vegetation': 0.01,
                'building': 0.005,  # Very slow decay for buildings
                'dynamic': 0.2      # Fast decay for dynamic objects
            }
            
            # Apply decay based on time since last update
            time_diff = current_time - self.last_update_time
            for layer_name, decay_rate in decay_rates.items():
                # Calculate decay factor
                decay_factor = 1.0 - (decay_rate * time_diff)
                decay_factor = max(0.0, min(1.0, decay_factor))
                
                # Apply decay to entire layer
                self.layers[layer_name] = (self.layers[layer_name] * decay_factor).astype(np.int8)
            
            # Update dynamic object positions based on tracked velocities
            if self.motion_prediction and self.dynamic_objects:
                # Create a copy of the dynamic layer for prediction
                dynamic_layer_copy = self.layers['dynamic'].copy()
                
                for obj_id, obj_data in self.dynamic_objects.items():
                    # Skip old objects
                    if current_time - obj_data['timestamp'] > self.decay_time:
                        continue
                        
                    # Predict new position based on velocity and time difference
                    dt = current_time - obj_data['timestamp']
                    new_x = obj_data['x'] + (obj_data['vx'] * dt)
                    new_y = obj_data['y'] + (obj_data['vy'] * dt)
                    
                    # Update dynamic layer with predicted position
                    grid_x, grid_y = self.world_to_map(new_x, new_y)
                    
                    if grid_x is not None and grid_y is not None:
                        # Mark predicted position with slightly lower confidence
                        confidence = obj_data['confidence'] * 0.9
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
                if current_time - obj_data['timestamp'] <= self.decay_time:
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
            marker_id += 1
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
            
            # Set lifetime
            marker.lifetime.sec = 1
            
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
                text_marker.lifetime.sec = 1
                
                marker_array.markers.append(text_marker)
        
        # Publish markers
        self.semantic_marker_pub.publish(marker_array)
    
    def publish_height_map(self):
        """Publish height map as point cloud"""
        # TODO: Implement height map visualization
        pass
    
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