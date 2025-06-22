#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import numpy as np
from sklearn.cluster import DBSCAN
import time
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math
import uuid

class RadarObjectDetector(Node):
    """
    Detects and tracks objects from radar data, classifying them as moving or static.
    
    This node:
    1. Subscribes to radar point cloud data
    2. Clusters points to identify objects
    3. Classifies objects as moving or static based on velocity
    4. Tracks objects over time
    5. Publishes visualization markers for moving and static objects
    """
    def __init__(self):
        super().__init__('radar_object_detector')
        
        # Declare parameters
        self.declare_parameters_if_not_declared([
            ('use_sim_time', False),
            ('frame_id', 'radar_link'),
            ('map_frame_id', 'map'),
            ('points_topic', '/radar/points'),
            ('moving_objects_topic', '/radar/moving_objects'),
            ('static_objects_topic', '/radar/static_objects'),
            ('object_tracking_topic', '/radar/object_tracking'),
            ('min_points_per_cluster', 1),
            ('cluster_distance_threshold', 1.5),
            ('static_velocity_threshold', 0.0),
            ('moving_velocity_threshold', 0.0),
            ('use_dbscan_clustering', True),
            ('dbscan_epsilon', 1.5),
            ('dbscan_min_samples', 1),
            ('track_objects', True),
            ('max_tracking_age', 3.0),
            ('min_track_confidence', 0.3),
            ('moving_object_color_r', 1.0),
            ('moving_object_color_g', 0.0),
            ('moving_object_color_b', 0.0),
            ('moving_object_color_a', 0.8),
            ('static_object_color_r', 0.0),
            ('static_object_color_g', 0.0),
            ('static_object_color_b', 1.0),
            ('static_object_color_a', 0.8),
            ('verbose_logging', True),
            ('marker_lifetime', 0.5),
            ('publish_rate', 30.0)
        ])
        
        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.frame_id = self.get_parameter('frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.points_topic = self.get_parameter('points_topic').value
        self.moving_objects_topic = self.get_parameter('moving_objects_topic').value
        self.static_objects_topic = self.get_parameter('static_objects_topic').value
        self.object_tracking_topic = self.get_parameter('object_tracking_topic').value
        self.min_points_per_cluster = self.get_parameter('min_points_per_cluster').value
        self.cluster_distance_threshold = self.get_parameter('cluster_distance_threshold').value
        self.static_velocity_threshold = self.get_parameter('static_velocity_threshold').value
        self.moving_velocity_threshold = self.get_parameter('moving_velocity_threshold').value
        self.use_dbscan_clustering = self.get_parameter('use_dbscan_clustering').value
        self.dbscan_epsilon = self.get_parameter('dbscan_epsilon').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.track_objects = self.get_parameter('track_objects').value
        self.max_tracking_age = self.get_parameter('max_tracking_age').value
        self.min_track_confidence = self.get_parameter('min_track_confidence').value
        self.moving_object_color = ColorRGBA(
            r=self.get_parameter('moving_object_color_r').value,
            g=self.get_parameter('moving_object_color_g').value,
            b=self.get_parameter('moving_object_color_b').value,
            a=self.get_parameter('moving_object_color_a').value
        )
        self.static_object_color = ColorRGBA(
            r=self.get_parameter('static_object_color_r').value,
            g=self.get_parameter('static_object_color_g').value,
            b=self.get_parameter('static_object_color_b').value,
            a=self.get_parameter('static_object_color_a').value
        )
        self.verbose_logging = self.get_parameter('verbose_logging').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        vis_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Initialize data structures
        self.radar_points = []
        self.radar_points_lock = threading.Lock()
        self.tracked_objects = {}  # Dictionary to track objects over time
        self.next_object_id = 0
        
        # Create subscribers
        self.points_sub = self.create_subscription(
            PointCloud2,
            self.points_topic,
            self.points_callback,
            sensor_qos
        )
        
        # Create publishers
        self.moving_objects_pub = self.create_publisher(
            MarkerArray,
            self.moving_objects_topic,
            vis_qos
        )
        
        self.static_objects_pub = self.create_publisher(
            MarkerArray,
            self.static_objects_topic,
            vis_qos
        )
        
        self.object_tracking_pub = self.create_publisher(
            MarkerArray,
            self.object_tracking_topic,
            vis_qos
        )
        
        # Create timer for processing and publishing
        self.process_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.process_and_publish
        )
        
        self.get_logger().info('Radar Object Detector initialized')
        self.get_logger().info(f'Subscribing to radar points: {self.points_topic}')
        self.get_logger().info(f'Publishing moving objects to: {self.moving_objects_topic}')
        self.get_logger().info(f'Publishing static objects to: {self.static_objects_topic}')
        self.get_logger().info(f'Publishing object tracking to: {self.object_tracking_topic}')
    
    def points_callback(self, msg):
        """Process incoming radar point cloud data"""
        try:
            # Extract points from PointCloud2 message
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z", "velocity"), skip_nans=True):
                x, y, z, velocity = point
                points.append({
                    'x': x,
                    'y': y,
                    'z': z,
                    'velocity': velocity,
                    'timestamp': time.time()
                })
            
            # Update points with lock for thread safety
            with self.radar_points_lock:
                self.radar_points = points
                
            if self.verbose_logging and len(points) > 0:
                self.get_logger().info(f'Received {len(points)} radar points')
                
        except Exception as e:
            self.get_logger().error(f'Error processing radar points: {str(e)}')
    
    def process_and_publish(self):
        """Process radar data and publish visualization markers"""
        try:
            # Get current points with lock
            with self.radar_points_lock:
                points = self.radar_points.copy()
            
            if not points:
                return
                
            # Cluster points into objects
            objects = self.cluster_points(points)
            
            # Classify objects as moving or static
            moving_objects = []
            static_objects = []
            
            for obj in objects:
                # MODIFIED: Always classify vehicles as moving objects regardless of velocity
                # This ensures all cars are detected and tracked even with minimal relative velocity
                if obj.get('is_vehicle', False):
                    obj['type'] = 'moving'
                    moving_objects.append(obj)
                    
                    # Log detection of vehicles
                    if self.verbose_logging:
                        is_low_profile = obj.get('is_low_profile', False)
                        car_type = "low-profile car" if is_low_profile else "vehicle"
                        self.get_logger().debug(f"Detected {car_type}: vel={obj['avg_velocity']:.2f}, points={obj['num_points']}")
                
                # For non-vehicle objects, use velocity-based classification
                elif obj['avg_velocity'] > 0.1:  # Reduced threshold to detect more moving objects
                    obj['type'] = 'moving'
                    moving_objects.append(obj)
                else:
                    obj['type'] = 'static'
                    static_objects.append(obj)
            
            # Track objects over time if enabled
            if self.track_objects:
                self.update_object_tracking(moving_objects + static_objects)
            
            # Create and publish visualization markers
            self.publish_moving_objects(moving_objects)
            self.publish_static_objects(static_objects)
            self.publish_object_tracking()
            
            if self.verbose_logging:
                vehicle_count = sum(1 for obj in objects if obj.get('is_vehicle', False))
                low_profile_count = sum(1 for obj in objects if obj.get('is_low_profile', False))
                self.get_logger().info(f"Detected {len(moving_objects)} moving and {len(static_objects)} static objects, "
                                      f"including {vehicle_count} vehicles ({low_profile_count} low-profile)")
                
        except Exception as e:
            self.get_logger().error(f'Error in process_and_publish: {str(e)}')
    
    def cluster_points(self, points):
        """Cluster radar points into objects"""
        if not points:
            return []
            
        # MODIFIED: Always process points even if there's just one
        # This ensures we don't miss any potential car detections
        
        # Extract point coordinates for clustering
        point_coords = np.array([[p['x'], p['y'], p['z']] for p in points])
        
        clusters = []
        
        if self.use_dbscan_clustering:
            # MODIFIED: Always use adaptive clustering with more sensitive parameters
            # for better detection of low-profile cars
            
            # For very small point clouds, use very lenient parameters
            if len(points) < 5:
                # With very few points, use maximum epsilon and minimum samples
                dbscan = DBSCAN(eps=2.0, min_samples=1)
                self.get_logger().debug(f"Using very lenient clustering for {len(points)} points: eps=2.0, min_samples=1")
            else:
                # Calculate average distance between points
                distances = []
                for i in range(min(100, len(points))):  # Limit computation for large point clouds
                    p1 = point_coords[i]
                    for j in range(i+1, min(100, len(points))):
                        p2 = point_coords[j]
                        dist = np.linalg.norm(p1 - p2)
                        distances.append(dist)
                
                if distances:
                    # Calculate statistics
                    avg_dist = np.mean(distances)
                    min_dist = np.min(distances)
                    
                    # Adjust epsilon based on point distribution
                    # Always use more lenient parameters than the defaults
                    adaptive_epsilon = max(1.0, self.dbscan_epsilon * 1.5)
                    adaptive_min_samples = 1  # Always use minimum samples of 1
                    
                    if avg_dist > 1.0:  # Points are far apart
                        adaptive_epsilon = min(3.0, self.dbscan_epsilon * 2.0)  # Increase epsilon even more, max 3.0
                    
                    self.get_logger().debug(f"Adaptive clustering: epsilon={adaptive_epsilon:.2f}, min_samples={adaptive_min_samples}, avg_dist={avg_dist:.2f}")
                    
                    # Use DBSCAN with adaptive parameters
                    dbscan = DBSCAN(eps=adaptive_epsilon, min_samples=adaptive_min_samples)
                else:
                    # Fallback to lenient parameters
                    dbscan = DBSCAN(eps=1.5, min_samples=1)
            
            # Run clustering
            cluster_labels = dbscan.fit_predict(point_coords)
            
            # Group points by cluster
            cluster_dict = {}
            for i, label in enumerate(cluster_labels):
                if label == -1:  # Skip noise points
                    continue
                    
                if label not in cluster_dict:
                    cluster_dict[label] = []
                    
                cluster_dict[label].append(points[i])
                
            # Create object for each cluster
            for label, cluster_points in cluster_dict.items():
                # MODIFIED: Process all clusters, even with a single point
                obj = self.calculate_object_properties(cluster_points)
                clusters.append(obj)
                
        else:
            # Enhanced simple distance-based clustering
            remaining_points = points.copy()
            
            while remaining_points:
                # Start a new cluster with the first point
                current_cluster = [remaining_points.pop(0)]
                
                # Find all points within distance threshold
                i = 0
                while i < len(remaining_points):
                    p = remaining_points[i]
                    
                    # Check if point is close to any point in the current cluster
                    for cp in current_cluster:
                        dist = math.sqrt((p['x'] - cp['x'])**2 + (p['y'] - cp['y'])**2 + (p['z'] - cp['z'])**2)
                        
                        # MODIFIED: Always use a larger threshold for better clustering
                        adaptive_threshold = max(1.0, self.cluster_distance_threshold * 2.0)
                        
                        if dist <= adaptive_threshold:
                            current_cluster.append(p)
                            remaining_points.pop(i)
                            i -= 1
                            break
                    
                    i += 1
                
                # MODIFIED: Process all clusters, even with a single point
                obj = self.calculate_object_properties(current_cluster)
                clusters.append(obj)
        
        return clusters
    
    def calculate_object_properties(self, cluster_points):
        """Calculate properties of an object from its cluster points"""
        # Calculate center position
        x_sum = sum(p['x'] for p in cluster_points)
        y_sum = sum(p['y'] for p in cluster_points)
        z_sum = sum(p['z'] for p in cluster_points)
        
        center_x = x_sum / len(cluster_points)
        center_y = y_sum / len(cluster_points)
        center_z = z_sum / len(cluster_points)
        
        # Calculate size (bounding box)
        x_min = min(p['x'] for p in cluster_points)
        x_max = max(p['x'] for p in cluster_points)
        y_min = min(p['y'] for p in cluster_points)
        y_max = max(p['y'] for p in cluster_points)
        z_min = min(p['z'] for p in cluster_points)
        z_max = max(p['z'] for p in cluster_points)
        
        # Raw dimensions from point cloud
        raw_size_x = max(0.1, x_max - x_min)
        raw_size_y = max(0.1, y_max - y_min)
        raw_size_z = max(0.1, z_max - z_min)
        
        # MODIFIED: Enhanced vehicle detection with more lenient criteria for all cars
        is_vehicle = False
        is_low_profile = False
        
        # Require only a single point for potential car detection
        if len(cluster_points) >= 1:
            # Check for standard vehicles - wider range of dimensions
            if (0.3 <= raw_size_x <= 7.0 and 0.3 <= raw_size_y <= 3.5) or \
               (0.3 <= raw_size_y <= 7.0 and 0.3 <= raw_size_x <= 3.5):
                is_vehicle = True
                
                # Identify if it's a low-profile car
                if raw_size_z <= 1.5:
                    is_low_profile = True
                    self.get_logger().debug(f"Detected low-profile vehicle: size={raw_size_x:.2f}x{raw_size_y:.2f}x{raw_size_z:.2f}m")
            
            # Special check for very small returns that might be from distant cars
            elif (0.1 <= raw_size_x <= 1.0 and 0.1 <= raw_size_y <= 1.0):
                # Small objects that might be cars at a distance
                is_vehicle = True
                is_low_profile = True
                self.get_logger().debug(f"Detected potential distant vehicle: size={raw_size_x:.2f}x{raw_size_y:.2f}x{raw_size_z:.2f}m")
        
        # Set size - use vehicle dimensions if detected, otherwise ensure minimum size
        if is_vehicle:
            # Use typical vehicle dimensions - helps when only partial points are detected
            # Choose the larger dimension as the length
            if raw_size_x > raw_size_y:
                if is_low_profile:
                    size_x = max(raw_size_x, 2.0)  # Even smaller minimum length for low-profile cars
                    size_y = max(raw_size_y, 1.2)  # Even smaller minimum width for low-profile cars
                else:
                    size_x = max(raw_size_x, 4.0)  # Standard minimum length
                    size_y = max(raw_size_y, 1.7)  # Standard minimum width
            else:
                if is_low_profile:
                    size_x = max(raw_size_x, 1.2)  # Even smaller minimum width for low-profile cars
                    size_y = max(raw_size_y, 2.0)  # Even smaller minimum length for low-profile cars
                else:
                    size_x = max(raw_size_x, 1.7)  # Standard minimum width
                    size_y = max(raw_size_y, 4.0)  # Standard minimum length
            
            # Adjust height for low-profile vehicles
            if is_low_profile:
                size_z = max(raw_size_z, 0.2)  # Even lower minimum height for low-profile cars
            else:
                size_z = max(raw_size_z, 0.5)  # Standard minimum height
        else:
            # Use minimum size for non-vehicle objects
            size_x = max(0.3, raw_size_x)  # Minimum size of 0.3m
            size_y = max(0.3, raw_size_y)
            size_z = max(0.3, raw_size_z)
        
        # Calculate average velocity
        velocities = [abs(float(p['velocity'])) for p in cluster_points]
        avg_velocity = sum(velocities) / len(velocities)
        
        # MODIFIED: Don't boost velocities, just record the actual values
        # We'll classify all vehicles as moving regardless of velocity
        
        # Calculate velocity vector (simplified)
        vx = 0.0
        vy = 0.0
        vz = 0.0
        
        for p in cluster_points:
            # This is a simplification - in reality, we'd need the actual velocity components
            # Here we're using the point position relative to center as a proxy for direction
            dx = float(p['x'] - center_x)
            dy = float(p['y'] - center_y)
            dz = float(p['z'] - center_z)
            
            # Normalize
            mag = math.sqrt(dx*dx + dy*dy + dz*dz)
            if mag > 0:
                dx /= mag
                dy /= mag
                dz /= mag
                
            # Apply velocity magnitude
            velocity = float(p['velocity'])
            vx += dx * velocity
            vy += dy * velocity
            vz += dz * velocity
        
        # Average velocity vector
        if len(cluster_points) > 0:
            vx /= len(cluster_points)
            vy /= len(cluster_points)
            vz /= len(cluster_points)
        
        # Create object
        obj = {
            'center': {'x': float(center_x), 'y': float(center_y), 'z': float(center_z)},
            'size': {'x': float(size_x), 'y': float(size_y), 'z': float(size_z)},
            'avg_velocity': float(avg_velocity),
            'velocity_vector': {'x': float(vx), 'y': float(vy), 'z': float(vz)},
            'num_points': len(cluster_points),
            'timestamp': time.time(),
            'points': cluster_points,
            'is_vehicle': is_vehicle,  # Add vehicle flag
            'is_low_profile': is_low_profile  # Add low-profile flag
        }
        
        return obj
    
    def update_object_tracking(self, objects):
        """Track objects over time"""
        current_time = time.time()
        
        # Match current objects with tracked objects
        matched_ids = set()
        
        # First pass: associate objects with existing tracks
        for obj in objects:
            best_match_id = None
            best_match_dist = float('inf')
            
            # Determine association distance threshold based on object type
            # Use larger threshold for low-profile cars to improve tracking
            is_low_profile = obj.get('is_low_profile', False)
            max_association_distance = 3.0 if is_low_profile else 2.0  # Larger threshold for low-profile cars
            
            for obj_id, tracked_obj in self.tracked_objects.items():
                # Skip if this tracked object was already matched
                if obj_id in matched_ids:
                    continue
                    
                # Calculate distance between object centers
                dx = obj['center']['x'] - tracked_obj['center']['x']
                dy = obj['center']['y'] - tracked_obj['center']['y']
                dz = obj['center']['z'] - tracked_obj['center']['z']
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Check if this is a potential match
                # For low-profile cars, also consider if the tracked object was a low-profile car
                if dist < best_match_dist and dist < max_association_distance:
                    # If both are low-profile cars or both are not, prefer this match
                    if is_low_profile == tracked_obj.get('is_low_profile', False):
                        best_match_id = obj_id
                        best_match_dist = dist
                    # Otherwise, only match if no better match has been found
                    elif best_match_id is None:
                        best_match_id = obj_id
                        best_match_dist = dist
            
            if best_match_id is not None:
                # Update the tracked object
                tracked_obj = self.tracked_objects[best_match_id]
                
                # Update position with some smoothing
                # Use different smoothing factor based on object type
                if is_low_profile:
                    alpha = 0.8  # Higher weight for new measurements of low-profile cars
                else:
                    alpha = 0.7  # Standard weight for regular objects
                
                obj['center']['x'] = alpha * obj['center']['x'] + (1-alpha) * tracked_obj['center']['x']
                obj['center']['y'] = alpha * obj['center']['y'] + (1-alpha) * tracked_obj['center']['y']
                obj['center']['z'] = alpha * obj['center']['z'] + (1-alpha) * tracked_obj['center']['z']
                
                # Update velocity with smoothing
                obj['avg_velocity'] = alpha * obj['avg_velocity'] + (1-alpha) * tracked_obj['avg_velocity']
                
                # Keep track ID and update timestamp
                obj['id'] = best_match_id
                obj['track_age'] = current_time - tracked_obj['first_seen']
                obj['first_seen'] = tracked_obj['first_seen']
                
                # Update confidence - boost confidence for low-profile cars
                confidence_boost = 0.15 if is_low_profile else 0.1
                obj['confidence'] = min(1.0, tracked_obj['confidence'] + confidence_boost)
                
                # Preserve low-profile flag if it was set in either object
                obj['is_low_profile'] = is_low_profile or tracked_obj.get('is_low_profile', False)
                
                # Update tracked object
                self.tracked_objects[best_match_id] = obj
                matched_ids.add(best_match_id)
            else:
                # New object
                obj_id = self.next_object_id
                self.next_object_id += 1
                
                obj['id'] = obj_id
                obj['first_seen'] = current_time
                obj['track_age'] = 0.0
                
                # Set initial confidence based on object type
                # Higher initial confidence for low-profile cars to prevent early removal
                obj['confidence'] = 0.6 if is_low_profile else 0.5
                
                self.tracked_objects[obj_id] = obj
        
        # Remove old tracks
        ids_to_remove = []
        for obj_id, tracked_obj in self.tracked_objects.items():
            if obj_id not in matched_ids:
                # Object wasn't matched in this update
                age = current_time - tracked_obj['timestamp']
                
                # Use different tracking age thresholds based on object type
                is_low_profile = tracked_obj.get('is_low_profile', False)
                max_age = self.max_tracking_age * 1.5 if is_low_profile else self.max_tracking_age
                
                if age > max_age:
                    ids_to_remove.append(obj_id)
                else:
                    # Decrease confidence for unmatched objects
                    # Decrease more slowly for low-profile cars
                    confidence_decrease = 0.05 if is_low_profile else 0.1
                    tracked_obj['confidence'] = max(0.0, tracked_obj['confidence'] - confidence_decrease)
                    
                    # Log when a low-profile car is about to be removed
                    if is_low_profile and tracked_obj['confidence'] < 0.2:
                        self.get_logger().debug(f"Low-profile car with ID {obj_id} has low confidence: {tracked_obj['confidence']:.2f}")
        
        for obj_id in ids_to_remove:
            # Log when removing a tracked low-profile car
            if self.tracked_objects[obj_id].get('is_low_profile', False):
                self.get_logger().info(f"Removing tracked low-profile car with ID {obj_id}, age: {current_time - self.tracked_objects[obj_id]['timestamp']:.2f}s")
            del self.tracked_objects[obj_id]
    
    def publish_moving_objects(self, objects):
        """Publish visualization markers for moving objects"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects):
            # Create cube marker for object
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "moving_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = float(obj['center']['x'])
            marker.pose.position.y = float(obj['center']['y'])
            marker.pose.position.z = float(obj['center']['z'])
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = float(obj['size']['x'])
            marker.scale.y = float(obj['size']['y'])
            marker.scale.z = float(obj['size']['z'])
            
            # Set color - use special color for low-profile cars
            if obj.get('is_low_profile', False):
                # Use bright orange for low-profile cars to make them more visible
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.9  # Higher alpha for better visibility
            else:
                # Standard red for regular moving objects
                marker.color = self.moving_object_color
            
            # Set lifetime
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            marker_array.markers.append(marker)
            
            # Add velocity arrow if velocity is significant
            if obj['avg_velocity'] > 0.5:
                arrow = Marker()
                arrow.header.frame_id = self.map_frame_id
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = "moving_objects_velocity"
                arrow.id = i
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                # Start point at object center
                start = Point(x=float(obj['center']['x']), y=float(obj['center']['y']), z=float(obj['center']['z']))
                
                # End point based on velocity vector
                scale = min(5.0, max(0.5, float(obj['avg_velocity'])))  # Scale arrow by velocity
                end = Point(
                    x=float(obj['center']['x'] + obj['velocity_vector']['x'] * scale),
                    y=float(obj['center']['y'] + obj['velocity_vector']['y'] * scale),
                    z=float(obj['center']['z'] + obj['velocity_vector']['z'] * scale)
                )
                
                arrow.points.append(start)
                arrow.points.append(end)
                
                # Set arrow properties
                arrow.scale.x = 0.1  # shaft diameter
                arrow.scale.y = 0.2  # head diameter
                arrow.scale.z = 0.1  # head length
                
                # Set color (yellow for velocity)
                arrow.color.r = 1.0
                arrow.color.g = 1.0
                arrow.color.b = 0.0
                arrow.color.a = 0.8
                
                # Set lifetime
                arrow.lifetime.sec = int(self.marker_lifetime)
                arrow.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                
                marker_array.markers.append(arrow)
            
            # For low-profile cars, add an additional marker to enhance visibility
            if obj.get('is_low_profile', False):
                # Add a sphere marker slightly above the car
                sphere = Marker()
                sphere.header.frame_id = self.map_frame_id
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = "low_profile_indicator"
                sphere.id = i
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                
                # Position slightly above the car
                sphere.pose.position.x = float(obj['center']['x'])
                sphere.pose.position.y = float(obj['center']['y'])
                sphere.pose.position.z = float(obj['center']['z'] + obj['size']['z'] + 0.3)
                
                # Set orientation (identity quaternion)
                sphere.pose.orientation.w = 1.0
                
                # Small sphere
                sphere.scale.x = 0.4
                sphere.scale.y = 0.4
                sphere.scale.z = 0.4
                
                # Bright color for visibility
                sphere.color.r = 1.0
                sphere.color.g = 0.0
                sphere.color.b = 1.0  # Purple
                sphere.color.a = 0.9
                
                # Set lifetime
                sphere.lifetime.sec = int(self.marker_lifetime)
                sphere.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                
                marker_array.markers.append(sphere)
        
        self.moving_objects_pub.publish(marker_array)
    
    def publish_static_objects(self, objects):
        """Publish visualization markers for static objects"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(objects):
            # Create cube marker for object
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "static_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = float(obj['center']['x'])
            marker.pose.position.y = float(obj['center']['y'])
            marker.pose.position.z = float(obj['center']['z'])
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = float(obj['size']['x'])
            marker.scale.y = float(obj['size']['y'])
            marker.scale.z = float(obj['size']['z'])
            
            # Set color - use special color for low-profile cars
            if obj.get('is_low_profile', False):
                # Use cyan for static low-profile cars
                marker.color.r = 0.0
                marker.color.g = 0.8
                marker.color.b = 1.0
                marker.color.a = 0.9  # Higher alpha for better visibility
            else:
                # Standard blue for regular static objects
                marker.color = self.static_object_color
            
            # Set lifetime
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            marker_array.markers.append(marker)
            
            # For low-profile cars, add an additional marker to enhance visibility
            if obj.get('is_low_profile', False):
                # Add a sphere marker slightly above the car
                sphere = Marker()
                sphere.header.frame_id = self.map_frame_id
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = "low_profile_indicator_static"
                sphere.id = i
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                
                # Position slightly above the car
                sphere.pose.position.x = float(obj['center']['x'])
                sphere.pose.position.y = float(obj['center']['y'])
                sphere.pose.position.z = float(obj['center']['z'] + obj['size']['z'] + 0.3)
                
                # Set orientation (identity quaternion)
                sphere.pose.orientation.w = 1.0
                
                # Small sphere
                sphere.scale.x = 0.4
                sphere.scale.y = 0.4
                sphere.scale.z = 0.4
                
                # Bright color for visibility
                sphere.color.r = 0.0
                sphere.color.g = 1.0
                sphere.color.b = 1.0  # Cyan
                sphere.color.a = 0.9
                
                # Set lifetime
                sphere.lifetime.sec = int(self.marker_lifetime)
                sphere.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
                
                marker_array.markers.append(sphere)
        
        self.static_objects_pub.publish(marker_array)
    
    def publish_object_tracking(self):
        """Publish visualization markers for tracked objects"""
        if not self.track_objects:
            return
            
        marker_array = MarkerArray()
        
        for i, (obj_id, obj) in enumerate(self.tracked_objects.items()):
            if obj['confidence'] < self.min_track_confidence:
                continue
                
            # Create text marker for object ID
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "object_tracking"
            marker.id = obj_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # Set position (slightly above object)
            marker.pose.position.x = float(obj['center']['x'])
            marker.pose.position.y = float(obj['center']['y'])
            marker.pose.position.z = float(obj['center']['z'] + obj['size']['z'] / 2 + 0.3)
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.z = 0.3  # text height
            
            # Set color (white)
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            # Set text
            v_type = "Moving" if obj['type'] == 'moving' else "Static"
            v_speed = f"{float(obj['avg_velocity']):.1f} m/s"
            marker.text = f"ID: {obj_id}\n{v_type}\n{v_speed}"
            
            # Set lifetime
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            marker_array.markers.append(marker)
        
        self.object_tracking_pub.publish(marker_array)

    def declare_parameters_if_not_declared(self, parameters_list):
        """
        Declare parameters only if they haven't been declared already
        
        Args:
            parameters_list: List of tuples (parameter_name, default_value)
        """
        for param_name, default_value in parameters_list:
            # Check if parameter exists already
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default_value)
                if self.get_parameter(param_name).type_ == Parameter.Type.NOT_SET:
                    self.get_logger().warn(f"Parameter '{param_name}' was declared but not set")
            else:
                self.get_logger().debug(f"Parameter '{param_name}' already declared, skipping declaration")

def main(args=None):
    rclpy.init(args=args)
    node = RadarObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 