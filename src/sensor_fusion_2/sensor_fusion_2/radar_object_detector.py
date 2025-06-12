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
            ('min_points_per_cluster', 3),
            ('cluster_distance_threshold', 0.8),
            ('static_velocity_threshold', 0.5),
            ('moving_velocity_threshold', 1.0),
            ('use_dbscan_clustering', True),
            ('dbscan_epsilon', 0.7),
            ('dbscan_min_samples', 3),
            ('track_objects', True),
            ('max_tracking_age', 2.0),
            ('min_track_confidence', 0.6),
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
            ('publish_rate', 10.0)
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
                if obj['avg_velocity'] > self.moving_velocity_threshold:
                    obj['type'] = 'moving'
                    moving_objects.append(obj)
                elif obj['avg_velocity'] <= self.static_velocity_threshold:
                    obj['type'] = 'static'
                    static_objects.append(obj)
                else:
                    # Objects with velocity between thresholds are considered uncertain
                    # We'll classify them based on their previous classification if tracked
                    if self.track_objects and 'id' in obj and obj['id'] in self.tracked_objects:
                        prev_type = self.tracked_objects[obj['id']]['type']
                        obj['type'] = prev_type
                        if prev_type == 'moving':
                            moving_objects.append(obj)
                        else:
                            static_objects.append(obj)
                    else:
                        # Default to static for uncertain objects
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
                self.get_logger().info(f'Detected {len(moving_objects)} moving and {len(static_objects)} static objects')
                
        except Exception as e:
            self.get_logger().error(f'Error in process_and_publish: {str(e)}')
    
    def cluster_points(self, points):
        """Cluster radar points into objects"""
        if not points:
            return []
            
        if len(points) < self.min_points_per_cluster:
            return []
            
        # Extract point coordinates for clustering
        point_coords = np.array([[p['x'], p['y'], p['z']] for p in points])
        
        clusters = []
        
        if self.use_dbscan_clustering:
            # Use DBSCAN for clustering
            dbscan = DBSCAN(eps=self.dbscan_epsilon, min_samples=self.dbscan_min_samples)
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
                if len(cluster_points) < self.min_points_per_cluster:
                    continue
                    
                # Calculate cluster properties
                obj = self.calculate_object_properties(cluster_points)
                clusters.append(obj)
                
        else:
            # Simple distance-based clustering
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
                        if dist <= self.cluster_distance_threshold:
                            current_cluster.append(p)
                            remaining_points.pop(i)
                            i -= 1
                            break
                    
                    i += 1
                
                # Create object from cluster if it has enough points
                if len(current_cluster) >= self.min_points_per_cluster:
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
        
        size_x = max(0.3, x_max - x_min)  # Minimum size of 0.3m
        size_y = max(0.3, y_max - y_min)
        size_z = max(0.3, z_max - z_min)
        
        # Calculate average velocity
        velocities = [abs(p['velocity']) for p in cluster_points]
        avg_velocity = sum(velocities) / len(velocities)
        
        # Calculate velocity vector (simplified)
        vx = 0
        vy = 0
        vz = 0
        
        for p in cluster_points:
            # This is a simplification - in reality, we'd need the actual velocity components
            # Here we're using the point position relative to center as a proxy for direction
            dx = p['x'] - center_x
            dy = p['y'] - center_y
            dz = p['z'] - center_z
            
            # Normalize
            mag = math.sqrt(dx*dx + dy*dy + dz*dz)
            if mag > 0:
                dx /= mag
                dy /= mag
                dz /= mag
                
            # Apply velocity magnitude
            vx += dx * p['velocity']
            vy += dy * p['velocity']
            vz += dz * p['velocity']
        
        # Average velocity vector
        if len(cluster_points) > 0:
            vx /= len(cluster_points)
            vy /= len(cluster_points)
            vz /= len(cluster_points)
        
        # Create object
        obj = {
            'center': {'x': center_x, 'y': center_y, 'z': center_z},
            'size': {'x': size_x, 'y': size_y, 'z': size_z},
            'avg_velocity': avg_velocity,
            'velocity_vector': {'x': vx, 'y': vy, 'z': vz},
            'num_points': len(cluster_points),
            'timestamp': time.time(),
            'points': cluster_points
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
                if dist < best_match_dist and dist < 2.0:  # Maximum association distance
                    best_match_id = obj_id
                    best_match_dist = dist
            
            if best_match_id is not None:
                # Update the tracked object
                tracked_obj = self.tracked_objects[best_match_id]
                
                # Update position with some smoothing
                alpha = 0.7  # Weight for new measurement
                obj['center']['x'] = alpha * obj['center']['x'] + (1-alpha) * tracked_obj['center']['x']
                obj['center']['y'] = alpha * obj['center']['y'] + (1-alpha) * tracked_obj['center']['y']
                obj['center']['z'] = alpha * obj['center']['z'] + (1-alpha) * tracked_obj['center']['z']
                
                # Update velocity with smoothing
                obj['avg_velocity'] = alpha * obj['avg_velocity'] + (1-alpha) * tracked_obj['avg_velocity']
                
                # Keep track ID and update timestamp
                obj['id'] = best_match_id
                obj['track_age'] = current_time - tracked_obj['first_seen']
                obj['first_seen'] = tracked_obj['first_seen']
                obj['confidence'] = min(1.0, tracked_obj['confidence'] + 0.1)  # Increase confidence
                
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
                obj['confidence'] = 0.5  # Initial confidence
                
                self.tracked_objects[obj_id] = obj
        
        # Remove old tracks
        ids_to_remove = []
        for obj_id, tracked_obj in self.tracked_objects.items():
            if obj_id not in matched_ids:
                # Object wasn't matched in this update
                age = current_time - tracked_obj['timestamp']
                if age > self.max_tracking_age:
                    ids_to_remove.append(obj_id)
                else:
                    # Decrease confidence for unmatched objects
                    tracked_obj['confidence'] = max(0.0, tracked_obj['confidence'] - 0.1)
        
        for obj_id in ids_to_remove:
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
            marker.pose.position.x = obj['center']['x']
            marker.pose.position.y = obj['center']['y']
            marker.pose.position.z = obj['center']['z']
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = obj['size']['x']
            marker.scale.y = obj['size']['y']
            marker.scale.z = obj['size']['z']
            
            # Set color (red for moving objects)
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
                start = Point(x=obj['center']['x'], y=obj['center']['y'], z=obj['center']['z'])
                
                # End point based on velocity vector
                scale = min(5.0, max(0.5, obj['avg_velocity']))  # Scale arrow by velocity
                end = Point(
                    x=obj['center']['x'] + obj['velocity_vector']['x'] * scale,
                    y=obj['center']['y'] + obj['velocity_vector']['y'] * scale,
                    z=obj['center']['z'] + obj['velocity_vector']['z'] * scale
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
            marker.pose.position.x = obj['center']['x']
            marker.pose.position.y = obj['center']['y']
            marker.pose.position.z = obj['center']['z']
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = obj['size']['x']
            marker.scale.y = obj['size']['y']
            marker.scale.z = obj['size']['z']
            
            # Set color (blue for static objects)
            marker.color = self.static_object_color
            
            # Set lifetime
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime % 1) * 1e9)
            
            marker_array.markers.append(marker)
        
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
            marker.pose.position.x = obj['center']['x']
            marker.pose.position.y = obj['center']['y']
            marker.pose.position.z = obj['center']['z'] + obj['size']['z'] / 2 + 0.3
            
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
            v_speed = f"{obj['avg_velocity']:.1f} m/s"
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