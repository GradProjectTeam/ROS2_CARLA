#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA
import numpy as np
import math
import tf2_ros
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2
import time
from collections import deque
from scipy.spatial import KDTree
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RadarObject:
    """Class to track individual radar objects over time"""
    def __init__(self, position, velocity, timestamp, id=None):
        self.id = id
        self.positions = [position]  # List of (x, y, z) positions
        self.velocities = [velocity]  # List of (vx, vy, vz) velocities
        self.timestamps = [timestamp]  # List of timestamps
        self.last_update = timestamp
        self.is_moving = False
        self.predicted_positions = []  # Future positions for motion prediction
        self.confidence = 1.0  # Confidence level (1.0 = high confidence)
        self.lifetime = 0.0  # How long we've been tracking this object
        self.age = 0.0  # Time since last update
        
    def update(self, position, velocity, timestamp):
        """Update object with new measurement"""
        self.positions.append(position)
        self.velocities.append(velocity)
        self.timestamps.append(timestamp)
        
        # Keep only recent history
        if len(self.positions) > 10:
            self.positions.pop(0)
            self.velocities.pop(0)
            self.timestamps.pop(0)
            
        self.last_update = timestamp
        
        # Calculate if object is moving
        if len(self.velocities) >= 2:
            avg_velocity = np.mean([np.linalg.norm(v) for v in self.velocities[-3:]])
            self.is_moving = avg_velocity > 0.5  # Moving if average velocity > 0.5 m/s
            
        # Update lifetime
        if len(self.timestamps) >= 2:
            self.lifetime = (self.timestamps[-1] - self.timestamps[0]).nanoseconds / 1e9
            
    def predict_future_positions(self, prediction_time=1.0, steps=5):
        """Predict future positions based on current velocity"""
        if not self.velocities:
            return []
            
        # Get latest position and velocity
        latest_pos = self.positions[-1]
        latest_vel = self.velocities[-1]
        
        # Generate predictions
        predictions = []
        time_step = prediction_time / steps
        
        for i in range(1, steps + 1):
            t = i * time_step
            pred_x = latest_pos[0] + latest_vel[0] * t
            pred_y = latest_pos[1] + latest_vel[1] * t
            pred_z = latest_pos[2] + latest_vel[2] * t
            predictions.append((pred_x, pred_y, pred_z))
            
        self.predicted_positions = predictions
        return predictions
        
    def get_latest_position(self):
        """Get the latest position"""
        return self.positions[-1] if self.positions else None
        
    def get_latest_velocity(self):
        """Get the latest velocity"""
        return self.velocities[-1] if self.velocities else (0, 0, 0)
        
    def get_velocity_magnitude(self):
        """Get the magnitude of the latest velocity"""
        if not self.velocities:
            return 0.0
        return np.linalg.norm(self.velocities[-1])
        
    def update_age(self, current_time):
        """Update the age of this object"""
        self.age = (current_time - self.last_update).nanoseconds / 1e9
        
    def is_approaching(self):
        """Determine if object is approaching (negative radial velocity)"""
        if not self.positions or not self.velocities:
            return False
            
        # Calculate radial velocity (projection of velocity onto position vector)
        pos = np.array(self.positions[-1])
        vel = np.array(self.velocities[-1])
        
        # Avoid division by zero
        pos_norm = np.linalg.norm(pos)
        if pos_norm < 0.001:
            return False
            
        # Calculate radial velocity component
        radial_velocity = np.dot(pos, vel) / pos_norm
        
        # Negative radial velocity means approaching
        return radial_velocity < 0

class RadarCostmapCreator(Node):
    """
    Enhanced radar costmap creator that specializes in detecting and tracking moving objects.
    
    This node:
    1. Subscribes to radar point cloud and marker data
    2. Tracks objects over time to distinguish between static and dynamic obstacles
    3. Performs motion prediction for dynamic obstacles
    4. Creates a specialized costmap highlighting moving objects
    5. Publishes to both dedicated radar costmap and realtime_map topics
    """
    def __init__(self):
        super().__init__('radar_costmap_creator')
        
        # Declare parameters
        self.declare_parameter('map_resolution', 0.2)       # meters per cell
        self.declare_parameter('map_width', 300)           # cells (60m with 0.2m resolution)
        self.declare_parameter('map_height', 300)          # cells
        self.declare_parameter('map_origin_x', -30.0)       # meters
        self.declare_parameter('map_origin_y', -30.0)       # meters
        self.declare_parameter('publish_rate', 10.0)        # Hz
        self.declare_parameter('velocity_impact_factor', 30.0)  # Cost increase per m/s
        self.declare_parameter('max_object_age', 2.0)       # seconds
        self.declare_parameter('radar_weight', 0.7)         # Weight for radar data in fusion
        self.declare_parameter('prediction_time', 1.5)      # Time to predict into future (seconds)
        self.declare_parameter('min_velocity_threshold', 0.3)  # Min velocity to consider moving (m/s)
        self.declare_parameter('publish_to_realtime_map', True)  # Whether to publish to realtime map
        self.declare_parameter('realtime_map_topic', '/realtime_map')  # Realtime map topic
        self.declare_parameter('use_transient_local_durability', True)  # Use transient local durability
        self.declare_parameter('object_tracking_distance', 1.0)  # Max distance for object association (meters)
        self.declare_parameter('enable_visualization', True)  # Enable visualization markers
        self.declare_parameter('map_frame', 'map')          # Map frame ID
        
        # Get parameters
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.map_origin_x = self.get_parameter('map_origin_x').value
        self.map_origin_y = self.get_parameter('map_origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.velocity_impact_factor = self.get_parameter('velocity_impact_factor').value
        self.max_object_age = self.get_parameter('max_object_age').value
        self.radar_weight = self.get_parameter('radar_weight').value
        self.prediction_time = self.get_parameter('prediction_time').value
        self.min_velocity_threshold = self.get_parameter('min_velocity_threshold').value
        self.publish_to_realtime_map = self.get_parameter('publish_to_realtime_map').value
        self.realtime_map_topic = self.get_parameter('realtime_map_topic').value
        self.use_transient_local_durability = self.get_parameter('use_transient_local_durability').value
        self.object_tracking_distance = self.get_parameter('object_tracking_distance').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # Initialize costmap layer
        self.radar_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Object tracking
        self.tracked_objects = {}  # {object_id: RadarObject}
        self.next_object_id = 0
        
        # QoS profile for reliable map publishing
        durability = QoSDurabilityPolicy.TRANSIENT_LOCAL if self.use_transient_local_durability else QoSDurabilityPolicy.VOLATILE
        
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=durability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.costmap_publisher = self.create_publisher(
            OccupancyGrid, 
            '/radar_costmap', 
            map_qos
        )
        
        # Realtime map publisher if enabled
        if self.publish_to_realtime_map:
            self.realtime_map_publisher = self.create_publisher(
                OccupancyGrid,
                self.realtime_map_topic,
                map_qos
            )
        
        # Visualization publishers
        if self.enable_visualization:
            self.tracked_objects_pub = self.create_publisher(
                MarkerArray,
                '/radar/tracked_objects',
                10
            )
            
            self.predicted_paths_pub = self.create_publisher(
                MarkerArray,
                '/radar/predicted_paths',
                10
            )
        
        # Subscribers
        self.create_subscription(
            MarkerArray,
            '/radar/markers',
            self.marker_callback,
            10
        )
        
        self.create_subscription(
            MarkerArray,
            '/radar/velocity_vectors',
            self.velocity_callback,
            10
        )
        
        self.create_subscription(
            PointCloud2,
            '/radar/points',
            self.pointcloud_callback,
            10
        )
        
        # TF buffer for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timers
        self.create_timer(1.0/self.publish_rate, self.publish_costmap)
        self.create_timer(0.1, self.update_object_tracking)  # 10Hz tracking updates
        self.create_timer(0.5, self.publish_visualizations)  # 2Hz visualization updates
        
        # Data storage
        self.radar_points = []
        self.radar_velocities = {}  # {point_id: velocity_vector}
        self.last_update_time = self.get_clock().now()
        
        self.get_logger().info('Enhanced Radar Costmap Creator initialized')
        self.get_logger().info(f'Map dimensions: {self.map_width}x{self.map_height} cells at {self.map_resolution}m resolution')
        self.get_logger().info(f'Publishing rate: {self.publish_rate}Hz')
        if self.publish_to_realtime_map:
            self.get_logger().info(f'Publishing to realtime map topic: {self.realtime_map_topic}')
    
    def marker_callback(self, msg):
        """Process radar markers"""
        current_time = self.get_clock().now()
        
        # Extract point positions from markers
        new_points = []
        
        for marker in msg.markers:
            if marker.type == Marker.SPHERE and marker.ns == "radar_points":
                position = (
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                )
                
                # Store point with ID
                point_id = f"{marker.ns}_{marker.id}"
                new_points.append((point_id, position))
        
        # Update points list
        self.radar_points = new_points
        self.last_update_time = current_time
    
    def velocity_callback(self, msg):
        """Process velocity vectors"""
        # Clear previous velocities
        self.radar_velocities = {}
        
        for marker in msg.markers:
            if marker.type == Marker.ARROW and marker.ns == "velocity_vectors":
                # Extract velocity from arrow marker
                if len(marker.points) >= 2:
                    start = marker.points[0]
                    end = marker.points[1]
                    
                    # Calculate velocity vector
                    velocity = (
                        end.x - start.x,
                        end.y - start.y,
                        end.z - start.z
                    )
                    
                    # Store with corresponding point ID
                    point_id = f"radar_points_{marker.id}"
                    self.radar_velocities[point_id] = velocity
    
    def pointcloud_callback(self, msg):
        """Process radar point cloud data"""
        # This is an alternative data source that could be used
        # instead of or in addition to the marker callbacks
        pass
    
    def update_object_tracking(self):
        """Update object tracking with new measurements"""
        current_time = self.get_clock().now()
        
        # Update age of all tracked objects
        for obj_id, obj in self.tracked_objects.items():
            obj.update_age(current_time)
        
        # Remove old objects
        old_objects = [obj_id for obj_id, obj in self.tracked_objects.items() 
                      if obj.age > self.max_object_age]
        
        for obj_id in old_objects:
            del self.tracked_objects[obj_id]
        
        if old_objects:
            self.get_logger().debug(f'Removed {len(old_objects)} old objects')
        
        # Process new measurements
        if not self.radar_points:
            return
            
        # Get positions of existing objects
        if self.tracked_objects:
            existing_positions = np.array([obj.get_latest_position() 
                                         for obj in self.tracked_objects.values()])
            existing_ids = list(self.tracked_objects.keys())
            
            # Build KD-tree for efficient nearest neighbor search
            if len(existing_positions) > 0:
                tree = KDTree(existing_positions)
            else:
                tree = None
        else:
            tree = None
            existing_ids = []
        
        # Process each new point
        associated_points = set()
        
        for point_id, position in self.radar_points:
            # Get velocity if available
            velocity = self.radar_velocities.get(point_id, (0, 0, 0))
            
            # Try to associate with existing object
            if tree is not None:
                # Find nearest existing object
                distances, indices = tree.query([position], k=1)
                
                # If close enough, update existing object
                if distances[0] < self.object_tracking_distance:
                    obj_id = existing_ids[indices[0]]
                    self.tracked_objects[obj_id].update(position, velocity, current_time)
                    associated_points.add(point_id)
                    continue
            
            # If no association, create new object
            if point_id not in associated_points:
                new_obj = RadarObject(position, velocity, current_time, id=self.next_object_id)
                self.tracked_objects[self.next_object_id] = new_obj
                self.next_object_id += 1
        
        # Update predictions for all objects
        for obj in self.tracked_objects.values():
            obj.predict_future_positions(self.prediction_time)
    
    def update_costmap(self):
        """Update costmap based on tracked objects"""
        # Reset costmap
        self.radar_costmap = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Process each tracked object
        for obj_id, obj in self.tracked_objects.items():
            # Skip objects that haven't been updated recently
            if obj.age > self.max_object_age:
                continue
                
            # Get latest position and velocity
            position = obj.get_latest_position()
            if position is None:
                continue
                
            # Convert to grid coordinates
            grid_x = int((position[0] - self.map_origin_x) / self.map_resolution)
            grid_y = int((position[1] - self.map_origin_y) / self.map_resolution)
            
            # Skip if out of bounds
            if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
                continue
            
            # Calculate base cost (50 is default occupied)
            base_cost = 50
            
            # Calculate velocity-based cost
            velocity_magnitude = obj.get_velocity_magnitude()
            
            # Higher cost for moving objects
            if velocity_magnitude > self.min_velocity_threshold:
                # Higher cost for faster objects
                velocity_cost = min(int(velocity_magnitude * self.velocity_impact_factor), 50)
                
                # Even higher cost for approaching objects
                if obj.is_approaching():
                    velocity_cost = int(velocity_cost * 1.5)
                    
                dynamic_cost = min(base_cost + velocity_cost, 100)
            else:
                dynamic_cost = base_cost
            
            # Apply confidence factor
            dynamic_cost = int(dynamic_cost * obj.confidence)
            
            # Apply cost to grid and surrounding cells (inflation)
            # Larger inflation for faster objects
            inflation_radius = int((1.0 + velocity_magnitude * 0.5) / self.map_resolution)
            
            for dx in range(-inflation_radius, inflation_radius + 1):
                for dy in range(-inflation_radius, inflation_radius + 1):
                    nx, ny = grid_x + dx, grid_y + dy
                    
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        # Distance-based cost decay
                        distance = math.sqrt(dx**2 + dy**2)
                        if distance <= inflation_radius:
                            decay_factor = 1.0 - (distance / inflation_radius)
                            cell_cost = int(dynamic_cost * decay_factor)
                            
                            # Only update if new cost is higher than existing
                            if cell_cost > self.radar_costmap[ny, nx]:
                                self.radar_costmap[ny, nx] = cell_cost
            
            # Add predicted path to costmap with lower cost
            for i, pred_pos in enumerate(obj.predicted_positions):
                # Convert to grid coordinates
                pred_x = int((pred_pos[0] - self.map_origin_x) / self.map_resolution)
                pred_y = int((pred_pos[1] - self.map_origin_y) / self.map_resolution)
                
                # Skip if out of bounds
                if not (0 <= pred_x < self.map_width and 0 <= pred_y < self.map_height):
                    continue
                
                # Decrease cost for future positions based on time
                future_cost = int(dynamic_cost * (1.0 - (i+1) / (len(obj.predicted_positions) + 1)))
                
                # Smaller inflation for predicted positions
                pred_radius = max(1, int(inflation_radius * 0.7))
                
                for dx in range(-pred_radius, pred_radius + 1):
                    for dy in range(-pred_radius, pred_radius + 1):
                        nx, ny = pred_x + dx, pred_y + dy
                        
                        if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                            # Distance-based cost decay
                            distance = math.sqrt(dx**2 + dy**2)
                            if distance <= pred_radius:
                                decay_factor = 1.0 - (distance / pred_radius)
                                cell_cost = int(future_cost * decay_factor)
                                
                                # Only update if new cost is higher than existing
                                if cell_cost > self.radar_costmap[ny, nx]:
                                    self.radar_costmap[ny, nx] = cell_cost
    
    def publish_costmap(self):
        """Publish the radar costmap"""
        # Update costmap
        self.update_costmap()
        
        # Create occupancy grid message
        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = self.map_frame
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        
        costmap_msg.info.resolution = self.map_resolution
        costmap_msg.info.width = self.map_width
        costmap_msg.info.height = self.map_height
        costmap_msg.info.origin.position.x = self.map_origin_x
        costmap_msg.info.origin.position.y = self.map_origin_y
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        
        # Convert numpy array to 1D list
        costmap_msg.data = self.radar_costmap.flatten().tolist()
        
        # Publish costmap
        self.costmap_publisher.publish(costmap_msg)
        
        # Also publish to realtime map if enabled
        if self.publish_to_realtime_map:
            # Create a copy of the message for realtime map
            realtime_grid_msg = OccupancyGrid()
            realtime_grid_msg.header = costmap_msg.header
            realtime_grid_msg.info = costmap_msg.info
            realtime_grid_msg.data = costmap_msg.data
            
            # Publish to realtime map topic
            self.realtime_map_publisher.publish(realtime_grid_msg)
    
    def publish_visualizations(self):
        """Publish visualization markers for tracked objects and predictions"""
        if not self.enable_visualization:
            return
            
        # Create marker arrays
        tracked_markers = MarkerArray()
        prediction_markers = MarkerArray()
        
        # Current time for marker timestamps
        now = self.get_clock().now()
        
        # Create markers for each tracked object
        for obj_id, obj in self.tracked_objects.items():
            # Skip objects that haven't been updated recently
            if obj.age > self.max_object_age:
                continue
                
            position = obj.get_latest_position()
            if position is None:
                continue
                
            velocity = obj.get_latest_velocity()
            velocity_magnitude = obj.get_velocity_magnitude()
            
            # Object marker
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now.to_msg()
            marker.ns = "tracked_radar_objects"
            marker.id = obj_id
            marker.type = Marker.SPHERE
            
            # Set position
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.w = 1.0
            
            # Size based on confidence and velocity
            size_factor = 0.5 + 0.5 * obj.confidence + 0.2 * velocity_magnitude
            marker.scale.x = size_factor
            marker.scale.y = size_factor
            marker.scale.z = size_factor
            
            # Color based on velocity (red for fast, green for slow)
            marker.color.a = 0.8
            if velocity_magnitude > self.min_velocity_threshold:
                # Red for fast approaching objects
                if obj.is_approaching():
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    # Yellow for fast receding objects
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
            else:
                # Green for slow/stationary objects
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            
            marker.lifetime.sec = 1  # 1 second lifetime
            
            tracked_markers.markers.append(marker)
            
            # Velocity vector marker
            if velocity_magnitude > self.min_velocity_threshold:
                vel_marker = Marker()
                vel_marker.header.frame_id = self.map_frame
                vel_marker.header.stamp = now.to_msg()
                vel_marker.ns = "tracked_radar_velocities"
                vel_marker.id = obj_id
                vel_marker.type = Marker.ARROW
                
                # Start point
                p1 = Point()
                p1.x = position[0]
                p1.y = position[1]
                p1.z = position[2]
                
                # End point
                p2 = Point()
                p2.x = position[0] + velocity[0]
                p2.y = position[1] + velocity[1]
                p2.z = position[2] + velocity[2]
                
                vel_marker.points = [p1, p2]
                
                # Arrow size
                vel_marker.scale.x = 0.1  # shaft diameter
                vel_marker.scale.y = 0.2  # head diameter
                vel_marker.scale.z = 0.2  # head length
                
                # Same color as object
                vel_marker.color = marker.color
                vel_marker.lifetime.sec = 1
                
                tracked_markers.markers.append(vel_marker)
            
            # Predicted path marker
            if obj.predicted_positions:
                path_marker = Marker()
                path_marker.header.frame_id = self.map_frame
                path_marker.header.stamp = now.to_msg()
                path_marker.ns = "predicted_paths"
                path_marker.id = obj_id
                path_marker.type = Marker.LINE_STRIP
                
                # Add current position
                points = [Point(x=position[0], y=position[1], z=position[2])]
                
                # Add predicted positions
                for pred_pos in obj.predicted_positions:
                    p = Point()
                    p.x = pred_pos[0]
                    p.y = pred_pos[1]
                    p.z = pred_pos[2]
                    points.append(p)
                
                path_marker.points = points
                
                # Line properties
                path_marker.scale.x = 0.05  # line width
                
                # Color based on approach/recede (red for approaching, blue for receding)
                path_marker.color.a = 0.8
                if obj.is_approaching():
                    path_marker.color.r = 1.0
                    path_marker.color.g = 0.0
                    path_marker.color.b = 0.0
                else:
                    path_marker.color.r = 0.0
                    path_marker.color.g = 0.0
                    path_marker.color.b = 1.0
                
                path_marker.lifetime.sec = 1
                
                prediction_markers.markers.append(path_marker)
        
        # Publish markers
        if tracked_markers.markers:
            self.tracked_objects_pub.publish(tracked_markers)
        
        if prediction_markers.markers:
            self.predicted_paths_pub.publish(prediction_markers)

def main(args=None):
    rclpy.init(args=args)
    
    node = RadarCostmapCreator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 