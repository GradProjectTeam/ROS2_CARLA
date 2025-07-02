#!/usr/bin/env python3
"""
Enhanced DWA Planner Node

This node implements an enhanced version of the Dynamic Window Approach (DWA) for local path planning.
It combines the ROS2 integration from dwa_planner_node.py with advanced features from DWA 35_new5.py
including improved obstacle detection, parking spot detection, and advanced path planning.

Author: Mostafa
"""

import numpy as np
import math
import threading
import time
from queue import Queue, Empty
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseStamped, Twist, PoseArray, Pose
from std_msgs.msg import Header, ColorRGBA, Int32MultiArray, Bool
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration
from tf2_geometry_msgs import do_transform_pose
from tf2_msgs.msg import TFMessage

# Fix the exceptions import
# In ROS2, these exceptions are directly in tf2_ros
try:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
except ImportError:
    # Define fallback exception classes if needed
    class LookupException(Exception):
        pass
    
    class ConnectivityException(Exception):
        pass
    
    class ExtrapolationException(Exception):
        pass

# Replace tf2_transformations with the correct imports for ROS2
try:
    # Try to import from tf_transformations package (ROS2)
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except ImportError:
    try:
        # Try to import from transforms3d as a fallback
        from transforms3d.euler import euler2quat, quat2euler
        
        def quaternion_from_euler(roll, pitch, yaw):
            return euler2quat(roll, pitch, yaw)
        
        def euler_from_quaternion(quaternion):
            return quat2euler(quaternion)
    except ImportError:
        # Define a simple implementation if neither is available
        def quaternion_from_euler(roll, pitch, yaw):
            # Simple implementation of quaternion from Euler angles
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            q = [0, 0, 0, 0]
            q[0] = sr * cp * cy - cr * sp * sy  # x
            q[1] = cr * sp * cy + sr * cp * sy  # y
            q[2] = cr * cp * sy - sr * sp * cy  # z
            q[3] = cr * cp * cy + sr * sp * sy  # w
            return q

        def euler_from_quaternion(quaternion):
            # Simple implementation to extract yaw from quaternion
            x, y, z, w = quaternion
            
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            # Pitch (y-axis rotation)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
            else:
                pitch = math.asin(sinp)
            
            # Yaw (z-axis rotation)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return (roll, pitch, yaw)

@dataclass
class ParkingSpot:
    """Represents a detected parking spot in the environment"""
    x: float
    y: float
    width: float
    length: float
    angle: float
    is_safe: bool = True


class EnhancedDWAPlanner(Node):
    def __init__(self):
        super().__init__('enhanced_dwa_planner')
        
        # Declare parameters
        self.declare_parameter('map_frame_id', 'local_map_link')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('waypoints_topic', '/carla/waypoints')
        self.declare_parameter('waypoint_markers_topic', '/carla/waypoint_markers')
        self.declare_parameter('binary_map_topic', '/combined_binary_map')
        self.declare_parameter('path_topic', '/dwa/planned_path')
        self.declare_parameter('cmd_vel_topic', '/dwa/cmd_vel')
        self.declare_parameter('parking_spots_topic', '/dwa/parking_spots')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('max_speed', 30.0)
        self.declare_parameter('min_speed', -4.0)
        self.declare_parameter('max_yaw_rate', 1.5)
        self.declare_parameter('max_accel', 8.0)
        self.declare_parameter('max_delta_yaw_rate', 0.8)
        self.declare_parameter('dt', 0.2)
        self.declare_parameter('predict_time', 2.0)
        self.declare_parameter('to_goal_cost_gain', 1.0)
        self.declare_parameter('speed_cost_gain', 0.3)
        self.declare_parameter('obstacle_cost_gain', 10.0)
        self.declare_parameter('path_following_gain', 0.002)
        self.declare_parameter('lookahead_distance', 3.0)
        self.declare_parameter('obstacle_threshold', 100)
        self.declare_parameter('safe_distance', 2.0)
        self.declare_parameter('default_lane_width', 3.5)
        self.declare_parameter('lane_width_factor', 0.8)
        self.declare_parameter('min_parking_width', 2.5)
        self.declare_parameter('min_parking_length', 5.0)
        self.declare_parameter('enable_parking_detection', True)
        self.declare_parameter('extended_predict_time', 8.0)  # Longer prediction time for visualization
        self.declare_parameter('parking_gray_threshold', 0.4)  # Threshold for gray lane detection
        self.declare_parameter('parking_visualization_lifetime', 5.0)  # Lifetime for parking spot visualization
        self.declare_parameter('prevent_unnecessary_stops', True)  # New parameter to control stopping behavior
        self.declare_parameter('min_continuous_speed', 5.0)  # Minimum speed to maintain when not stopping
        self.declare_parameter('use_lane_based_obstacles', True)
        self.declare_parameter('ignore_obstacles_outside_lane', True)
        self.declare_parameter('strict_lane_obstacle_detection', True)
        self.declare_parameter('obstacle_influence_radius', 1.5)
        self.declare_parameter('obstacle_detection_range', 20.0)
        self.declare_parameter('debug_obstacle_detection', True)
        self.declare_parameter('stop_for_lane_obstacles', True)
        self.declare_parameter('min_obstacle_count', 2)
        self.declare_parameter('obstacle_detection_threshold', 0.1)
        self.declare_parameter('start_point_offset', 0.8)  # Offset for the start point of the path
        self.declare_parameter('min_obstacle_distance', 2.5)  # Minimum distance to obstacles
        self.declare_parameter('path_smoothing_factor', 0.7)  # Factor for path smoothing
        self.declare_parameter('lateral_safety_margin', 0.5)  # Lateral safety margin
        self.declare_parameter('obstacle_weight_decay', 0.8)  # Weight decay for obstacles with distance
        self.declare_parameter('dynamic_obstacle_prediction', True)  # Enable dynamic obstacle prediction
        self.declare_parameter('adaptive_lookahead', True)  # Enable adaptive lookahead distance
        
        # New parameters for enhanced obstacle avoidance
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('emergency_brake_distance', 8.0)
        self.declare_parameter('obstacle_path_pruning', True)
        self.declare_parameter('obstacle_velocity_factor', 1.5)
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').get_parameter_value().string_value
        self.waypoints_topic = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        self.waypoint_markers_topic = self.get_parameter('waypoint_markers_topic').get_parameter_value().string_value
        self.binary_map_topic = self.get_parameter('binary_map_topic').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.parking_spots_topic = self.get_parameter('parking_spots_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # DWA parameters
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = 0.0  # Set minimum speed to 0.0 to prevent backward motion
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.max_accel = 8.0  # Set to match reference implementation
        self.max_delta_yaw_rate = 0.8  # Set to match reference implementation
        self.dt = 0.2  # Set to match reference implementation
        self.predict_time = 2.0  # Set to match reference implementation
        self.to_goal_cost_gain = 1.0  # Set to match reference implementation
        self.speed_cost_gain = 0.3  # Set to match reference implementation
        self.obstacle_cost_gain = 10.0  # Set to match reference implementation
        self.path_following_gain = 0.002  # Set to match reference implementation
        self.lookahead_distance = 3.0  # Set to match reference implementation
        self.obstacle_threshold = 0.7  # Set to match reference implementation (as a normalized value)
        self.safe_distance = 2.0  # Set to match reference implementation
        self.default_lane_width = self.get_parameter('default_lane_width').get_parameter_value().double_value
        self.lane_width_factor = self.get_parameter('lane_width_factor').get_parameter_value().double_value
        self.min_parking_width = self.get_parameter('min_parking_width').get_parameter_value().double_value
        self.min_parking_length = self.get_parameter('min_parking_length').get_parameter_value().double_value
        self.enable_parking_detection = self.get_parameter('enable_parking_detection').get_parameter_value().bool_value
        self.extended_predict_time = self.get_parameter('extended_predict_time').get_parameter_value().double_value
        self.parking_gray_threshold = self.get_parameter('parking_gray_threshold').get_parameter_value().double_value
        self.parking_visualization_lifetime = self.get_parameter('parking_visualization_lifetime').get_parameter_value().double_value
        self.prevent_unnecessary_stops = self.get_parameter('prevent_unnecessary_stops').get_parameter_value().bool_value
        self.min_continuous_speed = self.get_parameter('min_continuous_speed').get_parameter_value().double_value
        self.use_lane_based_obstacles = self.get_parameter('use_lane_based_obstacles').get_parameter_value().bool_value
        self.ignore_obstacles_outside_lane = self.get_parameter('ignore_obstacles_outside_lane').get_parameter_value().bool_value
        self.strict_lane_obstacle_detection = self.get_parameter('strict_lane_obstacle_detection').get_parameter_value().bool_value
        self.obstacle_influence_radius = self.get_parameter('obstacle_influence_radius').get_parameter_value().double_value
        self.obstacle_detection_range = self.get_parameter('obstacle_detection_range').get_parameter_value().double_value
        self.debug_obstacle_detection = self.get_parameter('debug_obstacle_detection').get_parameter_value().bool_value
        self.stop_for_lane_obstacles = self.get_parameter('stop_for_lane_obstacles').get_parameter_value().bool_value
        self.min_obstacle_count = self.get_parameter('min_obstacle_count').get_parameter_value().integer_value
        self.obstacle_detection_threshold = self.get_parameter('obstacle_detection_threshold').get_parameter_value().double_value
        self.start_point_offset = self.get_parameter('start_point_offset').get_parameter_value().double_value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').get_parameter_value().double_value
        self.path_smoothing_factor = self.get_parameter('path_smoothing_factor').get_parameter_value().double_value
        self.lateral_safety_margin = self.get_parameter('lateral_safety_margin').get_parameter_value().double_value
        self.obstacle_weight_decay = self.get_parameter('obstacle_weight_decay').get_parameter_value().double_value
        self.dynamic_obstacle_prediction = self.get_parameter('dynamic_obstacle_prediction').get_parameter_value().bool_value
        self.adaptive_lookahead = self.get_parameter('adaptive_lookahead').get_parameter_value().bool_value
        
        # Get new parameters for enhanced obstacle avoidance
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').get_parameter_value().bool_value
        self.emergency_brake_distance = self.get_parameter('emergency_brake_distance').get_parameter_value().double_value
        self.obstacle_path_pruning = self.get_parameter('obstacle_path_pruning').get_parameter_value().bool_value
        self.obstacle_velocity_factor = self.get_parameter('obstacle_velocity_factor').get_parameter_value().double_value
        
        # Set up QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.path_publisher = self.create_publisher(
            Path,
            self.path_topic,
            qos_profile
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            qos_profile
        )
        
        # Create visualization marker publisher
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.path_topic + '_markers',
            qos_profile
        )
        
        # Create parking spots marker publisher
        self.parking_spots_publisher = self.create_publisher(
            MarkerArray,
            self.parking_spots_topic,
            qos_profile
        )
        
        # Create obstacle detection publisher
        self.obstacle_detected_publisher = self.create_publisher(
            Bool,
            '/dwa/obstacle_detected',
            qos_profile
        )
        
        # Create subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.binary_map_topic,
            self.map_callback,
            qos_profile
        )
        
        self.waypoints_subscriber = self.create_subscription(
            PoseArray,
            self.waypoints_topic,
            self.waypoints_callback,
            qos_profile
        )
        
        # Also subscribe to waypoint metadata to get lane information
        self.metadata_subscriber = self.create_subscription(
            Int32MultiArray,
            self.waypoints_topic + "_metadata",
            self.metadata_callback,
            qos_profile
        )
        
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            self.waypoint_markers_topic,
            self.markers_callback,
            qos_profile
        )
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize data structures
        self.cost_map = None
        self.binary_map = None  # Initialize binary_map attribute
        self.map_resolution = 0.5  # Default value, will be updated from map
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 1000  # Default value, will be updated from map
        self.map_height = 1000  # Default value, will be updated from map
        self.waypoints = []
        self.lane_widths = []  # Store lane widths for each waypoint
        self.current_lane_width = self.default_lane_width
        self.parking_spots = []
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_v = 2.0  # Initialize with a higher positive velocity for better start
        self.current_omega = 0.0
        self.lane_obstacle_positions = []  # Initialize empty list for lane obstacle positions
        
        # Planner state tracking
        self.planner_state = "READY"  # Start in READY state instead of INITIALIZING
        self.stop_reason = None
        self.last_state_change = self.get_clock().now()
        self.last_state_log = self.get_clock().now()
        
        # Locks for thread safety
        self.map_lock = threading.Lock()
        self.waypoints_lock = threading.Lock()
        self.state_lock = threading.Lock()
        
        # Create timer for planning and control
        self.timer = self.create_timer(1.0 / self.publish_rate, self.planning_callback)
        
        # Create timer for parking spot detection (at a slower rate)
        if self.enable_parking_detection:
            self.parking_timer = self.create_timer(2.0, self.detect_parking_spots)
        
        # Status variables
        self.has_map = False
        self.has_waypoints = False
        self.last_plan_time = self.get_clock().now()
        
        self.get_logger().info("Enhanced DWA Planner node initialized")
    
    def map_callback(self, msg):
        """Process incoming binary map"""
        try:
            with self.map_lock:
                # Extract map data
                self.map_width = msg.info.width
                self.map_height = msg.info.height
                self.map_resolution = msg.info.resolution
                self.map_origin_x = msg.info.origin.position.x
                self.map_origin_y = msg.info.origin.position.y
                
                # Convert from 1D array to 2D numpy array
                self.cost_map = np.array(msg.data).reshape((self.map_height, self.map_width)) / 100.0
                self.binary_map = msg  # Store the original binary map message
                self.has_map = True
                
                self.get_logger().debug(f"Received map {self.map_width}x{self.map_height} with resolution {self.map_resolution}")
        except Exception as e:
            self.get_logger().error(f"Error processing map: {e}")
    
    def waypoints_callback(self, msg):
        """Process incoming waypoints"""
        try:
            with self.waypoints_lock:
                # Extract waypoints
                waypoints = []
                for pose in msg.poses:
                    waypoints.append({
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    })
                
                if waypoints:
                    self.waypoints = waypoints
                    self.has_waypoints = True
                    self.get_logger().debug(f"Received {len(waypoints)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoints: {e}")
    
    def metadata_callback(self, msg):
        """Process waypoint metadata including lane information"""
        try:
            # Extract lane information
            metadata = msg.data
            if len(metadata) >= 3 and len(metadata) % 3 == 0:
                # Extract lane widths
                lane_widths = []
                for i in range(0, len(metadata), 3):
                    road_id = metadata[i]
                    lane_id = metadata[i+1]
                    lane_type = metadata[i+2]
                    
                    # Lane width will be updated from markers_callback
                    # This is just a placeholder
                    lane_widths.append(self.default_lane_width)
                
                with self.waypoints_lock:
                    if len(lane_widths) == len(self.waypoints):
                        self.lane_widths = lane_widths
                        self.get_logger().debug(f"Processed metadata for {len(lane_widths)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint metadata: {e}")
    
    def markers_callback(self, msg):
        """Process waypoint markers to extract lane width information"""
        try:
            # Extract lane width from marker data
            for marker in msg.markers:
                if hasattr(marker, 'text') and marker.text:
                    # Try to extract lane width information from text
                    if 'lane_width' in marker.text.lower():
                        try:
                            # Parse lane width value
                            parts = marker.text.split(':')
                            if len(parts) >= 2:
                                lane_width = float(parts[1].strip())
                                if 0.5 <= lane_width <= 10.0:  # Sanity check
                                    self.current_lane_width = lane_width
                                    self.get_logger().debug(f"Updated lane width to {lane_width}")
                        except:
                            pass
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint markers: {e}")
    
    def update_vehicle_state(self, vehicle_pose):
        """Update vehicle state from TF"""
        try:
            # Get vehicle position and orientation from TF
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.vehicle_frame_id,
                rclpy.time.Time()
            )
            
            with self.state_lock:
                # Update position
                self.current_x = transform.transform.translation.x
                self.current_y = transform.transform.translation.y
                
                # Update orientation (extract yaw from quaternion)
                q = transform.transform.rotation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.current_theta = math.atan2(siny_cosp, cosy_cosp)
                
                # Note: velocity would ideally come from odometry
                # For now, we'll keep the existing velocity
                
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not update vehicle state: {e}")
            return False
    
    def detect_parking_spots(self):
        """Detect potential parking spots in the environment"""
        if not self.has_map:
            return
        
        try:
            with self.map_lock:
                if self.cost_map is None:
                    return
                
                # Get current vehicle position
                with self.state_lock:
                    vehicle_pos = (self.current_x, self.current_y)
                
                # Convert from map coordinates to grid coordinates
                grid_x = int((vehicle_pos[0] - self.map_origin_x) / self.map_resolution)
                grid_y = int((vehicle_pos[1] - self.map_origin_y) / self.map_resolution)
                vehicle_grid_pos = (grid_x, grid_y)
                
                # Find parking spots
                parking_spots = self._find_safe_parking_spots(vehicle_grid_pos)
                
                if parking_spots:
                    self.parking_spots = parking_spots
                    self.publish_parking_spots()
                    self.get_logger().info(f"Found {len(parking_spots)} potential parking spots")
        except Exception as e:
            self.get_logger().error(f"Error detecting parking spots: {e}")
    
    def _find_safe_parking_spots(self, vehicle_pos):
        """Find safe parking spots in the environment using the binary map"""
        if self.cost_map is None:
            return []
        
        import cv2
        
        # Create a mask for potential parking areas
        # We're looking for areas that are not completely occupied (black) but also not completely free (white)
        # These gray areas often represent lane markings or areas suitable for parking
        parking_mask = ((self.cost_map > 0.1) & (self.cost_map < self.parking_gray_threshold)).astype(np.uint8)
        
        # Apply some morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        parking_mask = cv2.morphologyEx(parking_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find edges in the parking mask
        edges = cv2.Canny((parking_mask * 255).astype(np.uint8), 50, 150)
        
        # Find points that are both in the parking mask and on edges
        edge_points = np.argwhere((parking_mask == 1) & (edges > 0))
        
        # Filter points to be within a reasonable distance from the vehicle
        max_search_distance = 40  # meters
        max_grid_distance = int(max_search_distance / self.map_resolution)
        
        parking_spots = []
        for y, x in edge_points:  # Note: OpenCV uses (y, x) indexing
            # Check if point is within search distance
            if abs(x - vehicle_pos[0]) > max_grid_distance or abs(y - vehicle_pos[1]) > max_grid_distance:
                continue
                
            if self._check_parking_space_on_lane(y, x):
                angle = self._calculate_parking_angle(y, x)
                if self._is_safe_distance(y, x):
                    # Convert grid coordinates back to map coordinates
                    map_x = x * self.map_resolution + self.map_origin_x
                    map_y = y * self.map_resolution + self.map_origin_y
                    
                    parking_spots.append(ParkingSpot(
                        x=map_x,
                        y=map_y,
                        width=self.min_parking_width,
                        length=self.min_parking_length,
                        angle=angle
                    ))
        
        # Filter out duplicate spots that are too close to each other
        filtered_spots = []
        min_spot_distance = max(self.min_parking_width, self.min_parking_length) * 0.8
        
        for spot in parking_spots:
            # Check if this spot is far enough from all existing filtered spots
            is_unique = True
            for existing_spot in filtered_spots:
                dist = math.hypot(spot.x - existing_spot.x, spot.y - existing_spot.y)
                if dist < min_spot_distance:
                    is_unique = False
                    break
            
            if is_unique:
                filtered_spots.append(spot)
        
        self.get_logger().debug(f"Found {len(filtered_spots)} parking spots after filtering")
        return filtered_spots
    
    def _check_parking_space_on_lane(self, y, x):
        """Check if an area is suitable for parking on a gray lane"""
        if self.cost_map is None:
            return False
        
        for angle in [0, np.pi/2]:
            width = int(self.min_parking_width / self.map_resolution * np.cos(angle) + 
                        self.min_parking_length / self.map_resolution * np.sin(angle))
            length = int(self.min_parking_width / self.map_resolution * np.sin(angle) + 
                         self.min_parking_length / self.map_resolution * np.cos(angle))
            
            y1, x1 = max(0, y - width//2), max(0, x - length//2)
            y2, x2 = min(self.cost_map.shape[0], y + width//2), min(self.cost_map.shape[1], x + length//2)
            
            area = self.cost_map[y1:y2, x1:x2]
            
            # Check if the area is suitable for parking:
            # - Most of the area should be gray lane (between 0.1 and parking_gray_threshold)
            if area.size > 0:
                gray_pixels = np.sum((area > 0.1) & (area < self.parking_gray_threshold))
                gray_ratio = gray_pixels / area.size
                
                # At least 70% of the area should be gray lane
                if gray_ratio > 0.7:
                    return True
        
        return False
    
    def _calculate_parking_angle(self, y, x):
        """Calculate the orientation angle for a parking spot"""
        import cv2
        
        edges = cv2.Canny((self.cost_map * 255).astype(np.uint8), 50, 150)
        if edges[y, x] > 0:
            grad_y = cv2.Sobel(edges, cv2.CV_64F, 1, 0, ksize=3)
            grad_x = cv2.Sobel(edges, cv2.CV_64F, 0, 1, ksize=3)
            angle = np.arctan2(grad_y[y, x], grad_x[y, x])
            return angle + np.pi/2
        return 0.0
    
    def _is_safe_distance(self, y, x):
        """Check if a location has safe distance from obstacles"""
        if self.cost_map is None:
            return False
        
        window_size = int(self.safe_distance / self.map_resolution)
        y1, x1 = max(0, y - window_size), max(0, x - window_size)
        y2, x2 = min(self.cost_map.shape[0], y + window_size), min(self.cost_map.shape[1], x + window_size)
        area = self.cost_map[y1:y2, x1:x2]
        return np.mean(area) < 0.3
    
    def find_obstacles_on_path(self, path_points, binary_map, vehicle_pose):
        """Find obstacles on the planned path."""
        # Extract vehicle position
        x = vehicle_pose.position.x
        y = vehicle_pose.position.y
        
        # Extract orientation (yaw) from quaternion
        quat = vehicle_pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Initialize variables
        has_obstacles = False
        obstacle_count = 0
        closest_obstacle_distance = float('inf')
        
        # Clear previous obstacle positions
        self.lane_obstacle_positions = []
        
        # Extract the binary map data as a numpy array
        binary_data = np.array(binary_map.data).reshape((self.map_height, self.map_width))
        
        # Check path points for obstacles
        for i, (px, py) in enumerate(path_points):
            # Get distance from vehicle to path point
            distance = math.sqrt((px - x) ** 2 + (py - y) ** 2)
            
            # Skip points that are too close to the vehicle - use a smaller distance to ensure
            # we don't miss obstacles very close to the vehicle
            if distance < 0.3:  # Reduced from 0.5 to check closer to the vehicle
                continue
                
            # Convert path point to map coordinates
            map_x = int((px - self.map_origin_x) / self.map_resolution)
            map_y = int((py - self.map_origin_y) / self.map_resolution)
            
            # Check if point is within map bounds
            if (map_x < 0 or map_x >= self.map_width or
                map_y < 0 or map_y >= self.map_height):
                continue
                
            # Check if point is an obstacle
            if binary_data[map_y, map_x] > self.obstacle_threshold:
                obstacle_count += 1
                
                # Store obstacle position for visualization and avoidance
                self.lane_obstacle_positions.append((px, py))
                
                # Update closest obstacle distance
                if distance < closest_obstacle_distance:
                    closest_obstacle_distance = distance
                
                # Enhanced obstacle detection: Check if emergency stop is needed
                if self.emergency_stop_enabled and distance < self.emergency_brake_distance:
                    self.get_logger().warn(f"Emergency stop triggered: Obstacle at {distance:.2f}m")
                    has_obstacles = True
        
        # Set has_obstacles based on obstacle count and threshold
        if obstacle_count >= self.min_obstacle_count:
            has_obstacles = True
            self.get_logger().info(f"Obstacles detected on path: {obstacle_count} obstacles, closest at {closest_obstacle_distance:.2f}m")
        
        # Additional check for very close obstacles - always stop if obstacles are very close
        if closest_obstacle_distance < 2.0:  # Stop for obstacles within 2 meters
            has_obstacles = True
            self.get_logger().warn(f"Close obstacle detected at {closest_obstacle_distance:.2f}m - stopping")
        
        # If no obstacles were detected but we have a very close obstacle distance, something's wrong
        # This likely means we have a false positive - log it but don't stop
        if has_obstacles and closest_obstacle_distance > 10.0:
            self.get_logger().warn(f"Possible false obstacle detection: {obstacle_count} obstacles, but closest is {closest_obstacle_distance:.2f}m away")
            # Only consider it a real obstacle if we have multiple detections
            has_obstacles = obstacle_count >= self.min_obstacle_count * 2
        
        # If we have obstacles, make sure to warn about it
        if has_obstacles:
            self.get_logger().warn("Emergency stop: Obstacle detected on path")
        
        return has_obstacles
    
    def is_obstacle_in_lane(self, point, vehicle_pose):
        """Check if a point is within the vehicle's lane."""
        # Get vehicle position and orientation
        vehicle_x, vehicle_y = vehicle_pose.position.x, vehicle_pose.position.y
        quat = vehicle_pose.orientation
        _, _, vehicle_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Calculate lane boundaries based on vehicle orientation
        lane_width = self.default_lane_width * self.lane_width_factor
        half_width = lane_width / 2.0
        
        # Calculate the vehicle's forward direction vector
        forward_x = math.cos(vehicle_yaw)
        forward_y = math.sin(vehicle_yaw)
        
        # Calculate the vehicle's right direction vector (perpendicular to forward)
        right_x = -forward_y
        right_y = forward_x
        
        # Calculate the vector from vehicle to point
        dx = point[0] - vehicle_x
        dy = point[1] - vehicle_y
        
        # Project the vector onto the forward and right directions
        forward_proj = dx * forward_x + dy * forward_y
        right_proj = dx * right_x + dy * right_y
        
        # Check if the point is within the lane boundaries and in front of the vehicle
        is_in_lane = abs(right_proj) <= half_width
        is_in_front = forward_proj > 0
        
        # Add additional constraint for strict lane detection
        if self.strict_lane_obstacle_detection:
            # Further restrict lane width for stricter detection
            strict_half_width = half_width * 0.8
            is_in_lane = abs(right_proj) <= strict_half_width
            
            # Only consider obstacles within a reasonable distance ahead
            max_forward_distance = self.obstacle_detection_range
            is_in_front = forward_proj > 0 and forward_proj < max_forward_distance
        
        return is_in_lane and is_in_front
    
    def find_closest_waypoint(self, vehicle_pose):
        """Find the closest waypoint to the current vehicle position."""
        if not self.waypoints:
            return None
            
        # Get vehicle position
        vehicle_x = vehicle_pose.position.x
        vehicle_y = vehicle_pose.position.y
        
        # Find closest waypoint
        min_dist = float('inf')
        closest_idx = None
        
        for i, waypoint in enumerate(self.waypoints):
            dist = math.hypot(waypoint['x'] - vehicle_x, waypoint['y'] - vehicle_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx
    
    def get_target_waypoint(self, closest_idx, vehicle_pose):
        """Get a target waypoint with lookahead distance."""
        if closest_idx is None or not self.waypoints:
            return None, None
            
        # Get vehicle position
        vehicle_x = vehicle_pose.position.x
        vehicle_y = vehicle_pose.position.y
        
        # Initialize variables
        target_idx = closest_idx
        lookahead_point = None
        
        # Look ahead from the closest waypoint to find a point at lookahead_distance
        for i in range(closest_idx, len(self.waypoints)):
            waypoint = self.waypoints[i]
            # Calculate distance from vehicle to this waypoint
            dist = math.hypot(waypoint['x'] - vehicle_x, waypoint['y'] - vehicle_y)
            
            # If we've found a point beyond the lookahead distance, use it
            if dist >= self.lookahead_distance:
                target_idx = i
                lookahead_point = waypoint
                break
                
        # If no point was found at lookahead distance, use the last waypoint
        if lookahead_point is None and self.waypoints:
            target_idx = len(self.waypoints) - 1
            lookahead_point = self.waypoints[target_idx]
        
        # If adaptive lookahead is enabled, adjust lookahead distance based on velocity
        if self.adaptive_lookahead and lookahead_point:
            # Increase lookahead distance at higher speeds
            speed_factor = min(1.0, abs(self.current_v) / self.max_speed)
            adjusted_idx = min(target_idx + int(speed_factor * 10), len(self.waypoints) - 1)
            if adjusted_idx > target_idx:
                target_idx = adjusted_idx
                lookahead_point = self.waypoints[target_idx]
                
        return target_idx, lookahead_point
    
    def dwa_control(self, vehicle_pose, target_waypoint):
        """Dynamic Window Approach control."""
        # Get current state
        x = vehicle_pose.position.x
        y = vehicle_pose.position.y
        
        # Extract orientation (yaw) from quaternion
        quat = vehicle_pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Get goal position
        gx = target_waypoint['x']
        gy = target_waypoint['y']
        
        # Current velocity and angular velocity (ideally from odometry)
        # For now, use the stored values, but ensure they're not negative or zero
        v = max(0.5, self.current_v)  # Force minimum positive velocity
        omega = self.current_omega
        
        # Define dynamic window based on current state
        dw = self._calc_dynamic_window(v, omega)
        
        # Ensure dynamic window always includes positive velocities
        if dw[0] <= 0.0 and dw[1] <= 0.0:
            dw[1] = 2.0  # Force a positive velocity option
        
        # Calculate best trajectory
        v_opt, omega_opt, best_trajectory = self._calc_control_and_trajectory(x, y, yaw, v, omega, dw, gx, gy)
        
        # Force positive velocity if no obstacles
        if v_opt < 0.5:
            v_opt = 2.0  # Force a reasonable forward velocity
            # Recalculate trajectory with forced velocity
            _, _, best_trajectory = self._calc_control_and_trajectory(x, y, yaw, v, omega, [v_opt, v_opt, dw[2], dw[3]], gx, gy)
        
        # Update current velocity and angular velocity
        self.current_v = v_opt
        self.current_omega = omega_opt
        
        # Extract trajectory data
        if best_trajectory:
            path_x = [point[0] for point in best_trajectory]
            path_y = [point[1] for point in best_trajectory]
            path_yaw = [point[2] for point in best_trajectory]
            path_v = [point[3] for point in best_trajectory]
        else:
            # If no trajectory found, return current position with default velocity
            path_x = [x]
            path_y = [y]
            path_yaw = [yaw]
            path_v = [2.0]  # Force positive velocity in output
        
        # Return control commands and trajectory
        return path_x, path_y, path_yaw, path_v, [v_opt, omega_opt]
        
    def _calc_dynamic_window(self, v, omega):
        """Calculate the dynamic window based on current state."""
        # Window from robot specification
        v_min = self.min_speed
        v_max = self.max_speed
        omega_min = -self.max_yaw_rate
        omega_max = self.max_yaw_rate
        
        # Limit the maximum yaw rate based on current velocity
        # Higher velocities should have more limited steering
        if v > 0.5:
            # Reduce max steering angle at higher speeds - make this more aggressive
            speed_factor = min(1.0, v / self.max_speed)
            # Reduce max steering to 40% at maximum speed for stability
            omega_min = -self.max_yaw_rate * (1.0 - 0.6 * speed_factor)
            omega_max = self.max_yaw_rate * (1.0 - 0.6 * speed_factor)
            
            # Additional hard limit on steering at higher speeds
            if v > 5.0:
                max_steering_limit = 0.5  # Hard limit steering to 0.5 rad/s at higher speeds
                omega_min = max(omega_min, -max_steering_limit)
                omega_max = min(omega_max, max_steering_limit)
        
        # Window from motion model (using reference implementation approach)
        v_lower = max(v_min, v - self.max_accel * self.dt)
        v_upper = min(v_max, v + self.max_accel * self.dt)
        
        # Add dampening effect to limit angular velocity changes
        # Reduce the maximum rate of change of angular velocity
        omega_delta = self.max_delta_yaw_rate * self.dt * 0.7  # Reduce maximum change by 30%
        omega_lower = max(omega_min, omega - omega_delta)
        omega_upper = min(omega_max, omega + omega_delta)
        
        # Add a bias toward straightening the steering when turning sharply
        if abs(omega) > 0.5:
            # If current steering is high, bias toward reducing it
            straightening_bias = 0.1 * (1.0 if omega < 0 else -1.0)
            omega_lower += straightening_bias
            omega_upper += straightening_bias
        
        return [v_lower, v_upper, omega_lower, omega_upper]
        
    def _calc_control_and_trajectory(self, x, y, yaw, v, omega, dw, gx, gy):
        """Calculate control commands and trajectory."""
        x_init = x
        y_init = y
        yaw_init = yaw
        v_init = v
        omega_init = omega
        
        best_cost = float('inf')
        best_u = [0.0, 0.0]
        best_trajectory = []
        
        # Evaluate more samples for smoother control (using reference implementation approach)
        for v in np.linspace(dw[0], dw[1], 11):  # Increased from 5 to 11 samples
            for omega in np.linspace(dw[2], dw[3], 11):  # Increased from 5 to 11 samples
                # Simulate trajectory
                trajectory = self._predict_trajectory(x_init, y_init, yaw_init, v_init, omega_init, v, omega)
                
                # Skip if trajectory is empty
                if not trajectory:
                    continue
                
                # Calculate cost
                speed_cost = self._calc_speed_cost(v)
                obstacle_cost = self._calc_obstacle_cost(trajectory)
                goal_cost = self._calc_goal_cost(trajectory, gx, gy)
                path_cost = self._calc_path_following_cost(trajectory)
                
                # Add a steering cost to penalize large steering angles - make this more significant
                steering_cost = (abs(omega) / self.max_yaw_rate) ** 2  # Square to make high values even more costly
                
                # Add a continuity cost to penalize large changes in steering direction
                continuity_cost = abs(omega - omega_init) / self.max_delta_yaw_rate
                
                # Skip if trajectory hits an obstacle
                if obstacle_cost >= float('inf'):
                    continue
                
                # Total cost - increase the weight of steering costs
                final_cost = (
                    self.to_goal_cost_gain * goal_cost +
                    self.speed_cost_gain * speed_cost +
                    self.obstacle_cost_gain * obstacle_cost +
                    self.path_following_gain * path_cost +
                    0.6 * steering_cost +  # Increased from 0.3 to 0.6
                    0.4 * continuity_cost  # Add continuity cost with weight 0.4
                )
                
                # Update best trajectory
                if final_cost < best_cost:
                    best_cost = final_cost
                    best_u = [v, omega]
                    best_trajectory = trajectory
        
        # If no valid trajectory found, ensure we still return something reasonable
        if not best_trajectory:
            # Try to create a simple trajectory with minimum speed and no steering
            v_safe = 0.5  # Safe low speed
            omega_safe = 0.0  # No turning
            best_trajectory = self._predict_trajectory(x_init, y_init, yaw_init, v_init, omega_init, v_safe, omega_safe)
            best_u = [v_safe, omega_safe]
            
        # Ensure we're not returning zero velocity unless necessary
        if abs(best_u[0]) < 0.1 and not self.lane_obstacle_positions:
            best_u[0] = 0.5  # Small positive velocity if no obstacles
            
        # Limit maximum steering angle to prevent erratic behavior - make this limit stronger
        if abs(best_u[1]) > 0.5:  # Changed from 0.7 to 0.5
            best_u[1] = 0.5 * (1.0 if best_u[1] > 0 else -1.0)
            
        return best_u[0], best_u[1], best_trajectory
        
    def _predict_trajectory(self, x_init, y_init, yaw_init, v_init, omega_init, v, omega):
        """Predict trajectory with constant velocity and angular velocity."""
        trajectory = []
        time = 0.0
        x = x_init
        y = y_init
        yaw = yaw_init
        v_current = v_init
        omega_current = omega_init
        
        # Simple simulation with more gradual acceleration (matching reference implementation)
        while time <= self.predict_time:
            # Update velocity with more gradual acceleration
            if abs(v - v_current) > 0.1:
                v_sign = 1.0 if v > v_current else -1.0
                v_current += v_sign * min(self.max_accel * self.dt, abs(v - v_current))
            else:
                v_current = v
            
            # Update angular velocity with more gradual changes
            if abs(omega - omega_current) > 0.1:
                omega_sign = 1.0 if omega > omega_current else -1.0
                omega_current += omega_sign * min(self.max_delta_yaw_rate * self.dt, abs(omega - omega_current))
            else:
                omega_current = omega
            
            # Update state
            x += v_current * math.cos(yaw) * self.dt
            y += v_current * math.sin(yaw) * self.dt
            yaw += omega_current * self.dt
            
            # Normalize yaw angle
            yaw = self._normalize_angle(yaw)
            
            # Add state to trajectory
            trajectory.append([x, y, yaw, v_current])
            
            time += self.dt
        
        return trajectory
        
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
        
    def _calc_speed_cost(self, v):
        """Calculate speed cost using the reference implementation approach."""
        # For forward motion, prefer speeds closer to max_speed
        if v < 0.5:
            return float('inf')  # Strongly penalize low or negative speeds
            
        # Use a cost function that heavily favors higher speeds
        # Lower cost means better (preferred) velocity
        # We want to strongly prefer higher velocities
        cost = 1.0 - (v / self.max_speed)
        
        # Apply exponential weighting to make higher speeds much more preferable
        return cost * cost  # Square the cost to make the preference for high speeds stronger
        
    def _calc_obstacle_cost(self, trajectory):
        """Calculate obstacle cost using the reference implementation approach."""
        if self.cost_map is None:
            return float('inf')
            
        min_dist = float('inf')
        
        for point in trajectory:
            x, y = point[0], point[1]
            
            # Convert from map coordinates to grid coordinates
            grid_x = int((x - self.map_origin_x) / self.map_resolution)
            grid_y = int((y - self.map_origin_y) / self.map_resolution)
            
            # Check if point is within map bounds
            if 0 <= grid_x < self.cost_map.shape[1] and 0 <= grid_y < self.cost_map.shape[0]:
                # Get cost at this point
                cost = self.cost_map[grid_y, grid_x]
                
                # If cost is high (obstacle), return infinite cost with a clearer threshold
                if cost > self.obstacle_threshold:
                    return float('inf')
                
                # Track minimum distance to obstacles
                dist_to_obstacle = 1.0 - cost  # Higher cost means closer to obstacle
                min_dist = min(min_dist, dist_to_obstacle)
                
                # Check surrounding pixels for obstacles (wider search area)
                for dx in range(-2, 3):  # -2 to +2 (5x5 window)
                    for dy in range(-2, 3):
                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < self.cost_map.shape[1] and 0 <= ny < self.cost_map.shape[0]:
                            surr_cost = self.cost_map[ny, nx]
                            if surr_cost > self.obstacle_threshold:
                                # Calculate distance to this cell
                                cell_dist = math.hypot(dx, dy) * self.map_resolution
                                if cell_dist < self.safe_distance:
                                    return float('inf')
        
        # If trajectory is clear of obstacles, return cost based on minimum distance
        if min_dist == float('inf'):
            return 0.0  # No obstacles nearby
        
        # Return cost that's inversely proportional to distance - closer obstacles = higher cost
        return 1.0 / (min_dist + 1e-6)  # Add small epsilon to prevent division by zero
        
    def _calc_goal_cost(self, trajectory, gx, gy):
        """Calculate goal cost using the reference implementation approach."""
        # Use the last point in the trajectory
        if not trajectory:
            return float('inf')
            
        # Calculate Euclidean distance to goal
        dx = trajectory[-1][0] - gx
        dy = trajectory[-1][1] - gy
        dist = math.hypot(dx, dy)
        
        return dist
        
    def _calc_path_following_cost(self, trajectory):
        """Calculate path following cost using the reference implementation approach."""
        if not self.waypoints or not trajectory:
            return 0.0
            
        # Calculate cost based on distance to the reference path
        total_dist = 0.0
        
        # Get vehicle position from first trajectory point
        vx, vy = trajectory[0][0], trajectory[0][1]
        
        # Find closest waypoint to vehicle
        closest_wp_idx = 0
        min_dist = float('inf')
        for i, waypoint in enumerate(self.waypoints):
            dist = math.hypot(waypoint['x'] - vx, waypoint['y'] - vy)
            if dist < min_dist:
                min_dist = dist
                closest_wp_idx = i
        
        # Look ahead by a few waypoints for better path following
        lookahead_idx = min(closest_wp_idx + 5, len(self.waypoints) - 1)
        
        # Calculate cost for each trajectory point
        for i, point in enumerate(trajectory):
            x, y = point[0], point[1]
            
            # Progressive waypoint targeting - look further ahead for later points in trajectory
            target_wp_idx = min(closest_wp_idx + i//2, len(self.waypoints) - 1)
            
            # Find closest waypoint within a window around the target index
            window_size = 5  # Look at 5 waypoints before and after
            start_idx = max(0, target_wp_idx - window_size)
            end_idx = min(len(self.waypoints) - 1, target_wp_idx + window_size)
            
            # Find closest waypoint in this window
            min_wp_dist = float('inf')
            for j in range(start_idx, end_idx + 1):
                wp = self.waypoints[j]
                dist = math.hypot(wp['x'] - x, wp['y'] - y)
                min_wp_dist = min(min_wp_dist, dist)
            
            # Add to cost - weight later points more heavily
            # This encourages the trajectory to converge to the path over time
            point_weight = 1.0 + i * 0.1  # Increase weight for later points
            total_dist += min_wp_dist * point_weight
        
        # Average cost
        avg_cost = total_dist / len(trajectory)
        
        # Apply a non-linear penalty for large deviations
        # This makes the planner strongly avoid trajectories that deviate significantly
        if avg_cost > 2.0:  # If average deviation is more than 2 meters
            avg_cost = avg_cost * 1.5  # Apply a stronger penalty
        
        return avg_cost
    
    def find_safe_point_around(self, robot_pos, obstacles, search_radius=15, cost_thresh=0.2):
        """Find a safe point around the robot when obstacles are detected"""
        if self.cost_map is None:
            return None
        
        # Convert from map coordinates to grid coordinates
        grid_x = int((robot_pos[0] - self.map_origin_x) / self.map_resolution)
        grid_y = int((robot_pos[1] - self.map_origin_y) / self.map_resolution)
        
        h, w = self.cost_map.shape
        safe_points = []
        
        # Increase search radius for better path planning
        search_radius = 20  # Increased from 15 to 20
        
        for dx in range(-search_radius, search_radius+1):
            for dy in range(-search_radius, search_radius+1):
                nx, ny = grid_x + dx, grid_y + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if self.cost_map[ny, nx] < cost_thresh:
                        # Check if point is not too close to obstacles
                        is_near_obstacle = False
                        for obs in obstacles:
                            # Extract x,y coordinates from obstacle tuple
                            ox, oy = obs[0], obs[1]
                            
                            # Convert obstacle to grid coordinates
                            obs_grid_x = int((ox - self.map_origin_x) / self.map_resolution)
                            obs_grid_y = int((oy - self.map_origin_y) / self.map_resolution)
                            
                            if np.hypot(nx - obs_grid_x, ny - obs_grid_y) < 5:  # Increased safety distance from 3 to 5
                                is_near_obstacle = True
                                break
                        
                        if not is_near_obstacle:
                            # Convert back to map coordinates
                            safe_x = nx * self.map_resolution + self.map_origin_x
                            safe_y = ny * self.map_resolution + self.map_origin_y
                            safe_points.append((safe_x, safe_y))
        
        # Return the closest safe point to the robot's position
        if safe_points:
            safe_points.sort(key=lambda p: np.hypot(p[0] - robot_pos[0], p[1] - robot_pos[1]))
            return safe_points[0]
        
        return None
    
    def publish_parking_spots(self):
        """Publish visualization markers for detected parking spots"""
        if not self.parking_spots:
            return
        
        marker_array = MarkerArray()
        
        for i, spot in enumerate(self.parking_spots):
            # Create marker for parking spot
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "parking_spots"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = spot.x
            marker.pose.position.y = spot.y
            marker.pose.position.z = 0.1  # Slightly above ground
            
            # Set orientation from angle
            q = quaternion_from_euler(0, 0, spot.angle)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            # Set dimensions
            marker.scale.x = spot.length
            marker.scale.y = spot.width
            marker.scale.z = 0.1
            
            # Set color (blue for safe, yellow for unsafe)
            marker.color.r = 0.0 if spot.is_safe else 1.0
            marker.color.g = 0.7 if spot.is_safe else 1.0
            marker.color.b = 1.0 if spot.is_safe else 0.0
            marker.color.a = 0.7  # Semi-transparent
            
            marker.lifetime = Duration(sec=int(self.parking_visualization_lifetime), 
                                      nanosec=int((self.parking_visualization_lifetime % 1) * 1e9))
            
            marker_array.markers.append(marker)
            
            # Add text marker with dimensions
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "parking_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = spot.x
            text_marker.pose.position.y = spot.y
            text_marker.pose.position.z = 0.5  # Above the spot
            text_marker.text = f"Parking: {spot.width:.1f}x{spot.length:.1f}m"
            text_marker.scale.z = 0.5  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = Duration(sec=int(self.parking_visualization_lifetime), 
                                           nanosec=int((self.parking_visualization_lifetime % 1) * 1e9))
            
            marker_array.markers.append(text_marker)
            
            # Add arrow marker to show orientation
            arrow_marker = Marker()
            arrow_marker.header = marker.header
            arrow_marker.ns = "parking_arrows"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose = marker.pose
            arrow_marker.scale.x = spot.length * 0.8  # Arrow length
            arrow_marker.scale.y = 0.2  # Arrow width
            arrow_marker.scale.z = 0.2  # Arrow height
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.0
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 0.8
            arrow_marker.lifetime = Duration(sec=int(self.parking_visualization_lifetime), 
                                            nanosec=int((self.parking_visualization_lifetime % 1) * 1e9))
            
            marker_array.markers.append(arrow_marker)
        
        # Publish markers
        self.parking_spots_publisher.publish(marker_array)
    
    def generate_curved_path(self, start_point, end_point, obstacles, num_points=10):
        """Generate a curved path to navigate around obstacles"""
        if not obstacles:
            # No obstacles, return a straight line
            return [start_point, end_point]
            
        # Find the midpoint between start and end
        mid_x = (start_point[0] + end_point[0]) / 2
        mid_y = (start_point[1] + end_point[1]) / 2
        
        # Find the nearest obstacle to the midpoint
        nearest_obstacle = None
        min_dist = float('inf')
        for obs in obstacles:
            # Extract just the x,y coordinates from the obstacle tuple
            ox, oy = obs[0], obs[1]
            dist = np.hypot(ox - mid_x, oy - mid_y)
            if dist < min_dist:
                min_dist = dist
                nearest_obstacle = (ox, oy)
        
        if nearest_obstacle is None:
            # No obstacles, return a straight line
            return [start_point, end_point]
        
        # If the nearest obstacle is too far, return a straight line
        # Increased threshold from 3.0 to 5.0 to be less sensitive to distant obstacles
        if min_dist > 5.0:  
            return [start_point, end_point]
            
        # Calculate the vector from start to end
        vec_x = end_point[0] - start_point[0]
        vec_y = end_point[1] - start_point[1]
        vec_len = np.hypot(vec_x, vec_y)
        
        if vec_len < 0.001:  # Avoid division by zero
            return [start_point]
        
        # Normalize the vector
        vec_x /= vec_len
        vec_y /= vec_len
        
        # Calculate the perpendicular vector (rotate 90 degrees)
        perp_x = -vec_y
        perp_y = vec_x
        
        # Calculate the vector from midpoint to obstacle
        obs_vec_x = nearest_obstacle[0] - mid_x
        obs_vec_y = nearest_obstacle[1] - mid_y
        
        # Determine which side to curve around (opposite to the obstacle)
        dot_product = obs_vec_x * perp_x + obs_vec_y * perp_y
        
        # Choose direction based on dot product (opposite to the obstacle)
        if dot_product > 0:
            perp_x = -perp_x
            perp_y = -perp_y
        
        # Calculate curve offset based on obstacle distance
        # The closer the obstacle, the larger the curve
        # Reduced the curve scale to make curves less aggressive
        curve_scale = min(3.0, 5.0 / (min_dist + 0.1))
        
        # Generate curved path points using a quadratic Bezier curve
        curved_path = []
        for t in np.linspace(0, 1, num_points):
            # Quadratic Bezier curve
            # B(t) = (1-t)^2 * P0 + 2(1-t)t * P1 + t^2 * P2
            # Where P1 is the control point offset from the midpoint
            control_x = mid_x + perp_x * curve_scale
            control_y = mid_y + perp_y * curve_scale
            
            x = (1-t)**2 * start_point[0] + 2*(1-t)*t * control_x + t**2 * end_point[0]
            y = (1-t)**2 * start_point[1] + 2*(1-t)*t * control_y + t**2 * end_point[1]
            
            curved_path.append((x, y))
        
        return curved_path
    
    def planning_callback(self):
        """Main planning callback."""
        # Check if we have received map and waypoints
        if self.binary_map is None:
            self.get_logger().debug("Waiting for binary map...")
            return
        
        if not self.waypoints:
            self.get_logger().debug("Waiting for waypoints...")
            return
        
        # Ensure we are in a valid state
        if self.planner_state == "INITIALIZING":
            self.planner_state = "READY"
            self.get_logger().info("Planner state changed from INITIALIZING to READY")
        
        # Ensure current velocity is not negative or too low
        if self.current_v < 1.0:
            self.current_v = 2.0  # Set to a good positive value for reliable movement
            self.get_logger().info(f"Boosted velocity to {self.current_v} m/s")
        
        # Reset current omega if it's too high to prevent steering issues
        if abs(self.current_omega) > 0.5:
            self.get_logger().warn(f"Resetting high angular velocity: {self.current_omega:.2f} -> 0.0")
            self.current_omega = 0.0
        
        # Get current vehicle pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.vehicle_frame_id,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform: {str(e)}")
            return
        
        # Extract vehicle pose from transform
        vehicle_pose = Pose()
        vehicle_pose.position.x = transform.transform.translation.x
        vehicle_pose.position.y = transform.transform.translation.y
        vehicle_pose.position.z = transform.transform.translation.z
        vehicle_pose.orientation = transform.transform.rotation
        
        # Update vehicle state
        self.update_vehicle_state(vehicle_pose)
        
        # Find closest waypoint
        closest_idx = self.find_closest_waypoint(vehicle_pose)
        if closest_idx is None:
            self.get_logger().warn("No valid waypoint found")
            return
            
        # Get target waypoint with lookahead
        target_idx, target_waypoint = self.get_target_waypoint(closest_idx, vehicle_pose)
        if target_idx is None:
            self.get_logger().warn("No valid target waypoint found")
            return
            
        # Log the target waypoint for debugging
        self.get_logger().debug(f"Target waypoint: idx={target_idx}, pos=({target_waypoint['x']:.2f}, {target_waypoint['y']:.2f})")
        
        # Visualize the target waypoint
        self.visualize_target_waypoint(target_waypoint)
        
        # Generate path using DWA
        path_x, path_y, path_yaw, path_v, best_u = self.dwa_control(vehicle_pose, target_waypoint)
        
        # Check if path is valid
        if not path_x or len(path_x) < 2:
            self.get_logger().warn("Failed to generate valid path")
            return
        
        # Log the velocity and angular velocity
        self.get_logger().debug(f"DWA generated v={best_u[0]:.2f}, omega={best_u[1]:.2f}")
            
        # Create path points for obstacle detection
        path_points = [(path_x[i], path_y[i]) for i in range(len(path_x))]
        
        # Check for obstacles on path - make this more sensitive again
        has_lane_obstacles = self.find_obstacles_on_path(path_points, self.binary_map, vehicle_pose)
        
        # Use a more balanced obstacle detection threshold
        min_obstacle_count_for_stop = self.min_obstacle_count  # Use the configured value
        
        if has_lane_obstacles:
            self.get_logger().info(f"Detected {len(self.lane_obstacle_positions)} obstacle points")
            # Visualize obstacles for debugging
            self.visualize_lane_obstacles()
        
        # Make obstacle detection appropriately sensitive
        stop_for_obstacles = has_lane_obstacles
        
        # Run enhanced DWA control with obstacle awareness
        cmd_vel = self.enhanced_dwa_control(best_u, vehicle_pose, stop_for_obstacles)
        
        # Ensure we're always moving forward if there are no obstacles
        if not stop_for_obstacles and cmd_vel.linear.x < 1.0:
            self.get_logger().warn(f"Forcing forward motion: {cmd_vel.linear.x:.2f} -> 2.0 m/s")
            cmd_vel.linear.x = 2.0
        
        # Publish command velocity
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Create and publish path message with more points for smoother MPC following
        path_msg = self.create_enhanced_path_message(path_x, path_y, path_yaw, path_v, vehicle_pose, target_idx)
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        
        # Publish obstacle detection status - be more responsive to obstacles
        obstacle_msg = Bool()
        obstacle_msg.data = stop_for_obstacles
        self.obstacle_detected_publisher.publish(obstacle_msg)
        
        # Periodically log the planner state and velocity
        if (self.get_clock().now() - self.last_state_log).nanoseconds / 1e9 > 2.0:  # Log every 2 seconds
            planning_rate = 1.0/(self.get_clock().now() - self.last_plan_time).nanoseconds / 1e9
            self.get_logger().info(f"Planner state: {self.planner_state}, v={path_v[0]:.2f}, omega={path_yaw[0]:.2f}, planning rate: {planning_rate:.2f} Hz")
            self.last_state_log = self.get_clock().now()
            
        # Update the last plan time
        self.last_plan_time = self.get_clock().now()
        
    def create_enhanced_path_message(self, path_x, path_y, path_yaw, path_v, vehicle_pose, target_idx):
        """Create an enhanced path message with more points for smoother MPC following."""
        # Create path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id
        
        # Start with the current vehicle position
        start_pose = PoseStamped()
        start_pose.header = path_msg.header
        start_pose.pose = vehicle_pose
        path_msg.poses.append(start_pose)
        
        # Add DWA path points
        for i in range(len(path_x)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = path_x[i]
            pose.pose.position.y = path_y[i]
            pose.pose.position.z = 0.0
            
            # Set orientation based on path yaw
            q = quaternion_from_euler(0, 0, path_yaw[i])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
        
        # Add additional waypoints beyond the DWA path for better MPC planning
        # This helps the MPC controller see further ahead
        if target_idx is not None and self.waypoints:
            # Add a few more waypoints beyond the target
            num_extra_waypoints = 5  # Add 5 more waypoints
            for i in range(target_idx, min(target_idx + num_extra_waypoints, len(self.waypoints))):
                wp = self.waypoints[i]
                
                # Calculate orientation based on the direction to the next waypoint
                next_idx = min(i + 1, len(self.waypoints) - 1)
                if next_idx > i:
                    next_wp = self.waypoints[next_idx]
                    dx = next_wp['x'] - wp['x']
                    dy = next_wp['y'] - wp['y']
                    yaw = math.atan2(dy, dx)
                else:
                    # Use the last known yaw if this is the final waypoint
                    yaw = path_yaw[-1] if path_yaw else 0.0
                
                # Create pose
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wp['x']
                pose.pose.position.y = wp['y']
                pose.pose.position.z = 0.0
                
                # Set orientation
                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                
                path_msg.poses.append(pose)
        
        # Ensure we have enough points for MPC
        if len(path_msg.poses) < 5:
            # If we don't have enough points, duplicate the last point a few times
            last_pose = path_msg.poses[-1] if path_msg.poses else start_pose
            for _ in range(5 - len(path_msg.poses)):
                path_msg.poses.append(last_pose)
        
        return path_msg
    
    def enhanced_dwa_control(self, best_u, vehicle_pose, stop_for_obstacles):
        """Apply enhanced DWA control with obstacle awareness."""
        cmd_vel = Twist()
        
        # Extract vehicle position and orientation
        x = vehicle_pose.position.x
        y = vehicle_pose.position.y
        quat = vehicle_pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Extract best velocity commands
        v_opt, omega_opt = best_u
        
        # Check if we should stop for obstacles
        if stop_for_obstacles:
            # Find the closest obstacle distance
            closest_obstacle = float('inf')
            if self.lane_obstacle_positions:
                for obs_x, obs_y in self.lane_obstacle_positions:
                    dist = math.sqrt((obs_x - x) ** 2 + (obs_y - y) ** 2)
                    closest_obstacle = min(closest_obstacle, dist)
            
            # Apply progressive braking based on obstacle distance
            if closest_obstacle < self.emergency_brake_distance * 0.5:
                # Very close obstacle - emergency stop
                v_opt = 0.0  # Full stop
                omega_opt = 0.0  # Stop turning
                self.get_logger().warn(f"Emergency stop: Obstacle at {closest_obstacle:.2f}m")
            elif closest_obstacle < self.emergency_brake_distance:
                # Obstacle in braking zone - slow down proportionally
                slow_factor = closest_obstacle / self.emergency_brake_distance
                v_opt = max(0.0, v_opt * slow_factor)
                self.get_logger().info(f"Slowing down: Obstacle at {closest_obstacle:.2f}m, speed reduced to {v_opt:.2f} m/s")
            else:
                # Obstacle detected but not in immediate danger zone - reduce speed slightly
                v_opt = max(0.0, v_opt * 0.7)
                self.get_logger().info(f"Cautious speed: Obstacle at {closest_obstacle:.2f}m, speed reduced to {v_opt:.2f} m/s")
            
            # Try to steer away from obstacles if we're still moving
            if v_opt > 0.1 and self.lane_obstacle_positions:
                avoidance_omega = self._calculate_avoidance_steering(vehicle_pose)
                if avoidance_omega is not None:
                    # Blend the avoidance steering with the optimal steering
                    omega_opt = 0.7 * avoidance_omega + 0.3 * omega_opt
                    self.get_logger().info(f"Obstacle avoidance steering: {omega_opt:.2f} rad/s")
            
            # Publish obstacle detected message
            obstacle_msg = Bool()
            obstacle_msg.data = True
            self.obstacle_detected_publisher.publish(obstacle_msg)
        else:
            # No obstacles, use a reasonable forward velocity
            # Force a minimum forward velocity to get the vehicle moving
            v_opt = max(2.0, v_opt)  # Always at least 2.0 m/s
            
            # Publish normal message
            obstacle_msg = Bool()
            obstacle_msg.data = False
            self.obstacle_detected_publisher.publish(obstacle_msg)
            
            self.get_logger().info(f"Moving forward at velocity: {v_opt:.2f} m/s")
        
        # Apply steering smoothing for more natural motion
        # Use stronger smoothing for steering
        smoothing_factor = 0.5  # Changed from 0.7 to 0.5 (50% new, 50% previous)
        smoothed_omega = smoothing_factor * omega_opt + (1.0 - smoothing_factor) * self.current_omega
        
        # Limit steering angle at higher speeds for stability
        speed_factor = min(1.0, abs(v_opt) / self.max_speed)
        max_omega_at_speed = self.max_yaw_rate * (1.0 - 0.7 * speed_factor)  # Reduce max steering at high speeds
        smoothed_omega = max(min(smoothed_omega, max_omega_at_speed), -max_omega_at_speed)
        
        # Set the command velocity
        cmd_vel.linear.x = v_opt
        cmd_vel.angular.z = smoothed_omega
        
        # Update current velocity and angular velocity
        self.current_v = v_opt
        self.current_omega = smoothed_omega
        
        return cmd_vel
    
    def _calculate_avoidance_steering(self, vehicle_pose):
        """Calculate steering command for obstacle avoidance."""
        if not self.lane_obstacle_positions or len(self.lane_obstacle_positions) == 0:
            return 0.0
        
        # Get vehicle position and orientation
        vehicle_x, vehicle_y = vehicle_pose.position.x, vehicle_pose.position.y
        quat = vehicle_pose.orientation
        _, _, vehicle_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Calculate average obstacle position
        avg_x = sum(pos[0] for pos in self.lane_obstacle_positions) / len(self.lane_obstacle_positions)
        avg_y = sum(pos[1] for pos in self.lane_obstacle_positions) / len(self.lane_obstacle_positions)
        
        # Calculate vector from vehicle to average obstacle
        dx = avg_x - vehicle_x
        dy = avg_y - vehicle_y
        
        # Calculate distance to obstacle
        distance = math.sqrt(dx*dx + dy*dy)
        
        # If obstacle is too far, don't steer
        if distance > self.obstacle_detection_range:
            return 0.0
        
        # Calculate angle to obstacle in vehicle frame
        obstacle_angle = math.atan2(dy, dx) - vehicle_yaw
        
        # Normalize angle to [-pi, pi]
        while obstacle_angle > math.pi:
            obstacle_angle -= 2.0 * math.pi
        while obstacle_angle < -math.pi:
            obstacle_angle += 2.0 * math.pi
        
        # Determine steering direction (away from obstacle)
        # If obstacle is on the right, steer left (negative angle)
        # If obstacle is on the left, steer right (positive angle)
        steering_direction = -1.0 if obstacle_angle > 0.0 else 1.0
        
        # Calculate steering magnitude based on obstacle angle and distance
        # Closer obstacles and more central obstacles require stronger steering
        steering_magnitude = abs(math.sin(obstacle_angle)) * (1.0 - min(1.0, distance / self.obstacle_detection_range))
        
        # Apply a gain to the steering command
        steering_gain = 0.8
        steering_cmd = steering_direction * steering_magnitude * steering_gain
        
        # Limit maximum steering
        max_steering = 0.8
        if abs(steering_cmd) > max_steering:
            steering_cmd = max_steering * (1.0 if steering_cmd > 0.0 else -1.0)
        
        return steering_cmd
    
    def _find_safe_path_around_obstacles(self, vehicle_pose):
        """Find a safe path around obstacles when stopped."""
        if not self.lane_obstacle_positions or len(self.lane_obstacle_positions) == 0:
            return None
            
        # Get vehicle position
        vehicle_x, vehicle_y = vehicle_pose.position.x, vehicle_pose.position.y
        quat = vehicle_pose.orientation
        _, _, vehicle_yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Try different steering angles to find a safe path
        best_steering = None
        best_clearance = 0.0
        
        # Test a range of steering angles
        for steering_angle in np.linspace(-0.8, 0.8, 16):  # Test 16 different angles
            # Predict path with this steering angle
            test_path = self._predict_path(vehicle_pose, 0.5, steering_angle, 20)  # 20 points ahead
            
            # Check if path is clear of obstacles
            clearance = self._check_path_clearance(test_path)
            
            if clearance > best_clearance:
                best_clearance = clearance
                best_steering = steering_angle
                
        # Only return a steering command if we found a path with sufficient clearance
        if best_clearance > 0.5:  # Threshold for considering a path safe
            return best_steering
            
            return None
        
    def _predict_path(self, vehicle_pose, speed, steering, num_points):
        """Predict a path given speed and steering."""
        path = []
        
        # Get vehicle position and orientation
        x = vehicle_pose.position.x
        y = vehicle_pose.position.y
        quat = vehicle_pose.orientation
        _, _, theta = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Simple kinematic model to predict path
        dt = 0.1  # time step
        for i in range(num_points):
            # Update position
            x += speed * math.cos(theta) * dt
            y += speed * math.sin(theta) * dt
            theta += steering * dt
            
            path.append((x, y))
            
        return path
        
    def _check_path_clearance(self, path):
        """Check how clear a path is from obstacles."""
        if not self.lane_obstacle_positions or len(self.lane_obstacle_positions) == 0:
            return 1.0  # Fully clear
            
        min_distance = float('inf')
        
        # Check distance from each path point to each obstacle
        for path_point in path:
            for obstacle_point in self.lane_obstacle_positions:
                dist = math.hypot(path_point[0] - obstacle_point[0], 
                                 path_point[1] - obstacle_point[1])
                min_distance = min(min_distance, dist)
                
        # Normalize clearance score
        if min_distance == float('inf'):
            return 1.0  # Fully clear
        elif min_distance < 0.5:
            return 0.0  # Too close to obstacle
        else:
            return min(1.0, min_distance / 3.0)  # Normalize to [0, 1]
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        # Stop publishing commands
        try:
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
        except:
            pass
        
        super().destroy_node()
    
    def visualize_lane_obstacles(self, obstacles, lane_width):
        """Visualize lane obstacles for debugging"""
        marker_array = MarkerArray()
        
        for i, (ox, oy, wp_idx) in enumerate(obstacles):
            # Create marker for obstacle
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "lane_obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = ox
            marker.pose.position.y = oy
            marker.pose.position.z = 0.5  # Half height above ground
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set dimensions
            marker.scale.x = 0.8  # Smaller diameter for obstacles (not as wide as lane)
            marker.scale.y = 0.8
            marker.scale.z = 1.5  # Taller height for better visibility
            
            # Set color (bright red for obstacles, distinct from parking spots)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7  # More opaque
            
            marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))  # 0.5 second lifetime
            
            marker_array.markers.append(marker)
            
            # Add text label for obstacles
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "obstacle_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = ox
            text_marker.pose.position.y = oy
            text_marker.pose.position.z = 1.5  # Above the obstacle
            text_marker.text = "OBSTACLE"
            text_marker.scale.z = 0.5  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
            
            marker_array.markers.append(text_marker)
        
        # Publish the markers
        self.marker_publisher.publish(marker_array)
    
    def visualize_lane_obstacles(self):
        """Visualize obstacles detected in the vehicle's lane."""
        if not self.lane_obstacle_positions or len(self.lane_obstacle_positions) == 0:
            return
            
        # Create a marker array for visualization
        marker_array = MarkerArray()
        
        # Create markers for each obstacle
        for i, (x, y) in enumerate(self.lane_obstacle_positions):
            # Create sphere marker
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.map_frame_id
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "lane_obstacles"
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            # Set position
            sphere_marker.pose.position.x = x
            sphere_marker.pose.position.y = y
            sphere_marker.pose.position.z = 0.5  # Slightly above ground
            
            # Set orientation (identity quaternion)
            sphere_marker.pose.orientation.w = 1.0
            
            # Set scale
            sphere_marker.scale.x = 0.5
            sphere_marker.scale.y = 0.5
            sphere_marker.scale.z = 0.5
            
            # Set color (bright red for obstacles)
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            
            # Set lifetime - fix Duration usage
            lifetime_sec = 0.5
            sphere_marker.lifetime = Duration(sec=int(lifetime_sec), 
                                             nanosec=int((lifetime_sec % 1) * 1e9))
            
            marker_array.markers.append(sphere_marker)
            
            # Add text marker
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "lane_obstacles_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set position (slightly above the sphere)
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 1.0
            
            # Set text
            text_marker.text = "OBSTACLE"
            
            # Set scale
            text_marker.scale.z = 0.5  # Text height
            
            # Set color (white text)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.8
            
            # Set lifetime - fix Duration usage
            text_marker.lifetime = Duration(sec=int(lifetime_sec), 
                                           nanosec=int((lifetime_sec % 1) * 1e9))
                            
            marker_array.markers.append(text_marker)
        
        # Publish marker array
        self.parking_spots_publisher.publish(marker_array)
    
    def visualize_target_waypoint(self, target_waypoint):
        """Visualize the target waypoint for debugging."""
        if not target_waypoint:
            return
            
        # Create a marker array for visualization
        marker_array = MarkerArray()
        
        # Create sphere marker for target waypoint
        sphere_marker = Marker()
        sphere_marker.header.frame_id = self.map_frame_id
        sphere_marker.header.stamp = self.get_clock().now().to_msg()
        sphere_marker.ns = "target_waypoint"
        sphere_marker.id = 0
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        
        # Set position
        sphere_marker.pose.position.x = target_waypoint['x']
        sphere_marker.pose.position.y = target_waypoint['y']
        sphere_marker.pose.position.z = 0.5  # Slightly above ground
        
        # Set orientation (identity quaternion)
        sphere_marker.pose.orientation.w = 1.0
        
        # Set scale
        sphere_marker.scale.x = 1.0
        sphere_marker.scale.y = 1.0
        sphere_marker.scale.z = 1.0
        
        # Set color (green for target waypoint)
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 1.0
        sphere_marker.color.b = 0.0
        sphere_marker.color.a = 0.8
        
        # Set lifetime - fix Duration usage
        lifetime_sec = 0.2
        sphere_marker.lifetime = Duration(sec=int(lifetime_sec), 
                                         nanosec=int((lifetime_sec % 1) * 1e9))
        
        marker_array.markers.append(sphere_marker)
        
        # Add text marker
        text_marker = Marker()
        text_marker.header.frame_id = self.map_frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "target_waypoint_text"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Set position (slightly above the sphere)
        text_marker.pose.position.x = target_waypoint['x']
        text_marker.pose.position.y = target_waypoint['y']
        text_marker.pose.position.z = 1.5
        
        # Set text
        text_marker.text = "TARGET"
        
        # Set scale
        text_marker.scale.z = 0.5  # Text height
        
        # Set color (white text)
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 0.8
        
        # Set lifetime - fix Duration usage
        text_marker.lifetime = Duration(sec=int(lifetime_sec), 
                                       nanosec=int((lifetime_sec % 1) * 1e9))
                        
        marker_array.markers.append(text_marker)
        
        # Publish marker array
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        enhanced_dwa_planner = EnhancedDWAPlanner()
        rclpy.spin(enhanced_dwa_planner)
    except Exception as e:
        print(f"Error in Enhanced DWA planner node: {e}")
    finally:
        if 'enhanced_dwa_planner' in locals():
            enhanced_dwa_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
