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
from std_msgs.msg import Header, ColorRGBA, Int32MultiArray
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration

# Import tf_transformations properly
try:
    # Try to import from tf_transformations package (ROS2)
    from tf_transformations import quaternion_from_euler
except ImportError:
    try:
        # Try to import from transforms3d as a fallback
        from transforms3d.euler import euler2quat
        def quaternion_from_euler(roll, pitch, yaw):
            return euler2quat(roll, pitch, yaw)
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
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').get_parameter_value().double_value
        self.max_accel = self.get_parameter('max_accel').get_parameter_value().double_value
        self.max_delta_yaw_rate = self.get_parameter('max_delta_yaw_rate').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.predict_time = self.get_parameter('predict_time').get_parameter_value().double_value
        self.to_goal_cost_gain = self.get_parameter('to_goal_cost_gain').get_parameter_value().double_value
        self.speed_cost_gain = self.get_parameter('speed_cost_gain').get_parameter_value().double_value
        self.obstacle_cost_gain = self.get_parameter('obstacle_cost_gain').get_parameter_value().double_value
        self.path_following_gain = self.get_parameter('path_following_gain').get_parameter_value().double_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().integer_value
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value
        self.default_lane_width = self.get_parameter('default_lane_width').get_parameter_value().double_value
        self.lane_width_factor = self.get_parameter('lane_width_factor').get_parameter_value().double_value
        self.min_parking_width = self.get_parameter('min_parking_width').get_parameter_value().double_value
        self.min_parking_length = self.get_parameter('min_parking_length').get_parameter_value().double_value
        self.enable_parking_detection = self.get_parameter('enable_parking_detection').get_parameter_value().bool_value
        self.extended_predict_time = self.get_parameter('extended_predict_time').get_parameter_value().double_value
        self.parking_gray_threshold = self.get_parameter('parking_gray_threshold').get_parameter_value().double_value
        self.parking_visualization_lifetime = self.get_parameter('parking_visualization_lifetime').get_parameter_value().double_value
        
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
        self.map_resolution = 0.5  # Default value, will be updated from map
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.waypoints = []
        self.lane_widths = []  # Store lane widths for each waypoint
        self.current_lane_width = self.default_lane_width
        self.parking_spots = []
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_v = 0.0
        self.current_omega = 0.0
        
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
                width = msg.info.width
                height = msg.info.height
                self.map_resolution = msg.info.resolution
                self.map_origin_x = msg.info.origin.position.x
                self.map_origin_y = msg.info.origin.position.y
                
                # Convert from 1D array to 2D numpy array
                self.cost_map = np.array(msg.data).reshape((height, width)) / 100.0
                self.has_map = True
                
                self.get_logger().debug(f"Received map {width}x{height} with resolution {self.map_resolution}")
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
    
    def update_vehicle_state(self):
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
    
    def find_obstacles_on_path(self, waypoints, threshold=0.5, window=3):
        """Find obstacles and any black areas near the planned path"""
        if self.cost_map is None:
            return []
        
        obstacles = []
        h, w = self.cost_map.shape
        
        # Increase the detection window for better black area detection
        window = 5  # Increased from 3 to 5
        
        # Increase the lookahead for obstacle detection
        lookahead_points = min(len(waypoints), 30)  # Look at the next 30 waypoints maximum
        
        # Lower threshold to detect any black areas (not just strong obstacles)
        # In binary maps, black areas typically have higher values (closer to 1.0)
        detection_threshold = 0.5  # Reduced from 0.8 to detect more black areas
        
        for wp_idx in range(min(len(waypoints), lookahead_points)):
            wp = waypoints[wp_idx]
            # Convert from map coordinates to grid coordinates
            grid_x = int((wp[0] - self.map_origin_x) / self.map_resolution)
            grid_y = int((wp[1] - self.map_origin_y) / self.map_resolution)
            
            # Skip if outside map bounds
            if not (0 <= grid_x < w and 0 <= grid_y < h):
                continue
                
            for dx in range(-window, window+1):
                for dy in range(-window, window+1):
                    nx, ny = grid_x + dx, grid_y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        # Detect any black area in the map (lower threshold)
                        if self.cost_map[ny, nx] > detection_threshold:
                            # Convert back to map coordinates
                            obs_x = nx * self.map_resolution + self.map_origin_x
                            obs_y = ny * self.map_resolution + self.map_origin_y
                            # Store black area with the waypoint index it's close to
                            obstacles.append((obs_x, obs_y, wp_idx))
        
        # Remove duplicates while preserving waypoint index
        unique_obstacles = []
        seen_coords = set()
        for ox, oy, idx in obstacles:
            coord = (round(ox, 1), round(oy, 1))  # Round to reduce duplicates due to floating point
            if coord not in seen_coords:
                seen_coords.add(coord)
                unique_obstacles.append((ox, oy, idx))
        
        return unique_obstacles
    
    def is_obstacle_in_lane(self, obstacle, waypoints, lane_width):
        """Check if an obstacle is within the lane defined by waypoints"""
        ox, oy, wp_idx = obstacle
        
        # Get the waypoint the obstacle is close to
        if wp_idx >= len(waypoints) or wp_idx < 0:
            return False
        
        wp = waypoints[wp_idx]
        
        # If we have previous and next waypoints, use them to define the lane direction
        if 0 <= wp_idx - 1 < len(waypoints) and wp_idx + 1 < len(waypoints):
            prev_wp = waypoints[wp_idx - 1]
            next_wp = waypoints[wp_idx + 1]
            
            # Calculate lane direction vector
            lane_dir_x = next_wp[0] - prev_wp[0]
            lane_dir_y = next_wp[1] - prev_wp[1]
            lane_dir_len = math.hypot(lane_dir_x, lane_dir_y)
            
            if lane_dir_len > 0.001:  # Avoid division by zero
                # Normalize lane direction
                lane_dir_x /= lane_dir_len
                lane_dir_y /= lane_dir_len
                
                # Calculate perpendicular vector to lane direction
                perp_x = -lane_dir_y
                perp_y = lane_dir_x
                
                # Calculate vector from waypoint to obstacle
                to_obs_x = ox - wp[0]
                to_obs_y = oy - wp[1]
                
                # Calculate distance along lane and perpendicular to lane
                along_dist = to_obs_x * lane_dir_x + to_obs_y * lane_dir_y
                perp_dist = abs(to_obs_x * perp_x + to_obs_y * perp_y)
                
                # Check if obstacle is within lane width
                half_lane = lane_width / 2.0
                return perp_dist <= half_lane
        
        # Fallback to simple distance check if we can't calculate lane direction
        dist = math.hypot(ox - wp[0], oy - wp[1])
        half_lane = lane_width / 2.0
        return dist <= half_lane
    
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
        """Main planning and control loop with enhanced obstacle avoidance"""
        if not self.has_map or not self.has_waypoints:
            self.get_logger().debug("Waiting for map and waypoints...")
            return
        
        # Update vehicle state
        if not self.update_vehicle_state():
            return
        
        try:
            # Get current state
            with self.state_lock:
                x = self.current_x
                y = self.current_y
                theta = self.current_theta
                v = self.current_v
                omega = self.current_omega
            
            # Get waypoints
            with self.waypoints_lock:
                waypoints = self.waypoints.copy()
                lane_width = self.current_lane_width
            
            # Get cost map
            with self.map_lock:
                if self.cost_map is None:
                    return
                cost_map = self.cost_map.copy()
                map_resolution = self.map_resolution
                map_origin_x = self.map_origin_x
                map_origin_y = self.map_origin_y
            
            # Adjust planning parameters based on lane width
            adjusted_lookahead = min(self.lookahead_distance, lane_width * self.lane_width_factor)
            
            # Convert waypoints to planner format
            original_path = []
            for wp in waypoints:
                original_path.append((wp['x'], wp['y']))
            
            # If no waypoints, stop the vehicle
            if not original_path:
                self.get_logger().warn("No waypoints available, stopping vehicle")
                self.publish_cmd_vel(0.0, 0.0)
                return
            
            # Check for black areas on the path (using lower threshold)
            all_obstacles = self.find_obstacles_on_path(original_path, threshold=0.5)
            
            # Filter obstacles to only those in the lane
            lane_obstacles = [obs for obs in all_obstacles if self.is_obstacle_in_lane(obs, original_path, lane_width)]
            
            # Only consider it dangerous if black areas are actually in the lane
            danger = len(lane_obstacles) > 0
            
            # Log the number of detected black areas
            if all_obstacles:
                self.get_logger().debug(f"Detected {len(all_obstacles)} black areas, {len(lane_obstacles)} in lane")
            
            # Create a modified path that curves around black areas only if they are in the lane
            modified_path = []
            
            # Current position as tuple
            current_pos = (x, y)
            
            # Get goal point (lookahead on path)
            goal = self._get_lookahead_point(x, y, original_path, adjusted_lookahead)
            if goal is None and original_path:
                goal = original_path[-1]
            
            if danger and goal and len(lane_obstacles) > 0:
                self.get_logger().info(f"Detected {len(lane_obstacles)} black areas in lane")
                
                # Generate a curved path around black areas
                curved_path = self.generate_curved_path(current_pos, goal, lane_obstacles, num_points=15)
                modified_path = curved_path
                
                # Add the remaining waypoints after the goal
                goal_idx = -1
                for i, wp in enumerate(original_path):
                    if wp == goal:
                        goal_idx = i
                        break
                
                if goal_idx >= 0 and goal_idx < len(original_path) - 1:
                    modified_path.extend(original_path[goal_idx+1:])
            else:
                # No black areas in lane, use original path but start from current position
                if len(original_path) > 0:
                    # Add current position as first point
                    modified_path = [current_pos]
                    
                    # Find closest point on original path
                    min_dist = float('inf')
                    closest_idx = 0
                    for i, wp in enumerate(original_path):
                        dist = math.hypot(wp[0] - x, wp[1] - y)
                        if dist < min_dist:
                            min_dist = dist
                            closest_idx = i
                    
                    # Add waypoints from closest point onwards
                    modified_path.extend(original_path[closest_idx:])
                else:
                    modified_path = original_path
            
            # Use the modified path for planning
            path = modified_path
            
            # Get goal point (lookahead on modified path)
            goal = self._get_lookahead_point(x, y, path, adjusted_lookahead)
            if goal is None and path:
                goal = path[-1]
            
            if goal is None:
                self.get_logger().warn("No goal point available")
                return
            
            # Calculate if we're at the end of the path
            at_end_of_path = False
            if len(original_path) > 0:
                dist_to_end = math.hypot(x - original_path[-1][0], y - original_path[-1][1])
                at_end_of_path = dist_to_end < 1.0  # Within 1 meter of the end
            
            # Check for obstacles directly in front of the vehicle (inspired by DWA_35_new5.py)
            front_danger = False
            front_distance = 5.0  # Look 5 meters ahead
            front_obstacles = []
            
            # Scan in front of the vehicle
            for d in np.linspace(0, front_distance, num=10):
                fx = x + d * math.cos(theta)
                fy = y + d * math.sin(theta)
                
                # Convert to grid coordinates
                grid_x = int((fx - map_origin_x) / map_resolution)
                grid_y = int((fy - map_origin_y) / map_resolution)
                
                # Check if in map bounds and is an obstacle
                if (0 <= grid_x < cost_map.shape[1] and 
                    0 <= grid_y < cost_map.shape[0] and
                    cost_map[grid_y, grid_x] > 0.5):
                    front_danger = True
                    front_obstacles.append((fx, fy))
                    break
            
            # Combine danger flags
            danger = danger or front_danger
            
            # Run enhanced DWA planner for control
            best_v, best_omega, predicted_trajectory = self.enhanced_dwa_control(
                x, y, theta, v, goal, path, cost_map, 
                map_resolution, map_origin_x, map_origin_y,
                danger=danger,
                at_end_of_path=at_end_of_path
            )
            
            # Update current velocity
            with self.state_lock:
                self.current_v = best_v
                self.current_omega = best_omega
            
            # Publish control command
            self.publish_cmd_vel(best_v, best_omega)
            
            # Publish planned path with predicted trajectory for visualization
            self.publish_path(predicted_trajectory, goal, danger, original_path, modified_path)
            
            # Log planning rate
            now = self.get_clock().now()
            dt = (now - self.last_plan_time).nanoseconds / 1e9
            self.last_plan_time = now
            self.get_logger().debug(f"Planning rate: {1.0/dt:.2f} Hz, v={best_v:.2f}, omega={best_omega:.2f}")
            
        except Exception as e:
            self.get_logger().error(f"Error in planning loop: {e}")
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def _predict_extended_trajectory(self, x, y, theta, v, omega, predict_time):
        """Predict an extended trajectory for visualization"""
        trajectory = []
        time = 0
        
        # Current state
        curr_x = x
        curr_y = y
        curr_theta = theta
        
        # Use smaller time step for smoother visualization
        dt = min(self.dt, 0.1)
        
        while time <= predict_time:
            # Update state
            curr_x += v * math.cos(curr_theta) * dt
            curr_y += v * math.sin(curr_theta) * dt
            curr_theta += omega * dt
            
            # Add to trajectory
            trajectory.append((curr_x, curr_y))
            time += dt
        
        return trajectory
    
    def enhanced_dwa_control(self, x, y, theta, v, goal, path, cost_map, 
                            map_resolution, map_origin_x, map_origin_y, 
                            danger=False, at_end_of_path=False):
        """Enhanced Dynamic Window Approach control with improved obstacle handling"""
        # Adjust parameters based on danger level
        if danger:
            # More conservative parameters when obstacles are detected
            v_max = min(self.max_speed * 0.7, v + self.max_accel * self.dt)  # Reduce max speed
            obstacle_gain = self.obstacle_cost_gain * 1.5  # Increase obstacle avoidance
            path_gain = self.path_following_gain * 0.5  # Reduce path following priority
        else:
            # Normal parameters
            v_max = min(self.max_speed, v + self.max_accel * self.dt)
            obstacle_gain = self.obstacle_cost_gain
            # Increase path following gain when not in danger to stay closer to the path
            path_gain = self.path_following_gain * 2.0
        
        # If at the end of path, allow stopping
        if at_end_of_path:
            v_min = 0.0
            v_max = min(v_max, 0.5)  # Slow down near the end
        else:
            v_min = max(self.min_speed, v - self.max_accel * self.dt)
        
        # Calculate dynamic window
        omega_min = max(-self.max_yaw_rate, -self.max_delta_yaw_rate)
        omega_max = min(self.max_yaw_rate, self.max_delta_yaw_rate)
        
        # Create velocity samples - more samples for better resolution
        v_samples = np.linspace(v_min, v_max, 11)
        omega_samples = np.linspace(omega_min, omega_max, 11)
        
        # Calculate distance to goal
        dist_to_goal = math.hypot(x - goal[0], y - goal[1])
        
        # Calculate angle to goal for directional preference
        angle_to_goal = math.atan2(goal[1] - y, goal[0] - x)
        angle_diff = self._normalize_angle(angle_to_goal - theta)
        
        best_cost = float('inf')
        best_v = v
        best_omega = 0.0
        best_trajectory = []
        
        # Evaluate each velocity pair
        for v_candidate in v_samples:
            for omega_candidate in omega_samples:
                # Predict trajectory
                trajectory = self._predict_trajectory(x, y, theta, v_candidate, omega_candidate)
                
                # Calculate costs
                to_goal_cost = self._calc_to_goal_cost(trajectory, goal)
                speed_cost = self._calc_speed_cost(v_candidate)
                obstacle_cost = self._calc_obstacle_cost(
                    trajectory, cost_map, map_resolution, map_origin_x, map_origin_y
                )
                path_cost = self._calc_path_following_cost(trajectory, path)
                
                # Total cost with adjusted weights
                total_cost = (
                    self.to_goal_cost_gain * to_goal_cost +
                    self.speed_cost_gain * speed_cost +
                    obstacle_gain * obstacle_cost +
                    path_gain * path_cost
                )
                
                # Update best if this is better
                if total_cost < best_cost and obstacle_cost < float('inf'):
                    best_cost = total_cost
                    best_v = v_candidate
                    best_omega = omega_candidate
                    best_trajectory = trajectory
        
        # If in danger and no valid trajectory found, prioritize stopping
        if danger and best_cost == float('inf'):
            self.get_logger().warn("No safe trajectory found, emergency stop!")
            best_v = 0.0
            best_omega = 0.0
            best_trajectory = [(x, y)]
        
        # Ensure minimum speed unless at end of path
        if abs(best_v) < 0.1 and not danger and not at_end_of_path:
            best_v = 0.1 if best_v >= 0 else -0.1
        
        return best_v, best_omega, best_trajectory
    
    def _predict_trajectory(self, x, y, theta, v, omega):
        """Predict trajectory given control inputs"""
        trajectory = []
        time = 0
        
        # Current state
        curr_x = x
        curr_y = y
        curr_theta = theta
        
        while time <= self.predict_time:
            # Update state
            curr_x += v * math.cos(curr_theta) * self.dt
            curr_y += v * math.sin(curr_theta) * self.dt
            curr_theta += omega * self.dt
            
            # Add to trajectory
            trajectory.append((curr_x, curr_y))
            time += self.dt
        
        return trajectory
    
    def _calc_to_goal_cost(self, trajectory, goal):
        """Calculate cost to goal"""
        if not trajectory:
            return float('inf')
        
        # Distance to goal
        dx = trajectory[-1][0] - goal[0]
        dy = trajectory[-1][1] - goal[1]
        return math.hypot(dx, dy)
    
    def _calc_speed_cost(self, v):
        """Calculate speed cost"""
        # Prefer higher speeds
        return self.max_speed - v
    
    def _calc_obstacle_cost(self, trajectory, cost_map, map_resolution, map_origin_x, map_origin_y):
        """Calculate obstacle cost with improved distance calculation - detecting any black areas"""
        min_dist = float('inf')
        
        # Lower threshold to detect any black areas (not just strong obstacles)
        detection_threshold = 0.5  # Reduced from obstacle_threshold/100.0
        
        for x, y in trajectory:
            # Convert to map coordinates
            map_x = int((x - map_origin_x) / map_resolution)
            map_y = int((y - map_origin_y) / map_resolution)
            
            # Check if in map bounds
            if (0 <= map_x < cost_map.shape[1] and 
                0 <= map_y < cost_map.shape[0]):
                
                # Check if collision with any black area
                if cost_map[map_y, map_x] > detection_threshold:
                    return float('inf')
                
                # Enhanced: Check larger area for black areas with distance weighting
                search_window = 5  # Larger search window
                for dx in range(-search_window, search_window + 1):
                    for dy in range(-search_window, search_window + 1):
                        nx, ny = map_x + dx, map_y + dy
                        if (0 <= nx < cost_map.shape[1] and 
                            0 <= ny < cost_map.shape[0]):
                            
                            # Weight by black area value and distance
                            black_value = cost_map[ny, nx]
                            if black_value > detection_threshold:
                                dist = math.hypot(dx * map_resolution, dy * map_resolution)
                                if dist < min_dist:
                                    min_dist = dist
        
        # Return cost based on distance - higher cost for closer black areas
        if min_dist < float('inf'):
            # Exponential cost increase as distance decreases
            return math.exp(1.0 / (min_dist + 1e-6))
        else:
            return 0.0
    
    def _calc_path_following_cost(self, trajectory, path):
        """Calculate path following cost with improved distance metric"""
        if not trajectory or not path:
            return float('inf')
        
        # Calculate average distance to path with progressive weighting
        total_dist = 0.0
        weights = np.linspace(0.5, 1.0, len(trajectory))  # Higher weight for later points
        
        for i, (x, y) in enumerate(trajectory):
            min_dist = float('inf')
            for px, py in path:
                dist = math.hypot(x - px, y - py)
                min_dist = min(min_dist, dist)
            total_dist += min_dist * weights[i]
        
        return total_dist / sum(weights)
    
    def _get_lookahead_point(self, x, y, path, lookahead_distance=None):
        """Get lookahead point on path with improved selection"""
        if not path:
            return None
        
        if lookahead_distance is None:
            lookahead_distance = self.lookahead_distance
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(path):
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Check if we're near the end of the path
        if closest_idx >= len(path) - 3:  # If we're at one of the last 3 points
            return path[-1]  # Return the last point
        
        # Use a more stable lookahead distance - don't make it too small
        actual_lookahead = max(lookahead_distance, 1.5)
            
        # Find point at lookahead distance
        for i in range(closest_idx, len(path)):
            dist = math.hypot(path[i][0] - x, path[i][1] - y)
            if dist >= actual_lookahead:
                return path[i]
        
        # If no point at lookahead distance, return last point
        return path[-1] if path else None
    
    def publish_cmd_vel(self, v, omega):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_vel_publisher.publish(cmd)
    
    def publish_path(self, trajectory, goal=None, danger=False, original_path=None, modified_path=None):
        """Publish planned path with enhanced visualization"""
        # Create path message
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id
        
        # Add trajectory points
        for x, y in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        # Publish path
        self.path_publisher.publish(path_msg)
        
        # Create visualization markers
        self.publish_visualization(trajectory, goal, danger, original_path, modified_path)
    
    def publish_visualization(self, trajectory, goal=None, danger=False, original_path=None, modified_path=None):
        """Publish enhanced visualization markers with color gradient for extended trajectory"""
        marker_array = MarkerArray()
        
        # Original waypoint path (thin yellow line)
        if original_path and len(original_path) > 1:
            original_marker = Marker()
            original_marker.header.frame_id = self.map_frame_id
            original_marker.header.stamp = self.get_clock().now().to_msg()
            original_marker.ns = "original_waypoint_path"
            original_marker.id = 10
            original_marker.type = Marker.LINE_STRIP
            original_marker.action = Marker.ADD
            original_marker.scale.x = 0.05  # Thin line
            original_marker.color.r = 1.0
            original_marker.color.g = 1.0
            original_marker.color.b = 0.0
            original_marker.color.a = 0.6
            original_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
            
            for x, y in original_path:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1  # Just above ground
                original_marker.points.append(p)
            
            marker_array.markers.append(original_marker)
            
            # Add waypoint markers
            for i, (x, y) in enumerate(original_path):
                if i % 5 == 0:  # Only show every 5th waypoint to avoid cluttering
                    wp_marker = Marker()
                    wp_marker.header.frame_id = self.map_frame_id
                    wp_marker.header.stamp = self.get_clock().now().to_msg()
                    wp_marker.ns = "waypoint_markers"
                    wp_marker.id = i
                    wp_marker.type = Marker.SPHERE
                    wp_marker.action = Marker.ADD
                    wp_marker.pose.position.x = x
                    wp_marker.pose.position.y = y
                    wp_marker.pose.position.z = 0.15
                    wp_marker.scale.x = 0.2
                    wp_marker.scale.y = 0.2
                    wp_marker.scale.z = 0.2
                    wp_marker.color.r = 1.0
                    wp_marker.color.g = 1.0
                    wp_marker.color.b = 0.0
                    wp_marker.color.a = 0.8
                    wp_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
                    marker_array.markers.append(wp_marker)
        
        # Modified path (thicker purple line)
        if modified_path and len(modified_path) > 1:
            # Check if modified path is significantly different from original path
            is_different = False
            if original_path and len(modified_path) != len(original_path):
                is_different = True
            elif original_path:
                # Sample a few points to check if paths are different
                sample_indices = [min(i, len(modified_path)-1) for i in range(0, len(original_path), max(1, len(original_path)//5))]
                for idx in sample_indices:
                    if idx < len(original_path) and idx < len(modified_path):
                        mp = modified_path[idx]
                        op = original_path[idx]
                        if math.hypot(mp[0] - op[0], mp[1] - op[1]) > 0.5:  # If points differ by more than 0.5m
                            is_different = True
                            break
            
            if is_different:
                modified_marker = Marker()
                modified_marker.header.frame_id = self.map_frame_id
                modified_marker.header.stamp = self.get_clock().now().to_msg()
                modified_marker.ns = "modified_path"
                modified_marker.id = 11
                modified_marker.type = Marker.LINE_STRIP
                modified_marker.action = Marker.ADD
                modified_marker.scale.x = 0.15  # Thicker line
                modified_marker.color.r = 0.8
                modified_marker.color.g = 0.2
                modified_marker.color.b = 0.8
                modified_marker.color.a = 0.8
                modified_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
                
                for x, y in modified_path:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = 0.15  # Above the original path
                    modified_marker.points.append(p)
                
                marker_array.markers.append(modified_marker)
        
        # Trajectory marker with color gradient
        if trajectory:
            # Create line strip marker for the trajectory
            line_marker = Marker()
            line_marker.header.frame_id = self.map_frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "dwa_trajectory_line"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.1  # Line width
            
            # Base color based on danger state (yellow=danger, green=safe)
            line_marker.color.r = 1.0 if danger else 0.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0
            line_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))  # 0.5 second
            
            # Add points to line strip
            for x, y in trajectory:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.2  # Slightly above ground
                line_marker.points.append(p)
            
            marker_array.markers.append(line_marker)
            
            # Add points with color gradient for better visualization of the trajectory
            points_marker = Marker()
            points_marker.header.frame_id = self.map_frame_id
            points_marker.header.stamp = self.get_clock().now().to_msg()
            points_marker.ns = "dwa_trajectory_points"
            points_marker.id = 3
            points_marker.type = Marker.POINTS
            points_marker.action = Marker.ADD
            points_marker.scale.x = 0.15  # Point size
            points_marker.scale.y = 0.15
            points_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))  # 0.5 second
            
            # Add points with color gradient
            num_points = len(trajectory)
            for i, (x, y) in enumerate(trajectory):
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.2  # Slightly above ground
                points_marker.points.append(p)
                
                # Create color gradient from green/yellow to red
                color = ColorRGBA()
                ratio = i / max(1, num_points - 1)  # 0 at start, 1 at end
                
                if danger:
                    # Yellow to red gradient for danger
                    color.r = 1.0
                    color.g = 1.0 - ratio * 0.8
                    color.b = 0.0
                else:
                    # Green to blue gradient for safe
                    color.r = ratio * 0.5
                    color.g = 1.0 - ratio * 0.5
                    color.b = ratio
                
                color.a = 1.0
                points_marker.colors.append(color)
            
            marker_array.markers.append(points_marker)
        
        # Goal marker
        if goal:
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dwa_goal"
            marker.id = 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = goal[0]
            marker.pose.position.y = goal[1]
            marker.pose.position.z = 0.2
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Color based on danger (red=danger, blue=normal)
            marker.color.r = 1.0
            marker.color.g = 0.0 if danger else 0.5
            marker.color.b = 0.0 if danger else 1.0
            marker.color.a = 1.0
            marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))  # 0.5 second
            
            marker_array.markers.append(marker)
            
            # Add lane width and status visualization
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "lane_width"
            text_marker.id = 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = goal[0]
            text_marker.pose.position.y = goal[1]
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.5  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            # Show status information
            status = "DANGER: Black Area Avoidance" if danger else "Normal Navigation"
            text_marker.text = f"Lane Width: {self.current_lane_width:.2f}m | {status} | Pred: {self.extended_predict_time:.1f}s"
            text_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))  # 0.5 second
            
            marker_array.markers.append(text_marker)
        
        # Visualize detected black areas
        with self.map_lock:
            if self.cost_map is not None and self.has_waypoints:
                with self.waypoints_lock:
                    waypoints = self.waypoints.copy()
                    lane_width = self.current_lane_width
                
                original_path = []
                for wp in waypoints:
                    original_path.append((wp['x'], wp['y']))
                
                if original_path:
                    # Detect black areas on the path
                    all_obstacles = self.find_obstacles_on_path(original_path, threshold=0.5)
                    lane_obstacles = [obs for obs in all_obstacles if self.is_obstacle_in_lane(obs, original_path, lane_width)]
                    
                    # Visualize all detected black areas
                    for i, (ox, oy, wp_idx) in enumerate(all_obstacles):
                        # Check if this black area is in the lane
                        is_in_lane = any(obs[0] == ox and obs[1] == oy for obs in lane_obstacles)
                        
                        # Create marker for black area
                        obs_marker = Marker()
                        obs_marker.header.frame_id = self.map_frame_id
                        obs_marker.header.stamp = self.get_clock().now().to_msg()
                        obs_marker.ns = "black_areas"
                        obs_marker.id = i + 1000  # Offset to avoid ID conflicts
                        obs_marker.type = Marker.CYLINDER
                        obs_marker.action = Marker.ADD
                        obs_marker.pose.position.x = ox
                        obs_marker.pose.position.y = oy
                        obs_marker.pose.position.z = 0.05  # Just above ground
                        obs_marker.scale.x = 0.3  # Diameter
                        obs_marker.scale.y = 0.3
                        obs_marker.scale.z = 0.1  # Height
                        
                        # Color: red for in-lane, orange for out-of-lane
                        if is_in_lane:
                            obs_marker.color.r = 1.0
                            obs_marker.color.g = 0.0
                            obs_marker.color.b = 0.0
                        else:
                            obs_marker.color.r = 1.0
                            obs_marker.color.g = 0.5
                            obs_marker.color.b = 0.0
                        
                        obs_marker.color.a = 0.7
                        obs_marker.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
                        
                        marker_array.markers.append(obs_marker)
                    
                    # Visualize lane boundaries
                    if len(original_path) > 1:
                        left_boundary = Marker()
                        left_boundary.header.frame_id = self.map_frame_id
                        left_boundary.header.stamp = self.get_clock().now().to_msg()
                        left_boundary.ns = "lane_boundaries"
                        left_boundary.id = 1
                        left_boundary.type = Marker.LINE_STRIP
                        left_boundary.action = Marker.ADD
                        left_boundary.scale.x = 0.03  # Thin line
                        left_boundary.color.r = 0.0
                        left_boundary.color.g = 0.8
                        left_boundary.color.b = 0.8
                        left_boundary.color.a = 0.5
                        left_boundary.lifetime = Duration(sec=0, nanosec=int(0.5 * 1e9))
                        
                        right_boundary = Marker()
                        right_boundary.header = left_boundary.header
                        right_boundary.ns = "lane_boundaries"
                        right_boundary.id = 2
                        right_boundary.type = Marker.LINE_STRIP
                        right_boundary.action = Marker.ADD
                        right_boundary.scale.x = 0.03
                        right_boundary.color = left_boundary.color
                        right_boundary.lifetime = left_boundary.lifetime
                        
                        half_lane = lane_width / 2.0
                        
                        for i in range(len(original_path) - 1):
                            p1 = original_path[i]
                            p2 = original_path[i+1]
                            
                            # Calculate direction vector
                            dx = p2[0] - p1[0]
                            dy = p2[1] - p1[1]
                            length = math.hypot(dx, dy)
                            
                            if length > 0.001:  # Avoid division by zero
                                # Normalize
                                dx /= length
                                dy /= length
                                
                                # Perpendicular vector
                                px = -dy
                                py = dx
                                
                                # Left and right boundary points
                                left_x = p1[0] + px * half_lane
                                left_y = p1[1] + py * half_lane
                                right_x = p1[0] - px * half_lane
                                right_y = p1[1] - py * half_lane
                                
                                # Add points to markers
                                left_point = Point()
                                left_point.x = left_x
                                left_point.y = left_y
                                left_point.z = 0.05
                                left_boundary.points.append(left_point)
                                
                                right_point = Point()
                                right_point.x = right_x
                                right_point.y = right_y
                                right_point.z = 0.05
                                right_boundary.points.append(right_point)
                        
                        # Add last points
                        if len(original_path) > 0:
                            last = original_path[-1]
                            p1 = original_path[-2] if len(original_path) > 1 else original_path[0]
                            
                            # Calculate direction vector (use same direction as last segment)
                            dx = last[0] - p1[0]
                            dy = last[1] - p1[1]
                            length = math.hypot(dx, dy)
                            
                            if length > 0.001:
                                # Normalize
                                dx /= length
                                dy /= length
                                
                                # Perpendicular vector
                                px = -dy
                                py = dx
                                
                                # Left and right boundary points
                                left_x = last[0] + px * half_lane
                                left_y = last[1] + py * half_lane
                                right_x = last[0] - px * half_lane
                                right_y = last[1] - py * half_lane
                                
                                # Add points to markers
                                left_point = Point()
                                left_point.x = left_x
                                left_point.y = left_y
                                left_point.z = 0.05
                                left_boundary.points.append(left_point)
                                
                                right_point = Point()
                                right_point.x = right_x
                                right_point.y = right_y
                                right_point.z = 0.05
                                right_boundary.points.append(right_point)
                        
                        marker_array.markers.append(left_boundary)
                        marker_array.markers.append(right_boundary)
        
        # Publish markers
        self.marker_publisher.publish(marker_array)
    
    def destroy_node(self):
        """Clean up when node is destroyed"""
        # Stop publishing commands
        try:
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
        except:
            pass
        
        super().destroy_node()


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
