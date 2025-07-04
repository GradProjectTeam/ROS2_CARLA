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
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseStamped, Twist, PoseArray, Pose
from std_msgs.msg import Header, ColorRGBA, Int32MultiArray, Bool, Float32
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration
from tf2_geometry_msgs import do_transform_pose
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2, LaserScan


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
        self.declare_parameter('path_topic', '/dwa/path')
        self.declare_parameter('cmd_vel_topic', '/dwa/cmd_vel')
        self.declare_parameter('obstacle_detected_topic', '/dwa/obstacle_detected')
        self.declare_parameter('publish_rate', 20.0)  # Hz - Updated to match unified_rate
        self.declare_parameter('max_speed', 30.0)  # Reduced from 50.0 to 30.0 to better match MPC's max speed
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_yaw_rate', 0.5)  # Increased for better heading control
        self.declare_parameter('max_accel', 8.0)  # Increased for faster acceleration
        self.declare_parameter('max_delta_yaw_rate', 0.3)  # Increased for more responsive heading changes
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('predict_time', 2.5)  # Reduced for more frequent heading updates
        self.declare_parameter('to_goal_cost_gain', 1.5)  # Adjusted for better balance
        self.declare_parameter('speed_cost_gain', 0.2)  # Reduced to prioritize obstacle avoidance
        self.declare_parameter('obstacle_cost_gain', 25.0)  # Further increased for stronger obstacle avoidance
        self.declare_parameter('path_following_gain', 15.0)  # Adjusted for better path following
        self.declare_parameter('lookahead_distance', 10.0)  # Reduced for tighter heading control
        self.declare_parameter('obstacle_threshold', 30)  # Significantly lowered threshold for black obstacle detection
        self.declare_parameter('safe_distance', 6.0)  # Increased safety margin
        self.declare_parameter('default_lane_width', 3.2)
        self.declare_parameter('lane_width_factor', 0.9)  # Increased for wider lane consideration
        self.declare_parameter('start_point_offset', 0.3)  # Reduced to start turns earlier
        self.declare_parameter('min_obstacle_distance', 2.0)  # Increased minimum distance to obstacles
        self.declare_parameter('path_smoothing_factor', 0.5)  # Reduced for more precise path following
        self.declare_parameter('lateral_safety_margin', 1.8)  # Increased lateral safety margin
        self.declare_parameter('obstacle_weight_decay', 0.1)  # Adjusted decay for stronger close-range avoidance
        self.declare_parameter('adaptive_lookahead', True)
        
        # New parameters for enhanced obstacle avoidance
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('emergency_brake_distance', 8.0)  # Increased emergency brake distance
        self.declare_parameter('obstacle_path_pruning', True)
        self.declare_parameter('debug_level', 'debug')  # Force debug level for this node
        
        # Additional parameters from launch file
        self.declare_parameter('verbose_logging', True)  # Enable verbose logging
        self.declare_parameter('log_map_processing', True)  # Log details about map processing
        self.declare_parameter('log_obstacle_detection', True)  # Log details about obstacle detection
        self.declare_parameter('log_path_planning', True)  # Log details about path planning
        self.declare_parameter('publish_debug_images', True)  # Publish debug images
        self.declare_parameter('publish_debug_markers', True)  # Publish debug markers
        self.declare_parameter('diagnostic_level', 2)  # Maximum diagnostic level
        
        # Enhanced parameters for black obstacles on gray lanes
        self.declare_parameter('lane_obstacle_detection_enabled', True)  # Enable specific detection of obstacles crossing lanes
        self.declare_parameter('lane_obstacle_threshold', 30)  # Lower threshold for detecting black obstacles (black is close to 0)
        self.declare_parameter('lane_gray_min_threshold', 100)  # Minimum gray value to be considered a lane
        self.declare_parameter('lane_gray_max_threshold', 200)  # Maximum gray value to be considered a lane
        self.declare_parameter('lane_width_for_obstacles', 4.0)  # Width of lane to check for obstacles (meters)
        self.declare_parameter('lane_obstacle_min_area', 30)  # Minimum area of obstacle pixels to consider
        self.declare_parameter('lane_obstacle_stop_distance', 10.0)  # Distance to start stopping when lane obstacle detected
        self.declare_parameter('lane_obstacle_slow_distance', 15.0)  # Distance to start slowing down when lane obstacle detected
        
        # Black lane detection parameters
        self.declare_parameter('black_lane_detection_enabled', True)  # Enable black lane detection
        self.declare_parameter('black_lane_threshold', 0)  # Lower threshold for detecting black lanes (0-255)
        self.declare_parameter('black_lane_min_area', 10)  # Minimum area of black pixels to consider as a lane
        self.declare_parameter('black_lane_stop_distance', 20.0)  # Distance to start stopping when black lane detected
        self.declare_parameter('black_lane_slow_distance', 30.0)  # Distance to start slowing down when black lane detected
        self.declare_parameter('min_cmd_vel_for_stop', 0.3)  # Lower velocity to send when stopping (for more immediate stops)
        
        # Waypoint crossing obstacle parameters
        self.declare_parameter('waypoint_obstacle_check_enabled', True)  # Enable checking if obstacles cross waypoints
        self.declare_parameter('waypoint_corridor_width', 3.5)  # Width of corridor around waypoints to check for obstacles
        self.declare_parameter('waypoint_obstacle_lookahead', 20.0)  # How far ahead to check waypoints for obstacles
        self.declare_parameter('waypoint_obstacle_threshold', 30)  # Lower threshold for detecting black obstacles on waypoint path
        self.declare_parameter('waypoint_obstacle_stop_command', True)  # Send stop command when obstacle crosses waypoints
        
        # Color-specific detection
        self.declare_parameter('color_detection_enabled', True)  # Enable color-specific detection
        self.declare_parameter('black_obstacle_max_value', 50)  # Maximum pixel value to be considered black (obstacles)
        self.declare_parameter('gray_lane_min_value', 100)  # Minimum pixel value to be considered gray (lanes)
        self.declare_parameter('gray_lane_max_value', 200)  # Maximum pixel value to be considered gray (lanes)
        self.declare_parameter('contrast_enhancement_enabled', True)  # Enable contrast enhancement for better detection
        
        # Map subscription parameters
        self.declare_parameter('map_subscription_qos', 'reliable_transient')  # Use reliable and transient local QoS for map subscription
        self.declare_parameter('map_subscription_timeout', 60.0)  # Wait up to 60 seconds for map data
        self.declare_parameter('map_subscription_retry_interval', 2.0)  # Retry every 2 seconds
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').get_parameter_value().string_value
        self.waypoints_topic = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        self.waypoint_markers_topic = self.get_parameter('waypoint_markers_topic').get_parameter_value().string_value
        self.binary_map_topic = self.get_parameter('binary_map_topic').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.obstacle_detected_topic = self.get_parameter('obstacle_detected_topic').get_parameter_value().string_value
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
        self.start_point_offset = self.get_parameter('start_point_offset').get_parameter_value().double_value
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').get_parameter_value().double_value
        self.path_smoothing_factor = self.get_parameter('path_smoothing_factor').get_parameter_value().double_value
        self.lateral_safety_margin = self.get_parameter('lateral_safety_margin').get_parameter_value().double_value
        self.obstacle_weight_decay = self.get_parameter('obstacle_weight_decay').get_parameter_value().double_value
        self.adaptive_lookahead = self.get_parameter('adaptive_lookahead').get_parameter_value().bool_value
        
        # Get new parameters for enhanced obstacle avoidance
        self.emergency_stop_enabled = self.get_parameter('emergency_stop_enabled').get_parameter_value().bool_value
        self.emergency_brake_distance = self.get_parameter('emergency_brake_distance').get_parameter_value().double_value
        self.obstacle_path_pruning = self.get_parameter('obstacle_path_pruning').get_parameter_value().bool_value
        self.debug_level = self.get_parameter('debug_level').get_parameter_value().string_value
        
        # Get additional parameters from launch file
        self.verbose_logging = self.get_parameter('verbose_logging').get_parameter_value().bool_value
        self.log_map_processing = self.get_parameter('log_map_processing').get_parameter_value().bool_value
        self.log_obstacle_detection = self.get_parameter('log_obstacle_detection').get_parameter_value().bool_value
        self.log_path_planning = self.get_parameter('log_path_planning').get_parameter_value().bool_value
        self.publish_debug_images = self.get_parameter('publish_debug_images').get_parameter_value().bool_value
        self.publish_debug_markers = self.get_parameter('publish_debug_markers').get_parameter_value().bool_value
        self.diagnostic_level = self.get_parameter('diagnostic_level').get_parameter_value().integer_value
        
        # Get enhanced parameters for black obstacles on gray lanes
        self.lane_obstacle_detection_enabled = self.get_parameter('lane_obstacle_detection_enabled').get_parameter_value().bool_value
        self.lane_obstacle_threshold = self.get_parameter('lane_obstacle_threshold').get_parameter_value().integer_value
        self.lane_gray_min_threshold = self.get_parameter('lane_gray_min_threshold').get_parameter_value().integer_value
        self.lane_gray_max_threshold = self.get_parameter('lane_gray_max_threshold').get_parameter_value().integer_value
        self.lane_width_for_obstacles = self.get_parameter('lane_width_for_obstacles').get_parameter_value().double_value
        self.lane_obstacle_min_area = self.get_parameter('lane_obstacle_min_area').get_parameter_value().integer_value
        self.lane_obstacle_stop_distance = self.get_parameter('lane_obstacle_stop_distance').get_parameter_value().double_value
        self.lane_obstacle_slow_distance = self.get_parameter('lane_obstacle_slow_distance').get_parameter_value().double_value
        
        # Get black lane detection parameters
        self.black_lane_detection_enabled = self.get_parameter('black_lane_detection_enabled').get_parameter_value().bool_value
        self.black_lane_threshold = self.get_parameter('black_lane_threshold').get_parameter_value().integer_value
        self.black_lane_min_area = self.get_parameter('black_lane_min_area').get_parameter_value().integer_value
        self.black_lane_stop_distance = self.get_parameter('black_lane_stop_distance').get_parameter_value().double_value
        self.black_lane_slow_distance = self.get_parameter('black_lane_slow_distance').get_parameter_value().double_value
        self.min_cmd_vel_for_stop = self.get_parameter('min_cmd_vel_for_stop').get_parameter_value().double_value
        
        # Get waypoint crossing obstacle parameters
        self.waypoint_obstacle_check_enabled = self.get_parameter('waypoint_obstacle_check_enabled').get_parameter_value().bool_value
        self.waypoint_corridor_width = self.get_parameter('waypoint_corridor_width').get_parameter_value().double_value
        self.waypoint_obstacle_lookahead = self.get_parameter('waypoint_obstacle_lookahead').get_parameter_value().double_value
        self.waypoint_obstacle_threshold = self.get_parameter('waypoint_obstacle_threshold').get_parameter_value().integer_value
        self.waypoint_obstacle_stop_command = self.get_parameter('waypoint_obstacle_stop_command').get_parameter_value().bool_value
        
        # Get color-specific detection parameters
        self.color_detection_enabled = self.get_parameter('color_detection_enabled').get_parameter_value().bool_value
        self.black_obstacle_max_value = self.get_parameter('black_obstacle_max_value').get_parameter_value().integer_value
        self.gray_lane_min_value = self.get_parameter('gray_lane_min_value').get_parameter_value().integer_value
        self.gray_lane_max_value = self.get_parameter('gray_lane_max_value').get_parameter_value().integer_value
        self.contrast_enhancement_enabled = self.get_parameter('contrast_enhancement_enabled').get_parameter_value().bool_value
        
        # Get map subscription parameters
        self.map_subscription_qos = self.get_parameter('map_subscription_qos').get_parameter_value().string_value
        self.map_subscription_timeout = self.get_parameter('map_subscription_timeout').get_parameter_value().double_value
        self.map_subscription_retry_interval = self.get_parameter('map_subscription_retry_interval').get_parameter_value().double_value
        
        # For obstacle detection
        self.min_obstacle_count = 1  # Minimum number of obstacle points to trigger a stop
        self.obstacle_inflation_radius = 2  # Radius to inflate obstacles for collision checking
        
        # Set logging level
        if self.debug_level == 'debug':
            rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)
        elif self.debug_level == 'info':
            rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.INFO)
        elif self.debug_level == 'warn':
            rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.WARN)
        elif self.debug_level == 'error':
            rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.ERROR)
        
        # Initialize sensor-based obstacle data
        self.lane_obstacle_positions = []  # Initialize empty list for lane obstacle positions
        self.detected_obstacles = []  # Initialize empty list for detected obstacles
        self.obstacle_memory_time = 2.0  # How long to remember obstacles (seconds)
        self.last_obstacle_time = self.get_clock().now()  # Time of last obstacle detection
        
        # Create different QoS profiles for different types of communications
        
        # For map data - use transient local for late joining and reliable communication
        map_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
            
        # For waypoints - use the same QoS profile as the waypoint_listener node
        waypoints_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,  # Changed from TRANSIENT_LOCAL to VOLATILE to match waypoint_listener
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # For control commands - use reliable but not transient local (best effort might be better for real-time)
        control_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only need the latest command
        )
        
        # For visualization - can use best effort for better performance
        viz_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only need the latest visualization
        )
        
        # Create publishers
        self.path_publisher = self.create_publisher(
            Path,
            self.path_topic,
            control_qos_profile  # Path is used for control, so use control QoS
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            control_qos_profile  # Critical control commands
        )
        
        # Create visualization marker publisher
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            self.path_topic + '_markers',
            viz_qos_profile  # Visualization can use best effort
        )
        
        # Create obstacle detection publisher - this is safety critical so use reliable
        self.obstacle_detected_publisher = self.create_publisher(
            Bool,
            self.obstacle_detected_topic,
            control_qos_profile  # Safety critical, use reliable
        )
        
        # Create parking spots visualization publisher
        self.parking_spots_publisher = self.create_publisher(
            MarkerArray,
            self.path_topic + '_parking_spots',
            viz_qos_profile  # Visualization can use best effort
        )
        
        # Create obstacle visualization publisher
        self.obstacle_viz_pub = self.create_publisher(
            MarkerArray,
            self.path_topic + '_obstacles',
            viz_qos_profile  # Visualization can use best effort
        )
        
        # Create subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            self.binary_map_topic,
            self.map_callback,
            map_qos_profile  # Map data needs transient local
        )
        
        self.waypoints_subscriber = self.create_subscription(
            PoseArray,
            self.waypoints_topic,
            self.waypoints_callback,
            waypoints_qos_profile  # Waypoints need transient local
        )
        
        # Also subscribe to waypoint metadata to get lane information
        self.metadata_subscriber = self.create_subscription(
            Int32MultiArray,
            self.waypoints_topic + "_metadata",
            self.metadata_callback,
            waypoints_qos_profile  # Metadata should match waypoints QoS
        )
        
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            self.waypoint_markers_topic,
            self.markers_callback,
            viz_qos_profile  # Markers are visualization, can use best effort
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
        
        # Define lane threshold values for obstacle cost calculation
        self.lane_threshold_min = self.lane_gray_min_threshold if hasattr(self, 'lane_gray_min_threshold') else 40
        self.lane_threshold_max = self.lane_gray_max_threshold if hasattr(self, 'lane_gray_max_threshold') else 60
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_v = 2.0  # Initialize with a higher positive velocity for better start
        self.current_omega = 0.0
        self.current_pose = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.current_goal = None  # Will be set based on target waypoint
        
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
        
        # Status variables
        self.has_map = False
        self.has_waypoints = False
        self.last_plan_time = self.get_clock().now()
        
        # Add velocity feedback
        self.current_velocity = 0.0  # m/s
        self.velocity_sub = self.create_subscription(
            Float32,  # Velocity comes as Float32
            '/carla/ego_vehicle/velocity',  # Topic published by velocity_listener
            self.velocity_callback,
            10
        )
        
        self.get_logger().info("Enhanced DWA Planner node initialized")
    
    def velocity_callback(self, msg):
        """Handle velocity feedback"""
        self.current_velocity = msg.data  # m/s
        self.get_logger().debug(f"Received velocity: {self.current_velocity} m/s")
        
    def map_callback(self, msg: OccupancyGrid):
        """Process incoming binary map that combines obstacles (black) and waypoint lanes (grey)"""
        try:
            with self.map_lock:
                # Extract map data
                self.map_width = msg.info.width
                self.map_height = msg.info.height
                self.map_resolution = msg.info.resolution
                self.map_origin_x = msg.info.origin.position.x
                self.map_origin_y = msg.info.origin.position.y
                
                # Store the raw binary map data
                self.binary_map = msg  # Store the entire OccupancyGrid message
                
                # Convert from 1D array to 2D numpy array (0-100 scale)
                self.cost_map = np.array(msg.data).reshape((self.map_height, self.map_width))
                
                # Create separate maps for obstacles and lanes
                # Obstacles (black) are represented by values close to 0 (in binary map)
                # The threshold is defined by obstacle_threshold parameter (default 30)
                self.obstacle_map = (self.cost_map < self.obstacle_threshold).astype(np.float32)
                
                # Lanes (grey) are represented by values around 50
                if hasattr(self, 'lane_gray_min_threshold') and hasattr(self, 'lane_gray_max_threshold'):
                    self.lane_map = ((self.cost_map >= self.lane_gray_min_threshold) & 
                                    (self.cost_map <= self.lane_gray_max_threshold)).astype(np.float32)
                    # Update lane thresholds for obstacle cost calculation
                    self.lane_threshold_min = self.lane_gray_min_threshold
                    self.lane_threshold_max = self.lane_gray_max_threshold
                else:
                    # Default range if specific thresholds aren't available
                    self.lane_map = ((self.cost_map >= 40) & (self.cost_map <= 60)).astype(np.float32)
                    # Update lane thresholds with default values
                    self.lane_threshold_min = 40
                    self.lane_threshold_max = 60
                
                # Store map data for obstacle detection algorithms
                self.map_data = self.cost_map
                
                self.has_map = True
                
                if self.verbose_logging:
                    self.get_logger().info(f"Received binary map {self.map_width}x{self.map_height} with resolution {self.map_resolution}")
                else:
                    self.get_logger().debug(f"Received binary map {self.map_width}x{self.map_height} with resolution {self.map_resolution}")
        except Exception as e:
            self.get_logger().error(f"Error processing map: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
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
                
                # Update current_pose
                self.current_pose = [self.current_x, self.current_y, self.current_theta]
                
                # Note: velocity would ideally come from odometry
                # For now, we'll keep the existing velocity
                
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not update vehicle state: {e}")
            return False
    
    def find_obstacles_on_path(self, path_points, binary_map, vehicle_pose):
        """Find obstacles on the planned path."""
        # Check if binary_map is None
        if binary_map is None:
            self.get_logger().warn("Binary map is None, cannot detect obstacles")
            return False
            
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
        
        try:
            # Extract the binary map data as a numpy array
            map_width = binary_map.info.width
            map_height = binary_map.info.height
            map_resolution = binary_map.info.resolution
            map_origin_x = binary_map.info.origin.position.x
            map_origin_y = binary_map.info.origin.position.y
            
            # Convert 1D array to 2D numpy array
            binary_data = np.array(binary_map.data).reshape((map_height, map_width))
            
            # Log map info if verbose logging is enabled
            if self.verbose_logging and self.log_obstacle_detection:
                self.get_logger().info(f"Checking path with {len(path_points)} points against binary map {map_width}x{map_height}")
                self.get_logger().info(f"Lane threshold min: {self.lane_threshold_min}, max: {self.lane_threshold_max}")
                self.get_logger().info(f"Obstacle threshold: {self.obstacle_threshold}")
            
            # Check path points for obstacles
            for i, (px, py) in enumerate(path_points):
                # Get distance from vehicle to path point
                distance = math.sqrt((px - x) ** 2 + (py - y) ** 2)
                
                # Skip points that are too close to the vehicle
                if distance < 0.3:
                    continue
                    
                # Convert path point to map coordinates
                map_x = int((px - map_origin_x) / map_resolution)
                map_y = int((py - map_origin_y) / map_resolution)
                
                # Check if point is within map bounds
                if (map_x < 0 or map_x >= map_width or
                    map_y < 0 or map_y >= map_height):
                    continue
                
                # Get pixel value at this point
                pixel_value = binary_data[map_y, map_x]
                
                # FIRST CHECK: Detect any high-cost cells (like cars) when they're close
                # The semantic costmap visualizer sets costs to 100 for detected cars
                if pixel_value >= 90 and distance < 5.0:  # High cost cells within 5 meters
                    obstacle_count += 1
                    self.lane_obstacle_positions.append((px, py))
                    
                    if self.log_obstacle_detection:
                        self.get_logger().warn(f"HIGH COST OBSTACLE detected at ({px:.2f}, {py:.2f}), distance: {distance:.2f}m, pixel value: {pixel_value}")
                    
                    # Update closest obstacle distance
                    if distance < closest_obstacle_distance:
                        closest_obstacle_distance = distance
                    
                    # Check if emergency stop is needed
                    if self.emergency_stop_enabled and distance < self.emergency_brake_distance:
                        self.get_logger().warn(f"Emergency stop triggered: High cost obstacle at {distance:.2f}m")
                        has_obstacles = True
                
                # SECOND CHECK: Original check for black obstacles on gray lanes
                # Check if this point is on a lane (gray area)
                is_on_lane = self.lane_threshold_min <= pixel_value <= self.lane_threshold_max
                
                # Only consider obstacles that are black pixels on gray lane areas
                if is_on_lane:
                    # Check if there are black pixels (obstacles) in the vicinity
                    black_pixels_found = 0
                    for dy in range(-2, 3):  # Check a 5x5 area
                        for dx in range(-2, 3):
                            check_x, check_y = map_x + dx, map_y + dy
                            if (0 <= check_x < map_width and 0 <= check_y < map_height):
                                check_value = binary_data[check_y, check_x]
                                # Black pixels have values below obstacle_threshold
                                if check_value < self.obstacle_threshold:
                                    black_pixels_found += 1
                    
                    # Only consider it an obstacle if we find multiple black pixels
                    if black_pixels_found >= 3 and distance < self.min_obstacle_distance:
                        obstacle_count += 1
                        self.lane_obstacle_positions.append((px, py))
                        
                        if self.log_obstacle_detection:
                            self.get_logger().info(f"Black obstacle ON LANE detected at ({px:.2f}, {py:.2f}), distance: {distance:.2f}m, pixel value: {pixel_value}, black pixels: {black_pixels_found}")
                        
                        # Update closest obstacle distance
                        if distance < closest_obstacle_distance:
                            closest_obstacle_distance = distance
                        
                        # Check if emergency stop is needed
                        if self.emergency_stop_enabled and distance < self.emergency_brake_distance:
                            self.get_logger().warn(f"Emergency stop triggered: Black obstacle on lane at {distance:.2f}m")
                            has_obstacles = True
                
                # THIRD CHECK: Look for any obstacles near the vehicle regardless of color/value
                # This is a safety check for very close objects
                if distance < 3.0:  # Within 3 meters
                    # Check surrounding area for any non-free space
                    high_cost_pixels = 0
                    for dy in range(-3, 4):  # Check a 7x7 area
                        for dx in range(-3, 4):
                            check_x, check_y = map_x + dx, map_y + dy
                            if (0 <= check_x < map_width and 0 <= check_y < map_height):
                                check_value = binary_data[check_y, check_x]
                                # Consider any non-free space as potential obstacle
                                if check_value > 60:  # Adjust this threshold as needed
                                    high_cost_pixels += 1
                    
                    # If we find enough high cost pixels, consider it an obstacle
                    if high_cost_pixels >= 5:  # Adjust this threshold as needed
                        obstacle_count += 1
                        self.lane_obstacle_positions.append((px, py))
                        
                        if self.log_obstacle_detection:
                            self.get_logger().warn(f"CLOSE PROXIMITY OBSTACLE detected at ({px:.2f}, {py:.2f}), distance: {distance:.2f}m, high cost pixels: {high_cost_pixels}")
                        
                        # Update closest obstacle distance
                        if distance < closest_obstacle_distance:
                            closest_obstacle_distance = distance
                        
                        # Immediate stop for very close obstacles
                        if distance < 2.0:  # Very close
                            self.get_logger().error(f"IMMEDIATE STOP: Very close obstacle at {distance:.2f}m")
                            has_obstacles = True
            
            # Set has_obstacles based on obstacle count and threshold
            if obstacle_count >= self.min_obstacle_count:
                has_obstacles = True
                self.get_logger().info(f"Obstacles detected: {obstacle_count} obstacles, closest at {closest_obstacle_distance:.2f}m")
            
            # Additional check for very close obstacles
            if closest_obstacle_distance < self.min_obstacle_distance and obstacle_count > 0:
                has_obstacles = True
                self.get_logger().warn(f"Close obstacle detected at {closest_obstacle_distance:.2f}m - stopping (min distance: {self.min_obstacle_distance}m)")
            
            # If we have obstacles, make sure to warn about it
            if has_obstacles:
                self.get_logger().warn(f"Emergency stop: {obstacle_count} obstacles detected")
                
                # Publish obstacle detected message
                obstacle_msg = Bool()
                obstacle_msg.data = True
                self.obstacle_detected_publisher.publish(obstacle_msg)
                
                # Visualize the obstacles
                self.visualize_lane_obstacles()
            else:
                # No obstacles detected
                if self.log_obstacle_detection:
                    self.get_logger().debug("No obstacles detected")
            
            return has_obstacles
            
        except Exception as e:
            self.get_logger().error(f"Error in find_obstacles_on_path: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
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
        """Calculate the dynamic window based on current state and velocity feedback."""
        # Use velocity feedback from the velocity listener
        current_v = self.current_velocity
        
        # Window from robot specification
        v_min = self.min_speed
        v_max = self.max_speed
        omega_min = -self.max_yaw_rate
        omega_max = self.max_yaw_rate
        
        # Limit the maximum yaw rate based on current velocity
        # Higher velocities should have more limited steering
        if current_v > 0.5:
            # Reduce max steering angle at higher speeds - make this more aggressive
            speed_factor = min(1.0, current_v / self.max_speed)
            # Reduce max steering to 30% at maximum speed for stability
            omega_min = -self.max_yaw_rate * (1.0 - 0.7 * speed_factor)
            omega_max = self.max_yaw_rate * (1.0 - 0.7 * speed_factor)
            
            # Additional hard limit on steering at higher speeds
            if current_v > 16.67:  # 60 km/h
                max_steering_limit = 0.3  # Hard limit steering to 0.3 rad/s at higher speeds
                omega_min = max(omega_min, -max_steering_limit)
                omega_max = min(omega_max, max_steering_limit)
        
        # Window from motion model
        v_lower = max(v_min, current_v - self.max_accel * self.dt)
        v_upper = min(v_max, current_v + self.max_accel * self.dt)
        
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
        
    def _calc_speed_cost(self, v, w=0.0):
        """Calculate speed cost using the reference implementation approach."""
        # For forward motion, prefer speeds closer to max_speed
        if v < 0.5:
            return float('inf')  # Strongly penalize low or negative speeds
            
        # Use a cost function that heavily favors higher speeds
        # Lower cost means better (preferred) velocity
        # We want to strongly prefer higher velocities
        speed_cost = 1.0 - (v / self.max_speed)
        
        # Add penalty for high angular velocity
        yaw_rate_cost = abs(w) / self.max_yaw_rate
        
        # Apply exponential weighting to make higher speeds much more preferable
        return speed_cost * speed_cost + yaw_rate_cost * 0.5  # Square the cost to make the preference for high speeds stronger
        
    def _calc_obstacle_cost(self, trajectory):
        """Calculate cost based on proximity to obstacles."""
        if self.map_data is None:
            return float('inf')
            
        obstacle_cost = 0.0
        min_dist = float('inf')
        points_outside_lane = 0
        black_obstacles_on_lane = 0
        total_points = len(trajectory)
        
        for i in range(len(trajectory)):
            x, y = trajectory[i][0], trajectory[i][1]
            map_x, map_y = self._world_to_map(x, y)
            
            # Skip if outside map boundaries
            if not (0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]):
                continue
                
            # Get the pixel value at this point
            pixel_value = self.map_data[map_y, map_x]
            
            # Check if point is on a lane (gray area)
            is_on_lane = self.lane_threshold_min <= pixel_value <= self.lane_threshold_max
            
            # Check if point is outside lane
            if not is_on_lane:
                points_outside_lane += 1
                
            # Check for black obstacles on lane
            if is_on_lane:
                # Count black pixels in the surrounding area
                black_pixels_count = 0
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        check_x, check_y = map_x + dx, map_y + dy
                        if (0 <= check_x < self.map_data.shape[1] and 
                            0 <= check_y < self.map_data.shape[0]):
                            check_value = self.map_data[check_y, check_x]
                            if check_value < self.obstacle_threshold:
                                black_pixels_count += 1
                
                # If we find enough black pixels on a lane, consider it an obstacle
                if black_pixels_count >= 3:
                    black_obstacles_on_lane += 1
                    # Immediate high cost for trajectories with black obstacles on lane
                    return float('inf')
                
            # Calculate minimum distance to any obstacle
            for dy in range(-10, 11):
                for dx in range(-10, 11):
                    check_x, check_y = map_x + dx, map_y + dy
                    if (0 <= check_x < self.map_data.shape[1] and
                        0 <= check_y < self.map_data.shape[0]):
                        check_value = self.map_data[check_y, check_x]
                        # Only consider black pixels as obstacles
                        if check_value < self.obstacle_threshold:
                            dist = np.hypot(dx, dy)
                            if dist < min_dist:
                                min_dist = dist
        
        # Penalize trajectories with too many points outside the lane
        lane_penalty = 0.0
        if total_points > 0:
            outside_lane_ratio = points_outside_lane / total_points
            if outside_lane_ratio > 0.5:  # More than 50% outside lane
                lane_penalty = 2.0 * outside_lane_ratio
        
        # Distance cost (higher when closer to obstacles)
        if min_dist < float('inf'):
            obstacle_cost = 1.0 / (min_dist + 0.001)
        
        # Add a severe penalty for trajectories with black obstacles on lane
        if black_obstacles_on_lane > 0:
            obstacle_cost += 10.0 * black_obstacles_on_lane
        
        return obstacle_cost + lane_penalty

    def _is_trajectory_safe(self, trajectory):
        """Check if a trajectory is safe (no collisions and mostly within lane)."""
        if self.map_data is None:
            return False

        points_outside_lane = 0
        total_points = len(trajectory)
        black_obstacles_on_lane = 0
        
        for x, y in trajectory:
            map_x, map_y = self._world_to_map(x, y)
            
            # Check map bounds
            if not (0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]):
                return False
            
            # Get the pixel value at this point
            pixel_value = self.map_data[map_y, map_x]
            
            # Check if point is on a lane (gray area)
            is_on_lane = self.lane_threshold_min <= pixel_value <= self.lane_threshold_max
            
            # Count points outside lane
            if not is_on_lane:
                points_outside_lane += 1
            
            # Check for black obstacles on lane
            if is_on_lane:
                # Count black pixels in the surrounding area
                black_pixels_count = 0
                for dy in range(-2, 3):
                    for dx in range(-2, 3):
                        check_x, check_y = map_x + dx, map_y + dy
                        if (0 <= check_x < self.map_data.shape[1] and 
                            0 <= check_y < self.map_data.shape[0]):
                            check_value = self.map_data[check_y, check_x]
                            if check_value < self.obstacle_threshold:
                                black_pixels_count += 1
                
                # If we find enough black pixels on a lane, consider it unsafe
                if black_pixels_count >= 3:
                    black_obstacles_on_lane += 1
                    # Trajectory is unsafe if we find black obstacles on lane
                    if black_obstacles_on_lane > 0:
                        return False

        # Reject if more than 50% of points are outside lane
        return (points_outside_lane / total_points) <= 0.5

    def _calc_goal_cost(self, trajectory, gx, gy):
        """Calculate goal cost for a trajectory."""
        if not trajectory:
            return float('inf')

        # Get end position and orientation
        end_x, end_y = trajectory[-1][0], trajectory[-1][1]
        if len(trajectory) > 1:
            dx = end_x - trajectory[-2][0]
            dy = end_y - trajectory[-2][1]
            end_yaw = np.arctan2(dy, dx)
        else:
            # If only one point, use current vehicle orientation
            with self.state_lock:
                end_yaw = self.current_theta

        # Calculate distance to goal
        goal_x, goal_y = gx, gy
        dist_to_goal = np.hypot(goal_x - end_x, goal_y - end_y)

        # Calculate heading error to goal
        goal_heading = np.arctan2(goal_y - end_y, goal_x - end_x)
        heading_error = abs(self._normalize_angle(goal_heading - end_yaw))

        # Combine distance and heading costs
        distance_cost = dist_to_goal * self.to_goal_cost_gain
        heading_cost = heading_error * 2.0  # Weight for heading error

        return distance_cost + heading_cost

    def _is_near_goal(self, threshold=3.0):
        """Check if we're near the goal."""
        if not self.current_goal:
            return False
        current_x, current_y = self.current_pose[0], self.current_pose[1]
        goal_x, goal_y = self.current_goal
        return np.hypot(goal_x - current_x, goal_y - current_y) < threshold

    def _is_near_obstacle(self, threshold=5.0):
        """Check if there are obstacles nearby."""
        if self.map_data is None:
            return False
            
        current_x, current_y = self.current_pose[0], self.current_pose[1]
        map_x, map_y = self._world_to_map(current_x, current_y)
        
        for dy in range(-int(threshold), int(threshold) + 1):
            for dx in range(-int(threshold), int(threshold) + 1):
                check_x, check_y = map_x + dx, map_y + dy
                if (0 <= check_x < self.map_data.shape[1] and 
                    0 <= check_y < self.map_data.shape[0]):
                    check_value = self.map_data[check_y, check_x]
                    if check_value > self.obstacle_threshold:
                        return True
        return False
    
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
        """Enhanced DWA control with velocity-aware heading alignment."""
        if best_u is None or len(best_u) != 2:
            return None

        # Get current target waypoint
        closest_idx = self.find_closest_waypoint(vehicle_pose)
        if closest_idx is None:
            return None
            
        target_idx, target_waypoint = self.get_target_waypoint(closest_idx, vehicle_pose)
        if target_waypoint is None:
            return None

        # Calculate desired heading to waypoint
        dx = target_waypoint['x'] - vehicle_pose.position.x
        dy = target_waypoint['y'] - vehicle_pose.position.y
        desired_heading = np.arctan2(dy, dx)
        
        # Current heading
        quat = vehicle_pose.orientation
        _, _, current_heading = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Calculate heading error
        heading_error = self._normalize_angle(desired_heading - current_heading)
        
        # Use velocity feedback from the velocity listener
        # Note: self.current_velocity is updated by the velocity_callback method
        current_speed = max(self.current_velocity, 0.1)  # m/s, ensure non-zero
        
        # Dynamic steering gain based on velocity
        # Higher gain at lower speeds, lower gain at higher speeds
        base_gain = 2.0
        speed_factor = 16.67 / current_speed  # 16.67 m/s = 60 km/h reference speed
        Kp = base_gain * min(speed_factor, 2.0)  # Cap the gain increase at low speeds
        
        # Calculate steering command
        # Note: Inverting the sign because right steer corresponds to left on the map
        omega = -Kp * heading_error
        
        # Additional damping at high speeds
        if current_speed > 16.67:  # Above 60 km/h
            damping_factor = 0.7 * (current_speed - 16.67) / 16.67
            omega *= (1.0 - damping_factor)
        
        # Limit omega based on speed
        # Lower max yaw rate at higher speeds for stability
        speed_scaled_max_yaw = self.max_yaw_rate * (1.0 - min(current_speed / 27.78, 0.7))
        omega = max(min(omega, speed_scaled_max_yaw), -speed_scaled_max_yaw)
        
        # Initialize velocity
        v_opt = best_u[0]
        
        # Check if we should stop for obstacles
        if stop_for_obstacles:
            # Find the closest obstacle distance
            closest_obstacle = float('inf')
            obstacle_count = 0
            
            if self.lane_obstacle_positions:
                vehicle_x, vehicle_y = vehicle_pose.position.x, vehicle_pose.position.y
                
                for obs_x, obs_y in self.lane_obstacle_positions:
                    # Calculate distance to obstacle
                    dist = math.sqrt((obs_x - vehicle_x) ** 2 + (obs_y - vehicle_y) ** 2)
                    obstacle_count += 1
                    closest_obstacle = min(closest_obstacle, dist)
                    
                    # Log obstacle information
                    map_x, map_y = self._world_to_map(obs_x, obs_y)
                    if (0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]):
                        pixel_value = self.map_data[map_y, map_x]
                        self.get_logger().debug(f"Obstacle at ({obs_x:.2f}, {obs_y:.2f}), distance: {dist:.2f}m, pixel value: {pixel_value}")
            
            # Apply emergency stop based on obstacle distance
            if closest_obstacle < self.min_obstacle_distance and obstacle_count > 0:
                # Stop completely when obstacles are detected within min_obstacle_distance
                v_opt = 0.0  # Full stop
                self.get_logger().warn(f"EMERGENCY STOP: {obstacle_count} obstacles detected at {closest_obstacle:.2f}m")
                
                # Reduce steering when stopped to prevent oscillations
                if closest_obstacle < self.min_obstacle_distance * 0.5:
                    omega = 0.0  # No steering when very close to obstacles
                    self.get_logger().warn("Steering disabled due to very close obstacle")
                else:
                    # Reduce steering when close to obstacles but not extremely close
                    omega *= 0.5  # Reduce steering by half
            elif closest_obstacle < self.emergency_brake_distance and obstacle_count > 0:
                # Slow down when obstacles are within emergency_brake_distance but beyond min_obstacle_distance
                slow_factor = (closest_obstacle - self.min_obstacle_distance) / (self.emergency_brake_distance - self.min_obstacle_distance)
                v_opt = best_u[0] * max(0.2, slow_factor)  # Ensure at least 20% of speed
                self.get_logger().info(f"Slowing down: Obstacle at {closest_obstacle:.2f}m, speed reduced to {v_opt:.2f} m/s")
            
            # Publish obstacle detected message if we have obstacles
            if obstacle_count > 0:
                obstacle_msg = Bool()
                obstacle_msg.data = True
                self.obstacle_detected_publisher.publish(obstacle_msg)
        else:
            # No obstacles, use a reasonable forward velocity
            # Adjust speed based on heading error
            heading_factor = 1.0 - (abs(heading_error) / np.pi)
            
            if current_speed > 16.67:  # Above 60 km/h
                heading_factor = heading_factor ** 2  # Squared for stronger effect at high speeds
            
            v_opt = best_u[0] * max(0.4, heading_factor)  # Maintain at least 40% of speed
            
            # Force a minimum forward velocity to get the vehicle moving
            v_opt = max(2.0, v_opt)
            
            # Publish normal message
            obstacle_msg = Bool()
            obstacle_msg.data = False
            self.obstacle_detected_publisher.publish(obstacle_msg)
            
            self.get_logger().debug(f"Moving forward at velocity: {v_opt:.2f} m/s, heading error: {heading_error:.2f} rad")
        
        # Create and return control message
        control_msg = Twist()
        control_msg.linear.x = v_opt
        control_msg.angular.z = omega
        
        # Update current velocity and angular velocity
        self.current_v = v_opt
        self.current_omega = omega

        return control_msg
    
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
    
    def visualize_lane_obstacles(self):
        """Visualize obstacles detected on the path."""
        if not self.lane_obstacle_positions or len(self.lane_obstacle_positions) == 0:
            return
            
        # Create a marker array for visualization
        marker_array = MarkerArray()
        
        # Create markers for each obstacle
        for i, (x, y) in enumerate(self.lane_obstacle_positions):
            # Check the obstacle type
            map_x, map_y = self._world_to_map(x, y)
            obstacle_type = "UNKNOWN"
            obstacle_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Default red
            
            if (0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]):
                pixel_value = self.map_data[map_y, map_x]
                
                # Determine obstacle type based on pixel value
                if pixel_value >= 90:
                    obstacle_type = "HIGH COST OBSTACLE"
                    obstacle_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
                elif pixel_value < self.obstacle_threshold:
                    # Check if it's on a lane
                    is_on_lane = self.lane_threshold_min <= pixel_value <= self.lane_threshold_max
                    if is_on_lane:
                        obstacle_type = "BLACK OBSTACLE ON LANE"
                        obstacle_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
                    else:
                        obstacle_type = "BLACK OBSTACLE"
                        obstacle_color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.8)  # Purple
                else:
                    # Check surrounding area for high cost pixels
                    high_cost_pixels = 0
                    for dy in range(-3, 4):
                        for dx in range(-3, 4):
                            check_x, check_y = map_x + dx, map_y + dy
                            if (0 <= check_x < self.map_data.shape[1] and 
                                0 <= check_y < self.map_data.shape[0]):
                                check_value = self.map_data[check_y, check_x]
                                if check_value > 60:
                                    high_cost_pixels += 1
                    
                    if high_cost_pixels >= 5:
                        obstacle_type = "PROXIMITY OBSTACLE"
                        obstacle_color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # Orange
            
            # Calculate distance from vehicle position to obstacle
            vehicle_x, vehicle_y = self.current_pose[0], self.current_pose[1]
            distance = math.sqrt((x - vehicle_x) ** 2 + (y - vehicle_y) ** 2)
            
            # Create sphere marker
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.map_frame_id
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "obstacles"
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            
            # Set position
            sphere_marker.pose.position.x = x
            sphere_marker.pose.position.y = y
            sphere_marker.pose.position.z = 0.5  # Slightly above ground
            
            # Set orientation (identity quaternion)
            sphere_marker.pose.orientation.w = 1.0
            
            # Set scale - make closer obstacles appear larger
            scale_factor = max(0.5, min(2.0, 3.0 / (distance + 0.1)))
            sphere_marker.scale.x = 0.5 * scale_factor
            sphere_marker.scale.y = 0.5 * scale_factor
            sphere_marker.scale.z = 0.5 * scale_factor
            
            # Set color
            sphere_marker.color = obstacle_color
            
            # Set lifetime
            lifetime_sec = 0.5
            sphere_marker.lifetime = Duration(sec=int(lifetime_sec), 
                                            nanosec=int((lifetime_sec % 1) * 1e9))
            
            marker_array.markers.append(sphere_marker)
            
            # Add text marker
            text_marker = Marker()
            text_marker.header.frame_id = self.map_frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "obstacle_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # Set position (slightly above the sphere)
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y
            text_marker.pose.position.z = 1.0
            
            # Set text
            text_marker.text = f"{obstacle_type}\nDist: {distance:.2f}m"
            
            # Set scale
            text_marker.scale.z = 0.5  # Text height
            
            # Set color (white text)
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.8
            
            # Set lifetime
            text_marker.lifetime = Duration(sec=int(lifetime_sec), 
                                        nanosec=int((lifetime_sec % 1) * 1e9))
                        
            marker_array.markers.append(text_marker)
        
        # Publish marker array
        self.obstacle_viz_pub.publish(marker_array)
    
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
    
    
        
    def radar_callback(self, msg):
        """Process radar detection data"""
        if not self.use_sensor_obstacles:
            return
            
        # Process radar detections
        self.radar_obstacles = []
        for detection in msg.detections:
            x = detection.pose.position.x
            y = detection.pose.position.y
            vel_x = detection.velocity.x
            vel_y = detection.velocity.y
            velocity = math.sqrt(vel_x**2 + vel_y**2)
            
            # Only track moving obstacles above velocity threshold
            if velocity > self.obstacle_velocity_threshold:
                self.radar_obstacles.append((x, y, velocity))
                
        self.update_dynamic_obstacles()
        
    def obstacle_map_callback(self, msg):
        """Process obstacle map data"""
        if not self.use_sensor_obstacles:
            return
            
        self.obstacle_map = msg
        self.update_obstacle_grid()
        
    def obstacle_detection_callback(self, msg):
        """Process obstacle detection data"""
        if not self.use_sensor_obstacles:
            return
            
        # Update detected obstacles
        self.detected_obstacles = []
        for obstacle in msg.obstacles:
            x = obstacle.pose.position.x
            y = obstacle.pose.position.y
            obstacle_class = obstacle.classification
            self.detected_obstacles.append((x, y, obstacle_class))
            
        self.last_obstacle_time = self.get_clock().now()
        
    def cluster_and_track_obstacles(self):
        """Cluster point cloud data into obstacles and track them"""
        if len(self.lidar_obstacles) < self.min_obstacle_points:
            return
            
        # Use DBSCAN clustering
        clusters = []
        points = np.array(self.lidar_obstacles)
        db = DBSCAN(eps=self.obstacle_cluster_tolerance, min_samples=self.min_obstacle_points).fit(points)
        labels = db.labels_
        
        # Process clusters
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:  # Skip noise
                continue
                
            cluster_points = points[labels == label]
            center = np.mean(cluster_points, axis=0)
            clusters.append(center)
            
        # Update obstacle tracking
        self.update_obstacle_tracking(clusters)
        
    def update_dynamic_obstacles(self):
        """Update dynamic obstacle tracking"""
        if not self.dynamic_obstacle_tracking:
            return
            
        current_time = self.get_clock().now()
        
        # Predict obstacle positions based on velocity
        for obstacle in self.radar_obstacles:
            x, y, velocity = obstacle
            dt = (current_time - self.last_obstacle_time).nanoseconds / 1e9
            
            # Simple linear prediction
            pred_x = x + velocity * dt * math.cos(math.atan2(y, x))
            pred_y = y + velocity * dt * math.sin(math.atan2(y, x))
            
            # Add predicted position to tracking
            self.update_obstacle_tracking([(pred_x, pred_y)])
            
    def update_obstacle_grid(self):
        """Update the obstacle grid with sensor data"""
        if self.obstacle_map is None:
            return
            
        # Combine all obstacle sources
        all_obstacles = []
        all_obstacles.extend(self.lidar_obstacles)
        all_obstacles.extend([(x, y) for x, y, _ in self.radar_obstacles])
        all_obstacles.extend([(x, y) for x, y, _ in self.detected_obstacles])
        
        # Update grid cells
        for x, y in all_obstacles:
            # Convert to grid coordinates
            grid_x = int((x - self.obstacle_map.info.origin.position.x) / self.obstacle_map.info.resolution)
            grid_y = int((y - self.obstacle_map.info.origin.position.y) / self.obstacle_map.info.resolution)
            
            # Check bounds
            if (0 <= grid_x < self.obstacle_map.info.width and 
                0 <= grid_y < self.obstacle_map.info.height):
                # Mark cell as obstacle
                index = grid_y * self.obstacle_map.info.width + grid_x
                self.obstacle_map.data[index] = 100
                
        # Inflate obstacles
        self.inflate_obstacles()
        
    def inflate_obstacles(self):
        """Inflate obstacles in the grid"""
        if self.obstacle_map is None:
            return
            
        # Calculate inflation radius in grid cells
        inflation_cells = int(self.obstacle_inflation_radius / self.obstacle_map.info.resolution)
        
        # Create a copy of the grid
        inflated_grid = list(self.obstacle_map.data)
        
        # Inflate each obstacle
        for y in range(self.obstacle_map.info.height):
            for x in range(self.obstacle_map.info.width):
                index = y * self.obstacle_map.info.width + x
                if self.obstacle_map.data[index] == 100:  # If cell is an obstacle
                    # Inflate in a circle around the obstacle
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        for dx in range(-inflation_cells, inflation_cells + 1):
                            # Check if within inflation radius
                            if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                                new_x = x + dx
                                new_y = y + dy
                                if (0 <= new_x < self.obstacle_map.info.width and 
                                    0 <= new_y < self.obstacle_map.info.height):
                                    new_index = new_y * self.obstacle_map.info.width + new_x
                                    inflated_grid[new_index] = 100
                                    
        # Update the grid with inflated obstacles
        self.obstacle_map.data = inflated_grid
        
    def update_obstacle_tracking(self, obstacles):
        """Update obstacle tracking with new detections"""
        current_time = self.get_clock().now()
        
        # Remove old obstacles
        self.detected_obstacles = [
            obs for obs in self.detected_obstacles
            if (current_time - self.last_obstacle_time).nanoseconds / 1e9 < self.obstacle_memory_time
        ]
        
        # Add new obstacles
        for obs in obstacles:
            if isinstance(obs, tuple):
                x, y = obs
                self.detected_obstacles.append((x, y, "unknown"))
            
        # Publish visualization
        self.publish_obstacle_visualization()
        
    def publish_obstacle_visualization(self):
        """Publish visualization markers for detected obstacles"""
        marker_array = MarkerArray()
        
        # Create markers for different types of obstacles
        for i, (x, y, obs_type) in enumerate(self.detected_obstacles):
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            
            marker.scale.x = self.obstacle_inflation_radius * 2
            marker.scale.y = self.obstacle_inflation_radius * 2
            marker.scale.z = 1.0
            
            # Color based on obstacle type
            if obs_type == "dynamic":
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
            marker.color.a = 0.6
            
            marker_array.markers.append(marker)
            
        self.obstacle_viz_pub.publish(marker_array)

    def _calc_path_following_cost(self, trajectory):
        """Calculate cost based on how well the trajectory follows the path."""
        if not trajectory or not self.waypoints:
            return 0.0  # No cost if no trajectory or waypoints
            
        # Get end position of trajectory
        end_x, end_y = trajectory[-1][0], trajectory[-1][1]
        
        # Find closest waypoint to end position
        min_dist = float('inf')
        closest_wp = None
        
        for wp in self.waypoints:
            dist = np.hypot(wp['x'] - end_x, wp['y'] - end_y)
            if dist < min_dist:
                min_dist = dist
                closest_wp = wp
                
        # If no waypoint found, return no cost
        if closest_wp is None:
            return 0.0
            
        # Calculate cost based on distance to closest waypoint
        # Lower distance means better path following
        path_cost = min_dist
        
        return path_cost

    def _world_to_map(self, x, y):
        """Convert world coordinates to map coordinates."""
        map_x = int((x - self.map_origin_x) / self.map_resolution)
        map_y = int((y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y


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
