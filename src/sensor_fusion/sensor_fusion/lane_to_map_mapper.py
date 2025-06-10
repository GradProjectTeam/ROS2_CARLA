#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA, Float32
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
import tf2_geometry_msgs
from collections import deque
import time

class LaneToMapMapper(Node):
    """
    Node that subscribes to lane detections from the camera and maps them onto a 2D map.
    Also maintains lane memory for continuous tracking when lanes are intermittent.
    """
    def __init__(self):
        super().__init__('lane_to_map_mapper')
        
        # Declare parameters
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('lane_projection_distance', 20.0)
        self.declare_parameter('lane_width', 3.5)
        self.declare_parameter('processed_lanes_topic', '/carla/camera/processed_lanes')
        self.declare_parameter('raw_image_topic', '/carla/camera/rgb/image_raw')
        self.declare_parameter('imu_heading_topic', '/vehicle/heading')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('fallback_to_identity_transform', True)
        self.declare_parameter('map_resolution', 0.1)  # meters per pixel
        self.declare_parameter('map_width', 50.0)      # meters
        self.declare_parameter('map_height', 50.0)     # meters
        self.declare_parameter('lane_memory_timeout', 5.0)  # seconds to remember lanes
        self.declare_parameter('min_lane_points', 5)   # minimum points to consider a valid lane
        self.declare_parameter('lane_confidence_threshold', 0.5)  # minimum confidence to use detected lanes
        
        # Get parameters
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.lane_projection_distance = self.get_parameter('lane_projection_distance').value
        self.lane_width = self.get_parameter('lane_width').value
        self.processed_lanes_topic = self.get_parameter('processed_lanes_topic').value
        self.raw_image_topic = self.get_parameter('raw_image_topic').value
        self.imu_heading_topic = self.get_parameter('imu_heading_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.fallback_to_identity_transform = self.get_parameter('fallback_to_identity_transform').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.lane_memory_timeout = self.get_parameter('lane_memory_timeout').value
        self.min_lane_points = self.get_parameter('min_lane_points').value
        self.lane_confidence_threshold = self.get_parameter('lane_confidence_threshold').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize a static transform broadcaster to publish fallback transforms if needed
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Flag to track if we've published fallback transforms
        self.published_fallback_transforms = False
        
        # Initialize lane data
        self.left_lane_points = []
        self.right_lane_points = []
        self.lane_center_points = []
        self.last_lane_update_time = None
        self.processed_lane_image = None
        
        # Lane memory - store world-coordinate points
        self.memory_left_lane_points = []
        self.memory_right_lane_points = []
        self.memory_center_lane_points = []
        self.last_memory_update_time = self.get_clock().now()
        
        # IMU data
        self.current_heading = 0.0
        self.previous_heading = 0.0
        self.heading_change = 0.0
        self.last_imu_update_time = None
        
        # Lane detection quality metrics
        self.lane_detection_confidence = 0.0  # 0.0-1.0
        self.use_memory_lanes = False
        
        # Create publishers
        self.lane_marker_publisher = self.create_publisher(
            MarkerArray,
            '/map_lane_markers',
            10
        )
        
        self.lane_grid_publisher = self.create_publisher(
            OccupancyGrid,
            '/lane_grid',
            10
        )
        
        # Create subscribers
        self.processed_lanes_subscription = self.create_subscription(
            Image,
            self.processed_lanes_topic,
            self.processed_lanes_callback,
            10
        )
        
        # Subscribe to IMU heading data
        self.imu_heading_subscription = self.create_subscription(
            Float32,
            self.imu_heading_topic,
            self.imu_heading_callback,
            10
        )
        
        # Create timer for publishing lane markers
        self.create_timer(1.0/self.publish_rate, self.publish_lane_markers)
        
        # Create timer for checking and publishing fallback transforms if needed
        self.create_timer(2.0, self.check_and_publish_fallback_transforms)
        
        # Create timer for publishing lane grid
        self.create_timer(1.0/self.publish_rate, self.publish_lane_grid)
        
        self.get_logger().info('Lane to Map Mapper node initialized with lane memory')
    
    def imu_heading_callback(self, msg):
        """Process IMU heading data"""
        # Store previous heading for calculating change
        self.previous_heading = self.current_heading
        self.current_heading = msg.data
        
        # Calculate heading change since last update
        self.heading_change = self.current_heading - self.previous_heading
        if self.heading_change > 180.0:
            self.heading_change -= 360.0
        elif self.heading_change < -180.0:
            self.heading_change += 360.0
            
        # If we have memory lanes, update their positions based on heading change
        if self.memory_left_lane_points or self.memory_right_lane_points:
            self.update_memory_lanes_with_heading()
            
        self.last_imu_update_time = self.get_clock().now()
        
    def update_memory_lanes_with_heading(self):
        """Update the positions of memory lanes based on vehicle heading change"""
        if abs(self.heading_change) < 0.1:
            return  # Skip tiny heading changes
            
        # Convert heading change to radians
        heading_change_rad = math.radians(self.heading_change)
        
        # Rotation matrix for heading change
        cos_theta = math.cos(heading_change_rad)
        sin_theta = math.sin(heading_change_rad)
        
        # Update memory lane points by rotating around vehicle position
        # For simplicity, we assume vehicle is at (0,0) in the map frame
        # and transform lane points relative to that
        
        # Helper function to rotate a point
        def rotate_point(point):
            x, y, z = point
            # Rotate around origin
            new_x = x * cos_theta - y * sin_theta
            new_y = x * sin_theta + y * cos_theta
            return (new_x, new_y, z)
        
        # Rotate each set of memory lane points
        if self.memory_left_lane_points:
            self.memory_left_lane_points = [rotate_point(p) for p in self.memory_left_lane_points]
            
        if self.memory_right_lane_points:
            self.memory_right_lane_points = [rotate_point(p) for p in self.memory_right_lane_points]
            
        if self.memory_center_lane_points:
            self.memory_center_lane_points = [rotate_point(p) for p in self.memory_center_lane_points]
    
    def check_and_publish_fallback_transforms(self):
        """Check if transforms exist, and publish fallback transforms if needed and enabled"""
        if not self.fallback_to_identity_transform or self.published_fallback_transforms:
            return
            
        try:
            # Try to get the transform
            self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.camera_frame_id,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Transform not available, publishing fallback transforms: {e}')
            
            # Publish map -> base_link transform
            self.publish_static_transform(
                self.map_frame_id,
                self.base_frame_id,
                [0.0, 0.0, 0.0],  # x, y, z
                [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)
            )
            
            # Publish base_link -> camera_link transform
            self.publish_static_transform(
                self.base_frame_id,
                self.camera_frame_id,
                [2.0, 0.0, 1.5],  # x, y, z - camera is 2m in front and 1.5m up from base
                [0.0, 0.0, 0.0, 1.0]  # quaternion (x, y, z, w)
            )
            
            self.published_fallback_transforms = True
            self.get_logger().info('Published fallback transforms')
    
    def publish_static_transform(self, parent_frame, child_frame, translation, rotation):
        """Publish a static transform"""
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = parent_frame
        transform_msg.child_frame_id = child_frame
        
        transform_msg.transform.translation.x = translation[0]
        transform_msg.transform.translation.y = translation[1]
        transform_msg.transform.translation.z = translation[2]
        
        transform_msg.transform.rotation.x = rotation[0]
        transform_msg.transform.rotation.y = rotation[1]
        transform_msg.transform.rotation.z = rotation[2]
        transform_msg.transform.rotation.w = rotation[3]
        
        self.tf_static_broadcaster.sendTransform(transform_msg)
    
    def processed_lanes_callback(self, msg):
        """Process lane detections from the camera"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Store the processed lane image for grid generation
            self.processed_lane_image = cv_image
            
            # Extract lane information from the image
            self.extract_lane_points(cv_image)
            
            # Calculate lane detection confidence based on the number of points detected
            left_confidence = min(1.0, len(self.left_lane_points) / 20.0)
            right_confidence = min(1.0, len(self.right_lane_points) / 20.0)
            self.lane_detection_confidence = (left_confidence + right_confidence) / 2.0
            
            # If we have only one lane line detected, estimate the other based on lane width
            if len(self.left_lane_points) >= self.min_lane_points and len(self.right_lane_points) < self.min_lane_points:
                self.estimate_missing_lane_line('right')
                self.get_logger().debug('Estimated missing right lane from left lane')
                
            elif len(self.right_lane_points) >= self.min_lane_points and len(self.left_lane_points) < self.min_lane_points:
                self.estimate_missing_lane_line('left')
                self.get_logger().debug('Estimated missing left lane from right lane')
            
            # After potential estimation, recalculate center points
            if self.left_lane_points and self.right_lane_points:
                self.calculate_lane_center()
            
            # Update memory lanes if detection confidence is good
            if self.lane_detection_confidence >= self.lane_confidence_threshold:
                self.update_memory_lanes()
                self.last_memory_update_time = self.get_clock().now()
                self.use_memory_lanes = False
            else:
                # Use memory lanes if current detection is poor
                now = self.get_clock().now()
                memory_age = (now - self.last_memory_update_time).nanoseconds / 1e9
                if memory_age < self.lane_memory_timeout:
                    self.use_memory_lanes = True
                    self.get_logger().debug('Using memory lanes due to low detection confidence')
                else:
                    # Memory is too old, but detection is poor - try best effort
                    self.use_memory_lanes = False
                    self.get_logger().debug('Detection poor and memory expired - using best effort')
            
            # Store time of last update
            self.last_lane_update_time = msg.header.stamp
            
        except Exception as e:
            self.get_logger().error(f'Error processing lane image: {e}')
    
    def estimate_missing_lane_line(self, side_to_estimate):
        """
        Estimate a missing lane line based on the other lane line and expected lane width.
        
        Args:
            side_to_estimate: 'left' or 'right' indicating which lane line to estimate
        """
        if side_to_estimate == 'left' and len(self.right_lane_points) >= self.min_lane_points:
            # Estimate left lane based on right lane
            estimated_left_points = []
            
            # Sort right points by y-coordinate (bottom to top of image)
            sorted_right_points = sorted(self.right_lane_points, key=lambda p: -p[1])
            
            for right_x, right_y in sorted_right_points:
                # Estimate left point based on expected lane width in pixels
                # Lane width in pixels depends on the y-coordinate (perspective)
                distance_factor = 1.0 - (right_y / 480.0)
                # Approximate lane width in pixels at this y-coordinate
                # The 320 is half the image width (640/2) and 0.8 is a scaling factor for perspective
                lane_width_pixels = int(0.8 * 320 * self.lane_width / (self.lane_projection_distance * distance_factor))
                left_x = right_x - lane_width_pixels
                estimated_left_points.append((left_x, right_y))
                
            self.left_lane_points = estimated_left_points
            
        elif side_to_estimate == 'right' and len(self.left_lane_points) >= self.min_lane_points:
            # Estimate right lane based on left lane
            estimated_right_points = []
            
            # Sort left points by y-coordinate (bottom to top of image)
            sorted_left_points = sorted(self.left_lane_points, key=lambda p: -p[1])
            
            for left_x, left_y in sorted_left_points:
                # Estimate right point based on expected lane width in pixels
                distance_factor = 1.0 - (left_y / 480.0)
                # Approximate lane width in pixels at this y-coordinate
                lane_width_pixels = int(0.8 * 320 * self.lane_width / (self.lane_projection_distance * distance_factor))
                right_x = left_x + lane_width_pixels
                estimated_right_points.append((right_x, left_y))
                
            self.right_lane_points = estimated_right_points

    def calculate_lane_center(self):
        """Calculate lane center points based on left and right lane points"""
        # Make sure we have both left and right lane points
        if not self.left_lane_points or not self.right_lane_points:
            self.lane_center_points = []
            return
            
        # Sort points by y-coordinate (bottom to top of image)
        left_points = sorted(self.left_lane_points, key=lambda p: -p[1])
        right_points = sorted(self.right_lane_points, key=lambda p: -p[1])
        
        # Take the first few points (closest to the car)
        left_points = left_points[:10]
        right_points = right_points[:10]
        
        # Match left and right points by y-coordinate
        center_points = []
        
        # Use the shorter list length
        max_points = min(len(left_points), len(right_points))
        
        for i in range(max_points):
            left_x, left_y = left_points[i]
            right_x, right_y = right_points[i]
            
            # Use the average y if they don't exactly match
            center_y = (left_y + right_y) // 2
            center_x = (left_x + right_x) // 2
            center_points.append((center_x, center_y))
        
        self.lane_center_points = center_points

    def extract_lane_points(self, image):
        """Extract lane points from the processed image"""
        # The processed image has green lines for detected lanes
        # We'll extract the green pixels to find the lane lines
        
        # Create a mask for green pixels (lanes are drawn in green)
        lower_green = np.array([0, 200, 0])
        upper_green = np.array([100, 255, 100])
        mask = cv2.inRange(image, lower_green, upper_green)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the left and right lane contours
        left_points = []
        right_points = []
        
        image_center_x = image.shape[1] // 2
        
        for contour in contours:
            # Calculate the centroid of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Classify as left or right lane based on position relative to image center
                if cx < image_center_x:
                    left_points.append((cx, cy))
                else:
                    right_points.append((cx, cy))
        
        # Store lane points
        self.left_lane_points = left_points
        self.right_lane_points = right_points
        
        # Calculate lane center points if both lanes are detected
        if left_points and right_points:
            self.calculate_lane_center()
        else:
            self.lane_center_points = []
    
    def pixel_to_3d_point(self, pixel_x, pixel_y, distance):
        """
        Convert a pixel coordinate to a 3D point in the camera frame.
        
        Args:
            pixel_x: x-coordinate in the image (horizontal, 0 to 640)
            pixel_y: y-coordinate in the image (vertical, 0 to 480)
            distance: distance from camera in meters
            
        Returns:
            (x, y, z) coordinates in the camera frame
        """
        # Convert from pixel coordinates to normalized device coordinates
        # Assuming 640x480 image and 90-degree horizontal FOV
        # Center of image is (320, 240)
        norm_x = (pixel_x - 320) / 320  # -1 to 1
        norm_y = (240 - pixel_y) / 240  # 1 to -1 (inverted y-axis)
        
        # Scale based on distance and FOV
        # For a 90-degree FOV, tan(45°) = 1
        # For different FOVs, adjust the scaling factor
        scaling_factor = 1.0  # Adjust based on actual camera FOV
        
        # Calculate 3D coordinates in camera frame
        # x is forward, y is right, z is down in camera coordinates
        x = distance
        y = norm_x * distance * scaling_factor
        z = norm_y * distance * scaling_factor
        
        return (x, y, z)
    
    def get_transform(self, target_frame, source_frame):
        """
        Get the transform between two frames, with fallback logic.
        Returns the transform or None if not available.
        """
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
        except Exception as e:
            # Only log the error at debug level to avoid spamming
            self.get_logger().debug(f'Failed to get transform from {source_frame} to {target_frame}: {e}')
            return None
    
    def create_identity_transform(self, target_frame, source_frame):
        """Create an identity transform between frames for fallback purposes"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = target_frame
        transform.child_frame_id = source_frame
        transform.transform.rotation.w = 1.0
        return transform
    
    def publish_lane_grid(self):
        """Publish the lane information as an occupancy grid"""
        # Only return if we have no data at all (no current detection and no memory)
        if (self.processed_lane_image is None and not self.use_memory_lanes and 
            not self.memory_left_lane_points and not self.memory_right_lane_points):
            return
            
        try:
            # Create occupancy grid message
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = self.map_frame_id
            
            # Set map metadata
            grid_msg.info.resolution = self.map_resolution
            grid_width_pixels = int(self.map_width / self.map_resolution)
            grid_height_pixels = int(self.map_height / self.map_resolution)
            grid_msg.info.width = grid_width_pixels
            grid_msg.info.height = grid_height_pixels
            
            # Set the origin of the map to be centered on the vehicle
            # with the vehicle at the bottom center of the map
            grid_msg.info.origin.position.x = -self.map_width / 2
            grid_msg.info.origin.position.y = -self.map_height / 2
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0
            
            # Create an empty grid
            grid_data = np.zeros(grid_width_pixels * grid_height_pixels, dtype=np.int8)
            
            # Get the transform from camera to map
            transform = self.get_transform(self.map_frame_id, self.camera_frame_id)
            
            # If transform not available and fallback not enabled, return
            if transform is None and not self.fallback_to_identity_transform:
                return
                
            # If transform not available but fallback is enabled, use identity transform
            if transform is None:
                transform = self.create_identity_transform(self.map_frame_id, self.camera_frame_id)
            
            # Decide which lane data to use for visualization
            use_memory = self.use_memory_lanes or (
                # Also use memory if current detection has very few points
                (len(self.left_lane_points) < self.min_lane_points and 
                 len(self.right_lane_points) < self.min_lane_points)
            )
            
            # Check if memory is still valid
            now = self.get_clock().now()
            memory_age = (now - self.last_memory_update_time).nanoseconds / 1e9
            memory_valid = memory_age < self.lane_memory_timeout
            
            if use_memory and memory_valid:
                # Use memory lanes (these need to be transformed to world coordinates)
                self.get_logger().debug('Using memory lanes for grid visualization')
                
                # Get memory lanes in world coordinates
                memory_left_world, memory_right_world, memory_center_world = self.get_memory_lane_world_coordinates()
                
                # Process left lane memory points
                for point in memory_left_world:
                    map_x, map_y, _ = point
                    
                    # Convert to grid coordinates
                    grid_x = int((map_x - grid_msg.info.origin.position.x) / self.map_resolution)
                    grid_y = int((map_y - grid_msg.info.origin.position.y) / self.map_resolution)
                    
                    # Check if the point is within grid bounds
                    if 0 <= grid_x < grid_width_pixels and 0 <= grid_y < grid_height_pixels:
                        # Draw a small circle around the point to make the lane visible
                        for dx in range(-2, 3):
                            for dy in range(-2, 3):
                                if dx*dx + dy*dy <= 4:  # Circle with radius 2
                                    x = grid_x + dx
                                    y = grid_y + dy
                                    if 0 <= x < grid_width_pixels and 0 <= y < grid_height_pixels:
                                        # Mark as lane (100% occupied)
                                        grid_index = y * grid_width_pixels + x
                                        grid_data[grid_index] = 100
                                        
                # Process right lane memory points
                for point in memory_right_world:
                    map_x, map_y, _ = point
                    
                    # Convert to grid coordinates
                    grid_x = int((map_x - grid_msg.info.origin.position.x) / self.map_resolution)
                    grid_y = int((map_y - grid_msg.info.origin.position.y) / self.map_resolution)
                    
                    # Check if the point is within grid bounds
                    if 0 <= grid_x < grid_width_pixels and 0 <= grid_y < grid_height_pixels:
                        # Draw a small circle around the point to make the lane visible
                        for dx in range(-2, 3):
                            for dy in range(-2, 3):
                                if dx*dx + dy*dy <= 4:  # Circle with radius 2
                                    x = grid_x + dx
                                    y = grid_y + dy
                                    if 0 <= x < grid_width_pixels and 0 <= y < grid_height_pixels:
                                        # Mark as lane (100% occupied)
                                        grid_index = y * grid_width_pixels + x
                                        grid_data[grid_index] = 100
                                        
                # Process center lane memory points
                for point in memory_center_world:
                    map_x, map_y, _ = point
                    
                    # Convert to grid coordinates
                    grid_x = int((map_x - grid_msg.info.origin.position.x) / self.map_resolution)
                    grid_y = int((map_y - grid_msg.info.origin.position.y) / self.map_resolution)
                    
                    # Check if the point is within grid bounds
                    if 0 <= grid_x < grid_width_pixels and 0 <= grid_y < grid_height_pixels:
                        # Draw a small circle around the point to make the lane visible
                        for dx in range(-2, 3):
                            for dy in range(-2, 3):
                                if dx*dx + dy*dy <= 4:  # Circle with radius 2
                                    x = grid_x + dx
                                    y = grid_y + dy
                                    if 0 <= x < grid_width_pixels and 0 <= y < grid_height_pixels:
                                        # Mark as lane (100% occupied)
                                        grid_index = y * grid_width_pixels + x
                                        grid_data[grid_index] = 100
            
            # Always process current lane detections if available (can combine with memory)
            if self.processed_lane_image is not None:
                # Mark lane points on the grid
                lane_points = []
                if self.left_lane_points:
                    lane_points.extend(self.left_lane_points)
                if self.right_lane_points:
                    lane_points.extend(self.right_lane_points)
                if self.lane_center_points:
                    lane_points.extend(self.lane_center_points)
                    
                for point in lane_points:
                    pixel_x, pixel_y = point
                    
                    # Calculate distance based on y position in image
                    # Lower in the image = closer to the camera
                    distance_factor = 1.0 - (pixel_y / 480.0)
                    distance = 2.0 + (self.lane_projection_distance * distance_factor)
                    
                    # Convert pixel to 3D point in camera frame
                    x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                    
                    # Create a point in the camera frame
                    p_cam = PoseStamped()
                    p_cam.header.frame_id = self.camera_frame_id
                    p_cam.header.stamp = self.get_clock().now().to_msg()
                    p_cam.pose.position.x = x_cam
                    p_cam.pose.position.y = y_cam
                    p_cam.pose.position.z = z_cam
                    p_cam.pose.orientation.w = 1.0
                    
                    try:
                        # Transform to map frame
                        if self.fallback_to_identity_transform and transform is not None:
                            # Manual transform when using fallback identity transform
                            map_x = transform.transform.translation.x + x_cam
                            map_y = transform.transform.translation.y + y_cam
                        else:
                            # Use tf2 transform if available
                            p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                            map_x = p_map.pose.position.x
                            map_y = p_map.pose.position.y
                        
                        # Convert to grid coordinates
                        grid_x = int((map_x - grid_msg.info.origin.position.x) / self.map_resolution)
                        grid_y = int((map_y - grid_msg.info.origin.position.y) / self.map_resolution)
                        
                        # Check if the point is within grid bounds
                        if 0 <= grid_x < grid_width_pixels and 0 <= grid_y < grid_height_pixels:
                            # Draw a small circle around the point to make the lane visible
                            for dx in range(-2, 3):
                                for dy in range(-2, 3):
                                    if dx*dx + dy*dy <= 4:  # Circle with radius 2
                                        x = grid_x + dx
                                        y = grid_y + dy
                                        if 0 <= x < grid_width_pixels and 0 <= y < grid_height_pixels:
                                            # Mark as lane (100% occupied)
                                            grid_index = y * grid_width_pixels + x
                                            grid_data[grid_index] = 100
                    except Exception as e:
                        self.get_logger().debug(f'Failed to transform point to grid: {e}')
            
            # Set the grid data
            grid_msg.data = grid_data.tolist()
            
            # Publish the grid
            self.lane_grid_publisher.publish(grid_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing lane grid: {e}')
    
    def publish_lane_markers(self):
        """Publish lane markers on the map"""
        # Only return if we have no data at all
        if (not self.left_lane_points and not self.right_lane_points and 
            not self.memory_left_lane_points and not self.memory_right_lane_points):
            return
            
        try:
            # Get the transform from camera to map
            transform = self.get_transform(self.map_frame_id, self.camera_frame_id)
            
            # If transform not available and fallback not enabled, return
            if transform is None and not self.fallback_to_identity_transform:
                return
                
            # If transform not available but fallback is enabled, use identity transform
            if transform is None:
                self.get_logger().debug('Using fallback identity transform for visualization')
                transform = self.create_identity_transform(self.map_frame_id, self.camera_frame_id)
                
            # Create marker array message
            marker_array = MarkerArray()
            now = self.get_clock().now().to_msg()
            
            # Decide which lane data to use for visualization
            use_memory = self.use_memory_lanes or (
                # Also use memory if current detection has very few points
                (len(self.left_lane_points) < self.min_lane_points and 
                 len(self.right_lane_points) < self.min_lane_points)
            )
            
            # Check if memory is still valid
            memory_age = (self.get_clock().now() - self.last_memory_update_time).nanoseconds / 1e9
            memory_valid = memory_age < self.lane_memory_timeout
            
            # Always visualize current detections if available
            if self.left_lane_points or self.right_lane_points:
                # Create markers for left lane
                if self.left_lane_points:
                    left_marker = Marker()
                    left_marker.header.frame_id = self.map_frame_id
                    left_marker.header.stamp = now
                    left_marker.ns = "map_left_lane"
                    left_marker.id = 0
                    left_marker.type = Marker.LINE_STRIP
                    left_marker.action = Marker.ADD
                    left_marker.pose.orientation.w = 1.0
                    left_marker.scale.x = 0.2  # Line width
                    left_marker.color.r = 1.0
                    left_marker.color.g = 1.0
                    left_marker.color.b = 0.0
                    left_marker.color.a = 1.0
                    left_marker.lifetime.sec = 0  # Persist until explicitly deleted
                    
                    # Transform lane points to map frame
                    for point in self.left_lane_points:
                        pixel_x, pixel_y = point
                        
                        # Calculate distance based on y position in image
                        # Lower in the image = closer to the camera
                        distance_factor = 1.0 - (pixel_y / 480.0)
                        distance = 2.0 + (self.lane_projection_distance * distance_factor)
                        
                        # Convert pixel to 3D point in camera frame
                        x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                        
                        # Create a point in the camera frame
                        p_cam = PoseStamped()
                        p_cam.header.frame_id = self.camera_frame_id
                        p_cam.header.stamp = now
                        p_cam.pose.position.x = x_cam
                        p_cam.pose.position.y = y_cam
                        p_cam.pose.position.z = 0.0  # Project to 2D plane
                        p_cam.pose.orientation.w = 1.0
                        
                        # Transform to map frame
                        try:
                            # Use tf_buffer if available, otherwise fall back to manual transform
                            if self.fallback_to_identity_transform and transform is not None:
                                # Manual transform when using fallback identity transform
                                p_map = PoseStamped()
                                p_map.header.frame_id = self.map_frame_id
                                p_map.header.stamp = now
                                p_map.pose.position.x = transform.transform.translation.x + x_cam
                                p_map.pose.position.y = transform.transform.translation.y + y_cam
                                p_map.pose.position.z = 0.0  # 2D map
                                p_map.pose.orientation.w = 1.0
                            else:
                                # Use tf2 transform if available
                                p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                                p_map.pose.position.z = 0.0  # Ensure 2D projection
                                
                            left_marker.points.append(p_map.pose.position)
                        except Exception as e:
                            self.get_logger().debug(f'Failed to transform point: {e}')
                        
                    marker_array.markers.append(left_marker)
                
                # Create markers for right lane
                if self.right_lane_points:
                    right_marker = Marker()
                    right_marker.header.frame_id = self.map_frame_id
                    right_marker.header.stamp = now
                    right_marker.ns = "map_right_lane"
                    right_marker.id = 1
                    right_marker.type = Marker.LINE_STRIP
                    right_marker.action = Marker.ADD
                    right_marker.pose.orientation.w = 1.0
                    right_marker.scale.x = 0.2  # Line width
                    right_marker.color.r = 1.0
                    right_marker.color.g = 1.0
                    right_marker.color.b = 0.0
                    right_marker.color.a = 1.0
                    right_marker.lifetime.sec = 0  # Persist until explicitly deleted
                    
                    # Transform lane points to map frame
                    for point in self.right_lane_points:
                        pixel_x, pixel_y = point
                        
                        # Calculate distance based on y position in image
                        # Lower in the image = closer to the camera
                        distance_factor = 1.0 - (pixel_y / 480.0)
                        distance = 2.0 + (self.lane_projection_distance * distance_factor)
                        
                        # Convert pixel to 3D point in camera frame
                        x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                        
                        # Create a point in the camera frame
                        p_cam = PoseStamped()
                        p_cam.header.frame_id = self.camera_frame_id
                        p_cam.header.stamp = now
                        p_cam.pose.position.x = x_cam
                        p_cam.pose.position.y = y_cam
                        p_cam.pose.position.z = 0.0  # Project to 2D plane
                        p_cam.pose.orientation.w = 1.0
                        
                        # Transform to map frame
                        try:
                            # Use tf_buffer if available, otherwise fall back to manual transform
                            if self.fallback_to_identity_transform and transform is not None:
                                # Manual transform when using fallback identity transform
                                p_map = PoseStamped()
                                p_map.header.frame_id = self.map_frame_id
                                p_map.header.stamp = now
                                p_map.pose.position.x = transform.transform.translation.x + x_cam
                                p_map.pose.position.y = transform.transform.translation.y + y_cam
                                p_map.pose.position.z = 0.0  # 2D map
                                p_map.pose.orientation.w = 1.0
                            else:
                                # Use tf2 transform if available
                                p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                                p_map.pose.position.z = 0.0  # Ensure 2D projection
                                
                            right_marker.points.append(p_map.pose.position)
                        except Exception as e:
                            self.get_logger().debug(f'Failed to transform point: {e}')
                    
                    marker_array.markers.append(right_marker)
                
                # Create marker for lane center
                if self.lane_center_points:
                    center_marker = Marker()
                    center_marker.header.frame_id = self.map_frame_id
                    center_marker.header.stamp = now
                    center_marker.ns = "map_lane_center"
                    center_marker.id = 2
                    center_marker.type = Marker.LINE_STRIP
                    center_marker.action = Marker.ADD
                    center_marker.pose.orientation.w = 1.0
                    center_marker.scale.x = 0.1  # Line width
                    center_marker.color.r = 0.0
                    center_marker.color.g = 1.0
                    center_marker.color.b = 0.0
                    center_marker.color.a = 1.0
                    center_marker.lifetime.sec = 0  # Persist until explicitly deleted
                    
                    # Transform lane center points to map frame
                    for point in self.lane_center_points:
                        pixel_x, pixel_y = point
                        
                        # Calculate distance based on y position in image
                        # Lower in the image = closer to the camera
                        distance_factor = 1.0 - (pixel_y / 480.0)
                        distance = 2.0 + (self.lane_projection_distance * distance_factor)
                        
                        # Convert pixel to 3D point in camera frame
                        x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                        
                        # Create a point in the camera frame
                        p_cam = PoseStamped()
                        p_cam.header.frame_id = self.camera_frame_id
                        p_cam.header.stamp = now
                        p_cam.pose.position.x = x_cam
                        p_cam.pose.position.y = y_cam
                        p_cam.pose.position.z = 0.0  # Project to 2D plane
                        p_cam.pose.orientation.w = 1.0
                        
                        # Transform to map frame
                        try:
                            # Use tf_buffer if available, otherwise fall back to manual transform
                            if self.fallback_to_identity_transform and transform is not None:
                                # Manual transform when using fallback identity transform
                                p_map = PoseStamped()
                                p_map.header.frame_id = self.map_frame_id
                                p_map.header.stamp = now
                                p_map.pose.position.x = transform.transform.translation.x + x_cam
                                p_map.pose.position.y = transform.transform.translation.y + y_cam
                                p_map.pose.position.z = 0.0  # 2D map
                                p_map.pose.orientation.w = 1.0
                            else:
                                # Use tf2 transform if available
                                p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                                p_map.pose.position.z = 0.0  # Ensure 2D projection
                                
                            center_marker.points.append(p_map.pose.position)
                        except Exception as e:
                            self.get_logger().debug(f'Failed to transform point: {e}')
                    
                    marker_array.markers.append(center_marker)
            
            # Visualize memory lanes if they should be used and are valid
            if use_memory and memory_valid:
                self.get_logger().debug('Using memory lanes for marker visualization')
                
                # Get memory lanes in world coordinates
                memory_left_world, memory_right_world, memory_center_world = self.get_memory_lane_world_coordinates()
                
                # Create markers for left memory lane
                if memory_left_world and (not self.left_lane_points or len(self.left_lane_points) < self.min_lane_points):
                    left_marker = Marker()
                    left_marker.header.frame_id = self.map_frame_id
                    left_marker.header.stamp = now
                    left_marker.ns = "memory_left_lane"
                    left_marker.id = 3  # Different ID to not overwrite current detections
                    left_marker.type = Marker.LINE_STRIP
                    left_marker.action = Marker.ADD
                    left_marker.pose.orientation.w = 1.0
                    left_marker.scale.x = 0.2  # Line width
                    left_marker.color.r = 1.0
                    left_marker.color.g = 0.5
                    left_marker.color.b = 0.0
                    left_marker.color.a = 1.0
                    left_marker.lifetime.sec = 0  # Persist until explicitly deleted
                    
                    # Memory points are already in map frame
                    for point in memory_left_world:
                        p = Point()
                        p.x = point[0]
                        p.y = point[1]
                        p.z = 0.0  # Keep in 2D plane
                        left_marker.points.append(p)
                        
                    marker_array.markers.append(left_marker)
                
                # Create markers for right memory lane
                if memory_right_world and (not self.right_lane_points or len(self.right_lane_points) < self.min_lane_points):
                    right_marker = Marker()
                    right_marker.header.frame_id = self.map_frame_id
                    right_marker.header.stamp = now
                    right_marker.ns = "memory_right_lane"
                    right_marker.id = 4  # Different ID to not overwrite current detections
                    right_marker.type = Marker.LINE_STRIP
                    right_marker.action = Marker.ADD
                    right_marker.pose.orientation.w = 1.0
                    right_marker.scale.x = 0.2  # Line width
                    right_marker.color.r = 1.0
                    right_marker.color.g = 0.5
                    right_marker.color.b = 0.0
                    right_marker.color.a = 1.0
                    right_marker.lifetime.sec = 0  # Persist until explicitly deleted
                    
                    # Memory points are already in map frame
                    for point in memory_right_world:
                        p = Point()
                        p.x = point[0]
                        p.y = point[1]
                        p.z = 0.0  # Keep in 2D plane
                        right_marker.points.append(p)
                        
                    marker_array.markers.append(right_marker)
                
                # Create markers for center memory lane
                if memory_center_world and (not self.lane_center_points or len(self.lane_center_points) < self.min_lane_points):
                    center_marker = Marker()
                    center_marker.header.frame_id = self.map_frame_id
                    center_marker.header.stamp = now
                    center_marker.ns = "memory_center_lane"
                    center_marker.id = 5  # Different ID to not overwrite current detections
                    center_marker.type = Marker.LINE_STRIP
                    center_marker.action = Marker.ADD
                    center_marker.pose.orientation.w = 1.0
                    center_marker.scale.x = 0.1  # Line width
                    center_marker.color.r = 0.0
                    center_marker.color.g = 1.0
                    center_marker.color.b = 0.5
                    center_marker.color.a = 1.0
                    center_marker.lifetime.sec = 0  # Persist until explicitly deleted
                    
                    # Memory points are already in map frame
                    for point in memory_center_world:
                        p = Point()
                        p.x = point[0]
                        p.y = point[1]
                        p.z = 0.0  # Keep in 2D plane
                        center_marker.points.append(p)
                        
                    marker_array.markers.append(center_marker)
            
            # Publish the marker array
            self.lane_marker_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing lane markers: {e}')

    def update_memory_lanes(self):
        """Update the lane memory with current detections in world coordinates"""
        transform = self.get_transform(self.map_frame_id, self.camera_frame_id)
        if transform is None and not self.fallback_to_identity_transform:
            return
            
        if transform is None:
            transform = self.create_identity_transform(self.map_frame_id, self.camera_frame_id)
        
        # Get vehicle position from transform (base_link position in map frame)
        vehicle_transform = self.get_transform(self.map_frame_id, self.base_frame_id)
        if vehicle_transform is None and not self.fallback_to_identity_transform:
            return
            
        if vehicle_transform is None:
            vehicle_transform = self.create_identity_transform(self.map_frame_id, self.base_frame_id)
        
        # Extract vehicle position for lane filtering
        vehicle_x = vehicle_transform.transform.translation.x
        vehicle_y = vehicle_transform.transform.translation.y
        
        # Clear previous memory
        self.memory_left_lane_points = []
        self.memory_right_lane_points = []
        self.memory_center_lane_points = []
        
        now = self.get_clock().now().to_msg()
        
        # Transform left lane points to map frame and store in memory
        if len(self.left_lane_points) >= self.min_lane_points:
            for point in self.left_lane_points:
                pixel_x, pixel_y = point
                distance_factor = 1.0 - (pixel_y / 480.0)
                distance = 2.0 + (self.lane_projection_distance * distance_factor)
                x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                
                p_cam = PoseStamped()
                p_cam.header.frame_id = self.camera_frame_id
                p_cam.header.stamp = now
                p_cam.pose.position.x = x_cam
                p_cam.pose.position.y = y_cam
                p_cam.pose.position.z = 0.0
                p_cam.pose.orientation.w = 1.0
                
                try:
                    if self.fallback_to_identity_transform and transform is not None:
                        # Manual transform
                        map_x = transform.transform.translation.x + x_cam
                        map_y = transform.transform.translation.y + y_cam
                        map_z = 0.0
                    else:
                        # TF2 transform
                        p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                        map_x = p_map.pose.position.x
                        map_y = p_map.pose.position.y
                        map_z = 0.0
                        
                    # Store the point relative to the current vehicle position
                    # This helps keep the lanes properly aligned even with drift
                    rel_x = map_x - vehicle_x
                    rel_y = map_y - vehicle_y
                    self.memory_left_lane_points.append((rel_x, rel_y, map_z))
                except Exception as e:
                    self.get_logger().debug(f'Failed to transform point for memory: {e}')
        
        # Transform right lane points to map frame and store in memory
        if len(self.right_lane_points) >= self.min_lane_points:
            for point in self.right_lane_points:
                pixel_x, pixel_y = point
                distance_factor = 1.0 - (pixel_y / 480.0)
                distance = 2.0 + (self.lane_projection_distance * distance_factor)
                x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                
                p_cam = PoseStamped()
                p_cam.header.frame_id = self.camera_frame_id
                p_cam.header.stamp = now
                p_cam.pose.position.x = x_cam
                p_cam.pose.position.y = y_cam
                p_cam.pose.position.z = 0.0
                p_cam.pose.orientation.w = 1.0
                
                try:
                    if self.fallback_to_identity_transform and transform is not None:
                        # Manual transform
                        map_x = transform.transform.translation.x + x_cam
                        map_y = transform.transform.translation.y + y_cam
                        map_z = 0.0
                    else:
                        # TF2 transform
                        p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                        map_x = p_map.pose.position.x
                        map_y = p_map.pose.position.y
                        map_z = 0.0
                        
                    # Store the point relative to the current vehicle position
                    rel_x = map_x - vehicle_x
                    rel_y = map_y - vehicle_y
                    self.memory_right_lane_points.append((rel_x, rel_y, map_z))
                except Exception as e:
                    self.get_logger().debug(f'Failed to transform point for memory: {e}')
        
        # Transform center lane points to map frame and store in memory
        if len(self.lane_center_points) >= self.min_lane_points:
            for point in self.lane_center_points:
                pixel_x, pixel_y = point
                distance_factor = 1.0 - (pixel_y / 480.0)
                distance = 2.0 + (self.lane_projection_distance * distance_factor)
                x_cam, y_cam, z_cam = self.pixel_to_3d_point(pixel_x, pixel_y, distance)
                
                p_cam = PoseStamped()
                p_cam.header.frame_id = self.camera_frame_id
                p_cam.header.stamp = now
                p_cam.pose.position.x = x_cam
                p_cam.pose.position.y = y_cam
                p_cam.pose.position.z = 0.0
                p_cam.pose.orientation.w = 1.0
                
                try:
                    if self.fallback_to_identity_transform and transform is not None:
                        # Manual transform
                        map_x = transform.transform.translation.x + x_cam
                        map_y = transform.transform.translation.y + y_cam
                        map_z = 0.0
                    else:
                        # TF2 transform
                        p_map = self.tf_buffer.transform(p_cam, self.map_frame_id)
                        map_x = p_map.pose.position.x
                        map_y = p_map.pose.position.y
                        map_z = 0.0
                        
                    # Store the point relative to the current vehicle position
                    rel_x = map_x - vehicle_x
                    rel_y = map_y - vehicle_y
                    self.memory_center_lane_points.append((rel_x, rel_y, map_z))
                except Exception as e:
                    self.get_logger().debug(f'Failed to transform point for memory: {e}')
        
        self.get_logger().debug(f'Updated memory lanes: L={len(self.memory_left_lane_points)}, R={len(self.memory_right_lane_points)}, C={len(self.memory_center_lane_points)}')
        
    def get_memory_lane_world_coordinates(self):
        """
        Transform memory lane points from vehicle-relative coordinates to world coordinates.
        Returns: (memory_left_world, memory_right_world, memory_center_world) tuples of points
        """
        # Get current vehicle position
        vehicle_transform = self.get_transform(self.map_frame_id, self.base_frame_id)
        if vehicle_transform is None and not self.fallback_to_identity_transform:
            return [], [], []
            
        if vehicle_transform is None:
            vehicle_transform = self.create_identity_transform(self.map_frame_id, self.base_frame_id)
        
        # Extract vehicle position
        vehicle_x = vehicle_transform.transform.translation.x
        vehicle_y = vehicle_transform.transform.translation.y
        
        # Transform memory points to world coordinates
        memory_left_world = []
        for rel_x, rel_y, z in self.memory_left_lane_points:
            world_x = vehicle_x + rel_x
            world_y = vehicle_y + rel_y
            memory_left_world.append((world_x, world_y, z))
            
        memory_right_world = []
        for rel_x, rel_y, z in self.memory_right_lane_points:
            world_x = vehicle_x + rel_x
            world_y = vehicle_y + rel_y
            memory_right_world.append((world_x, world_y, z))
            
        memory_center_world = []
        for rel_x, rel_y, z in self.memory_center_lane_points:
            world_x = vehicle_x + rel_x
            world_y = vehicle_y + rel_y
            memory_center_world.append((world_x, world_y, z))
            
        return memory_left_world, memory_right_world, memory_center_world

def main(args=None):
    rclpy.init(args=args)
    node = LaneToMapMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 