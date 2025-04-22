#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from collections import deque
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Define parameter descriptors for better parameter service
        # Simplified parameter descriptor without range constraints to avoid validation issues
        yaw_offset_descriptor = ParameterDescriptor(
            description='Yaw offset to adjust IMU orientation in radians'
        )
        
        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('use_filtered_yaw', True)
        self.declare_parameter('yaw_filter_size', 3)  # Smaller filter size for faster response (was 5)
        self.declare_parameter('yaw_weight', 0.8)
        self.declare_parameter('goal_distance', 5.0)
        self.declare_parameter('yaw_offset', 1.5, yaw_offset_descriptor)  # Default 1.5 radians (approximately 90 degrees)
        self.declare_parameter('publish_rate', 20.0)  # Higher publish rate (was 10Hz, now 20Hz) 
        
        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.use_filtered_yaw = self.get_parameter('use_filtered_yaw').value
        self.yaw_filter_size = self.get_parameter('yaw_filter_size').value
        self.yaw_weight = self.get_parameter('yaw_weight').value
        self.goal_distance = self.get_parameter('goal_distance').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Add parameter callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Publishers
        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            '/current_pose',
            qos_profile
        )
        
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )
        
        # New IMU subscriber - higher QoS depth for better responsiveness
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            20  # Increased QoS depth (was 10)
        )
        
        # Initialize state
        self.map_data = None
        self.map_ready = False
        self.current_yaw = 0.0
        self.raw_yaw = 0.0  # Store the raw IMU yaw for debugging
        self.yaw_buffer = deque(maxlen=self.yaw_filter_size)
        self.filtered_yaw = 0.0
        self.imu_ready = False
        self.last_orientation_yaw = None
        self.last_orientation_time = time.time()
        self.force_goal_update = False
        
        # Timer for publishing - use the publish_rate parameter
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_poses)
        
        # Add a higher frequency IMU monitoring timer to react faster to orientation changes
        self.imu_monitor_timer = self.create_timer(0.02, self.check_orientation_changes)  # 50Hz monitoring
        
        self.get_logger().info('Navigation tester node initialized')
        self.get_logger().info(f'Using IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Using map topic: {self.map_topic}')
        self.get_logger().info(f'Using yaw offset: {math.degrees(self.yaw_offset):.1f} degrees')
        self.get_logger().info(f'Goal distance: {self.goal_distance:.1f} meters')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
        self.get_logger().info('Waiting for map and IMU data...')
        self.get_logger().info('You can dynamically adjust parameters with:')
        self.get_logger().info('  ros2 param set /navigation_tester yaw_offset <value_in_radians>')
        self.get_logger().info('  ros2 param set /navigation_tester goal_distance <value_in_meters>')
        self.get_logger().info('  For example, to set 90 degrees rotation: ros2 param set /navigation_tester yaw_offset 1.6')
        self.get_logger().info('  To extend path to 10 meters: ros2 param set /navigation_tester goal_distance 10.0')
    
    def parameters_callback(self, params):
        """Handle dynamic parameter updates"""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'yaw_offset':
                old_offset = self.yaw_offset
                self.yaw_offset = param.value
                self.get_logger().info(f'Updated yaw_offset from {math.degrees(old_offset):.1f} to {math.degrees(self.yaw_offset):.1f} degrees')
                
                # Clear the yaw buffer to apply the new offset immediately
                self.yaw_buffer.clear()
                
                # If we have raw yaw data, update the current yaw with the new offset
                if hasattr(self, 'raw_yaw'):
                    corrected_yaw = self.normalize_angle(self.raw_yaw + self.yaw_offset)
                    self.current_yaw = corrected_yaw
            
            elif param.name == 'goal_distance':
                old_distance = self.goal_distance
                self.goal_distance = param.value
                self.get_logger().info(f'Updated goal_distance from {old_distance:.1f} to {self.goal_distance:.1f} meters')
                self.get_logger().info(f'Path length has been adjusted. The goal will now be {self.goal_distance} meters ahead.')
                    
        return result
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
        
    def map_callback(self, msg):
        self.map_data = msg
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info('Map data received')
    
    def imu_callback(self, msg):
        """Process incoming IMU data to extract yaw information"""
        # Extract quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        # Convert to Euler angles
        _, _, yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        
        # Store the raw yaw for debugging and dynamic offset adjustments
        self.raw_yaw = yaw
        
        # Apply yaw offset to correct for IMU orientation
        corrected_yaw = self.normalize_angle(yaw + self.yaw_offset)
        
        # Calculate change from current yaw
        old_yaw = self.current_yaw
        yaw_diff = abs(self.normalize_angle(corrected_yaw - old_yaw))
        
        # Store the current yaw
        self.current_yaw = corrected_yaw
        
        # Apply moving average filter if enabled - using a weighted average for faster response
        if self.use_filtered_yaw:
            self.yaw_buffer.append(corrected_yaw)
            
            # For faster response, use a weighted average that emphasizes recent values
            if len(self.yaw_buffer) >= 2:
                # Convert to unit vectors to handle wrapping correctly
                sin_vals = [math.sin(y) for y in self.yaw_buffer]
                cos_vals = [math.cos(y) for y in self.yaw_buffer]
                
                # Apply weights - more recent values have higher weight
                weights = [0.5 + 0.5 * i / (len(self.yaw_buffer) - 1) for i in range(len(self.yaw_buffer))]
                
                # Normalize weights
                sum_weights = sum(weights)
                weights = [w / sum_weights for w in weights]
                
                # Compute weighted average
                weighted_sin = sum(s * w for s, w in zip(sin_vals, weights))
                weighted_cos = sum(c * w for c, w in zip(cos_vals, weights))
                
                self.filtered_yaw = math.atan2(weighted_sin, weighted_cos)
            else:
                # Not enough samples yet, use current value
                self.filtered_yaw = corrected_yaw
        
        # Force goal update if significant rotation (more than 2 degrees)
        if yaw_diff > math.radians(2.0):
            self.force_goal_update = True
        
        if not self.imu_ready:
            self.imu_ready = True
            self.get_logger().info('IMU data received')
            self.get_logger().info(f'Initial IMU yaw: {math.degrees(yaw):.1f} degrees')
            self.get_logger().info(f'Corrected yaw: {math.degrees(corrected_yaw):.1f} degrees')
    
    def normalize_angle(self, angle):
        """Normalize angle to be between -π and π"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
            
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        # Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = cy * cp * cr + sy * sp * sr  # w
        q[1] = cy * cp * sr - sy * sp * cr  # x
        q[2] = sy * cp * sr + cy * sp * cr  # y
        q[3] = sy * cp * cr - cy * sp * sr  # z
        
        return q
            
    def publish_poses(self):
        """Publish current pose and goal pose with IMU yaw integration"""
        # Only publish if both map and IMU are available
        if not (self.map_ready and self.imu_ready):
            return
        
        # Was a forced update requested by the orientation monitor?
        force_update = self.force_goal_update
        if force_update:
            self.force_goal_update = False  # Reset the flag
        
        # Publish current pose - simulated position at the center of the map
        if self.map_data:
            # Get map center
            map_center_x = self.map_data.info.origin.position.x + (self.map_data.info.width * self.map_data.info.resolution) / 2.0
            map_center_y = self.map_data.info.origin.position.y + (self.map_data.info.height * self.map_data.info.resolution) / 2.0
            
            # Create pose message
            current_pose = PoseStamped()
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.header.frame_id = 'map'
            
            # Set position to map center
            current_pose.pose.position.x = map_center_x
            current_pose.pose.position.y = map_center_y
            current_pose.pose.position.z = 0.0
            
            # Get the yaw to use (filtered or direct)
            orientation_yaw = self.filtered_yaw if self.use_filtered_yaw else self.current_yaw
            
            # Set orientation based on IMU yaw
            q = self.quaternion_from_euler(0.0, 0.0, orientation_yaw)
            current_pose.pose.orientation.x = q[1]
            current_pose.pose.orientation.y = q[2]
            current_pose.pose.orientation.z = q[3]
            current_pose.pose.orientation.w = q[0]
            
            self.current_pose_pub.publish(current_pose)
            
            # Calculate goal position based on current pose and orientation
            # Always place the goal directly in front of the vehicle based on its current orientation
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'
            
            # CHANGE: Force recalculation of goal position using current orientation every time
            # This ensures the goal rotates when the vehicle rotates
            goal_pose.pose.position.x = map_center_x + self.goal_distance * math.cos(orientation_yaw)
            goal_pose.pose.position.y = map_center_y + self.goal_distance * math.sin(orientation_yaw)
            goal_pose.pose.position.z = 0.0
            
            # Set goal orientation to be the same as current orientation
            # This ensures the path aligns with the vehicle's direction
            goal_pose.pose.orientation.x = q[1]
            goal_pose.pose.orientation.y = q[2]
            goal_pose.pose.orientation.z = q[3]
            goal_pose.pose.orientation.w = q[0]
            
            self.goal_pose_pub.publish(goal_pose)
            
            # Log the current orientation periodically or when forced update occurs
            current_time = time.time()
            if force_update or not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 5.0:
                self.get_logger().info(f'Raw yaw: {math.degrees(self.raw_yaw):.1f} degrees')
                self.get_logger().info(f'Vehicle orientation: {math.degrees(orientation_yaw):.1f} degrees')
                self.get_logger().info(f'Goal position: {goal_pose.pose.position.x:.1f}, {goal_pose.pose.position.y:.1f}')
                self.get_logger().info(f'Path direction: Forward along vehicle heading')
                self.last_log_time = current_time

    def check_orientation_changes(self):
        """
        Monitor orientation changes at high frequency and force 
        goal updates when significant changes are detected
        """
        if not self.imu_ready:
            return
        
        # Get the current orientation
        current_orientation_yaw = self.filtered_yaw if self.use_filtered_yaw else self.current_yaw
        current_time = time.time()
        
        # Initialize on first call
        if self.last_orientation_yaw is None:
            self.last_orientation_yaw = current_orientation_yaw
            self.last_orientation_time = current_time
            return
        
        # Check for significant orientation change (more than 1 degree)
        yaw_diff = abs(self.normalize_angle(current_orientation_yaw - self.last_orientation_yaw))
        if yaw_diff > math.radians(1.0):
            # Log the change
            self.get_logger().debug(f'Orientation changed by {math.degrees(yaw_diff):.1f} degrees')
            
            # Force an immediate goal update
            self.force_goal_update = True
            
            # Immediately publish updated poses
            self.publish_poses()
            
            # Update the stored values
            self.last_orientation_yaw = current_orientation_yaw
            self.last_orientation_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = NavigationTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 