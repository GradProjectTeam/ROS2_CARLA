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
        yaw_offset_descriptor = ParameterDescriptor(
            description='Yaw offset to adjust IMU orientation in radians',
            floating_point_range=[FloatingPointRange(
                from_value=-3.14159,
                to_value=3.14159,
                step=0.01
            )]
        )
        
        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('use_filtered_yaw', True)
        self.declare_parameter('yaw_filter_size', 5)
        self.declare_parameter('yaw_weight', 0.8)
        self.declare_parameter('goal_distance', 5.0)
        self.declare_parameter('yaw_offset', math.pi, yaw_offset_descriptor)  # Default 180 degrees (π radians) offset
        
        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.use_filtered_yaw = self.get_parameter('use_filtered_yaw').value
        self.yaw_filter_size = self.get_parameter('yaw_filter_size').value
        self.yaw_weight = self.get_parameter('yaw_weight').value
        self.goal_distance = self.get_parameter('goal_distance').value
        self.yaw_offset = self.get_parameter('yaw_offset').value
        
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
        
        # New IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )
        
        # Initialize state
        self.map_data = None
        self.map_ready = False
        self.current_yaw = 0.0
        self.raw_yaw = 0.0  # Store the raw IMU yaw for debugging
        self.yaw_buffer = deque(maxlen=self.yaw_filter_size)
        self.filtered_yaw = 0.0
        self.imu_ready = False
        
        # Timer for publishing
        self.publish_timer = self.create_timer(0.1, self.publish_poses)
        
        self.get_logger().info('Navigation tester node initialized')
        self.get_logger().info(f'Using IMU topic: {self.imu_topic}')
        self.get_logger().info(f'Using map topic: {self.map_topic}')
        self.get_logger().info(f'Using yaw offset: {math.degrees(self.yaw_offset):.1f} degrees')
        self.get_logger().info('Waiting for map and IMU data...')
        self.get_logger().info('You can dynamically adjust the yaw_offset parameter with:')
        self.get_logger().info('  ros2 param set /test_navigation yaw_offset <value_in_radians>')
    
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
        
        # Store the current yaw
        self.current_yaw = corrected_yaw
        
        # Apply moving average filter if enabled
        if self.use_filtered_yaw:
            self.yaw_buffer.append(corrected_yaw)
            
            # Compute filtered yaw (handle wrap-around for angles)
            sin_sum = sum(math.sin(y) for y in self.yaw_buffer)
            cos_sum = sum(math.cos(y) for y in self.yaw_buffer)
            self.filtered_yaw = math.atan2(sin_sum, cos_sum)
        
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
            # Using trigonometry to place the goal in front of the current position in the direction of yaw
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'
            
            # Calculate position using current yaw (front of the robot)
            goal_pose.pose.position.x = map_center_x + self.goal_distance * math.cos(orientation_yaw)
            goal_pose.pose.position.y = map_center_y + self.goal_distance * math.sin(orientation_yaw)
            goal_pose.pose.position.z = 0.0
            
            # Set goal orientation to current yaw 
            goal_pose.pose.orientation.x = q[1]
            goal_pose.pose.orientation.y = q[2]
            goal_pose.pose.orientation.z = q[3]
            goal_pose.pose.orientation.w = q[0]
            
            self.goal_pose_pub.publish(goal_pose)
            
            # Log the current orientation periodically
            current_time = time.time()
            if not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 5.0:
                self.get_logger().info(f'Raw yaw: {math.degrees(self.raw_yaw):.1f} degrees')
                self.get_logger().info(f'Current yaw: {math.degrees(orientation_yaw):.1f} degrees')
                self.get_logger().info(f'Goal position: {goal_pose.pose.position.x:.1f}, {goal_pose.pose.position.y:.1f}')
                self.last_log_time = current_time

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