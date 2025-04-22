#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import math
import random
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TestGoalPosePublisher(Node):
    def __init__(self):
        super().__init__('test_goal_pose_publisher')
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)  # How often to publish goal poses (Hz)
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('random_goals', False)  # Whether to generate random goals
        self.declare_parameter('goal_distance', 10.0)  # Distance from current position for goals
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.random_goals = self.get_parameter('random_goals').value
        self.goal_distance = self.get_parameter('goal_distance').value
        
        # Add parameter callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Publishers
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            qos_profile
        )
        
        # Subscriber for the map to get dimensions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/realtime_map',
            self.map_callback,
            qos_profile
        )
        
        # Initialize state
        self.map_data = None
        self.map_ready = False
        self.goal_index = 0
        
        # Predefined goal positions (will be updated when map is received)
        self.goals = [
            (10.0, 0.0, 0.0),   # Front
            (0.0, 10.0, 1.57),  # Left (90 degrees)
            (-10.0, 0.0, 3.14), # Back (180 degrees)
            (0.0, -10.0, 4.71)  # Right (270 degrees)
        ]
        
        # Timer for publishing
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_goal_pose)
        
        self.get_logger().info('Test Goal Pose Publisher initialized')
        self.get_logger().info(f'Publishing frequency: {self.publish_rate} Hz')
        self.get_logger().info(f'Using map frame: {self.map_frame_id}')
        self.get_logger().info(f'Random goals: {self.random_goals}')
        self.get_logger().info(f'Goal distance: {self.goal_distance} meters')
        self.get_logger().info('Waiting for map data...')
    
    def parameters_callback(self, params):
        """Handle dynamic parameter updates"""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'publish_rate':
                old_rate = self.publish_rate
                self.publish_rate = param.value
                # Update the timer
                self.publish_timer.timer_period_ns = int(1.0 / self.publish_rate * 1e9)
                self.get_logger().info(f'Updated publish_rate from {old_rate} to {self.publish_rate} Hz')
            
            elif param.name == 'random_goals':
                self.random_goals = param.value
                self.get_logger().info(f'Updated random_goals to {self.random_goals}')
            
            elif param.name == 'goal_distance':
                self.goal_distance = param.value
                self.get_logger().info(f'Updated goal_distance to {self.goal_distance} meters')
                # Update predefined goals
                if self.map_ready:
                    self.update_predefined_goals()
        
        return result
    
    def map_callback(self, msg):
        """Process the map to get dimensions for goal placement"""
        self.map_data = msg
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info('Map data received')
            self.update_predefined_goals()
    
    def update_predefined_goals(self):
        """Update predefined goals based on the map and goal distance"""
        if self.map_data:
            # Calculate map center
            map_center_x = self.map_data.info.origin.position.x + (self.map_data.info.width * self.map_data.info.resolution) / 2.0
            map_center_y = self.map_data.info.origin.position.y + (self.map_data.info.height * self.map_data.info.resolution) / 2.0
            
            # Update predefined goals based on map center and goal distance
            self.goals = [
                (map_center_x + self.goal_distance, map_center_y, 0.0),                  # Front
                (map_center_x, map_center_y + self.goal_distance, math.pi/2),            # Left
                (map_center_x - self.goal_distance, map_center_y, math.pi),              # Back
                (map_center_x, map_center_y - self.goal_distance, 3*math.pi/2)           # Right
            ]
            
            self.get_logger().info(f'Updated predefined goals based on map center ({map_center_x}, {map_center_y})')
    
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
    
    def publish_goal_pose(self):
        """Publish a goal pose for testing"""
        if not self.map_ready:
            return
        
        # Create the goal pose message
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.map_frame_id
        
        if self.random_goals:
            # Generate a random goal within the map bounds
            if self.map_data:
                map_width_meters = self.map_data.info.width * self.map_data.info.resolution
                map_height_meters = self.map_data.info.height * self.map_data.info.resolution
                
                # Random position within map bounds
                x = self.map_data.info.origin.position.x + random.uniform(0, map_width_meters)
                y = self.map_data.info.origin.position.y + random.uniform(0, map_height_meters)
                yaw = random.uniform(-math.pi, math.pi)
                
                goal_pose.pose.position.x = x
                goal_pose.pose.position.y = y
                goal_pose.pose.position.z = 0.0
            else:
                # Fallback if no map - use origin with random orientation
                goal_pose.pose.position.x = random.uniform(-self.goal_distance, self.goal_distance)
                goal_pose.pose.position.y = random.uniform(-self.goal_distance, self.goal_distance)
                goal_pose.pose.position.z = 0.0
                yaw = random.uniform(-math.pi, math.pi)
        else:
            # Use predefined goals in sequence
            x, y, yaw = self.goals[self.goal_index]
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.position.z = 0.0
            
            # Increment goal index for next time
            self.goal_index = (self.goal_index + 1) % len(self.goals)
        
        # Set orientation
        q = self.quaternion_from_euler(0.0, 0.0, yaw)
        goal_pose.pose.orientation.x = q[1]
        goal_pose.pose.orientation.y = q[2]
        goal_pose.pose.orientation.z = q[3]
        goal_pose.pose.orientation.w = q[0]
        
        # Publish the goal pose
        self.goal_pose_pub.publish(goal_pose)
        self.get_logger().info(f'Published goal pose at ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}) with yaw {math.degrees(yaw):.1f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = TestGoalPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 