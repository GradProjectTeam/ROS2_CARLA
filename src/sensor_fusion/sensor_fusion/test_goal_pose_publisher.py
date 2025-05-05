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
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

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
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('random_goals', False)  # Whether to generate random goals
        self.declare_parameter('goal_distance', 10.0)  # Distance from current position for goals
        self.declare_parameter('publish_topic', '/goal_pose')  # Topic to publish poses on
        self.declare_parameter('use_current_pose', False)  # Whether to publish current robot pose instead of goals
        self.declare_parameter('goal_x', 10.0)  # Fixed goal X coordinate
        self.declare_parameter('goal_y', 0.0)   # Fixed goal Y coordinate
        self.declare_parameter('goal_yaw', 0.0)  # Fixed goal yaw angle
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.random_goals = self.get_parameter('random_goals').value
        self.goal_distance = self.get_parameter('goal_distance').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.use_current_pose = self.get_parameter('use_current_pose').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        
        # Add parameter callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.publish_topic,
            qos_profile
        )
        
        # TF buffer and listener for getting robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriber for the map to get dimensions
        if not self.use_current_pose:
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
        if not self.use_current_pose:
            self.goals = [
                (self.goal_x, self.goal_y, self.goal_yaw),  # Use parameter values
                (0.0, 10.0, 1.57),  # Left (90 degrees)
                (-10.0, 0.0, 3.14), # Back (180 degrees)
                (0.0, -10.0, 4.71)  # Right (270 degrees)
            ]
        
        # Timer for publishing
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_pose)
        
        self.get_logger().info(f'Goal/Pose Publisher initialized on topic: {self.publish_topic}')
        self.get_logger().info(f'Publishing frequency: {self.publish_rate} Hz')
        self.get_logger().info(f'Using map frame: {self.map_frame_id}')
        
        if self.use_current_pose:
            self.get_logger().info(f'Publishing robot current pose from TF')
            self.get_logger().info(f'Using base frame: {self.base_frame_id}')
        else:
            self.get_logger().info(f'Publishing goal poses')
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
                if self.map_ready and not self.use_current_pose:
                    self.update_predefined_goals()
            
            elif param.name == 'publish_topic':
                old_topic = self.publish_topic
                self.publish_topic = param.value
                
                # Create a new publisher with the updated topic
                self.pose_pub = self.create_publisher(
                    PoseStamped,
                    self.publish_topic,
                    QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=10
                    )
                )
                self.get_logger().info(f'Updated publish_topic from {old_topic} to {self.publish_topic}')
            
            elif param.name == 'use_current_pose':
                self.use_current_pose = param.value
                self.get_logger().info(f'Updated use_current_pose to {self.use_current_pose}')
            
            elif param.name in ['goal_x', 'goal_y', 'goal_yaw']:
                if param.name == 'goal_x':
                    self.goal_x = param.value
                elif param.name == 'goal_y':
                    self.goal_y = param.value
                elif param.name == 'goal_yaw':
                    self.goal_yaw = param.value
                
                # Update the first goal in the list
                if not self.use_current_pose and len(self.goals) > 0:
                    self.goals[0] = (self.goal_x, self.goal_y, self.goal_yaw)
                    self.get_logger().info(f'Updated fixed goal to ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
        
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
        if self.map_data and not self.use_current_pose:
            # Calculate map center
            map_center_x = self.map_data.info.origin.position.x + (self.map_data.info.width * self.map_data.info.resolution) / 2.0
            map_center_y = self.map_data.info.origin.position.y + (self.map_data.info.height * self.map_data.info.resolution) / 2.0
            
            # Update predefined goals based on map center and goal distance
            self.goals = [
                (self.goal_x, self.goal_y, self.goal_yaw),  # Use parameter values for first goal
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
    
    def get_robot_pose(self):
        """Get the current robot pose from TF"""
        try:
            # Look up the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Convert transform to PoseStamped message
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame_id
            
            # Set position from translation
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            
            # Set orientation from rotation
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose: {ex}')
            return None
    
    def publish_pose(self):
        """Publish either the robot's current pose or a goal pose for testing"""
        if self.use_current_pose:
            # Get and publish current robot pose from TF
            pose_msg = self.get_robot_pose()
            if pose_msg:
                self.pose_pub.publish(pose_msg)
                self.get_logger().debug(f'Published robot pose at ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f})')
            return
        
        # If we're publishing goal poses, make sure map data is ready
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
        self.pose_pub.publish(goal_pose)
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