#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class NavigationTester(Node):
    def __init__(self):
        super().__init__('navigation_tester')
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
            '/realtime_map',
            self.map_callback,
            qos_profile
        )
        
        # Initialize state
        self.map_data = None
        self.map_ready = False
        
        # Timer for publishing
        self.publish_timer = self.create_timer(0.1, self.publish_poses)
        
        self.get_logger().info('Navigation tester node initialized')
        
        # Wait for the map to become available
        self.get_logger().info('Waiting for map data...')
        
    def map_callback(self, msg):
        self.map_data = msg
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info('Map data received')
            
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
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr
        
        return q
            
    def publish_poses(self):
        """Publish current pose and goal pose"""
        # Only publish if map is available
        if not self.map_ready:
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
            
            # Set orientation to zero (facing east)
            q = self.quaternion_from_euler(0.0, 0.0, 0.0)
            current_pose.pose.orientation.x = q[1]
            current_pose.pose.orientation.y = q[2]
            current_pose.pose.orientation.z = q[3]
            current_pose.pose.orientation.w = q[0]
            
            self.current_pose_pub.publish(current_pose)
            
            # Publish goal pose - 5 meters ahead of current pose
            goal_pose = PoseStamped()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.header.frame_id = 'map'
            
            # Set position 5 meters ahead (east)
            goal_pose.pose.position.x = map_center_x + 5.0
            goal_pose.pose.position.y = map_center_y
            goal_pose.pose.position.z = 0.0
            
            # Set orientation to zero (facing east)
            goal_pose.pose.orientation.x = q[1]
            goal_pose.pose.orientation.y = q[2]
            goal_pose.pose.orientation.z = q[3]
            goal_pose.pose.orientation.w = q[0]
            
            self.goal_pose_pub.publish(goal_pose)

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