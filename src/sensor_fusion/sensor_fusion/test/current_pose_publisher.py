#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
import math
import tf2_ros
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CurrentPosePublisher(Node):
    def __init__(self):
        super().__init__('current_pose_publisher')
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # How often to publish current pose (Hz)
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        
        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # Add parameter callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            '/current_pose',
            qos_profile
        )
        
        # Subscriber for the map to determine if the system is ready
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
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_current_pose)
        
        self.get_logger().info('Current Pose Publisher initialized')
        self.get_logger().info(f'Publishing frequency: {self.publish_rate} Hz')
        self.get_logger().info(f'Vehicle frame: {self.vehicle_frame_id}')
        self.get_logger().info(f'Map frame: {self.map_frame_id}')
        self.get_logger().info('Waiting for map data and transforms...')
    
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
            
            elif param.name == 'vehicle_frame_id':
                self.vehicle_frame_id = param.value
                self.get_logger().info(f'Updated vehicle_frame_id to {self.vehicle_frame_id}')
            
            elif param.name == 'map_frame_id':
                self.map_frame_id = param.value
                self.get_logger().info(f'Updated map_frame_id to {self.map_frame_id}')
        
        return result
    
    def map_callback(self, msg):
        """Process the map to determine if the system is ready"""
        self.map_data = msg
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info('Map data received')
    
    def get_vehicle_transform(self):
        """Get the transform from map to vehicle"""
        try:
            # Look up transform from map to vehicle
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.vehicle_frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warning(f'Failed to get transform: {str(e)}')
            return None
    
    def publish_current_pose(self):
        """Publish the current vehicle pose"""
        if not self.map_ready:
            return
        
        # Get the current transform
        transform = self.get_vehicle_transform()
        if not transform:
            self.get_logger().warning('Transform not available yet')
            return
        
        # Create the current pose message
        current_pose = PoseStamped()
        current_pose.header = transform.header
        current_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position from transform
        current_pose.pose.position.x = transform.transform.translation.x
        current_pose.pose.position.y = transform.transform.translation.y
        current_pose.pose.position.z = transform.transform.translation.z
        
        # Set orientation from transform
        current_pose.pose.orientation = transform.transform.rotation
        
        # Publish the current pose
        self.current_pose_pub.publish(current_pose)
        self.get_logger().debug(f'Published current pose at ({current_pose.pose.position.x:.2f}, {current_pose.pose.position.y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 