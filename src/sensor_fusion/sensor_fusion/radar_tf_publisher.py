#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from math import sin, cos
import numpy as np

class RadarTFPublisher(Node):
    def __init__(self):
        super().__init__('radar_tf_publisher')
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create a timer to publish TF frames
        self.timer = self.create_timer(0.1, self.publish_tf)  # 10Hz
        
        self.get_logger().info('Radar TF publisher started')
        
    def publish_tf(self):
        # Current time for TF stamping
        now = self.get_clock().now().to_msg()
        
        # Publish base_link frame (vehicle)
        base_link_tf = TransformStamped()
        base_link_tf.header.stamp = now
        base_link_tf.header.frame_id = "map"
        base_link_tf.child_frame_id = "base_link"
        base_link_tf.transform.translation.x = 0.0
        base_link_tf.transform.translation.y = 0.0
        base_link_tf.transform.translation.z = 0.0
        base_link_tf.transform.rotation.x = 0.0
        base_link_tf.transform.rotation.y = 0.0
        base_link_tf.transform.rotation.z = 0.0
        base_link_tf.transform.rotation.w = 1.0
        
        # Publish radar_link frame (radar sensor)
        radar_link_tf = TransformStamped()
        radar_link_tf.header.stamp = now
        radar_link_tf.header.frame_id = "base_link"
        radar_link_tf.child_frame_id = "radar_link"
        radar_link_tf.transform.translation.x = 1.5  # 1.5m forward from base_link
        radar_link_tf.transform.translation.y = 0.0
        radar_link_tf.transform.translation.z = 2.0  # 2m up from base_link
        radar_link_tf.transform.rotation.x = 0.0
        radar_link_tf.transform.rotation.y = 0.0
        radar_link_tf.transform.rotation.z = 0.0
        radar_link_tf.transform.rotation.w = 1.0
        
        # Broadcast the transforms
        self.tf_broadcaster.sendTransform([base_link_tf, radar_link_tf])

def main(args=None):
    rclpy.init(args=args)
    node = RadarTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 