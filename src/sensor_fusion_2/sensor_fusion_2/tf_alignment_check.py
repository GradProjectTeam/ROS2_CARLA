#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
import math
import time

class TFAlignmentChecker(Node):
    """
    A tool to visualize the alignment between LiDAR and radar sensors.
    
    This node:
    1. Publishes directional arrows from each sensor frame
    2. Publishes reference points at fixed distances
    3. Helps visualize the alignment between sensors
    """
    def __init__(self):
        super().__init__('tf_alignment_checker')
        
        # Parameters
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('radar_frame', 'radar_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('arrow_length', 5.0)  # meters
        self.declare_parameter('reference_distance', 10.0)  # meters
        
        # Get parameters
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.radar_frame = self.get_parameter('radar_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.arrow_length = self.get_parameter('arrow_length').value
        self.reference_distance = self.get_parameter('reference_distance').value
        
        # Create marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/sensor_alignment/markers',
            10
        )
        
        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create timer for publishing markers
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_markers
        )
        
        self.get_logger().info('TF Alignment Checker initialized')
        self.get_logger().info(f'Checking alignment between {self.lidar_frame} and {self.radar_frame}')
    
    def publish_markers(self):
        """Publish markers to visualize sensor alignment"""
        try:
            # Wait for transforms to be available
            if not self.tf_buffer.can_transform(self.map_frame, self.lidar_frame, rclpy.time.Time()):
                self.get_logger().warn(f'Transform from {self.map_frame} to {self.lidar_frame} not available')
                return
                
            if not self.tf_buffer.can_transform(self.map_frame, self.radar_frame, rclpy.time.Time()):
                self.get_logger().warn(f'Transform from {self.map_frame} to {self.radar_frame} not available')
                return
            
            # Create marker array
            marker_array = MarkerArray()
            
            # Add LiDAR direction arrow
            lidar_arrow = self.create_direction_arrow(
                self.lidar_frame,
                0,  # ID
                [0.0, 1.0, 0.0, 0.7],  # Green
                self.arrow_length
            )
            marker_array.markers.append(lidar_arrow)
            
            # Add radar direction arrow
            radar_arrow = self.create_direction_arrow(
                self.radar_frame,
                1,  # ID
                [1.0, 0.0, 0.0, 0.7],  # Red
                self.arrow_length
            )
            marker_array.markers.append(radar_arrow)
            
            # Add reference points
            self.add_reference_points(marker_array, 2)
            
            # Publish marker array
            self.marker_pub.publish(marker_array)
            
        except TransformException as e:
            self.get_logger().error(f'Transform error: {str(e)}')
    
    def create_direction_arrow(self, frame_id, marker_id, color, length):
        """Create a direction arrow marker for the given frame"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sensor_directions"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow points from origin forward
        marker.points = []
        start_point = PointStamped()
        start_point.point.x = 0.0
        start_point.point.y = 0.0
        start_point.point.z = 0.0
        marker.points.append(start_point.point)
        
        end_point = PointStamped()
        end_point.point.x = length  # Forward in the sensor's frame
        end_point.point.y = 0.0
        end_point.point.z = 0.0
        marker.points.append(end_point.point)
        
        # Set scale (shaft diameter, head diameter, head length)
        marker.scale.x = 0.1  # Shaft diameter
        marker.scale.y = 0.2  # Head diameter
        marker.scale.z = 0.3  # Head length
        
        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Set lifetime
        marker.lifetime.sec = 1
        
        return marker
    
    def add_reference_points(self, marker_array, start_id):
        """Add reference points at fixed distances"""
        try:
            # Get transforms
            lidar_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.lidar_frame,
                rclpy.time.Time()
            )
            
            radar_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.radar_frame,
                rclpy.time.Time()
            )
            
            # Create reference point in front of LiDAR
            lidar_ref = Marker()
            lidar_ref.header.frame_id = self.map_frame
            lidar_ref.header.stamp = self.get_clock().now().to_msg()
            lidar_ref.ns = "reference_points"
            lidar_ref.id = start_id
            lidar_ref.type = Marker.SPHERE
            lidar_ref.action = Marker.ADD
            
            # Calculate point in front of LiDAR
            lidar_x = lidar_transform.transform.translation.x
            lidar_y = lidar_transform.transform.translation.y
            lidar_z = lidar_transform.transform.translation.z
            
            # Extract rotation from quaternion
            qx = lidar_transform.transform.rotation.x
            qy = lidar_transform.transform.rotation.y
            qz = lidar_transform.transform.rotation.z
            qw = lidar_transform.transform.rotation.w
            
            # Convert to rotation matrix (simplified for yaw only)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Calculate point at reference distance in front of LiDAR
            lidar_ref.pose.position.x = lidar_x + self.reference_distance * math.cos(yaw)
            lidar_ref.pose.position.y = lidar_y + self.reference_distance * math.sin(yaw)
            lidar_ref.pose.position.z = lidar_z
            
            lidar_ref.pose.orientation.w = 1.0
            lidar_ref.scale.x = 0.5
            lidar_ref.scale.y = 0.5
            lidar_ref.scale.z = 0.5
            lidar_ref.color.r = 0.0
            lidar_ref.color.g = 1.0
            lidar_ref.color.b = 0.0
            lidar_ref.color.a = 0.7
            lidar_ref.lifetime.sec = 1
            
            marker_array.markers.append(lidar_ref)
            
            # Create reference point in front of radar
            radar_ref = Marker()
            radar_ref.header.frame_id = self.map_frame
            radar_ref.header.stamp = self.get_clock().now().to_msg()
            radar_ref.ns = "reference_points"
            radar_ref.id = start_id + 1
            radar_ref.type = Marker.SPHERE
            radar_ref.action = Marker.ADD
            
            # Calculate point in front of radar
            radar_x = radar_transform.transform.translation.x
            radar_y = radar_transform.transform.translation.y
            radar_z = radar_transform.transform.translation.z
            
            # Extract rotation from quaternion
            qx = radar_transform.transform.rotation.x
            qy = radar_transform.transform.rotation.y
            qz = radar_transform.transform.rotation.z
            qw = radar_transform.transform.rotation.w
            
            # Convert to rotation matrix (simplified for yaw only)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Calculate point at reference distance in front of radar
            radar_ref.pose.position.x = radar_x + self.reference_distance * math.cos(yaw)
            radar_ref.pose.position.y = radar_y + self.reference_distance * math.sin(yaw)
            radar_ref.pose.position.z = radar_z
            
            radar_ref.pose.orientation.w = 1.0
            radar_ref.scale.x = 0.5
            radar_ref.scale.y = 0.5
            radar_ref.scale.z = 0.5
            radar_ref.color.r = 1.0
            radar_ref.color.g = 0.0
            radar_ref.color.b = 0.0
            radar_ref.color.a = 0.7
            radar_ref.lifetime.sec = 1
            
            marker_array.markers.append(radar_ref)
            
            # Add a line connecting the two reference points
            connection_line = Marker()
            connection_line.header.frame_id = self.map_frame
            connection_line.header.stamp = self.get_clock().now().to_msg()
            connection_line.ns = "reference_connections"
            connection_line.id = start_id + 2
            connection_line.type = Marker.LINE_STRIP
            connection_line.action = Marker.ADD
            
            # Add points to line
            p1 = PointStamped()
            p1.point.x = lidar_ref.pose.position.x
            p1.point.y = lidar_ref.pose.position.y
            p1.point.z = lidar_ref.pose.position.z
            connection_line.points.append(p1.point)
            
            p2 = PointStamped()
            p2.point.x = radar_ref.pose.position.x
            p2.point.y = radar_ref.pose.position.y
            p2.point.z = radar_ref.pose.position.z
            connection_line.points.append(p2.point)
            
            connection_line.scale.x = 0.1  # Line width
            connection_line.color.r = 1.0
            connection_line.color.g = 1.0
            connection_line.color.b = 0.0
            connection_line.color.a = 0.7
            connection_line.lifetime.sec = 1
            
            marker_array.markers.append(connection_line)
            
            # Calculate and log the distance between reference points
            dx = lidar_ref.pose.position.x - radar_ref.pose.position.x
            dy = lidar_ref.pose.position.y - radar_ref.pose.position.y
            dz = lidar_ref.pose.position.z - radar_ref.pose.position.z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Log the distance periodically
            if int(time.time()) % 5 == 0:  # Log every 5 seconds
                self.get_logger().info(f'Distance between reference points: {distance:.2f} meters')
                
                # Calculate angle between directions
                lidar_yaw = math.atan2(lidar_ref.pose.position.y - lidar_y, 
                                      lidar_ref.pose.position.x - lidar_x)
                radar_yaw = math.atan2(radar_ref.pose.position.y - radar_y, 
                                      radar_ref.pose.position.x - radar_x)
                angle_diff = math.degrees(abs(lidar_yaw - radar_yaw))
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                    
                self.get_logger().info(f'Angle between sensor directions: {angle_diff:.2f} degrees')
                
                if angle_diff < 10:
                    self.get_logger().info('Sensors are well-aligned in direction')
                elif angle_diff < 30:
                    self.get_logger().warn('Sensors have moderate directional misalignment')
                else:
                    self.get_logger().error('Sensors have significant directional misalignment')
            
        except TransformException as e:
            self.get_logger().error(f'Transform error in reference points: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TFAlignmentChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()