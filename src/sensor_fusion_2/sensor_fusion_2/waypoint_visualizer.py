#!/usr/bin/env python3
# waypoint_visualizer.py
"""
Waypoint Visualizer Node

This node subscribes to waypoint messages and publishes visualization markers
for display in RViz. It can color waypoints based on lane type and display
lane and road information.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int32MultiArray
from nav_msgs.msg import OccupancyGrid

class WaypointVisualizer(Node):
    """
    ROS2 Node for visualizing waypoints in RViz
    """
    def __init__(self):
        super().__init__('waypoint_visualizer')
        
        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('waypoints_topic', '/carla/waypoints')
        self.declare_parameter('visualization_topic', '/visualization/waypoints')
        self.declare_parameter('marker_size', 0.2)
        self.declare_parameter('line_width', 0.1)
        self.declare_parameter('marker_lifetime', 0.5)
        self.declare_parameter('show_lane_info', True)
        self.declare_parameter('show_road_info', True)
        self.declare_parameter('color_by_lane_type', True)
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        self.show_lane_info = self.get_parameter('show_lane_info').get_parameter_value().bool_value
        self.show_road_info = self.get_parameter('show_road_info').get_parameter_value().bool_value
        self.color_by_lane_type = self.get_parameter('color_by_lane_type').get_parameter_value().bool_value
        
        # Setup QoS profile for better reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers and publishers
        # Use PoseArray for waypoint positions and Int32MultiArray for metadata
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            self.get_parameter('waypoints_topic').get_parameter_value().string_value,
            self.waypoints_callback,
            qos_profile
        )
        
        # Subscribe to metadata (road_id, lane_id, lane_type) if available
        self.metadata_sub = self.create_subscription(
            Int32MultiArray,
            self.get_parameter('waypoints_topic').get_parameter_value().string_value + "_metadata",
            self.metadata_callback,
            qos_profile
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('visualization_topic').get_parameter_value().string_value,
            qos_profile
        )
        
        # Store the latest metadata
        self.latest_metadata = None
        
        self.get_logger().info('Waypoint Visualizer Node initialized')
    
    def metadata_callback(self, msg):
        """
        Store metadata for waypoints
        
        Args:
            msg (Int32MultiArray): Array of metadata (road_id, lane_id, lane_type)
        """
        self.latest_metadata = msg.data
        self.get_logger().debug(f'Received metadata for {len(self.latest_metadata) // 3} waypoints')
        
    def waypoints_callback(self, msg):
        """
        Process incoming waypoint messages and publish visualization markers
        
        Args:
            msg (PoseArray): Array of waypoint positions
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty waypoint array')
            return
            
        self.get_logger().debug(f'Received {len(msg.poses)} waypoints')
        
        # Create marker array for visualization
        marker_array = MarkerArray()
        
        # Add point markers for each waypoint
        points_marker = Marker()
        points_marker.header.frame_id = self.frame_id
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "waypoints"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = self.marker_size
        points_marker.scale.y = self.marker_size
        points_marker.scale.z = self.marker_size
        points_marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
        
        # Add line strip connecting waypoints
        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "waypoint_connections"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = self.line_width
        line_marker.color.r = 1.0
        line_marker.color.g = 1.0
        line_marker.color.b = 1.0
        line_marker.color.a = 0.8
        line_marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
        
        # Text markers for lane/road info if enabled
        text_markers = []
        
        # Check if we have metadata
        has_metadata = self.latest_metadata is not None and len(self.latest_metadata) >= len(msg.poses) * 3
        
        # Process each waypoint
        for i, pose in enumerate(msg.poses):
            # Add point to the points marker
            p = Point()
            p.x = pose.position.x
            p.y = pose.position.y
            p.z = pose.position.z
            points_marker.points.append(p)
            
            # Get metadata if available
            road_id = -1
            lane_id = -1
            lane_type = -1
            if has_metadata and i * 3 + 2 < len(self.latest_metadata):
                road_id = self.latest_metadata[i * 3]
                lane_id = self.latest_metadata[i * 3 + 1]
                lane_type = self.latest_metadata[i * 3 + 2]
            
            # Add color based on lane type if enabled
            color = self.get_color_for_lane_type(lane_type)
            points_marker.colors.append(color)
            
            # Add point to the line strip
            line_marker.points.append(p)
            
            # Create text markers for every 5th waypoint if info display is enabled
            if (i % 5 == 0) and (self.show_lane_info or self.show_road_info) and has_metadata:
                text_marker = Marker()
                text_marker.header.frame_id = self.frame_id
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "waypoint_info"
                text_marker.id = 100 + i  # Unique ID for each text marker
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = pose.position.x
                text_marker.pose.position.y = pose.position.y
                text_marker.pose.position.z = pose.position.z + 0.5  # Offset text above waypoint
                text_marker.scale.z = 0.5  # Text size
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                
                # Construct text based on enabled info
                info_text = ""
                if self.show_road_info:
                    info_text += f"Road: {road_id}"
                if self.show_lane_info:
                    if info_text:
                        info_text += "\n"
                    info_text += f"Lane: {lane_id}"
                    info_text += f"\nType: {self.get_lane_type_name(lane_type)}"
                
                text_marker.text = info_text
                text_marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
                text_markers.append(text_marker)
        
        # Add all markers to the array
        marker_array.markers.append(points_marker)
        marker_array.markers.append(line_marker)
        for marker in text_markers:
            marker_array.markers.append(marker)
        
        # Publish the marker array
        self.marker_pub.publish(marker_array)
    
    def get_color_for_lane_type(self, lane_type):
        """
        Return color based on lane type
        
        Args:
            lane_type (int): Lane type code
            
        Returns:
            ColorRGBA: Color for the lane type
        """
        color = ColorRGBA()
        color.a = 1.0  # Full opacity
        
        # Define colors based on lane type if enabled
        if self.color_by_lane_type:
            if lane_type == 1:  # Driving
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
            elif lane_type == 2:  # Shoulder
                color.r = 1.0
                color.g = 1.0
                color.b = 0.0
            elif lane_type == 3:  # Sidewalk
                color.r = 1.0
                color.g = 0.0
                color.b = 0.0
            else:  # Other/Unknown
                color.r = 0.0
                color.g = 0.0
                color.b = 1.0
        else:
            # Default color if not coloring by lane type
            color.r = 0.0
            color.g = 0.7
            color.b = 1.0
            
        return color
        
    def get_lane_type_name(self, lane_type):
        """
        Convert lane type code to human-readable name
        
        Args:
            lane_type (int): Lane type code
            
        Returns:
            str: Human-readable lane type name
        """
        lane_types = {
            0: "NONE",
            1: "Driving",
            2: "Shoulder",
            3: "Sidewalk",
            4: "Border",
            5: "Restricted",
            6: "Parking",
            7: "Bidirectional",
            8: "Median",
            9: "Special1",
            10: "Special2",
            11: "Special3",
            12: "RoadWorks",
            13: "Tram",
            14: "Rail",
            15: "Entry",
            16: "Exit",
            17: "OffRamp",
            18: "OnRamp"
        }
        return lane_types.get(lane_type, f"Unknown({lane_type})")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down waypoint visualizer')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 