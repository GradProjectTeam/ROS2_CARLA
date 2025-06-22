#!/usr/bin/env python3
"""
Waypoint Path Planner Node

This node subscribes to waypoint data, processes it into a path plan,
and publishes the path for navigation. It integrates with the existing
waypoint visualization system and provides a path that can be used for
vehicle control.

Author: Mostafa
"""

import numpy as np
import threading
import time
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header, ColorRGBA, Int32MultiArray
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from builtin_interfaces.msg import Duration

class WaypointPathPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_path_planner')
        
        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('waypoints_topic', '/carla/waypoints')
        self.declare_parameter('path_topic', '/waypoint_path')
        self.declare_parameter('path_visualization_topic', '/visualization/waypoint_path')
        self.declare_parameter('path_update_rate', 10.0)  # Hz
        self.declare_parameter('path_lookahead', 50.0)  # meters
        self.declare_parameter('path_density', 1.0)  # points per meter
        self.declare_parameter('path_smoothing', True)  # Apply smoothing to path
        self.declare_parameter('path_width', 0.2)  # meters
        self.declare_parameter('path_lifetime', 0.0)  # seconds (0.0 = never expire)
        self.declare_parameter('use_binary_map', True)  # Use binary map for path planning
        self.declare_parameter('binary_map_topic', '/waypoint_map/binary')
        self.declare_parameter('fixed_origin', True)  # Use fixed origin for path planning
        self.declare_parameter('use_local_coordinates', True)  # Use local coordinates
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').get_parameter_value().string_value
        self.waypoints_topic = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.path_visualization_topic = self.get_parameter('path_visualization_topic').get_parameter_value().string_value
        self.path_update_rate = self.get_parameter('path_update_rate').get_parameter_value().double_value
        self.path_lookahead = self.get_parameter('path_lookahead').get_parameter_value().double_value
        self.path_density = self.get_parameter('path_density').get_parameter_value().double_value
        self.path_smoothing = self.get_parameter('path_smoothing').get_parameter_value().bool_value
        self.path_width = self.get_parameter('path_width').get_parameter_value().double_value
        self.path_lifetime = self.get_parameter('path_lifetime').get_parameter_value().double_value
        self.use_binary_map = self.get_parameter('use_binary_map').get_parameter_value().bool_value
        self.binary_map_topic = self.get_parameter('binary_map_topic').get_parameter_value().string_value
        self.fixed_origin = self.get_parameter('fixed_origin').get_parameter_value().bool_value
        self.use_local_coordinates = self.get_parameter('use_local_coordinates').get_parameter_value().bool_value
        
        # Set up QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create persistent QoS profile for path visualization
        persistent_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Create publishers
        self.path_publisher = self.create_publisher(
            Path,
            self.path_topic,
            persistent_qos  # Use persistent QoS for path
        )
        
        self.path_viz_publisher = self.create_publisher(
            Marker,
            self.path_visualization_topic,
            persistent_qos  # Use persistent QoS for visualization
        )
        
        # Create subscribers
        self.waypoint_subscriber = self.create_subscription(
            PoseArray,
            self.waypoints_topic,
            self.waypoint_callback,
            qos_profile
        )
        
        self.metadata_subscriber = self.create_subscription(
            Int32MultiArray,
            self.waypoints_topic + "_metadata",
            self.metadata_callback,
            qos_profile
        )
        
        if self.use_binary_map:
            self.map_subscriber = self.create_subscription(
                OccupancyGrid,
                self.binary_map_topic,
                self.map_callback,
                qos_profile
            )
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize data structures
        self.waypoints = []
        self.waypoint_metadata = {}
        self.binary_map = None
        self.map_info = None
        self.path = None
        self.waypoint_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.path_lock = threading.Lock()
        
        # Create processing queue
        self.waypoint_queue = Queue(maxsize=2)
        self.has_new_waypoints = False
        
        # Create timer for path planning
        self.path_timer = self.create_timer(1.0 / self.path_update_rate, self.plan_path)
        
        # Start processing thread
        self.running = True
        self.process_thread = threading.Thread(target=self.process_waypoints)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        self.get_logger().info("Waypoint path planner initialized")
    
    def waypoint_callback(self, pose_array):
        """Process incoming waypoint poses"""
        try:
            with self.waypoint_lock:
                self.waypoints = pose_array.poses
            
            # Signal that we have new waypoints
            if self.waypoint_queue.full():
                try:
                    self.waypoint_queue.get_nowait()
                except Empty:
                    pass
            self.waypoint_queue.put(True)
            self.has_new_waypoints = True
            
            self.get_logger().debug(f"Received {len(pose_array.poses)} waypoint poses")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint poses: {e}")
    
    def metadata_callback(self, metadata):
        """Process waypoint metadata"""
        try:
            # Extract metadata from Int32MultiArray
            # Format: [road_id, lane_id, lane_type] for each waypoint
            data = metadata.data
            if len(data) % 3 != 0:
                self.get_logger().warn(f"Invalid metadata format: {len(data)} values not divisible by 3")
                return
                
            with self.waypoint_lock:
                self.waypoint_metadata = {
                    'road_ids': data[0::3],
                    'lane_ids': data[1::3],
                    'lane_types': data[2::3]
                }
                
            self.get_logger().debug(f"Received metadata for {len(data)//3} waypoints")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint metadata: {e}")
    
    def map_callback(self, map_msg):
        """Process binary map"""
        try:
            with self.map_lock:
                # Convert map data to numpy array for easier processing
                width = map_msg.info.width
                height = map_msg.info.height
                self.binary_map = np.array(map_msg.data).reshape((height, width))
                self.map_info = map_msg.info
                
            self.get_logger().debug("Received binary map")
        except Exception as e:
            self.get_logger().error(f"Error processing binary map: {e}")
    
    def process_waypoints(self):
        """Process waypoints from queue"""
        while self.running:
            try:
                if self.has_new_waypoints and not self.waypoint_queue.empty():
                    self.waypoint_queue.get()
                    self.generate_path()
                    self.has_new_waypoints = False
                else:
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f"Error in process_waypoints: {e}")
    
    def generate_path(self):
        """Generate path from waypoints"""
        try:
            # Get current waypoints
            with self.waypoint_lock:
                if not self.waypoints:
                    return
                waypoints = self.waypoints.copy()
                metadata = self.waypoint_metadata.copy() if self.waypoint_metadata else {}
            
            # Get vehicle position
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    self.vehicle_frame_id,
                    rclpy.time.Time()
                )
                vehicle_x = transform.transform.translation.x
                vehicle_y = transform.transform.translation.y
            except Exception as e:
                self.get_logger().warn(f"Could not get vehicle transform: {e}")
                vehicle_x, vehicle_y = 0.0, 0.0
            
            # Filter waypoints based on lookahead distance
            filtered_waypoints = []
            for wp in waypoints:
                # Calculate distance from vehicle
                dx = wp.position.x - vehicle_x
                dy = wp.position.y - vehicle_y
                distance = np.sqrt(dx*dx + dy*dy)
                
                if distance <= self.path_lookahead:
                    filtered_waypoints.append(wp)
            
            if not filtered_waypoints:
                self.get_logger().warn("No waypoints within lookahead distance")
                return
                
            # Sort waypoints by distance from vehicle
            filtered_waypoints.sort(key=lambda wp: 
                np.sqrt((wp.position.x - vehicle_x)**2 + (wp.position.y - vehicle_y)**2))
            
            # Apply path smoothing if enabled
            if self.path_smoothing and len(filtered_waypoints) > 2:
                smoothed_waypoints = self.smooth_path(filtered_waypoints)
            else:
                smoothed_waypoints = filtered_waypoints
            
            # Interpolate path to desired density
            if len(smoothed_waypoints) > 1 and self.path_density > 0:
                interpolated_waypoints = self.interpolate_path(smoothed_waypoints)
            else:
                interpolated_waypoints = smoothed_waypoints
            
            # Create path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.frame_id
            
            for wp in interpolated_waypoints:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose = wp
                path_msg.poses.append(pose_stamped)
            
            # Update path
            with self.path_lock:
                self.path = path_msg
            
            self.get_logger().debug(f"Generated path with {len(path_msg.poses)} poses")
            
        except Exception as e:
            self.get_logger().error(f"Error generating path: {e}")
    
    def smooth_path(self, waypoints):
        """Apply smoothing to waypoints"""
        try:
            if len(waypoints) < 3:
                return waypoints
                
            # Simple moving average smoothing
            smoothed = []
            window_size = 3  # Use 3-point moving average
            
            # Keep first point
            smoothed.append(waypoints[0])
            
            # Smooth middle points
            for i in range(1, len(waypoints) - 1):
                # Get window points
                prev = waypoints[i-1]
                curr = waypoints[i]
                next_wp = waypoints[i+1]
                
                # Create smoothed pose
                smooth_pose = Pose()
                
                # Average positions
                smooth_pose.position.x = (prev.position.x + curr.position.x + next_wp.position.x) / 3.0
                smooth_pose.position.y = (prev.position.y + curr.position.y + next_wp.position.y) / 3.0
                smooth_pose.position.z = (prev.position.z + curr.position.z + next_wp.position.z) / 3.0
                
                # Keep original orientation
                smooth_pose.orientation = curr.orientation
                
                smoothed.append(smooth_pose)
            
            # Keep last point
            smoothed.append(waypoints[-1])
            
            return smoothed
            
        except Exception as e:
            self.get_logger().error(f"Error smoothing path: {e}")
            return waypoints
    
    def interpolate_path(self, waypoints):
        """Interpolate waypoints to desired density"""
        try:
            if len(waypoints) < 2:
                return waypoints
                
            interpolated = []
            
            # Add first waypoint
            interpolated.append(waypoints[0])
            
            # Interpolate between each pair of waypoints
            for i in range(len(waypoints) - 1):
                wp1 = waypoints[i]
                wp2 = waypoints[i + 1]
                
                # Calculate distance between waypoints
                dx = wp2.position.x - wp1.position.x
                dy = wp2.position.y - wp1.position.y
                dz = wp2.position.z - wp1.position.z
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                
                # Calculate number of points to add
                num_points = int(distance * self.path_density)
                
                if num_points > 0:
                    # Add interpolated points
                    for j in range(1, num_points + 1):
                        t = j / (num_points + 1)
                        
                        interp_pose = Pose()
                        interp_pose.position.x = wp1.position.x + t * dx
                        interp_pose.position.y = wp1.position.y + t * dy
                        interp_pose.position.z = wp1.position.z + t * dz
                        
                        # Interpolate orientation (simple linear interpolation)
                        # Note: This is not a proper quaternion interpolation (SLERP)
                        interp_pose.orientation.x = wp1.orientation.x + t * (wp2.orientation.x - wp1.orientation.x)
                        interp_pose.orientation.y = wp1.orientation.y + t * (wp2.orientation.y - wp1.orientation.y)
                        interp_pose.orientation.z = wp1.orientation.z + t * (wp2.orientation.z - wp1.orientation.z)
                        interp_pose.orientation.w = wp1.orientation.w + t * (wp2.orientation.w - wp1.orientation.w)
                        
                        # Normalize quaternion
                        norm = np.sqrt(interp_pose.orientation.x**2 + 
                                      interp_pose.orientation.y**2 + 
                                      interp_pose.orientation.z**2 + 
                                      interp_pose.orientation.w**2)
                        
                        if norm > 0:
                            interp_pose.orientation.x /= norm
                            interp_pose.orientation.y /= norm
                            interp_pose.orientation.z /= norm
                            interp_pose.orientation.w /= norm
                        
                        interpolated.append(interp_pose)
            
            # Add last waypoint
            interpolated.append(waypoints[-1])
            
            return interpolated
            
        except Exception as e:
            self.get_logger().error(f"Error interpolating path: {e}")
            return waypoints
    
    def plan_path(self):
        """Plan path and publish results"""
        try:
            # Get current path
            with self.path_lock:
                if self.path is None:
                    return
                path = self.path
            
            # Publish path
            self.path_publisher.publish(path)
            
            # Create visualization marker
            marker = self.create_path_marker(path)
            self.path_viz_publisher.publish(marker)
            
        except Exception as e:
            self.get_logger().error(f"Error planning path: {e}")
    
    def create_path_marker(self, path):
        """Create visualization marker for path"""
        try:
            marker = Marker()
            marker.header = path.header
            marker.ns = "waypoint_path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Set marker properties
            marker.scale.x = self.path_width
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # Set lifetime (0 = forever)
            if self.path_lifetime > 0:
                marker.lifetime = Duration(sec=int(self.path_lifetime), 
                                         nanosec=int((self.path_lifetime % 1) * 1e9))
            
            # Add points from path
            for pose_stamped in path.poses:
                marker.points.append(pose_stamped.pose.position)
            
            return marker
            
        except Exception as e:
            self.get_logger().error(f"Error creating path marker: {e}")
            return None
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        self.get_logger().info("Shutting down waypoint path planner")
        self.running = False
        
        if self.process_thread and self.process_thread.is_alive():
            self.process_thread.join(timeout=1.0)
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    waypoint_path_planner = WaypointPathPlanner()
    
    try:
        rclpy.spin(waypoint_path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 