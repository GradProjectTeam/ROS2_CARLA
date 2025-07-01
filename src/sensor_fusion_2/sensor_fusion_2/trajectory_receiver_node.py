#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String, Int32MultiArray
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Path
import time

class TrajectoryReceiverNode(Node):
    def __init__(self):
        super().__init__('trajectory_receiver_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('path_topic', '/dwa/planned_path')  # DWA path topic
        self.declare_parameter('waypoints_topic', '/carla/waypoints')  # Waypoints topic
        self.declare_parameter('use_waypoints', True)  # Whether to use waypoints
        self.declare_parameter('start_point_offset', 1.5)  # Offset from vehicle for starting point
        self.declare_parameter('min_path_points', 8)  # Minimum number of points required for a valid path
        self.declare_parameter('filter_close_points', True)  # Whether to filter points too close to the vehicle
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.path_topic = self.get_parameter('path_topic').value
        self.waypoints_topic = self.get_parameter('waypoints_topic').value
        self.use_waypoints = self.get_parameter('use_waypoints').value
        self.start_point_offset = self.get_parameter('start_point_offset').value
        self.min_path_points = self.get_parameter('min_path_points').value
        self.filter_close_points = self.get_parameter('filter_close_points').value
        
        # Create publisher for trajectory
        self.trajectory_publisher = self.create_publisher(PoseArray, 'reference_trajectory', 10)
        
        # Reference trajectory
        self.reference_trajectory = PoseArray()
        self.reference_trajectory.header.frame_id = "map"
        
        # Last update timestamps
        self.last_waypoints_time = 0.0
        self.last_dwa_path_time = 0.0
            
        # Create subscriber for DWA planned path
        self.dwa_path_subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.dwa_path_callback,
            10)
        
        self.get_logger().info(f'Subscribed to DWA path topic: {self.path_topic}')
        
        # Create subscriber for waypoints if enabled
        if self.use_waypoints:
            self.waypoints_subscription = self.create_subscription(
                PoseArray,
                self.waypoints_topic,
                self.waypoints_callback,
                10)
            
            # Also subscribe to waypoint metadata to get lane information
        self.metadata_subscription = self.create_subscription(
            Int32MultiArray,
                self.waypoints_topic + "_metadata",
            self.metadata_callback,
            10)
            
        self.get_logger().info(f'Subscribed to waypoints topic: {self.waypoints_topic}')
        
        # Create timer for publishing trajectory
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_trajectory)
        
        self.get_logger().info(f'Trajectory receiver node initialized - Using {"waypoints and " if self.use_waypoints else ""}DWA purple path')
    
    def dwa_path_callback(self, msg):
        """Callback for receiving DWA planned path"""
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty DWA path")
            return
            
        self.get_logger().info(f"Received DWA path with {len(msg.poses)} points")
        
        # Convert Path to PoseArray
        new_trajectory = PoseArray()
        new_trajectory.header = msg.header
        
        # Process the path to ensure a good distribution of points
        # This helps with path following quality
        processed_poses = []
        min_distance = 0.8  # Minimum distance between points (meters)
        max_distance = 3.0  # Maximum distance between points
        
        # Skip the first few points if they're too close to the vehicle
        # This helps avoid conflicts with the car's position
        start_idx = 0
        if self.filter_close_points:
            # Find the first point that's at least start_point_offset away from origin
            # (assuming vehicle is at origin in the path's reference frame)
            for i, pose_stamped in enumerate(msg.poses):
                pose = pose_stamped.pose
                distance = np.sqrt(pose.position.x**2 + pose.position.y**2)
                if distance >= self.start_point_offset:
                    start_idx = i
                    break
            
            if start_idx > 0:
                self.get_logger().debug(f"Skipping first {start_idx} points that are too close to vehicle")
        
        last_pose = None
        cumulative_distance = 0.0
        
        for i in range(start_idx, len(msg.poses)):
            current_pose = msg.poses[i].pose
            
            # For the first point, always include it
            if last_pose is None:
                processed_poses.append(current_pose)
                last_pose = current_pose
                continue
            
            # Calculate distance between points
            dx = current_pose.position.x - last_pose.position.x
            dy = current_pose.position.y - last_pose.position.y
            distance = np.sqrt(dx*dx + dy*dy)
            
            cumulative_distance += distance
            
            # Add point if it's far enough from the last one or it's the last point
            # or if we've gone too far without adding a point
            if cumulative_distance >= min_distance or i == len(msg.poses) - 1 or cumulative_distance >= max_distance:
                # Calculate orientation based on direction to next point
                if i < len(msg.poses) - 1:
                    # Look ahead 2 points if possible for smoother orientation
                    look_ahead = min(i + 2, len(msg.poses) - 1)
                    next_pose = msg.poses[look_ahead].pose
                    direction_x = next_pose.position.x - current_pose.position.x
                    direction_y = next_pose.position.y - current_pose.position.y
                    yaw = np.arctan2(direction_y, direction_x)
                    
                    # Convert yaw to quaternion
                    qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
                    current_pose.orientation.x = qx
                    current_pose.orientation.y = qy
                    current_pose.orientation.z = qz
                    current_pose.orientation.w = qw
                
                processed_poses.append(current_pose)
                last_pose = current_pose
                cumulative_distance = 0.0
        
        # Add the processed poses to the trajectory
        new_trajectory.poses = processed_poses
        
        # Apply path smoothing if we have enough points
        if len(new_trajectory.poses) >= 3:
            self.smooth_path(new_trajectory.poses)
        
        # Only update if we have enough points
        if len(new_trajectory.poses) >= self.min_path_points:
            # Update the timestamp for DWA path
            self.last_dwa_path_time = time.time()
            
            # Only use DWA path if we're not using waypoints or if waypoints are old
            if not self.use_waypoints or (time.time() - self.last_waypoints_time > 1.0):
                self.reference_trajectory = new_trajectory
                self.get_logger().info(f"Updated trajectory with DWA purple path ({len(new_trajectory.poses)} points)")
        else:
            self.get_logger().warn(f"DWA path too short after processing: {len(new_trajectory.poses)} points (need {self.min_path_points})")
    
    def smooth_path(self, poses):
        """Apply smoothing to the path to reduce sharp turns"""
        if len(poses) < 3:
            return
            
        # Simple moving average for positions (not the first and last points)
        for i in range(1, len(poses) - 1):
            # Get neighboring poses
            prev_pose = poses[i-1]
            current_pose = poses[i]
            next_pose = poses[i+1]
            
            # Calculate weighted average of positions (0.25, 0.5, 0.25)
            smooth_x = 0.25 * prev_pose.position.x + 0.5 * current_pose.position.x + 0.25 * next_pose.position.x
            smooth_y = 0.25 * prev_pose.position.y + 0.5 * current_pose.position.y + 0.25 * next_pose.position.y
            
            # Update the position
            current_pose.position.x = smooth_x
            current_pose.position.y = smooth_y
            
        # Recalculate orientations based on smoothed positions
        for i in range(len(poses) - 1):
            current_pose = poses[i]
            next_pose = poses[i+1]
            
            # Calculate direction to next point
            direction_x = next_pose.position.x - current_pose.position.x
            direction_y = next_pose.position.y - current_pose.position.y
            yaw = np.arctan2(direction_y, direction_x)
            
            # Convert yaw to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
            current_pose.orientation.x = qx
            current_pose.orientation.y = qy
            current_pose.orientation.z = qz
            current_pose.orientation.w = qw
        
        # Set the last point's orientation same as the second-to-last point
        if len(poses) > 1:
            poses[-1].orientation = poses[-2].orientation
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr
        
        return qx, qy, qz, qw
    
    def waypoints_callback(self, msg):
        """Callback for receiving waypoints"""
        if not self.use_waypoints:
            return
            
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty waypoints")
            return
            
        self.get_logger().info(f"Received waypoints with {len(msg.poses)} points")
        
        # Process the waypoints to ensure proper orientation and spacing
        new_trajectory = PoseArray()
        new_trajectory.header = msg.header
        
        # Make a copy of the poses to modify
        processed_poses = []
        
        # Minimum distance between waypoints to include
        min_distance = 1.0
        last_pose = None
        
        # Skip the first few points if they're too close to the vehicle
        start_idx = 0
        if self.filter_close_points:
            # Find the first point that's at least start_point_offset away from origin
            for i, pose in enumerate(msg.poses):
                distance = np.sqrt(pose.position.x**2 + pose.position.y**2)
                if distance >= self.start_point_offset:
                    start_idx = i
                    break
            
            if start_idx > 0:
                self.get_logger().debug(f"Skipping first {start_idx} waypoints that are too close to vehicle")
        
        for i in range(start_idx, len(msg.poses)):
            pose = msg.poses[i]
            
            # Skip points that are too close to the previous one
            if last_pose is not None:
                dx = pose.position.x - last_pose.position.x
                dy = pose.position.y - last_pose.position.y
                distance = np.sqrt(dx*dx + dy*dy)
                if distance < min_distance and i < len(msg.poses) - 1:
                    continue
            
            # Deep copy the pose
            new_pose = Pose()
            new_pose.position.x = pose.position.x
            new_pose.position.y = pose.position.y
            new_pose.position.z = pose.position.z
            
            # Check if orientation is set (all zeros means not set)
            if (pose.orientation.x == 0.0 and pose.orientation.y == 0.0 and
                pose.orientation.z == 0.0 and pose.orientation.w == 0.0):
                
                # Calculate orientation based on next waypoint
                if i < len(msg.poses) - 1:
                    # Look ahead 2 points if possible for smoother orientation
                    look_ahead = min(i + 2, len(msg.poses) - 1)
                    next_pose = msg.poses[look_ahead]
                    direction_x = next_pose.position.x - pose.position.x
                    direction_y = next_pose.position.y - pose.position.y
                    yaw = np.arctan2(direction_y, direction_x)
                    
                    # Convert yaw to quaternion
                    qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
                    new_pose.orientation.x = qx
                    new_pose.orientation.y = qy
                    new_pose.orientation.z = qz
                    new_pose.orientation.w = qw
                elif i > 0:
                    # For the last point, use the same orientation as the previous point
                    prev_pose = msg.poses[i-1]
                    direction_x = pose.position.x - prev_pose.position.x
                    direction_y = pose.position.y - prev_pose.position.y
                    yaw = np.arctan2(direction_y, direction_x)
                    
                    # Convert yaw to quaternion
                    qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
                    new_pose.orientation.x = qx
                    new_pose.orientation.y = qy
                    new_pose.orientation.z = qz
                    new_pose.orientation.w = qw
                else:
                    # If it's the only point, use default orientation
                    new_pose.orientation.w = 1.0
            else:
                # Use the original orientation
                new_pose.orientation = pose.orientation
            
            processed_poses.append(new_pose)
            last_pose = pose
        
        # Apply path smoothing if we have enough points
        if len(processed_poses) >= 3:
            self.smooth_path(processed_poses)
        
        new_trajectory.poses = processed_poses
        
        # Only update if we have enough points
        if len(new_trajectory.poses) >= self.min_path_points:
            # Update the timestamp for waypoints
            self.last_waypoints_time = time.time()
            
            # Use waypoints as trajectory
            self.reference_trajectory = new_trajectory
            self.get_logger().info(f"Updated trajectory with waypoints ({len(new_trajectory.poses)} points)")
        else:
            self.get_logger().warn(f"Waypoint path too short after processing: {len(new_trajectory.poses)} points (need {self.min_path_points})")
    
    def metadata_callback(self, msg):
        """Process waypoint metadata"""
        if not self.use_waypoints:
            return
            
        # Extract metadata
        metadata = msg.data
        if len(metadata) >= 3 and len(metadata) % 3 == 0:
            self.get_logger().debug(f"Received metadata for {len(metadata)//3} waypoints")
    
    def publish_trajectory(self):
        """Publish the current reference trajectory"""
        if len(self.reference_trajectory.poses) > 0:
            # Update the timestamp
            self.reference_trajectory.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_publisher.publish(self.reference_trajectory)
            
            # Log the source of the trajectory
            if time.time() - self.last_waypoints_time < 1.0 and self.use_waypoints:
                source = "waypoints"
            else:
                source = "DWA purple path"
                
            self.get_logger().debug(f"Published trajectory from {source} with {len(self.reference_trajectory.poses)} points")
    
    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info("Cleaning up trajectory receiver node")

def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 