#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from sensor_msgs.msg import Imu
import numpy as np
import math
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Lock
import time


class TrajectoryPlanner(Node):
    """
    Plans optimal trajectories based on fused sensor data and vehicle state.
    
    This node:
    1. Subscribes to fused costmap from fusion_costmap_creator
    2. Considers vehicle state and dynamics from IMU
    3. Computes feasible and safe trajectories
    4. Publishes path and velocity commands
    """
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Declare parameters
        self.declare_parameter('planning_frequency', 10.0)  # Hz
        self.declare_parameter('max_speed', 5.0)            # m/s
        self.declare_parameter('max_acceleration', 2.0)     # m/s^2
        self.declare_parameter('max_deceleration', 3.0)     # m/s^2
        self.declare_parameter('wheel_base', 2.7)           # meters
        self.declare_parameter('max_steering_angle', 0.5)   # radians
        self.declare_parameter('goal_tolerance', 1.0)       # meters
        self.declare_parameter('prediction_horizon', 3.0)   # seconds
        self.declare_parameter('safety_margin', 0.5)        # meters
        self.declare_parameter('use_dynamic_window', True)  # Use DWA for local planning
        self.declare_parameter('debug_visualization', True) # Show debug trajectories
        self.declare_parameter('path_optimization', True)   # Optimize path smoothness
        
        # Get parameters
        self.planning_frequency = self.get_parameter('planning_frequency').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.use_dynamic_window = self.get_parameter('use_dynamic_window').value
        self.debug_visualization = self.get_parameter('debug_visualization').value
        self.path_optimization = self.get_parameter('path_optimization').value
        
        # Vehicle state
        self.current_pose = None
        self.current_velocity = 0.0  # m/s
        self.current_yaw = 0.0       # radians
        self.current_acceleration = 0.0  # m/s^2
        self.current_steering_angle = 0.0  # radians
        
        # Goals and planning
        self.global_path = []        # List of (x, y) waypoints
        self.current_goal = None     # Current goal position (x, y)
        self.costmap = None          # Current costmap
        
        # Locks for thread safety
        self.state_lock = Lock()
        self.costmap_lock = Lock()
        self.path_lock = Lock()
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.path_publisher = self.create_publisher(
            Path, 
            '/planned_path', 
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )
        
        # Debug publishers
        if self.debug_visualization:
            self.debug_trajectories_publisher = self.create_publisher(
                Path,
                '/debug/trajectories',
                10
            )
        
        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            '/fusion/costmap',  # From fusion_costmap_creator
            self.costmap_callback,
            10
        )
        
        self.create_subscription(
            Imu,
            '/imu/data',  # From imu_listener
            self.imu_callback,
            10
        )
        
        # TODO: Subscribe to goal position (could be from RVIZ or higher-level planner)
        # self.create_subscription(
        #     PoseStamped,
        #     '/goal_pose',
        #     self.goal_callback,
        #     10
        # )
        
        # Planning timer
        self.create_timer(1.0/self.planning_frequency, self.plan_trajectory)
        
        # Performance monitoring
        self.planning_count = 0
        self.last_performance_time = time.time()
        self.create_timer(10.0, self.report_performance)
        
        # Test goal (for demonstration)
        self.set_test_goal()
        
        self.get_logger().info('Trajectory planner initialized')
        self.get_logger().info(f'Planning frequency: {self.planning_frequency} Hz')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'Prediction horizon: {self.prediction_horizon} seconds')
    
    def set_test_goal(self):
        """Set a test goal for demonstration purposes"""
        self.current_goal = (20.0, 0.0)  # 20 meters ahead
        self.get_logger().info(f'Set test goal at ({self.current_goal[0]}, {self.current_goal[1]})')
    
    def costmap_callback(self, msg):
        """Process incoming costmap"""
        with self.costmap_lock:
            self.costmap = msg
            self.get_logger().debug(f'Received costmap: {msg.info.width}x{msg.info.height}')
    
    def imu_callback(self, msg):
        """Process IMU data to update vehicle state"""
        with self.state_lock:
            # Extract orientation as quaternion
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            
            # Convert to Euler angles (roll, pitch, yaw)
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
            
            # Update vehicle state
            self.current_yaw = yaw
            
            # Extract linear acceleration
            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            
            # Project acceleration onto vehicle's forward direction
            self.current_acceleration = accel_x * math.cos(yaw) + accel_y * math.sin(yaw)
            
            # Update velocity estimate (simple integration)
            # In a real system, you would use more sophisticated methods
            dt = 0.1  # Assumption about IMU update rate
            self.current_velocity += self.current_acceleration * dt
            
            # Clamp velocity to reasonable range
            self.current_velocity = max(0.0, min(self.max_speed, self.current_velocity))
            
            self.get_logger().debug(f'Updated vehicle state: yaw={yaw:.2f}, vel={self.current_velocity:.2f}, accel={self.current_acceleration:.2f}')
    
    def goal_callback(self, msg):
        """Process incoming goal position"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.current_goal = (x, y)
        self.get_logger().info(f'Received new goal: ({x}, {y})')
    
    def is_position_safe(self, x, y):
        """Check if a position is safe (not in collision)"""
        if self.costmap is None:
            return False
            
        with self.costmap_lock:
            # Convert world coordinates to costmap cell coordinates
            resolution = self.costmap.info.resolution
            origin_x = self.costmap.info.origin.position.x
            origin_y = self.costmap.info.origin.position.y
            
            cell_x = int((x - origin_x) / resolution)
            cell_y = int((y - origin_y) / resolution)
            
            # Check if coordinates are within costmap bounds
            width = self.costmap.info.width
            height = self.costmap.info.height
            
            if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
                return False
                
            # Get cell value (0=free, 100=occupied, -1=unknown)
            index = cell_y * width + cell_x
            if index >= len(self.costmap.data):
                return False
                
            value = self.costmap.data[index]
            
            # Consider values above threshold as unsafe
            safety_threshold = 50  # Cells with occupancy > 50 are considered unsafe
            return value < safety_threshold
    
    def generate_bicycle_trajectory(self, start_x, start_y, start_yaw, steering_angle, speed, duration, dt=0.1):
        """Generate a trajectory using the bicycle model for vehicle kinematics"""
        trajectory = []
        x, y, yaw = start_x, start_y, start_yaw
        
        # Generate trajectory points at dt intervals
        steps = int(duration / dt)
        for _ in range(steps):
            # Bicycle model equations
            beta = math.atan(0.5 * math.tan(steering_angle))  # Slip angle at center of gravity
            
            # Update position
            x += speed * math.cos(yaw + beta) * dt
            y += speed * math.sin(yaw + beta) * dt
            
            # Update heading
            yaw += speed * math.tan(steering_angle) / self.wheel_base * dt
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # Normalize angle
            
            trajectory.append((x, y, yaw))
            
        return trajectory
    
    def evaluate_trajectory(self, trajectory):
        """Evaluate a trajectory for safety and goal progress"""
        if not trajectory:
            return float('-inf')
            
        # Check for collisions
        for x, y, _ in trajectory:
            if not self.is_position_safe(x, y):
                return float('-inf')
                
        # Calculate distance to goal at end of trajectory
        end_x, end_y, _ = trajectory[-1]
        if self.current_goal is None:
            goal_distance_score = 0.0
        else:
            goal_x, goal_y = self.current_goal
            goal_distance = math.sqrt((end_x - goal_x)**2 + (end_y - goal_y)**2)
            goal_distance_score = -goal_distance  # Negative distance (higher is better)
            
        # Calculate smoothness score
        smoothness_score = 0.0
        if len(trajectory) > 2:
            angle_changes = 0.0
            for i in range(1, len(trajectory)-1):
                prev_yaw = trajectory[i-1][2]
                curr_yaw = trajectory[i][2]
                next_yaw = trajectory[i+1][2]
                
                # Calculate difference in consecutive heading changes
                angle_change = abs((next_yaw - curr_yaw) - (curr_yaw - prev_yaw))
                angle_changes += angle_change
                
            smoothness_score = -angle_changes  # Negative angle changes (higher is better)
        
        # Calculate clearance score (distance to obstacles)
        clearance_score = 0.0
        # In a real implementation, you would use the costmap to calculate clearance
            
        # Combine scores with weights
        w_goal = 1.0
        w_smoothness = 0.3
        w_clearance = 0.2
        
        total_score = w_goal * goal_distance_score + w_smoothness * smoothness_score + w_clearance * clearance_score
        return total_score
    
    def plan_trajectory(self):
        """Generate and evaluate multiple trajectories, select the best one"""
        if self.current_goal is None or self.costmap is None:
            return
            
        with self.state_lock, self.costmap_lock, self.path_lock:
            # Get current state
            # In a real system, you would get the vehicle's position from localization
            # Here we're just assuming it's at the origin for simplicity
            current_x = 0.0
            current_y = 0.0
            current_yaw = self.current_yaw
            current_speed = self.current_velocity
            
            # Sample different control inputs (steering angles and speeds)
            # These would typically be constrained by the vehicle dynamics
            steering_angles = np.linspace(-self.max_steering_angle, self.max_steering_angle, 7)
            speeds = np.linspace(0.2 * self.max_speed, self.max_speed, 5)
            
            best_trajectory = None
            best_score = float('-inf')
            all_trajectories = []
            
            # Generate and evaluate trajectories
            for steering in steering_angles:
                for speed in speeds:
                    # Generate a trajectory using the bicycle model
                    trajectory = self.generate_bicycle_trajectory(
                        current_x, current_y, current_yaw, 
                        steering, speed, self.prediction_horizon
                    )
                    
                    # Evaluate the trajectory
                    score = self.evaluate_trajectory(trajectory)
                    
                    # Save for debug visualization
                    if self.debug_visualization:
                        all_trajectories.append((trajectory, score))
                    
                    # Update best trajectory
                    if score > best_score:
                        best_score = score
                        best_trajectory = trajectory
            
            # If we found a valid trajectory, publish it
            if best_trajectory is not None:
                self.publish_path(best_trajectory)
                self.publish_cmd_vel(best_trajectory)
                self.planning_count += 1
                
                if self.debug_visualization:
                    self.publish_debug_trajectories(all_trajectories)
    
    def publish_path(self, trajectory):
        """Publish the planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y, yaw in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            q = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            path_msg.poses.append(pose)
            
        self.path_publisher.publish(path_msg)
    
    def publish_cmd_vel(self, trajectory):
        """Publish velocity command for the first step of the trajectory"""
        if not trajectory:
            return
            
        # Extract steering and speed from first trajectory segment
        # In a real system, you'd map this to wheel velocities or steering commands
        # Here we're just publishing a Twist message for demonstration
        
        # Get first segment heading change to estimate steering
        if len(trajectory) > 1:
            _, _, yaw0 = trajectory[0]
            _, _, yaw1 = trajectory[1]
            
            # Calculate linear and angular velocities
            x1, y1, _ = trajectory[1]
            distance = math.sqrt(x1**2 + y1**2)  # Distance from origin
            dt = self.prediction_horizon / len(trajectory)
            linear_vel = distance / dt
            
            # Yaw change per time is angular velocity
            angular_vel = (yaw1 - yaw0) / dt
            
            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().debug(f'Published cmd_vel: linear={linear_vel:.2f}, angular={angular_vel:.2f}')
    
    def publish_debug_trajectories(self, trajectories):
        """Publish all evaluated trajectories for debugging"""
        if not trajectories:
            return
            
        # Sort trajectories by score for visualization
        trajectories.sort(key=lambda x: x[1], reverse=True)
        
        # Create a path message with all trajectories
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        # Add all trajectories (up to some limit for clarity)
        max_trajectories = 10
        for trajectory, _ in trajectories[:max_trajectories]:
            for x, y, yaw in trajectory:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                
                # Convert yaw to quaternion
                q = quaternion_from_euler(0.0, 0.0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                
                path_msg.poses.append(pose)
                
        self.debug_trajectories_publisher.publish(path_msg)
    
    def report_performance(self):
        """Report planning performance statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_performance_time
        
        if elapsed > 0 and self.planning_count > 0:
            planning_rate = self.planning_count / elapsed
            self.get_logger().info(f'Trajectory planning performance: {planning_rate:.2f} Hz')
            
            self.planning_count = 0
            self.last_performance_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 