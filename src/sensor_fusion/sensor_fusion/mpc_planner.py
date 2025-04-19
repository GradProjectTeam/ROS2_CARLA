#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import time
import math
import cvxpy as cp
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MPCPlanner(Node):
    def __init__(self):
        super().__init__('mpc_planner')
        
        # Declare parameters
        self.declare_parameter('horizon', 10)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('wheelbase', 2.5)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('map_topic', '/realtime_map')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        
        # Get parameters
        self.horizon = self.get_parameter('horizon').value
        self.dt = self.get_parameter('dt').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_topic = self.get_parameter('map_topic').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        
        # Initial state
        self.current_pose = None
        self.current_velocity = 0.0
        self.goal_pose = None
        self.map_data = None
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            Path, 
            '/planned_trajectory', 
            qos_profile
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            qos_profile
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_profile
        )
        self.twist_sub = self.create_subscription(
            TwistStamped,
            '/current_twist',
            self.twist_callback,
            qos_profile
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos_profile
        )
        
        # Timer for planning
        self.planning_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.planning_callback
        )
        
        self.get_logger().info('MPC Planner node initialized')
    
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
    def twist_callback(self, msg):
        self.current_velocity = math.sqrt(
            msg.twist.linear.x**2 + 
            msg.twist.linear.y**2
        )
        
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().debug('Received map update')
        
    def generate_reference_path(self):
        """Generate a simple reference path from current position to goal"""
        if not self.current_pose or not self.goal_pose:
            return None
            
        # Extract current position
        x0 = self.current_pose.pose.position.x
        y0 = self.current_pose.pose.position.y
        
        # Extract goal position
        xg = self.goal_pose.pose.position.x
        yg = self.goal_pose.pose.position.y
        
        # Generate a straight-line path (this could be replaced with a more sophisticated path)
        ref_path = []
        for i in range(self.horizon):
            alpha = float(i) / float(self.horizon - 1)
            x = x0 + alpha * (xg - x0)
            y = y0 + alpha * (yg - y0)
            ref_path.append((x, y))
            
        return ref_path
    
    def get_current_state(self):
        """Extract the current vehicle state from ROS messages"""
        if not self.current_pose:
            return None
            
        # Extract position
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        # Extract orientation (yaw) from quaternion
        qx = self.current_pose.pose.orientation.x
        qy = self.current_pose.pose.orientation.y
        qz = self.current_pose.pose.orientation.z
        qw = self.current_pose.pose.orientation.w
        
        # Convert quaternion to Euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [x, y, yaw, self.current_velocity]
    
    def plan(self, initial_state, ref_path):
        """MPC-based planning algorithm"""
        x = cp.Variable(self.horizon)
        y = cp.Variable(self.horizon)
        yaw = cp.Variable(self.horizon)
        v = cp.Variable(self.horizon)
        delta = cp.Variable(self.horizon - 1)
        a = cp.Variable(self.horizon - 1)

        cost = 0
        constraints = []

        # Initial state
        x0, y0, yaw0, v0 = initial_state
        constraints += [x[0] == x0, y[0] == y0, yaw[0] == yaw0, v[0] == v0]

        for t in range(self.horizon - 1):
            cost += cp.square(x[t] - ref_path[t][0]) + cp.square(y[t] - ref_path[t][1])
            cost += cp.square(a[t]) + cp.square(delta[t])
            cost += cp.square(v[t] - 5.0)  # prefer constant speed

            # Kinematic model constraints
            constraints += [
                x[t + 1] == x[t] + v[t] * cp.cos(yaw[t]) * self.dt,
                y[t + 1] == y[t] + v[t] * cp.sin(yaw[t]) * self.dt,
                yaw[t + 1] == yaw[t] + v[t] / self.wheelbase * delta[t] * self.dt,
                v[t + 1] == v[t] + a[t] * self.dt,
                cp.abs(delta[t]) <= 0.5,
                cp.abs(a[t]) <= 1.0
            ]

        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP)
            
            if prob.status != cp.OPTIMAL:
                self.get_logger().warning("MPC failed to find optimal trajectory.")
                return None
                
            return [(x.value[t], y.value[t], yaw.value[t], v.value[t]) for t in range(self.horizon)]
        except Exception as e:
            self.get_logger().error(f"Error solving MPC: {e}")
            return None
    
    def trajectory_to_path_msg(self, trajectory):
        """Convert trajectory to ROS Path message"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame_id
        
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            
            # Convert yaw to quaternion
            yaw = point[2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path_msg.poses.append(pose)
            
        return path_msg
    
    def trajectory_to_cmd_vel(self, trajectory):
        """Extract control commands from trajectory for the first timestep"""
        if not trajectory or len(trajectory) < 2:
            return None
            
        # Extract current and next state
        current = trajectory[0]
        next_state = trajectory[1]
        
        # Calculate commands
        dt = self.dt
        dx = next_state[0] - current[0]
        dy = next_state[1] - current[1]
        dyaw = next_state[2] - current[2]
        
        # Linear velocity
        v = math.sqrt(dx**2 + dy**2) / dt
        
        # Angular velocity
        omega = dyaw / dt
        
        # Create Twist message
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        
        return cmd
    
    def planning_callback(self):
        """Main planning loop"""
        # Check if we have all necessary data
        if not self.current_pose or not self.goal_pose or not self.map_data:
            return
            
        # Get current state
        current_state = self.get_current_state()
        if not current_state:
            return
            
        # Generate reference path
        ref_path = self.generate_reference_path()
        if not ref_path:
            return
            
        # Run MPC planner
        start_time = time.time()
        trajectory = self.plan(current_state, ref_path)
        planning_time = time.time() - start_time
        
        if not trajectory:
            self.get_logger().warning("Failed to generate trajectory")
            return
            
        # Publish results
        path_msg = self.trajectory_to_path_msg(trajectory)
        self.trajectory_pub.publish(path_msg)
        
        cmd_vel = self.trajectory_to_cmd_vel(trajectory)
        if cmd_vel:
            self.cmd_vel_pub.publish(cmd_vel)
            
        self.get_logger().debug(f"Planning completed in {planning_time:.3f} seconds")


def main(args=None):
    rclpy.init(args=args)
    node = MPCPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 