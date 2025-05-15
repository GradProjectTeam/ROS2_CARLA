#!/usr/bin/env python3
# MPC Trajectory Controller Node
# Follows a trajectory using Model Predictive Control

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import numpy as np
import math
import time
import casadi as ca
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class MPCTrajectoryController(Node):
    def __init__(self):
        super().__init__('mpc_trajectory_controller')
        
        # Declare parameters
        self.declare_parameter('trajectory_topic', '/planner/hybrid_astar/trajectory')
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('control_rate', 20.0)  # Hz
        
        # MPC parameters
        self.declare_parameter('prediction_horizon', 10)
        self.declare_parameter('control_horizon', 5)
        self.declare_parameter('dt', 0.1)  # Time step for MPC discretization
        self.declare_parameter('max_linear_velocity', 2.0)
        self.declare_parameter('min_linear_velocity', 0.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('min_angular_velocity', -1.0)
        self.declare_parameter('max_linear_acceleration', 1.0)
        self.declare_parameter('max_angular_acceleration', 0.5)
        self.declare_parameter('wheelbase', 2.7)  # Vehicle wheelbase in meters
        
        # MPC weights
        self.declare_parameter('w_x', 1.0)  # Weight for x position tracking
        self.declare_parameter('w_y', 1.0)  # Weight for y position tracking
        self.declare_parameter('w_theta', 0.5)  # Weight for heading tracking
        self.declare_parameter('w_v', 0.1)  # Weight for velocity tracking
        self.declare_parameter('w_linear_rate', 0.1)  # Weight for minimizing linear acceleration
        self.declare_parameter('w_angular_rate', 0.1)  # Weight for minimizing angular acceleration
        
        # Get parameters
        self.trajectory_topic = self.get_parameter('trajectory_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # Get MPC parameters
        self.N = self.get_parameter('prediction_horizon').value
        self.control_horizon = self.get_parameter('control_horizon').value
        self.dt = self.get_parameter('dt').value
        self.max_v = self.get_parameter('max_linear_velocity').value
        self.min_v = self.get_parameter('min_linear_velocity').value
        self.max_omega = self.get_parameter('max_angular_velocity').value
        self.min_omega = self.get_parameter('min_angular_velocity').value
        self.max_linear_accel = self.get_parameter('max_linear_acceleration').value
        self.max_angular_accel = self.get_parameter('max_angular_acceleration').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # Get MPC weights
        self.w_x = self.get_parameter('w_x').value
        self.w_y = self.get_parameter('w_y').value
        self.w_theta = self.get_parameter('w_theta').value
        self.w_v = self.get_parameter('w_v').value
        self.w_linear_rate = self.get_parameter('w_linear_rate').value
        self.w_angular_rate = self.get_parameter('w_angular_rate').value
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            self.trajectory_topic,
            self.trajectory_callback,
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile
        )
        
        # Publisher
        self.control_pub = self.create_publisher(
            Twist,
            self.control_topic,
            qos_profile
        )
        
        # State variables
        self.current_trajectory = None
        self.current_odom = None
        self.last_control_time = None
        self.last_v = 0.0
        self.last_omega = 0.0
        
        # Initialize MPC solver
        self.initialize_mpc()
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_callback
        )
        
        self.get_logger().info('MPC Trajectory Controller initialized')
        self.get_logger().info(f'Subscribing to trajectory topic: {self.trajectory_topic}')
        self.get_logger().info(f'Publishing control commands to: {self.control_topic}')
    
    def initialize_mpc(self):
        """Initialize the MPC optimization problem using CasADi"""
        # State variables [x, y, theta, v]
        self.nx = 4
        # Control variables [v_cmd, omega_cmd]
        self.nu = 2
        
        # Create CasADi symbols for the MPC problem
        self.x = ca.SX.sym('x', self.nx)  # State: [x, y, theta, v]
        self.u = ca.SX.sym('u', self.nu)  # Control: [v_cmd, omega_cmd]
        
        # System dynamics (bicycle model)
        # x_dot = v * cos(theta)
        # y_dot = v * sin(theta)
        # theta_dot = v * tan(steering) / wheelbase = omega
        # v_dot = a (acceleration to be determined by the controller)
        
        # Define system dynamics
        self.x_next = ca.vertcat(
            self.x[0] + self.dt * self.x[3] * ca.cos(self.x[2]),  # x position
            self.x[1] + self.dt * self.x[3] * ca.sin(self.x[2]),  # y position
            self.x[2] + self.dt * self.u[1],                       # theta (using omega directly)
            self.u[0]                                              # v (direct control input for simplicity)
        )
        
        # Create the system dynamics function
        self.f = ca.Function('f', [self.x, self.u], [self.x_next], ['x', 'u'], ['x_next'])
        
        # Decision variables for the full horizon
        self.opt_x = ca.SX.sym('opt_x', self.nx, self.N+1)  # States over horizon
        self.opt_u = ca.SX.sym('opt_u', self.nu, self.N)    # Controls over horizon
        
        # Reference trajectory placeholders
        self.ref_x = ca.SX.sym('ref_x', self.nx, self.N+1)
        
        # Initial state (to be updated each iteration)
        self.x0 = ca.SX.sym('x0', self.nx)
        
        # Previous control for rate limiting
        self.u_prev = ca.SX.sym('u_prev', self.nu)
        
        # Initialize objective and constraints
        obj = 0
        g = []  # Constraints
        
        # Initial state constraint
        g.append(self.opt_x[:, 0] - self.x0)
        
        # Construct the MPC problem
        for k in range(self.N):
            # State cost (tracking error)
            state_error = self.opt_x[:, k] - self.ref_x[:, k]
            obj += (
                self.w_x * state_error[0]**2 +    # x position error
                self.w_y * state_error[1]**2 +    # y position error
                self.w_theta * self.normalize_angle_casadi(state_error[2])**2 +  # heading error
                self.w_v * state_error[3]**2      # velocity error
            )
            
            # Control rate cost (smoothness)
            if k == 0:
                u_rate = self.opt_u[:, k] - self.u_prev
            else:
                u_rate = self.opt_u[:, k] - self.opt_u[:, k-1]
            
            obj += (
                self.w_linear_rate * u_rate[0]**2 +   # Linear velocity rate
                self.w_angular_rate * u_rate[1]**2    # Angular velocity rate
            )
            
            # System dynamics constraint
            x_next = self.f(x=self.opt_x[:, k], u=self.opt_u[:, k])
            g.append(self.opt_x[:, k+1] - x_next)
            
            # Control constraints
            g.append(self.opt_u[0, k] - self.max_v)  # Max linear velocity
            g.append(self.min_v - self.opt_u[0, k])  # Min linear velocity
            g.append(self.opt_u[1, k] - self.max_omega)  # Max angular velocity
            g.append(self.min_omega - self.opt_u[1, k])  # Min angular velocity
            
            # Control rate constraints
            if k == 0:
                g.append(self.opt_u[0, k] - self.u_prev[0] - self.max_linear_accel * self.dt)
                g.append(self.u_prev[0] - self.opt_u[0, k] - self.max_linear_accel * self.dt)
                g.append(self.opt_u[1, k] - self.u_prev[1] - self.max_angular_accel * self.dt)
                g.append(self.u_prev[1] - self.opt_u[1, k] - self.max_angular_accel * self.dt)
            else:
                g.append(self.opt_u[0, k] - self.opt_u[0, k-1] - self.max_linear_accel * self.dt)
                g.append(self.opt_u[0, k-1] - self.opt_u[0, k] - self.max_linear_accel * self.dt)
                g.append(self.opt_u[1, k] - self.opt_u[1, k-1] - self.max_angular_accel * self.dt)
                g.append(self.opt_u[1, k-1] - self.opt_u[1, k] - self.max_angular_accel * self.dt)
        
        # Terminal cost (optional, can be higher weight)
        state_error = self.opt_x[:, self.N] - self.ref_x[:, self.N]
        obj += (
            2.0 * self.w_x * state_error[0]**2 +    # x position error
            2.0 * self.w_y * state_error[1]**2 +    # y position error
            2.0 * self.w_theta * self.normalize_angle_casadi(state_error[2])**2 +  # heading error
            2.0 * self.w_v * state_error[3]**2      # velocity error
        )
        
        # Create optimization variables
        opt_vars = ca.vertcat(
            ca.reshape(self.opt_x, -1, 1),  # Reshape to column vector
            ca.reshape(self.opt_u, -1, 1)   # Reshape to column vector
        )
        
        # Convert constraints to column vector
        g = ca.vertcat(*g)
        
        # Create the NLP problem
        nlp = {
            'x': opt_vars,
            'f': obj,
            'g': g,
            'p': ca.vertcat(
                ca.reshape(self.x0, -1, 1),
                ca.reshape(self.ref_x, -1, 1),
                self.u_prev
            )
        }
        
        # Solver options
        opts = {
            'ipopt': {
                'max_iter': 100,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        
        # Create solver
        self.solver = ca.nlpsol('solver', 'ipopt', nlp, opts)
        
        # Calculate the number of constraints and variables for bounds setup
        self.n_states = self.nx * (self.N + 1)
        self.n_controls = self.nu * self.N
        self.n_opt_vars = self.n_states + self.n_controls
        
        # Constraints upper and lower bounds
        # For dynamics and initial constraints: equality constraints (bounds are 0)
        # For control constraints: inequality constraints
        
        # Initialize bounds at the right sizes
        self.lbg = []
        self.ubg = []
        
        # Initial state constraints (equality)
        self.lbg.extend([0.0] * self.nx)
        self.ubg.extend([0.0] * self.nx)
        
        # System dynamics constraints (equality)
        for _ in range(self.N):
            self.lbg.extend([0.0] * self.nx)
            self.ubg.extend([0.0] * self.nx)
            
            # Control constraints (inequality)
            self.lbg.extend([-float('inf')] * 4)  # v_max, -v_min, omega_max, -omega_min
            self.ubg.extend([0.0] * 4)
            
            # Control rate constraints (inequality)
            self.lbg.extend([-float('inf')] * 4)  # Max rates
            self.ubg.extend([0.0] * 4)
        
        # Convert to numpy arrays for easier handling
        self.lbg = np.array(self.lbg)
        self.ubg = np.array(self.ubg)
        
        # Initialize solution
        self.x_sol = np.zeros((self.nx, self.N+1))
        self.u_sol = np.zeros((self.nu, self.N))
    
    def normalize_angle_casadi(self, angle):
        """Normalize angle to [-pi, pi] using CasADi functions"""
        return ca.fmod(angle + ca.pi, 2*ca.pi) - ca.pi
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def trajectory_callback(self, msg):
        """Process incoming trajectory message"""
        if len(msg.points) == 0:
            self.get_logger().warn('Received empty trajectory')
            return
        
        self.current_trajectory = msg
        self.get_logger().debug(f'Received trajectory with {len(msg.points)} points')
    
    def odom_callback(self, msg):
        """Process incoming odometry message"""
        self.current_odom = msg
    
    def get_reference_trajectory(self, current_x, current_y, current_heading, current_velocity):
        """Extract reference trajectory from the current trajectory message"""
        if self.current_trajectory is None or len(self.current_trajectory.points) == 0:
            return None
        
        # Find the closest point on the trajectory
        min_dist = float('inf')
        closest_idx = 0
        
        for i, point in enumerate(self.current_trajectory.points):
            x = point.positions[0]
            y = point.positions[1]
            dist = math.sqrt((x - current_x)**2 + (y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Extract reference trajectory for the prediction horizon
        ref_traj = np.zeros((self.nx, self.N+1))
        
        for i in range(self.N+1):
            idx = closest_idx + i
            if idx < len(self.current_trajectory.points):
                point = self.current_trajectory.points[idx]
                ref_traj[0, i] = point.positions[0]  # x
                ref_traj[1, i] = point.positions[1]  # y
                ref_traj[2, i] = point.positions[2]  # theta (heading)
                ref_traj[3, i] = point.velocities[3]  # v (linear velocity)
            else:
                # If we've reached the end of the trajectory, use the last point
                point = self.current_trajectory.points[-1]
                ref_traj[0, i] = point.positions[0]
                ref_traj[1, i] = point.positions[1]
                ref_traj[2, i] = point.positions[2]
                ref_traj[3, i] = 0.0  # Stop at the end of the trajectory
        
        return ref_traj
    
    def control_callback(self):
        """Main control loop that runs at control_rate frequency"""
        if self.current_trajectory is None or self.current_odom is None:
            self.get_logger().debug('Waiting for trajectory and odometry data')
            return
        
        try:
            # Get current state from odometry
            current_pose = self.current_odom.pose.pose
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            
            # Extract heading from quaternion
            qx = current_pose.orientation.x
            qy = current_pose.orientation.y
            qz = current_pose.orientation.z
            qw = current_pose.orientation.w
            current_heading = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            
            # Get current velocity
            current_twist = self.current_odom.twist.twist
            current_velocity = current_twist.linear.x
            
            # Current state
            x0 = np.array([current_x, current_y, current_heading, current_velocity])
            
            # Get reference trajectory
            ref_traj = self.get_reference_trajectory(current_x, current_y, current_heading, current_velocity)
            
            if ref_traj is None:
                self.get_logger().warn('Failed to extract reference trajectory')
                self.stop_vehicle()
                return
            
            # Previous control values
            u_prev = np.array([self.last_v, self.last_omega])
            
            # Prepare parameters for the solver
            p = np.concatenate([
                x0.flatten(),
                ref_traj.flatten(),
                u_prev
            ])
            
            # Initialize optimization variables
            x_init = np.zeros((self.n_opt_vars, 1))
            
            # Initial guess for states: linearly interpolate from current state to reference
            for i in range(self.N+1):
                x_init[i*self.nx:(i+1)*self.nx] = (
                    x0 * (self.N-i)/self.N + ref_traj[:, i] * i/self.N
                ).reshape(-1, 1)
            
            # Initial guess for controls: last applied control
            for i in range(self.N):
                x_init[self.n_states + i*self.nu:self.n_states + (i+1)*self.nu] = u_prev.reshape(-1, 1)
            
            # Solve optimization problem
            solution = self.solver(
                x0=x_init,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p
            )
            
            # Extract solution
            x_opt = np.array(solution['x'])
            
            # Reshape solution
            self.x_sol = x_opt[:self.n_states].reshape(self.nx, self.N+1, order='F')
            self.u_sol = x_opt[self.n_states:].reshape(self.nu, self.N, order='F')
            
            # Apply the first control
            v_cmd = float(self.u_sol[0, 0])
            omega_cmd = float(self.u_sol[1, 0])
            
            # Store for next iteration
            self.last_v = v_cmd
            self.last_omega = omega_cmd
            
            # Create and publish control command
            cmd = Twist()
            cmd.linear.x = v_cmd
            cmd.angular.z = omega_cmd
            
            self.control_pub.publish(cmd)
            
            self.get_logger().debug(f'MPC Control: linear={v_cmd:.2f}, angular={omega_cmd:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error in MPC control: {str(e)}')
            self.stop_vehicle()
    
    def stop_vehicle(self):
        """Send a command to stop the vehicle"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.control_pub.publish(cmd)
        
        # Reset control history
        self.last_v = 0.0
        self.last_omega = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = MPCTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 