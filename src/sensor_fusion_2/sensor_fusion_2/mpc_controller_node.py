#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import socket
import sys
import os
import time
import threading
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import Int32MultiArray, Float32, Bool
import casadi as ca
import logging
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Imu  # Added IMU import

# Remove external import
# from sensor_fusion_2.sensor_fusion_2.Control_standalone_mpc_controller import MPCController

# Add vehicle model and MPC controller classes directly
class VehicleModel:
    def __init__(self):
        # Tesla Model 3 parameters
        self.m = 1847  # mass [kg] (Model 3 curb weight)
        self.Iz = 2500  # yaw moment of inertia [kg*m^2] (estimated based on dimensions)
        self.lf = 1.58  # distance from CG to front axle [m] (based on wheelbase 2.875m and weight distribution)
        self.lr = 1.295  # distance from CG to rear axle [m]
        self.caf = 80000  # front tire cornering stiffness [N/rad] (typical for performance tires)
        self.car = 80000  # rear tire cornering stiffness [N/rad]
        
        # State variables: [x, y, psi, vx, vy, omega]
        self.n_states = 6
        # Control inputs: [acceleration, steering]
        self.n_controls = 2
        
    def get_dynamics(self, state, control):
        """Calculate vehicle dynamics"""
        # Extract states
        x = state[0]
        y = state[1]
        psi = state[2]
        vx = state[3]
        vy = state[4]
        omega = state[5]
        
        # Extract controls
        acc = control[0]
        delta = control[1]
        
        # Small positive value to prevent division by zero
        eps = 1e-6
        
        # Calculate slip angles
        alpha_f = ca.atan2(vy + self.lf * omega, vx + eps) - delta
        alpha_r = ca.atan2(vy - self.lr * omega, vx + eps)
        
        # Calculate lateral forces
        Fyf = -self.caf * alpha_f
        Fyr = -self.car * alpha_r
        
        # State derivatives
        dx = vx * ca.cos(psi) - vy * ca.sin(psi)
        dy = vx * ca.sin(psi) + vy * ca.cos(psi)
        dpsi = omega
        dvx = acc + omega * vy
        dvy = (Fyf * ca.cos(delta) + Fyr) / self.m - omega * vx
        domega = (self.lf * Fyf * ca.cos(delta) - self.lr * Fyr) / self.Iz
        
        return ca.vertcat(dx, dy, dpsi, dvx, dvy, domega)

# Rename to avoid name conflict with the Node class
class MPCControllerImpl:
    def __init__(self, horizon=15, dt=0.1):
        """
        Initialize the MPC controller
        Args:
            horizon: Prediction horizon
            dt: Time step [s]
        """
        self.horizon = horizon
        self.dt = dt
        self.vehicle = VehicleModel()

        # Control limits for Tesla Model 3
        self.max_acc = 20.0  # Increased from 15.0 to 20.0 m/s^2 for even higher acceleration
        self.max_steer = np.deg2rad(50)  # Increased from 45 to 50 degrees for even more aggressive steering
        
        # Steering normalization factor (to convert from radians to normalized [-1,1])
        self.steer_norm_factor = 0.9  # Increased from 0.8 to 0.9 for even more responsive steering

        # Cost function weights - dramatically increased heading weight to force curve following
        self.Q = np.diag([20, 20, 300, 1, 1, 15])  # Dramatically increased heading weight (index 2) from 200 to 300, and omega weight from 10 to 15
        self.R = np.diag([0.2, 0.8])  # Further reduced steering weight from 1.0 to 0.8 and acceleration weight from 0.3 to 0.2 for more aggressive control

        self.prev_control = np.zeros(2)
        self.prev_steering = 0.0  # Store previous steering for smoothing

    def normalize_steering(self, steering_rad):
        """Convert steering angle from radians to normalized [-1,1] range with smooth deadzone"""
        # Apply deadzone
        if abs(steering_rad) < np.deg2rad(0.05):  # Reduced deadzone from 0.1 to 0.05 degree for even more responsive steering
            return 0.0
            
        # Apply smooth normalization
        normalized = steering_rad * self.steer_norm_factor
        
        # Apply smoothing
        if hasattr(self, 'prev_steering'):
            smoothing_factor = 0.9  # Increased from 0.8 to 0.9 for even more responsive steering (90% new, 10% previous)
            normalized = smoothing_factor * normalized + (1.0 - smoothing_factor) * self.prev_steering
            self.prev_steering = normalized
            
        return np.clip(normalized, -1.0, 1.0)

    def get_control(self, current_state, reference_trajectory):
        """
        Calculate optimal control inputs using MPC
        Args:
            current_state: Current vehicle state [x, y, psi, vx, vy, omega]
            reference_trajectory: List of reference states
        Returns:
            Dictionary with acceleration, steering, and brake values
        """
        try:
            # Find the closest point in the trajectory to the current position
            current_pos = np.array([current_state[0], current_state[1]])
            
            # Optimize for large trajectory sets
            if len(reference_trajectory) > 100:
                # If we have many waypoints, use a more efficient approach
                # Start by checking a window around the first few points
                search_window = reference_trajectory[:min(20, len(reference_trajectory))]
                distances = []
                for ref_state in search_window:
                    ref_pos = np.array([ref_state[0], ref_state[1]])
                    dist = np.linalg.norm(ref_pos - current_pos)
                    distances.append(dist)
                
                window_closest_idx = np.argmin(distances)
                min_dist = distances[window_closest_idx]
                
                # If the closest point is near the end of our window, search more
                if window_closest_idx > 15:
                    # Extend search to more of the trajectory
                    extended_window = reference_trajectory[:min(50, len(reference_trajectory))]
                    distances = []
                    for ref_state in extended_window:
                        ref_pos = np.array([ref_state[0], ref_state[1]])
                        dist = np.linalg.norm(ref_pos - current_pos)
                        distances.append(dist)
                    closest_idx = np.argmin(distances)
                else:
                    closest_idx = window_closest_idx
            else:
                # For smaller trajectory sets, calculate all distances
                distances = []
                for ref_state in reference_trajectory:
                    ref_pos = np.array([ref_state[0], ref_state[1]])
                    dist = np.linalg.norm(ref_pos - current_pos)
                    distances.append(dist)
                closest_idx = np.argmin(distances)
            
            # For large trajectories, use a shorter lookahead distance to focus on immediate path
            # This helps the controller follow curves better
            if len(reference_trajectory) > 50:
                lookahead_distance = 1.5  # Reduced from 2.0 to 1.5 meters to focus even more on immediate path
            else:
                lookahead_distance = 0.8  # Reduced from 1.0 to 0.8 meters for even more immediate steering response
            
            # Create horizon trajectory with focus on near waypoints
            horizon_trajectory = []
            
            # Start with the closest point for more precise path following
            # This helps the controller follow curves better
            start_idx = closest_idx
            
            # Add the closest point first for more precise path following
            horizon_trajectory.append(reference_trajectory[start_idx])
            
            # For large trajectory sets, include more points in the horizon
            # to improve planning and smoothness
            if len(reference_trajectory) > 50:
                # Use more points for the horizon with smaller spacing for better curve following
                for i in range(1, min(self.horizon, len(reference_trajectory) - start_idx)):
                    next_idx = start_idx + i
                    if next_idx < len(reference_trajectory):
                        horizon_trajectory.append(reference_trajectory[next_idx])
                    else:
                        # If we run out of points, use the last point
                        horizon_trajectory.append(reference_trajectory[-1])
            else:
                # For local waypoint following with fewer points, prioritize the next few points
                for i in range(1, min(self.horizon, len(reference_trajectory) - start_idx)):
                    next_idx = start_idx + i
                    if next_idx < len(reference_trajectory):
                        horizon_trajectory.append(reference_trajectory[next_idx])
                    else:
                        # If we run out of points, use the last point
                        horizon_trajectory.append(reference_trajectory[-1])
            
            # If we still need more points to fill the horizon, repeat the last point
            while len(horizon_trajectory) < self.horizon:
                horizon_trajectory.append(reference_trajectory[-1])
            
            # Setup optimization problem
            opti = ca.Opti()
            X = opti.variable(self.vehicle.n_states, self.horizon + 1)
            U = opti.variable(self.vehicle.n_controls, self.horizon)

            # Initial state constraint
            opti.subject_to(X[:, 0] == current_state)

            # System dynamics constraints
            for k in range(self.horizon):
                x_next = X[:, k] + self.dt * self.vehicle.get_dynamics(X[:, k], U[:, k])
                opti.subject_to(X[:, k + 1] == x_next)

            # Control input constraints
            opti.subject_to(opti.bounded(-self.max_acc, U[0, :], self.max_acc))
            opti.subject_to(opti.bounded(-self.max_steer, U[1, :], self.max_steer))

            # Objective function with enhanced weights for local waypoint following
            cost = 0
            for k in range(self.horizon):
                state_error = X[:, k] - horizon_trajectory[k]
                
                # For local waypoint following, prioritize position and heading errors
                # in the first few steps of the horizon
                position_weight = self.Q[0, 0] * (1.0 + 8.0 / (k + 1))  # Increased from 5.0 to 8.0 for more immediate position following
                heading_weight = self.Q[2, 2] * (1.0 + 12.0 / (k + 1))   # Increased from 8.0 to 12.0 for much more immediate heading following
                
                # Create a weighted error vector that emphasizes position and heading
                weighted_error = state_error.copy()
                weighted_error[0] = state_error[0] * position_weight / self.Q[0, 0]  # x position
                weighted_error[1] = state_error[1] * position_weight / self.Q[1, 1]  # y position
                weighted_error[2] = state_error[2] * heading_weight / self.Q[2, 2]   # heading
                
                cost += ca.mtimes([weighted_error.T, self.Q, weighted_error])
                control_error = U[:, k] - self.prev_control
                cost += ca.mtimes([control_error.T, self.R, control_error])

            # Higher terminal cost to ensure we reach the end of the horizon
            terminal_error = X[:, -1] - horizon_trajectory[-1]
            cost += ca.mtimes([terminal_error.T, 15 * self.Q, terminal_error])  # Increased from 10 to 15 times

            opti.minimize(cost)
            p_opts = {"expand": True}
            s_opts = {
                "max_iter": 300,
                "tol": 1e-3,
                "print_level": 0,
                "acceptable_tol": 1e-2,
                "acceptable_obj_change_tol": 1e-2
            }
            opti.solver('ipopt', p_opts, s_opts)

            try:
                sol = opti.solve()
                optimal_control = sol.value(U[:, 0])
                print(f"Optimization successful. Control: {optimal_control}")
            except Exception as e:
                print(f"Optimization failed: {str(e)}")
                # Enhanced fallback control for local waypoint following
                error = horizon_trajectory[0] - current_state
                
                # Calculate a more precise steering angle based on position error
                # and current heading
                dx = horizon_trajectory[0][0] - current_state[0]
                dy = horizon_trajectory[0][1] - current_state[1]
                target_heading = np.arctan2(dy, dx)
                heading_error = target_heading - current_state[2]
                
                # Normalize heading error to [-pi, pi]
                while heading_error > np.pi:
                    heading_error -= 2 * np.pi
                while heading_error < -np.pi:
                    heading_error += 2 * np.pi
                    
                # Calculate steering based on heading error and distance - MORE AGGRESSIVE
                distance = np.sqrt(dx*dx + dy*dy)
                # Increase steering response for sharper turns
                steering_rad = np.clip(heading_error * (1.5 + 2.0/max(0.1, distance)), -self.max_steer, self.max_steer)  # Increased from 1.2 + 1.5/distance to 1.5 + 2.0/distance
                
                # Apply the steering
                steering = self.normalize_steering(steering_rad)
                print(f"Using enhanced fallback control: {steering}")
                optimal_control = np.array([0.5, steering_rad])  # Increased from 0.3 to 0.5 for higher acceleration with calculated steering

            # Process the control outputs
            acceleration = np.clip(optimal_control[0] / self.max_acc, -1, 1)
            steering_rad = optimal_control[1]
            steering = self.normalize_steering(steering_rad)
            
            # For higher speeds, use more aggressive acceleration and gentler braking
            brake = max(0, -acceleration * 0.5) if acceleration < 0 else 0.0  # Reduced from 0.7 to 0.5 for gentler braking
            acceleration = max(0, acceleration * 1.5)  # Increased from 1.2 to 1.5 for even more aggressive acceleration

            self.prev_control = optimal_control

            return {
                'acceleration': acceleration,
                'steering': steering,
                'brake': brake
            }

        except Exception as e:
            print(f"Error in MPC control: {str(e)}")
            return {
                'acceleration': 0.5,  # Increased from 0.2 to 0.5 for higher acceleration as fallback
                'steering': 0.0,
                'brake': 0.0
            }

    def get_control_limits(self, vx):
        max_acc = min(5.0, 0.4 * vx)  # Increased from 4.0 to 5.0 and from 0.3 to 0.4
        max_steer = min(np.deg2rad(45), np.deg2rad(30) * (10 / max(1.0, vx)))  # Increased from 40 to 45 and from 20 to 30
        return max_acc, max_steer

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Declare parameters
        self.declare_parameter('tcp_port', 12344)
        self.declare_parameter('tcp_host', '0.0.0.0')
        self.declare_parameter('update_rate', 60.0)  # Hz
        self.declare_parameter('reconnect_interval', 1.0)  # seconds
        self.declare_parameter('max_reconnect_attempts', 10)
        self.declare_parameter('path_topic', '/dwa/path')
        self.declare_parameter('use_waypoints', True)  # Use waypoints as fallback
        self.declare_parameter('waypoints_topic', '/carla/waypoints')
        self.declare_parameter('debug_level', 'info')  # Logging level: debug, info, warn, error
        self.declare_parameter('test_mode', False)  # Enable test mode for straight line movement
        self.declare_parameter('test_throttle', 0.3)  # Throttle value for test mode [0.0-1.0]
        self.declare_parameter('test_duration', 5.0)  # Duration of test in seconds
        self.declare_parameter('cmd_vel_topic', '/mpc/cmd_vel')
        self.declare_parameter('dwa_cmd_vel_topic', '/dwa/cmd_vel')
        self.declare_parameter('obstacle_detected_topic', '/dwa/obstacle_detected')
        self.declare_parameter('direct_dwa_connection', True)
        self.declare_parameter('bypass_trajectory_receiver', True)  # Default to True since we use path directly
        self.declare_parameter('min_path_points', 2)  # Minimum number of points to consider a valid path
        self.declare_parameter('path_update_timeout', 2.0)  # Path is considered stale after this many seconds
        self.declare_parameter('velocity_topic', '/carla/ego_vehicle/velocity')
        self.declare_parameter('vehicle_frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'local_map_link')
        self.declare_parameter('fallback_throttle', 0.9)  # Default throttle when no path is available
        self.declare_parameter('use_imu', True)  # Enable IMU integration by default
        self.declare_parameter('imu_topic', '/carla/ego_vehicle/imu')
        self.declare_parameter('max_speed', 20.0)  # Maximum speed in m/s
        self.declare_parameter('curve_detection_threshold', 3.0)  # Threshold in degrees for curve detection
        self.declare_parameter('min_throttle', 0.1)  # Minimum throttle during gradual stop
        self.declare_parameter('max_throttle', 0.8)  # Maximum throttle on straight segments
        self.declare_parameter('max_steering', 0.9)  # Maximum steering value
        self.declare_parameter('dwa_stop_threshold', 0.3)  # Threshold for detecting DWA stop command
        self.declare_parameter('emergency_brake_force', 0.9)  # Strong brake force when obstacle detected
        self.declare_parameter('obstacle_stop_timeout', 3.0)  # How long to remain stopped after obstacle detection
        self.declare_parameter('obstacle_resume_threshold', 0.5)  # Speed to resume after obstacle is cleared
        self.declare_parameter('prioritize_dwa_commands', True)  # Always prioritize DWA commands for safety
        self.declare_parameter('waypoint_obstacle_response', True)  # Enable specific response to obstacles on waypoint lane
        
        # Get parameters
        self.tcp_port = self.get_parameter('tcp_port').value
        self.tcp_host = self.get_parameter('tcp_host').value
        self.update_rate = self.get_parameter('update_rate').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value
        self.path_topic = self.get_parameter('path_topic').value
        self.use_waypoints = self.get_parameter('use_waypoints').value
        self.waypoints_topic = self.get_parameter('waypoints_topic').value
        self.debug_level = self.get_parameter('debug_level').value
        self.test_mode = self.get_parameter('test_mode').value
        self.test_throttle = self.get_parameter('test_throttle').value
        self.test_duration = self.get_parameter('test_duration').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.dwa_cmd_vel_topic = self.get_parameter('dwa_cmd_vel_topic').value
        self.obstacle_detected_topic = self.get_parameter('obstacle_detected_topic').value
        self.direct_dwa_connection = self.get_parameter('direct_dwa_connection').value
        self.bypass_trajectory_receiver = self.get_parameter('bypass_trajectory_receiver').value
        self.min_path_points = self.get_parameter('min_path_points').value
        self.path_update_timeout = self.get_parameter('path_update_timeout').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.vehicle_frame_id = self.get_parameter('vehicle_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.fallback_throttle = self.get_parameter('fallback_throttle').value
        self.use_imu = self.get_parameter('use_imu').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.max_speed = self.get_parameter('max_speed').value
        self.curve_detection_threshold = self.get_parameter('curve_detection_threshold').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.max_steering = self.get_parameter('max_steering').value
        self.dwa_stop_threshold = self.get_parameter('dwa_stop_threshold').value
        self.emergency_brake_force = self.get_parameter('emergency_brake_force').value
        self.obstacle_stop_timeout = self.get_parameter('obstacle_stop_timeout').value
        self.obstacle_resume_threshold = self.get_parameter('obstacle_resume_threshold').value
        self.prioritize_dwa_commands = self.get_parameter('prioritize_dwa_commands').value
        self.waypoint_obstacle_response = self.get_parameter('waypoint_obstacle_response').value
        
        # Set logging level
        log_level = logging.INFO
        if self.debug_level == 'debug':
            log_level = logging.DEBUG
        elif self.debug_level == 'warn' or self.debug_level == 'warning':
            log_level = logging.WARN
        elif self.debug_level == 'error':
            log_level = logging.ERROR
        
        # Apply logging level to ROS2 logger
        logging.getLogger().setLevel(log_level)
        
        # Initialize controller
        self.controller = MPCControllerImpl(horizon=8, dt=1.0/self.update_rate)
        
        # Client connection
        self.client_socket = None
        self.is_connected = False
        
        # Add velocity feedback variables
        self.current_velocity = 0.0  # m/s
        
        # Create velocity subscriber
        self.velocity_subscription = self.create_subscription(
            Float32,
            self.velocity_topic,
            self.velocity_callback,
            10
        )
        
        # Add direct waypoints subscription
        if self.use_waypoints:
            self.waypoints_subscription = self.create_subscription(
                PoseArray,
                self.waypoints_topic,
                self.waypoints_callback,
                10
            )
            self.raw_waypoints = []
            self.last_waypoints_time = self.get_clock().now()
            self.get_logger().info(f"Subscribing to waypoints at {self.waypoints_topic}")
        
        # Create cmd_vel publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )
        
        # Add path tracking variables
        self.last_path_time = None
        self.path_count = 0
        self.has_valid_path = False
        self.current_path_length = 0
        
        # Improved Path subscriber with better QoS
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscriber for DWA path
        self.path_subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            path_qos
        )
        
        # State variables
        self.current_state = np.zeros(6)  # [x, y, psi, vx, vy, omega]
        self.reference_trajectory = []
        self.has_new_waypoints = False
        self.has_valid_waypoints = False
        
        # Thread control
        self.running = True
        self.connection_thread = None
        
        # Initialize obstacle detection state
        self.obstacle_detected = False
        self.obstacle_detection_time = None
        self.curve_detected = False
        self.distance_to_curve = 0.0
        self.dwa_stop_requested = False  # Flag for DWA stop request
        self.black_lane_detected = False  # Flag for black lane detection
        
        # Start client connection thread
        try:
            self.connection_thread = threading.Thread(target=self.connect_to_server)
            self.connection_thread.daemon = True
            self.connection_thread.start()
            self.get_logger().info("Started TCP connection thread")
        except Exception as e:
            self.get_logger().error(f"Failed to start connection thread: {str(e)}")
        
        # Create timer for control loop
        self.timer = self.create_timer(1.0/self.update_rate, self.control_loop)
        
        # Subscribe to obstacle detection topic
        self.obstacle_detected_subscription = self.create_subscription(
            Bool,
            self.obstacle_detected_topic,
            self.obstacle_detected_callback,
            10
        )
        
        # Create DWA command subscriber if direct connection is enabled
        if self.direct_dwa_connection:
            self.dwa_cmd_subscription = self.create_subscription(
                Twist,
                self.dwa_cmd_vel_topic,
                self.dwa_cmd_callback,
                10
            )
            self.dwa_cmd = None
            self.last_dwa_cmd_time = self.get_clock().now()
            self.get_logger().info(f"Direct DWA connection enabled, subscribing to {self.dwa_cmd_vel_topic}")
        
        # Add IMU data variables
        self.imu_data = None
        self.last_imu_time = None
        
        # Create IMU subscriber if enabled
        if self.use_imu:
            self.imu_subscription = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                10
            )
            self.get_logger().info(f"IMU integration enabled, subscribing to {self.imu_topic}")
        
        self.get_logger().info(f'MPC controller node initialized - Connected to {self.tcp_host}:{self.tcp_port}')
    
    def send_control_commands(self, throttle, steer, brake):
        """Send control commands over TCP"""
        if not self.is_connected or self.client_socket is None:
            self.get_logger().warn("Cannot send control commands: not connected to server")
            # Try to reconnect
            if self.connection_thread and not self.connection_thread.is_alive():
                self.get_logger().info("Restarting connection thread...")
                self.connection_thread = threading.Thread(target=self.connect_to_server)
                self.connection_thread.daemon = True
                self.connection_thread.start()
            return
            
        try:
            # Apply larger deadzone to steering to prevent small oscillations
            if abs(steer) < 0.05:  # Increased from 0.02 to 0.05
                steer = 0.0
            
            # Apply stronger smoothing to steering
            if not hasattr(self, 'prev_sent_steer'):
                self.prev_sent_steer = 0.0
                
            # Apply stronger smoothing (80% previous, 20% new) for more stability
            smoothing_factor = 0.2  # Reduced from 0.9 to 0.2 (more smoothing)
            steer = smoothing_factor * steer + (1.0 - smoothing_factor) * self.prev_sent_steer
            self.prev_sent_steer = steer
            
            # Ensure valid ranges
            throttle = np.clip(throttle, 0.0, 1.0)
            brake = np.clip(brake, 0.0, 1.0)
            steer = np.clip(steer, -self.max_steering, self.max_steering)
            
            # Format and send the control message
            control_msg = f"{throttle:.4f},{steer:.4f},{brake:.4f}\n"
            self.client_socket.send(control_msg.encode())
            
        except Exception as e:
            self.get_logger().error(f"Error sending control commands: {str(e)}")
            self.is_connected = False
    
    def move_straight_line(self, throttle=None, duration=None, steer=0.0):
        """
        Function to move in a straight line for testing purposes
        """
        if throttle is None:
            throttle = self.test_throttle
        
        if duration is None:
            duration = self.test_duration
        
        # Ensure values are within valid ranges
        throttle = np.clip(throttle, 0.0, 1.0)
        steer = np.clip(steer, -1.0, 1.0)
        brake = 0.0  # No braking during the test
        
        # Set up test variables
        self.test_start_time = time.time()
        self.test_running = True
        self.test_end_time = self.test_start_time + duration
        
        self.get_logger().info(f"Starting straight line test: throttle={throttle:.2f}, steer={steer:.2f}, duration={duration:.1f}s")
        
        # Send the control command
        self.send_control_commands(throttle, steer, brake)
        
        # Return the control values used
        return throttle, steer, brake
    
    def detect_curve_in_waypoints(self, waypoints):
        """
        Simple function to detect if there's a curve in the waypoints
        Returns:
            is_curve: True if a curve is detected
            distance_to_curve: Distance to the curve
            curve_direction: 1 for right turn, -1 for left turn, 0 for straight
        """
        if len(waypoints) < 3:
            return False, 0.0, 0
        
        # Calculate distances between consecutive waypoints
        distances = []
        total_distance = 0.0
        
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i].position
            p2 = waypoints[i + 1].position
            dist = np.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            distances.append(dist)
            total_distance += dist
        
        # Calculate angle changes to detect curves
        angles = []
        curve_directions = []
        
        for i in range(1, len(waypoints) - 1):
            p1 = waypoints[i-1].position
            p2 = waypoints[i].position
            p3 = waypoints[i+1].position
            
            # Calculate vectors
            v1 = np.array([p2.x - p1.x, p2.y - p1.y])
            v2 = np.array([p3.x - p2.x, p3.y - p2.y])
            
            # Normalize vectors
            v1_norm = np.linalg.norm(v1)
            v2_norm = np.linalg.norm(v2)
            
            if v1_norm > 0.01 and v2_norm > 0.01:
                v1 = v1 / v1_norm
                v2 = v2 / v2_norm
                
                # Calculate angle between vectors
                dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
                angle = np.arccos(dot_product)
                angles.append(angle)
                
                # Determine curve direction using cross product
                # Positive: right turn, Negative: left turn
                cross_z = v1[0]*v2[1] - v1[1]*v2[0]
                curve_directions.append(np.sign(cross_z))
        
        if not angles:
            return False, 0.0, 0
        
        max_angle = max(angles)
        max_angle_deg = np.rad2deg(max_angle)
        
        # Determine if there's a curve using the threshold parameter
        is_curve = max_angle_deg > self.curve_detection_threshold
        
        # Calculate distance to curve
        distance_to_curve = 0.0
        curve_direction = 0
        
        if is_curve:
            # Find the index of the maximum angle
            max_angle_idx = angles.index(max_angle)
            
            # Calculate distance to the curve
            distance_to_curve = sum(distances[:max_angle_idx+1])
            
            # Determine curve direction
            curve_direction = curve_directions[max_angle_idx]
            
            self.get_logger().info(f"Curve detected: {max_angle_deg:.1f}° at {distance_to_curve:.1f}m, direction: {'right' if curve_direction > 0 else 'left'}")
        
        return is_curve, distance_to_curve, curve_direction
    
    def calculate_gradual_stop_throttle(self, distance_to_curve, current_velocity):
        """
        Calculate throttle for gradual stop based on distance to curve
        """
        # If we're very close to the curve, use minimum throttle
        if distance_to_curve < 1.0:
            return self.min_throttle
        
        # Calculate throttle based on distance to curve
        # Linear interpolation between min_throttle and max_throttle
        throttle_range = self.max_throttle - self.min_throttle
        distance_ratio = min(1.0, distance_to_curve / self.gradual_stop_distance)
        
        # Throttle decreases as we approach the curve
        throttle = self.max_throttle - throttle_range * (1.0 - distance_ratio)
        
        # Adjust based on current velocity
        # If we're moving fast, reduce throttle more aggressively
        if current_velocity > 10.0:
            throttle *= 0.8
        
        return throttle
    
    def control_loop(self):
        """Main MPC control loop"""
        try:
            # Check if test mode is enabled
            if self.test_mode:
                # Use test mode to send simple commands
                throttle = self.test_throttle
                brake = 0.0
                steer = 0.0
                
                # Send control commands over TCP
                self.send_control_commands(throttle, steer, brake)
                
                # Also publish Twist message for ROS integration
                cmd_vel = Twist()
                cmd_vel.linear.x = throttle * 10.0
                cmd_vel.angular.z = steer * 0.5
                
                # Publish the Twist message
                self.cmd_vel_publisher.publish(cmd_vel)
                self.get_logger().info(f"Test mode: throttle={throttle:.3f}, brake={brake:.3f}, steer={steer:.3f}")
                return
            
            # Check for obstacles detected by DWA
            if self.obstacle_detected or self.dwa_stop_requested or self.black_lane_detected:
                # COMPLETE STOP - apply strong brake
                throttle = 0.0
                brake = 0.8  # Strong brake
                steer = 0.0
                
                # Send stop commands
                if self.is_connected:
                    self.send_control_commands(throttle, steer, brake)
                
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0  # Complete stop
                cmd_vel.angular.z = 0.0
                
                reason = "obstacle" if self.obstacle_detected else "black lane" if self.black_lane_detected else "DWA stop request"
                self.get_logger().warn(f"COMPLETE STOP: {reason} detected - throttle={throttle:.2f}, brake={brake:.2f}")
                self.cmd_vel_publisher.publish(cmd_vel)
                return
            
            # Check if we have valid waypoints
            has_waypoints = self.use_waypoints and len(self.raw_waypoints) >= self.min_path_points
            
            if not has_waypoints:
                # If we have no waypoints, use fallback throttle
                throttle = self.fallback_throttle
                brake = 0.0
                steer = 0.0
                
                if self.is_connected:
                    self.send_control_commands(throttle, steer, brake)
                        
                    # Also publish Twist message    
                    cmd_vel = Twist()
                    cmd_vel.linear.x = throttle * 10.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_publisher.publish(cmd_vel)
                    
                self.get_logger().warn(f"No waypoints available, using fallback throttle={throttle:.3f}")
                return
            
            # Detect curves in waypoints
            is_curve = False
            curve_direction = 0  # 0: straight, 1: right turn, -1: left turn
            
            if len(self.raw_waypoints) >= 3:
                # Check for curves by analyzing waypoints
                angles = []
                curve_dirs = []
                
                for i in range(1, len(self.raw_waypoints) - 1):
                    p1 = self.raw_waypoints[i-1].position
                    p2 = self.raw_waypoints[i].position
                    p3 = self.raw_waypoints[i+1].position
                    
                    # Calculate vectors
                    v1 = np.array([p2.x - p1.x, p2.y - p1.y])
                    v2 = np.array([p3.x - p2.x, p3.y - p2.y])
                    
                    # Normalize vectors
                    v1_norm = np.linalg.norm(v1)
                    v2_norm = np.linalg.norm(v2)
                    
                    if v1_norm > 0.01 and v2_norm > 0.01:
                        v1 = v1 / v1_norm
                        v2 = v2 / v2_norm
                        
                        # Calculate angle between vectors
                        dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
                        angle = np.arccos(dot_product)
                        angles.append(angle)
                        
                        # Determine curve direction using cross product
                        cross_z = v1[0]*v2[1] - v1[1]*v2[0]
                        curve_dirs.append(np.sign(cross_z))
                
                if angles:
                    max_angle = max(angles)
                    max_angle_idx = angles.index(max_angle)
                    
                    # If angle is significant, we have a curve
                    if np.rad2deg(max_angle) > self.curve_detection_threshold:
                        is_curve = True
                        curve_direction = curve_dirs[max_angle_idx]
            
            # COMPLETE STOP for curves
            if is_curve:
                self.get_logger().info(f"Curve detected! Direction: {'right' if curve_direction > 0 else 'left'}, angle: {np.rad2deg(max_angle):.1f}° - STOPPING")
                
                # Complete stop with strong brake
                throttle = 0.0
                brake = 0.8  # Strong brake
                steer = 0.0
                
                # Send control commands
                self.send_control_commands(throttle, steer, brake)
                
                # Publish cmd_vel with zero speed
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0  # Complete stop
                cmd_vel.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_vel)
                
                self.get_logger().warn(f"COMPLETE STOP due to curve: throttle={throttle:.3f}, brake={brake:.3f}")
                return
            
            # Straight line following - IMPROVED FOR SMOOTHER CONTROL
            throttle = 0.7  # Good speed for straight lines
            brake = 0.0
            steer = 0.0
            
            # Calculate steering to follow waypoints with smoother control
            if len(self.raw_waypoints) >= 2:
                # Get multiple waypoints for smoother path following
                num_points_to_consider = min(5, len(self.raw_waypoints))
                waypoint_positions = []
                
                for i in range(num_points_to_consider):
                    waypoint_positions.append(np.array([
                        self.raw_waypoints[i].position.x,
                        self.raw_waypoints[i].position.y
                    ]))
                
                # Calculate average direction of the path
                avg_direction_x = 0.0
                avg_direction_y = 0.0
                
                for i in range(1, len(waypoint_positions)):
                    direction = waypoint_positions[i] - waypoint_positions[0]
                    direction_norm = np.linalg.norm(direction)
                    if direction_norm > 0.01:
                        direction = direction / direction_norm
                        avg_direction_x += direction[0]
                        avg_direction_y += direction[1]
                
                if abs(avg_direction_x) > 0.01 or abs(avg_direction_y) > 0.01:
                    # Normalize the average direction
                    direction_norm = np.sqrt(avg_direction_x**2 + avg_direction_y**2)
                    avg_direction_x /= direction_norm
                    avg_direction_y /= direction_norm
                    
                    # Calculate heading based on average direction
                    desired_heading = np.arctan2(avg_direction_y, avg_direction_x)
                    
                    # Calculate cross-track error (lateral deviation)
                    # Use the first two waypoints to define a line
                    if len(waypoint_positions) >= 2:
                        p1 = waypoint_positions[0]
                        p2 = waypoint_positions[1]
                        
                        # Calculate cross-track error (distance from origin to line)
                        # For a line defined by two points p1 and p2, and a point p0 (origin),
                        # the cross-track error is the distance from p0 to the line
                        if np.linalg.norm(p2 - p1) > 0.01:
                            # Vector from p1 to p2
                            v = p2 - p1
                            # Unit vector perpendicular to v
                            perp = np.array([-v[1], v[0]]) / np.linalg.norm(v)
                            # Cross-track error is the dot product of p1 and perp
                            cross_track_error = np.dot(p1, perp)
                            
                            # Adjust desired heading based on cross-track error
                            # This helps to converge to the path
                            cross_track_correction = np.arctan2(0.3 * cross_track_error, 1.0)
                            desired_heading += cross_track_correction
                    
                    # Apply a very gentle proportional control for steering
                    # Reduce the steering gain for smoother control
                    steer = np.clip(0.2 * desired_heading, -0.5, 0.5)
                    
                    # Apply additional smoothing to steering
                    if hasattr(self, 'prev_steer'):
                        # Strong smoothing (70% previous, 30% new)
                        steer = 0.3 * steer + 0.7 * self.prev_steer
                    
                    # Store current steering for next iteration
                    self.prev_steer = steer
            
            # Check if DWA has provided a valid command for straight line
            if self.direct_dwa_connection and hasattr(self, 'dwa_cmd') and self.dwa_cmd:
                dwa_cmd_age = (self.get_clock().now() - self.last_dwa_cmd_time).nanoseconds / 1e9
                if dwa_cmd_age < 0.5:  # Only use recent DWA commands (less than 0.5 seconds old)
                    # Use DWA's linear velocity as a guide for throttle
                    dwa_linear = self.dwa_cmd.linear.x
                    if dwa_linear > 0:
                        # Scale DWA's velocity to throttle (assuming max velocity is around 10 m/s)
                        dwa_throttle = min(0.7, dwa_linear / 10.0)
                        throttle = min(throttle, dwa_throttle)  # Use the lower of the two throttles
                        self.get_logger().debug(f"Using DWA throttle guidance: {dwa_throttle:.3f}")
            
            # Send control commands
            self.send_control_commands(throttle, steer, brake)
            
            # Publish cmd_vel
            cmd_vel = Twist()
            cmd_vel.linear.x = throttle * 10.0
            cmd_vel.angular.z = steer * 0.5
            self.cmd_vel_publisher.publish(cmd_vel)
            
            self.get_logger().info(f"Straight line: throttle={throttle:.3f}, brake={brake:.3f}, steer={steer:.3f}")
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {str(e)}")
            # Send safe values in case of a general error
            if self.is_connected:
                self.send_control_commands(0.0, 0.0, 0.3)  # No throttle, no steering, light brake
            
            # Also publish stop command as Twist
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel)
    
    def waypoints_callback(self, msg):
        """Process waypoints for direct waypoint following"""
        try:
            # Store the raw waypoints
            self.raw_waypoints = msg.poses
            self.last_waypoints_time = self.get_clock().now()
            
            # Log waypoints received
            self.get_logger().debug(f"Received {len(self.raw_waypoints)} waypoints")
            
            # Check for curves in waypoints
            is_curve, distance_to_curve, curve_direction = self.detect_curve_in_waypoints(self.raw_waypoints)
            
            # Store curve information for use in control loop
            self.curve_detected = is_curve
            self.distance_to_curve = distance_to_curve
            self.curve_direction = curve_direction
            
            # If a curve is detected, prepare for gradual stopping
            if is_curve:
                # Calculate throttle reduction based on distance to curve
                # The closer we are to the curve, the more we reduce throttle
                if distance_to_curve < 5.0:  # Very close to curve
                    throttle_factor = 0.3  # Significant reduction
                elif distance_to_curve < 10.0:  # Approaching curve
                    throttle_factor = 0.5  # Moderate reduction
                else:  # Far from curve
                    throttle_factor = 0.7  # Slight reduction
                
                self.get_logger().info(f"Preparing for curve: distance={distance_to_curve:.1f}m, throttle_factor={throttle_factor:.1f}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing waypoints: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def velocity_callback(self, msg):
        """Handle velocity feedback"""
        self.current_velocity = msg.data
        self.get_logger().debug(f"Current velocity: {self.current_velocity:.2f} m/s")
    
    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg
        self.last_imu_time = self.get_clock().now()
    
    def dwa_cmd_callback(self, msg):
        """Handle DWA command velocities directly - Now also handles obstacle detection"""
        if not self.direct_dwa_connection:
            return
        
        self.dwa_cmd = msg
        self.last_dwa_cmd_time = self.get_clock().now()
        
        # Update current velocity from DWA command if velocity feedback is not available
        if hasattr(self, 'current_velocity') and self.current_velocity < 0.1:
            self.current_velocity = abs(msg.linear.x)
        
        # Check if this is a stop command due to an obstacle or black lane
        is_stop_cmd = abs(msg.linear.x) < self.dwa_stop_threshold
        
        # Analyze DWA command to determine reason for stop
        if is_stop_cmd:
            # DWA is requesting a stop
            was_obstacle_detected = self.obstacle_detected
            self.obstacle_detected = True
            self.dwa_stop_requested = True
            
            # Check if this is a new detection
            if not was_obstacle_detected:
                self.obstacle_first_detected_time = self.get_clock().now()
                self.get_logger().warn("Stop command received from DWA - Obstacle or black lane detected")
                
                # Send immediate stop command
                if self.is_connected:
                    self.send_control_commands(0.0, 0.0, 0.8)  # No throttle, no steering, strong brake
                    
                    # Also publish stop command as Twist
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_publisher.publish(cmd_vel)
        else:
            # DWA is not requesting a stop, clear flags
            was_obstacle_detected = self.obstacle_detected
            self.obstacle_detected = False
            self.dwa_stop_requested = False
            self.black_lane_detected = False
            
            # Log if we're resuming after a stop
            if was_obstacle_detected:
                if hasattr(self, 'obstacle_first_detected_time'):
                    duration = (self.get_clock().now() - self.obstacle_first_detected_time).nanoseconds / 1e9
                    self.get_logger().info(f"DWA stop command cleared after {duration:.2f}s - Resuming normal operation")
        
        # Log DWA commands if debug is enabled
        if self.debug_level == 'debug':
            self.get_logger().debug(f"DWA cmd: lin={msg.linear.x:.2f}, ang={msg.angular.z:.2f}")
            
        # Check for steering commands from DWA
        has_steering = abs(msg.angular.z) > 0.1
        if has_steering and not is_stop_cmd:
            # DWA is providing steering guidance, consider using it
            # This could be useful for lane following or obstacle avoidance
            dwa_steer = np.clip(msg.angular.z * 0.5, -0.5, 0.5)  # Scale DWA angular velocity to steering
            
            # We could use this steering in the control loop, but for now we'll just log it
            self.get_logger().debug(f"DWA steering guidance: {dwa_steer:.3f}")

    def path_callback(self, msg):
        """Process path messages from DWA planner"""
        try:
            path_points = len(msg.poses)
            if path_points < self.min_path_points:
                self.get_logger().warn(f"Received path with only {path_points} points (minimum required: {self.min_path_points})")
                return
            
            self.path_count += 1
            self.current_path_length = path_points
            self.last_path_time = self.get_clock().now()
            
            # Log path statistics periodically
            if self.path_count % 10 == 0:
                self.get_logger().info(f"Path stats: received {self.path_count} paths, current length: {path_points} points")
            
        except Exception as e:
            self.get_logger().error(f"Error processing path: {str(e)}")
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.client_socket is not None:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            self.is_connected = False

    def connect_to_server(self):
        """Connect to the control server"""
        reconnect_count = 0
        last_heartbeat_time = time.time()
        
        while self.running:
            # Check if we need to connect
            if self.client_socket is None or not self.is_connected:
                try:
                    # Close any existing socket
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except:
                            pass
                        self.client_socket = None
                    
                    # Create new socket and connect
                    self.get_logger().info(f"Connecting to control server at {self.tcp_host}:{self.tcp_port}...")
                    self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    
                    # Set keep-alive options to detect broken connections
                    self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    
                    # Set timeout for connection attempt
                    self.client_socket.settimeout(5.0)
                    
                    # Connect to the server
                    self.client_socket.connect((self.tcp_host, self.tcp_port))
                    
                    # Reset timeout for normal operation
                    self.client_socket.settimeout(None)
                    
                    self.is_connected = True
                    reconnect_count = 0
                    last_heartbeat_time = time.time()
                    self.get_logger().info(f"Connected to control server at {self.tcp_host}:{self.tcp_port}")
                    
                    # Send initial ping to confirm connection
                    try:
                        # Send a ping message (0,0,0 means no movement)
                        self.client_socket.sendall("0.0000,0.0000,0.0000\n".encode('utf-8'))
                        self.get_logger().debug("Sent initial ping to server")
                    except Exception as e:
                        self.get_logger().error(f"Failed to send initial ping: {str(e)}")
                        self.is_connected = False
                        try:
                            self.client_socket.close()
                        except:
                            pass
                        self.client_socket = None
                        continue
                        
                except socket.timeout:
                    self.get_logger().error("Connection attempt timed out")
                    self.is_connected = False
                    reconnect_count += 1
                    
                except socket.error as e:
                    reconnect_count += 1
                    self.is_connected = False
                    self.get_logger().error(f"Failed to connect to control server: {e}")
                    
                if not self.is_connected:
                    if reconnect_count <= self.max_reconnect_attempts or self.max_reconnect_attempts <= 0:
                        self.get_logger().info(f"Retrying in {self.reconnect_interval} seconds (attempt {reconnect_count}/{self.max_reconnect_attempts})...")
                        time.sleep(self.reconnect_interval)
                    else:
                        self.get_logger().warn(f"Max reconnection attempts ({self.max_reconnect_attempts}) reached. Will continue trying in the background.")
                        # Don't break, but increase the sleep time
                        time.sleep(self.reconnect_interval * 2)
                        # Reset counter to allow future reconnection attempts
                        reconnect_count = 0
                else:
                    # We're connected, send heartbeat every 5 seconds
                    current_time = time.time()
                    if current_time - last_heartbeat_time > 5.0:
                        try:
                            # Send a heartbeat (minimal movement to check connection)
                            self.client_socket.settimeout(1.0)
                            self.client_socket.sendall("0.0000,0.0000,0.0000\n".encode('utf-8'))
                            self.client_socket.settimeout(None)
                            last_heartbeat_time = current_time
                            self.get_logger().debug("Sent heartbeat to server")
                        except Exception as e:
                            self.get_logger().error(f"Heartbeat failed, connection lost: {str(e)}")
                            self.is_connected = False
                            try:
                                self.client_socket.close()
                            except:
                                pass
                            self.client_socket = None
                            continue
                
                    # Sleep for a bit to avoid busy waiting
                    time.sleep(0.5)

    def obstacle_detected_callback(self, msg):
        """Handle obstacle detection messages from DWA"""
        was_obstacle_detected = self.obstacle_detected
        self.obstacle_detected = msg.data
        
        if self.obstacle_detected and not was_obstacle_detected:
            # New obstacle detected
            self.obstacle_first_detected_time = self.get_clock().now()
            self.get_logger().warn("Obstacle detected by DWA")
        elif not self.obstacle_detected and was_obstacle_detected:
            # Obstacle cleared
            if hasattr(self, 'obstacle_first_detected_time'):
                duration = (self.get_clock().now() - self.obstacle_first_detected_time).nanoseconds / 1e9
                self.get_logger().info(f"Obstacle cleared after {duration:.2f}s")

def main(args=None):
    rclpy.init(args=args)
    
    node = MPCController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 