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
        self.max_acc = 3.0  # m/s^2
        self.max_steer = np.deg2rad(30)  # radians

        # Cost function weights - significantly increased weights for position and orientation
        self.Q = np.diag([100, 100, 50, 5, 5, 5])  # state weights: higher weights for position and orientation
        self.R = np.diag([1.0, 5.0])  # control weights: higher weight for steering to make it smoother

        self.prev_control = np.zeros(2)

    def get_control_cleaned(self, current_state, reference_trajectory):
        """Wrapper to ensure controls are properly clipped before CARLA"""
        raw_controls = self.get_control(current_state, reference_trajectory)
        return {
            'acceleration': np.clip(raw_controls['acceleration'], 0, self.max_acc),
            'steering': np.clip(raw_controls['steering'], -self.max_steer, self.max_steer),
            'brake': raw_controls['brake']
        }

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
            # Make sure we have enough reference points for the horizon
            if len(reference_trajectory) < self.horizon:
                last_point = reference_trajectory[-1]
                reference_trajectory.extend([last_point] * (self.horizon - len(reference_trajectory)))
            
            # Find the closest point in the trajectory to the current position
            current_pos = np.array([current_state[0], current_state[1]])
            distances = []
            for ref_state in reference_trajectory:
                ref_pos = np.array([ref_state[0], ref_state[1]])
                dist = np.linalg.norm(ref_pos - current_pos)
                distances.append(dist)
            
            closest_idx = np.argmin(distances)
            
            # Look ahead from the closest point to create the reference trajectory
            # This helps the controller look ahead on the path
            horizon_trajectory = []
            lookahead_distance = 1.0  # meters to look ahead on path
            
            # First point is always the closest
            horizon_trajectory.append(reference_trajectory[closest_idx])
            
            # Add points ahead on the path, ensuring we space them properly
            cumulative_distance = 0.0
            last_pos = np.array([reference_trajectory[closest_idx][0], reference_trajectory[closest_idx][1]])
            
            for i in range(1, self.horizon):
                target_idx = min(closest_idx + i, len(reference_trajectory) - 1)
                next_pos = np.array([reference_trajectory[target_idx][0], reference_trajectory[target_idx][1]])
                
                # Calculate distance from last point
                segment_distance = np.linalg.norm(next_pos - last_pos)
                cumulative_distance += segment_distance
                
                # Use this point if it's at an appropriate distance
                if cumulative_distance >= lookahead_distance * i or target_idx == len(reference_trajectory) - 1:
                    horizon_trajectory.append(reference_trajectory[target_idx])
                    last_pos = next_pos
                else:
                    # If points are too close, use the last point in the reference
                    # to maintain the horizon length
                    horizon_trajectory.append(reference_trajectory[-1])
            
            # Ensure we have enough points for the horizon
            while len(horizon_trajectory) < self.horizon:
                horizon_trajectory.append(reference_trajectory[-1])
            
            opti = ca.Opti()
            X = opti.variable(self.vehicle.n_states, self.horizon + 1)
            U = opti.variable(self.vehicle.n_controls, self.horizon)

            opti.subject_to(X[:, 0] == current_state)

            for k in range(self.horizon):
                x_next = X[:, k] + self.dt * self.vehicle.get_dynamics(X[:, k], U[:, k])
                opti.subject_to(X[:, k + 1] == x_next)

            opti.subject_to(opti.bounded(-self.max_acc, U[0, :], self.max_acc))
            opti.subject_to(opti.bounded(-self.max_steer, U[1, :], self.max_steer))

            cost = 0
            for k in range(self.horizon):
                state_error = X[:, k] - horizon_trajectory[k]
                cost += ca.mtimes([state_error.T, self.Q, state_error])
                control_error = U[:, k] - self.prev_control
                cost += ca.mtimes([control_error.T, self.R, control_error])

            terminal_error = X[:, -1] - horizon_trajectory[-1]
            cost += ca.mtimes([terminal_error.T, 3 * self.Q, terminal_error])  # Increased terminal cost

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
                error = horizon_trajectory[0] - current_state
                optimal_control = np.array([
                    np.clip(error[3] * 0.5, -self.max_acc, self.max_acc),
                    np.clip(error[2] * 0.8, -self.max_steer, self.max_steer)  # Increased steering response
                ])
                print(f"Using fallback control: {optimal_control}")

            acceleration = np.clip(optimal_control[0] / self.max_acc, -1, 1)
            steering = np.clip(optimal_control[1] / self.max_steer, -1, 1)
            brake = max(0, -acceleration) if acceleration < 0 else 0.0
            acceleration = max(0, acceleration)

            self.prev_control = optimal_control

            return {
                'acceleration': acceleration,
                'steering': steering,
                'brake': brake
            }

        except Exception as e:
            print(f"Error in MPC control: {str(e)}")
            return {
                'acceleration': 0.0,
                'steering': 0.0,
                'brake': 1.0
            }

    def get_control_limits(self, vx):
        max_acc = min(2.0, 0.2 * vx)
        max_steer = min(np.deg2rad(30), np.deg2rad(10) * (10 / vx))
        return max_acc, max_steer

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Declare parameters
        self.declare_parameter('tcp_port', 12344)
        self.declare_parameter('tcp_host', '0.0.0.0')
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('reconnect_interval', 2.0)  # seconds
        self.declare_parameter('max_reconnect_attempts', 20)  # Increased from 5 to 20 for more resilience
        self.declare_parameter('waypoints_topic', '/carla/waypoints')
        self.declare_parameter('waypoints_metadata_topic', '/carla/waypoints_metadata')
        self.declare_parameter('use_waypoints', False)  # Default to False to use trajectory_receiver
        self.declare_parameter('debug_level', 'info')  # Logging level: debug, info, warn, error
        self.declare_parameter('test_mode', False)  # Enable test mode for straight line movement
        self.declare_parameter('test_throttle', 0.3)  # Throttle value for test mode [0.0-1.0]
        self.declare_parameter('test_duration', 5.0)  # Duration of test in seconds
        self.declare_parameter('respect_obstacle_detection', True)
        self.declare_parameter('obstacle_detected_topic', '/dwa/obstacle_detected')
        self.declare_parameter('emergency_stop_distance', 1.5)
        self.declare_parameter('obstacle_stop_threshold', 0.1)
        self.declare_parameter('strict_path_following', True)
        self.declare_parameter('hard_brake_on_obstacle', True)
        self.declare_parameter('obstacle_reaction_factor', 2.0)
        self.declare_parameter('cmd_vel_topic', '/mpc/cmd_vel')
        self.declare_parameter('dwa_cmd_vel_topic', '/dwa/cmd_vel')
        self.declare_parameter('direct_dwa_connection', True)
        self.declare_parameter('bypass_trajectory_receiver', False)
        self.declare_parameter('path_topic', '/dwa/path')
        self.declare_parameter('dwa_path_priority', True)  # Prioritize DWA path over other sources
        self.declare_parameter('min_path_points', 3)  # Minimum number of points to consider a valid path
        self.declare_parameter('path_update_timeout', 2.0)  # Path is considered stale after this many seconds
        
        # Get parameters
        self.tcp_port = self.get_parameter('tcp_port').value
        self.tcp_host = self.get_parameter('tcp_host').value
        self.update_rate = self.get_parameter('update_rate').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value
        self.waypoints_topic = self.get_parameter('waypoints_topic').value
        self.waypoints_metadata_topic = self.get_parameter('waypoints_metadata_topic').value
        self.use_waypoints = self.get_parameter('use_waypoints').value
        self.debug_level = self.get_parameter('debug_level').value
        self.test_mode = self.get_parameter('test_mode').value
        self.test_throttle = self.get_parameter('test_throttle').value
        self.test_duration = self.get_parameter('test_duration').value
        self.respect_obstacle_detection = self.get_parameter('respect_obstacle_detection').value
        self.obstacle_detected_topic = self.get_parameter('obstacle_detected_topic').value
        self.emergency_stop_distance = self.get_parameter('emergency_stop_distance').value
        self.obstacle_stop_threshold = self.get_parameter('obstacle_stop_threshold').value
        self.strict_path_following = self.get_parameter('strict_path_following').value
        self.hard_brake_on_obstacle = self.get_parameter('hard_brake_on_obstacle').value
        self.obstacle_reaction_factor = self.get_parameter('obstacle_reaction_factor').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.dwa_cmd_vel_topic = self.get_parameter('dwa_cmd_vel_topic').value
        self.direct_dwa_connection = self.get_parameter('direct_dwa_connection').value
        self.bypass_trajectory_receiver = self.get_parameter('bypass_trajectory_receiver').value
        self.path_topic = self.get_parameter('path_topic').value
        self.dwa_path_priority = self.get_parameter('dwa_path_priority').value
        self.min_path_points = self.get_parameter('min_path_points').value
        self.path_update_timeout = self.get_parameter('path_update_timeout').value
        
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
        
        # Initialize MPC controller with larger horizon
        self.controller = MPCControllerImpl(horizon=15, dt=1.0/self.update_rate)
        
        # Client connection
        self.client_socket = None
        self.is_connected = False
        
        # Create subscriber for trajectory (for compatibility)
        self.trajectory_subscription = self.create_subscription(
            PoseArray,
            'reference_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Create subscribers for waypoints from waypoint_listener
        self.waypoints_subscription = self.create_subscription(
            PoseArray,
            self.waypoints_topic,
            self.waypoints_callback,
            10
        )
        
        self.waypoints_metadata_subscription = self.create_subscription(
            Int32MultiArray,
            self.waypoints_topic + "_metadata",
            self.waypoints_metadata_callback,
            10
        )
        
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
        
        # Create subscriber for DWA path with enhanced QoS
        self.path_subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            path_qos
        )
        
        # State variables
        self.current_state = np.zeros(6)  # [x, y, psi, vx, vy, omega]
        self.reference_trajectory = []
        self.latest_waypoints = []
        self.latest_metadata = []
        self.has_new_waypoints = False
        self.current_waypoint_index = 0  # Track the current waypoint index
        
        # Thread control
        self.running = True
        self.connection_thread = None
        
        # Initialize obstacle detection state
        self.obstacle_detected = False
        self.obstacle_detection_time = None
        
        # Subscribe to obstacle detection topic
        if self.respect_obstacle_detection:
            self.obstacle_detection_sub = self.create_subscription(
                Bool,
                self.obstacle_detected_topic,
                self.obstacle_detected_callback,
                10
            )
        
        # Start client connection thread - with proper error handling
        try:
            self.connection_thread = threading.Thread(target=self.connect_to_server)
            self.connection_thread.daemon = True
            self.connection_thread.start()
            self.get_logger().info("Started TCP connection thread")
        except Exception as e:
            self.get_logger().error(f"Failed to start connection thread: {str(e)}")
        
        # Create timer for MPC control loop
        self.timer = self.create_timer(1.0/self.update_rate, self.control_loop)
        
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
        
        self.get_logger().info(f'MPC controller node initialized - Connected to {self.tcp_host}:{self.tcp_port} - Using DWA path priority: {self.dwa_path_priority}')
    
    def connect_to_server(self):
        """Connect to the control server in Gui_Control_py35.py"""
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
    
    def waypoints_callback(self, msg):
        """Callback for receiving waypoints from waypoint_listener"""
        try:
            if not self.use_waypoints:
                return
                
            # Convert PoseArray to list of waypoints
            waypoints = []
            for pose in msg.poses:
                waypoints.append({
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z,
                    'orientation': pose.orientation
                })
            
            if waypoints:
                self.latest_waypoints = waypoints
                self.has_new_waypoints = True
                self.get_logger().debug(f"Received {len(waypoints)} waypoints from waypoint_listener")
                
                # Update reference trajectory if we have both waypoints and metadata
                self.update_reference_trajectory()
        except Exception as e:
            self.get_logger().error(f"Error processing waypoints: {str(e)}")
    
    def waypoints_metadata_callback(self, msg):
        """Callback for receiving waypoints metadata from waypoint_listener"""
        try:
            if not self.use_waypoints:
                return
                
            # Process metadata (road_id, lane_id, lane_type for each waypoint)
            metadata = []
            data = msg.data
            
            # Each waypoint has 3 metadata values: road_id, lane_id, lane_type
            if len(data) % 3 == 0:
                num_waypoints = len(data) // 3
                for i in range(num_waypoints):
                    metadata.append({
                        'road_id': data[i*3],
                        'lane_id': data[i*3+1],
                        'lane_type': data[i*3+2]
                    })
                
                self.latest_metadata = metadata
                self.get_logger().debug(f"Received metadata for {len(metadata)} waypoints")
                
                # Update reference trajectory if we have both waypoints and metadata
                self.update_reference_trajectory()
            else:
                self.get_logger().warn(f"Invalid metadata format: expected multiple of 3 values, got {len(data)}")
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint metadata: {str(e)}")
    
    def update_reference_trajectory(self):
        """Update reference trajectory from waypoints and metadata"""
        try:
            # Only update if we have both waypoints and metadata
            if not self.latest_waypoints or not self.latest_metadata:
                return
                
            # Make sure we have the same number of waypoints and metadata
            if len(self.latest_waypoints) != len(self.latest_metadata):
                self.get_logger().warn(f"Mismatch between waypoints ({len(self.latest_waypoints)}) and metadata ({len(self.latest_metadata)})")
                return
                
            # Create reference trajectory for MPC
            trajectory = []
            
            # Calculate orientations based on waypoint sequence
            for i in range(len(self.latest_waypoints)):
                wp = self.latest_waypoints[i]
                md = self.latest_metadata[i]
                
                # Calculate orientation (heading) based on next waypoint if available
                psi = 0.0
                vx = 5.0  # Reduced default velocity from 10.0 to 5.0
                
                # Calculate orientation based on next point
                if i < len(self.latest_waypoints) - 1:
                    next_wp = self.latest_waypoints[i + 1]
                    dx = next_wp['x'] - wp['x']
                    dy = next_wp['y'] - wp['y']
                    psi = np.arctan2(dy, dx)  # Calculate heading angle
                elif i > 0:  # For the last point, use the same orientation as the previous point
                    prev_wp = self.latest_waypoints[i - 1]
                    dx = wp['x'] - prev_wp['x']
                    dy = wp['y'] - prev_wp['y']
                    psi = np.arctan2(dy, dx)
                
                # Create state vector for MPC
                # [x, y, psi, vx, vy, omega]
                state = np.array([
                    wp['x'], 
                    wp['y'], 
                    psi,  # Calculated orientation
                    vx,   # Reduced velocity
                    0.0,  # No lateral velocity
                    0.0   # No angular velocity
                ])
                
                trajectory.append(state)
            
            # Update reference trajectory
            if trajectory:
                self.reference_trajectory = trajectory
                self.has_new_waypoints = False
                self.current_waypoint_index = 0  # Reset the current waypoint index
                
                # Initialize current state with first point if not set
                if np.all(self.current_state == 0):
                    self.current_state = trajectory[0].copy()
                    
                self.get_logger().info(f"Updated reference trajectory with {len(trajectory)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Error updating reference trajectory: {str(e)}")
    
    def trajectory_callback(self, msg):
        """Callback for receiving trajectory updates from trajectory_receiver"""
        try:
            # Skip if we're prioritizing DWA path and already have a valid path
            if self.dwa_path_priority and self.has_valid_path:
                self.get_logger().debug("Ignoring trajectory update from trajectory_receiver (DWA path has priority)")
                return
                
            # Convert PoseArray to list of numpy arrays for MPC
            trajectory = []
            
            # Extract orientation information if available
            for i, pose in enumerate(msg.poses):
                # Extract pose data
                x = pose.position.x
                y = pose.position.y
                
                # Calculate orientation based on next point if available
                psi = 0.0
                vx = 5.0  # Default velocity
                
                # Try to extract orientation from quaternion
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                
                # If quaternion is not set (all zeros), calculate from position sequence
                if qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0:
                    # Calculate orientation based on next point
                    if i < len(msg.poses) - 1:
                        next_pose = msg.poses[i + 1]
                        dx = next_pose.position.x - x
                        dy = next_pose.position.y - y
                        psi = np.arctan2(dy, dx)
                    elif i > 0:  # For the last point, use the same orientation as the previous point
                        prev_pose = msg.poses[i - 1]
                        dx = x - prev_pose.position.x
                        dy = y - prev_pose.position.y
                        psi = np.arctan2(dy, dx)
                else:
                    # Extract yaw from quaternion
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    psi = np.arctan2(siny_cosp, cosy_cosp)
                
                # Create state vector for MPC
                # [x, y, psi, vx, vy, omega]
                state = np.array([
                    x, y, psi,  # Position and orientation
                    vx, 0.0, 0.0  # Velocity (vx, vy, omega)
                ])
                trajectory.append(state)
            
            if trajectory:
                self.reference_trajectory = trajectory
                self.get_logger().info(f"Updated reference trajectory with {len(trajectory)} points from trajectory_receiver")
                
                # Initialize current state with first point if not set
                if np.all(self.current_state == 0):
                    self.current_state = trajectory[0].copy()
        except Exception as e:
            self.get_logger().error(f"Error processing trajectory: {str(e)}")
    
    def send_control_commands(self, throttle, brake, steer):
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
            # Format the control message - ensure format matches what server expects
            # Format: throttle,brake,steer\n
            # Ensure values are clipped to the expected range [0.0-1.0]
            throttle_clipped = np.clip(throttle, 0.0, 1.0)
            brake_clipped = np.clip(brake, 0.0, 1.0)
            steer_clipped = np.clip(steer, -1.0, 1.0)
            
            control_msg = f"{throttle_clipped:.4f},{brake_clipped:.4f},{steer_clipped:.4f}\n"
            
            # Send data with timeout
            self.client_socket.settimeout(1.0)  # 1 second timeout
            self.client_socket.sendall(control_msg.encode('utf-8'))
            
            # Log commands at info level instead of debug to see what's being sent
            self.get_logger().info(f"Sent control: {control_msg.strip()}")
            
            # Reset timeout
            self.client_socket.settimeout(None)
            
        except socket.timeout:
            self.get_logger().error("Socket send timed out")
            self.is_connected = False
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            
        except BrokenPipeError:
            self.get_logger().error("Broken pipe - server disconnected")
            self.is_connected = False
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            
        except ConnectionResetError:
            self.get_logger().error("Connection reset by peer")
            self.is_connected = False
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
            
        except Exception as e:
            self.get_logger().error(f"Failed to send control commands: {str(e)}")
            # Mark as disconnected and let the connection thread reconnect
            self.is_connected = False
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
    
    def move_straight_line(self, throttle=None, duration=None, steer=0.0):
        """
        Function to move in a straight line for testing purposes
        
        Args:
            throttle: Throttle value between 0.0 and 1.0 (default: self.test_throttle)
            duration: Duration of the test in seconds (default: self.test_duration)
            steer: Steering value between -1.0 and 1.0 (default: 0.0 for straight)
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
        self.send_control_commands(throttle, brake, steer)
        
        # Return the control values used
        return throttle, steer, brake
        
    def control_loop(self):
        """Main MPC control loop"""
        try:
            # Check if path is stale
            if self.last_path_time is not None:
                path_age = (self.get_clock().now() - self.last_path_time).nanoseconds / 1e9
                if path_age > self.path_update_timeout:
                    if self.has_valid_path:
                        self.get_logger().warn(f"DWA path is stale ({path_age:.1f}s old). Path may be interrupted.")
                        # Don't clear the path immediately, just warn about it
            
            # Check if test mode is enabled
            if self.test_mode:
                # Use test mode to send simple commands
                throttle = self.test_throttle
                brake = 0.0
                steer = 0.0
                
                # Send control commands over TCP
                self.send_control_commands(throttle, brake, steer)
                
                # Also publish Twist message for ROS integration
                cmd_vel = Twist()
                cmd_vel.linear.x = throttle * 10.0  # Scale to reasonable velocity
                cmd_vel.angular.z = steer * 0.8  # Scale steering to reasonable angular velocity
                
                # Publish the Twist message
                self.cmd_vel_publisher.publish(cmd_vel)
                self.get_logger().info(f"Test mode: throttle={throttle:.3f}, brake={brake:.3f}, steer={steer:.3f}")
                return
            
            # Check for obstacles before calculating control
            if self.respect_obstacle_detection and self.obstacle_detected:
                # Emergency stop due to obstacle
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                
                # Also send emergency stop via TCP
                if self.is_connected:
                    self.send_control_commands(0.0, 1.0, 0.0)  # No throttle, full brake, no steering
                
                self.get_logger().debug("Emergency stop due to obstacle")
                self.cmd_vel_publisher.publish(cmd_vel)
                return
            
            # Only proceed if we have a trajectory
            if not self.reference_trajectory:
                # If we have no trajectory but we have direct DWA connection
                if self.direct_dwa_connection and self.dwa_cmd:
                    # Use DWA command as fallback
                    linear_x = self.dwa_cmd.linear.x
                    angular_z = self.dwa_cmd.angular.z
                    
                    # Scale to appropriate values for TCP
                    throttle = max(0.0, linear_x / 10.0)  # Convert velocity to throttle
                    brake = max(0.0, -linear_x / 10.0) if linear_x < 0 else 0.0  # Brake if negative
                    steer = angular_z / 0.8  # Convert angular velocity to steering
                    
                    # Ensure valid ranges
                    throttle = np.clip(throttle, 0.0, 1.0)
                    brake = np.clip(brake, 0.0, 1.0)
                    steer = np.clip(steer, -1.0, 1.0)
                    
                    # Send control commands over TCP
                    self.send_control_commands(throttle, brake, steer)
                    
                    # Also publish Twist message
                    cmd_vel = Twist()
                    cmd_vel.linear.x = throttle * 10.0
                    cmd_vel.angular.z = steer * 0.8
                    self.cmd_vel_publisher.publish(cmd_vel)
                    
                    self.get_logger().info(f"Using DWA velocity commands as fallback: throttle={throttle:.3f}, brake={brake:.3f}, steer={steer:.3f}")
                    return
                else:
                    # If we have no trajectory and no DWA commands, send forward command
                    # Initialize throttle, brake and steer variables before using them
                    throttle = 0.4  # Higher throttle for faster movement
                    brake = 0.0
                    steer = 0.0
                    
                    if self.is_connected:
                        self.send_control_commands(throttle, brake, steer)
                        
                        # Also publish Twist message
                        cmd_vel = Twist()
                        cmd_vel.linear.x = throttle * 10.0
                        cmd_vel.angular.z = 0.0
                        self.cmd_vel_publisher.publish(cmd_vel)
                        
                    self.get_logger().info(f"No trajectory available, sending forward command: throttle={throttle:.3f}")
                    return
            
            # Regular MPC control based on trajectory
            # Update current state based on the nearest waypoint in the trajectory
            try:
                # Find the closest waypoint to the current position
                current_pos = np.array([self.current_state[0], self.current_state[1]])
                distances = []
                for ref_state in self.reference_trajectory:
                    ref_pos = np.array([ref_state[0], ref_state[1]])
                    dist = np.linalg.norm(ref_pos - current_pos)
                    distances.append(dist)
                
                closest_idx = np.argmin(distances)
                min_dist = distances[closest_idx]
                
                # If we're very close to a waypoint and there's a next one, use the next one
                # to keep the controller looking ahead
                if min_dist < 1.0 and closest_idx + 1 < len(self.reference_trajectory):
                    target_idx = closest_idx + 1
                    self.get_logger().debug(f"Using waypoint {target_idx}/{len(self.reference_trajectory)} (looking ahead)")
                else:
                    target_idx = closest_idx
                    self.get_logger().debug(f"Using waypoint {target_idx}/{len(self.reference_trajectory)}")
                
                # Get MPC control commands
                control_commands = self.controller.get_control(self.current_state, self.reference_trajectory)
                
                # Extract the control values
                throttle = control_commands['acceleration']
                brake = control_commands['brake']
                steer = control_commands['steering']
                
                # Apply additional scaling to ensure precise control with higher speed
                throttle = throttle * 0.8  # Increased from 0.6 for higher speed
                
                # Add minimum throttle to ensure car keeps moving
                if throttle < 0.1 and brake < 0.1:
                    throttle = 0.2  # Minimum throttle to maintain movement
                
                # Ensure valid ranges
                throttle = np.clip(throttle, 0.0, 1.0)
                brake = np.clip(brake, 0.0, 1.0)
                steer = np.clip(steer, -1.0, 1.0)
                
                # Log the control commands and path following info
                progress_msg = f"Following waypoint {target_idx+1}/{len(self.reference_trajectory)}"
                self.get_logger().info(f"{progress_msg} - Control: throttle={throttle:.3f}, brake={brake:.3f}, steer={steer:.3f}")
                
                # Apply enhanced obstacle handling if an obstacle is detected
                if self.obstacle_detected and self.respect_obstacle_detection:
                    if self.hard_brake_on_obstacle:
                        # Emergency braking - override commands
                        throttle = 0.0
                        steer = 0.0
                        brake = 1.0  # Full brake
                        self.get_logger().warn("MPC: EMERGENCY BRAKING activated")
                    else:
                        # Gradual braking based on reaction factor
                        brake_factor = min(1.0, self.obstacle_reaction_factor * self.obstacle_stop_threshold)
                        throttle = 0.0
                        steer = steer  # Keep steering
                        brake = brake_factor
                        self.get_logger().info(f"MPC: Progressive braking (factor: {brake_factor:.2f})")
                
                # Send control commands over TCP
                self.send_control_commands(throttle, brake, steer)
                
                # Also publish as Twist message for ROS integration
                cmd_vel = Twist()
                cmd_vel.linear.x = throttle * 10.0  # Scale to reasonable velocity
                cmd_vel.angular.z = steer * 0.8  # Scale steering to reasonable angular velocity
                
                # Apply brake if needed
                if brake > 0.1:
                    cmd_vel.linear.x = 0.0  # Stop when braking
                
                # Publish the Twist message
                self.cmd_vel_publisher.publish(cmd_vel)
                
            except Exception as e:
                self.get_logger().error(f"Error computing control commands: {str(e)}")
                # Send safe values in case of error
                self.send_control_commands(0.0, 0.5, 0.0)  # No throttle, moderate brake, no steering
                
                # Also publish stop command as Twist
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_vel)
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {str(e)}")
            # Send safe values in case of a general error
            if self.is_connected:
                self.send_control_commands(0.0, 0.3, 0.0)  # No throttle, light brake, no steering
            
            # Also publish stop command as Twist
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel)
    
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

    def obstacle_detected_callback(self, msg):
        """Process obstacle detection messages"""
        was_obstacle_detected = self.obstacle_detected
        self.obstacle_detected = msg.data
        
        # Log state change with timestamp
        if self.obstacle_detected != was_obstacle_detected:
            if self.obstacle_detected:
                self.get_logger().warn("Obstacle detected - Activating emergency braking")
                self.obstacle_first_detected_time = self.get_clock().now()
            else:
                self.get_logger().info("Obstacle cleared - Resuming normal operation")
                
                # If we were stopped for a long time, apply gradual acceleration
                if was_obstacle_detected and hasattr(self, 'obstacle_first_detected_time'):
                    stop_duration = (self.get_clock().now() - self.obstacle_first_detected_time).nanoseconds / 1e9
                    if stop_duration > 2.0:  # If stopped for more than 2 seconds
                        self.get_logger().info(f"Gradual acceleration after {stop_duration:.1f}s stop")
                        # The gradual acceleration will be handled in the control loop
                
    def calculate_control_commands(self, current_state, reference_trajectory):
        """Calculate control commands using MPC"""
        # ... existing code ...

        # Apply enhanced obstacle handling if an obstacle is detected
        if self.obstacle_detected and self.respect_obstacle_detection:
            if self.hard_brake_on_obstacle:
                # Emergency braking - override commands
                throttle = 0.0
                steer = 0.0
                brake = 1.0  # Full brake
                self.get_logger().warn("MPC: EMERGENCY BRAKING activated")
            else:
                # Gradual braking based on reaction factor
                brake_factor = min(1.0, self.obstacle_reaction_factor * self.obstacle_stop_threshold)
                throttle = 0.0
                steer = steer  # Keep steering
                brake = brake_factor
                self.get_logger().info(f"MPC: Progressive braking (factor: {brake_factor:.2f})")
                
            return throttle, steer, brake
            
        # ... existing code ...

    def dwa_cmd_callback(self, msg):
        """Handle DWA command velocities directly"""
        if not self.direct_dwa_connection:
            return
        
        self.dwa_cmd = msg
        self.last_dwa_cmd_time = self.get_clock().now()
        
        # Check if this is a stop command due to an obstacle
        is_stop_cmd = abs(msg.linear.x) < 0.01
        has_steering = abs(msg.angular.z) > 0.1
        
        if is_stop_cmd and has_steering:
            # This looks like a stop command due to obstacle
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self.obstacle_first_detected_time = self.get_clock().now()
                self.get_logger().warn("⚠️ Obstacle detected from DWA commands! Emergency braking activated.")
        elif self.obstacle_detected:
            # No longer a stop command, clear obstacle detection
            self.obstacle_detected = False
            if hasattr(self, 'obstacle_first_detected_time'):
                duration = (self.get_clock().now() - self.obstacle_first_detected_time).nanoseconds / 1e9
                self.get_logger().info(f"Obstacle cleared after {duration:.2f}s, resuming normal operation")
        
        # Log DWA commands if debug is enabled
        if self.debug_level == 'debug':
            self.get_logger().debug(f"DWA cmd: lin={msg.linear.x:.2f}, ang={msg.angular.z:.2f}")

    def path_callback(self, msg):
        """
        Process path messages from DWA planner.
        This is the primary trajectory source when dwa_path_priority is True.
        """
        try:
            path_points = len(msg.poses)
            if path_points < self.min_path_points:
                self.get_logger().warn(f"Received path with only {path_points} points (minimum required: {self.min_path_points})")
                return
            
            self.path_count += 1
            self.current_path_length = path_points
            self.last_path_time = self.get_clock().now()
            
            # Log path statistics periodically (every 10th path)
            if self.path_count % 10 == 0:
                self.get_logger().info(f"Path stats: received {self.path_count} paths, current length: {path_points} points")
            
            # Convert Path message to reference trajectory for MPC
            trajectory = []
            
            # Extract poses and calculate dynamic properties
            for i, pose_stamped in enumerate(msg.poses):
                pose = pose_stamped.pose
                
                # Extract position
                x = pose.position.x
                y = pose.position.y
                
                # Calculate orientation from quaternion or from sequence
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                qw = pose.orientation.w
                
                # Default velocity - higher for faster movement
                vx = 8.0  # Target velocity for movement
                
                # If quaternion is not properly set, calculate from position sequence
                if abs(qx) < 0.001 and abs(qy) < 0.001 and abs(qz) < 0.001 and abs(qw) < 0.001:
                    # Calculate orientation based on next point if available
                    if i < path_points - 1:
                        next_pose = msg.poses[i + 1].pose
                        dx = next_pose.position.x - x
                        dy = next_pose.position.y - y
                        psi = np.arctan2(dy, dx)
                    elif i > 0:  # For the last point, use previous point direction
                        prev_pose = msg.poses[i - 1].pose
                        dx = x - prev_pose.position.x
                        dy = y - prev_pose.position.y
                        psi = np.arctan2(dy, dx)
                    else:
                        psi = 0.0  # Default if only one point
                else:
                    # Extract yaw from quaternion
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    psi = np.arctan2(siny_cosp, cosy_cosp)
                
                # Adjust velocity based on path curvature
                if i > 0 and i < path_points - 1:
                    # Calculate curvature using three points
                    prev_pose = msg.poses[max(0, i-1)].pose
                    curr_pose = pose
                    next_pose = msg.poses[min(path_points-1, i+1)].pose
                    
                    # Vectors between points
                    v1 = np.array([curr_pose.position.x - prev_pose.position.x, 
                                   curr_pose.position.y - prev_pose.position.y])
                    v2 = np.array([next_pose.position.x - curr_pose.position.x, 
                                   next_pose.position.y - curr_pose.position.y])
                    
                    # Normalize vectors
                    v1_norm = np.linalg.norm(v1)
                    v2_norm = np.linalg.norm(v2)
                    
                    if v1_norm > 0.01 and v2_norm > 0.01:
                        v1 = v1 / v1_norm
                        v2 = v2 / v2_norm
                        
                        # Calculate angle between vectors
                        dot_product = np.clip(np.dot(v1, v2), -1.0, 1.0)
                        angle = np.arccos(dot_product)
                        
                        # Adjust velocity based on curvature (higher angle = lower velocity)
                        curvature_factor = 1.0 - (angle / np.pi) * 0.5
                        vx = vx * (0.7 + 0.3 * curvature_factor)
                
                # Create state vector for MPC [x, y, psi, vx, vy, omega]
                state = np.array([x, y, psi, vx, 0.0, 0.0])
                trajectory.append(state)
            
            # Update reference trajectory if enough points
            if len(trajectory) >= self.min_path_points:
                # Set flag indicating we have a valid trajectory
                self.has_valid_path = True
                
                # Update the reference trajectory
                self.reference_trajectory = trajectory
                self.get_logger().debug(f"Updated reference trajectory with {len(trajectory)} points from DWA path")
                
                # Initialize current state with first point if not set
                if np.all(self.current_state == 0):
                    self.current_state = trajectory[0].copy()
                else:
                    # Update current state yaw and velocity but keep position
                    # This prevents jumps when updating trajectories
                    self.current_state[2] = trajectory[0][2]  # Update yaw
                    self.current_state[3] = trajectory[0][3]  # Update velocity
            else:
                self.get_logger().warn("Path conversion resulted in too few trajectory points")
        
        except Exception as e:
            self.get_logger().error(f"Error processing path: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

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