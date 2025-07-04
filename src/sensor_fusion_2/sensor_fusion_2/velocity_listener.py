#!/usr/bin/env python3
"""
Velocity Listener Node

This node sets up a TCP server to receive velocity data from external sources.
It processes the velocity data and publishes it as a ROS2 topic for use by other nodes.

Author: Claude
"""

import socket
import struct
import time
import threading
import math
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

class VelocityListener(Node):
    def __init__(self):
        super().__init__('velocity_listener')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '0.0.0.0')  # Listen on all interfaces
        self.declare_parameter('tcp_port', 12346)
        self.declare_parameter('velocity_topic', '/carla/ego_vehicle/velocity')
        self.declare_parameter('velocity_vector_topic', '/carla/ego_vehicle/velocity_vector')
        self.declare_parameter('socket_buffer_size', 4096)
        self.declare_parameter('socket_timeout', 0.5)
        self.declare_parameter('enable_tcp_nodelay', True)
        self.declare_parameter('enable_socket_keepalive', True)
        self.declare_parameter('verbose_logging', True)
        self.declare_parameter('reconnect_interval', 2.0)  # Seconds to wait before reconnect attempts
        self.declare_parameter('connection_timeout', 10.0)  # Seconds to wait for connection
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.velocity_topic = self.get_parameter('velocity_topic').get_parameter_value().string_value
        self.velocity_vector_topic = self.get_parameter('velocity_vector_topic').get_parameter_value().string_value
        self.socket_buffer_size = self.get_parameter('socket_buffer_size').get_parameter_value().integer_value
        self.socket_timeout = self.get_parameter('socket_timeout').get_parameter_value().double_value
        self.enable_tcp_nodelay = self.get_parameter('enable_tcp_nodelay').get_parameter_value().bool_value
        self.enable_socket_keepalive = self.get_parameter('enable_socket_keepalive').get_parameter_value().bool_value
        self.verbose_logging = self.get_parameter('verbose_logging').get_parameter_value().bool_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Set up QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.velocity_publisher = self.create_publisher(
            Float32,
            self.velocity_topic,
            qos_profile
        )
        
        # Also publish velocity as Vector3 for components
        self.velocity_vector_publisher = self.create_publisher(
            Vector3,
            self.velocity_vector_topic,
            qos_profile
        )
        
        # Socket variables
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.running = True
        self.velocity_queue = Queue(maxsize=10)
        self.connection_lock = threading.Lock()  # Add lock for connection management
        
        # Velocity data
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
        self.current_velocity_z = 0.0
        self.current_velocity_magnitude = 0.0
        self.last_velocity_time = time.time()
        
        # Setup TCP server
        self.setup_tcp_server()
        
        # Flag to track connection status
        self.is_connected = False
        self.connection_event = threading.Event()
        
        # Start processing threads
        self.server_thread = threading.Thread(target=self.accept_connections)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # Create a timer for publishing velocity
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_velocity)
        
        # Create a timer for status updates
        self.status_timer = self.create_timer(5.0, self.log_status)
        
        self.get_logger().info("Velocity listener initialized and waiting for connections")
        
        # Wait for initial connection
        self.wait_for_connection()
        
    def setup_tcp_server(self):
        """Set up TCP server to listen for connections"""
        with self.connection_lock:
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
                self.server_socket = None
                
            try:
                self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                
                # Set socket options
                if self.enable_tcp_nodelay:
                    self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                if self.enable_socket_keepalive:
                    self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                
                self.server_socket.settimeout(self.socket_timeout)
                
                self.get_logger().info(f"Setting up TCP server on {self.tcp_ip}:{self.tcp_port}")
                self.server_socket.bind((self.tcp_ip, self.tcp_port))
                self.server_socket.listen(1)
                self.get_logger().info(f"TCP server listening on {self.tcp_ip}:{self.tcp_port}")
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to set up TCP server: {e}")
                return False
    
    def wait_for_connection(self):
        """Wait for a client to connect before proceeding"""
        self.get_logger().info(f"Waiting for a client to connect on port {self.tcp_port}...")
        
        # Wait for connection with timeout
        connection_timeout = self.connection_timeout
        wait_start_time = time.time()
        
        while not self.is_connected and self.running:
            # Wait for the connection event with a short timeout to allow checking conditions
            self.connection_event.wait(timeout=1.0)
            
            # Check if connected
            if self.is_connected:
                self.get_logger().info("Client connected successfully!")
                return True
            
            # Check if we've exceeded the timeout
            elapsed_time = time.time() - wait_start_time
            if elapsed_time > connection_timeout:
                self.get_logger().warn(f"Connection timeout after {connection_timeout} seconds. Continuing without connection.")
                return False
            
            # Log waiting message every 5 seconds
            if int(elapsed_time) % 5 == 0 and elapsed_time > 1:
                self.get_logger().info(f"Still waiting for connection... ({int(elapsed_time)}/{int(connection_timeout)} seconds)")
        
        return self.is_connected
    
    def accept_connections(self):
        """Accept incoming connections and start receive thread for each client"""
        connection_attempts = 0
        last_attempt_time = 0
        
        while self.running:
            try:
                # Check if we need to restart the server socket
                if self.server_socket is None:
                    current_time = time.time()
                    if current_time - last_attempt_time >= self.reconnect_interval:
                        last_attempt_time = current_time
                        connection_attempts += 1
                        self.get_logger().info(f"Attempting to restart server (attempt {connection_attempts})")
                        if not self.setup_tcp_server():
                            self.get_logger().warn(f"Failed to restart server, will retry in {self.reconnect_interval} seconds")
                            time.sleep(self.reconnect_interval)
                            continue
                
                # Accept new connection
                client_socket, client_address = self.server_socket.accept()
                
                # Close previous client connection if exists
                with self.connection_lock:
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except:
                            pass
                    
                    self.client_socket = client_socket
                    self.client_address = client_address
                    self.is_connected = True
                    self.connection_event.set()  # Signal that connection is established
                
                self.get_logger().info(f"Accepted connection from {client_address}")
                connection_attempts = 0  # Reset connection attempts on successful connection
                
                # Start receive thread for this client
                receive_thread = threading.Thread(target=self.receive_velocity_data, args=(client_socket,))
                receive_thread.daemon = True
                receive_thread.start()
                
            except socket.timeout:
                # This is normal, just continue
                continue
            except Exception as e:
                self.get_logger().error(f"Error accepting connection: {e}")
                
                # If server socket is broken, reset it
                with self.connection_lock:
                    if self.server_socket:
                        try:
                            self.server_socket.close()
                        except:
                            pass
                        self.server_socket = None
                    self.is_connected = False
                    self.connection_event.clear()  # Reset connection event
                
                time.sleep(self.reconnect_interval)  # Wait before trying again
    
    def receive_velocity_data(self, client_socket):
        """Receive velocity data from client via TCP"""
        client_socket.settimeout(self.socket_timeout)
        FLOAT_SIZE = 4  # Size of float in bytes
        PACKET_SIZE = FLOAT_SIZE * 3  # 3 floats (x, y, z)
        
        self.get_logger().info(f"Connected to client: {self.client_address}")
        
        while self.running and self.is_connected:
            try:
                # Read exactly 12 bytes (3 floats, 4 bytes each)
                data = client_socket.recv(PACKET_SIZE)
                
                if not data:
                    self.get_logger().warn("Client disconnected")
                    break
                
                if len(data) != PACKET_SIZE:
                    self.get_logger().warn(f"Received incomplete packet: {len(data)} bytes instead of {PACKET_SIZE}")
                    continue
                
                # Unpack the velocity data (3 floats)
                try:
                    vx, vy, vz = struct.unpack('!fff', data)
                    
                    # Update velocity data
                    self.current_velocity_x = vx
                    self.current_velocity_y = vy
                    self.current_velocity_z = vz
                    self.current_velocity_magnitude = math.sqrt(vx*vx + vy*vy + vz*vz)
                    self.last_velocity_time = time.time()
                    
                    if self.verbose_logging:
                        self.get_logger().debug(f"Received velocity: ({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s, magnitude: {self.current_velocity_magnitude:.2f} m/s")
                    
                except struct.error as e:
                    self.get_logger().error(f"Error unpacking velocity data: {e}")
                
            except socket.timeout:
                # This is normal, just continue
                continue
            except ConnectionResetError:
                self.get_logger().warn("Connection reset by peer")
                break
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")
                break
        
        # Clean up connection
        with self.connection_lock:
            if client_socket == self.client_socket:  # Only reset if this is the current client
                self.is_connected = False
                self.connection_event.clear()
                self.client_socket = None
                self.client_address = None
        
        try:
            client_socket.close()
        except:
            pass
        
        self.get_logger().info("Client disconnected, waiting for new connection")
    
    def publish_velocity(self):
        """Publish velocity data to ROS topics"""
        # Check if we have recent velocity data (within last second)
        current_time = time.time()
        if current_time - self.last_velocity_time > 1.0:
            # No recent data, publish zero velocity if not connected
            if not self.is_connected:
                self.current_velocity_x = 0.0
                self.current_velocity_y = 0.0
                self.current_velocity_z = 0.0
                self.current_velocity_magnitude = 0.0
        
        # Publish velocity magnitude
        velocity_msg = Float32()
        velocity_msg.data = float(self.current_velocity_magnitude)
        self.velocity_publisher.publish(velocity_msg)
        
        # Publish velocity vector
        vector_msg = Vector3()
        vector_msg.x = float(self.current_velocity_x)
        vector_msg.y = float(self.current_velocity_y)
        vector_msg.z = float(self.current_velocity_z)
        self.velocity_vector_publisher.publish(vector_msg)
    
    def log_status(self):
        """Log current status"""
        if self.is_connected:
            self.get_logger().info(f"Connected to {self.client_address}, current velocity: {self.current_velocity_magnitude:.2f} m/s")
        else:
            self.get_logger().info("Waiting for client connection")
    
    def destroy_node(self):
        """Clean up resources when node is shutting down"""
        self.get_logger().info("Shutting down velocity listener")
        self.running = False
        
        # Close sockets
        with self.connection_lock:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
            
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        velocity_listener = VelocityListener()
        rclpy.spin(velocity_listener)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in velocity listener node: {e}")
    finally:
        if 'velocity_listener' in locals():
            velocity_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 