#!/usr/bin/env python3
"""
Waypoint Listener Node

This node sets up a TCP server to receive waypoint data from CARLA/Gui_Control.
It processes the waypoints and publishes them for visualization and path planning.

Author: Mostafa
"""

import socket
import struct
import time
import threading
import numpy as np
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, Int32MultiArray, Int32, Bool
from builtin_interfaces.msg import Duration

class WaypointListener(Node):
    def __init__(self):
        super().__init__('waypoint_listener')
        
        # Declare parameters
        self.declare_parameter('tcp_ip', '127.0.0.1')  # Listen on all interfaces
        self.declare_parameter('tcp_port', 12343)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('waypoints_topic', '/carla/waypoints')
        self.declare_parameter('marker_topic', '/carla/waypoint_markers')
        self.declare_parameter('waypoint_marker_size', 0.2)
        self.declare_parameter('waypoint_line_width', 0.05)
        self.declare_parameter('waypoint_lifetime', 0.5)
        self.declare_parameter('socket_buffer_size', 262144)
        self.declare_parameter('socket_timeout', 0.5)
        self.declare_parameter('enable_tcp_nodelay', True)
        self.declare_parameter('enable_socket_keepalive', True)
        self.declare_parameter('verbose_logging', True)
        self.declare_parameter('reconnect_interval', 2.0)  # Seconds to wait before reconnect attempts
        self.declare_parameter('connection_timeout', 10.0)  # Seconds to wait for connection
        self.declare_parameter('current_point_topic', '/carla/current_waypoint')
        self.declare_parameter('lane_change_status_topic', '/carla/lane_change_status')
        


        # carla.LaneType.NONE: "None",
        # carla.LaneType.Driving: "Driving",
        # carla.LaneType.Stop: "Stop",
        # carla.LaneType.Shoulder: "Shoulder",
        # carla.LaneType.Bidirectional: "Bidirectional",
        # carla.LaneType.Parking: "Parking",
        # carla.LaneType.Restricted: "Restricted",
        # carla.LaneType.Border: "Border",
        # carla.LaneType.Sidewalk: "Sidewalk",
        # carla.LaneType.Biking: "Biking",
        # carla.LaneType.Tram: "Tram",
        # carla.LaneType.Rail: "Rail"
        # Lane type mapping dictionary
        self.lane_type_map = {
            0: "None",
            1: "stop",
            2: "driving",
            32: "sidewalk",
            4: "bidirectional",
            5: "parking",
            6: "restricted",
            7: "border",
            8: "shoulder",
            9: "Biking",
            10: "Tram",
            11: "Rail"
        }
        
        # Get parameters
        self.tcp_ip = self.get_parameter('tcp_ip').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value
        self.waypoints_topic = self.get_parameter('waypoints_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.waypoint_marker_size = self.get_parameter('waypoint_marker_size').get_parameter_value().double_value
        self.waypoint_line_width = self.get_parameter('waypoint_line_width').get_parameter_value().double_value
        self.waypoint_lifetime = self.get_parameter('waypoint_lifetime').get_parameter_value().double_value
        self.socket_buffer_size = self.get_parameter('socket_buffer_size').get_parameter_value().integer_value
        self.socket_timeout = self.get_parameter('socket_timeout').get_parameter_value().double_value
        self.enable_tcp_nodelay = self.get_parameter('enable_tcp_nodelay').get_parameter_value().bool_value
        self.enable_socket_keepalive = self.get_parameter('enable_socket_keepalive').get_parameter_value().bool_value
        self.verbose_logging = self.get_parameter('verbose_logging').get_parameter_value().bool_value
        self.reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.connection_timeout = self.get_parameter('connection_timeout').get_parameter_value().double_value
        self.current_point_topic = self.get_parameter('current_point_topic').get_parameter_value().string_value
        self.lane_change_status_topic = self.get_parameter('lane_change_status_topic').get_parameter_value().string_value
        
        # Set up QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.waypoint_publisher = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            qos_profile
        )
        
        # Also publish waypoints as PoseArray for path planning
        self.pose_publisher = self.create_publisher(
            PoseArray,
            self.waypoints_topic,
            qos_profile
        )
        
        # And publish metadata as Int32MultiArray
        self.metadata_publisher = self.create_publisher(
            Int32MultiArray,
            self.waypoints_topic + "_metadata",
            qos_profile
        )
        
        # Add after other publishers
        self.current_point_pub = self.create_publisher(Int32, self.current_point_topic, qos_profile)
        self.lane_change_status_pub = self.create_publisher(Bool, self.lane_change_status_topic, qos_profile)
        
        # Socket variables
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.running = True
        self.waypoint_queue = Queue(maxsize=10)  # Increased queue size
        self.connection_lock = threading.Lock()  # Add lock for connection management
        
        # Add after other class variables
        self.current_point_index = 0
        self.total_points = 0
        self.points_in_current_lane = 0
        self.lane_change_performed = False
        self.current_lane_id = None
        
        # Setup TCP server
        self.setup_tcp_server()
        
        # Flag to track connection status
        self.is_connected = False
        self.connection_event = threading.Event()
        
        # Start processing threads
        self.server_thread = threading.Thread(target=self.accept_connections)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.process_thread = threading.Thread(target=self.process_waypoints)
        self.process_thread.daemon = True
        self.process_thread.start()
        
        # Create a timer for status updates
        self.status_timer = self.create_timer(5.0, self.log_status)
        
        self.get_logger().info("Waypoint listener initialized and waiting for connections")
        
        # Wait for initial connection
        self.wait_for_connection()
        
        # Add subscription for vehicle position updates (if available)
        self.create_subscription(
            Point,  # Assuming you have a topic publishing vehicle position
            '/carla/vehicle_position',
            self.vehicle_position_callback,
            qos_profile
        )
        
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
                receive_thread = threading.Thread(target=self.receive_waypoints, args=(client_socket,))
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
    
    def receive_waypoints(self, client_socket):
        """Receive waypoints from client via TCP"""
        client_socket.settimeout(self.socket_timeout)
        receive_errors = 0
        max_errors = 5  # Maximum consecutive errors before closing connection
        
        self.get_logger().info("Starting to receive waypoints from client")
        print("\n[DEBUG] Starting to receive waypoints from client\n")
        
        while self.running:
            try:
                # First receive the number of waypoints
                self.get_logger().info("Waiting to receive waypoint count...")
                print("\n[DEBUG] Waiting to receive waypoint count...\n")
                
                num_waypoints_data = client_socket.recv(4)
                if not num_waypoints_data or len(num_waypoints_data) < 4:
                    self.get_logger().warn("Connection closed by client or incomplete data")
                    print("\n[DEBUG] Connection closed or incomplete data received\n")
                    break
                    
                num_waypoints = struct.unpack('!I', num_waypoints_data)[0]
                
                self.get_logger().info(f"Received waypoint count: {num_waypoints}")
                print(f"\n[DEBUG] Received waypoint count: {num_waypoints}\n")
                
                if self.verbose_logging:
                    self.get_logger().info(f"Receiving {num_waypoints} waypoints")
                
                # Each waypoint has: x, y, z, road_id, lane_id, lane_type, has_right_lane
                # Format: !fffiii (3 floats, 3 ints) = 28 bytes
                expected_bytes = num_waypoints * 28
                
                self.get_logger().info(f"Expecting {expected_bytes} bytes of waypoint data")
                print(f"\n[DEBUG] Expecting {expected_bytes} bytes of waypoint data\n")
                
                # Receive all waypoint data
                waypoint_data = bytearray()
                bytes_received = 0
                
                while bytes_received < expected_bytes:
                    chunk = client_socket.recv(min(self.socket_buffer_size, expected_bytes - bytes_received))
                    if not chunk:
                        self.get_logger().warn("Connection closed by client during data transfer")
                        print("\n[DEBUG] Connection closed during data transfer\n")
                        break
                    waypoint_data.extend(chunk)
                    bytes_received += len(chunk)
                    
                    # Log progress for large transfers
                    if expected_bytes > 1000 and bytes_received % 1000 == 0:
                        self.get_logger().info(f"Received {bytes_received}/{expected_bytes} bytes")
                
                if bytes_received == expected_bytes:
                    self.get_logger().info(f"Successfully received {bytes_received} bytes of waypoint data")
                    print(f"\n[DEBUG] Successfully received {bytes_received} bytes of waypoint data\n")
                    
                    # Process the waypoint data
                    waypoints = []
                    for i in range(num_waypoints):
                        offset = i * 28
                        try:
                            # Unpack x, y, z (floats) and road_id, lane_id, lane_type, has_right_lane (ints)
                            x, y, z, road_id, lane_id, lane_type, has_right_lane = struct.unpack('!fffiiii', waypoint_data[offset:offset+28])
                            
                            waypoints.append({
                                'x': x,
                                'y': y,
                                'z': z,
                                'road_id': road_id,
                                'lane_id': lane_id,
                                'lane_type': lane_type,
                                'has_right_lane': has_right_lane
                            })
                            
                            # Add detailed logging for each waypoint with lane type string
                            lane_type_str = self.get_lane_type_string(lane_type)
                            print(f"Received waypoint: x={x:.2f}, y={y:.2f}, z={z:.2f}, road_id={road_id}, lane_id={lane_id}, lane_type={lane_type} ({lane_type_str}), has_right_lane={has_right_lane}")
                        except Exception as e:
                            self.get_logger().error(f"Error unpacking waypoint at index {i}: {e}")
                            print(f"\n[DEBUG] Error unpacking waypoint at index {i}: {e}\n")
                            continue

                    # save waypoints to file
                    try:
                        with open('/tmp/waypoints.txt', 'a') as f:
                            f.write("x, y, z, road_id, lane_id, lane_type_int, lane_type_str, has_right_lane\n")  # Add header line with column names
                            for wp in waypoints:
                                lane_type_str = self.get_lane_type_string(wp['lane_type'])
                                f.write(f"{wp['x']}, {wp['y']}, {wp['z']}, {wp['road_id']}, {wp['lane_id']}, {wp['lane_type']}, {lane_type_str}, {wp['has_right_lane']}\n")
                            print(f"Saved {len(waypoints)} waypoints to /tmp/waypoints.txt")
                    except Exception as e:
                        self.get_logger().error(f"Error saving waypoints to file: {e}")
                        print(f"\n[DEBUG] Error saving waypoints to file: {e}\n")
                    
                    # Put waypoints in queue, replacing old data if queue is full
                    if self.waypoint_queue.full():
                        try:
                            self.waypoint_queue.get_nowait()
                        except Empty:
                            pass
                    self.waypoint_queue.put(waypoints)
                    
                    if self.verbose_logging and num_waypoints > 0:
                        first_wp = waypoints[0]
                        lane_type_str = self.get_lane_type_string(first_wp['lane_type'])
                        right_lane_str = "with right lane" if first_wp['has_right_lane'] == 1 else "no right lane"
                        self.get_logger().info(f"Received {num_waypoints} waypoints. First waypoint: pos=({first_wp['x']:.2f}, {first_wp['y']:.2f}, {first_wp['z']:.2f}), road={first_wp['road_id']}, lane={first_wp['lane_id']}, type={first_wp['lane_type']} ({lane_type_str}), {right_lane_str}")
                    
                    # Reset error counter on successful receive
                    receive_errors = 0
                else:
                    receive_errors += 1
                    self.get_logger().warn(f"Incomplete data received: got {bytes_received}/{expected_bytes} bytes")
                
            except socket.timeout:
                # This is normal, just continue
                continue
            except ConnectionResetError:
                self.get_logger().error("Connection reset by client")
                print("\n[DEBUG] Connection reset by client\n")
                break
            except BrokenPipeError:
                self.get_logger().error("Broken pipe error - connection lost")
                print("\n[DEBUG] Broken pipe error - connection lost\n")
                break
            except Exception as e:
                receive_errors += 1
                self.get_logger().error(f"Error receiving waypoints: {e}")
                print(f"\n[DEBUG] Error receiving waypoints: {e}\n")
                if receive_errors >= max_errors:
                    self.get_logger().error(f"Too many consecutive errors ({receive_errors}), closing connection")
                    break
                time.sleep(0.1)  # Short delay before retry
        
        # Connection closed or error
        self.get_logger().info("Client disconnected, waiting for new connection")
        try:
            client_socket.close()
        except:
            pass
        with self.connection_lock:
            if client_socket == self.client_socket:
                self.client_socket = None
                self.client_address = None
                self.is_connected = False
                self.connection_event.clear()  # Reset connection event
    
    def process_waypoints(self):
        """Process waypoints from queue and publish them"""
        while self.running:
            try:
                if not self.waypoint_queue.empty():
                    waypoints = self.waypoint_queue.get()
                    
                    # Store waypoints for position tracking
                    self.current_waypoints = waypoints
                    self.total_points = len(waypoints)
                    
                    # Reset tracking when new waypoints received
                    self.current_point_index = 0
                    self.points_in_current_lane = 0
                    self.lane_change_performed = False
                    self.current_lane_id = None
                    
                    # Publish waypoints
                    self.publish_waypoint_markers(waypoints)
                    self.publish_waypoint_poses(waypoints)
                    
                    if self.verbose_logging:
                        self.get_logger().info(f"Processed and published {len(waypoints)} waypoints")
                else:
                    time.sleep(0.01)
            except Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error processing waypoints: {e}")
                time.sleep(0.1)
    
    def publish_waypoint_poses(self, waypoints):
        """Publish waypoints as PoseArray and metadata for path planning"""
        if not waypoints:
            return
            
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = self.map_frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        # Create Int32MultiArray for metadata
        metadata = Int32MultiArray()
        metadata_values = []
        
        # Process waypoints
        for wp in waypoints:
            # Add pose to pose array
            pose = Pose()
            pose.position.x = float(wp['x'])
            pose.position.y = float(wp['y'])
            pose.position.z = float(wp['z'])
            
            # Set orientation to default (identity quaternion)
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
            
            # Add metadata
            metadata_values.append(wp['road_id'])
            metadata_values.append(wp['lane_id'])
            metadata_values.append(wp['lane_type'])
            metadata_values.append(wp['has_right_lane'])  # Add right lane availability information
        
        # Set metadata values
        metadata.data = metadata_values
        
        # Publish messages
        self.pose_publisher.publish(pose_array)
        self.metadata_publisher.publish(metadata)
    
    def get_lane_type_string(self, lane_type):
        """Convert lane type number to descriptive string"""
        # Make sure lane_type is treated as an integer
        try:
            lane_type_int = int(lane_type)
            # Check if lane_type_int is within the valid range (0-7)
            if 0 <= lane_type_int <= 32:
                return self.lane_type_map.get(lane_type_int)
            else:
                return f"Unknown({lane_type_int})"
        except (ValueError, TypeError):
            # If lane_type is already a string, return it as is
            if isinstance(lane_type, str):
                return lane_type
            return f"Unknown({lane_type})"
    
    def publish_waypoint_markers(self, waypoints):
        """Publish waypoints as visualization markers"""
        if not waypoints:
            return
            
        marker_array = MarkerArray()
        
        # Create a marker for waypoint points
        points_marker = Marker()
        points_marker.header.frame_id = self.map_frame_id
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "waypoint_points"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.scale.x = self.waypoint_marker_size
        points_marker.scale.y = self.waypoint_marker_size
        points_marker.scale.z = self.waypoint_marker_size
        points_marker.pose.orientation.w = 1.0
        
        # Create Duration message properly
        lifetime_sec = int(self.waypoint_lifetime)
        lifetime_nsec = int((self.waypoint_lifetime - lifetime_sec) * 1e9)
        points_marker.lifetime.sec = lifetime_sec
        points_marker.lifetime.nanosec = lifetime_nsec
        
        # Create a marker for waypoint lines
        lines_marker = Marker()
        lines_marker.header.frame_id = self.map_frame_id
        lines_marker.header.stamp = self.get_clock().now().to_msg()
        lines_marker.ns = "waypoint_lines"
        lines_marker.id = 1
        lines_marker.type = Marker.LINE_STRIP
        lines_marker.action = Marker.ADD
        lines_marker.scale.x = self.waypoint_line_width
        lines_marker.pose.orientation.w = 1.0
        
        # Set lifetime the same way
        lines_marker.lifetime.sec = lifetime_sec
        lines_marker.lifetime.nanosec = lifetime_nsec
        
        # Create a marker for right lane availability
        right_lane_marker = Marker()
        right_lane_marker.header.frame_id = self.map_frame_id
        right_lane_marker.header.stamp = self.get_clock().now().to_msg()
        right_lane_marker.ns = "right_lane_availability"
        right_lane_marker.id = 2
        right_lane_marker.type = Marker.SPHERE_LIST
        right_lane_marker.action = Marker.ADD
        right_lane_marker.scale.x = self.waypoint_marker_size * 1.5
        right_lane_marker.scale.y = self.waypoint_marker_size * 1.5
        right_lane_marker.scale.z = self.waypoint_marker_size * 1.5
        right_lane_marker.pose.orientation.w = 1.0
        right_lane_marker.lifetime.sec = lifetime_sec
        right_lane_marker.lifetime.nanosec = lifetime_nsec
        
        # Create a marker for lane type information
        lane_type_markers = []
        
        # Create a marker for lane IDs
        text_markers = []
        
        # Process waypoints
        for i, wp in enumerate(waypoints):
            # Add point to points marker
            p = Point()
            p.x = float(wp['x'])
            p.y = float(wp['y'])
            p.z = float(wp['z'])
            
            # Set color based on lane type
            color = ColorRGBA()
            lane_type = wp['lane_type']
            has_right_lane = wp['has_right_lane']
            
            # Color based on lane type
            # 0: None, 1: Driving, 2: Stop, 3: Shoulder, 4: Biking, 5: Sidewalk, 6: Border, 7: Restricted, 8: Parking, 9: Bidirectional
            color.a = 1.0  # Set full opacity for all colors
            if lane_type == 0:  # None
                color.r = 0.5
                color.g = 0.5
                color.b = 0.5
            elif lane_type == 1:  # Driving
                color.r = 0.0
                color.g = 1.0
                color.b = 0.0
            elif lane_type == 2:  # Stop
                color.r = 1.0
                color.g = 0.0
                color.b = 0.0
            elif lane_type == 3:  # Shoulder
                color.r = 1.0
                color.g = 1.0
                color.b = 0.0
            elif lane_type == 4:  # Biking
                color.r = 0.0
                color.g = 0.0
                color.b = 1.0
            elif lane_type == 5:  # Sidewalk
                color.r = 1.0
                color.g = 0.0
                color.b = 1.0
            elif lane_type == 6:  # Border
                color.r = 0.5
                color.g = 0.5
                color.b = 0.5
            elif lane_type == 7:  # Restricted
                color.r = 1.0
                color.g = 0.5
                color.b = 0.0
            elif lane_type == 8:  # Parking
                color.r = 0.0
                color.g = 0.5
                color.b = 1.0
            elif lane_type == 9:  # Bidirectional
                color.r = 0.5
                color.g = 0.0
                color.b = 0.5
            elif lane_type == 10:  # Tram
                color.r = 0.0
                color.g = 1.0
                color.b = 1.0
            elif lane_type == 11:  # Rail
                color.r = 1.0   
            else:  # Unknown
                color.r = 1.0
                color.g = 1.0
                color.b = 1.0
            
            
            # Add to standard points marker
            points_marker.points.append(p)
            points_marker.colors.append(color)
            
            # Add to lines marker
            lines_marker.points.append(p)
            lines_marker.colors.append(color)
            
            # If right lane is available, add to right lane marker with special color
            if has_right_lane == 1:
                right_lane_color = ColorRGBA()
                right_lane_color.r = 0.0
                right_lane_color.g = 0.0
                right_lane_color.b = 1.0  # Blue for right lane availability
                right_lane_color.a = 0.7
                
                right_lane_marker.points.append(p)
                right_lane_marker.colors.append(right_lane_color)
                
            # Create text marker for lane ID every few waypoints
            if i % 5 == 0:
                text_marker = Marker()
                text_marker.header.frame_id = self.map_frame_id
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "lane_ids"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = p.x
                text_marker.pose.position.y = p.y
                text_marker.pose.position.z = p.z + 0.5  # Slightly above the waypoint
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.5  # Text height
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 0.8
                
                # Include right lane info in text
                right_lane_text = "R" if has_right_lane == 1 else ""
                
                # Get lane type string
                lane_type_str = self.get_lane_type_string(lane_type)
                
                # Format text with road_id, lane_id, lane_type, and right lane indicator
                text_marker.text = f"{wp['road_id']}/{wp['lane_id']} {lane_type_str} {right_lane_text}"
                
                text_marker.lifetime.sec = lifetime_sec
                text_marker.lifetime.nanosec = lifetime_nsec
                
                text_markers.append(text_marker)
                
                # Create a separate marker for lane type that's more visible
                lane_type_marker = Marker()
                lane_type_marker.header.frame_id = self.map_frame_id
                lane_type_marker.header.stamp = self.get_clock().now().to_msg()
                lane_type_marker.ns = "lane_types"
                lane_type_marker.id = i
                lane_type_marker.type = Marker.TEXT_VIEW_FACING
                lane_type_marker.action = Marker.ADD
                lane_type_marker.pose.position.x = p.x
                lane_type_marker.pose.position.y = p.y
                lane_type_marker.pose.position.z = p.z + 1.0  # Higher above the waypoint
                lane_type_marker.pose.orientation.w = 1.0
                lane_type_marker.scale.z = 0.7  # Larger text
                
                # Color based on lane type
                lane_type_marker.color = ColorRGBA()
                if lane_type == 0:  # None
                    lane_type_marker.color.r = 0.5
                    lane_type_marker.color.g = 0.5
                    lane_type_marker.color.b = 0.5
                elif lane_type == 1:  # Driving
                    lane_type_marker.color.r = 0.0
                    lane_type_marker.color.g = 1.0
                    lane_type_marker.color.b = 0.0
                elif lane_type == 2:  # Stop
                    lane_type_marker.color.r = 1.0
                    lane_type_marker.color.g = 0.0
                    lane_type_marker.color.b = 0.0
                elif lane_type == 3:  # Shoulder
                    lane_type_marker.color.r = 1.0
                    lane_type_marker.color.g = 1.0
                    lane_type_marker.color.b = 0.0
                elif lane_type == 4:  # Biking
                    lane_type_marker.color.r = 0.0
                    lane_type_marker.color.g = 0.0
                    lane_type_marker.color.b = 1.0
                elif lane_type == 5:  # Sidewalk
                    lane_type_marker.color.r = 1.0
                    lane_type_marker.color.g = 0.0
                    lane_type_marker.color.b = 1.0
                elif lane_type == 6:  # Border
                    lane_type_marker.color.r = 0.5
                    lane_type_marker.color.g = 0.5
                    lane_type_marker.color.b = 0.5
                elif lane_type == 7:  # Restricted
                    lane_type_marker.color.r = 1.0
                    lane_type_marker.color.g = 0.5
                    lane_type_marker.color.b = 0.0
                elif lane_type == 8:  # Parking
                    lane_type_marker.color.r = 0.0
                    lane_type_marker.color.g = 0.5
                    lane_type_marker.color.b = 1.0
                elif lane_type == 9:  # Bidirectional
                    lane_type_marker.color.r = 0.5
                    lane_type_marker.color.g = 0.0
                    lane_type_marker.color.b = 0.5
                else:  # Unknown
                    lane_type_marker.color.r = 1.0
                    lane_type_marker.color.g = 1.0
                    lane_type_marker.color.b = 1.0
                
                lane_type_marker.color.a = 1.0
                lane_type_marker.text = lane_type_str
                
                lane_type_marker.lifetime.sec = lifetime_sec
                lane_type_marker.lifetime.nanosec = lifetime_nsec
                
                lane_type_markers.append(lane_type_marker)
        
        # Add markers to array
        marker_array.markers.append(points_marker)
        marker_array.markers.append(lines_marker)
        marker_array.markers.append(right_lane_marker)
        
        # Add text markers
        for marker in text_markers:
            marker_array.markers.append(marker)
            
        # Add lane type markers
        for marker in lane_type_markers:
            marker_array.markers.append(marker)
        
        # Publish marker array
        self.waypoint_publisher.publish(marker_array)
    
    def log_status(self):
        """Log the current status of the node"""
        with self.connection_lock:
            if self.client_socket:
                self.get_logger().info(f"Connected to client: {self.client_address}")
            else:
                self.get_logger().info("Waiting for client connection")
    
    def destroy_node(self):
        """Clean up resources when the node is shut down"""
        self.running = False
        
        with self.connection_lock:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
                self.client_socket = None
            
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass
                self.server_socket = None
            
            self.is_connected = False
            self.connection_event.set()  # Set event to unblock any waiting threads
        
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=1.0)
            
        if self.process_thread and self.process_thread.is_alive():
            self.process_thread.join(timeout=1.0)
        
        super().destroy_node()

    def vehicle_position_callback(self, msg):
        """Update current waypoint based on vehicle position"""
        if not hasattr(self, 'current_waypoints') or not self.current_waypoints:
            return
            
        # Find closest waypoint to current position
        min_dist = float('inf')
        closest_idx = self.current_point_index
        
        # Look at next few waypoints to find closest
        search_range = min(5, len(self.current_waypoints) - self.current_point_index)
        for i in range(self.current_point_index, self.current_point_index + search_range):
            wp = self.current_waypoints[i]
            dist = ((wp.x - msg.x) ** 2 + (wp.y - msg.y) ** 2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx != self.current_point_index:
            self.update_current_point(closest_idx)

    def update_current_point(self, new_index):
        """Update the current point index and related tracking"""
        if not hasattr(self, 'current_waypoints') or not self.current_waypoints:
            return
            
        self.current_point_index = new_index
        
        # Publish current point index
        msg = Int32()
        msg.data = self.current_point_index
        self.current_point_pub.publish(msg)
        
        # Track points in current lane
        current_wp = self.current_waypoints[self.current_point_index]
        if self.current_lane_id is None:
            self.current_lane_id = current_wp.lane_id
            self.points_in_current_lane = 1
        elif current_wp.lane_id == self.current_lane_id:
            self.points_in_current_lane += 1
        else:
            # Lane changed
            self.current_lane_id = current_wp.lane_id
            self.points_in_current_lane = 1
            self.lane_change_performed = True
            
            # Publish lane change status
            msg = Bool()
            msg.data = True
            self.lane_change_status_pub.publish(msg)
            
            self.get_logger().info(f"Lane change detected at point {self.current_point_index}")

def main(args=None):
    rclpy.init(args=args)
    waypoint_listener = WaypointListener()
    
    try:
        rclpy.spin(waypoint_listener)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 