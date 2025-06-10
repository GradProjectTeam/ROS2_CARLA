#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import socket
import struct
import numpy as np
import cv2
import math
from collections import deque

class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor')
        
        # Fixed camera resolution
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.CHANNELS = 4  # RGBA format
        
        # Create publisher for camera images
        self.image_publisher = self.create_publisher(
            Image,
            '/carla/camera/rgb/image_raw',
            10)
            
        # Create publisher for processed lane images
        self.processed_image_publisher = self.create_publisher(
            Image,
            '/carla/camera/processed_lanes',
            10)
            
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Socket setup
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 12342)  # Port for four_sensors camera
        
        self.server_host = self.get_parameter('server_host').value
        self.server_port = self.get_parameter('server_port').value
        
        # Create a timer for connection attempts
        self.socket = None
        self.connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 20  # Try 20 times before giving up
        
        # Memory for lane tracking
        self.last_valid_right_lane = None
        self.last_valid_left_lane = None
        self.last_lane_center = 320  # Default center
        self.lane_memory_frames = 10  # Remember lanes for this many frames
        
        # Queue to store recent lane positions for smoothing
        self.right_lane_history = deque(maxlen=5)
        self.left_lane_history = deque(maxlen=5)
        self.lane_center_history = deque(maxlen=5)
        self.lane_center_history.append(320)  # Initialize with center position
        
        # Define expected lane positions
        self.expected_left_x = 220   # Expected x-position of left lane at bottom of image
        self.expected_right_x = 420  # Expected x-position of right lane at bottom of image
        self.lane_width_tolerance = 100  # Tolerance for lane width variation
        
        # Lane detection parameters
        self.min_lane_points = 5     # Minimum points needed to fit a lane
        self.min_line_length = 10    # Minimum line length for Hough transform
        self.max_line_gap = 20       # Maximum line gap for Hough transform
        self.lane_detection_threshold = 25  # Hough transform threshold

        # Declare parameters for lane detection configuration
        self.declare_parameter('canny_low_threshold', 50)
        self.declare_parameter('canny_high_threshold', 100)
        self.declare_parameter('hough_threshold', 25)
        self.declare_parameter('min_line_length', 10)
        self.declare_parameter('max_line_gap', 20)
        self.declare_parameter('lane_fill_alpha', 0.9)  # Transparency of lane fill

        # Get parameters
        self.canny_low_threshold = self.get_parameter('canny_low_threshold').value
        self.canny_high_threshold = self.get_parameter('canny_high_threshold').value
        self.hough_threshold = self.get_parameter('hough_threshold').value
        self.min_line_length = self.get_parameter('min_line_length').value
        self.max_line_gap = self.get_parameter('max_line_gap').value
        self.lane_fill_alpha = self.get_parameter('lane_fill_alpha').value

        # Create connection timer
        self.create_timer(1.0, self.connect_to_camera_server)

    def connect_to_camera_server(self):
        """Attempt to connect to the camera server"""
        if self.connected:
            # Already connected, no need to reconnect
            return
            
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            self.get_logger().error('Maximum reconnection attempts reached. Giving up.')
            return
            
        try:
            # Close any existing socket
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
                    
            # Create a new socket and attempt to connect
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)  # 5 second timeout for connection
            
            self.get_logger().info(f'Connecting to camera server at {self.server_host}:{self.server_port} (attempt {self.reconnect_attempts+1}/{self.max_reconnect_attempts})')
            self.socket.connect((self.server_host, self.server_port))
            
            # Connection successful
            self.connected = True
            self.reconnect_attempts = 0
            self.get_logger().info(f'Successfully connected to camera server, expecting {self.IMAGE_WIDTH}x{self.IMAGE_HEIGHT} images')
            
            # Create timer for processing camera data once connected
            self.create_timer(0.1, self.timer_callback)  # 10Hz
            
        except ConnectionRefusedError:
            self.reconnect_attempts += 1
            self.get_logger().warn(f'Connection refused. Is the camera server running? Retrying in 1 second...')
            
        except Exception as e:
            self.reconnect_attempts += 1
            self.get_logger().error(f'Failed to connect to camera server: {e}. Retrying in 1 second...')

    def publish_camera_tf(self, timestamp):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        
        # Adjusted camera position for 640x480 view
        t.transform.translation.x = 1.5  # 1.5m forward
        t.transform.translation.y = 0.0  # centered
        t.transform.translation.z = 1.2  # 1.2m up
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = -0.13053
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.99144
        
        self.tf_broadcaster.sendTransform(t)

    def sumMatrix(self, pt1, pt2):
        """Add two points together and return the result as a list"""
        A = np.array(pt1)
        B = np.array(pt2)
        ans = A + B
        return ans.tolist()
    
    def detect_lanes(self, image):
        """
        Detect lanes in an image using improved algorithm
        from the CARLA Lane Detection code
        """
        # Ensure image is RGB
        if image.shape[2] == 4:  # RGBA
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
        elif image.shape[2] == 3:  # BGR
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
        # Resize the image to VGA resolution if needed
        if image.shape[0] != 480 or image.shape[1] != 640:
            size_im = cv2.resize(image, dsize=(640, 480))
        else:
            size_im = image.copy()
            
        # Define the region of interest (ROI)
        roi = size_im[240:480, 108:532]  # Bottom half of the image
        roi_im = cv2.resize(roi, (424, 240))
        
        # Apply Gaussian Blur to the ROI
        blur_im = cv2.bilateralFilter(roi_im, d=-1, sigmaColor=5, sigmaSpace=5)
        
        # Detect edges using Canny edge detector
        edges = cv2.Canny(blur_im, self.canny_low_threshold, self.canny_high_threshold)
        
        # Apply Hough Transformation to detect lines
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180.0, threshold=self.hough_threshold, 
                               minLineLength=self.min_line_length, maxLineGap=self.max_line_gap)
        
        # Initialize variables for lane detection
        pt1_sum_ri = [0, 0]
        pt2_sum_ri = [0, 0]
        pt1_sum_le = [0, 0]
        pt2_sum_le = [0, 0]
        count_posi_num_ri = 0
        count_posi_num_le = 0
        
        # Process detected lines
        valid_right_lane = False
        valid_left_lane = False
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Skip vertical lines to avoid division by zero
                if x2 == x1:
                    continue
                
                # Calculate line slope and angle
                slope = (y2 - y1) / (x2 - x1)
                angle = math.atan(slope) * 180.0 / math.pi
                
                # Adjust coordinates back to original image space
                pt1_ri = [x1 + 108, y1 + 240]
                pt2_ri = [x2 + 108, y2 + 240]
                pt1_le = [x1 + 108, y1 + 240]
                pt2_le = [x2 + 108, y2 + 240]
                
                # Classify as right lane (positive slope)
                if 20.0 < angle < 90.0:
                    count_posi_num_ri += 1
                    pt1_sum_ri = self.sumMatrix(pt1_ri, pt1_sum_ri)
                    pt2_sum_ri = self.sumMatrix(pt2_ri, pt2_sum_ri)
                    
                # Classify as left lane (negative slope)
                elif -90.0 < angle < -20.0:
                    count_posi_num_le += 1
                    pt1_sum_le = self.sumMatrix(pt1_le, pt1_sum_le)
                    pt2_sum_le = self.sumMatrix(pt2_le, pt2_sum_le)
        
        # Process right lane if valid points were found
        if count_posi_num_ri > 0:
            valid_right_lane = True
            pt1_avg_ri = (np.array(pt1_sum_ri) / count_posi_num_ri).astype(int)
            pt2_avg_ri = (np.array(pt2_sum_ri) / count_posi_num_ri).astype(int)
            x1_avg_ri, y1_avg_ri = pt1_avg_ri
            x2_avg_ri, y2_avg_ri = pt2_avg_ri
            
            # Store valid right lane for future frames
            self.last_valid_right_lane = (pt1_avg_ri, pt2_avg_ri)
            self.right_lane_history.append((pt1_avg_ri, pt2_avg_ri))
            
            # Calculate slope and intercept for line extension
            if x2_avg_ri != x1_avg_ri:  # Avoid division by zero
                a_avg_ri = (y2_avg_ri - y1_avg_ri) / (x2_avg_ri - x1_avg_ri)
                b_avg_ri = y2_avg_ri - (a_avg_ri * x2_avg_ri)
                
                # Extend line to bottom of image
                pt2_y2_fi_ri = 480
                
                # Check if slope is valid before calculating x-coordinate
                if a_avg_ri != 0:
                    pt2_x2_fi_ri = int((pt2_y2_fi_ri - b_avg_ri) / a_avg_ri)
                    # Ensure point is within image bounds
                    pt2_x2_fi_ri = max(0, min(640, pt2_x2_fi_ri))
                    pt2_fi_ri = (pt2_x2_fi_ri, pt2_y2_fi_ri)
                    
                    # Draw the right lane line
                    cv2.line(size_im, tuple(pt1_avg_ri), pt2_fi_ri, (0, 255, 0), 2)
                else:
                    valid_right_lane = False
            else:
                valid_right_lane = False
        else:
            # Use last valid right lane if available
            if self.last_valid_right_lane is not None:
                pt1_avg_ri, pt2_avg_ri = self.last_valid_right_lane
                x1_avg_ri, y1_avg_ri = pt1_avg_ri
                x2_avg_ri, y2_avg_ri = pt2_avg_ri
                
                # Calculate slope and intercept
                if x2_avg_ri != x1_avg_ri:
                    a_avg_ri = (y2_avg_ri - y1_avg_ri) / (x2_avg_ri - x1_avg_ri)
                    b_avg_ri = y2_avg_ri - (a_avg_ri * x2_avg_ri)
                    
                    # Extend line to bottom of image
                    pt2_y2_fi_ri = 480
                    pt2_x2_fi_ri = int((pt2_y2_fi_ri - b_avg_ri) / a_avg_ri)
                    # Ensure point is within image bounds
                    pt2_x2_fi_ri = max(0, min(640, pt2_x2_fi_ri))
                    pt2_fi_ri = (pt2_x2_fi_ri, pt2_y2_fi_ri)
                    
                    # Draw the right lane line with reduced opacity to indicate it's from memory
                    cv2.line(size_im, tuple(pt1_avg_ri), pt2_fi_ri, (0, 128, 0), 2)
                    valid_right_lane = True
                else:
                    valid_right_lane = False

        # Process left lane if valid points were found
        if count_posi_num_le > 0:
            valid_left_lane = True
            pt1_avg_le = (np.array(pt1_sum_le) / count_posi_num_le).astype(int)
            pt2_avg_le = (np.array(pt2_sum_le) / count_posi_num_le).astype(int)
            x1_avg_le, y1_avg_le = pt1_avg_le
            x2_avg_le, y2_avg_le = pt2_avg_le
            
            # Store valid left lane for future frames
            self.last_valid_left_lane = (pt1_avg_le, pt2_avg_le)
            self.left_lane_history.append((pt1_avg_le, pt2_avg_le))
            
            # Calculate slope and intercept for line extension
            if x2_avg_le != x1_avg_le:  # Avoid division by zero
                a_avg_le = (y2_avg_le - y1_avg_le) / (x2_avg_le - x1_avg_le)
                b_avg_le = y2_avg_le - (a_avg_le * x2_avg_le)
                
                # Extend line to bottom of image
                pt1_y1_fi_le = 480
                
                # Check if slope is valid before calculating x-coordinate
                if a_avg_le != 0:
                    pt1_x1_fi_le = int((pt1_y1_fi_le - b_avg_le) / a_avg_le)
                    # Ensure point is within image bounds
                    pt1_x1_fi_le = max(0, min(640, pt1_x1_fi_le))
                    pt1_fi_le = (pt1_x1_fi_le, pt1_y1_fi_le)
                    
                    # Draw the left lane line
                    cv2.line(size_im, tuple(pt2_avg_le), pt1_fi_le, (0, 255, 0), 2)
                else:
                    valid_left_lane = False
            else:
                valid_left_lane = False
        else:
            # Use last valid left lane if available
            if self.last_valid_left_lane is not None:
                pt1_avg_le, pt2_avg_le = self.last_valid_left_lane
                x1_avg_le, y1_avg_le = pt1_avg_le
                x2_avg_le, y2_avg_le = pt2_avg_le
                
                # Calculate slope and intercept
                if x2_avg_le != x1_avg_le:
                    a_avg_le = (y2_avg_le - y1_avg_le) / (x2_avg_le - x1_avg_le)
                    b_avg_le = y2_avg_le - (a_avg_le * x2_avg_le)
                    
                    # Extend line to bottom of image
                    pt1_y1_fi_le = 480
                    pt1_x1_fi_le = int((pt1_y1_fi_le - b_avg_le) / a_avg_le)
                    # Ensure point is within image bounds
                    pt1_x1_fi_le = max(0, min(640, pt1_x1_fi_le))
                    pt1_fi_le = (pt1_x1_fi_le, pt1_y1_fi_le)
                    
                    # Draw the left lane line with reduced opacity to indicate it's from memory
                    cv2.line(size_im, tuple(pt2_avg_le), pt1_fi_le, (0, 128, 0), 2)
                    valid_left_lane = True
                else:
                    valid_left_lane = False

        # Draw center line
        cv2.line(size_im, (320, 480), (320, 360), (0, 228, 255), 1)

        # Only proceed with lane highlighting and steering if both lanes are valid
        if valid_right_lane and valid_left_lane:
            try:
                # Highlight the possible lane area
                FCP_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8) + 0
                FCP = np.array([pt2_avg_le, pt1_fi_le, pt2_fi_ri, pt1_avg_ri])
                cv2.fillConvexPoly(FCP_img, FCP, color=(255, 242, 213))
                size_im = cv2.addWeighted(size_im, self.lane_fill_alpha, FCP_img, 1 - self.lane_fill_alpha, 0)

                # Calculate the lane center and steering direction
                lane_center_y_ri = 360
                lane_center_y_le = 360
                
                # Avoid NaN values in calculations
                if a_avg_ri != 0 and a_avg_le != 0:
                    lane_center_x_ri = int((lane_center_y_ri - b_avg_ri) / a_avg_ri)
                    lane_center_x_le = int((lane_center_y_le - b_avg_le) / a_avg_le)
                    
                    # Ensure values are within image bounds
                    lane_center_x_ri = max(0, min(640, lane_center_x_ri))
                    lane_center_x_le = max(0, min(640, lane_center_x_le))
                    
                    lane_center_x = ((lane_center_x_ri - lane_center_x_le) // 2) + lane_center_x_le
                    
                    # Store lane center for smoothing
                    self.lane_center_history.append(lane_center_x)
                    
                    # Calculate smoothed lane center
                    smoothed_center = int(sum(self.lane_center_history) / len(self.lane_center_history))
                    self.last_lane_center = smoothed_center

                    # Draw the lane center lines
                    cv2.line(size_im, (lane_center_x_le, lane_center_y_le - 10), (lane_center_x_le, lane_center_y_le + 10), (0, 228, 255), 1)
                    cv2.line(size_im, (lane_center_x_ri, lane_center_y_ri - 10), (lane_center_x_ri, lane_center_y_ri + 10), (0, 228, 255), 1)
                    cv2.line(size_im, (smoothed_center, lane_center_y_ri - 10), (smoothed_center, lane_center_y_le + 10), (0, 228, 255), 1)

                    # Display the steering direction
                    text_left = 'Turn Left'
                    text_right = 'Turn Right'
                    text_center = 'Center'
                    org = (320, 440)
                    font = cv2.FONT_HERSHEY_SIMPLEX

                    if 0 < smoothed_center <= 318:
                        cv2.putText(size_im, text_left, org, font, 0.7, (0, 0, 255), 2)
                    elif 318 < smoothed_center < 322:
                        cv2.putText(size_im, text_center, org, font, 0.7, (0, 0, 255), 2)
                    elif smoothed_center >= 322:
                        cv2.putText(size_im, text_right, org, font, 0.7, (0, 0, 255), 2)
            except Exception as e:
                self.get_logger().warn(f'Error in lane calculation: {str(e)}')
        
        # Add some text to indicate lane detection status
        status_text = "Lanes: "
        if valid_left_lane and valid_right_lane:
            status_text += "Left & Right"
        elif valid_left_lane:
            status_text += "Left only"
        elif valid_right_lane:
            status_text += "Right only"
        else:
            status_text += "None detected"
            
        cv2.putText(size_im, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return size_im, valid_left_lane, valid_right_lane

    def timer_callback(self):
        try:
            # Check if we're connected
            if not self.connected or not self.socket:
                self.get_logger().warn('Not connected to camera server. Skipping frame processing.')
                return
                
            # Receive image size
            try:
                size_data = self.socket.recv(4)
                if not size_data:
                    self.get_logger().warn('Connection lost (empty data). Will try to reconnect.')
                    self.connected = False
                    return
            except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().warn(f'Connection error: {e}. Will try to reconnect.')
                self.connected = False
                return
                
            image_size = struct.unpack('!I', size_data)[0]
            
            # Expected size check
            expected_size = self.IMAGE_WIDTH * self.IMAGE_HEIGHT * self.CHANNELS
            if image_size != expected_size:
                self.get_logger().warn(f'Received image size {image_size} differs from expected {expected_size}')
            
            # Receive image data
            image_data = b''
            try:
                while len(image_data) < image_size:
                    chunk = self.socket.recv(min(4096, image_size - len(image_data)))
                    if not chunk:
                        self.get_logger().warn('Connection lost while receiving image data. Will try to reconnect.')
                        self.connected = False
                        return
                    image_data += chunk
            except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
                self.get_logger().warn(f'Connection error while receiving image data: {e}. Will try to reconnect.')
                self.connected = False
                return
            
            # Convert to numpy array and reshape
            image_array = np.frombuffer(image_data, dtype=np.uint8)
            try:
                image = image_array.reshape((self.IMAGE_HEIGHT, self.IMAGE_WIDTH, self.CHANNELS))
            except ValueError as e:
                self.get_logger().error(f'Failed to reshape image: {str(e)}')
                return
            
            # Create timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Convert to ROS Image message and publish raw camera feed
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='rgba8')
            ros_image.header.stamp = timestamp
            ros_image.header.frame_id = 'camera_link'
            self.image_publisher.publish(ros_image)
            
            # Process lanes and get processed image
            processed_image, left_lane_detected, right_lane_detected = self.detect_lanes(image)
            
            # Convert processed image to ROS Image message and publish
            processed_ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding='rgb8')
            processed_ros_image.header.stamp = timestamp
            processed_ros_image.header.frame_id = 'camera_link'
            self.processed_image_publisher.publish(processed_ros_image)
            
            # Publish camera transform
            self.publish_camera_tf(timestamp)
            
            # Log lane detection status at a lower frequency
            if (self.get_clock().now().nanoseconds // 1000000000) % 5 == 0:  # Every 5 seconds
                lane_status = []
                if left_lane_detected:
                    lane_status.append("Left")
                if right_lane_detected:
                    lane_status.append("Right")
                
                if lane_status:
                    self.get_logger().info(f"Detected lanes: {' & '.join(lane_status)}")
                else:
                    self.get_logger().info("No lanes detected")
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def __del__(self):
        if hasattr(self, 'socket') and self.socket:
            try:
                self.socket.close()
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 