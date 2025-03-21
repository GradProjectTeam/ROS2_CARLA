import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros
import numpy as np
import math
import threading

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        
        # Create QoS profile for reliable path planning
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribe to the local costmap for path planning
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap',
            self.costmap_callback,
            qos_profile
        )
        
        # Create a publisher for the planned path
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )
        
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Create an action client for the NavigateToPose action
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Store the latest costmap
        self.costmap = None
        self.costmap_lock = threading.Lock()
        
        # Current navigation goal (if any)
        self.current_goal = None
        
        # Flag to indicate if navigation is in progress
        self.is_navigating = False
        
        # Create a timer for checking navigation status
        self.status_timer = self.create_timer(1.0, self.check_navigation_status)
        
        self.get_logger().info('Navigator Node initialized')

    def costmap_callback(self, msg):
        """
        Process the costmap for path planning.
        
        Args:
            msg (OccupancyGrid): The local costmap
        """
        with self.costmap_lock:
            self.costmap = msg
            self.get_logger().debug('Received costmap update')

    def send_goal(self, x, y, theta):
        """
        Send a navigation goal to move to the specified pose.
        
        Args:
            x (float): Target X coordinate
            y (float): Target Y coordinate
            theta (float): Target orientation in radians
        """
        self.get_logger().info(f'Sending navigation goal: x={x}, y={y}, theta={theta}')
        
        # Create the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Create the action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Wait for the action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False
        
        # Store the current goal
        self.current_goal = goal_pose
        
        # Send the goal
        self.is_navigating = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """
        Callback for handling the goal response.
        
        Args:
            future: The future object containing the goal response
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server')
            self.is_navigating = False
            return
            
        self.get_logger().info('Goal accepted by the action server')
        
        # Get the result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback for handling the action result.
        
        Args:
            future: The future object containing the result
        """
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # Succeeded
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().error(f'Goal failed with status code: {status}')
            
        self.is_navigating = False
        self.current_goal = None

    def feedback_callback(self, feedback_msg):
        """
        Callback for handling navigation feedback.
        
        Args:
            feedback_msg: The feedback message from the action
        """
        feedback = feedback_msg.feedback
        current_pose = feedback.current_pose
        
        # Calculate distance to goal
        if self.current_goal is not None:
            dx = current_pose.pose.position.x - self.current_goal.pose.position.x
            dy = current_pose.pose.position.y - self.current_goal.pose.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().debug(f'Distance to goal: {distance:.2f} meters')

    def check_navigation_status(self):
        """
        Periodically check the navigation status and update the path visualization.
        """
        if not self.is_navigating:
            return
            
        # In a real implementation, you would:
        # 1. Check if the path needs to be replanned
        # 2. Visualize the current path
        # 3. Handle any navigation problems
        
        self.get_logger().debug('Navigation in progress...')

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        """
        Plan a path from the start position to the goal position using the costmap.
        This is a simplified implementation. In a real system, you'd use A* or similar.
        
        Args:
            start_x (float): Start X coordinate
            start_y (float): Start Y coordinate
            goal_x (float): Goal X coordinate
            goal_y (float): Goal Y coordinate
            
        Returns:
            Path: The planned path
        """
        with self.costmap_lock:
            if self.costmap is None:
                self.get_logger().error('No costmap available for path planning')
                return None
                
            # Create a path message
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            
            # In a real implementation, you would:
            # 1. Convert start and goal from world coordinates to grid cells
            # 2. Implement A* or similar algorithm to find a path through the costmap
            # 3. Convert the path back to world coordinates
            
            # For this simple example, we'll just create a straight line path
            # This is not obstacle-aware and is just for demonstration
            steps = 20
            for i in range(steps + 1):
                t = i / steps
                x = start_x + t * (goal_x - start_x)
                y = start_y + t * (goal_y - start_y)
                
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                
                # Calculate orientation based on path direction
                if i < steps:
                    angle = math.atan2(goal_y - start_y, goal_x - start_x)
                    pose.pose.orientation.z = math.sin(angle / 2.0)
                    pose.pose.orientation.w = math.cos(angle / 2.0)
                else:
                    # At the goal, use the specified goal orientation
                    pose.pose.orientation.z = 0.0
                    pose.pose.orientation.w = 1.0
                
                path.poses.append(pose)
            
            # Publish the path for visualization
            self.path_pub.publish(path)
            
            return path

    def stop_navigation(self):
        """
        Stop the current navigation task.
        """
        if not self.is_navigating:
            self.get_logger().info('No navigation in progress')
            return
            
        # Send a cancel request to the action server
        # This would be implemented with the action client's cancel_goal method
        
        # Also send a zero velocity command to stop the robot
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.is_navigating = False
        self.current_goal = None
        self.get_logger().info('Navigation stopped')

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 