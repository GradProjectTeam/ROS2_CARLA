#!/usr/bin/env python3

import rclpy
import numpy as np
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from sensor_fusion.hybrid_astar_planner import HybridAStarPlanner

def main(args=None):
    """Test script for Hybrid A* Planner with custom map data"""
    
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create hybrid A* planner instance
    planner = HybridAStarPlanner()
    
    print("Hybrid A* Planner initialized. Waiting for node to set up...")
    time.sleep(2)  # Give the node time to initialize
    
    # Create a simple test cost map (500x500 with some obstacles)
    cost_map = np.zeros((500, 500), dtype=np.int8)
    
    # Add some obstacles in the middle (a maze-like pattern)
    # Vertical walls
    cost_map[100:400, 200] = 100
    cost_map[100:300, 300] = 100
    cost_map[200:400, 100] = 100
    
    # Horizontal walls
    cost_map[100, 100:300] = 100
    cost_map[200, 200:400] = 100
    cost_map[300, 100:300] = 100
    cost_map[400, 200:400] = 100
    
    # Create the map data dictionary
    map_data = {
        'resolution': 0.5,  # meters per grid cell
        'width': 500,
        'height': 500,
        'origin': (0.0, 0.0),  # [x, y]
        'data': cost_map.flatten()
    }
    
    # Update the planner with our custom map
    print("Updating planner with custom map...")
    planner.update_map_from_dict(map_data)
    
    # Set start pose (current position)
    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.pose.position.x = 50.0
    start_pose.pose.position.y = 50.0
    start_pose.pose.orientation.w = 1.0  # Unit quaternion
    
    # Set goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = 450.0
    goal_pose.pose.position.y = 450.0
    goal_pose.pose.orientation.w = 1.0  # Unit quaternion
    
    # Manually set the poses in the planner
    planner.current_pose = start_pose
    planner.goal_pose = goal_pose
    
    print(f"Planning from ({start_pose.pose.position.x}, {start_pose.pose.position.y}) "
          f"to ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})")
    
    # Set up callback to capture the path
    path_received = False
    received_path = None
    
    def path_callback(msg):
        nonlocal path_received, received_path
        path_received = True
        received_path = msg
        print(f"Received path with {len(msg.poses)} points")
    
    # Subscribe to the path topic
    path_sub = planner.create_subscription(
        Path,
        '/hybrid_astar_path',
        path_callback,
        10
    )
    
    # Manual planning (instead of waiting for timer)
    start_node = planner.get_current_state()
    goal_node = planner.get_goal_state()
    
    if start_node and goal_node:
        print("Planning path...")
        path = planner.plan(start_node, goal_node)
        
        if path:
            path_msg = planner.path_to_msg(path)
            if path_msg:
                planner.latest_path = path_msg
                planner.path_pub.publish(path_msg)
                
                # Calculate and print path information
                path_length = 0
                for i in range(1, len(path_msg.poses)):
                    p1 = path_msg.poses[i-1].pose.position
                    p2 = path_msg.poses[i].pose.position
                    path_length += np.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
                
                print(f"Published path with {len(path_msg.poses)} points, length: {path_length:.2f}m")
        else:
            print("No path found!")
    else:
        print("Invalid start or goal state")
    
    # Spin for a short while to process publications/subscriptions
    timeout = 30.0  # seconds
    start_time = time.time()
    
    print("Waiting for results (max 30 seconds)...")
    while rclpy.ok() and time.time() - start_time < timeout:
        rclpy.spin_once(planner, timeout_sec=0.1)
        if path_received:
            break
    
    # Print results
    if path_received:
        print("Test successful! Path was generated and published.")
        path_length = 0
        for i in range(1, len(received_path.poses)):
            p1 = received_path.poses[i-1].pose.position
            p2 = received_path.poses[i].pose.position
            path_length += np.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
        
        print(f"Path details:")
        print(f"  - Number of points: {len(received_path.poses)}")
        print(f"  - Total length: {path_length:.2f} meters")
        print(f"  - Frame ID: {received_path.header.frame_id}")
    else:
        print("Test failed: No path was received within the timeout period.")
    
    # Clean up
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 