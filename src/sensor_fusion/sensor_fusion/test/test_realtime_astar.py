#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import time

from sensor_fusion.hybrid_astar_planner import HybridAStarPlanner

class RealtimeAStarTester(Node):
    """
    Test node that demonstrates integration between realtime_mapper and Hybrid A* planner.
    This node subscribes to the realtime map and uses it for path planning.
    """
    def __init__(self):
        super().__init__('realtime_astar_tester')
        
        # Create A* planner instance
        self.planner = HybridAStarPlanner()
        
        # Subscribe to the realtime map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/realtime_map',
            self.map_callback,
            10
        )
        
        # Subscribe to the path produced by the planner
        self.path_sub = self.create_subscription(
            Path,
            '/hybrid_astar_path',
            self.path_callback,
            10
        )
        
        # Variables to track state
        self.received_map = False
        self.received_path = False
        self.path_data = None
        
        self.get_logger().info('Realtime A* Tester initialized. Waiting for map data...')
    
    def map_callback(self, msg):
        """Process incoming map from realtime_mapper"""
        if not self.received_map:
            self.get_logger().info(f"Received first map: {msg.info.width}x{msg.info.height} cells, resolution: {msg.info.resolution}m")
            self.received_map = True
        else:
            self.get_logger().debug("Received map update")
        
        # Convert the OccupancyGrid to our map_data format
        map_data = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin': (msg.info.origin.position.x, msg.info.origin.position.y),
            'data': np.array(msg.data).flatten()
        }
        
        # Update the planner with the map
        self.planner.update_map_from_dict(map_data)
        
        # Show map details
        free_cells = np.sum(np.array(msg.data) < self.planner.obstacle_threshold)
        obstacle_cells = np.sum(np.array(msg.data) >= self.planner.obstacle_threshold)
        unknown_cells = np.sum(np.array(msg.data) == -1)
        
        self.get_logger().info(f"Map statistics: {free_cells} free cells, {obstacle_cells} obstacles, {unknown_cells} unknown")
        
        # After receiving a map, set start and goal poses and plan
        self.set_poses_and_plan()
    
    def path_callback(self, msg):
        """Handle path published by the planner"""
        if not self.received_path:
            self.get_logger().info(f"Received first path with {len(msg.poses)} points")
            self.received_path = True
        else:
            self.get_logger().debug(f"Received path update with {len(msg.poses)} points")
        
        # Store the path
        self.path_data = msg
        
        # Calculate path length
        path_length = 0.0
        for i in range(1, len(msg.poses)):
            p1 = msg.poses[i-1].pose.position
            p2 = msg.poses[i].pose.position
            path_length += np.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
        
        self.get_logger().info(f"Path length: {path_length:.2f}m with {len(msg.poses)} waypoints")
    
    def set_poses_and_plan(self):
        """Set start and goal poses for the planner"""
        # Create start pose (use center-left of the map as starting point)
        start_x = self.planner.cost_map['origin'][0] + self.planner.cost_map['resolution'] * 50
        start_y = self.planner.cost_map['origin'][1] + self.planner.cost_map['height'] * self.planner.cost_map['resolution'] / 2
        
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = start_x
        start_pose.pose.position.y = start_y
        start_pose.pose.orientation.w = 1.0
        
        # Create goal pose (use center-right of the map as goal point)
        goal_x = self.planner.cost_map['origin'][0] + self.planner.cost_map['width'] * self.planner.cost_map['resolution'] - 50
        goal_y = self.planner.cost_map['origin'][1] + self.planner.cost_map['height'] * self.planner.cost_map['resolution'] / 2
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map" 
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.w = 1.0
        
        # Set the poses in the planner
        self.planner.current_pose = start_pose
        self.planner.goal_pose = goal_pose
        
        self.get_logger().info(f"Set start pose: ({start_x}, {start_y})")
        self.get_logger().info(f"Set goal pose: ({goal_x}, {goal_y})")
        
        # Clear any existing path
        self.planner.latest_path = None
        
        # Force planning
        start_node = self.planner.get_current_state()
        goal_node = self.planner.get_goal_state()
        
        if start_node and goal_node:
            self.get_logger().info("Planning path...")
            path = self.planner.plan(start_node, goal_node)
            
            if path:
                path_msg = self.planner.path_to_msg(path)
                if path_msg:
                    self.planner.latest_path = path_msg
                    self.planner.path_pub.publish(path_msg)
                    self.get_logger().info(f"Successfully planned and published path with {len(path_msg.poses)} points")
                else:
                    self.get_logger().error("Failed to convert path to message")
            else:
                self.get_logger().error("Failed to find a path")
        else:
            self.get_logger().error("Invalid start or goal state")

def main(args=None):
    rclpy.init(args=args)
    
    tester = RealtimeAStarTester()
    
    print("Waiting for realtime map and planning paths...")
    print("Press Ctrl+C to exit.")
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 