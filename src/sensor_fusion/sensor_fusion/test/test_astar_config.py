#!/usr/bin/env python3

import rclpy
import numpy as np
import time
from sensor_fusion.hybrid_astar_planner import HybridAStarPlanner

def main(args=None):
    """Test script for Hybrid A* Planner configuration and map update"""
    
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create hybrid A* planner instance
    planner = HybridAStarPlanner()
    
    print("Hybrid A* Planner initialized. Waiting for node to set up...")
    time.sleep(2)  # Give the node time to initialize
    
    # Test 1: Call config method directly
    print("\n--- Test 1: Current Configuration ---")
    planner.config()
    
    # Test 2: Update map with new resolution and dimensions
    print("\n--- Test 2: Update Map Data ---")
    new_map_data = {
        'resolution': 0.25,  # Higher resolution (meters per grid cell)
        'width': 1000,
        'height': 800,
        'origin': (-250.0, -200.0),  # [x, y]
        'data': np.zeros(1000 * 800, dtype=np.int8)  # Empty cost map
    }
    
    # Create some random obstacles (20% of cells)
    obstacle_indices = np.random.choice(
        len(new_map_data['data']), 
        size=int(0.01 * len(new_map_data['data'])),  # 1% of cells are obstacles
        replace=False
    )
    new_map_data['data'][obstacle_indices] = 100
    
    # Update the planner with our custom map
    print("Updating planner with custom map...")
    planner.update_map_from_dict(new_map_data)
    
    # Test 3: Change velocity and time step
    print("\n--- Test 3: Update Motion Parameters ---")
    planner.velocity = 2.5  # m/s
    planner.dt = 0.5  # s
    
    # Display updated configuration
    planner.config()
    
    # Test 4: Some simple is_obstacle checks
    print("\n--- Test 4: Test is_obstacle Method ---")
    # Center of the map
    center_x = new_map_data['origin'][0] + (new_map_data['width'] * new_map_data['resolution'] / 2)
    center_y = new_map_data['origin'][1] + (new_map_data['height'] * new_map_data['resolution'] / 2)
    
    # Check a few points
    test_points = [
        (center_x, center_y),
        (new_map_data['origin'][0], new_map_data['origin'][1]),  # Bottom left corner
        (new_map_data['origin'][0] + new_map_data['width'] * new_map_data['resolution'], 
         new_map_data['origin'][1] + new_map_data['height'] * new_map_data['resolution']),  # Top right corner
        (-1000, -1000)  # Out of bounds
    ]
    
    for i, (x, y) in enumerate(test_points):
        is_obs = planner.is_obstacle(x, y)
        print(f"Point {i+1} ({x}, {y}): {'Obstacle' if is_obs else 'Free space'}")
    
    # Clean up
    planner.destroy_node()
    rclpy.shutdown()
    
    print("\nTest completed successfully!")

if __name__ == '__main__':
    main() 