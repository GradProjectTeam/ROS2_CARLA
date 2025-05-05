#!/usr/bin/env python3

import rclpy
import numpy as np
from sensor_fusion.hybrid_astar_planner import HybridAStarPlanner

def main(args=None):
    """
    Test script demonstrating how to use the map_data format provided in the query.
    This script shows how to integrate the exact map_data format as shown in the original query.
    """
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create hybrid A* planner instance
    planner = HybridAStarPlanner()
    
    print("Hybrid A* Planner initialized.")
    
    # Create a simple cost map - in a real application, this would come 
    # from your sensor fusion pipeline or other mapping system
    cost_map = np.zeros((500, 500), dtype=np.int8)
    
    # Add some sample obstacles
    # Create a simple corridor with an opening
    cost_map[100:400, 200] = 80  # Vertical wall
    cost_map[100:400, 300] = 80  # Vertical wall
    cost_map[100, 200:300] = 80  # Horizontal wall (top)
    cost_map[250:300, 200:300] = 0  # Opening in the middle
    cost_map[400, 200:300] = 80  # Horizontal wall (bottom)
    
    # Create the map_data dictionary exactly as in the query
    map_data = {
        'resolution': 0.5,  # meters per grid cell
        'width': 500,
        'height': 500,
        'origin': (0.0, 0.0),  # [x, y]
        'data': np.array(cost_map).flatten()
    }
    
    # Print information about our cost map
    print("\nCost Map Information:")
    print(f"Dimensions: {map_data['width']} x {map_data['height']} cells")
    print(f"Resolution: {map_data['resolution']} meters per cell")
    print(f"Physical size: {map_data['width'] * map_data['resolution']} x "
          f"{map_data['height'] * map_data['resolution']} meters")
    print(f"Origin: {map_data['origin']}")
    
    # Update the planner with our map
    print("\nUpdating the planner with the map data...")
    success = planner.update_map_from_dict(map_data)
    
    if success:
        print("Map data successfully updated!")
    else:
        print("Failed to update map data!")
    
    # Show the planner configuration which will include the map info
    print("\nCurrent planner configuration:")
    planner.config()
    
    # Test obstacle detection with the new map
    print("\nTesting obstacle detection...")
    
    # Test points (free space, occupied, out of bounds)
    test_points = [
        (50, 50),    # Free space
        (250, 200),  # Wall
        (270, 250),  # Middle of opening
        (600, 600)   # Out of bounds
    ]
    
    for x, y in test_points:
        # Convert to world coordinates
        world_x = x * map_data['resolution'] + map_data['origin'][0]
        world_y = y * map_data['resolution'] + map_data['origin'][1]
        
        is_obs = planner.is_obstacle(world_x, world_y)
        print(f"Cell ({x}, {y}) / World ({world_x}, {world_y}): "
              f"{'Obstacle/Invalid' if is_obs else 'Free space'}")
    
    # Clean up
    planner.destroy_node()
    rclpy.shutdown()
    
    print("\nTest completed!")

if __name__ == '__main__':
    main() 