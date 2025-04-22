# LiDAR Cube Visualizer for CARLA Sensors

This documentation covers the LiDAR Cube Visualizer system, which provides advanced 3D visualization of LiDAR data with clustered point clouds represented as cubes, along with a 2D costmap for navigation.

## Overview

The LiDAR Cube Visualizer consists of two main components:

1. **lidar_listener_clusters_2** - Receives LiDAR point cloud data from a TCP server, processes it into clusters, and visualizes the data with 3D cubes in RViz.
2. **lidar_costmap_creator** - Converts the LiDAR point cloud data into a 2D costmap (occupancy grid) for navigation.

These components work together to provide both 3D visualization of LiDAR data and a 2D representation suitable for robotic navigation.

## System Architecture

```
┌─────────────────────┐      ┌─────────────────────┐      ┌─────────────────┐
│ LiDAR TCP Server    │      │lidar_listener_       │      │      RViz       │
│ (Carla or External) │──────▶   clusters_2        │──────▶  Visualization  │
└─────────────────────┘      └─────────────────────┘      └─────────────────┘
                                      │
                                      │ /lidar/points
                                      ▼
                             ┌─────────────────────┐
                             │ lidar_costmap_      │
                             │     creator        │
                             └─────────────────────┘
                                      │
                                      │ /lidar_costmap
                                      ▼
                             ┌─────────────────────┐
                             │      RViz           │
                             │  Costmap Display    │
                             └─────────────────────┘
```

## Components

### 1. lidar_listener_clusters_2

This node receives LiDAR data from a TCP server, processes the data into clusters, and publishes visualizations.

#### Features:
- Connects to a TCP server to receive point cloud data
- Processes points into clusters
- Calculates cluster statistics (center, dimensions, orientation)
- Visualizes clusters using 3D cubes with adjustable transparency
- Optional convex hull visualization around clusters
- Color-coding of different clusters for easy identification

#### Published Topics:
- `/lidar/points` (sensor_msgs/PointCloud2): Raw point cloud data
- `/lidar/markers` (visualization_msgs/MarkerArray): Cluster center markers
- `/lidar/hulls` (visualization_msgs/MarkerArray): Convex hull visualization
- `/lidar/cubes` (visualization_msgs/MarkerArray): 3D cube visualization of clusters

#### Parameters:
- `tcp_ip` (string, default: "127.0.0.1"): IP address of the TCP server
- `tcp_port` (int, default: 12350): Port number of the TCP server
- `point_size` (float, default: 1.5): Size of individual point markers
- `center_size` (float, default: 3.0): Size of cluster center markers
- `use_convex_hull` (bool, default: true): Whether to display 2D convex hull around clusters
- `use_point_markers` (bool, default: false): Whether to display individual point markers
- `use_cluster_stats` (bool, default: true): Whether to display cluster statistics
- `verbose_logging` (bool, default: false): Enable verbose logging
- `cube_alpha` (float, default: 0.3): Transparency for cube visualization (0.0-1.0)

### 2. lidar_costmap_creator

This node creates a 2D occupancy grid (costmap) from LiDAR point cloud data, suitable for navigation planning.

#### Features:
- Subscribes to point cloud data from lidar_listener_clusters_2
- Filters out ground plane and noise
- Creates an occupancy grid with obstacle detection
- Performs raycasting to identify free space vs. obstacles
- Optimized for real-time performance with adjustable parameters
- Provides a costmap suitable for navigation algorithms

#### Published Topics:
- `/lidar_costmap` (nav_msgs/OccupancyGrid): 2D occupancy grid for navigation
- `/lidar_costmap_debug` (nav_msgs/OccupancyGrid): Debug visualization of the costmap

#### Parameters:
- `map_resolution` (float, default: 0.1): Resolution of the costmap (meters per cell)
- `map_width` (int, default: 1000): Width of the costmap in cells
- `map_height` (int, default: 1000): Height of the costmap in cells
- `map_width_meters` (float, default: 100.0): Width of the costmap in meters
- `map_height_meters` (float, default: 100.0): Height of the costmap in meters
- `map_origin_x` (float, default: -50.0): X-coordinate of the map origin
- `map_origin_y` (float, default: -50.0): Y-coordinate of the map origin
- `vehicle_x_offset` (float, default: 0.0): X offset for vehicle position
- `vehicle_y_offset` (float, default: 0.0): Y offset for vehicle position
- `publish_rate` (float, default: 5.0): Rate to publish costmap (Hz)
- `process_rate` (float, default: 10.0): Rate to process costmap data (Hz)
- `raycast_skip` (int, default: 3): Only raytrace every Nth point for better performance
- `max_points_to_process` (int, default: 5000): Maximum number of points to process per update
- `detection_radius` (float, default: 35.0): Detection radius for the costmap (meters)
- `obstacle_inflation` (float, default: 0.3): Inflation radius for obstacles (meters)
- `max_data_age` (float, default: 2.0): Maximum age of data to consider valid (seconds)
- `min_height` (float, default: -3.0): Minimum height of points to consider (meters)
- `max_height` (float, default: 4.0): Maximum height of points to consider (meters)
- `ground_threshold` (float, default: 0.15): Height threshold for ground plane detection
- `center_priority_radius` (float, default: 10.0): Radius around vehicle to prioritize in costmap
- `vehicle_center_weight` (float, default: 1.5): Weight for centering vehicle (0-1)

## Launch File: lidar_cube_visualizer.launch.py

The launch file combines all components and provides customization through launch arguments.

### Nodes Launched:
1. `lidar_listener_clusters_2` (named as "lidar_cube_visualizer")
2. `lidar_costmap_creator` 
3. `rviz2` with a prepared configuration for visualization
4. `static_transform_publisher` to provide a TF frame for the map

### Launch Arguments:
- TCP connection parameters: `tcp_ip`, `tcp_port`
- Visualization parameters: `point_size`, `center_size`, `use_point_markers`, `use_convex_hull`, `use_cluster_stats`, `verbose_logging`, `cube_alpha`
- Costmap parameters: `map_resolution`, `map_width_meters`, `map_height_meters`, `detection_radius`, `obstacle_inflation`
- Performance parameters: `publish_rate`, `process_rate`, `raycast_skip`, `max_points_to_process`

## Usage

### Prerequisites
- ROS2 installed
- CARLA simulator or other LiDAR TCP server

### Running the System

1. Start your LiDAR data source (CARLA simulator or other TCP server)

2. Launch the LiDAR Cube Visualizer:
   ```bash
   ros2 launch carla_sensors lidar_cube_visualizer.launch.py
   ```

3. To customize parameters, use launch arguments:
   ```bash
   ros2 launch carla_sensors lidar_cube_visualizer.launch.py tcp_ip:=192.168.1.100 tcp_port:=12345 cube_alpha:=0.5
   ```

## Performance Considerations

- Setting `use_point_markers` to false significantly improves performance
- Adjust `raycast_skip` and `max_points_to_process` to balance between accuracy and performance
- The `publish_rate` and `process_rate` parameters control the CPU usage
- For large areas, consider adjusting the map resolution to reduce memory usage

## Troubleshooting

### Common Issues:

1. **TCP Connection Failed**
   - Ensure your LiDAR server is running
   - Check IP address and port configuration
   - Verify network connectivity

2. **High CPU Usage**
   - Reduce `process_rate` and `publish_rate`
   - Increase `raycast_skip`
   - Set `use_point_markers` to false
   - Reduce the map dimensions

3. **No Visualization in RViz**
   - Ensure correct frame IDs are set (default is "map")
   - Check that all topics are properly visualized in RViz
   - Verify that the static transform publisher is running

4. **Inaccurate Costmap**
   - Adjust `ground_threshold` for better ground plane detection
   - Modify `obstacle_inflation` to change obstacle size
   - Tune `min_height` and `max_height` to filter unwanted points

## Advanced Usage

### Custom TCP Data Format

The TCP server is expected to send data in the following format:

1. Number of clusters (uint32)
2. For each cluster:
   - Number of points (uint32)
   - For each point:
     - x, y, z coordinates (3 float32)

### Integration with Navigation Stack

The costmap can be used with the ROS2 Navigation Stack (Nav2):

1. Add the `/lidar_costmap` topic as an observation source in your navigation configuration
2. Configure the costmap layers to use this data
3. Adjust the inflation and resolution parameters to match your robot's needs

## Development and Extension

To extend this system:

1. For new visualization types, modify the lidar_listener_clusters_2.py file
2. For additional costmap features, extend lidar_costmap_creator.py
3. To add new parameters, update both the node and the launch file 