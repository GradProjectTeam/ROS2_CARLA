# LIDAR Processing and Visualization System Documentation

## Overview

This document provides a comprehensive guide to the ROS2-based LIDAR processing and visualization system developed for Carla simulation. The system consists of several nodes that work together to process LIDAR data, visualize point clouds, and generate 2D costmaps for potential navigation applications.

## System Components

The system consists of the following main components:

1. **LIDAR Listener Node (`lidar_listener_clusters_2`)**: Receives raw LIDAR data from Carla simulation via TCP, processes it into clusters, and publishes point clouds and visualization markers.

2. **Costmap Creator Node (`lidar_costmap_creator`)**: Converts LIDAR data into a 2D occupancy grid (costmap) for potential navigation.

3. **RViz Visualization**: Provides 3D visualization of LIDAR data, including point clouds, cluster markers, and costmap.

4. **Launch File**: Orchestrates the startup of all components with configurable parameters.

## Installation Requirements

The system requires:
- ROS2 (tested on Humble/Foxy)
- Carla Simulator (tested on 0.9.8)
- Python 3.x
- NumPy, SciPy
- Additional ROS2 dependencies: `tf2_ros`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`

## Nodes Description

### 1. LIDAR Listener Node (`lidar_listener_clusters_2`)

This node:
- Connects to Carla simulation via TCP socket
- Receives LIDAR point clouds
- Processes points into clusters
- Visualizes clusters with different markers
- Publishes data for downstream processing

**Key Topics Published:**
- `/lidar/points` (PointCloud2): Raw LIDAR point cloud data
- `/lidar/markers` (MarkerArray): Visualization markers for clusters
- `/lidar/hulls` (MarkerArray): Convex hull visualization for clusters
- `/lidar/cubes` (MarkerArray): 3D cube visualization for clusters

**Key Parameters:**
- `tcp_ip`: TCP server IP address (Carla)
- `tcp_port`: TCP port number
- `point_size`: Size of individual point markers
- `center_size`: Size of cluster center markers
- `use_point_markers`: Enable/disable individual point markers
- `use_convex_hull`: Enable/disable convex hull display
- `use_cluster_stats`: Enable/disable cluster statistics display
- `cube_alpha`: Transparency for 3D cube visualization

### 2. Costmap Creator Node (`lidar_costmap_creator`)

This node:
- Subscribes to LIDAR point cloud data
- Processes points to create a 2D occupancy grid (costmap)
- Uses optimized algorithms for real-time performance
- Publishes costmap data for navigation

**Key Topics Published:**
- `/lidar_costmap` (OccupancyGrid): Main costmap data
- `/lidar_costmap_debug` (OccupancyGrid): Duplicate for debugging

**Key Topics Subscribed:**
- `/lidar/points` (PointCloud2): Point cloud from the LIDAR listener
- `/lidar/markers` (MarkerArray): Marker data from the LIDAR listener

**Key Parameters:**
- `map_resolution`: Costmap resolution in meters per cell (default: 0.1m)
- `map_width`/`map_height`: Size of the costmap in cells (default: 1000x1000)
- `map_width_meters`/`map_height_meters`: Size in meters (default: 100x100m)
- `publish_rate`: Publishing frequency in Hz (default: 5Hz)
- `process_rate`: Processing frequency in Hz (default: 10Hz)
- `raycast_skip`: Process every Nth obstacle for raytracing (default: 3)
- `max_points_to_process`: Maximum points to process per update (default: 5000)
- `detection_radius`: Maximum range for obstacle detection (default: 35m)
- `vehicle_x_offset`/`vehicle_y_offset`: Offsets to center the vehicle
- `center_priority_radius`: Radius around vehicle to prioritize (default: 10m)

## Commands to Run the System

### 1. Start Carla Simulator

```bash
# Navigate to your Carla directory and run:
./CarlaUE4.sh
```

### 2. Run the Carla Sensor Client

```bash
# Navigate to your Carla Python scripts directory:
cd /home/mostafa/ROS2andCarla/CARLA/Sensors/
# Run the sensor client:
python3 three_sensors_with_pygame_6.py
```

### 3. Build the ROS2 Package

```bash
# Navigate to your ROS2 workspace:
cd /home/mostafa/GP/ROS2
# Build the package:
colcon build --packages-select carla_sensors --symlink-install
# Source the workspace:
source install/setup.bash
```

### 4. Launch the LIDAR Processing System

```bash
# Launch the system with default parameters:
ros2 launch carla_sensors lidar_cube_visualizer.launch.py

# Launch with custom parameters:
ros2 launch carla_sensors lidar_cube_visualizer.launch.py map_resolution:=0.1 publish_rate:=5.0 process_rate:=10.0 raycast_skip:=5 max_points_to_process:=3000
```

## Performance Optimization

The system includes several optimizations for better performance:

1. **Reduced Map Resolution**: Using 0.1m resolution instead of 0.05m reduces memory usage by 75%
2. **Point Cloud Subsampling**: Limiting processed points to a fixed maximum
3. **Raycasting Optimization**: Only raytracing a subset of detected obstacles
4. **Vectorized Operations**: Using NumPy vectorized operations for faster processing
5. **Rate Limiting**: Separate timers for processing and publishing
6. **Binary Dilation**: Faster obstacle inflation using binary image processing

## Common Issues and Solutions

1. **High CPU Usage**:
   - Reduce `process_rate` and `publish_rate`
   - Increase `raycast_skip` to process fewer obstacles
   - Decrease `map_resolution` for a smaller costmap

2. **Delayed Visualization**:
   - Check that all nodes are running correctly
   - Ensure TCP connection to Carla is stable
   - Reduce point cloud density in Carla settings

3. **Missing or Incomplete Costmap**:
   - Check map dimensions and origin settings
   - Verify that the LIDAR node is publishing point clouds
   - Ensure the transform between map and ego_vehicle exists

## Monitoring and Debugging

The system provides several ways to monitor performance:

1. **Performance Statistics**: The costmap node reports processing statistics every 10 seconds:
   ```
   [INFO] [lidar_costmap_creator]: Performance: 9.80 frames/sec, 49054.30 points/sec
   ```

2. **Costmap Statistics**: The node reports costmap composition every few seconds:
   ```
   [INFO] [lidar_costmap_creator]: LiDAR costmap stats: White (free): 78.2%, Black (obstacles): 3.1%, Gray (unknown): 18.7%
   ```

3. **RViz Visualization**: Use RViz to visually inspect:
   - Point clouds on the `/lidar/points` topic
   - Cluster visualizations on the `/lidar/markers`, `/lidar/hulls`, and `/lidar/cubes` topics
   - Costmap on the `/lidar_costmap` topic

## Conclusion

This system provides a complete solution for processing LIDAR data from Carla simulation, visualizing the results, and generating costmaps for navigation. The performance optimizations ensure the system can run in real-time, even on modest hardware.

By adjusting the parameters documented above, you can balance between processing quality and computational efficiency to meet your specific requirements.
