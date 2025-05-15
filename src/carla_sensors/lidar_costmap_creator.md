# LiDAR Costmap Creator Documentation

## Overview

The `lidar_costmap_creator` is a ROS2 node that processes LiDAR point cloud data and converts it into a cost map suitable for robotic navigation. It transforms 3D point cloud data from clusters into a 2D occupancy grid that represents obstacles and free space in the environment. This costmap can be integrated with navigation stacks such as Nav2 for path planning and obstacle avoidance.

## Key Features

- Processes clustered LiDAR point cloud data
- Filters out ground plane and noise
- Generates a 2D costmap with configurable resolution
- Performs obstacle inflation for safer navigation
- Publishes standard OccupancyGrid messages
- Configurable through ROS2 parameters

## Architecture

The node operates as follows:

1. **Subscription to LiDAR Data**: Subscribes to point cloud messages published by the `lidar_listener_clusters` node on the `/lidar/points` topic.

2. **Point Cloud Processing**:
   - Receives PointCloud2 messages
   - Extracts point coordinates
   - Filters out ground and noise

3. **Costmap Generation**:
   - Maps points to cells in a 2D grid
   - Assigns cost values based on point density
   - Inflates obstacles to account for robot size

4. **Publishing**:
   - Publishes the costmap as a standard ROS2 OccupancyGrid message on `/lidar_costmap`
   - Provides a debug visualization topic at `/lidar_costmap_debug`

## Messages

### Subscribed Topics
- `/lidar/points` ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)): 3D point cloud data from LiDAR clusters
- `/lidar/markers` ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)): Additional metadata about clusters

### Published Topics
- `/lidar_costmap` ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)): Main costmap for navigation
- `/lidar_costmap_debug` ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)): Visualization-friendly costmap

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map_resolution` | float | 0.1 | Resolution of the costmap in meters per cell |
| `map_width` | int | 1000 | Width of the costmap in cells |
| `map_height` | int | 1000 | Height of the costmap in cells |
| `map_origin_x` | float | -50.0 | X coordinate of the map origin in meters |
| `map_origin_y` | float | -50.0 | Y coordinate of the map origin in meters |
| `publish_rate` | float | 5.0 | How often to publish the costmap (Hz) |
| `ground_threshold` | float | 0.1 | Height threshold to consider a point as ground (meters) |
| `max_points_per_cell` | int | 5 | Maximum number of points per cell for confidence calculation |
| `min_height` | float | -5.0 | Minimum height to consider (meters) |
| `max_height` | float | 5.0 | Maximum height to consider (meters) |
| `obstacle_inflation` | float | 0.4 | How much to inflate obstacles (meters) |
| `max_data_age` | float | 60.0 | Maximum age of data to use (seconds) |

## Costmap Representation

The costmap uses the following cost values:
- **0**: Free space (no obstacles)
- **1-100**: Occupied space with varying confidence
  - **70-100**: Direct obstacle detections (higher values indicate higher confidence)
  - **1-69**: Inflated obstacle areas (cost decreases with distance from obstacles)

## Implementation Details

### Point Filtering

The `filter_ground_and_noise` method filters LiDAR points based on the following criteria:
- Height range (`min_height` to `max_height`)
- Ground threshold (points exactly at z=0 are filtered)

```python
def filter_ground_and_noise(self, points):
    """Filter out ground plane and noise points"""
    filtered_points = []
    
    for point in points:
        if len(point) >= 3:
            x, y, z = point[0], point[1], point[2]
            
            # Accept points within height range
            if not (self.min_height <= z <= self.max_height):
                continue
                
            # Filter exact ground points
            if abs(z) < 0.01:  # Almost exactly 0
                continue
                
            # Point passed all filters
            filtered_points.append((x, y, z))
            
    return filtered_points
```

### Costmap Update Process

The `update_costmap` method handles the main processing pipeline:

1. **Point Processing**:
   ```python
   # Filter points
   filtered_points = self.filter_ground_and_noise(self.lidar_points)
   
   # Fallback if filtering removed everything
   if not filtered_points:
       filtered_points = [(p[0], p[1], p[2]) for p in self.lidar_points if len(p) >= 3]
   ```

2. **Cell Mapping**:
   ```python
   # Count points per cell
   for x, y, z in filtered_points:
       grid_x = int((x - self.map_origin_x) / self.map_resolution)
       grid_y = int((y - self.map_origin_y) / self.map_resolution)
       
       # Track cell occupation
       cell_key = (grid_y, grid_x)
       if cell_key in cell_point_count:
           cell_point_count[cell_key] += 1
       else:
           cell_point_count[cell_key] = 1
   ```

3. **Cost Assignment**:
   ```python
   # Assign cost values based on point density
   for (grid_y, grid_x), count in cell_point_count.items():
       # Calculate confidence level
       confidence = min(count / max(1, self.max_points_per_cell * 0.5), 1.0)
       
       # Assign cost (70-100)
       cost = int(70 + 30 * confidence)
       self.lidar_costmap[grid_y, grid_x] = cost
   ```

4. **Obstacle Inflation**:
   ```python
   # For each obstacle cell, inflate it
   for y, x in zip(obstacle_cells[0], obstacle_cells[1]):
       # Apply inflation to nearby cells with distance-based decay
       for dy, dx in neighboring cells:
           distance = math.sqrt(dx**2 + dy**2)
           if distance <= inflation_radius:
               decay_factor = max(0, 1.0 - (distance / inflation_radius) ** 0.7)
               inflated_cost = int(original_cost * decay_factor)
               if inflated_cost > self.lidar_costmap[ny, nx]:
                   self.lidar_costmap[ny, nx] = inflated_cost
   ```

## Usage

### Launch File

The node can be launched using the `lidar_test.launch.py` file:

```bash
ros2 launch carla_sensors lidar_test.launch.py
```

This launch file starts:
1. The LiDAR listener node to receive TCP data
2. The LiDAR costmap creator node
3. RViz for visualization

### Configuration Options

You can customize parameters via the command line:

```bash
ros2 launch carla_sensors lidar_test.launch.py map_resolution:=0.05 map_width:=2000 map_height:=2000
```

### Visualization

The costmap can be visualized in RViz by:
1. Adding an "OccupancyGrid" display
2. Setting the topic to "/lidar_costmap"
3. Setting the color scheme to your preference

## Enhanced Costmap Version

For more advanced functionality, the `lidar_costmap_enhanced` node is available, which implements:
- A three-state system: unknown (-1), free (0), and occupied (1-100)
- Ray tracing to identify free space between the sensor and obstacles
- Better visualization of unknown vs. free space

Launch the enhanced version with:

```bash
ros2 launch carla_sensors lidar_enhanced_test.launch.py
```

## Integration with Nav2

To use this costmap with Nav2, add the following to your Nav2 parameters YAML:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "lidar_layer"]
      
      lidar_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/lidar_costmap"
        track_unknown_space: true
        use_maximum: true
```

## Dependencies

- ROS2 (tested on Humble)
- Python 3.8+
- NumPy
- sensor_msgs
- nav_msgs
- visualization_msgs
- sensor_msgs_py

## Troubleshooting

### No Points Detected
- Check if the LiDAR TCP server is running (`ps aux | grep Lidar_Processing`)
- Verify the TCP connection using `netstat -tuln | grep 12350`
- Increase verbosity with the `verbose_logging` parameter

### "LiDAR data too old" Warning
- Check if continuous data is being received from the LiDAR source
- Increase the `max_data_age` parameter temporarily for debugging
- Check timestamp handling in the `lidar_listener_clusters` node 

### Empty or Sparse Costmap
- Examine filter settings - ground_threshold may be too aggressive
- Check map resolution - too high may cause sparse representation
- Verify the coordinate transformation is correct

### RViz Display Issues
- Ensure the frame_id matches between RViz and the costmap message
- Try different coloring schemes in RViz
- Verify that the costmap topic is being published with `ros2 topic echo /lidar_costmap`

## Contributing

Contributions to improve the LiDAR costmap creator are welcome. Some potential areas for enhancement:

1. Adding dynamic obstacle tracking
2. Implementing more sophisticated ground plane detection
3. Adding temporal filtering for more stable costmaps
4. Optimizing performance for larger maps or higher update rates

---

*This documentation is provided for the `lidar_costmap_creator` and `lidar_costmap_enhanced` nodes in the carla_sensors package.* 