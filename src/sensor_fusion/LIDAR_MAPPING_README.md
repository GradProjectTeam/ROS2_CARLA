# LiDAR Mapping in Sensor Fusion Package

This document provides detailed information about the LiDAR mapping capabilities in the sensor fusion package.

## Overview

The package provides three main approaches to LiDAR-based mapping:

1. **Permanent Mapping**: Creates a persistent, high-precision map that accumulates data over time
2. **Real-time Mapping**: Generates a dynamic map focused on immediate surroundings with temporal decay
3. **IMU-LiDAR Fusion Mapping**: Combines IMU orientation data with LiDAR for improved mapping accuracy

## Mapping Node Comparison

| Feature | Permanent Mapper | Real-time Mapper | IMU-LiDAR Fusion |
|---------|------------------|------------------|------------------|
| **Map Persistence** | Long-term | Short-term (with decay) | Long-term |
| **Update Frequency** | Low (1-2 Hz) | High (10 Hz) | Low-Medium (1-5 Hz) |
| **Memory Usage** | High | Medium | High |
| **Obstacle Representation** | Probabilistic | Binary with decay | Probabilistic |
| **Best Use Case** | Static environment mapping | Dynamic obstacle avoidance | Mapping with rotation |

## LiDAR Data Processing Pipeline

1. **Data Acquisition**: Raw LiDAR point clouds are received via TCP from a LiDAR sensor or simulation
2. **Point Cloud Processing**: Points are filtered, clustered, and processed
3. **Map Integration**: Processed points are integrated into the map using either:
   - Bayesian updates (permanent map)
   - Temporal decay (real-time map)
   - Orientation-corrected updates (IMU-LiDAR fusion)
4. **Map Publication**: The resulting occupancy grid is published for visualization and planning

## Permanent Mapping

The permanent mapper creates a persistent occupancy grid using Bayesian probability updates.

### Key Features
- **Probabilistic Updates**: Uses hit/miss weights to update cell occupancy
- **Map Saving/Loading**: Can save and load maps to/from disk
- **Large Coverage Area**: Typically configured for larger areas (100m x 100m by default)

### Launch
```bash
ros2 launch sensor_fusion permanent_map.launch.py
```

### Key Parameters
- `hit_weight`: High value (0.99) for strong confidence in obstacles
- `miss_weight`: Low value (0.05) for minimal free space clearing
- `map_resolution`: Typically coarser (0.2m) for efficiency with large maps
- `enable_auto_save`: Enable automatic map saving
- `auto_save_interval`: How often to save the map (60s by default)

## Real-time Mapping

The real-time mapper prioritizes immediate surroundings and fast updates over persistence.

### Key Features
- **Temporal Decay**: Old data gradually fades away
- **Vehicle-Centered**: Map automatically stays centered on the vehicle
- **Binary Representation**: High contrast between obstacles and free space
- **Periodic Reset**: Full map reset at configurable intervals

### Launch
```bash
ros2 launch sensor_fusion realtime_map.launch.py
```

### Key Parameters
- `decay_rate`: How quickly old points disappear (0.9 by default)
- `update_threshold`: Threshold for immediate updates (0.001)
- `map_reset_interval`: How often to reset the entire map (20s)
- `map_resolution`: Higher resolution (0.1m) for better detail
- `center_on_vehicle`: Keep map centered on vehicle position

## IMU-LiDAR Fusion Mapping

Combines IMU orientation data with LiDAR mapping for improved accuracy.

### Key Features
- **Orientation Correction**: Uses IMU data to improve mapping accuracy during rotation
- **Yaw Fusion**: Specifically corrects for yaw drift in mapping
- **Configurable Fusion Weight**: Balance between IMU and LiDAR-derived orientation

### Launch
```bash
ros2 launch sensor_fusion imu_lidar_fusion.launch.py
```

### Key Parameters
- `initial_yaw_offset`: Initial yaw offset in radians
- `use_filtered_yaw`: Use filtered yaw for stability
- `yaw_filter_size`: Size of filter buffer for yaw smoothing
- `yaw_weight`: Weight of yaw in the fusion (0-1)

## Common Optimizations

### Performance Optimization
- `raycast_skip`: Only process every Nth point for raytracing
- `max_points_to_process`: Limit points processed per update
- `map_resolution`: Adjust resolution to balance detail vs. performance
- `publish_rate` vs `process_rate`: Separate processing from visualization updates

### Memory Optimization
- Limit map size with appropriate `map_width_meters` and `map_height_meters`
- Use higher `map_resolution` values (e.g., 0.2 instead of 0.1) to reduce cells
- For real-time mapping, enable `map_reset_interval` to clear unused data

## Visualization

All mapping nodes publish standard `nav_msgs/OccupancyGrid` messages that can be visualized in RViz:

1. Add a Map display in RViz
2. Set the topic to:
   - `/permanent_map` for permanent mapper
   - `/realtime_map` for real-time mapper
   - `/imu_lidar_fusion_map` for IMU-LiDAR fusion

## Integration with Navigation

The occupancy grids produced by these mappers can be used with navigation stacks:

1. **Nav2**: Use as a costmap source for planning
2. **Custom Planning**: Import the occupancy grid for path planning algorithms
3. **SLAM**: Use as a base map for localization

## Troubleshooting

### Common Issues

1. **Missing or Sparse Maps**
   - Check that LiDAR data is being received (monitor `/lidar/points` topic)
   - Verify transform tree setup (tf_world_to_map → tf_map_to_base_link → tf_base_to_imu → tf_imu_to_lidar)
   - Increase `hit_weight` for more visible obstacles

2. **High CPU Usage**
   - Reduce `publish_rate` and `process_rate`
   - Increase `raycast_skip` and reduce `max_points_to_process`
   - Use a larger `map_resolution` value

3. **Map Drift or Instability**
   - For IMU-LiDAR fusion, adjust `yaw_weight` to emphasize more stable source
   - Increase filter sizes (`yaw_filter_size`) for smoother data
   - Check for transform issues between sensor frames 