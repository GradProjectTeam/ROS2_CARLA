# ROS2 Sensor Fusion Package

This repository contains a ROS2 package for sensor fusion, with a focus on LiDAR-based mapping and localization.

## 🔍 LiDAR Mapping and Integration

The sensor fusion package provides several launch files for LiDAR-based mapping and sensor integration:

### Launch Files

| Launch File | Description | Key Features |
|-------------|-------------|-------------|
| **permanent_map.launch.py** | Creates a persistent map of the environment using LiDAR data | - High-precision mapping<br>- Map saving/loading<br>- Bayesian updates |
| **realtime_map.launch.py** | Creates a real-time map with focus on immediate surroundings | - Temporal decay for dynamic environments<br>- Focus on vehicle surroundings<br>- High update rate |
| **imu_lidar_fusion.launch.py** | Fuses IMU and LiDAR data for improved localization | - Improved rotational stability<br>- Better mapping in dynamic environments |
| **lidar_test.launch.py** | Test LiDAR functionality standalone | - Basic LiDAR visualization<br>- Point cloud processing |
| **sensor_fusion.launch.py** | Complete sensor fusion with LiDAR, Radar, and IMU | - Multi-sensor integration<br>- Comprehensive world model |

### LiDAR-Related Nodes

#### LiDAR Processing
- **lidar_listener_clusters_2**: Processes raw LiDAR data into clusters, filters vehicle points, and publishes visualization.
- **lidar_permanent_mapper**: Creates persistent occupancy grid maps with Bayesian updates.
- **lidar_realtime_mapper**: Generates real-time maps with temporal decay for dynamic environments.
- **lidar_costmap_creator**: Converts point clouds to 2D costmaps for navigation.

#### Sensor Fusion
- **imu_lidar_yaw_fusion**: Improves rotational stability by fusing IMU and LiDAR data.
- **sensor_fusion_node**: Integrates data from multiple sensors into a unified world model.

## 🚀 Quick Start

### Permanent Mapping
For high-precision mapping and environment building:
```bash
ros2 launch sensor_fusion permanent_map.launch.py
```

### Real-time Mapping
For dynamic environments and obstacle avoidance:
```bash
ros2 launch sensor_fusion realtime_map.launch.py
```

### IMU-LiDAR Fusion
For mapping with improved orientation:
```bash
ros2 launch sensor_fusion imu_lidar_fusion.launch.py
```

### Full Sensor Fusion
For comprehensive sensor fusion and planning:
```bash
ros2 launch sensor_fusion sensor_fusion.launch.py
```

## ⚙️ Common Parameters

### Map Parameters
- `map_resolution`: Resolution of the map in meters per cell (default: 0.2m for permanent, 0.1m for real-time)
- `map_width_meters`/`map_height_meters`: Size of the map in meters (default: 100x100m for permanent, 40x40m for real-time)
- `detection_radius`: Maximum range for obstacle detection (default: 50m)

### Processing Parameters
- `publish_rate`: Rate to publish map updates (default: 1.0Hz for permanent, 10.0Hz for real-time)
- `process_rate`: Rate to process map data (default: 2.0Hz for permanent, 10.0Hz for real-time)
- `raycast_skip`: Only raytrace every Nth point for better performance (default: 5 for permanent, 1 for real-time)
- `max_points_to_process`: Maximum number of points to process per update (default: 3000 for permanent, 8000 for real-time)

### Bayesian Update Parameters
- `hit_weight`: Weight for obstacle hits in Bayesian update (default: 0.99 for permanent)
- `miss_weight`: Weight for misses (free space) in Bayesian update (default: 0.05 for permanent)

### Real-time Specific Parameters
- `decay_rate`: How quickly old points disappear (default: 0.9)
- `center_on_vehicle`: Keep map centered on vehicle position (default: true)

### IMU-Fusion Parameters
- `initial_yaw_offset`: Initial yaw offset in radians (default: 0.0)
- `yaw_weight`: Weight of IMU yaw in the fusion (default: 0.7)

## 🔧 Advanced Usage Examples

### Customize Permanent Map Resolution
```bash
ros2 launch sensor_fusion permanent_map.launch.py map_resolution:=0.1 map_width_meters:=200.0 map_height_meters:=200.0
```

### Real-time Mapping with Different Decay Rate
```bash
ros2 launch sensor_fusion realtime_map.launch.py decay_rate:=0.8 publish_rate:=15.0
```

### IMU-LiDAR Fusion with Custom Parameters
```bash
ros2 launch sensor_fusion imu_lidar_fusion.launch.py yaw_weight:=0.8 enable_costmap:=false
```

## 📚 Additional Documentation

For more detailed information on specific components:
- [LiDAR Mapping Documentation](src/sensor_fusion/LIDAR_MAPPING_README.md) - Comprehensive guide to LiDAR mapping capabilities
- [IMU-LiDAR Integration Documentation](src/sensor_fusion/README_IMU_LIDAR_INTEGRATION.md) - Details on IMU and LiDAR fusion
- [Permanent Map Documentation](src/sensor_fusion/PERMANENT_MAP_README.md) - Specifics about permanent mapping
- [Launch Files Documentation](src/sensor_fusion/launch/sensor_fusion_documentation.md) - Overview of all launch files 