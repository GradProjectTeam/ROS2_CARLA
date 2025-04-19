# Sensor Fusion Package Launch Files Documentation

This document provides an overview of all launch files in the `sensor_fusion` package and the nodes they start.

## Launch Files Overview

| Launch File | Primary Purpose |
|-------------|-----------------|
| `permanent_map.launch.py` | Launch the permanent mapping system for creating persistent maps |
| `realtime_map.launch.py` | Launch the real-time mapping system for immediate obstacle detection |
| `imu_lidar_fusion.launch.py` | Launch the IMU and LiDAR fusion system |
| `imu_euler_viz.launch.py` | Launch IMU visualization only |
| `lidar_test.launch.py` | Test LiDAR functionality standalone |
| `radar_visualization.launch.py` | Visualize radar data |
| `radar_test.launch.py` | Test radar functionality standalone |
| `imu_test.launch.py` | Test IMU functionality standalone |
| `sensor_fusion.launch.py` | Launch the complete sensor fusion system |

## Launch Files Details

### 1. permanent_map.launch.py

**Purpose**: Creates a persistent map of the environment using LiDAR data.

**Launch Arguments**:
- TCP connection parameters: `lidar_tcp_ip`, `lidar_tcp_port`
- Visualization parameters: `point_size`, `center_size`, etc.
- Map parameters: `map_resolution`, `map_width_meters`, `map_height_meters`, etc.
- Performance parameters: `publish_rate`, `process_rate`, etc.
- Bayesian update parameters: `hit_weight`, `miss_weight`
- Map saving parameters: `map_save_dir`, `enable_auto_save`, etc.
- Vehicle filter parameters

**Nodes**:
- `lidar_cube_visualizer` (executable: `lidar_listener_clusters_2`)
- `lidar_permanent_mapper` (executable: `lidar_permanent_mapper`)
- `rviz2` (optional)

### 2. realtime_map.launch.py

**Purpose**: Creates a real-time map of the environment with focus on immediate surroundings.

**Launch Arguments**:
- TCP connection parameters: `imu_tcp_ip`, `imu_tcp_port`, `lidar_tcp_ip`, `lidar_tcp_port`
- Map parameters: `map_resolution`, `map_width_meters`, `map_height_meters`, etc.
- Performance parameters: `publish_rate`, `process_rate`
- Visualization parameters: `point_size`, `center_size`, etc.

**Nodes**:
- Static transform publishers:
  - `tf_world_to_map`
  - `tf_map_to_base_link`
  - `tf_base_to_imu`
  - `tf_imu_to_lidar`
- `imu_euler_visualizer`
- `lidar_cube_visualizer` (executable: `lidar_listener_clusters_2`)
- `lidar_realtime_mapper`
- `rviz2` (optional)

### 3. imu_lidar_fusion.launch.py

**Purpose**: Fuses IMU and LiDAR data for improved localization and mapping.

**Launch Arguments**:
- TCP connection parameters for IMU and LiDAR
- IMU parameters
- Map parameters
- Performance parameters
- Bayesian update parameters

**Nodes**:
- Static transform publishers
- `imu_euler_visualizer`
- `imu_lidar_yaw_fusion`
- `lidar_cube_visualizer` (executable: `lidar_listener_clusters_2`)
- `lidar_permanent_mapper` (with IMU integration)
- `rviz2` (optional)

### 4. imu_euler_viz.launch.py

**Purpose**: Visualize IMU data in Euler angle format.

**Launch Arguments**:
- TCP connection parameters: `imu_tcp_ip`, `imu_tcp_port`
- Visualization parameters

**Nodes**:
- Static transform publishers
- `imu_euler_visualizer`
- `rviz2` (optional)

### 5. lidar_test.launch.py

**Purpose**: Test LiDAR functionality standalone.

**Launch Arguments**:
- TCP connection parameters: `lidar_tcp_ip`, `lidar_tcp_port`
- Visualization parameters

**Nodes**:
- Static transform publishers
- `lidar_cube_visualizer` (executable: `lidar_listener_clusters_2`)
- `rviz2` (optional)

### 6. radar_visualization.launch.py

**Purpose**: Visualize radar data.

**Nodes**:
- `radar_visualizer`
- `rviz2` (optional)

### 7. radar_test.launch.py

**Purpose**: Test radar functionality standalone.

**Launch Arguments**:
- TCP connection parameters: `radar_tcp_ip`, `radar_tcp_port`
- Visualization parameters

**Nodes**:
- Static transform publishers
- `radar_listener_clusters`
- `rviz2` (optional)

### 8. imu_test.launch.py

**Purpose**: Test IMU functionality standalone.

**Launch Arguments**:
- TCP connection parameters: `imu_tcp_ip`, `imu_tcp_port`
- Visualization parameters

**Nodes**:
- Static transform publishers
- `imu_euler_visualizer`
- `rviz2` (optional)

### 9. sensor_fusion.launch.py

**Purpose**: Launch the complete sensor fusion system with LiDAR, Radar, and IMU integration.

**Launch Arguments**:
- TCP connection parameters for LiDAR, Radar, and IMU
- Fusion parameters: `lidar_weight`, `radar_weight`, `use_dynamic_weighting`
- Planning parameters: `max_speed`, `planning_frequency`
- Costmap parameters: `map_resolution`, `map_width_meters`, etc.
- Performance parameters: `publish_rate`, `process_rate`, etc.

**Nodes**:
- Static transform publishers
- `lidar_cube_visualizer` (executable: `lidar_listener_clusters_2`)
- `lidar_costmap_creator`
- `radar_listener_clusters`
- `imu_euler_visualizer`
- `sensor_fusion_node`
- `motion_planner`
- `rviz2` (optional)

## Node Details

### Key Executables

1. **lidar_listener_clusters_2**
   - Receives raw LiDAR data over TCP
   - Processes point clouds into clusters
   - Filters vehicle points
   - Publishes point clouds and visualization markers

2. **lidar_permanent_mapper**
   - Creates a persistent occupancy grid map from LiDAR data
   - Uses Bayesian updates for consistent mapping
   - Supports map saving and loading
   - Can incorporate IMU data when available

3. **lidar_realtime_mapper**
   - Creates a real-time occupancy grid map optimized for immediate surroundings
   - Uses temporal decay for fast updates
   - Centered on vehicle position

4. **imu_euler_visualizer**
   - Receives IMU data over TCP
   - Visualizes orientation in Euler angles
   - Publishes transforms and standard IMU messages

5. **imu_lidar_yaw_fusion**
   - Fuses IMU yaw data with LiDAR odometry
   - Improves rotational stability in mapping

6. **radar_listener_clusters**
   - Receives radar data over TCP
   - Processes radar returns into clusters
   - Publishes radar visualization and point clouds

7. **sensor_fusion_node**
   - Fuses data from LiDAR, Radar, and IMU
   - Creates a unified world model
   - Supports dynamic confidence-based weighting

8. **motion_planner**
   - Plans trajectories based on the fused sensor data
   - Avoids obstacles
   - Publishes control commands 