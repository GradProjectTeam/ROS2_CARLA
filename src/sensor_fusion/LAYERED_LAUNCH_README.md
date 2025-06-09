# Layered Launch System for Trajectory Control

This document explains the layered launch system created for the trajectory control stack. Each layer builds on the previous one, allowing you to test components individually or in combination.

## Overview of Layers

0. **Layer 0: LiDAR and IMU Fusion** (`layered_00_lidar_imu.launch.py`)
   - Sets up TCP connections to LiDAR and IMU sensors via native sensor clients
   - Creates TF tree (world → map → base_link → imu_link → lidar_link)
   - Processes raw LiDAR data using `lidar_listener_clusters_2` for point cloud visualization and clustering
   - Launches IMU visualization and processing via `imu_euler_visualizer`
   - Launches IMU-LiDAR fusion node for localization
   - Provides real-time binary (black/white) mapping optimized for path planning
   - Includes RViz for visualization

1. **Layer 1: TF Tree and Map Generation** (`layered_01_tf_map.launch.py`)
   - Includes Layer 0
   - Provides a higher-level interface with simplified parameters
   - Maintains backward compatibility with existing system

2. **Layer 2: Path Planning** (`layered_02_path_planning.launch.py`)
   - Includes Layer 1
   - Launches path publishers for various planners (Hybrid A*, A*, DWA, MPC)
   - Launches goal pose publisher

3. **Layer 3: Path to Trajectory Conversion** (`layered_03_path_to_trajectory.launch.py`)
   - Includes Layer 2
   - Launches the path to trajectory converter node

4. **Layer 4: Trajectory Visualization** (`layered_04_trajectory_visualization.launch.py`)
   - Includes Layer 3
   - Launches the trajectory visualizer node

5. **Layer 5: Odometry Support** (`layered_05_odometry.launch.py`)
   - Includes Layer 4
   - Launches the odometry publisher for testing

6. **Layer 6: MPC Control** (`layered_06_mpc_control.launch.py`)
   - Includes Layer 5
   - Launches the MPC trajectory controller node
   - Represents the complete trajectory control stack

## How to Use the Layered Launch System

The layered launch system allows you to test different parts of the trajectory control stack independently. Each layer includes all previous layers, so you only need to launch the highest-level layer you want to test.

### Testing the Full Stack

To test the complete trajectory control stack, launch Layer 6:

```bash
ros2 launch sensor_fusion layered_06_mpc_control.launch.py
```

### Testing Path Planning and Visualization Only

To test path planning and visualization without the controller, launch Layer 4:

```bash
ros2 launch sensor_fusion layered_04_trajectory_visualization.launch.py
```

### Testing Only LiDAR and IMU with Map Generation

To test just the LiDAR/IMU fusion and map generation:

```bash
ros2 launch sensor_fusion layered_00_lidar_imu.launch.py
```

## Common Launch Parameters

You can customize the behavior of the layers using these parameters:

### General Parameters

- `use_sim_time:=true|false` - Use simulation time
- `map_topic:=/your_map_topic` - Change the map topic name

### LiDAR and IMU Parameters (Layer 0)

- `imu_tcp_ip:=127.0.0.1` - IP address of the IMU TCP server
- `imu_tcp_port:=12345` - Port number of the IMU TCP server
- `lidar_tcp_ip:=127.0.0.1` - IP address of the LiDAR TCP server
- `lidar_tcp_port:=12350` - Port number of the LiDAR TCP server
- `map_resolution:=0.3` - Map resolution in meters per cell (optimized for path planning)
- `point_size:=1.0` - Size of visualization points for LiDAR data
- `center_size:=2.0` - Size of cluster center visualization
- `filter_vehicle_points:=true` - Whether to filter points on the vehicle itself

### LiDAR Visualization Parameters (Layer 0)

- `use_cluster_data:=true` - Whether to use clustered LiDAR data for mapping
- `use_convex_hull:=false` - Whether to show convex hull around point clusters
- `vehicle_length:=5.0` - Length of vehicle for point filtering (meters)
- `vehicle_width:=2.5` - Width of vehicle for point filtering (meters)
- `min_point_distance:=3.0` - Minimum distance to keep LiDAR points (creates blind spot)

### Binary Map Parameters (Layer 0)

- `use_binary_map:=true` - Use a binary (black/white) map representation
- `hit_weight:=1.5` - Weight for obstacle hits in Bayesian update (higher creates clearer obstacles)
- `miss_weight:=0.8` - Weight for misses in Bayesian update (higher clears space more aggressively)
- `obstacle_threshold:=0.5` - Threshold for classifying cells as obstacles
- `obstacle_inflation:=0.4` - How much to inflate obstacles for safer path planning

### Planner Parameters

- `planner_publish_rate:=10.0` - Rate at which planners publish paths (Hz)

### Vehicle Parameters

- `wheelbase:=2.7` - Vehicle wheelbase (meters)
- `vehicle_width:=2.0` - Vehicle width (meters)
- `vehicle_length:=4.0` - Vehicle length (meters)

### Controller Parameters

- `trajectory_topic:=/planner/hybrid_astar/trajectory` - Which planner's trajectory to follow
- `control_topic:=/cmd_vel` - Output topic for control commands
- `control_rate:=20.0` - Rate at which control commands are published (Hz)

### MPC Parameters (Layer 6 only)

- `prediction_horizon:=10` - MPC prediction horizon length
- `dt:=0.1` - MPC time step (seconds)
- `max_linear_velocity:=2.0` - Maximum linear velocity (m/s)
- `w_x:=1.0`, `w_y:=1.0`, etc. - MPC tracking weights

## Example: Choose a Different Planner's Trajectory

To use DWA's trajectory instead of Hybrid A*:

```bash
ros2 launch sensor_fusion layered_06_mpc_control.launch.py trajectory_topic:=/planner/dwa/trajectory
```

## Example: Adjust MPC Parameters

To adjust MPC prediction horizon and weights:

```bash
ros2 launch sensor_fusion layered_06_mpc_control.launch.py prediction_horizon:=15 w_x:=2.0 w_y:=2.0
```

## Example: Configure LiDAR and IMU Connections

To connect to real sensors at specific IP addresses:

```bash
ros2 launch sensor_fusion layered_00_lidar_imu.launch.py lidar_tcp_ip:=192.168.1.10 imu_tcp_port:=12340
```

## Example: Tune Map Parameters for Path Planning

To adjust map parameters for better path planning performance:

```bash
ros2 launch sensor_fusion layered_00_lidar_imu.launch.py map_resolution:=0.25 obstacle_inflation:=0.5 hit_weight:=2.0
```

## Troubleshooting

If a layer isn't working correctly, try launching the previous layer to isolate the issue. For example, if Layer 6 (MPC Control) isn't working, try Layer 5 to see if the odometry data is being published properly.

### Binary Map Troubleshooting

If the map appears too cluttered with obstacles:
- Increase `miss_weight` (e.g., `miss_weight:=1.0`)
- Increase `ground_threshold` (e.g., `ground_threshold:=0.25`)
- Reduce `hit_weight` (e.g., `hit_weight:=1.0`)

If the map doesn't show enough obstacles:
- Increase `hit_weight` (e.g., `hit_weight:=2.0`)
- Reduce `miss_weight` (e.g., `miss_weight:=0.5`)
- Reduce `ground_threshold` (e.g., `ground_threshold:=0.15`)

### LiDAR Data Issues

If you're not seeing LiDAR data:
- Verify the TCP connection parameters `lidar_tcp_ip` and `lidar_tcp_port`
- Check the topics `/clusters` and `/lidar/points` with `ros2 topic echo`
- Look for error messages from `lidar_listener_clusters_2` in the terminal output

For deeper debugging, you can launch each node individually using ROS2 commands. 