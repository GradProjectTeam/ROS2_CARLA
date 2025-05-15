# Autonomous Navigation Launch File

## Overview

The `autonomous_navigation.launch.py` file is the main entry point for launching the autonomous navigation system. It orchestrates all the necessary components, configures their parameters, and establishes the connections between them. This launch file is designed to provide a complete solution for autonomous navigation with obstacle avoidance in the Carla simulator environment.

## File Location

```
/home/mostafa/GP/ROS2/src/sensor_fusion/launch/autonomous_navigation.launch.py
```

## Components

The launch file sets up the following components:

1. **Transform Tree**: Establishes the coordinate frames relationship
2. **Sensor Data Processing**: Launches nodes for processing LiDAR and IMU data
3. **Sensor Fusion**: Fuses data from different sensors for improved localization
4. **Autonomous Navigation**: Configures and launches the DWA planner
5. **Trajectory Generation**: Converts paths to Carla-compatible trajectories
6. **Visualization**: Sets up RViz for real-time visualization

## Launch Arguments

The launch file provides numerous configurable parameters through launch arguments:

### Basic Parameters

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | "true" | Whether to use simulation time |

### Network Parameters

| Argument | Default | Description |
|----------|---------|-------------|
| `lidar_tcp_ip` | "127.0.0.1" | IP address for LiDAR TCP connection |
| `lidar_tcp_port` | "8080" | Port for LiDAR TCP connection |
| `imu_tcp_ip` | "127.0.0.1" | IP address for IMU TCP connection |
| `imu_tcp_port` | "8081" | Port for IMU TCP connection |

### LiDAR Mounting Parameters

| Argument | Default | Description |
|----------|---------|-------------|
| `lidar_x_offset` | "0.0" | X offset of LiDAR from IMU (meters) |
| `lidar_y_offset` | "0.0" | Y offset of LiDAR from IMU (meters) |
| `lidar_z_offset` | "0.0" | Z offset of LiDAR from IMU (meters) |
| `lidar_roll` | "0.0" | Roll angle of LiDAR (radians) |
| `lidar_pitch` | "0.0" | Pitch angle of LiDAR (radians) |
| `lidar_yaw` | "0.0" | Yaw angle of LiDAR (radians) |

### DWA Parameters

| Argument | Default | Description |
|----------|---------|-------------|
| `robot_radius` | "1.0" | Robot radius for collision checking (meters) |
| `max_linear_velocity` | "5.0" | Maximum linear velocity (m/s) |
| `min_linear_velocity` | "0.0" | Minimum linear velocity (m/s) |
| `max_angular_velocity` | "0.5" | Maximum angular velocity (rad/s) |
| `min_angular_velocity` | "-0.5" | Minimum angular velocity (rad/s) |
| `max_linear_accel` | "1.0" | Maximum linear acceleration (m/s²) |
| `max_angular_accel` | "0.5" | Maximum angular acceleration (rad/s²) |

### Cost Function Weights

| Argument | Default | Description |
|----------|---------|-------------|
| `obstacle_weight` | "2.0" | Weight for obstacle cost |
| `forward_weight` | "1.0" | Weight for forward motion preference |
| `heading_weight` | "1.0" | Weight for heading stability |
| `velocity_weight` | "0.5" | Weight for velocity preference |

### Navigation Parameters

| Argument | Default | Description |
|----------|---------|-------------|
| `preferred_direction` | "0.0" | Preferred direction in radians (0 = forward) |
| `clearance_threshold` | "2.0" | Minimum clearance to consider a path safe (meters) |
| `stop_threshold` | "0.5" | Distance to obstacle that triggers stopping (meters) |

## Nodes

The launch file starts the following nodes:

### Transform Nodes

1. **world_to_map_node**: Static transform from world to map
2. **map_to_base_link_node**: Static transform from map to base_link
3. **base_to_imu_node**: Static transform from base_link to imu_link
4. **imu_to_lidar_node**: Static transform from imu_link to lidar_link

### Sensor Processing Nodes

1. **lidar_listener_node**: Processes LiDAR data
   - Package: `sensor_fusion`
   - Executable: `lidar_listener_clusters_2`
   - Name: `lidar_cube_visualizer`

2. **imu_euler_visualizer_node**: Processes IMU data
   - Package: `sensor_fusion`
   - Executable: `imu_euler_visualizer`
   - Name: `imu_euler_visualizer`

### Fusion Node

1. **imu_lidar_fusion_node**: Fuses IMU and LiDAR data
   - Package: `sensor_fusion`
   - Executable: `imu_lidar_yaw_fusion`
   - Name: `imu_lidar_fusion`

### Navigation Node

1. **autonomous_dwa_node**: Implements autonomous navigation
   - Package: `sensor_fusion`
   - Executable: `autonomous_dwa_node`
   - Name: `autonomous_dwa_planner`

### Trajectory Node

1. **path_to_trajectory_converter**: Converts paths to trajectories
   - Package: `sensor_fusion`
   - Executable: `path_to_trajectory_converter`
   - Name: `path_to_trajectory_converter`

### Visualization Node

1. **rviz_node**: Provides visualization
   - Package: `rviz2`
   - Executable: `rviz2`
   - Name: `rviz2`

## Usage Examples

### Basic Usage

```bash
# Source the workspace
source /home/mostafa/GP/ROS2/install/setup.bash

# Launch with default parameters
ros2 launch sensor_fusion autonomous_navigation.launch.py
```

### Custom Configuration

```bash
# Launch with custom parameters
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  robot_radius:=1.5 \
  max_linear_velocity:=3.0 \
  obstacle_weight:=3.0 \
  clearance_threshold:=2.5
```

### Integration with Carla

```bash
# First, start Carla simulator
./CarlaUE4.sh

# Then launch the autonomous navigation system
ros2 launch sensor_fusion autonomous_navigation.launch.py use_sim_time:=true
```

## Customization

The launch file can be customized in several ways:

1. **Modifying Launch Arguments**: Pass different values for the launch arguments
2. **Editing the Launch File**: Modify the file directly to change node configurations
3. **Adding New Nodes**: Add additional nodes for extended functionality

## Dependencies

The launch file depends on the following packages:

- `sensor_fusion`: Contains the core navigation nodes
- `tf2_ros`: For transform management
- `rviz2`: For visualization

## Troubleshooting

Common issues and solutions:

1. **Connection Refused Errors**: Check that the TCP IP and port for sensors are correct
2. **Transform Errors**: Ensure the TF tree is properly configured
3. **Node Crashes**: Check node logs for specific error messages

## Related Files

- `/home/mostafa/GP/ROS2/src/sensor_fusion/sensor_fusion/autonomous_dwa_node.py`: Main navigation algorithm
- `/home/mostafa/GP/ROS2/src/sensor_fusion/rviz/dwa_planner.rviz`: RViz configuration for visualization
