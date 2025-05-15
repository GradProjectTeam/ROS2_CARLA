# Autonomous Navigation System Documentation

This documentation provides a comprehensive overview of the autonomous navigation system implemented in the sensor_fusion package. The system uses the Dynamic Window Approach (DWA) algorithm for obstacle avoidance without requiring a specific goal.

## Table of Contents

1. [System Overview](#system-overview)
2. [Launch File](#launch-file)
3. [Node Documentation](#node-documentation)
   - [Autonomous DWA Planner](#autonomous-dwa-planner)
   - [LiDAR Data Processing](#lidar-data-processing)
   - [IMU Data Processing](#imu-data-processing)
   - [IMU-LiDAR Fusion](#imu-lidar-fusion)
   - [Path to Trajectory Converter](#path-to-trajectory-converter)
4. [Parameter Configuration](#parameter-configuration)
5. [Visualization](#visualization)
6. [Integration with Carla](#integration-with-carla)
7. [Usage Examples](#usage-examples)

## System Overview

The autonomous navigation system is designed to enable a vehicle to navigate autonomously while avoiding obstacles. Unlike traditional navigation systems that require a specific goal, this system focuses on continuous forward motion with obstacle avoidance. It's particularly suited for exploratory missions or scenarios where the vehicle needs to navigate safely through an environment without a predetermined destination.

The complete system architecture consists of three main components:

1. **Carla Simulator**: Generates raw sensor data (Radar, LiDAR, IMU) and receives control commands
2. **C++ Preprocessing Components**: Process raw sensor data before sending it to ROS 2
3. **ROS 2 Navigation System**: Performs sensor fusion, path planning, and trajectory generation

The data flow is as follows:

1. Carla generates raw sensor data and sends it via TCP to C++ preprocessing components
2. C++ components process the data and send it via TCP to ROS 2 nodes
3. ROS 2 nodes fuse the sensor data and use the DWA algorithm to generate safe trajectories
4. Trajectories are converted to control commands and sent back to Carla via TCP

For a detailed visual representation, see the [Complete System Architecture](complete_system_architecture.md).

## Launch File

The `autonomous_navigation.launch.py` file orchestrates the entire system. It launches all necessary nodes, sets up the transform tree, and configures parameters for the system.

### Key Components:

1. **Transform Configuration**: Establishes the TF tree (world → map → base_link → imu_link → lidar_link)
2. **Sensor Data Processing**: Launches nodes for LiDAR and IMU data processing
3. **Autonomous Navigation**: Configures and launches the DWA planner for obstacle avoidance
4. **Trajectory Generation**: Converts planned paths to Carla-compatible trajectories
5. **Visualization**: Sets up RViz for real-time visualization

### Launch Arguments:

The launch file provides numerous configurable parameters, including:

- Simulation time settings
- Network parameters for sensors
- LiDAR mounting position
- Robot dimensions and dynamics
- Cost function weights for the DWA algorithm
- Navigation parameters

## Node Documentation

### Autonomous DWA Planner

**Node Name**: `autonomous_dwa_planner`  
**Executable**: `autonomous_dwa_node`

The Autonomous DWA Planner is the core of the navigation system. It implements the Dynamic Window Approach algorithm to generate safe trajectories that avoid obstacles while maintaining forward motion.

#### Functionality:

1. **Obstacle Detection**: Processes laser scan data to identify obstacles
2. **Dynamic Window Calculation**: Determines feasible velocity commands based on robot dynamics
3. **Trajectory Prediction**: Generates potential trajectories for different velocity commands
4. **Cost Evaluation**: Evaluates trajectories based on:
   - Obstacle proximity (highest priority)
   - Forward motion preference
   - Heading stability
   - Velocity optimization
5. **Command Generation**: Selects the optimal trajectory and publishes velocity commands

#### Subscribed Topics:

- `/scan` (sensor_msgs/LaserScan): Laser scan data for obstacle detection
- `/odom` (nav_msgs/Odometry): Odometry data for current pose and velocity

#### Published Topics:

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot control
- `/dwa/trajectories` (visualization_msgs/MarkerArray): Visualization of evaluated trajectories
- `/dwa/best_trajectory` (nav_msgs/Path): The selected optimal trajectory
- `/dwa/robot` (visualization_msgs/Marker): Robot visualization
- `/dwa/clearance` (visualization_msgs/Marker): Clearance threshold visualization

#### Parameters:

- **Robot Parameters**:
  - `robot_radius`: Robot radius for collision checking (default: 1.0m)
  - `max_linear_velocity`: Maximum linear velocity (default: 5.0m/s)
  - `min_linear_velocity`: Minimum linear velocity (default: 0.0m/s)
  - `max_angular_velocity`: Maximum angular velocity (default: 0.5rad/s)
  - `min_angular_velocity`: Minimum angular velocity (default: -0.5rad/s)
  - `max_linear_accel`: Maximum linear acceleration (default: 1.0m/s²)
  - `max_angular_accel`: Maximum angular acceleration (default: 0.5rad/s²)

- **Cost Function Weights**:
  - `obstacle_weight`: Weight for obstacle cost (default: 2.0)
  - `forward_weight`: Weight for forward motion preference (default: 1.0)
  - `heading_weight`: Weight for heading stability (default: 1.0)
  - `velocity_weight`: Weight for velocity preference (default: 0.5)

- **Navigation Parameters**:
  - `preferred_direction`: Preferred direction in radians (default: 0.0)
  - `clearance_threshold`: Minimum clearance to consider a path safe (default: 2.0m)
  - `stop_threshold`: Distance to obstacle that triggers stopping (default: 0.5m)

### LiDAR Data Processing

**Node Name**: `lidar_cube_visualizer`  
**Executable**: `lidar_listener_clusters_2`

This node processes raw LiDAR data, filters it, and extracts meaningful information about obstacles in the environment.

#### Functionality:

1. **Data Acquisition**: Connects to LiDAR sensor via TCP
2. **Point Cloud Processing**: Filters and processes point cloud data
3. **Obstacle Detection**: Identifies obstacles from point cloud data
4. **Visualization**: Generates visualization markers for obstacles

#### Parameters:

- `tcp_ip`: IP address for LiDAR TCP connection (default: 127.0.0.1)
- `tcp_port`: Port for LiDAR TCP connection (default: 8080)
- `use_sim_time`: Whether to use simulation time (default: true)

### IMU Data Processing

**Node Name**: `imu_euler_visualizer`  
**Executable**: `imu_euler_visualizer`

This node processes IMU data to extract orientation information and visualize it.

#### Functionality:

1. **Data Acquisition**: Connects to IMU sensor via TCP
2. **Orientation Extraction**: Processes IMU data to extract roll, pitch, and yaw
3. **Visualization**: Generates visualization markers for IMU orientation

#### Parameters:

- `tcp_ip`: IP address for IMU TCP connection (default: 127.0.0.1)
- `tcp_port`: Port for IMU TCP connection (default: 8081)
- `use_sim_time`: Whether to use simulation time (default: true)

### IMU-LiDAR Fusion

**Node Name**: `imu_lidar_fusion`  
**Executable**: `imu_lidar_yaw_fusion`

This node fuses IMU and LiDAR data to improve localization and orientation estimation.

#### Functionality:

1. **Data Fusion**: Combines IMU and LiDAR data for improved orientation estimation
2. **Transform Publication**: Publishes dynamic transforms based on fused data

#### Parameters:

- `use_sim_time`: Whether to use simulation time (default: true)

### Path to Trajectory Converter

**Node Name**: `path_to_trajectory_converter`  
**Executable**: `path_to_trajectory_converter`

This node converts the path generated by the DWA planner to a trajectory format compatible with Carla.

#### Functionality:

1. **Path Conversion**: Converts Path messages to Trajectory messages
2. **Vehicle Constraints**: Applies vehicle constraints to the trajectory
3. **Speed Profiling**: Generates speed profiles for the trajectory

#### Parameters:

- `input_path_topic`: Topic for input path (default: /dwa/best_trajectory)
- `output_trajectory_topic`: Topic for output trajectory (default: /trajectory)
- `vehicle_width`: Width of the vehicle (default: 2.0m)
- `vehicle_length`: Length of the vehicle (default: 4.5m)
- `max_speed`: Maximum speed for the trajectory (default: 5.0m/s)
- `min_speed`: Minimum speed for the trajectory (default: 0.0m/s)
- `publish_rate`: Rate at which to publish trajectories (default: 10.0Hz)

## Parameter Configuration

The system provides extensive parameter configuration options through launch arguments. These parameters can be adjusted to tune the system's behavior for different vehicles and environments.

### Example Configuration:

```bash
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  robot_radius:=1.5 \
  max_linear_velocity:=3.0 \
  obstacle_weight:=3.0 \
  clearance_threshold:=2.5
```

## Visualization

The system includes comprehensive visualization capabilities through RViz. The visualization includes:

- Robot representation
- Obstacle detection
- Evaluated trajectories (color-coded by cost)
- Selected optimal trajectory
- Clearance threshold

## Integration with Carla

The system is designed to integrate with the Carla simulator for autonomous vehicle control. The integration is achieved through:

1. **Path to Trajectory Conversion**: Converting DWA paths to Carla-compatible trajectories
2. **Vehicle Dynamics Consideration**: Accounting for vehicle dimensions and dynamics
3. **Simulation Time Synchronization**: Synchronizing with Carla's simulation time

## Usage Examples

### Basic Usage:

```bash
# Source the workspace
source /home/mostafa/GP/ROS2/install/setup.bash

# Launch the autonomous navigation system
ros2 launch sensor_fusion autonomous_navigation.launch.py
```

### Custom Configuration:

```bash
# Launch with custom parameters
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  robot_radius:=1.5 \
  max_linear_velocity:=3.0 \
  obstacle_weight:=3.0 \
  clearance_threshold:=2.5 \
  lidar_tcp_ip:=192.168.1.100 \
  lidar_tcp_port:=8080
```

### Integration with Carla:

```bash
# First, start Carla simulator
./CarlaUE4.sh

# Then launch the autonomous navigation system
ros2 launch sensor_fusion autonomous_navigation.launch.py use_sim_time:=true
```
