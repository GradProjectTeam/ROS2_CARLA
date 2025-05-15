# Carla Integration Guide

## Overview

This document explains how the autonomous navigation system integrates with the Carla simulator. The integration enables testing and validation of the autonomous navigation capabilities in a realistic virtual environment before deployment on real vehicles.

## System Architecture

The integration between the autonomous navigation system and Carla follows this architecture:

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                        CARLA SIMULATOR                              │
│                                                                     │
└───────────────┬─────────────────┬─────────────────┬─────────────────┘
                │                 │                 │
                │ TCP             │ TCP             │ TCP
                │ (Radar)         │ (LiDAR)         │ (IMU)
                ▼                 ▼                 ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────┐
│                   │   │                   │   │                   │
│ C++ Radar         │   │ C++ LiDAR         │   │ C++ IMU           │
│ Processing        │   │ Processing        │   │ Processing        │
│                   │   │                   │   │                   │
└─────────┬─────────┘   └─────────┬─────────┘   └─────────┬─────────┘
          │                       │                       │
          │ TCP                   │ TCP                   │ TCP
          │ (Processed            │ (Processed            │ (Processed
          │  Radar Data)          │  LiDAR Data)          │  IMU Data)
          ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                          ROS 2 SYSTEM                               │
│                                                                     │
│  ┌───────────────┐      ┌───────────────┐      ┌───────────────┐   │
│  │ Radar         │      │ LiDAR         │      │ IMU           │   │
│  │ Listener      │      │ Listener      │      │ Listener      │   │
│  │               │      │               │      │               │   │
│  └───────┬───────┘      └───────┬───────┘      └───────┬───────┘   │
│          │                      │                      │            │
│          └──────────────────────┼──────────────────────┘            │
│                                 │                                   │
│                                 ▼                                   │
│                       ┌───────────────┐                             │
│                       │ Autonomous    │                             │
│                       │ DWA Planner   │                             │
│                       │               │                             │
│                       └───────┬───────┘                             │
│                               │                                     │
│                               ▼                                     │
│                       ┌───────────────┐                             │
│                       │ Control       │                             │
│                       │ Command       │                             │
│                       │ Generator     │                             │
│                       └───────┬───────┘                             │
│                               │                                     │
└───────────────────────────────┼─────────────────────────────────────┘
                                │
                                │ TCP
                                │ (Control Commands)
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                        CARLA SIMULATOR                              │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Key Components

### 1. C++ Preprocessing Components

Before sensor data reaches the ROS 2 system, it is processed by dedicated C++ applications:

#### Radar Processing
- **Location**: `/home/mostafa/ROS2andCarla/CPP/Radar_processing`
- **Input**: Raw radar data from Carla via TCP
- **Processing**: Filters, processes, and extracts meaningful information from radar data
- **Output**: Processed radar data sent via TCP to ROS 2

#### LiDAR Processing
- **Location**: `/home/mostafa/ROS2andCarla/CPP/Lidar_Processing`
- **Input**: Raw LiDAR point cloud data from Carla via TCP
- **Processing**: Point cloud filtering, obstacle detection, and feature extraction
- **Output**: Processed LiDAR data sent via TCP to ROS 2

#### IMU Processing
- **Location**: `/home/mostafa/ROS2andCarla/CPP/IMU_Processing`
- **Input**: Raw IMU data from Carla via TCP
- **Processing**: Orientation extraction, noise filtering, and data transformation
- **Output**: Processed IMU data sent via TCP to ROS 2

### 2. ROS 2 Sensor Listeners

The ROS 2 system includes nodes that connect to the C++ preprocessing components:

**Radar Listener:**
- Connects to the Radar Processing TCP output
- Converts the data to ROS 2 messages

**LiDAR Listener:**
- Connects to the LiDAR Processing TCP output (port 8080 by default)
- Processed by the `lidar_listener_clusters_2` node

**IMU Listener:**
- Connects to the IMU Processing TCP output (port 8081 by default)
- Processed by the `imu_euler_visualizer` node

### 3. Path to Trajectory Converter

The `path_to_trajectory_converter` node is a critical component for Carla integration. It converts the paths generated by the DWA planner into a format that Carla can understand and execute.

**Node Details:**
- **Node Name**: `path_to_trajectory_converter`
- **Executable**: `path_to_trajectory_converter`
- **Input**: `/dwa/best_trajectory` (nav_msgs/Path)
- **Output**: `/trajectory` (custom trajectory message compatible with Carla)

**Parameters:**
- `input_path_topic`: Topic for input path (default: /dwa/best_trajectory)
- `output_trajectory_topic`: Topic for output trajectory (default: /trajectory)
- `vehicle_width`: Width of the vehicle (default: 2.0m)
- `vehicle_length`: Length of the vehicle (default: 4.5m)
- `max_speed`: Maximum speed for the trajectory (default: 5.0m/s)
- `min_speed`: Minimum speed for the trajectory (default: 0.0m/s)
- `publish_rate`: Rate at which to publish trajectories (default: 10.0Hz)

### 4. Control Command Generator

This component converts trajectories to control commands (steering, throttle, brake) and sends them back to Carla.

**Output:**
- Control commands sent via TCP to Carla

### 5. Simulation Time Synchronization

To ensure proper synchronization between Carla and the autonomous navigation system, simulation time is used:

- The `use_sim_time` parameter is set to `true` by default
- All nodes use the simulation time provided by Carla

## Setup Instructions

### Prerequisites

1. Carla Simulator (version 0.9.10 or newer)
2. ROS2 (Foxy or newer)
3. C++ development environment with required libraries

### Installation Steps

1. **Install Carla Simulator:**
   ```bash
   # Download and extract Carla
   wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz
   mkdir -p ~/carla
   tar -xzf CARLA_0.9.13.tar.gz -C ~/carla
   ```

2. **Build C++ Preprocessing Components:**
   ```bash
   # Navigate to the C++ components directory
   cd /home/mostafa/ROS2andCarla/CPP
   
   # Build Radar Processing
   cd Radar_processing
   mkdir -p build && cd build
   cmake ..
   make
   
   # Build LiDAR Processing
   cd ../../Lidar_Processing
   mkdir -p build && cd build
   cmake ..
   make
   
   # Build IMU Processing
   cd ../../IMU_Processing
   mkdir -p build && cd build
   cmake ..
   make
   ```

3. **Configure Network Settings:**
   
   Ensure that the TCP/IP settings in the launch file match your Carla configuration:
   ```bash
   # Example: Modify the launch file parameters
   ros2 launch sensor_fusion autonomous_navigation.launch.py \
     lidar_tcp_ip:=<carla_ip> \
     lidar_tcp_port:=8080 \
     imu_tcp_ip:=<carla_ip> \
     imu_tcp_port:=8081 \
     radar_tcp_ip:=<carla_ip> \
     radar_tcp_port:=8082 \
     control_tcp_ip:=<carla_ip> \
     control_tcp_port:=8090
   ```

### Running the Integration

1. **Start Carla Simulator:**
   ```bash
   cd ~/carla
   ./CarlaUE4.sh
   ```

2. **Start C++ Preprocessing Components:**
   ```bash
   # Start Radar Processing
   cd /home/mostafa/ROS2andCarla/CPP/Radar_processing/build
   ./radar_processing
   
   # Start LiDAR Processing (in a new terminal)
   cd /home/mostafa/ROS2andCarla/CPP/Lidar_Processing/build
   ./lidar_processing
   
   # Start IMU Processing (in a new terminal)
   cd /home/mostafa/ROS2andCarla/CPP/IMU_Processing/build
   ./imu_processing
   ```

3. **Launch the Autonomous Navigation System:**
   ```bash
   source /home/mostafa/GP/ROS2/install/setup.bash
   ros2 launch sensor_fusion autonomous_navigation.launch.py
   ```

## Data Flow

1. **Sensor Data Generation**:
   - Carla generates raw sensor data (Radar, LiDAR, IMU)
   - Data is sent via TCP to respective C++ processing applications

2. **Data Preprocessing**:
   - C++ applications process the raw sensor data
   - Processed data is sent via TCP to ROS 2 listeners

3. **Autonomous Navigation**:
   - ROS 2 system fuses sensor data
   - DWA planner generates safe trajectories
   - Trajectories are converted to control commands

4. **Vehicle Control**:
   - Control commands are sent back to Carla via TCP
   - Carla updates the vehicle's state based on these commands

## TCP Communication Details

### From Carla to C++ Preprocessing

| Data Type | Default Port | Data Format |
|-----------|--------------|-------------|
| Radar     | 8082         | Binary data stream |
| LiDAR     | 8080         | Binary point cloud data |
| IMU       | 8081         | Binary IMU data |

### From C++ Preprocessing to ROS 2

| Data Type | Default Port | Data Format |
|-----------|--------------|-------------|
| Processed Radar | 9082 | Structured radar data |
| Processed LiDAR | 9080 | Filtered point cloud data |
| Processed IMU   | 9081 | Orientation and acceleration data |

### From ROS 2 to Carla

| Data Type | Default Port | Data Format |
|-----------|--------------|-------------|
| Control Commands | 8090 | Steering, throttle, brake commands |

## Vehicle Configuration

The autonomous navigation system needs to be configured to match the vehicle in Carla:

### Physical Dimensions

Adjust these parameters to match the Carla vehicle:
- `robot_radius`: Set to half the vehicle width
- `vehicle_width`: Match the Carla vehicle width
- `vehicle_length`: Match the Carla vehicle length

### Dynamic Constraints

Adjust these parameters to match the Carla vehicle dynamics:
- `max_linear_velocity`: Maximum speed the vehicle can achieve
- `max_angular_velocity`: Maximum turning rate
- `max_linear_accel`: Maximum acceleration
- `max_angular_accel`: Maximum turning acceleration

## Sensor Configuration

### LiDAR Configuration

Configure the LiDAR parameters to match the Carla LiDAR sensor:
- `lidar_x_offset`, `lidar_y_offset`, `lidar_z_offset`: Position relative to the vehicle
- `lidar_roll`, `lidar_pitch`, `lidar_yaw`: Orientation of the LiDAR

### IMU Configuration

Ensure the IMU data format matches what the autonomous navigation system expects:
- The IMU data should include acceleration, gyroscope, and orientation information

## Testing Scenarios

Here are some recommended testing scenarios for validating the integration:

### 1. Basic Navigation

Test the vehicle's ability to navigate in an open environment:
```bash
# Launch with default parameters
ros2 launch sensor_fusion autonomous_navigation.launch.py
```

### 2. Obstacle Avoidance

Test the vehicle's ability to avoid obstacles:
```bash
# Launch with increased obstacle weight
ros2 launch sensor_fusion autonomous_navigation.launch.py obstacle_weight:=3.0
```

### 3. High-Speed Navigation

Test the vehicle's ability to navigate at higher speeds:
```bash
# Launch with increased maximum velocity
ros2 launch sensor_fusion autonomous_navigation.launch.py max_linear_velocity:=10.0
```

### 4. Cluttered Environment

Test the vehicle's ability to navigate in cluttered environments:
```bash
# Launch with increased clearance and reduced speed
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  clearance_threshold:=3.0 \
  max_linear_velocity:=3.0
```

## Troubleshooting

### Common Issues

1. **No sensor data received:**
   - Check TCP/IP settings
   - Ensure Carla is properly configured to send sensor data
   - Verify that the correct ports are open

2. **Vehicle not moving:**
   - Check that the trajectory is being published correctly
   - Verify that Carla is receiving and processing the trajectory commands
   - Check for any error messages in the node logs

3. **Collisions with obstacles:**
   - Increase the `obstacle_weight` parameter
   - Increase the `clearance_threshold` parameter
   - Reduce the `max_linear_velocity` parameter

4. **Jerky motion:**
   - Reduce the `max_linear_accel` and `max_angular_accel` parameters
   - Increase the control frequency for smoother updates

### Debugging Tools

1. **Topic Monitoring:**
   ```bash
   # Monitor trajectory messages
   ros2 topic echo /trajectory
   
   # Monitor sensor data
   ros2 topic echo /scan
   ros2 topic echo /imu/data
   ```

2. **Visualization:**
   ```bash
   # Launch RViz separately for detailed visualization
   ros2 run rviz2 rviz2 -d /home/mostafa/GP/ROS2/src/sensor_fusion/rviz/dwa_planner.rviz
   ```

3. **Node Information:**
   ```bash
   # Check node status
   ros2 node list
   ros2 node info /autonomous_dwa_planner
   ```

## Performance Optimization

To optimize the performance of the integration:

1. **Adjust Update Frequencies:**
   - Increase `control_frequency` for more responsive control
   - Decrease `visualization_frequency` to reduce computational load

2. **Tune DWA Parameters:**
   - Reduce `prediction_steps` for faster computation
   - Increase `velocity_resolution` and `angular_velocity_resolution` for faster sampling

3. **Optimize Network Communication:**
   - Use local connections when possible
   - Reduce sensor data rate if not all data is needed

## References

1. Carla Simulator Documentation: [https://carla.readthedocs.io/](https://carla.readthedocs.io/)
2. ROS2 Bridge for Carla: [https://github.com/carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)
3. Dynamic Window Approach: [Fox, D., Burgard, W., & Thrun, S. (1997). The dynamic window approach to collision avoidance](https://ieeexplore.ieee.org/document/580977)
