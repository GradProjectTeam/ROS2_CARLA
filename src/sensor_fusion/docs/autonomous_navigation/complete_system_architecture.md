# Complete System Architecture

This document describes the full architecture of the autonomous navigation system, including the data flow from Carla through C++ preprocessing components to ROS 2 and back to Carla.

## System Overview

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
│                       │ Sensor Fusion │                             │
│                       │               │                             │
│                       └───────┬───────┘                             │
│                               │                                     │
│                               ▼                                     │
│                       ┌───────────────┐                             │
│                       │ Autonomous    │                             │
│                       │ DWA Planner   │                             │
│                       │               │                             │
│                       └───────┬───────┘                             │
│                               │                                     │
│                               ▼                                     │
│                       ┌───────────────┐                             │
│                       │ Path to       │                             │
│                       │ Trajectory    │                             │
│                       │ Converter     │                             │
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

## Component Details

### 1. Carla Simulator (Data Source)

Carla generates sensor data and sends it over TCP connections:
- **Radar Data**: Sent on a dedicated TCP port
- **LiDAR Data**: Sent on a dedicated TCP port
- **IMU Data**: Sent on a dedicated TCP port

### 2. C++ Preprocessing Components

Three separate C++ applications process the raw sensor data before it reaches ROS 2:

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

### 3. ROS 2 System

The ROS 2 system receives the processed sensor data and performs autonomous navigation:

#### Sensor Listeners
- **Radar Listener**: Connects to the Radar Processing TCP output
- **LiDAR Listener**: Connects to the LiDAR Processing TCP output
- **IMU Listener**: Connects to the IMU Processing TCP output

#### Sensor Fusion
- Combines data from all sensors for improved localization and perception

#### Autonomous DWA Planner
- Implements the Dynamic Window Approach for obstacle avoidance
- Generates safe trajectories based on sensor data

#### Path to Trajectory Converter
- Converts DWA paths to a format compatible with Carla

#### Control Command Generator
- Generates control commands (steering, throttle, brake) from trajectories
- Sends these commands back to Carla via TCP

### 4. Carla Simulator (Control Target)

Carla receives the control commands via TCP and updates the vehicle's state accordingly.

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
| Radar     | (Port #)     | (Format)    |
| LiDAR     | (Port #)     | (Format)    |
| IMU       | (Port #)     | (Format)    |

### From C++ Preprocessing to ROS 2

| Data Type | Default Port | Data Format |
|-----------|--------------|-------------|
| Processed Radar | (Port #) | (Format) |
| Processed LiDAR | (Port #) | (Format) |
| Processed IMU   | (Port #) | (Format) |

### From ROS 2 to Carla

| Data Type | Default Port | Data Format |
|-----------|--------------|-------------|
| Control Commands | (Port #) | (Format) |

## Configuration

The TCP ports and IP addresses can be configured in the launch file:

```bash
# Example: Configure TCP connections
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  lidar_tcp_ip:=<ip> \
  lidar_tcp_port:=<port> \
  radar_tcp_ip:=<ip> \
  radar_tcp_port:=<port> \
  imu_tcp_ip:=<ip> \
  imu_tcp_port:=<port> \
  control_tcp_ip:=<ip> \
  control_tcp_port:=<port>
```
