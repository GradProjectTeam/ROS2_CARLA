# Autonomous Navigation System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                        AUTONOMOUS NAVIGATION SYSTEM                 │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                          SENSOR DATA INPUTS                         │
│                                                                     │
│  ┌───────────────┐              ┌───────────────┐                   │
│  │               │              │               │                   │
│  │  LiDAR Data   │◄─────TCP─────┤  Carla        │                   │
│  │  (LaserScan)  │    (8080)    │  Simulator    │                   │
│  │               │              │               │                   │
│  └───────────────┘              │               │                   │
│         │                       │               │                   │
│         ▼                       │               │                   │
│  ┌───────────────┐              │               │                   │
│  │ lidar_listener│              │               │                   │
│  │ _clusters_2   │              │               │                   │
│  └───────────────┘              │               │                   │
│         │                       │               │                   │
│         │                       │               │                   │
│         │                       │               │                   │
│  ┌───────────────┐              │               │                   │
│  │               │              │               │                   │
│  │   IMU Data    │◄─────TCP─────┤               │                   │
│  │               │    (8081)    │               │                   │
│  │               │              │               │                   │
│  └───────────────┘              └───────────────┘                   │
│         │                                                           │
│         ▼                                                           │
│  ┌───────────────┐                                                  │
│  │ imu_euler_    │                                                  │
│  │ visualizer    │                                                  │
│  └───────────────┘                                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                          SENSOR FUSION                              │
│                                                                     │
│  ┌───────────────┐                                                  │
│  │ imu_lidar_    │                                                  │
│  │ yaw_fusion    │                                                  │
│  └───────────────┘                                                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                       NAVIGATION PLANNING                           │
│                                                                     │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │                                                           │     │
│  │                  AUTONOMOUS DWA PLANNER                   │     │
│  │                                                           │     │
│  │  ┌───────────────┐      ┌───────────────┐                │     │
│  │  │ Dynamic       │      │ Trajectory    │                │     │
│  │  │ Window        │─────►│ Generation    │                │     │
│  │  │ Calculation   │      │               │                │     │
│  │  └───────────────┘      └───────────────┘                │     │
│  │                                │                         │     │
│  │                                ▼                         │     │
│  │  ┌───────────────┐      ┌───────────────┐                │     │
│  │  │ Cost Function │      │ Best          │                │     │
│  │  │ Evaluation    │◄─────┤ Trajectory    │                │     │
│  │  │               │      │ Selection     │                │     │
│  │  └───────────────┘      └───────────────┘                │     │
│  │                                │                         │     │
│  └───────────────────────────────┬───────────────────────────┘     │
│                                  │                                 │
└──────────────────────────────────┼─────────────────────────────────┘
                                   │
                                   ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                        OUTPUT PROCESSING                            │
│                                                                     │
│  ┌───────────────┐              ┌───────────────┐                   │
│  │ Velocity      │              │ Path to       │                   │
│  │ Commands      │              │ Trajectory    │                   │
│  │ (cmd_vel)     │              │ Converter     │                   │
│  └───────────────┘              └───────────────┘                   │
│         │                              │                            │
│         │                              │                            │
│         ▼                              ▼                            │
│  ┌───────────────┐              ┌───────────────┐                   │
│  │ Robot         │              │ Carla         │                   │
│  │ Control       │              │ Trajectory    │                   │
│  │               │              │ Format        │                   │
│  └───────────────┘              └───────────────┘                   │
│                                        │                            │
│                                        │                            │
│                                        ▼                            │
│                               ┌───────────────┐                     │
│                               │ Carla         │                     │
│                               │ Simulator     │                     │
│                               │               │                     │
│                               └───────────────┘                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                                                                     │
│                          VISUALIZATION                              │
│                                                                     │
│  ┌───────────────┐      ┌───────────────┐      ┌───────────────┐   │
│  │ Robot         │      │ Obstacle      │      │ Trajectory    │   │
│  │ Visualization │      │ Visualization │      │ Visualization │   │
│  │               │      │               │      │               │   │
│  └───────────────┘      └───────────────┘      └───────────────┘   │
│         │                     │                      │              │
│         └─────────────────────┼──────────────────────┘              │
│                               │                                     │
│                               ▼                                     │
│                       ┌───────────────┐                             │
│                       │ RViz2         │                             │
│                       │               │                             │
│                       └───────────────┘                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Key Components Description

### Sensor Data Inputs
- **LiDAR Data**: Provides obstacle detection through laser scans
- **IMU Data**: Provides orientation and acceleration information
- **Data Processing Nodes**: Process raw sensor data into usable formats

### Sensor Fusion
- **IMU-LiDAR Fusion**: Combines data from both sensors for improved localization
- **Provides**: Enhanced orientation estimation and obstacle positioning

### Navigation Planning
- **Dynamic Window Calculation**: Determines feasible velocity commands
- **Trajectory Generation**: Predicts potential trajectories
- **Cost Function Evaluation**: Evaluates trajectories based on multiple criteria
- **Best Trajectory Selection**: Selects optimal trajectory with lowest cost

### Output Processing
- **Velocity Commands**: Direct control of robot motion
- **Path to Trajectory Converter**: Converts DWA paths to Carla format
- **Carla Integration**: Sends compatible trajectory commands to Carla

### Visualization
- **Robot Visualization**: Shows robot position and orientation
- **Obstacle Visualization**: Shows detected obstacles
- **Trajectory Visualization**: Shows evaluated and selected trajectories
- **RViz2**: Integrates all visualizations in a single interface
