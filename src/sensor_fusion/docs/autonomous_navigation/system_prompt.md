# Autonomous Navigation System Prompt

## System Overview

This autonomous navigation system implements a Dynamic Window Approach (DWA) planner for obstacle avoidance without requiring a specific goal. The system is designed to integrate with the Carla simulator for autonomous vehicle control.

## Key Components

1. **Autonomous DWA Planner**: Core navigation algorithm that generates safe trajectories
2. **Sensor Processing**: Processes LiDAR and IMU data for environmental awareness
3. **Sensor Fusion**: Combines data from multiple sensors for improved localization
4. **Trajectory Conversion**: Converts DWA paths to Carla-compatible trajectories
5. **Visualization**: Provides real-time visualization of the navigation process

## Features

- **Pure Obstacle Avoidance**: Navigates without requiring a specific goal
- **Adaptive Navigation**: Adjusts trajectory based on detected obstacles
- **Carla Integration**: Seamlessly integrates with the Carla simulator
- **Configurable Parameters**: Extensive parameter set for tuning behavior
- **Real-time Visualization**: Visualizes trajectories and obstacles in RViz

## Usage Instructions

### Basic Usage

```bash
# Source the workspace
source /home/mostafa/GP/ROS2/install/setup.bash

# Launch the autonomous navigation system
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

## Parameter Quick Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 1.0 | Robot radius for collision checking (meters) |
| `max_linear_velocity` | 5.0 | Maximum linear velocity (m/s) |
| `max_angular_velocity` | 0.5 | Maximum angular velocity (rad/s) |
| `obstacle_weight` | 2.0 | Weight for obstacle cost |
| `forward_weight` | 1.0 | Weight for forward motion preference |
| `clearance_threshold` | 2.0 | Minimum clearance to obstacles (meters) |

## Example Scenarios

### Urban Navigation

For navigating in urban environments with many obstacles:

```bash
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  max_linear_velocity:=3.0 \
  obstacle_weight:=3.0 \
  clearance_threshold:=2.5
```

### Highway Navigation

For higher-speed navigation with fewer obstacles:

```bash
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  max_linear_velocity:=10.0 \
  obstacle_weight:=1.5 \
  clearance_threshold:=1.5
```

### Parking Lot Navigation

For slow, precise navigation in tight spaces:

```bash
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  max_linear_velocity:=1.5 \
  max_angular_velocity:=0.8 \
  obstacle_weight:=4.0 \
  clearance_threshold:=0.8
```

## Documentation References

For more detailed information, refer to the following documentation:

- [README.md](README.md): Complete system overview
- [autonomous_dwa_planner.md](autonomous_dwa_planner.md): DWA planner details
- [launch_file.md](launch_file.md): Launch file documentation
- [carla_integration.md](carla_integration.md): Carla integration guide
