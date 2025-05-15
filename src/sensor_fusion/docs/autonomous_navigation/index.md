# Autonomous Navigation System Documentation

## Overview

This documentation provides a comprehensive guide to the autonomous navigation system implemented in the sensor_fusion package. The system uses the Dynamic Window Approach (DWA) algorithm for obstacle avoidance without requiring a specific goal, making it ideal for exploratory navigation in the Carla simulator environment.

## Documentation Contents

### System Documentation

1. [System Overview](README.md) - Complete overview of the autonomous navigation system
2. [System Architecture](system_architecture.md) - Detailed architecture diagram of the ROS 2 components
3. [Complete System Architecture](complete_system_architecture.md) - Full architecture including C++ preprocessing components
4. [System Prompt](system_prompt.md) - Quick reference guide for system usage

### Component Documentation

4. [Autonomous DWA Planner](autonomous_dwa_planner.md) - Detailed documentation of the DWA planner
5. [Launch File Documentation](launch_file.md) - Comprehensive guide to the launch file and its parameters
6. [Carla Integration Guide](carla_integration.md) - Instructions for integrating with the Carla simulator

## Quick Start

To get started with the autonomous navigation system:

```bash
# Source the workspace
source /home/mostafa/GP/ROS2/install/setup.bash

# Launch the autonomous navigation system
ros2 launch sensor_fusion autonomous_navigation.launch.py
```

For custom configuration:

```bash
# Launch with custom parameters
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  robot_radius:=1.5 \
  max_linear_velocity:=3.0 \
  obstacle_weight:=3.0 \
  clearance_threshold:=2.5
```

## Key Features

- **Pure Obstacle Avoidance**: Navigates without requiring a specific goal
- **Adaptive Navigation**: Adjusts trajectory based on detected obstacles
- **Carla Integration**: Seamlessly integrates with the Carla simulator
- **Configurable Parameters**: Extensive parameter set for tuning behavior
- **Real-time Visualization**: Visualizes trajectories and obstacles in RViz

## System Requirements

- ROS 2 (Foxy or newer)
- Carla Simulator (0.9.10 or newer) for integration testing
- Python 3.6+
- Required ROS 2 packages: rclpy, sensor_msgs, nav_msgs, geometry_msgs, visualization_msgs

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/foxy/index.html)
- [Carla Simulator Documentation](https://carla.readthedocs.io/)
- [Dynamic Window Approach Paper](https://ieeexplore.ieee.org/document/580977)
