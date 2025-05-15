# Autonomous DWA Planner

## Overview

The Autonomous DWA (Dynamic Window Approach) Planner is a core component of the autonomous navigation system. It implements a modified version of the DWA algorithm that focuses on obstacle avoidance without requiring a specific goal. This makes it ideal for exploratory navigation or scenarios where the vehicle needs to navigate safely through an environment without a predetermined destination.

## Algorithm Description

The DWA algorithm works by:

1. **Defining a dynamic window** of feasible velocities based on the robot's current state and dynamics
2. **Generating trajectories** for sampled velocities within this window
3. **Evaluating trajectories** using a cost function
4. **Selecting the optimal trajectory** with the lowest cost

In this implementation, the cost function prioritizes:
- Obstacle avoidance (highest priority)
- Forward motion
- Heading stability
- Velocity optimization

## Node Details

- **Node Name**: `autonomous_dwa_planner`
- **Executable**: `autonomous_dwa_node`
- **Source File**: `/home/mostafa/GP/ROS2/src/sensor_fusion/sensor_fusion/autonomous_dwa_node.py`

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Laser scan data for obstacle detection |
| `/odom` | `nav_msgs/Odometry` | Odometry data for current pose and velocity |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for robot control |
| `/dwa/trajectories` | `visualization_msgs/MarkerArray` | Visualization of evaluated trajectories |
| `/dwa/best_trajectory` | `nav_msgs/Path` | The selected optimal trajectory |
| `/dwa/robot` | `visualization_msgs/Marker` | Robot visualization |
| `/dwa/clearance` | `visualization_msgs/Marker` | Clearance threshold visualization |

## Parameters

### Robot Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_radius` | float | 1.0 | Robot radius for collision checking (meters) |
| `max_linear_velocity` | float | 5.0 | Maximum linear velocity (m/s) |
| `min_linear_velocity` | float | 0.0 | Minimum linear velocity (m/s) |
| `max_angular_velocity` | float | 0.5 | Maximum angular velocity (rad/s) |
| `min_angular_velocity` | float | -0.5 | Minimum angular velocity (rad/s) |
| `max_linear_accel` | float | 1.0 | Maximum linear acceleration (m/s²) |
| `max_angular_accel` | float | 0.5 | Maximum angular acceleration (rad/s²) |
| `velocity_resolution` | float | 0.1 | Resolution for velocity sampling (m/s) |
| `angular_velocity_resolution` | float | 0.1 | Resolution for angular velocity sampling (rad/s) |
| `prediction_time` | float | 3.0 | Time horizon for trajectory prediction (seconds) |
| `prediction_steps` | int | 60 | Number of steps for trajectory prediction |

### Cost Function Weights

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `obstacle_weight` | float | 2.0 | Weight for obstacle cost |
| `forward_weight` | float | 1.0 | Weight for forward motion preference |
| `heading_weight` | float | 1.0 | Weight for heading stability |
| `velocity_weight` | float | 0.5 | Weight for velocity preference |

### Navigation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `preferred_direction` | float | 0.0 | Preferred direction in radians (0 = forward) |
| `clearance_threshold` | float | 2.0 | Minimum clearance to consider a path safe (meters) |
| `stop_threshold` | float | 0.5 | Distance to obstacle that triggers stopping (meters) |

### Other Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control_frequency` | float | 10.0 | Control loop frequency (Hz) |
| `visualization_frequency` | float | 5.0 | Visualization update frequency (Hz) |
| `base_frame` | string | "base_link" | Robot base frame ID |
| `odom_frame` | string | "odom" | Odometry frame ID |
| `map_frame` | string | "map" | Map frame ID |
| `show_trajectories` | bool | true | Whether to show predicted trajectories |
| `max_trajectories_shown` | int | 10 | Maximum number of trajectories to visualize |

## Implementation Details

### Key Methods

- `control_loop()`: Main control loop that runs at the control frequency
- `find_best_trajectory()`: Core DWA algorithm implementation
- `predict_trajectory()`: Predicts a trajectory for a given velocity pair
- `calculate_obstacle_cost()`: Calculates cost based on distance to obstacles
- `calculate_forward_cost()`: Calculates cost based on alignment with preferred direction
- `calculate_heading_cost()`: Calculates cost based on heading stability
- `calculate_velocity_cost()`: Calculates cost based on velocity preference

### Safety Features

- **Collision Detection**: Trajectories that would lead to collision are assigned infinite cost
- **Emergency Stop**: The robot stops if obstacles are detected within the stop threshold
- **Clearance Threshold**: Maintains a safe distance from obstacles during navigation

## Tuning Guidelines

### For Different Vehicles

- Adjust `robot_radius` to match the vehicle's physical dimensions
- Set `max_linear_velocity` and `max_angular_velocity` based on vehicle capabilities
- Tune `max_linear_accel` and `max_angular_accel` to match vehicle dynamics

### For Different Environments

- Increase `obstacle_weight` in cluttered environments
- Adjust `clearance_threshold` based on the environment's openness
- Modify `stop_threshold` based on sensor accuracy and safety requirements

### For Different Behaviors

- Increase `forward_weight` to prioritize forward motion
- Adjust `heading_weight` to control turning behavior
- Modify `velocity_weight` to balance speed vs. safety

## Example Usage

```bash
# Launch with default parameters
ros2 launch sensor_fusion autonomous_navigation.launch.py

# Launch with custom parameters for a larger vehicle
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  robot_radius:=1.5 \
  max_linear_velocity:=3.0 \
  obstacle_weight:=3.0 \
  clearance_threshold:=2.5

# Launch with custom parameters for a more cautious behavior
ros2 launch sensor_fusion autonomous_navigation.launch.py \
  max_linear_velocity:=2.0 \
  obstacle_weight:=4.0 \
  clearance_threshold:=3.0 \
  stop_threshold:=1.0
```
