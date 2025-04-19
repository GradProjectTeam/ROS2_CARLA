# ROS2 Navigation Stack

This navigation stack provides path planning and control capabilities for mobile robots based on sensor fusion data. It integrates IMU and LiDAR data to build a real-time map and navigate through it.

## Architecture

The navigation stack consists of the following components:

1. **Sensor Processing and Fusion**
   - IMU processing and visualization
   - LiDAR point cloud processing
   - IMU-LiDAR fusion for improved localization

2. **Mapping**
   - Real-time occupancy grid mapping
   - Fast map updates for dynamic environments

3. **Path Planning**
   - Hybrid A* global path planning
   - Frenet path smoothing for trajectory optimization
   - Model Predictive Control (MPC) for trajectory generation
   - Dynamic Window Approach (DWA) for local planning and obstacle avoidance

## Available Nodes

### Sensor Fusion Nodes
- `imu_euler_visualizer`: Visualizes IMU data in Euler angles
- `lidar_listener_clusters_2`: Processes LiDAR data and extracts clusters
- `lidar_realtime_mapper`: Creates a real-time occupancy grid map from LiDAR data
- `imu_lidar_yaw_fusion`: Fuses IMU and LiDAR data for improved orientation estimation

### Planning Nodes
- `hybrid_astar_planner`: Global path planning using Hybrid A* algorithm
- `frenet_path_smoother`: Smoothes paths using Frenet coordinate transformation
- `mpc_planner`: Generates optimal trajectories using Model Predictive Control
- `dwa_local_planner`: Local planning and obstacle avoidance using Dynamic Window Approach

## Launch Files

- `fast_imu_lidar_fusion.launch.py`: Launches only the sensor fusion stack
- `navigation_stack.launch.py`: Launches the complete navigation stack including path planning

## How to Use

### Prerequisites

Make sure you have the required dependencies installed:

```bash
pip install numpy scipy cvxpy
```

### Running the Navigation Stack

1. Build the package:
```bash
cd ~/GP/ROS2
colcon build --packages-select sensor_fusion
source install/setup.bash
```

2. Launch the navigation stack:
```bash
ros2 launch sensor_fusion navigation_stack.launch.py
```

3. Set a goal in RViz:
   - Select the "2D Goal Pose" tool in RViz
   - Click and drag on the map to set a goal position and orientation
   - The path planners will generate a path from the current position to the goal

### Configuration

You can customize the navigation stack by modifying the parameters in the launch file:

- **Map Parameters**: Change resolution, size, and update rate
- **Path Planning Parameters**: Adjust planning rates, obstacle thresholds, and control parameters
- **Sensor Fusion Parameters**: Modify fusion weights and filter sizes

## Topic Structure

### Input Topics
- `/imu/data`: IMU sensor data
- `/lidar/points`: LiDAR point cloud data
- `/goal_pose`: Goal position for navigation (set by RViz)
- `/current_pose`: Current position of the robot (from sensor fusion)

### Output Topics
- `/realtime_map`: Occupancy grid map from sensor fusion
- `/hybrid_astar_path`: Raw path from Hybrid A* planner
- `/smooth_path`: Smoothed path from Frenet smoother
- `/planned_trajectory`: Trajectory from MPC planner
- `/dwa_trajectories`: Visualization of DWA trajectories
- `/cmd_vel`: Velocity commands for robot control

## Customization

You can customize the behavior of the navigation stack by modifying parameters in the launch file. For example:

- Change the map resolution for different performance/accuracy tradeoffs
- Adjust planning rates to match your hardware capabilities
- Modify obstacle thresholds for different environments
- Tune planning weights for different robot behaviors

## Troubleshooting

- **Map not updating**: Check that LiDAR data is being published and that the transform tree is correctly set up
- **Path planning failures**: Adjust obstacle thresholds or increase planning timeouts
- **Control issues**: Tune the MPC/DWA parameters for your specific robot dynamics
- **Performance issues**: Lower the planning rates or process fewer LiDAR points 