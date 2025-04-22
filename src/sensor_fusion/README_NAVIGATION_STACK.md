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

## Parameter Optimizations

The navigation stack has been optimized with the following parameter improvements:

### Core Parameter Improvements

1. **Transform Publishers**
   - Increased publish frequency to 100Hz for smoother transforms
   - Improved frame tree consistency

2. **NavigationTester Node**
   - Renamed to 'navigation_tester' for consistency
   - Increased yaw_offset parameter step size to 0.1 for easier configuration
   - Updated documentation to show correct node name for parameter setting

3. **LiDAR Real-time Mapper**
   - Increased publish_rate from 10.0Hz to 15.0Hz for smoother visualization
   - Increased process_rate from 20.0Hz to 30.0Hz for faster updates
   - Increased hit_weight from 0.8 to 0.9 for better obstacle detection
   - Adjusted decay_rate from 0.95 to 0.98 for more responsive mapping
   - Reduced temporal_memory from 0.5s to 0.3s for more up-to-date representation
   - Enabled binary map output for clearer visualization
   - Increased max_points_to_process from 6000 to 8000 for better map detail

### Planning Components Optimizations

1. **Hybrid A* Path Planner**
   - Reduced grid_size from 0.5m to 0.25m for more precise planning
   - Added max_iterations parameter (8000) for more thorough planning
   - Added motion_resolution parameter (9) for smoother steering options
   - Added angle_resolution parameter (36) for 10-degree heading discretization
   - Added heuristic_weight parameter (1.1) for faster, near-optimal planning

2. **Frenet Path Smoother**
   - Increased num_points from 100 to 150 for smoother trajectory generation

3. **MPC Planner**
   - Increased horizon from 10 to 15 for longer trajectory lookahead
   - Increased publish_rate from 10.0Hz to 15.0Hz for more responsive control

4. **DWA Local Planner**
   - Increased max_linear_velocity from 1.0 to 1.2 m/s for better performance
   - Increased linear_acceleration from 0.5 to 0.6 m/s² for smoother acceleration
   - Increased angular_acceleration from 1.0 to 1.2 rad/s² for more responsive turning
   - Increased predict_horizon from 1.0s to 1.5s for better obstacle avoidance
   - Adjusted obstacle_weight from 1.0 to 1.2 for stronger obstacle avoidance
   - Reduced heading_weight from 0.8 to 0.7 to reduce oscillation
   - Increased velocity_weight from 0.2 to 0.3 for smoother speed control
   - Increased safety_radius from 0.3m to 0.4m for safer navigation

### Timing Improvements

1. **System Startup**
   - Reduced mapper delay from 2.0s to 1.5s for faster mapping initialization
   - Reduced planning node startup delay from 3.0s to 2.0s for quicker navigation readiness
   - Decreased IMU reconnect interval from 3.0s to 2.0s for faster hardware reconnection

These optimizations collectively improve the navigation stack's performance in terms of:
- Faster and more precise path planning
- Smoother trajectory execution
- More responsive obstacle avoidance
- Better real-time map updates
- Easier parameter configuration 