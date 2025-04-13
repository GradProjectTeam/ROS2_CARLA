# Permanent Map Node for ROS2 Sensor Fusion

This document describes how to use the `lidar_permanent_mapper` node in the sensor_fusion package.

## Overview

The Permanent Map node creates, maintains, and saves a persistent occupancy grid map based on LiDAR data. Unlike the standard costmap which is dynamically updated for local navigation, the permanent map builds a global representation of the environment that persists over time. This map can be saved to disk and reloaded for future use.

## Features

- Bayesian occupancy grid mapping for persistent environment modeling
- Automatic saving of the map at regular intervals
- Manual save/load through ROS2 services
- Visualization of both binary and probabilistic maps
- Configurable map size, resolution, and processing parameters

## Usage

### Prerequisites

Make sure you have the sensor_fusion package built and installed:

```bash
cd ~/GP/ROS2
colcon build --packages-select sensor_fusion
source install/setup.bash
```

### Launching with LiDAR

The permanent mapper is designed to work alongside existing LiDAR nodes. There are two main ways to use it:

#### 1. Launch the permanent mapper by itself

```bash
ros2 launch sensor_fusion permanent_map.launch.py
```

This will start only the permanent mapper node with RViz visualization. Make sure you're also running a LiDAR node that publishes to `/lidar/points`.

#### 2. Launch together with LiDAR test

```bash
# First launch the LiDAR test
ros2 launch sensor_fusion lidar_test.launch.py

# Then in another terminal
ros2 launch sensor_fusion permanent_map.launch.py
```

### Map Services

Once running, the node provides two services:

1. Save the current map manually:
   ```bash
   ros2 service call /save_permanent_map std_srvs/srv/Trigger
   ```

2. Load the most recent saved map:
   ```bash
   ros2 service call /load_permanent_map std_srvs/srv/Trigger
   ```

### RViz Visualization

The permanent map visualization shows two versions of the map:

1. **PermanentMap** - Binary representation with three states:
   - Free space (white)
   - Occupied (black)
   - Unknown (gray)

2. **PermanentMapVisualization** - Probabilistic representation with grayscale values (0-100)

### Configuration Parameters

The node is highly configurable. Key parameters include:

| Parameter | Default | Description |
|-----------|---------|-------------|
| map_resolution | 0.2 | Resolution of the map in meters per cell |
| map_width_meters | 200.0 | Width of the map in meters |
| map_height_meters | 200.0 | Height of the map in meters |
| enable_auto_save | true | Whether to automatically save the map |
| auto_save_interval | 60.0 | Seconds between auto-saves |
| hit_weight | 0.9 | Probability weight for occupied cells (0.0-1.0) |
| miss_weight | 0.3 | Probability weight for free cells (0.0-1.0) |
| count_threshold | 10.0 | Confidence threshold for "permanent" status |

You can customize these parameters when launching:

```bash
ros2 launch sensor_fusion permanent_map.launch.py map_resolution:=0.1 map_width_meters:=100.0
```

### Saved Maps

Maps are saved to the directory specified by the `map_save_dir` parameter (default: `/home/mostafa/GP/ROS2/maps`). Each saved map includes:

- `.map` file (Python pickle format containing the full map data)
- `.png` file (Visualization of the map for quick reference)

## Implementation Details

The permanent mapper uses:

- Bayesian updates with log-odds representation for stable probability estimation
- Bresenham's line algorithm for ray tracing
- Probabilistic hit/miss model for managing uncertainty
- Confidence tracking for each cell

## Troubleshooting

If the permanent map is not updating:

1. Verify that the LiDAR is publishing data to `/lidar/points`
   ```bash
   ros2 topic echo /lidar/points --no-arr
   ```

2. Check TF transformations between frames
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ```

3. Ensure the map directory exists and is writable:
   ```bash
   mkdir -p /home/mostafa/GP/ROS2/maps
   chmod 755 /home/mostafa/GP/ROS2/maps
   ```

## Integration with Navigation

The permanent map can be used with Nav2 by remapping the topic:

```bash
ros2 launch nav2_bringup nav2_bringup.launch.py map:=/permanent_map
```

This provides a stable, persistent map for global path planning while the dynamic costmap layers handle real-time obstacle avoidance. 