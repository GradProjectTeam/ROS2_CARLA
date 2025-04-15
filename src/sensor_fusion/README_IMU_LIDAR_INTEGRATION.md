# IMU-LiDAR Integration with Costmaps and Clustering

This package provides a comprehensive integration of IMU data with LiDAR point clouds, including point cloud clustering and costmap generation for robotic navigation.

## Overview

The IMU-LiDAR integration system uses orientation data from an IMU sensor to properly transform LiDAR point clouds, enhancing accuracy for mapping, navigation, and obstacle detection. The system also includes LiDAR clustering for object detection and costmap generation for path planning.

Key features:
- Accurate orientation calculation from IMU data
- TF2 transform broadcasting to relate IMU and LiDAR frames
- LiDAR point cloud clustering for object detection
- Costmap generation for navigation planning
- Visualization of all components in RViz2

## Components

The system consists of the following main components:

1. **IMU-LiDAR Fusion Node (`imu_listener.py`)**
   - Processes IMU data and publishes TF transforms
   - Calculates orientation quaternions and rotation matrices
   - Provides coordinate frame transformations for LiDAR data

2. **LiDAR Clustering Node (`lidar_listener_clusters_2.py`)**
   - Segments point clouds into clusters representing objects
   - Applies ground removal and filtering
   - Publishes visual markers for detected objects

3. **LiDAR Costmap Creator (`lidar_costmap_creator.py`)**
   - Generates occupancy grid maps from LiDAR data
   - Uses IMU orientation for accurate placement
   - Creates cost-based representation for navigation planning

## Requirements

- ROS2 (Foxy or newer)
- Python 3.6 or newer
- RViz2 with IMU plugin
- Point Cloud Library (PCL)
- NumPy

## Launch File Parameters

The launch file (`imu_lidar_fusion.launch.py`) supports the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim_time` | bool | `true` | Use simulation time |
| `enable_imu` | bool | `true` | Enable the IMU-LiDAR fusion node |
| `enable_lidar` | bool | `true` | Enable LiDAR processing |
| `enable_clustering` | bool | `true` | Enable LiDAR clustering |
| `enable_costmap` | bool | `true` | Enable costmap generation |
| `enable_rviz` | bool | `true` | Enable RViz visualization |

You can disable any component you don't need by setting its parameter to `false`.

## Setup and Running

### Starting the Complete System

Launch the entire system with a single command:

```bash
cd /home/mostafa/GP/ROS2
source install/setup.bash
ros2 launch sensor_fusion imu_lidar_fusion.launch.py
```

### Starting with Specific Components

To run with only specific components (e.g., without costmap generation):

```bash
ros2 launch sensor_fusion imu_lidar_fusion.launch.py enable_costmap:=false
```

### Data Sources

1. **IMU Data**: The system expects IMU data from:
   - CARLA simulator: `/home/mostafa/ROS2andCarla/CARLA/Sensors/three_sensors_with_pygame_6.py`
   - C++ processor: `/home/mostafa/ROS2andCarla/CPP/IMU_Processing/rooster`
   - Data is received via TCP on port 12345

2. **LiDAR Data**: The system expects LiDAR point clouds on the topic:
   - `/lidar/points` (PointCloud2 message type)

## Frame Relationships

The system sets up the following coordinate frames:
- `world`: The global reference frame
- `imu_link`: The frame attached to the IMU
- `lidar_link`: The frame attached to the LiDAR (transformed by IMU orientation)

## Visualizing the System

RViz2 is configured to display:
- IMU orientation with axes
- LiDAR point clouds
- Detected clusters with markers
- Generated costmaps

## Customization

### Adjusting LiDAR Clustering Parameters

Modify the LiDAR clustering parameters in the launch file:

```python
lidar_clustering_node = Node(
    # ...
    parameters=[{
        'min_cluster_size': 5,        # Minimum points per cluster
        'max_cluster_size': 500,      # Maximum points per cluster
        'cluster_tolerance': 0.2,     # Clustering distance threshold
        'min_height': -1.5,           # Minimum height for points
        'max_height': 1.5,            # Maximum height for points
        'ground_removal_threshold': 0.2  # Ground plane detection threshold
    }]
)
```

### Adjusting Costmap Parameters

Modify the costmap generation parameters in the launch file:

```python
lidar_costmap_node = Node(
    # ...
    parameters=[{
        'cell_size': 0.25,           # Size of each grid cell (meters)
        'cell_inflation': 2,         # Inflation radius for obstacles
        'costmap_size': 100,         # Size of the costmap (cells)
        'use_clustering': True,      # Use cluster data for costmap
        # ...
    }]
)
```

### Changing the LiDAR Position Relative to IMU

If your LiDAR is mounted at a specific position relative to the IMU, adjust the transform in `imu_listener.py`:

```python
# In the publish_tf_transform method
t_lidar.transform.translation.x = 0.0  # Forward offset
t_lidar.transform.translation.y = 0.0  # Left/right offset
t_lidar.transform.translation.z = 0.2  # Up/down offset
```

## Debugging

### Verifying Transforms

Check if transforms are being published correctly:

```bash
ros2 run tf2_ros tf2_echo world imu_link
ros2 run tf2_ros tf2_echo imu_link lidar_link
```

### Visualizing the TF Tree

Generate a visual representation of the transform tree:

```bash
ros2 run tf2_tools view_frames.py
```

### Checking Topic Data

Monitor the data on key topics:

```bash
# IMU data
ros2 topic echo /imu/data

# LiDAR point cloud info
ros2 topic info /lidar/points

# Cluster markers info
ros2 topic info /lidar/markers

# Costmap info
ros2 topic info /lidar_costmap
```

## Common Issues and Solutions

1. **No IMU data**: 
   - Ensure CARLA and the C++ processor are running
   - Check TCP connection settings (IP and port)

2. **LiDAR point clouds not properly oriented**:
   - Verify TF transforms are being published
   - Check that the LiDAR node is using the correct frame_id

3. **No clusters detected**:
   - Adjust clustering parameters (tolerance, min/max size)
   - Verify point cloud density is sufficient

4. **Costmap not generating**:
   - Check that LiDAR data is available
   - Verify the world frame is properly set up

5. **RViz not showing all components**:
   - Ensure the fixed frame is set to "world"
   - Check that all display panels are enabled

## Extensions and Future Work

Potential extensions to this system include:
- Adding dynamic object tracking based on cluster movement
- Implementing a full navigation stack with path planning
- Integrating multiple LiDAR sensors with fusion
- Adding semantic segmentation for better object classification
- Implementing SLAM for simultaneous mapping and localization

## License

This project is licensed under the MIT License - see the LICENSE file for details. 