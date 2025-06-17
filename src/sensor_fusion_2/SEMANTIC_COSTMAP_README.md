# Semantic Costmap Visualizer

## Overview

The Semantic Costmap Visualizer is an advanced visualization tool for LiDAR and radar sensor data in ROS2. It provides a multi-layered semantic costmap that classifies objects in the environment into different categories (ground, obstacles, vegetation, buildings, dynamic objects) and visualizes them in separate layers.

This tool enhances the standard occupancy grid visualization by adding:

1. **Semantic Classification**: Automatically categorizes objects based on their physical properties
2. **Multi-Layer Visualization**: Separate costmap layers for each semantic category
3. **Temporal Filtering**: Decay of old data with different rates for different object types
4. **Motion Prediction**: Prediction of dynamic object movement
5. **3D Visualization**: Height-based visualization of objects
6. **Interactive Labels**: Text labels showing object classifications

## Features

### Semantic Classification

Objects are classified into the following categories:

- **Ground**: Flat surfaces with low height (< 0.3m)
- **Obstacles**: General obstacles that don't fit other categories
- **Vegetation**: Tall, thin objects with high height-to-width ratio
- **Buildings**: Large, wide objects with significant height
- **Dynamic**: Objects with detected motion (especially from radar data)

### Multi-Layer Costmap

The visualizer creates separate occupancy grid layers for each semantic category, plus a combined layer:

- `/semantic_costmap/ground`: Ground-level surfaces
- `/semantic_costmap/obstacle`: General obstacles
- `/semantic_costmap/vegetation`: Vegetation objects
- `/semantic_costmap/building`: Building structures
- `/semantic_costmap/dynamic`: Moving objects
- `/semantic_costmap/combined`: Combined costmap with weighted layers

### Temporal Filtering

Different decay rates are applied to different object types:

- Ground and static objects: Slow decay
- Dynamic objects: Fast decay
- Buildings: Very slow decay

### Motion Prediction

For dynamic objects, the visualizer predicts future positions based on tracked velocity, creating a more accurate representation of moving objects.

## Installation

### Prerequisites

- ROS2 Humble or newer
- Python 3.8 or newer
- NumPy
- TF2 ROS

### Building the Package

```bash
cd ~/GP/ROS2
colcon build --packages-select sensor_fusion_2 --symlink-install
source install/setup.bash
```

## Usage

### Running the Complete Pipeline

To run the complete sensor processing pipeline with the semantic costmap visualizer:

```bash
ros2 launch sensor_fusion_2 semantic_costmap_complete.launch.py
```

This launch file starts:
1. LiDAR data processing nodes
2. Radar data processing nodes
3. TF tree setup
4. Semantic costmap visualizer
5. RViz with the semantic costmap configuration

### Running Only the Visualizer

If you already have sensor data being published, you can run just the visualizer:

```bash
ros2 launch sensor_fusion_2 semantic_costmap.launch.py
```

### Launch Arguments

The launch files accept several arguments to customize behavior:

```bash
# Example with custom parameters
ros2 launch sensor_fusion_2 semantic_costmap_complete.launch.py \
  map_resolution:=0.1 \
  map_width_meters:=80.0 \
  map_height_meters:=80.0 \
  publish_rate:=20.0 \
  enable_text_labels:=false
```

Common parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_resolution` | 0.2 | Resolution of the costmap in meters per cell |
| `map_width_meters` | 60.0 | Width of the costmap in meters |
| `map_height_meters` | 60.0 | Height of the costmap in meters |
| `publish_rate` | 10.0 | Rate at which to publish costmap layers (Hz) |
| `temporal_filtering` | true | Enable temporal filtering of costmap layers |
| `motion_prediction` | true | Enable motion prediction for dynamic objects |
| `enable_3d_visualization` | true | Enable 3D visualization of the costmap |
| `enable_text_labels` | true | Enable text labels for semantic categories |
| `lidar_tcp_ip` | 127.0.0.1 | IP address for LiDAR TCP connection |
| `lidar_tcp_port` | 12350 | Port for LiDAR TCP connection |
| `radar_tcp_ip` | 127.0.0.1 | IP address for radar TCP connection |
| `radar_tcp_port` | 12348 | Port for radar TCP connection |

## RViz Configuration

The package includes a pre-configured RViz setup (`semantic_costmap.rviz`) that visualizes:

1. All costmap layers (combined, ground, obstacle, vegetation, building, dynamic)
2. Semantic markers with color-coded classifications
3. Raw LiDAR and radar point clouds
4. LiDAR and radar clusters
5. TF tree visualization

## Node Details

### Semantic Costmap Visualizer Node

The main node (`semantic_costmap_visualizer`) subscribes to:

- `/lidar/points`: LiDAR point cloud
- `/lidar/cubes`: LiDAR clusters as markers
- `/radar/points`: Radar point cloud
- `/radar/clusters`: Radar clusters as markers

And publishes:

- `/semantic_costmap/{layer}`: Separate costmap layers
- `/semantic_costmap/markers`: Semantic classification markers

### Supporting Nodes

The complete pipeline includes:

- `lidar_listener_clusters_3`: Processes raw LiDAR data and creates clusters
- `radar_listener_clusters`: Processes raw radar data and creates clusters
- `radar_object_detector`: Detects and tracks objects from radar data
- `lidar_realtime_mapper`: Creates a real-time occupancy grid from LiDAR data
- `radar_map_generator`: Creates an occupancy grid from radar data

## Extending the Visualizer

### Adding New Semantic Categories

To add a new semantic category:

1. Add the new category to the `layers` dictionary in `initialize_costmap()`
2. Add the new category to the `layer_weights` dictionary
3. Update the `classify_cluster()` method to include the new category
4. Add visual properties for the new category in `publish_semantic_markers()`

### Customizing Visualization

The appearance of semantic markers can be customized by modifying the `semantic_classes` dictionary in the `publish_semantic_markers()` method.

## Troubleshooting

### Common Issues

1. **No data in costmap layers**: Ensure that LiDAR and radar data are being published on the expected topics.
2. **Misaligned sensors**: Check the TF tree configuration to ensure proper alignment between sensors.
3. **High CPU usage**: Reduce the publish rate or increase the map resolution.

### Debugging

Enable verbose logging by setting the `verbose_logging` parameter to `true` for the relevant nodes.

## License

This package is licensed under the Apache License 2.0.

## Contributors

- Mostafa Hendy (mostafahendy@std.mans.edu.eg)

## Acknowledgments

This work builds upon the sensor_fusion_2 package and integrates with the existing LiDAR and radar processing pipeline. 