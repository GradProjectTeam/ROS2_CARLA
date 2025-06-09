# Fast IMU-LiDAR Fusion Map

## Overview

The Fast IMU-LiDAR Fusion Map is a high-performance mapping solution that combines IMU (Inertial Measurement Unit) and LiDAR (Light Detection and Ranging) data to create real-time maps optimized for speed and minimal memory usage. This implementation prioritizes fast updates and rapid clearing of old points to maintain a clean, current map representation of the environment.

The system is designed for applications where mapping speed is critical and older map data should be quickly removed instead of persisting. Maps are automatically saved to disk at configurable intervals.

## Key Features

- **High-Speed Mapping**: Optimized for rapid map updates and visualization
- **Aggressive Data Decay**: Quickly removes old map points for a fresh perspective
- **IMU-LiDAR Fusion**: Combines orientation data from IMU with point clouds from LiDAR
- **Automatic Map Saving**: Periodically saves maps to disk in configurable formats
- **Low Memory Footprint**: Smaller map sizes and optimized parameters reduce memory usage
- **Fast Transform Updates**: Higher frequency transform publications for smoother operation

## Usage

1. Make sure your ROS2 workspace is built and sourced:
   ```bash
   cd /home/mostafa/GP/ROS2
   source install/setup.bash
   ```

2. Launch the Fast IMU-LiDAR Fusion Map:
   ```bash
   ros2 launch sensor_fusion fast_imu_lidar_fusion.launch.py
   ```

3. To customize parameters, you can specify them on the command line:
   ```bash
   ros2 launch sensor_fusion fast_imu_lidar_fusion.launch.py map_resolution:=0.1 auto_save_interval:=3.0
   ```

4. Maps will be saved to `/home/mostafa/GP/ROS2/maps` with the prefix `fast_fusion_map_` followed by a timestamp.

## Parameters

### Map Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| map_resolution | 0.15 | Resolution of the map in meters per cell |
| map_width_meters | 30.0 | Width of the map in meters |
| map_height_meters | 30.0 | Height of the map in meters |
| center_on_vehicle | true | Keep map centered on vehicle position |

### Performance Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| publish_rate | 20.0 | Rate to publish map updates (Hz) |
| process_rate | 40.0 | Rate to process incoming data (Hz) |
| raycast_skip | 2 | Number of points to skip in processing (higher = faster) |
| max_points_to_process | 6000 | Maximum number of points processed per update |

### Fast Mapping Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| decay_rate | 0.98 | Rate at which old data decays (0-1, higher = faster decay) |
| temporal_memory | 0.2 | Duration to remember points (seconds) |
| enable_map_reset | true | Enable periodic complete map reset |
| map_reset_interval | 10.0 | Interval between map resets (seconds) |
| use_binary_map | true | Use binary (black/white) map output |
| obstacle_threshold | 0.65 | Threshold for marking cells as obstacles (0-1) |

### Bayesian Update Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| hit_weight | 1.0 | Weight applied to obstacle hits |
| miss_weight | 0.4 | Weight applied to free space detections |
| count_threshold | 0.15 | Threshold for binary map updates |

### Map Saving Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| enable_map_save | true | Enable saving map to disk |
| map_save_dir | /home/mostafa/GP/ROS2/maps | Directory to save maps |
| enable_auto_save | true | Enable automatic saving of map at intervals |
| auto_save_interval | 5.0 | Interval for auto-saving the map (seconds) |
| map_base_filename | fast_fusion_map | Base filename for saved maps |
| save_format | png | Format to save maps in (png, pgm, or both) |

## Comparison with Other Mapping Solutions

| Feature | Fast Fusion Map | Permanent Map | Standard Realtime Map |
|---------|----------------|---------------|------------------------|
| Update Speed | Very Fast | Slow | Moderate |
| Memory Usage | Low | High | Moderate |
| Map Persistence | Very Short | Permanent | Short |
| Data Decay | Very Aggressive | None | Moderate |
| Save Interval | 5 seconds | 60 seconds | 10 seconds |
| Primary Use Case | Immediate navigation | Long-term mapping | Standard navigation |

## Nodes and Components

The launch file starts the following nodes:

1. **Transform Publishers**: Four static transform publishers for the transform tree
2. **lidar_fast_fusion_mapper**: The main mapping node based on lidar_realtime_mapper
3. **imu_euler_visualizer**: Visualizes IMU orientation data
4. **lidar_cube_visualizer**: Displays LiDAR point clouds and clusters
5. **imu_lidar_yaw_fusion**: Fuses IMU and LiDAR data for improved orientation
6. **rviz2**: Visualization using the realtime_map.rviz configuration

## Troubleshooting

1. **Transform Errors**: If you see "Failed to get vehicle position" errors, wait a few seconds for the transform tree to establish.

2. **No Map Displayed**: Ensure that the IMU and LiDAR data are being received properly on the configured topics.

3. **Map Not Saving**: Verify that the maps directory exists and is writable.

4. **High CPU Usage**: Reduce the publish_rate and process_rate parameters for better performance on slower systems.

## Implementation Notes

- The transform tree establishes the relationships between world → map → base_link → imu_link → lidar_link
- Maps are published on the `/realtime_map` topic
- All nodes use a 1-second startup delay to ensure the transform tree is properly established
- Transforms are published at 100Hz for maximum responsiveness

## Known Limitations

- Aggressive memory settings mean this is not suitable for persistent mapping
- May not work well in environments requiring detailed historical data
- Optimized for speed at the expense of some map detail 