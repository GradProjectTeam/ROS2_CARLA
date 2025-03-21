# Sensor Fusion System for ROS2 Navigation

This package provides a complete sensor fusion pipeline for converting radar and LiDAR data into costmaps suitable for use with the ROS2 Navigation2 (Nav2) stack. It includes real-time processing of sensor data, conversion to costmaps, and intelligent fusion based on the characteristics of each sensor.

## Features

- **Enhanced Radar Visualization**: Receive TCP data from the Radar_Processing C++ application and visualize it with semantic information
- **LiDAR Cluster Processing**: Receive TCP data from LiDAR clustering application and visualize clusters
- **Radar Costmap Creation**: Convert radar markers to a costmap with special handling for dynamic objects
- **LiDAR Costmap Creation**: Process LiDAR point clouds into a static obstacle costmap
- **Sensor Fusion**: Intelligently combine radar and LiDAR data, prioritizing radar for dynamic objects and LiDAR for static objects
- **Nav2 Integration**: Output costmap in a format compatible with the Nav2 navigation stack
- **RViz Visualization**: Pre-configured RViz setup to visualize all costmap layers

## System Architecture

```
┌───────────────────┐        ┌───────────────────┐
│ Radar_Processing  │        │  LiDAR Clustering │
│    (C++ app)      │        │    (C++ app)      │
└─────────┬─────────┘        └─────────┬─────────┘
          │ TCP (Port 12348)           │ TCP (Port 12350)
          ▼                            ▼
┌───────────────────┐        ┌───────────────────┐
│ enhanced_radar_   │        │ lidar_listener_   │
│ visualizer        │        │ clusters          │
└─────────┬─────────┘        └─────────┬─────────┘
          │                            │
          ▼                            ▼
┌───────────────────┐        ┌───────────────────┐
│ radar_costmap_    │        │ lidar_costmap_    │
│ creator           │        │ creator           │
└─────────┬─────────┘        └─────────┬─────────┘
          │                            │
          └────────────┬───────────────┘
                       │
                       ▼
           ┌───────────────────────┐
           │ sensor_fusion_costmap │
           └───────────┬───────────┘
                       │
                       ▼
           ┌───────────────────────┐
           │ Nav2 Navigation Stack │
           └───────────────────────┘
```

## Components

1. **enhanced_radar_visualizer.py**: Connects to the C++ Radar_Processing application via TCP/IP, receives radar data, and visualizes it as markers in RViz.

2. **lidar_listener_clusters.py**: Connects to the LiDAR Clustering application via TCP/IP, receives clustered LiDAR data, and publishes point clouds and markers.

3. **radar_costmap_creator.py**: Subscribes to radar visualization markers from enhanced_radar_visualizer and converts them to a costmap layer. Assigns higher costs to objects with higher velocities.

4. **lidar_costmap_creator.py**: Processes LiDAR point clouds from lidar_listener_clusters, filters ground and noise, and creates a costmap representing static obstacles.

5. **sensor_fusion_costmap.py**: Combines radar and LiDAR costmaps with intelligent weighting based on the characteristics of the data (dynamic vs. static objects).

## Prerequisites

- ROS2 Humble or newer
- Nav2 package installed
- Python 3.8 or newer
- NumPy
- Radar_Processing C++ application running
- LiDAR Clustering C++ application running

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/carla_sensors.git
   ```

2. Make the Python scripts executable:
   ```bash
   cd ~/ros2_ws/src/carla_sensors/carla_sensors
   chmod +x enhanced_radar_visualizer.py
   chmod +x lidar_listener_clusters.py
   chmod +x radar_costmap_creator.py
   chmod +x lidar_costmap_creator.py
   chmod +x sensor_fusion_costmap.py
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select carla_sensors
   source install/setup.bash
   ```

## Usage

### Starting the Sensor Fusion Pipeline

Make sure both the Radar_Processing and LiDAR Clustering C++ applications are running first, then launch the sensor fusion system:

```bash
ros2 launch carla_sensors sensor_fusion.launch.py
```

### Customizing the Map Parameters

You can customize the costmap parameters by passing them as arguments to the launch file:

```bash
ros2 launch carla_sensors sensor_fusion.launch.py map_resolution:=0.05 map_width:=2000 map_height:=2000 map_origin_x:=-100.0 map_origin_y:=-100.0
```

### Running Individual Components

You can also run each component separately for debugging or development:

```bash
# Run the data input nodes
ros2 run carla_sensors enhanced_radar_visualizer.py
ros2 run carla_sensors lidar_listener_clusters.py

# Run the costmap creator nodes
ros2 run carla_sensors radar_costmap_creator.py
ros2 run carla_sensors lidar_costmap_creator.py

# Run the sensor fusion node
ros2 run carla_sensors sensor_fusion_costmap.py
```

## Integration with Nav2

To use the fused costmap with Nav2, add the following to your Nav2 costmap parameters YAML file:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "fused_sensor_layer"]
      
      fused_sensor_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: "/global_costmap/fused_sensor_layer"
        subscribe_to_updates: true
```

## Visualizing Results

The included RViz configuration automatically displays the radar markers, radar costmap, LiDAR costmap, and fused costmap. You can also view them separately by adding the appropriate displays in RViz.

## Troubleshooting

### "Address already in use" error
- The default TCP ports (12348 for radar, 12350 for LiDAR) might already be in use
- Restart the C++ applications with different ports
- Update the ports in the launch file parameters

### No radar data appearing
- Check if the Radar_Processing application is running
- Verify that the TCP connection is established (check logs)
- Ensure the TCP port matches between the C++ app and the visualizer

### No LiDAR data appearing
- Check if the LiDAR Clustering application is running
- Verify that the TCP connection is established (check logs)
- Ensure the TCP port matches between the C++ app and the listener

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

# Sensor Fusion for CARLA-ROS2 Integration

This README provides information about the sensor fusion components integrated with CARLA simulation in ROS2.

## Overview

This sensor fusion system integrates radar and LiDAR data from CARLA to create a fused costmap for navigation purposes. The system includes several nodes:

1. **Enhanced Radar Visualizer** - Processes radar data into visualization markers
2. **LiDAR Listener Clusters** - Processes LiDAR data into point clusters
3. **IMU Listener** - Receives IMU data for orientation
4. **Radar Costmap Creator** - Converts radar data into costmap layer
5. **LiDAR Costmap Creator** - Converts LiDAR data into costmap layer
6. **Sensor Fusion Costmap** - Fuses radar and LiDAR costmaps with smart weighting

## Testing Individual Sensors

For better testing and debugging, we provide individual launch files for each sensor:

### Testing LiDAR Only
```
ros2 launch carla_sensors lidar_test.launch.py
```
This will start the LiDAR listener and costmap creator with specific visualization in RViz.

### Testing Radar Only
```
ros2 launch carla_sensors radar_test.launch.py
```
This will start the radar visualizer and costmap creator with specific visualization in RViz.

### Testing IMU Only
```
ros2 launch carla_sensors imu_test.launch.py
```
This will start the IMU listener with specific visualization in RViz.

## Running the Full Sensor Fusion System

To start the full sensor fusion system:

```
ros2 launch carla_sensors sensor_fusion.launch.py
```

## Required CARLA Script Modifications

The CARLA script needs to send specific data formats over TCP:

1. **Radar Data** (Port 12347): Each point as 4 floats (altitude, azimuth, depth, velocity)
2. **LiDAR Data** (Port 12349): Each point as 3 floats (x, y, z)
3. **IMU Data** (Port 12350): 7 floats pack format (ax, ay, az, gx, gy, gz, compass)

### IMU Data Update Required
In the CARLA script (three_sensors_with_pygame_4_2.py), update the IMU data packing to include roll, pitch, and yaw:

```python
# Current implementation:
data = struct.pack('fffffff', 
                 imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,
                 imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,
                 imu_data.compass)

# Update to:
data = struct.pack('ffffffffff', 
                 imu_data.accelerometer.x, imu_data.accelerometer.y, imu_data.accelerometer.z,
                 imu_data.gyroscope.x, imu_data.gyroscope.y, imu_data.gyroscope.z,
                 imu_data.compass,
                 imu_data.transform.rotation.roll,  # Add roll
                 imu_data.transform.rotation.pitch, # Add pitch
                 imu_data.transform.rotation.yaw)   # Add yaw
```

## Parameter Configuration

You can adjust various parameters for each node through the launch file parameters, including:

- Map resolution, width, height, and origin
- TCP connection settings
- Visualization settings
- Costmap fusion weights
- Data freshness thresholds

## Troubleshooting

Common issues and solutions:

1. **TCP Connection Errors**: Ensure the CARLA simulation is running and the TCP ports are correctly set
2. **Data Too Old Warnings**: These occur when sensors aren't publishing fresh data
3. **Visualization Issues**: Check if the correct RViz configuration file is being used 