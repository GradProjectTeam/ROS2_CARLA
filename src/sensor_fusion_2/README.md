# Sensor Fusion TCP Package

This ROS2 package provides nodes for receiving IMU, radar, and LiDAR data from TCP sources and publishing them as ROS2 topics.

## Features

- IMU TCP node that reads IMU data and publishes to `imu/data` topic and TF transforms
- Radar TCP node that reads radar data and publishes to `/radar/points` and `/radar/markers` topics
- LiDAR TCP node that reads LiDAR data and publishes to `/lidar/points`, `/lidar/markers`, `/lidar/hulls`, and `/lidar/cubes` topics
- Launch file for easily starting all nodes
- Selective activation of sensors through launch file parameters

## TCP Data Formats

### IMU Data Format
IMU data is expected in the following binary format:
- 10 32-bit floating point values (40 bytes per packet)
- Values: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, compass, roll, pitch, yaw

### Radar Data Format
Radar data is expected in the following binary format:
- Number of clusters (32-bit unsigned int)
- For each cluster:
  - Number of points in the cluster (32-bit unsigned int)
  - For each point:
    - altitude, azimuth, depth, velocity (4 32-bit floats, 16 bytes per point)

### LiDAR Data Format
LiDAR data is expected in the following binary format:
- Number of clusters (32-bit unsigned int)
- For each cluster:
  - Number of points in the cluster (32-bit unsigned int)
  - For each point:
    - x, y, z, intensity (4 32-bit floats, 16 bytes per point)

## How to Build

```bash
cd ~/GP/ROS2
colcon build --packages-select sensor_fusion_2
source install/setup.bash
```

## How to Run

### Using Launch File

```bash
# Run with default settings (all sensors enabled, localhost)
ros2 launch sensor_fusion_2 tcp_sensors.launch.py

# Run with custom TCP IP (when sensors are on a different machine)
ros2 launch sensor_fusion_2 tcp_sensors.launch.py tcp_ip:=192.168.1.100

# Selectively enable or disable specific sensors
ros2 launch sensor_fusion_2 tcp_sensors.launch.py enable_imu:=true enable_radar:=false enable_lidar:=true

# Run just one sensor (e.g., only LiDAR)
ros2 launch sensor_fusion_2 tcp_sensors.launch.py enable_imu:=false enable_radar:=false enable_lidar:=true
```

### Running Individual Nodes

```bash
# Run the IMU TCP node
ros2 run sensor_fusion_2 imu_tcp_node --ros-args -p tcp_ip:=127.0.0.1 -p tcp_port:=12345

# Run the radar TCP node
ros2 run sensor_fusion_2 radar_tcp_node --ros-args -p tcp_ip:=127.0.0.1 -p tcp_port:=12348

# Run the LiDAR TCP node
ros2 run sensor_fusion_2 lidar_tcp_node --ros-args -p tcp_ip:=127.0.0.1 -p tcp_port:=12350
```

## Launch Parameters

- `tcp_ip` (string, default: "127.0.0.1"): IP address for all TCP connections
- `enable_imu` (bool, default: true): Enable or disable the IMU sensor node
- `enable_radar` (bool, default: true): Enable or disable the radar sensor node
- `enable_lidar` (bool, default: true): Enable or disable the LiDAR sensor node

## Node Parameters

### IMU TCP Node
- `tcp_ip` (string, default: "127.0.0.1"): IP address of the IMU TCP server
- `tcp_port` (int, default: 12345): Port number of the IMU TCP server
- `frame_id` (string, default: "imu_link"): TF frame ID for IMU data
- `filter_window_size` (int, default: 5): Size of moving average filter window

### Radar TCP Node
- `tcp_ip` (string, default: "127.0.0.1"): IP address of the radar TCP server
- `tcp_port` (int, default: 12348): Port number of the radar TCP server
- `frame_id` (string, default: "radar_link"): TF frame ID for radar data

### LiDAR TCP Node
- `tcp_ip` (string, default: "127.0.0.1"): IP address of the LiDAR TCP server
- `tcp_port` (int, default: 12350): Port number of the LiDAR TCP server
- `frame_id` (string, default: "lidar_link"): TF frame ID for LiDAR data
- `point_size` (float, default: 2.0): Size of point markers
- `center_size` (float, default: 3.0): Size of centroid markers
- `use_convex_hull` (bool, default: true): Whether to publish convex hull markers
- `use_point_markers` (bool, default: true): Whether to publish point markers
- `filter_vehicle_points` (bool, default: false): Whether to filter out points that hit the vehicle
- `vehicle_length`, `vehicle_width`, `vehicle_height` (float): Vehicle dimensions for filtering
- `vehicle_x_offset`, `vehicle_y_offset` (float): Vehicle position offsets for filtering 