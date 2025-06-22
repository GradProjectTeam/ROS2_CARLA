# Waypoint Visualization System for CARLA

This system enables the visualization of waypoints received from CARLA simulation in ROS2. It includes TCP communication, waypoint processing, and visualization in RViz.

## Overview

The waypoint visualization system consists of the following components:

1. **Waypoint Listener Node**: Connects to CARLA via TCP to receive waypoint data and publishes it as visualization markers.
2. **Waypoint Map Generator Node**: Subscribes to waypoint markers and generates a binary occupancy map for path planning.
3. **Launch File**: Configures and starts all necessary nodes with appropriate parameters.
4. **RViz Configuration**: Provides a visualization setup for viewing waypoints and maps.

## Prerequisites

- ROS2 (tested with Humble)
- CARLA Simulator with waypoint publishing enabled
- Python 3.8+
- NumPy

## Setup

1. Make sure the `sensor_fusion_2` package is built and sourced:

```bash
cd ~/GP/ROS2
colcon build --packages-select sensor_fusion_2
source install/setup.bash
```

2. Ensure CARLA is running and configured to publish waypoints on the specified TCP port (default: 12343).

## Usage

Launch the waypoint visualization system:

```bash
ros2 launch sensor_fusion_2 waypoint_visualization.launch.py
```

### Launch Parameters

The launch file accepts several parameters to customize its behavior:

- `use_sim_time`: Whether to use simulation time (default: false)
- `show_rviz`: Whether to launch RViz (default: true)
- `waypoint_tcp_ip`: IP address for TCP connection (default: 127.0.0.1)
- `waypoint_tcp_port`: Port for TCP connection (default: 12343)
- `map_resolution`: Resolution of the binary map in meters per cell (default: 0.5)
- `map_width_meters`: Width of the map in meters (default: 120.0)
- `map_height_meters`: Height of the map in meters (default: 120.0)
- `waypoint_marker_size`: Size of waypoint markers (default: 0.2)
- `waypoint_line_width`: Width of waypoint lines (default: 0.1)
- `binary_topic`: Topic for binary map (default: /waypoint_map/binary)

Example with custom parameters:

```bash
ros2 launch sensor_fusion_2 waypoint_visualization.launch.py waypoint_tcp_port:=12345 map_resolution:=0.2
```

## Waypoint Data Format

The waypoint data from CARLA is expected in the following format:

1. First 4 bytes: Number of waypoints (unsigned int, network byte order)
2. For each waypoint (24 bytes per waypoint):
   - x position (float, 4 bytes)
   - y position (float, 4 bytes)
   - z position (float, 4 bytes)
   - road_id (int, 4 bytes)
   - lane_id (int, 4 bytes)
   - lane_type (int, 4 bytes)

## Integration with CARLA

To enable waypoint publishing in CARLA:

1. Add the waypoint sensor to your CARLA client script
2. Configure the sensor to publish to the TCP port specified in the launch file
3. Set the update frequency as needed

Example CARLA Python code snippet:

```python
# In your CARLA client script
import socket
import struct

# Create TCP socket
waypoint_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
waypoint_socket.bind(('0.0.0.0', 12343))
waypoint_socket.listen(1)
client_socket, addr = waypoint_socket.accept()

# Get waypoints from CARLA
waypoints = world.get_map().get_waypoints_in_radius(vehicle_location, radius=50.0)

# Send waypoints
num_waypoints = len(waypoints)
client_socket.send(struct.pack('!I', num_waypoints))

for wp in waypoints:
    # Pack waypoint data (x, y, z, road_id, lane_id, lane_type)
    data = struct.pack('!fffiii', 
                      wp.transform.location.x,
                      wp.transform.location.y,
                      wp.transform.location.z,
                      wp.road_id,
                      wp.lane_id,
                      int(wp.lane_type))
    client_socket.send(data)
```

## Visualization

The system provides several visualization options in RViz:

1. **Waypoint Markers**: Shows waypoints as spheres with colors based on lane type
2. **Waypoint Lines**: Connects waypoints with lines
3. **Binary Map**: Shows the occupancy grid representation of waypoints

## Troubleshooting

If you encounter issues:

1. **Connection Problems**: Check that CARLA is running and the TCP port is correctly configured
2. **No Waypoints Visible**: Ensure waypoints are being published from CARLA
3. **TF Errors**: Verify that the TF tree is properly set up with the correct frame IDs

## License

Apache License 2.0 