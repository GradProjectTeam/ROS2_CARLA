# Enhanced Radar Visualizer

This is an improved Python ROS2 node for visualizing the radar data clusters processed by the Radar Processing C++ application. It connects to the TCP output port of the Radar Processing application (default port: 12348) and displays the clustering results in RViz.

## Features

- **Robust TCP Connection Handling**: Automatically reconnects if the connection is lost or if the server isn't available on startup
- **Enhanced Visualization**:
  - Colored cluster points with size based on velocity
  - Convex hull visualization around clusters
  - Velocity vectors showing movement direction and speed
  - Detailed text labels with cluster statistics and classification
- **Comprehensive Statistics**: Tracks data points, clusters, connection status, and errors
- **Error Handling**: Gracefully handles connection issues, data corruption, and unexpected failures
- **Configurable Parameters**: Adjust visualization, connection settings, and behavior through ROS parameters

## Installation

1. Place the `enhanced_radar_visualizer.py` file in your ROS2 workspace
2. Make it executable:
   ```bash
   chmod +x enhanced_radar_visualizer.py
   ```
3. Build your workspace:
   ```bash
   colcon build --packages-select YOUR_PACKAGE_NAME
   source install/setup.bash
   ```

## Usage

Run the Radar Processing application first, then start the visualizer:

```bash
ros2 run YOUR_PACKAGE_NAME enhanced_radar_visualizer.py
```

### Configure Parameters (Optional)

You can configure the node using ROS2 parameters:

```bash
ros2 run YOUR_PACKAGE_NAME enhanced_radar_visualizer.py --ros-args -p tcp_ip:=127.0.0.1 -p tcp_port:=12348 -p marker_lifetime:=0.5
```

Available parameters:
- `tcp_ip`: IP address of the radar processing server (default: "127.0.0.1")
- `tcp_port`: TCP port to connect to (default: 12348)
- `reconnect_interval`: Time in seconds between connection attempts (default: 5.0)
- `marker_lifetime`: Duration in seconds that visualization markers remain visible (default: 0.5)
- `max_buffer_size`: Maximum size in bytes for the data buffer (default: 1048576)

## Visualization in RViz

To view the radar data:

1. Launch RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. Add a "MarkerArray" display:
   - Click "Add" in RViz
   - Select "MarkerArray"
   - Set the topic to "/radar/markers"

3. Make sure your fixed frame is set to "map" (or adjust the frame_id in the code if needed)

## Troubleshooting

- **Connection Issues**: Verify the C++ Radar Processing application is running and the port is correct
- **Visualization Problems**: Check that RViz is configured correctly with the right topic and frame
- **Data Parsing Errors**: If you see parse error messages, the data format may have changed

## Advanced Usage

### Building a Custom Launcher

Create a launch file to start both the visualizer and RViz:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='enhanced_radar_visualizer.py',
            name='enhanced_radar_visualizer',
            output='screen',
            parameters=[
                {'tcp_ip': '127.0.0.1'},
                {'tcp_port': 12348},
                {'marker_lifetime': 0.5}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'path/to/your/config.rviz']
        )
    ])
```

### Customizing Visualization

You can modify the code to customize the visualization:
- Change colors and sizes in the `process_complete_message` method
- Add additional marker types for different visualization techniques
- Modify the classification logic to match your specific requirements

## License

This software is licensed under the MIT License. 