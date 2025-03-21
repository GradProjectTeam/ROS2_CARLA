# Enhanced LiDAR Cluster Visualization

This ROS2 node provides advanced visualization of LiDAR point cloud clusters with multiple display options and real-time updates.

## Features

### Visualization Elements

- **Point Cloud**: Raw point cloud data (colored by cluster)
- **Individual Points**: Spheres for each point (configurable)
- **Cluster Centers**: Larger spheres indicating the center of each cluster
- **Bounding Boxes**: 3D wireframe boxes showing the dimensions of each cluster
- **Convex Hulls**: 2D convex hull outlines showing the footprint of each cluster
- **Information Labels**: Text markers showing cluster ID, point count, dimensions, and volume

### Real-Time Features

- **Live Updates**: Smooth animation with configurable update frequency
- **Fading Effects**: Clusters fade out gracefully when they disappear from view
- **Multi-Threaded**: Separate threads for data reception and visualization
- **Persistent Clusters**: Clusters remain visible briefly after disappearing for smoother transitions
- **Performance Monitoring**: Real-time statistics on visualization rate and data throughput

### Visualization Topics

- `/lidar/points`: Raw point cloud data (All points as a single PointCloud2)
- `/lidar/markers`: Points, centers, and labels
- `/lidar/hulls`: Bounding boxes and convex hulls

### Performance Features

- **Multi-Threaded Processing**: Separate threads for data reception and visualization
- **Efficient Marker Rendering**: Using POINTS type for large clusters
- **Configurable Update Rate**: Adjust visualization frequency to match system capabilities
- **Reduced CPU Usage**: Non-blocking socket operations with timeouts
- **Memory Management**: Automatic cleanup of stale clusters
- **Performance Statistics**: Real-time reporting of points/sec, clusters/sec, and visualization Hz

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tcp_ip` | "127.0.0.1" | IP address of the TCP server |
| `tcp_port` | 12350 | Port number of the TCP server |
| `point_size` | 2.0 | Size of individual point markers |
| `center_size` | 3.0 | Size of cluster center markers |
| `use_convex_hull` | true | Whether to display 2D convex hulls |
| `use_point_markers` | true | Whether to display individual point markers |
| `use_cluster_stats` | true | Whether to display detailed cluster stats |
| `verbose_logging` | false | Whether to enable verbose logging |
| `update_frequency` | 30.0 | Visualization update frequency in Hz |
| `fade_out_time` | 0.5 | Time in seconds for markers to fade out |
| `fade_out_clusters` | true | Whether to enable fading out of clusters |
| `live_animation` | true | Whether to enable smooth animation effects |
| `socket_timeout` | 0.1 | Socket timeout in seconds for non-blocking operations |

## Launch Example

Create a launch file to run the node with custom parameters:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('update_frequency', default_value='30.0'),
        DeclareLaunchArgument('fade_out_time', default_value='0.5'),
        
        Node(
            package='carla_sensors',
            executable='lidar_listener_clusters.py',
            name='lidar_cluster_visualizer',
            parameters=[{
                'tcp_ip': '127.0.0.1',
                'tcp_port': 12350,
                'point_size': 1.5,
                'update_frequency': float(LaunchConfiguration('update_frequency')),
                'fade_out_time': float(LaunchConfiguration('fade_out_time')),
                'use_convex_hull': True,
                'use_point_markers': False,  # Disable individual points for better performance
                'use_cluster_stats': True,
                'fade_out_clusters': True,
                'live_animation': True,
                'verbose_logging': False,
            }],
            output='screen'
        )
    ])
```

## Running

To launch with default settings:
```bash
ros2 run carla_sensors lidar_listener_clusters.py
```

To launch with custom update frequency:
```bash
ros2 launch carla_sensors lidar_cluster_visualizer.launch.py update_frequency:=60.0
```

## RViz Configuration

For best results in RViz, add the following displays:

1. **PointCloud2**: Subscribe to `/lidar/points`
2. **MarkerArray**: Subscribe to `/lidar/markers`
3. **MarkerArray**: Subscribe to `/lidar/hulls`

Make sure to set the "Fixed Frame" in RViz to "map".

## Advanced Usage

### Performance Optimization

For large point clouds, consider:
- Setting `use_point_markers` to `false` to disable individual point spheres
- Reducing `update_frequency` on slower systems
- Increasing `socket_timeout` if network connectivity is spotty
- Adjusting `fade_out_time` based on expected cluster persistence

### Customizing Animation

- Set `fade_out_clusters` to `true` for smoother transitions when clusters disappear
- Adjust `fade_out_time` to control how long clusters remain visible after disappearing
- Set `live_animation` to `false` if you prefer discrete updates over continuous animation

### Customizing Colors

The visualization uses a procedural color generation algorithm that creates visually distinct colors for each cluster. The colors are generated using the golden ratio in the HSV color space, ensuring good contrast between adjacent clusters.

## Troubleshooting

If you don't see any markers:
1. Check that the TCP server is running and accessible
2. Ensure RViz is using the correct frame ID ("map")
3. Try increasing the point size parameters
4. Check that the topics are being published using `ros2 topic list` and `ros2 topic echo`
5. Check the node logs with `ros2 topic echo /rosout` for error messages

If visualization is jerky or slow:
1. Lower the `update_frequency` parameter
2. Disable `use_point_markers` for better performance
3. Disable `use_convex_hull` if not needed
4. Run `top` or `htop` to check CPU usage 