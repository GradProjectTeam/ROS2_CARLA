# Lane Memory Capability

This feature enhances the lane mapping system by maintaining a memory of detected lanes, allowing the system to continue tracking lanes even when they are temporarily unavailable or poorly detected (such as with dashed lines or in poor lighting conditions).

## Features

- **Lane Memory**: The system stores previously detected lanes in memory when confidence is high
- **IMU-Based Rotation**: When the vehicle turns, the remembered lanes are rotated based on IMU heading data
- **Automatic Fallback**: The system automatically switches between using current detections and memory based on confidence
- **Configurable Memory**: Parameters allow tuning of memory timeout and confidence thresholds
- **Visualization**: Memory lanes are visualized in the 2D map with a different color to distinguish them from current detections

## How It Works

1. The system continuously processes lane detections from the camera
2. When lane detection confidence is high (above the threshold), lanes are stored in memory in map coordinates
3. When detection confidence drops (due to dashed lines, poor visibility, etc.), the system uses the stored memory lanes
4. As the vehicle moves, IMU heading data is used to transform the memory lanes to maintain their correct position
5. If memory becomes too old (beyond the timeout), the system will revert to using current detections even if poor

## Usage

Launch the system with the provided launch file:

```bash
ros2 launch sensor_fusion lane_mapper_with_memory.launch.py
```

### Configurable Parameters

You can adjust the following parameters when launching:

```bash
# Example with custom parameters
ros2 launch sensor_fusion lane_mapper_with_memory.launch.py \
  lane_memory_timeout:=10.0 \
  lane_confidence_threshold:=0.3 \
  imu_heading_topic:=/custom/heading/topic
```

Key parameters:

- `lane_memory_timeout`: How long (in seconds) to remember lanes when they are not detected (default: 5.0)
- `lane_confidence_threshold`: Confidence threshold below which memory is used (0.0-1.0, default: 0.5)
- `imu_heading_topic`: Topic providing vehicle heading data (default: /vehicle/heading)

## Visualization

The system publishes several visualization topics:

- `/map_lane_markers`: Lane markers in the map frame (both current and memory)
- `/lane_grid`: Occupancy grid representation of lanes

Memory lanes are displayed with a different color (orange-yellow) to distinguish them from current detections (bright yellow).

## Troubleshooting

- If lanes aren't being stored in memory, check that your confidence threshold isn't too high
- If memory lanes persist too long, reduce the memory timeout
- If memory lanes aren't rotating correctly with the vehicle, ensure the IMU heading topic is publishing correctly
- Enable debug logging for more detailed information about memory usage:
  ```bash
  ros2 run sensor_fusion lane_to_map_mapper --ros-args --log-level debug
  ``` 