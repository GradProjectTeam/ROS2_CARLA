#!/bin/bash

# Source the ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start the radar visualization launch file
ros2 launch sensor_fusion radar_visualization.launch.py 