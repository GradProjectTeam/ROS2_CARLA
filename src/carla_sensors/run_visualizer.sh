#!/bin/bash

echo "===== Starting IMU Data Visualizer ====="
echo "This script will run the modified IMU listener that visualizes incoming IMU data"
echo "Ctrl+C to exit"
echo

# First, we'll make sure the Python script has the right permissions
if [ -f "build/carla_sensors/imu_listener.py" ]; then
    chmod +x build/carla_sensors/imu_listener.py
fi

# Set ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the IMU visualizer
ros2 run carla_sensors imu_listener

echo "Visualizer terminated." 