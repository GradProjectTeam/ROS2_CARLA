#!/bin/bash

# Script to verify and fix TF tree issues before running navigation nodes

echo "Verifying TF tree configuration..."

# Function to check if transform exists
check_transform() {
    local source=$1
    local target=$2
    
    echo "Checking transform from ${source} to ${target}..."
    if ros2 run tf2_ros tf2_echo ${source} ${target} -t 0 > /dev/null 2>&1; then
        echo "✓ Transform ${source} -> ${target} exists."
        return 0
    else
        echo "✗ Transform ${source} -> ${target} NOT FOUND!"
        return 1
    fi
}

# Function to publish a static transform if missing
publish_transform() {
    local source=$1
    local target=$2
    local x=${3:-0.0}
    local y=${4:-0.0}
    local z=${5:-0.0}
    local roll=${6:-0.0}
    local pitch=${7:-0.0}
    local yaw=${8:-0.0}
    
    echo "Publishing static transform from ${source} to ${target}..."
    ros2 run tf2_ros static_transform_publisher ${x} ${y} ${z} ${roll} ${pitch} ${yaw} ${source} ${target} &
    PID=$!
    echo "Transform publisher started with PID: $PID"
    return $PID
}

# Wait for ROS2 to be ready
sleep 2

# Check critical transforms
check_transform "world" "map" || publish_transform "world" "map"
sleep 1
check_transform "map" "base_link" || publish_transform "map" "base_link"
sleep 1
check_transform "base_link" "imu_link" || publish_transform "base_link" "imu_link" 0 0 0.1
sleep 1
check_transform "imu_link" "lidar_link" || publish_transform "imu_link" "lidar_link" 0 0 0.2

# Verify the complete chain works
echo "Checking complete transform chain..."
sleep 2
if check_transform "world" "lidar_link"; then
    echo "✓ Complete transform chain is working!"
else
    echo "✗ Complete transform chain is NOT working!"
    echo "Attempting to fix complete chain..."
    # This is a fallback - publish direct transforms to ensure connectivity
    publish_transform "world" "lidar_link" 0 0 0.3 0 0 0
fi

# Show the full TF tree
echo "Generating TF tree visualization..."
ros2 run tf2_tools view_frames

echo "TF tree verification complete."
echo "If you continue to experience TF issues, manually restart the static transform publishers."
echo "==========================================" 