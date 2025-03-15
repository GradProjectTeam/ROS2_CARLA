#!/bin/bash

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Please install it first:"
    echo "sudo apt-get install tmux"
    exit 1
fi

source /home/mostafa/GP/ROS2/install/setup.bash
# Kill any existing tmux session with the same name
tmux kill-session -t lidar_session 2>/dev/null

# Create a new tmux session
tmux new-session -d -s lidar_session

# Split the window vertically
tmux split-window -v

# Split the bottom pane horizontally
tmux split-window -h

# Select first pane (top) and run the Lidar Processing executable
tmux select-pane -t 0
tmux send-keys "echo 'Starting Lidar Processing...'" C-m
tmux send-keys "/home/mostafa/ROS2andCarla/CPP/Lidar_Processing/build/Lidar_Processing" C-m

# Select second pane (bottom-left) and run the ROS2 lidar listener
tmux select-pane -t 1
tmux send-keys "echo 'Starting ROS2 Lidar Listener...'" C-m
tmux send-keys "ros2 run carla_sensors lidar_listener" C-m

# Select third pane (bottom-right) and run the lidar4.py executable
tmux select-pane -t 2
tmux send-keys "echo 'Starting Lidar4 Script...'" C-m
tmux send-keys "/home/mostafa/ROS2andCarla/CARLA/Sensors/lidar4.py" C-m

# Attach to the tmux session
tmux attach-session -t lidar_session