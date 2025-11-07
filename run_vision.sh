#!/bin/bash

# Source the ROS2 workspace
echo "Sourcing workspace..."
source ~/ros2_ws/install/setup.bash

# Run the vision_processing node
echo "Starting vision_processing node..."
ros2 run vision_processing vision_processing
