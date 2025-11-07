#!/bin/bash

# Build only the vision_processing package
echo "Building vision_processing..."
colcon build --packages-select vision_processing
if [ $? -ne 0 ]; then
    echo "Build failed. Exiting."
    exit 1
fi

# Source the ROS2 workspace
echo "Sourcing workspace..."
source install/setup.bash

# Run the vision_processing node
echo "Starting vision_processing node..."
ros2 run vision_processing vision_processing
