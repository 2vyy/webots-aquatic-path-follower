#!/bin/bash
# run_simulation.sh

echo "Sourcing ROS Iron..."
source /opt/ros/iron/setup.bash

echo "Sourcing Local Workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Warning: install/setup.bash not found. Building first..."
fi

echo "Building Package..."
colcon build --symlink-install

echo "Sourcing Local Workspace (post-build)..."
source install/setup.bash

echo "Launching Robot..."
ros2 launch my_package robot_launch.py
