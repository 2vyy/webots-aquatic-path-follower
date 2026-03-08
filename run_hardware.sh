#!/bin/bash
# run_hardware.sh — Launch the full USV stack in HARDWARE mode (Real BlueBoat)
#
# Prerequisites:
#   - MAVROS installed: sudo apt install ros-${ROS_DISTRO}-mavros
#   - Real LiDAR driver configured in hardware_launch.py
#   - If running on BlueOS companion computer, this script runs inside
#     the blueos-ros2 Docker container.

# 1. Cleanup old ROS 2 instances
echo "Killing old ROS 2 instances..."
pkill -9 -f rclcpp || true
pkill -9 -f nav2_ || true
pkill -9 -f planner_server || true
pkill -9 -f controller_server || true
pkill -9 -f mavros || true
sleep 2

# 2. Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Sourcing ROS 2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
else
    echo "ERROR: No valid ROS 2 distribution found!"
    exit 1
fi

# 3. Build the Workspace
cd ~/Projects/usv_ws
echo "Building Workspace..."
colcon build --symlink-install --packages-select usv_description usv_navigation

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo "Build Successful. Launching nodes..."
    source install/setup.bash
else
    echo "ERROR: Build Failed! Aborting launch."
    exit 1
fi

# Function to cleanup background processes on exit
cleanup() {
    echo -e "\nShutting down hardware stack..."
    kill $(jobs -p) 2>/dev/null
    exit
}

# Trap Ctrl+C (SIGINT) and SIGTERM
trap cleanup SIGINT SIGTERM

# 4. Launch everything via the unified main.launch.py
echo "Launching USV Stack (Hardware Mode)..."
ros2 launch usv_navigation main.launch.py use_sim:=false &

echo "System Launched. Press Ctrl+C to stop."

# Wait for background processes to keep the script running
wait
