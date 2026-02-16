#!/bin/bash
# run_simulation.sh

# 1. Cleanup old processes
echo "Killing old Webots and ROS 2 instances..."
pkill -9 -f webots || true
pkill -9 -f rclcpp || true
pkill -9 -f nav2_ || true
pkill -9 -f planner_server || true
pkill -9 -f controller_server || true
pkill -9 -f webots_controller || true
sleep 2

# 2. Source ROS 2 (Host Env)
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
colcon build --symlink-install --packages-select usv_simulation usv_description usv_navigation

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
    echo -e "\nShutting down simulation..."
    # Kill all background processes started by this script
    kill $(jobs -p) 2>/dev/null
    exit
}

# Trap Ctrl+C (SIGINT) and SIGTERM
trap cleanup SIGINT SIGTERM

# 4. Launch Simulation
# We sleep specifically to let Webots start the Clock before Nav2 tries to connect
echo "Launching Simulation..."
ros2 launch usv_simulation robot_launch.py &

echo "Waiting 5 seconds for simulation clock to start..."
sleep 5

# 5. Launch Navigation
echo "Launching Nav2 Stack..."
ros2 launch usv_navigation bringup_launch.py &

echo "System Launched. Press Ctrl+C to stop."

# Wait for background processes to keep the script running
wait