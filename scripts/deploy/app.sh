#!/bin/bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source the workspace setup
source /root/workspace/install/setup.bash

echo "Current directory: $(pwd)"
echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"

# List all packages
echo "All available packages:"
ros2 pkg list

# Try to find the limo_control package
echo "Attempting to find limo_control package:"
ros2 pkg prefix limo_control

# Run the launch file
echo "Attempting to launch limo_control:"
ros2 launch limo_control limo_complete.launch.py