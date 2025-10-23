#!/bin/bash

# Wait for a network interface to be up (not necessarily online)
while ! ip link show | grep -q "state UP"; do
    echo "[INFO] Waiting for active network interface..."
    sleep 1
done

echo "[INFO] Network interface is up!"

# Your actual ROS node start command goes here
echo "[INFO] Starting ROS node..."

# Source ROS 2 Humble setup script
source /opt/ros/humble/setup.bash

# Source your workspace setup script
source /home/clucky/rover-ros2/install/setup.bash

# CD to directory
cd /home/clucky/rover-ros2/

# Launch the ROS 2 node with the desired mode
ros2 launch anchor_pkg rover.launch.py mode:=anchor
