#!/usr/bin/env bash
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

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
source $SCRIPT_DIR/../install/setup.bash

# Launch the ROS 2 node
ros2 run headless_pkg headless_full

