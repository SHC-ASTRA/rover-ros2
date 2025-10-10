#!/bin/bash

ANCHOR_WS="/home/clucky/rover-ros2"
AUTONOMY_WS="/home/clucky/rover-Autonomy"
BAG_LOCATION="/home/clucky/bags/autostart"

# Wait for a network interface to be up (not necessarily online)
while ! ip link show | grep -q "state UP"; do
    echo "[INFO] Waiting for active network interface..."
    sleep 1
done

echo "[INFO] Network interface is up!"


source /opt/ros/humble/setup.bash
source $ANCHOR_WS/install/setup.bash
[ -f $AUTONOMY_WS/install/setup.bash ] && source $AUTONOMY_WS/install/setup.bash

cd $BAG_LOCATION

ros2 bag record -a
