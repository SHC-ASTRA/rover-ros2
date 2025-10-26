#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

ANCHOR_WS="$SCRIPT_DIR/.."
AUTONOMY_WS="$ANCHOR_WS/../rover-Autonomy"
BAG_LOCATION="$ANCHOR_WS/../bags/autostart"

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
