#!/usr/bin/env bash
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

[[ -z "$ANCHOR_WS" ]] && ANCHOR_WS="$SCRIPT_DIR/.."
[[ -z "$AUTONOMY_WS" ]] && AUTONOMY_WS="$HOME/rover-Autonomy"
BAG_LOCATION="$HOME/bags/autostart"
[[ ! -d "$BAG_LOCATION" ]] && mkdir -p "$BAG_LOCATION"

# Wait for a network interface to be up (not necessarily online)
while ! ip link show | grep -q "state UP"; do
    echo "[INFO] Waiting for active network interface..."
    sleep 1
done

echo "[INFO] Network interface is up!"


command -v ros2 || source /opt/ros/humble/setup.bash
source $ANCHOR_WS/install/setup.bash
[[ -f $AUTONOMY_WS/install/setup.bash ]] && source $AUTONOMY_WS/install/setup.bash

cd $BAG_LOCATION

ros2 bag record -a
