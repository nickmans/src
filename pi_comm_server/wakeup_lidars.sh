#!/bin/bash

set -euo pipefail

pick_workspace_setup() {
    local candidates=(
        "/home/nickolas/ros2_ws/install/setup.bash"
        "/home/nickolas/ros2_ws/src/omni_src/install/setup.bash"
        "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/setup.bash"
    )

    local candidate
    for candidate in "${candidates[@]}"; do
        if [ -f "$candidate" ]; then
            echo "$candidate"
            return 0
        fi
    done

    return 1
}

WS_SETUP="$(pick_workspace_setup || true)"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: /opt/ros/jazzy/setup.bash not found"
    exit 1
fi

if [ -z "$WS_SETUP" ]; then
    echo "ERROR: Could not find workspace setup.bash for LiDAR wakeup"
    exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_SETUP"
set -u

wait_for_service() {
    local svc="$1"
    local timeout_s="${2:-60}"
    for _ in $(seq 1 "$timeout_s"); do
        if ros2 service list 2>/dev/null | grep -qx "$svc"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_for_service "/lidar1/start_motor" 90 || true
wait_for_service "/lidar2/start_motor" 90 || true

for _ in {1..8}; do
    ros2 service call /lidar1/start_motor std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
    ros2 service call /lidar2/start_motor std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
    sleep 1

done
