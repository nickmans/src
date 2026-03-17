#!/bin/bash

set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /home/nickolas/ros2_ws/src/omni_src/install/setup.bash
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
