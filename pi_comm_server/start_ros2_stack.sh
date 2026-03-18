#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LAUNCH_FILE="dual_sllidar_with_mock_and_traj.launch.py"
STACK_LOCK_FILE="/tmp/omni_ros2_stack_controller.lock"

exec 9>"$STACK_LOCK_FILE"
if ! flock -n 9; then
    echo "ERROR: ROS2 stack controller is already active (lock: $STACK_LOCK_FILE)"
    echo "       Stop the existing controller before running start_ros2_stack.sh"
    exit 1
fi

echo "pid=$$ controller=start_ros2_stack.sh" 1>&9

HARDCODED_LIDAR1="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2608b4e7586eef118367e9c2c169b110-if00-port0"
HARDCODED_LIDAR2="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_420b6b8a586eef11a134e0c2c169b110-if00-port0"

wait_for_path() {
    local path="$1"
    local timeout_s="${2:-30}"
    local i
    for i in $(seq 1 "$timeout_s"); do
        if [ -e "$path" ]; then
            return 0
        fi
        sleep 1
    done
    return 1
}

pick_workspace_setup() {
    local candidates=(
        "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/setup.bash"
        "$PROJECT_ROOT/install/setup.bash"
        "/home/nickolas/ros2_ws/install/setup.bash"
        "/home/nickolas/ros2_ws/src/omni_src/install/setup.bash"
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

pick_lidar_ports() {
    local lidar1="${LIDAR1_SERIAL_PORT:-}"
    local lidar2="${LIDAR2_SERIAL_PORT:-}"

    if [ -n "$lidar1" ] && [ -n "$lidar2" ]; then
        echo "$lidar1;$lidar2"
        return 0
    fi

    if [ -e "$HARDCODED_LIDAR1" ] && [ -e "$HARDCODED_LIDAR2" ]; then
        echo "$HARDCODED_LIDAR1;$HARDCODED_LIDAR2"
        return 0
    fi

    if [ -d /dev/serial/by-id ]; then
        local serial_ports=()
        mapfile -t serial_ports < <(find /dev/serial/by-id -maxdepth 1 -type l -name '*CP2102*port0' | sort)
        if [ "${#serial_ports[@]}" -ge 2 ]; then
            echo "${serial_ports[0]};${serial_ports[1]}"
            return 0
        fi
    fi

    if [ -e /dev/ttyUSB0 ] && [ -e /dev/ttyUSB1 ]; then
        echo "/dev/ttyUSB0;/dev/ttyUSB1"
        return 0
    fi

    return 1
}

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: /opt/ros/jazzy/setup.bash not found"
    exit 1
fi

WS_SETUP="$(pick_workspace_setup || true)"
if [ -z "$WS_SETUP" ]; then
    echo "ERROR: Could not find workspace setup.bash"
    echo "Checked:"
    echo "  - $PROJECT_ROOT/install/setup.bash"
    echo "  - /home/nickolas/ros2_ws/install/setup.bash"
    echo "  - /home/nickolas/ros2_ws/src/omni_src/install/setup.bash"
    exit 1
fi

LIDAR_PORTS="$(pick_lidar_ports || true)"
if [ -z "$LIDAR_PORTS" ]; then
    echo "Waiting for LiDAR serial devices to appear..."
    wait_for_path /dev/serial/by-id 30 || true
    LIDAR_PORTS="$(pick_lidar_ports || true)"
fi

if [ -z "$LIDAR_PORTS" ]; then
    echo "ERROR: Could not determine LiDAR serial ports"
    ls -l /dev/serial/by-id 2>/dev/null || true
    ls -l /dev/ttyUSB* 2>/dev/null || true
    exit 1
fi

LIDAR1_PORT="${LIDAR_PORTS%;*}"
LIDAR2_PORT="${LIDAR_PORTS#*;}"

if ! wait_for_path "$LIDAR1_PORT" 30; then
    echo "ERROR: LiDAR1 device not ready: $LIDAR1_PORT"
    exit 1
fi

if ! wait_for_path "$LIDAR2_PORT" 30; then
    echo "ERROR: LiDAR2 device not ready: $LIDAR2_PORT"
    exit 1
fi

echo "Using workspace setup: $WS_SETUP"
echo "Using LiDAR ports: $LIDAR1_PORT | $LIDAR2_PORT"

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_SETUP"
set -u

launch_cmd=(
    ros2 launch omni_traj "$LAUNCH_FILE"
    traj_params_file:="$PROJECT_ROOT/omni_traj/config/waypoint_traj.yaml"
    use_mock_lidar:=false
    use_rviz:=false
    enable_amcl_localization:=false
    enable_slam_toolbox:=false
    map_frame:=map
    publish_odom_to_base_tf:=true
    publish_world_to_odom_tf:=false
    rolling_map_enable:=true
    rolling_map_margin_m:=1.0
    persistent_obstacles_enable:=false
    lidar1_serial_port:="$LIDAR1_PORT"
    lidar2_serial_port:="$LIDAR2_PORT"
)

"${launch_cmd[@]}" &
launch_pid=$!

(
    for _ in {1..60}; do
        if ros2 service list 2>/dev/null | grep -qx '/lidar1/start_motor' && ros2 service list 2>/dev/null | grep -qx '/lidar2/start_motor'; then
            break
        fi
        sleep 1
    done

    if ! ros2 topic echo --qos-profile sensor_data --once --timeout 3 --field header.frame_id /lidar1/scan >/dev/null 2>&1; then
        ros2 service call /lidar1/start_motor std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
    fi

    if ! ros2 topic echo --qos-profile sensor_data --once --timeout 3 --field header.frame_id /lidar2/scan >/dev/null 2>&1; then
        ros2 service call /lidar2/start_motor std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
    fi
) &
helper_pid=$!

trap 'kill -INT "$launch_pid" 2>/dev/null || true' INT TERM

wait "$launch_pid"
launch_rc=$?
wait "$helper_pid" 2>/dev/null || true
exit "$launch_rc"