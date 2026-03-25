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

UART_LIDAR1="/dev/ttyAMA0"
UART_LIDAR2="/dev/ttyAMA2"
UART_LIDAR1_ALT="/dev/serial0"
UART_LIDAR2_ALT="/dev/serial1"

get_console_devices() {
    if [ -r /proc/cmdline ]; then
        tr ' ' '\n' < /proc/cmdline | sed -n 's/^console=\([^, ]*\).*/\/dev\/\1/p'
    fi
}

is_console_device() {
    local path="$1"
    local dev
    while IFS= read -r dev; do
        [ -z "$dev" ] && continue
        if [ "$path" = "$dev" ]; then
            return 0
        fi
    done < <(get_console_devices)
    return 1
}

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

discover_ttyama_pair() {
    local ports=()
    mapfile -t ports < <(compgen -G "/dev/ttyAMA*" | sort -V)

    local candidates=()
    local port
    for port in "${ports[@]}"; do
        if is_console_device "$port"; then
            continue
        fi
        candidates+=("$port")
    done

    if [ "${#candidates[@]}" -ge 2 ]; then
        echo "${candidates[0]};${candidates[1]}"
        return 0
    fi

    return 1
}

find_waypoint_traj_impl() {
    local install_prefix="$1"
    local matches=()
    local egg_links=()
    local egg_link
    local source_dir

    mapfile -t matches < <(compgen -G "$install_prefix/omni_traj/lib/python*/site-packages/omni_traj/waypoint_traj_node.py" || true)
    if [ "${#matches[@]}" -gt 0 ]; then
        echo "${matches[0]}"
        return 0
    fi

    mapfile -t egg_links < <(compgen -G "$install_prefix/omni_traj/lib/python*/site-packages/*.egg-link" || true)
    for egg_link in "${egg_links[@]}"; do
        source_dir="$(head -n 1 "$egg_link" 2>/dev/null || true)"
        if [ -n "$source_dir" ] && [ -f "$source_dir/omni_traj/waypoint_traj_node.py" ]; then
            echo "$source_dir/omni_traj/waypoint_traj_node.py"
            return 0
        fi
    done

    return 1
}

workspace_has_synced_fusion() {
    local setup_path="$1"
    local install_prefix
    local impl_path

    install_prefix="$(cd "$(dirname "$setup_path")" && pwd)"
    impl_path="$(find_waypoint_traj_impl "$install_prefix" || true)"
    if [ -z "$impl_path" ]; then
        return 1
    fi

    if grep -q '_select_fusion_scans' "$impl_path"; then
        return 0
    fi

    return 1
}

pick_workspace_setup() {
    local candidates=(
        "/home/nickolas/ros2_ws/install/local_setup.bash"
        "/home/nickolas/ros2_ws/install/setup.bash"
        "$PROJECT_ROOT/install/local_setup.bash"
        "/home/nickolas/ros2_ws/src/omni_src/install/local_setup.bash"
        "$PROJECT_ROOT/install/setup.bash"
        "/home/nickolas/ros2_ws/src/omni_src/install/setup.bash"
        "/home/nickolas/ros2_ws/src/omni_src/omni_traj/install/setup.bash"
    )

    local candidate
    for candidate in "${candidates[@]}"; do
        if [ -f "$candidate" ] && workspace_has_synced_fusion "$candidate"; then
            echo "$candidate"
            return 0
        fi
    done

    for candidate in "${candidates[@]}"; do
        if [ -f "$candidate" ]; then
            echo "WARNING: Falling back to workspace setup without synchronized fusion marker: $candidate" >&2
            echo "$candidate"
            return 0
        fi
    done

    return 1
}

pick_lidar_ports() {
    local lidar1="${LIDAR1_SERIAL_PORT:-}"
    local lidar2="${LIDAR2_SERIAL_PORT:-}"

    if [ -n "$lidar1" ] && [ -n "$lidar2" ] && [ -e "$lidar1" ] && [ -e "$lidar2" ]; then
        echo "$lidar1;$lidar2"
        return 0
    fi

    local discovered_ports
    discovered_ports="$(discover_ttyama_pair || true)"
    if [ -n "$discovered_ports" ]; then
        echo "$discovered_ports"
        return 0
    fi

    if [ -z "$lidar1" ]; then
        if [ -e "$UART_LIDAR1" ] && ! is_console_device "$UART_LIDAR1"; then
            lidar1="$UART_LIDAR1"
        elif [ -e "$UART_LIDAR1_ALT" ] && ! is_console_device "$UART_LIDAR1_ALT"; then
            lidar1="$UART_LIDAR1_ALT"
        fi
    fi

    if [ -z "$lidar2" ]; then
        if [ -e "$UART_LIDAR2" ] && ! is_console_device "$UART_LIDAR2"; then
            lidar2="$UART_LIDAR2"
        elif [ -e "$UART_LIDAR2_ALT" ] && ! is_console_device "$UART_LIDAR2_ALT"; then
            lidar2="$UART_LIDAR2_ALT"
        fi
    fi

    if [ -n "$lidar1" ] && [ -n "$lidar2" ] && [ -e "$lidar1" ] && [ -e "$lidar2" ]; then
        echo "$lidar1;$lidar2"
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
    echo "  - $PROJECT_ROOT/install/local_setup.bash"
    echo "  - /home/nickolas/ros2_ws/src/omni_src/install/local_setup.bash"
    echo "  - $PROJECT_ROOT/install/setup.bash"
    echo "  - /home/nickolas/ros2_ws/install/setup.bash"
    echo "  - /home/nickolas/ros2_ws/src/omni_src/install/setup.bash"
    exit 1
fi

LIDAR_PORTS="$(pick_lidar_ports || true)"
if [ -z "$LIDAR_PORTS" ]; then
    echo "Waiting for LiDAR serial devices to appear..."
    wait_for_path "$UART_LIDAR1" 30 || true
    wait_for_path "$UART_LIDAR2" 30 || true
    wait_for_path "$UART_LIDAR1_ALT" 30 || true
    wait_for_path "$UART_LIDAR2_ALT" 30 || true
    wait_for_path "/dev/ttyAMA10" 30 || true
    LIDAR_PORTS="$(pick_lidar_ports || true)"
fi

if [ -z "$LIDAR_PORTS" ]; then
    echo "ERROR: Could not determine LiDAR UART ports"
    echo "       Tried defaults: $UART_LIDAR1, $UART_LIDAR2, $UART_LIDAR1_ALT, $UART_LIDAR2_ALT"
    echo "       Console UARTs in use:"
    get_console_devices | sed 's/^/         - /'
    ls -l /dev/ttyAMA* 2>/dev/null || true
    ls -l /dev/serial* 2>/dev/null || true
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

WS_INSTALL_PREFIX="$(cd "$(dirname "$WS_SETUP")" && pwd)"
WAYPOINT_TRAJ_IMPL="$(find_waypoint_traj_impl "$WS_INSTALL_PREFIX" || true)"
if [ -n "$WAYPOINT_TRAJ_IMPL" ]; then
    echo "Selected waypoint_traj implementation: $WAYPOINT_TRAJ_IMPL"
fi

ROOT_WS_SETUP="/home/nickolas/ros2_ws/install/local_setup.bash"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$ROOT_WS_SETUP" ]; then
    source "$ROOT_WS_SETUP"
fi
source "$WS_SETUP"
set -u

RESOLVED_WAYPOINT_TRAJ="$(python3 - <<'PY'
import omni_traj.waypoint_traj_node as module
print(module.__file__)
PY
)"
echo "Python resolved waypoint_traj module: $RESOLVED_WAYPOINT_TRAJ"

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
    serial_baudrate:=460800
    scan_mode:=Standard
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