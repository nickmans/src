#!/bin/bash
# restart_ros2_stack.sh
# Launch or relaunch the ROS2 stack (dual_sllidar_with_mock_and_traj.launch.py)

set -euo pipefail

WS_ROOT="/home/nickolas/ros2_ws"
LAUNCH_FILE="dual_sllidar_with_mock_and_traj.launch.py"
LOG_FILE="/tmp/ros2_stack.log"
LAUNCH_MATCH="ros2 launch omni_traj ${LAUNCH_FILE}"

source_env() {
	if [ -f "/opt/ros/jazzy/setup.bash" ]; then
		set +u
		source /opt/ros/jazzy/setup.bash
		set -u
	fi

	if [ -f "$WS_ROOT/install/setup.bash" ]; then
		set +u
		source "$WS_ROOT/install/setup.bash"
		set -u
	fi
}

stop_stack_sigint() {
	local pids
	pids="$(pgrep -f "$LAUNCH_MATCH" || true)"

	if [ -z "$pids" ]; then
		echo "ROS2 stack is not currently running."
		return 0
	fi

	echo "Sending SIGINT (Ctrl+C) to running ROS2 stack..."
	while IFS= read -r pid; do
		[ -z "$pid" ] && continue
		local pgid
		pgid="$(ps -o pgid= -p "$pid" | tr -d ' ' || true)"
		if [ -n "$pgid" ]; then
			kill -INT -- "-$pgid" 2>/dev/null || true
		fi
	done <<< "$pids"

	for _ in {1..50}; do
		if ! pgrep -f "$LAUNCH_MATCH" >/dev/null; then
			return 0
		fi
		sleep 0.2
	done

	echo "ROS2 stack did not stop on SIGINT; escalating to SIGTERM..."
	pids="$(pgrep -f "$LAUNCH_MATCH" || true)"
	while IFS= read -r pid; do
		[ -z "$pid" ] && continue
		local pgid
		pgid="$(ps -o pgid= -p "$pid" | tr -d ' ' || true)"
		if [ -n "$pgid" ]; then
			kill -TERM -- "-$pgid" 2>/dev/null || true
		fi
	done <<< "$pids"
}

source_env
stop_stack_sigint

LAUNCH_CMD=(
	ros2 launch omni_traj "$LAUNCH_FILE"
	use_mock_lidar:=false
	use_rviz:=false
	map_frame:=odom
	publish_odom_to_base_tf:=false
	publish_world_to_odom_tf:=true
	rolling_map_enable:=true
	rolling_map_margin_m:=1.0
	persistent_obstacles_enable:=false
	lidar1_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_2608b4e7586eef118367e9c2c169b110-if00-port0
	lidar2_serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_420b6b8a586eef11a134e0c2c169b110-if00-port0
)

printf -v LAUNCH_CMD_STR '%q ' "${LAUNCH_CMD[@]}"

nohup bash -lc "source /opt/ros/jazzy/setup.bash && source ${WS_ROOT}/install/setup.bash && exec ${LAUNCH_CMD_STR}" > "$LOG_FILE" 2>&1 &

echo "ROS2 stack launched/relaunched using startup-equivalent command."
