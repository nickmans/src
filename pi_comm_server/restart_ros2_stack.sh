#!/bin/bash
# restart_ros2_stack.sh
# Launch or relaunch the ROS2 stack (dual_sllidar_with_mock_and_traj.launch.py)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
START_SCRIPT="$SCRIPT_DIR/start_ros2_stack.sh"
LOG_FILE="/tmp/ros2_stack.log"
LAUNCH_MATCH="ros2 launch omni_traj dual_sllidar_with_mock_and_traj.launch.py"

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

stop_stack_sigint

if [ ! -x "$START_SCRIPT" ]; then
	chmod +x "$START_SCRIPT"
fi

nohup "$START_SCRIPT" > "$LOG_FILE" 2>&1 &

echo "ROS2 stack launched/relaunched using startup-equivalent command."
