#!/usr/bin/env bash
set -euo pipefail

SERVICES=(
  omni_ros2_stack.service
  omni_udp_server.service
  omni_pi_server.service
  omni_server.service
)

SYSTEMCTL_BIN="/usr/bin/systemctl"
SYNC_BIN="/usr/bin/sync"
SHUTDOWN_BIN="/usr/sbin/shutdown"

ASSUME_YES="${1:-}"
if [[ "$ASSUME_YES" != "-y" && "$ASSUME_YES" != "--yes" ]]; then
  echo "This will stop OMNI/ROS2 services and power off the Pi."
  read -r -p "Continue? [y/N] " reply
  if [[ ! "$reply" =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 0
  fi
fi

if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
else
  SUDO=""
fi

run_privileged() {
  if [[ -z "${SUDO}" ]]; then
    "$@"
    return $?
  fi

  sudo -n "$@"
  return $?
}

echo "Stopping services..."
ACTIVE_SERVICES=()
for service in "${SERVICES[@]}"; do
  if "$SYSTEMCTL_BIN" is-active --quiet "$service" 2>/dev/null; then
    ACTIVE_SERVICES+=("$service")
    echo "  - ${service}"
  else
    echo "  - ${service} (already inactive)"
  fi
done

if [[ ${#ACTIVE_SERVICES[@]} -gt 0 ]]; then
  STOP_FAILED=0
  for service in "${ACTIVE_SERVICES[@]}"; do
    if ! run_privileged "$SYSTEMCTL_BIN" stop "$service" >/dev/null 2>&1; then
      echo "ERROR: Could not stop ${service} non-interactively (sudo password required)."
      STOP_FAILED=1
    fi
  done

  if [[ "$STOP_FAILED" -ne 0 ]]; then
    echo "Run one of the following on the Pi to allow remote shutdown:"
    echo "  1) Configure passwordless sudo for shutdown/systemctl commands for this user"
    echo "  2) Run the OMNI UDP service as root"
    exit 1
  fi
fi

echo
echo "Service status after stop:"
for service in "${SERVICES[@]}"; do
  state="$("$SYSTEMCTL_BIN" is-active "$service" 2>/dev/null || true)"
  if [[ "$state" == "inactive" || "$state" == "failed" || "$state" == "unknown" ]]; then
    echo "  ✓ ${service}: ${state:-unknown}"
  else
    echo "  ! ${service}: ${state:-unknown}"
  fi
done

echo
echo "Syncing filesystem..."
if ! run_privileged "$SYNC_BIN"; then
  echo "WARNING: sync failed due to missing privileges"
fi

echo "Powering off now..."
if ! run_privileged "$SHUTDOWN_BIN" -h now; then
  echo "ERROR: shutdown command failed (likely sudo password required)."
  exit 1
fi
