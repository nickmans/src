#!/bin/bash
# Quick start script for STM32 Test Client

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "======================================================"
echo "  STM32 Test Client - Quick Start"
echo "======================================================"
echo ""

# Default values
HOST="${1:-192.168.1.100}"
PORT="${2:-9000}"
MOTION="${3:-stationary}"

echo "Configuration:"
echo "  - Server: $HOST:$PORT"
echo "  - Motion mode: $MOTION"
echo ""
echo "Available motion modes:"
echo "  - stationary: Robot stays at origin"
echo "  - forward: Robot moves forward"
echo "  - circle: Robot follows circular path"
echo ""
echo "Interactive commands:"
echo "  1 or start - Send START_TRAJ command"
echo "  2 or stop  - Send STOP_TRAJ command"
echo "  m <mode>   - Change motion mode"
echo "  q or quit  - Exit"
echo ""

# Run the test client
exec python3 test_stm32_client.py --host "$HOST" --port "$PORT" --motion "$MOTION"
