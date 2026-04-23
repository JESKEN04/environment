#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$ROS2_WS/install/setup.bash"

source "$ROS_SETUP" 2>/dev/null || true
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

python3 "$SCRIPT_DIR/formation_coordinator.py"
