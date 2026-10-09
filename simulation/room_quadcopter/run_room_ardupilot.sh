#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SCRIPT="${WORKSPACE_ROOT}/simulation/room_quadcopter/scripts/create_room_quadcopter.py"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-}"

# Launch with ArduPilot flight control bridge enabled
exec "${WORKSPACE_ROOT}/scripts/isaac_python.sh" "${SCRIPT}" \
  --flight-control ardupilot \
  --sitl-json-port 9002 \
  "$@"
