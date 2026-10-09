#!/usr/bin/env bash
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx isaac-mavros; then
  echo "isaac-mavros is not running. Start the flight-control stack first." >&2
  exit 1
fi

docker exec -it isaac-mavros bash -lc \
  'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/jazzy/setup.bash; exec python3 /workspace/docker/flight_control/live_telemetry.py "$@"' \
  bash "$@"
