#!/usr/bin/env bash
set -euo pipefail

docker exec isaac-mavros bash -lc \
  'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/jazzy/setup.bash; exec python3 /workspace/docker/flight_control/pid_tune.py "$@"' \
  bash "$@"
