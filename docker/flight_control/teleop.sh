#!/usr/bin/env bash
set -euo pipefail

namespace="${1:-${TELEOP_NAMESPACE:-/drone_1}}"

exec docker exec -it isaac-mavros bash -lc \
  "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ISAAC_SIM_FORCE_GROUND_DISARM=1; source /opt/ros/jazzy/setup.bash && python3 /workspace/docker/flight_control/teleop.py --namespace '${namespace}'"
