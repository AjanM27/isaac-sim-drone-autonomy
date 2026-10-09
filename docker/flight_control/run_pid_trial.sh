#!/usr/bin/env bash
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx isaac-mavros; then
  echo "isaac-mavros is not running. Start Isaac Sim, then docker/flight_control/start.sh."
  exit 1
fi

name="${1:-trial_$(date +%Y%m%d_%H%M%S)}"
if [[ $# -gt 0 ]]; then
  shift
fi

docker exec isaac-mavros bash -lc \
  'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ISAAC_SIM_FORCE_GROUND_DISARM=1; source /opt/ros/jazzy/setup.bash; exec python3 /workspace/docker/flight_control/pid_trial.py --name "$1" "${@:2}"' \
  bash "$name" "$@"
docker exec isaac-mavros chmod -R a+rwX /results/pid_tuning
