#!/usr/bin/env bash
set -euo pipefail

drone="${1:?usage: bash docker/flight_control/fleet_teleop.sh DRONE_NUMBER}"
if ! [[ "${drone}" =~ ^[0-9]+$ ]] || ((drone < 1 || drone > 16)); then
  echo "DRONE_NUMBER must be between 1 and 16" >&2
  exit 2
fi
index=$((drone - 1))
container="isaac-pid-mavros-batch"
docker inspect "${container}" >/dev/null 2>&1 || {
  echo "Warehouse fleet is not running. Start run_warehouse_fleet.sh first." >&2
  exit 1
}

exec docker exec -it "${container}" bash -lc \
  "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ISAAC_SIM_FORCE_GROUND_DISARM=1; source /opt/ros/jazzy/setup.bash && python3 /workspace/docker/flight_control/teleop.py --namespace /tuning/env_${index}"
