#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
count="${1:-9}"
run_id="warehouse_fleet_demo_${count}_$(date +%Y%m%d_%H%M%S)"

if ! [[ "${count}" =~ ^[0-9]+$ ]] || ((count < 1 || count > 16)); then
  echo "Drone count must be between 1 and 16" >&2
  exit 2
fi

cd "${root}"
export PARALLEL_PHYSICS_RATE="${PARALLEL_PHYSICS_RATE:-80.0}"
export ISAAC_DISABLE_ROTOR_VISUALS="${ISAAC_DISABLE_ROTOR_VISUALS:-1}"
export ISAAC_LIGHTWEIGHT_ROTOR_VISUALS="${ISAAC_LIGHTWEIGHT_ROTOR_VISUALS:-1}"
exec python3 docker/flight_control/benchmark_warehouse_fleet.py \
  --drones "${count}" \
  --protocol demo \
  --speed 0.6 \
  --warmup-seconds 3 \
  --hover-seconds 6 \
  --visible \
  --record \
  --run-id "${run_id}"
