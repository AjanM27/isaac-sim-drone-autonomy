#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
hospital="${HOSPITAL_USD_PATH:-${root}/simulation/room_quadcopter/assets/isaac/6.0/Isaac/Environments/Hospital/hospital.usd}"
results="${root}/simulation/room_quadcopter/results"

mkdir -p "${results}"
exec 9>"${results}/hospital_demo.lock"
if ! flock -n 9; then
  echo "A hospital simulation is already running." >&2
  echo "Close its Isaac Sim window before starting another copy." >&2
  exit 1
fi

if [[ ! -f "${hospital}" ]]; then
  echo "Offline Hospital USD not found: ${hospital}" >&2
  echo "Install it once while online with:" >&2
  echo "  ${root}/simulation/room_quadcopter/vendor_hospital.sh" >&2
  exit 1
fi

cd "${root}"
export ENVIRONMENT_USD_PATH="${hospital}"
export ISAAC_AUTO_RVIZ="${ISAAC_AUTO_RVIZ:-1}"
export RVIZ_WAIT_SECONDS="${RVIZ_WAIT_SECONDS:-300}"
export SLAM_RESET_DATABASE="${SLAM_RESET_DATABASE:-true}"
# The hospital is fully local. Complete USD/material composition before the
# timeline starts so the first visible frame, LiDAR scan, and SLAM update all
# refer to the same complete environment.
export ISAAC_SYNC_USD_LOADS="${ISAAC_SYNC_USD_LOADS:-1}"

echo "Starting ArduPilot, MAVROS, and a fresh RTAB-Map session..."
bash docker/flight_control/start.sh
echo "Opening the fully offline Hospital environment with 3D mapping and RViz..."
exec simulation/room_quadcopter/run_room.sh --flight-control ardupilot "$@"
