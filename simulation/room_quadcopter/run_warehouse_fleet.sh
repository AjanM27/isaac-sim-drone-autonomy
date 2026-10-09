#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
count=6
run_seconds=86400
headless=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drones) count="${2:?missing drone count}"; shift 2 ;;
    --run-seconds) run_seconds="${2:?missing duration}"; shift 2 ;;
    --headless) headless=1; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if ! [[ "${count}" =~ ^[0-9]+$ ]] || ((count < 1 || count > 16)); then
  echo "--drones must be between 1 and 16" >&2
  exit 2
fi

session="${root}/simulation/room_quadcopter/results/fleet/live_${count}"
profiles="${session}/profiles"
mkdir -p "${profiles}" "${session}/logs"
for ((index=0; index<count; index++)); do
  cp "${root}/docker/flight_control/isaac_sim_tuned.parm" "${profiles}/env_${index}.parm"
done

cleanup() {
  bash "${root}/docker/flight_control/stop_parallel_stack.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
cleanup
bash "${root}/docker/flight_control/start_parallel_stack.sh" \
  "${count}" "${profiles}" "${session}"

echo "Fleet controls are available from another terminal:"
echo "  bash docker/flight_control/fleet_teleop.sh 1"
echo "  ... through drone ${count}"

scene_args=(
  --env-count "${count}"
  --environment warehouse
  --spacing 2.5
  --run-seconds "${run_seconds}"
)
if [[ "${headless}" == 1 ]]; then
  scene_args+=(--headless)
else
  scene_args+=(--no-headless)
fi
cd "${root}"
ISAAC_AUTO_RVIZ=0 FLIGHT_CONTROL_RGB_CAPTURE=0 \
  simulation/room_quadcopter/run_parallel_tuning_scene.sh "${scene_args[@]}"
