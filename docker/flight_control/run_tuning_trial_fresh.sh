#!/usr/bin/env bash
set -euo pipefail

profile="${1:?usage: run_tuning_trial_fresh.sh PROFILE TRIAL_NAME [pid_trial options]}"
name="${2:?usage: run_tuning_trial_fresh.sh PROFILE TRIAL_NAME [pid_trial options]}"
shift 2
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
compose="${root}/docker/flight_control/compose.yml"
log="${root}/simulation/room_quadcopter/results/pid_tuning/${name}.isaac.log"
metadata="${root}/simulation/room_quadcopter/results/pid_tuning/${name}.runtime.txt"
mkdir -p "$(dirname "${log}")"

if pgrep -f 'create_room_quadcopter.py.*--flight-control ardupilot' >/dev/null; then
  echo "An ArduPilot Isaac simulation is already running. Close it before a clean tuning trial." >&2
  exit 1
fi

sim_pid=""
sim_pgid=""
cleanup() {
  if [[ -n "${sim_pgid}" ]]; then
    kill -INT -- "-${sim_pgid}" 2>/dev/null || true
    wait "${sim_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cd "${root}"
{
  date --iso-8601=seconds
  cpupower frequency-info 2>/dev/null | grep -E 'current policy|governor' || true
  nvidia-smi --query-gpu=name,driver_version,pstate --format=csv,noheader 2>/dev/null || true
} >"${metadata}"
docker compose -f "${compose}" up -d
docker compose -f "${compose}" restart ardupilot-sitl mavros rtabmap-slam
setsid env ISAAC_AUTO_RVIZ=0 FLIGHT_CONTROL_RGB_CAPTURE=0 \
  simulation/room_quadcopter/run_room.sh --flight-control ardupilot --headless --run-seconds 360 \
  >"${log}" 2>&1 &
sim_pid=$!
sim_pgid="$(ps -o pgid= -p "${sim_pid}" | tr -d ' ')"
docker exec isaac-mavros bash -lc \
  'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/jazzy/setup.bash; timeout 120 ros2 topic echo /clock --once >/dev/null'
pid_profile_args=()
if [[ -n "${PID_PROFILES_PATH:-}" ]]; then
  pid_profile_args+=(--profiles "${PID_PROFILES_PATH}")
fi
bash docker/flight_control/pid_tune.sh "${pid_profile_args[@]}" apply "${profile}"
bash docker/flight_control/run_pid_trial.sh "${name}" "$@"
latest_bin="$(docker exec isaac-ardupilot-sitl bash -lc "find /ardupilot/logs -maxdepth 1 -type f -name '*.BIN' -printf '%T@ %p\\n' | sort -nr | head -1 | cut -d' ' -f2-")"
if [[ -n "${latest_bin}" ]]; then
  docker cp "isaac-ardupilot-sitl:${latest_bin}" "${log%.isaac.log}.BIN" >/dev/null
fi
