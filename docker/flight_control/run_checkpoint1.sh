#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
compose="${root}/docker/flight_control/compose.yml"
results="${root}/simulation/room_quadcopter/results/checkpoint1"
mkdir -p "${results}"

if pgrep -f 'create_room_quadcopter.py.*--flight-control ardupilot' >/dev/null; then
  echo "An ArduPilot Isaac simulation is already running. Close it before the isolated benchmark." >&2
  exit 1
fi

sim_pid=""
sim_pgid=""
stop_sim() {
  if [[ -n "${sim_pgid}" ]]; then
    kill -INT -- "-${sim_pgid}" 2>/dev/null || true
    wait "${sim_pid}" 2>/dev/null || true
  fi
  sim_pid=""
  sim_pgid=""
}
trap stop_sim EXIT INT TERM

cd "${root}"
docker compose -f "${compose}" up -d

for speed in 0.25 0.5 1.0; do
  label="square_${speed//./p}_mps"
  run_dir="${results}/${label}"
  mkdir -p "${run_dir}"
  docker compose -f "${compose}" restart ardupilot-sitl mavros rtabmap-slam
  setsid env ISAAC_AUTO_RVIZ=0 FLIGHT_CONTROL_RGB_CAPTURE=0 \
    simulation/room_quadcopter/run_room.sh --flight-control ardupilot --headless --run-seconds 180 \
    >"${run_dir}/isaac.log" 2>&1 &
  sim_pid=$!
  sim_pgid="$(ps -o pgid= -p "${sim_pid}" | tr -d ' ')"
  echo "[${speed} m/s] Waiting for Isaac /clock and LiDAR..."
  docker exec isaac-mavros bash -lc \
    'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/jazzy/setup.bash; timeout 120 ros2 topic echo /clock --once >/dev/null'
  docker exec isaac-mavros bash -lc \
    'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; source /opt/ros/jazzy/setup.bash; timeout 60 ros2 topic echo /slam/lidar/points --once >/dev/null'
  bash docker/flight_control/run_benchmark.sh "${speed}" "${label}"
  stop_sim
done

python3 docker/flight_control/compare_benchmarks.py "${results}"
echo "Checkpoint-1 benchmark complete: ${results}/SUMMARY.md"
