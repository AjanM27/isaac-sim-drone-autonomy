#!/usr/bin/env bash
set -euo pipefail

speed="${1:?usage: run_benchmark.sh SPEED [NAME]}"
name="${2:-square_${speed//./p}_mps_$(date +%Y%m%d_%H%M%S)}"
run_dir="/results/checkpoint1/${name}"
host_run_dir="simulation/room_quadcopter/results/checkpoint1/${name}"

for container in isaac-mavros isaac-ardupilot-sitl isaac-rtabmap-slam; do
  if ! docker ps --format '{{.Names}}' | grep -qx "${container}"; then
    echo "${container} is not running. Start the Checkpoint-1 stack first." >&2
    exit 1
  fi
done

mkdir -p "${host_run_dir}"
started_at="$(date --iso-8601=seconds)"
docker exec isaac-rtabmap-slam bash -lc \
  'source /opt/ros/jazzy/setup.bash; source /slam_ws/install/setup.bash; exec python3 /workspace/docker/flight_control/slam_diagnostics_recorder.py --output "$1/slam_diagnostics.csv" --stop-file "$1/trajectory.done"' \
  bash "${run_dir}" &
diagnostics_pid=$!
cleanup() {
  touch "${host_run_dir}/trajectory.done"
  wait "${diagnostics_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

docker exec isaac-mavros bash -lc \
  'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ISAAC_SIM_FORCE_GROUND_DISARM=1 MPLCONFIGDIR=/tmp/matplotlib; source /opt/ros/jazzy/setup.bash; exec python3 /workspace/docker/flight_control/benchmark_trajectory.py --speed "$1" --name "$2"' \
  bash "${speed}" "${name}"

touch "${host_run_dir}/trajectory.done"
wait "${diagnostics_pid}"
trap - EXIT INT TERM
docker logs --since "${started_at}" isaac-rtabmap-slam > "${host_run_dir}/rtabmap.log" 2>&1
docker logs --since "${started_at}" isaac-ardupilot-sitl > "${host_run_dir}/ardupilot.log" 2>&1
python3 docker/flight_control/analyze_benchmark.py "${host_run_dir}"
docker exec isaac-mavros chmod -R a+rwX /results/checkpoint1

echo "Benchmark complete: simulation/room_quadcopter/results/checkpoint1/${name}"
