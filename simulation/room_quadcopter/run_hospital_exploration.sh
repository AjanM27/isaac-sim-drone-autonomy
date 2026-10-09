#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
hospital="${HOSPITAL_USD_PATH:-${root}/simulation/room_quadcopter/assets/isaac/6.0/Isaac/Environments/Hospital/hospital.usd}"
run_id="hospital_exploration_$(date +%Y%m%d_%H%M%S)"
output="${root}/simulation/room_quadcopter/results/exploration/${run_id}"
mkdir -p "${output}"
[[ -f "${hospital}" ]] || { echo "Offline hospital missing: ${hospital}" >&2; exit 1; }
stop_simulator_group() {
  [[ -n "${sim_pid:-}" ]] || return 0
  kill -TERM -- "-${sim_pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 -- "-${sim_pid}" 2>/dev/null || break
    sleep 0.1
  done
  # Kit can ignore TERM while a renderer/plugin is shutting down.
  kill -KILL -- "-${sim_pid}" 2>/dev/null || true
  wait "${sim_pid}" 2>/dev/null || true
}
stop_octomap_capture() {
  [[ -n "${octomap_capture_pid:-}" ]] || return 0
  docker exec isaac-rtabmap-slam pkill -TERM -f '[o]ctomap_capture.py' >/dev/null 2>&1 || true
  wait "${octomap_capture_pid}" 2>/dev/null || true
  unset octomap_capture_pid
}
cleanup() {
  # run_room starts Isaac Python and RViz children. They share the dedicated
  # session created below, so terminate the whole group and never leak a stale
  # UDP 9002 owner or stale ROS topics into the next run.
  stop_octomap_capture
  if [[ -n "${sim_pid:-}" ]]; then
    stop_simulator_group
  fi
  docker compose -f "${root}/docker/flight_control/compose.yml" stop -t 3 >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM
export SLAM_RESET_DATABASE=true ENVIRONMENT_USD_PATH="${hospital}"
export ISAAC_AUTO_RVIZ=1 FLIGHT_CONTROL_RGB_CAPTURE=0 ISAAC_CAMERA_FPS=1
export ISAAC_VIEWPORT_WIDTH=960 ISAAC_VIEWPORT_HEIGHT=540 ISAAC_RTX_SHADOWS=0
export ISAAC_RTX_REFLECTIONS=0 ISAAC_RTX_AMBIENT_OCCLUSION=0 ISAAC_RTX_INDIRECT_DIFFUSE=0
export ISAAC_DLSS_MODE=0
cd "${root}"
bash docker/flight_control/start.sh
launch_simulator() {
  setsid simulation/room_quadcopter/run_room.sh --flight-control ardupilot >>"${output}/isaac.log" 2>&1 &
  sim_pid=$!
}
launch_simulator
echo "Waiting for hospital SLAM and safety sensors..."
ready=0
for attempt in 1 2; do
  for _ in $(seq 1 180); do
    if ! kill -0 "${sim_pid}" 2>/dev/null; then
      break
    fi
    if docker exec isaac-mavros bash -lc 'source /opt/ros/jazzy/setup.bash; timeout 2 ros2 topic echo --once /range/up >/dev/null 2>&1' && docker exec isaac-mavros bash -lc 'source /opt/ros/jazzy/setup.bash; timeout 2 ros2 topic echo --once /slam/odom >/dev/null 2>&1'; then ready=1; break 2; fi
    sleep 1
  done
  if [[ "${attempt}" == "1" ]]; then
    echo "Isaac exited before sensors became ready; retrying once..." >&2
    if kill -0 "${sim_pid}" 2>/dev/null; then
      stop_simulator_group
    fi
    sleep 3
    launch_simulator
  fi
done
if [[ "${ready}" != "1" ]]; then
  echo "Timed out waiting for /range/up and /slam/odom." >&2
  tail -120 "${output}/isaac.log"
  exit 1
fi
echo "Starting collision-aware frontier exploration; results: ${output}"
docker exec -e PYTHONUNBUFFERED=1 isaac-rtabmap-slam \
  bash -lc 'source /opt/ros/jazzy/setup.bash; source /slam_ws/install/setup.bash; exec python3 -u /workspace/docker/slam/octomap_capture.py --output-dir "$1"' \
  bash "/results/exploration/${run_id}" >>"${output}/octomap_capture.log" 2>&1 &
octomap_capture_pid=$!
docker exec -e ISAAC_SIM_FORCE_GROUND_DISARM=1 -e PYTHONUNBUFFERED=1 isaac-mavros \
  bash -lc 'source /opt/ros/jazzy/setup.bash; exec python3 -X faulthandler -u /workspace/docker/flight_control/hospital_voxel_explorer.py --output-dir "$1" "${@:2}"' \
  bash "/results/exploration/${run_id}" "$@"
stop_octomap_capture
# Flush the SQLite map before copying the run artifact out of the shared path.
docker stop -t 10 isaac-rtabmap-slam >/dev/null 2>&1 || true
if [[ -f "${root}/simulation/room_quadcopter/results/slam/rtabmap.db" ]]; then
  cp "${root}/simulation/room_quadcopter/results/slam/rtabmap.db" "${output}/rtabmap.db"
fi
rtf_values="$(rg -o 'RTF=[0-9.]+' "${output}/isaac.log" | cut -d= -f2 || true)"
if [[ -n "${rtf_values}" ]]; then
  awk '{sum+=$1; count++; if(count==1||$1<min)min=$1} END {printf "samples=%d\nmean_rtf=%.4f\nminimum_rtf=%.4f\n",count,sum/count,min}' \
    <<<"${rtf_values}" >"${output}/rtf.txt"
else
  echo "samples=0" >"${output}/rtf.txt"
fi
echo "Exploration finished. Results saved under ${output}"
