#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
requested="${1:-}"
if [[ -z "${requested}" ]]; then
  requested="$(find "${root}/simulation/room_quadcopter/results/exploration" -mindepth 2 -maxdepth 2 -type f -name voxel_map.npz -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)"
fi
if [[ -d "${requested}" ]]; then
  map_path="${requested}/voxel_map.npz"
else
  map_path="${requested}"
fi
[[ -f "${map_path}" ]] || { echo "Saved voxel map not found: ${map_path}" >&2; exit 1; }

container="isaac-saved-voxel-viewer"
docker rm -f "${container}" >/dev/null 2>&1 || true
cleanup() { docker rm -f "${container}" >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

relative="${map_path#"${root}"/}"
docker run --rm --name "${container}" --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "${root}:/workspace:ro" \
  isaac-flight-control-mavros:local \
  bash -lc 'source /opt/ros/jazzy/setup.bash; exec python3 /workspace/docker/flight_control/view_voxel_map.py "$1"' \
  bash "/workspace/${relative}" &
publisher_pid=$!

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "Opening saved stabilized map: ${map_path}"
rviz2 -d "${root}/simulation/room_quadcopter/rviz/room_quadcopter.rviz" --ros-args -p use_sim_time:=false
wait "${publisher_pid}" 2>/dev/null || true
