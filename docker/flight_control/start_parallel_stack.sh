#!/usr/bin/env bash
set -euo pipefail

count="${1:?environment count is required}"
profiles="${2:?candidate profile directory is required}"
results="${3:?batch results directory is required}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
compose="${root}/docker/flight_control/compose.yml"

mkdir -p "${results}/logs"
for ((index=0; index<count; index++)); do
  [[ -f "${profiles}/env_${index}.parm" ]] || {
    echo "Missing candidate parameter file: ${profiles}/env_${index}.parm" >&2
    exit 1
  }
done

docker compose -f "${compose}" stop -t 2 ardupilot-sitl mavros rtabmap-slam >/dev/null
docker rm -f isaac-pid-sitl-batch isaac-pid-mavros-batch >/dev/null 2>&1 || true
sitl_image="isaac-flight-control-ardupilot-sitl:local"
mavros_image="isaac-flight-control-mavros:local"
if ! docker image inspect "${sitl_image}" "${mavros_image}" >/dev/null 2>&1; then
  docker compose -f "${compose}" build ardupilot-sitl mavros >/dev/null
fi

docker run -d --name isaac-pid-sitl-batch --network host \
  --user "$(id -u):$(id -g)" \
  -e PID_PORT_STRIDE=10 \
  -v "${root}:/workspace:ro" \
  -v "${profiles}:/profiles:ro" \
  -v "${results}:/results" \
  "${sitl_image}" \
  bash /workspace/docker/flight_control/parallel_sitl_entrypoint.sh "${count}" >/dev/null

docker run -d --name isaac-pid-mavros-batch --network host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e PID_PORT_STRIDE=10 \
  -v "${root}:/workspace:ro" \
  -v "${results}:/results" \
  "${mavros_image}" \
  bash /workspace/docker/flight_control/parallel_mavros_entrypoint.sh "${count}" >/dev/null

echo "Started ${count} isolated ArduPilot and MAVROS instances"
