#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAAC_PYTHON="${WORKSPACE_ROOT}/scripts/isaac_python.sh"
SCRIPT="${WORKSPACE_ROOT}/simulation/room_quadcopter/scripts/create_room_quadcopter.py"
DEFAULT_WAREHOUSE_USD="${WORKSPACE_ROOT}/simulation/room_quadcopter/assets/simple_warehouse/warehouse.usd"
DEFAULT_LIDAR_USD="${WORKSPACE_ROOT}/simulation/room_quadcopter/assets/isaac/6.0/Isaac/Sensors/Ouster/OS1/OS1.usd"
DEFAULT_DRONE_USD="${WORKSPACE_ROOT}/simulation/room_quadcopter/assets/iris/iris.usd"
DEFAULT_OFFLINE_MATERIALS_ROOT="${WORKSPACE_ROOT}/simulation/room_quadcopter/assets/materials"
RVIZ_LAUNCHER="${WORKSPACE_ROOT}/simulation/room_quadcopter/scripts/launch_rviz.sh"
RVIZ_CONFIG="${RVIZ_CONFIG:-${WORKSPACE_ROOT}/simulation/room_quadcopter/rviz/room_quadcopter.rviz}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-}"
export WAREHOUSE_USD_PATH="${WAREHOUSE_USD_PATH:-${DEFAULT_WAREHOUSE_USD}}"
export ENVIRONMENT_USD_PATH="${ENVIRONMENT_USD_PATH:-${WAREHOUSE_USD_PATH}}"
export LIDAR_USD_PATH="${LIDAR_USD_PATH:-${DEFAULT_LIDAR_USD}}"
export DRONE_USD_PATH="${DRONE_USD_PATH:-${DEFAULT_DRONE_USD}}"
export OFFLINE_MATERIALS_ROOT="${OFFLINE_MATERIALS_ROOT:-${DEFAULT_OFFLINE_MATERIALS_ROOT}}"

# Conservative RTX performance defaults. These affect only rendering; the
# 100 Hz physics callback and ArduPilot JSON bridge timing remain unchanged.
export ISAAC_ENABLE_RTX_OPTIMIZATION="${ISAAC_ENABLE_RTX_OPTIMIZATION:-1}"
# DLSS Performance (0) keeps the RTX 5080 viewport responsive while the
# 100 Hz physics loop remains independent from rendering.
export ISAAC_DLSS_MODE="${ISAAC_DLSS_MODE:-0}"
export ISAAC_VIEWPORT_WIDTH="${ISAAC_VIEWPORT_WIDTH:-960}"
export ISAAC_VIEWPORT_HEIGHT="${ISAAC_VIEWPORT_HEIGHT:-540}"
# Permit PhysX to catch up at 100 Hz when one RTX frame takes longer than
# 10 ms. Lower values prioritize simulation time over visible frame rate.
export ISAAC_PHYSX_MIN_FRAME_RATE="${ISAAC_PHYSX_MIN_FRAME_RATE:-15}"
export ISAAC_REAL_TIME_PACING="${ISAAC_REAL_TIME_PACING:-1}"
export ISAAC_RTX_REFLECTIONS="${ISAAC_RTX_REFLECTIONS:-0}"
export ISAAC_RTX_AMBIENT_OCCLUSION="${ISAAC_RTX_AMBIENT_OCCLUSION:-0}"
export ISAAC_RTX_TRANSLUCENCY="${ISAAC_RTX_TRANSLUCENCY:-0}"
export ISAAC_RTX_INDIRECT_DIFFUSE="${ISAAC_RTX_INDIRECT_DIFFUSE:-0}"
export ISAAC_CAMERA_WIDTH="${ISAAC_CAMERA_WIDTH:-320}"
export ISAAC_CAMERA_HEIGHT="${ISAAC_CAMERA_HEIGHT:-240}"
export ISAAC_CAMERA_FPS="${ISAAC_CAMERA_FPS:-5}"
export FLIGHT_CONTROL_RGB_CAPTURE="${FLIGHT_CONTROL_RGB_CAPTURE:-1}"
export ISAAC_AUTO_RVIZ="${ISAAC_AUTO_RVIZ:-1}"
export RVIZ_CONFIG

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "Project Isaac Sim launcher not found: ${ISAAC_PYTHON}" >&2
  exit 1
fi

if [[ ! -f "${ENVIRONMENT_USD_PATH}" ]]; then
  echo "Local environment USD not found: ${ENVIRONMENT_USD_PATH}" >&2
  exit 1
fi

if [[ ! -f "${LIDAR_USD_PATH}" ]]; then
  echo "Local OS1 LiDAR USD not found: ${LIDAR_USD_PATH}" >&2
  echo "Run simulation/room_quadcopter/scripts/vendor_hospital.py once while online." >&2
  exit 1
fi

if [[ ! -f "${DRONE_USD_PATH}" ]]; then
  echo "Local Iris drone USD not found: ${DRONE_USD_PATH}" >&2
  echo "Run simulation/room_quadcopter/scripts/vendor_hospital.py once while online." >&2
  exit 1
fi

rviz_pid=""
cleanup() {
  if [[ -n "${rviz_pid}" ]] && kill -0 "${rviz_pid}" 2>/dev/null; then
    kill "${rviz_pid}" 2>/dev/null || true
    wait "${rviz_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${ISAAC_AUTO_RVIZ}" == "1" && " $* " != *" --headless "* ]]; then
  "${RVIZ_LAUNCHER}" "${RVIZ_CONFIG}" &
  rviz_pid=$!
fi

"${ISAAC_PYTHON}" "${SCRIPT}" "$@"
